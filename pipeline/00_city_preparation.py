#!/usr/bin/env python
"""PedSimCity city preparation: build the base simulation layers for a city from OSM.

One parameterised, resumable pipeline built on the cityImage API (>= 1.2).

Folders (see pipeline/paths.py): raw inputs are read from ``inputData/<City>/``, the
final layers the Java simulation loads are written to ``src/main/resources/<City>/``,
and intermediates (``prep_staging/`` checkpoints, ``prep_config.json``) stay in
``inputData/<City>/`` so the resources folder holds nothing but simulation inputs.

Stages (each checkpointed under ``inputData/<City>/prep_staging/``; re-runs skip
completed stages unless ``--force``):

  buildings   built first (needs no network): two layers. 'obstructions' = the full context set
              (official <City>_officialBuildings.gpkg when supplied, otherwise OSM footprints);
              'buildings' = the analysed subset (see 'studyArea' / --analysis-radius below).
              Heights/base cascade (OSM height tags are NOT used): official layer's own
              height/base -> <City>_buildingHeights.gpkg (overlap join) -> DEM/DSM + DTM rasters,
              with terrain 'base' from the DTM. Land use / DMA always derive from OSM (raw -> OSM
              groups -> DMA) on the whole obstructions set; official footprints borrow them from
              OSM by largest overlap. The obstructions extent drives the network clip below.
  network     pedestrian network from OSM, clipped to the study area (or the obstructions' convex
              hull) + a fixed margin (--network-clip-buffer, default 300 m) -> clean -> consolidate
              -> betweenness centrality (Bc_Rd) -> dual graph
  districts   drive network -> dual graph -> Louvain regions (angular) -> polygonised
              partitions -> district + gateway per pedestrian node
  elevation   node z from an optional <City>_DTM raster (skipped when absent; z stays 0)
  barriers    road/water/railway/park barriers from OSM -> integration into edges
              (a_rivers, w_parks, p_barr, n_barr + structuring barriers)
  pois        activity POIs from OSM (amenity/shop/leisure/tourism/sport/office tags,
              the vocabulary pedsim.activity.agents.ActivityPurpose classifies)
  sightlines  3D sight lines nodes -> analysed buildings, occluded by the obstructions
              (requires building heights; skipped when unavailable). Capped at
              --max-sightline-distance metres; occlusion is computed analytically.
  landmarks   structural (against the full obstructions) + visibility + cultural (historic OSM
              elements) + pragmatic components -> global & local landmarkness scores

Optional raw inputs in inputData/<City>/ (also found in the resources folder),
projected in the city CRS: <City>_officialBuildings.gpkg (official footprints, legacy
name <City>_detailedBuildings.gpkg; its own height/base are used, but when the layer
carries absolute volume elevations Z_MAX_VOL/Z_MIN_VOL the above-ground height is derived
as Z_MAX_VOL - Z_MIN_VOL and base = Z_MIN_VOL, so height means height above ground),
<City>_studyArea.gpkg (polygon delimiting the analysed buildings; without it, use
--analysis-radius or analyse the whole place), <City>_buildingHeights.gpkg (height
polygons joined to footprints by overlap; needs a 'height' field, 'base' optional),
<City>_DTM.tif (terrain) and <City>_DEM.tif / <City>_DSM.tif (surface). DTM alone
gives node z and building base; surface + terrain additionally give building heights
(top - ground). OSM height/building:levels tags are never used for heights.

Final outputs, written to ``src/main/resources/<City>/`` with the names the Java
simulation reads (see ``pedsim.core.engine.Import`` / ``Environment``):

  <City>_nodes.gpkg           nodeID, Bc_Rd, district, gateway, z
  <City>_edges.gpkg           edgeID, u, v, highway, lit, a_rivers, w_parks, p_barr, n_barr
  <City>_nodesDual.gpkg       edgeID (dual centroids)
  <City>_edgesDual.gpkg       u, v, deg (deflection angle)
  <City>_barriers.gpkg        barrierID, type (road|secondary_road|railway|water|park)
  <City>_buildings.gpkg       buildingID, height, base, land_use, DMA, gScore_sc, lScore_sc
                              (the analysed buildings; landmarks are the runtime subset whose
                              scores pass the thresholds)
  <City>_obstructions.gpkg    buildingID, height, base, land_use, DMA (the full context set
                              used as occluders / neighbours for the scores)
  <City>_POIs.gpkg            poiID + OSM use tags (amenity, shop, leisure, tourism, sport,
                              office) as point features for the activity module
  <City>_sight_lines2D.gpkg   buildingID, nodeID (2D geometry)

Example:
  python pipeline/00_city_preparation.py --city Torino --place "Torino, Italy" --epsg 3003
"""

from __future__ import annotations

import argparse
import ast
import json
import logging
import shutil
import sys
import time
from datetime import datetime
from pathlib import Path

import geopandas as gpd
import numpy as np
import pandas as pd

import cityImage as ci

import paths

logging.basicConfig(level=logging.INFO, format="%(asctime)s  %(levelname)s  %(message)s")
log = logging.getLogger("city_preparation")

# Silence the per-file "Created N records" / driver chatter from the GIS I/O stack so the
# pipeline's own progress stays readable.
for _noisy in ("fiona", "fiona.ogrext", "fiona._env", "pyogrio", "pyogrio._io", "rasterio"):
    logging.getLogger(_noisy).setLevel(logging.WARNING)

STAGES = (
    "buildings", "network", "districts", "elevation", "barriers", "pois",
    "sightlines", "landmarks",
)

RASTER_SUFFIXES = (".tif", ".tiff", ".asc", ".vrt")

# Columns that hold Python lists in memory and strings on disk.
LIST_COLUMNS = (
    "a_rivers", "w_parks", "p_barr", "n_barr",
    "land_uses", "land_uses_overlap", "intersecting", "oldNodeIDs", "old_nodeIDs",
)


# ----------------------------------------------------------------------------
# Checkpoint helpers
# ----------------------------------------------------------------------------

def _stringify_list_columns(gdf: gpd.GeoDataFrame) -> gpd.GeoDataFrame:
    out = gdf.copy()
    for column in out.columns:
        if column == out.geometry.name:
            continue
        if out[column].apply(lambda v: isinstance(v, (list, tuple, set))).any():
            out[column] = out[column].apply(
                lambda v: str(list(v)) if isinstance(v, (list, tuple, set)) else v
            )
    return out


def _parse_list_columns(gdf: gpd.GeoDataFrame) -> gpd.GeoDataFrame:
    def _maybe_parse(value):
        if isinstance(value, str) and value.startswith("[") and value.endswith("]"):
            try:
                return ast.literal_eval(value)
            except (ValueError, SyntaxError):
                return value
        return value

    out = gdf.copy()
    for column in LIST_COLUMNS:
        if column in out.columns:
            out[column] = out[column].apply(_maybe_parse)
    return out


class Stager:
    """Saves/loads per-stage GeoPackage checkpoints under prep_staging/."""

    def __init__(self, staging_dir: Path, force: bool):
        self.dir = staging_dir
        self.force = force
        self.dir.mkdir(parents=True, exist_ok=True)

    def path(self, name: str) -> Path:
        return self.dir / f"{name}.gpkg"

    def done(self, *names: str) -> bool:
        return not self.force and all(self.path(n).exists() for n in names)

    def save(self, name: str, gdf: gpd.GeoDataFrame) -> None:
        _stringify_list_columns(gdf).to_file(self.path(name), driver="GPKG")
        log.info("checkpoint saved: %s (%d features)", self.path(name).name, len(gdf))

    def load(self, name: str) -> gpd.GeoDataFrame:
        return _parse_list_columns(gpd.read_file(self.path(name)))


# ----------------------------------------------------------------------------
# Raw-input helpers
# ----------------------------------------------------------------------------

def _find_raw(args, filename_suffix: str) -> Path | None:
    """First existing ``<City>_<filename_suffix>`` (inputData/ first, then resources)."""
    return paths.find_input(args.city_name, filename_suffix)


def _find_raster(args, kind: str) -> Path | None:
    """Locate an optional <City>_<kind> raster (e.g. Torino_DTM.tif)."""
    for suffix in RASTER_SUFFIXES:
        candidate = _find_raw(args, f"{kind}{suffix}")
        if candidate is not None:
            return candidate
    return None


def _column_unset(gdf: gpd.GeoDataFrame, column: str) -> bool:
    """True when the column is absent or entirely missing/NaN."""
    return column not in gdf.columns or pd.to_numeric(gdf[column], errors="coerce").isna().all()


def _assign_by_largest_overlap(target, source, columns, crs):
    """Copy ``columns`` onto each target polygon from the source polygon it overlaps
    most (by intersection area).

    Used to carry OSM-derived land use / DMA onto official footprints. Target polygons
    with no overlapping source get NaN in ``columns``.
    """
    if (target.crs != crs) or (source.crs != crs):
        raise ValueError(
            f"CRS mismatch in overlap transfer: target {target.crs}, source {source.crs}"
        )
    tgt = target.copy()
    tgt["_tix"] = np.arange(len(tgt))
    src = source[["geometry", *columns]].copy()
    src["_geo_src"] = src.geometry

    joined = gpd.sjoin(tgt[["_tix", "geometry"]], src, predicate="intersects", how="left")
    joined = joined[joined["_geo_src"].notna()]
    if joined.empty:
        for col in columns:
            tgt[col] = np.nan
    else:
        joined["_ov"] = joined.apply(
            lambda r: r["geometry"].intersection(r["_geo_src"]).area, axis=1
        )
        # Keep, per target polygon, the source row with the largest intersection area.
        best = joined.sort_values("_ov").groupby("_tix").tail(1).set_index("_tix")
        for col in columns:
            tgt[col] = tgt["_tix"].map(best[col])
    return tgt.drop(columns="_tix")


def _with_z(nodes: gpd.GeoDataFrame, stager: Stager) -> gpd.GeoDataFrame:
    """Join the DTM-sampled node elevations (elevation stage) onto a nodes frame."""
    if not stager.path("nodes_z").exists():
        return nodes
    z_table = stager.load("nodes_z")[["nodeID", "z"]]
    nodes = nodes.drop(columns=["z"], errors="ignore").merge(z_table, on="nodeID", how="left")
    nodes["z"] = pd.to_numeric(nodes["z"], errors="coerce").fillna(0.0).clip(lower=0.0)
    return nodes


# ----------------------------------------------------------------------------
# Stages
# ----------------------------------------------------------------------------

def _clip_network_to_boundary(args, stager, nodes, edges):
    """Confine the raw network to the study area (or, absent it, the obstructions' convex hull),
    buffered by a fixed margin (``--network-clip-buffer`` metres, default 300), so the network
    never extends far beyond the analysed area. Nodes outside are dropped and only edges between
    surviving nodes are kept;
    clean_network's island removal then keeps the connected remainder. No-op when neither boundary
    is available or the expected id columns are missing.
    """
    if "nodeID" not in nodes.columns or {"u", "v"} - set(edges.columns):
        log.warning("network: cannot clip (missing nodeID/u/v columns); keeping the full place")
        return nodes, edges

    study_path = _find_raw(args, "studyArea.gpkg")
    if study_path is not None:
        poly = gpd.read_file(study_path).to_crs(args.crs).geometry.union_all()
        source = study_path.name
    elif stager.path("obstructions").exists():
        # Convex hull of the building representative points (cheap on large sets; the buffer
        # absorbs the small gap to the footprint hull).
        poly = stager.load("obstructions").geometry.representative_point().union_all().convex_hull
        source = "obstructions convex hull"
    else:
        log.info("network: no study area or obstructions to clip to; keeping the full place")
        return nodes, edges

    boundary = poly.buffer(args.network_clip_buffer)

    before = len(edges)
    nodes = nodes[nodes.geometry.within(boundary)].copy()
    kept = set(nodes["nodeID"])
    edges = edges[edges["u"].isin(kept) & edges["v"].isin(kept)].copy()
    log.info(
        "network: clipped to %s + %.0f m: %d -> %d edges",
        source, args.network_clip_buffer, before, len(edges),
    )
    return nodes, edges


def stage_network(args, stager: Stager) -> None:
    if stager.done("nodes_network", "edges_network", "nodesDual", "edgesDual"):
        log.info("network: checkpoints present, skipping")
        return

    log.info("network: downloading pedestrian network for %r", args.place)
    nodes, edges = ci.pedestrian_network_from_osm(
        args.place, crs=args.crs, download_method=args.download_method
    )

    nodes, edges = _clip_network_to_boundary(args, stager, nodes, edges)

    log.info("network: cleaning (%d nodes, %d edges)", len(nodes), len(edges))
    # fix_topology nodes an edge only where another edge meets one of its existing internal
    # vertices (a missing junction); it is vertex-restricted, so grade-separated bridge/tunnel
    # crossings — which share no coordinate with the way below — are left intact.
    nodes, edges = ci.clean_network(
        nodes, edges,
        dead_ends=True, remove_islands=True,
        same_vertexes_edges=True, self_loops=True, fix_topology=True,
    )

    if args.consolidate_network:
        log.info("network: consolidating nodes (tolerance %.1f m)", args.consolidate_tolerance)
        nodes, edges = ci.consolidate_nodes(
            nodes, edges, consolidate_edges_too=True, tolerance=args.consolidate_tolerance
        )
    else:
        log.info("network: node consolidation disabled (--consolidate-network no)")

    log.info("network: computing betweenness centrality")
    graph = ci.graph_fromGDF(nodes, edges)
    centrality = ci.calculate_centrality(graph, measure="betweenness", weight="length")
    nodes["Bc_Rd"] = nodes.nodeID.map(centrality)

    # Observer elevation for the sight lines; missing/negative -> ground level.
    if "z" not in nodes.columns:
        nodes["z"] = 0.0
    nodes["z"] = pd.to_numeric(nodes["z"], errors="coerce").fillna(0.0).clip(lower=0.0)

    # The simulation reads 'lit' from the edges (street-lighting flag from OSM).
    if "lit" not in edges.columns:
        edges["lit"] = 0
    edges["lit"] = (
        edges["lit"].astype(str).str.lower().isin(("yes", "true", "1", "24/7", "automatic"))
    ).astype(int)

    log.info("network: building the dual graph")
    nodes_dual, edges_dual = ci.dual_gdf(nodes, edges, args.crs)
    nodes_dual = nodes_dual.drop(columns=["intersecting"], errors="ignore")

    stager.save("nodes_network", nodes)
    stager.save("edges_network", edges)
    stager.save("nodesDual", nodes_dual)
    stager.save("edgesDual", edges_dual)


def stage_districts(args, stager: Stager) -> None:
    if stager.done("nodes_districts"):
        log.info("districts: checkpoint present, skipping")
        return

    nodes = stager.load("nodes_network")
    edges = stager.load("edges_network")
    # cityImage's region-membership helpers (amend_nodes_membership) look nodes up by nodeID via
    # .loc, so index by nodeID: the checkpoint reload gives a plain RangeIndex, which KeyErrors as
    # soon as a small/disconnected district needs amending.
    nodes = nodes.set_index("nodeID", drop=False)
    nodes.index.name = None

    log.info("districts: downloading drive network for %r", args.place)
    nodes_drive, edges_drive = ci.network_from_osm(
        args.place, download_method=args.download_method, network_type="drive", crs=args.crs
    )
    nodes_drive, edges_drive = ci.clean_network(
        nodes_drive, edges_drive,
        dead_ends=False, remove_islands=True,
        same_vertexes_edges=False, self_loops=True, fix_topology=False,
    )

    log.info("districts: identifying regions on the drive dual graph (angular)")
    nodes_dual_drive, edges_dual_drive = ci.dual_gdf(nodes_drive, edges_drive, args.crs)
    dual_graph_drive = ci.dual_graph_fromGDF(nodes_dual_drive, edges_dual_drive)

    districts = edges_drive.copy()
    districts = ci.identify_regions(dual_graph_drive, districts, weight="rad")

    column = "p_rad"
    counts = dict(districts[column].value_counts())
    to_ignore = {label for label, count in counts.items() if count <= args.district_min_size}
    kept = districts[~(districts[column].isin(to_ignore) | (districts[column] == 999999))].copy()

    partitions = ci.polygonise_partitions(kept, column, convex_hull=False, buffer=5)

    log.info("districts: assigning %d partitions to the pedestrian nodes", len(partitions))
    nodes = ci.district_to_nodes_from_polygons(nodes, partitions, column)
    nodes[column] = nodes[column].astype(int)
    nodes = ci.amend_nodes_membership(nodes, edges, column, args.district_min_size)
    nodes = ci.find_gateways(nodes, edges, column)

    # Java contract: integer 'district' + 0/1 'gateway' on the nodes.
    nodes["district"] = nodes[column].astype(int)
    if "gateway" in nodes.columns:
        nodes["gateway"] = nodes["gateway"].fillna(0).astype(int)

    stager.save("nodes_districts", nodes)


def stage_elevation(args, stager: Stager) -> None:
    if stager.done("nodes_z"):
        log.info("elevation: checkpoint present, skipping")
        return

    dtm_path = _find_raster(args, "DTM")
    if dtm_path is None:
        log.info("elevation: no %s_DTM raster in inputData/%s or the resources folder; "
                 "node z stays 0", args.city_name, args.city_name)
        return

    nodes = stager.load("nodes_districts") if stager.path("nodes_districts").exists() \
        else stager.load("nodes_network")

    log.info("elevation: sampling node z from %s", dtm_path.name)
    try:
        nodes = ci.assign_height_from_dtm(nodes, str(dtm_path), z_col="z")
    except ImportError as e:
        log.warning("elevation: raster dependencies unavailable (%s); skipping stage", e)
        return

    stager.save("nodes_z", nodes[["nodeID", "z", "geometry"]])


def stage_barriers(args, stager: Stager) -> None:
    if stager.done("barriers", "edges_barriers"):
        log.info("barriers: checkpoints present, skipping")
        return

    edges = stager.load("edges_network")
    # cityImage's barrier-integration helpers (barriers_along) look edges up by edgeID via .loc, so
    # index by edgeID: the checkpoint reload gives a plain RangeIndex, which KeyErrors as soon as an
    # edgeID is not a valid row position.
    edges = edges.set_index("edgeID", drop=False)
    edges.index.name = None

    log.info("barriers: downloading barrier features for %r", args.place)
    barriers = ci.barriers_from_osm(
        args.place, download_method=args.download_method, crs=args.crs,
        include_primary=True, include_secondary=False, parks_min_area=100000,
    )
    barriers = barriers.reset_index(drop=True)
    barriers["barrierID"] = barriers.index.astype(int)
    # Java (Environment.generateBarriersMap) reads the barrier kind from a 'type' column.
    if "type" not in barriers.columns and "barrier_type" in barriers.columns:
        barriers["type"] = barriers["barrier_type"]

    # Clip to the study area (+ margin) so barrier-edge integration stays local.
    envelope = edges.union_all().envelope.buffer(50)
    barriers = gpd.clip(barriers, envelope).reset_index(drop=True)
    barriers["barrierID"] = barriers.index.astype(int)

    log.info("barriers: integrating %d barriers into the edges", len(barriers))
    barriers_within = barriers[barriers.intersects(envelope)]
    sindex = edges.sindex

    edges = ci.along_water(edges, barriers_within)
    edges = ci.along_within_parks(edges, barriers_within)
    edges["p_barr"] = edges["a_rivers"] + edges["w_parks"]
    edges["p_barr"] = edges["p_barr"].apply(lambda barrier_ids: list(set(barrier_ids)))

    negative = barriers_within[barriers_within["barrier_type"].isin(["railway", "road"])]
    edges["n_barr"] = edges.apply(
        lambda row: ci.barriers_along(row["edgeID"], edges, negative, sindex, offset=25),
        axis=1,
    )
    edges = ci.assign_structuring_barriers(edges, barriers_within)

    stager.save("barriers", barriers)
    stager.save("edges_barriers", edges)


# OSM tags matching the vocabulary the Java activity module classifies into purposes
# (pedsim.activity.agents.ActivityPurpose): presence-only keys shop/leisure/tourism/
# sport/office plus value-bearing amenity. Keep the tag *columns* under these exact
# names — PoiClassifier reads them as-is.
ACTIVITY_POI_TAGS = {
    "amenity": True, "shop": True, "leisure": True,
    "tourism": True, "sport": True, "office": True,
}
POI_TAG_COLUMNS = ("amenity", "shop", "leisure", "tourism", "sport", "office")


def stage_pois(args, stager: Stager) -> None:
    if stager.done("pois"):
        log.info("pois: checkpoint present, skipping")
        return

    log.info("pois: downloading activity POIs for %r", args.place)
    pois = ci.features_from_osm(
        args.place, ACTIVITY_POI_TAGS,
        download_method=args.download_method, crs=args.crs,
    )
    if pois is None or pois.empty:
        log.warning("pois: no tagged features found; the activity module will fall back "
                    "to census workplace/night weights")
        return

    pois = pois[pois.geometry.notna() & ~pois.geometry.is_empty].copy()
    # One point per POI: the Java side snaps each feature to its nearest network node,
    # so a representative point is enough and keeps the layer small.
    pois["geometry"] = pois.geometry.representative_point()

    keep = [c for c in POI_TAG_COLUMNS if c in pois.columns]
    extra = [c for c in ("name",) if c in pois.columns]
    pois = pois[keep + extra + ["geometry"]].reset_index(drop=True)
    # Drop features carrying none of the recognised tags (they cannot classify).
    pois = pois[pois[keep].notna().any(axis=1)].reset_index(drop=True)
    pois["poiID"] = pois.index.astype(int)

    log.info("pois: %d tagged features kept", len(pois))
    stager.save("pois", pois)


def _find_official(args) -> Path | None:
    """Official footprints layer: <City>_officialBuildings.gpkg (legacy: detailedBuildings)."""
    return _find_raw(args, "officialBuildings.gpkg") or _find_raw(args, "detailedBuildings.gpkg")


def _normalise_official_heights(buildings):
    """Convert an official layer's absolute volume elevations to cityImage's convention.

    Some official 3D building layers store *absolute* elevations: ``Z_MAX_VOL`` = roof
    elevation above sea level and ``Z_MIN_VOL`` = ground elevation above sea level (often
    also copied into 'height'/'base'). cityImage instead expects 'height' = the above-ground
    height and 'base' = the ground elevation, and places the roof at ``height + base``.
    Passing the absolute roof elevation as 'height' models a building at roughly its
    roof-elevation *height* (e.g. a 22 m building on 25 m ground becomes a ~47 m tower on the
    ground), which massively over-occludes the 3D sight lines. When the absolute extents are
    present, derive the above-ground height from them. Idempotent: recomputing from the
    (retained) Z_MAX_VOL/Z_MIN_VOL columns always yields the same result, so it also repairs
    caches written before this fix.
    """
    cols = {c.lower(): c for c in buildings.columns}
    zmax, zmin = cols.get("z_max_vol"), cols.get("z_min_vol")
    if zmax is None or zmin is None:
        return buildings
    ground = pd.to_numeric(buildings[zmin], errors="coerce")
    height = (pd.to_numeric(buildings[zmax], errors="coerce") - ground).clip(lower=0)
    buildings = buildings.copy()
    buildings["base"] = ground
    buildings["height"] = height
    log.info("buildings: official layer stores absolute elevations; using above-ground "
             "height = Z_MAX_VOL - Z_MIN_VOL (median %.1f m, ground base median %.1f m)",
             height.median(), ground.median())
    return buildings


def _assign_building_heights(buildings, args, heights_path, dtm_path, surface_path):
    """Height / base cascade (OSM height tags are NOT used): existing height/base ->
    <City>_buildingHeights.gpkg (overlap) -> DEM/DSM + DTM rasters. The DTM (terrain)
    supplies the ground 'base'; the surface model (DSM, often shipped as "DEM") supplies
    above-ground heights."""
    if heights_path is not None and _column_unset(buildings, "height"):
        log.info("buildings: assigning heights from %s (overlap join)", heights_path.name)
        heights = gpd.read_file(heights_path).to_crs(args.crs)
        buildings = ci.assign_building_heights_from_other_gdf(
            buildings, heights, args.crs, min_overlap=0.30
        )

    # Non-destructive: footprints outside the raster extent keep their rows with NaN.
    want_height = _column_unset(buildings, "height") and surface_path is not None
    want_base = _column_unset(buildings, "base")
    if dtm_path and (want_height or want_base):
        surface = str(surface_path) if want_height else None
        log.info("buildings: assigning %s from %s%s",
                 "base + height" if want_height else "terrain base",
                 dtm_path.name,
                 f" + {surface_path.name}" if want_height else "")
        try:
            _, buildings = ci.assign_elevations_from_rasters(
                None, buildings, str(dtm_path), surface_path=surface
            )
        except ImportError as e:
            log.warning("buildings: raster dependencies unavailable (%s)", e)

    if _column_unset(buildings, "height"):
        log.warning(
            "buildings: no official heights, no %s_buildingHeights.gpkg, and no DEM/DTM "
            "pair; sight lines will be skipped",
            args.city_name,
        )
    return buildings


def _select_analysis_buildings(obstructions, args):
    """The subset of obstructions actually analysed and shipped as <City>_buildings.gpkg.

    A building belongs to the analysis when its representative point falls inside the
    user-supplied <City>_studyArea.gpkg, or, absent that, within --analysis-radius of the
    obstructions' union centroid. With neither, every obstruction is analysed (the scores
    then degenerate to the whole-place behaviour). buildingID is inherited from the
    obstructions, so a building keeps the same ID in both layers.
    """
    points = obstructions.geometry.representative_point()

    study_path = _find_raw(args, "studyArea.gpkg")
    if study_path is not None:
        area = gpd.read_file(study_path).to_crs(args.crs)
        subset = obstructions[points.within(area.geometry.union_all())].copy()
        log.info("buildings: %d/%d obstructions inside study area %s",
                 len(subset), len(obstructions), study_path.name)
    elif args.analysis_radius:
        centre = obstructions.geometry.union_all().centroid
        core = centre.buffer(args.analysis_radius)
        subset = obstructions[points.within(core)].copy()
        log.info("buildings: %d/%d obstructions within %.0f m of the union centroid",
                 len(subset), len(obstructions), args.analysis_radius)
    else:
        log.info("buildings: no study area or --analysis-radius; analysing all obstructions")
        return obstructions.copy()

    if subset.empty:
        log.warning("buildings: analysis area selected 0 buildings; "
                    "falling back to all obstructions")
        return obstructions.copy()
    return subset


def _load_or_build_obstructions(args):
    """Obstructions = the full context set (occluders + neighbourhood the scores sample).

    Reused from ``inputData/<City>/<City>_obstructions.gpkg`` when already present, so later
    runs skip the OSM/official regeneration; otherwise built from the official footprints
    (<City>_officialBuildings.gpkg) or OSM, given land use / DMA and the height / base
    cascade, then written there as a durable, reusable input.
    """
    obs_path = args.raw_dir / f"{args.city_name}_obstructions.gpkg"
    if obs_path.exists():
        log.info("buildings: reusing obstructions from %s (skipping regeneration)", obs_path.name)
        obstructions = _parse_list_columns(gpd.read_file(obs_path).to_crs(args.crs))
        # Repair caches whose 'height' is an absolute roof elevation (see the helper).
        obstructions = _normalise_official_heights(obstructions)
        if "buildingID" not in obstructions.columns:
            obstructions = obstructions.reset_index(drop=True)
            obstructions["buildingID"] = obstructions.index.astype(int)
        if "area" not in obstructions.columns:
            obstructions["area"] = obstructions.geometry.area
        return obstructions

    dtm_path = _find_raster(args, "DTM")
    surface_path = _find_raster(args, "DEM") or _find_raster(args, "DSM")
    official_path = _find_official(args)
    heights_path = _find_raw(args, "buildingHeights.gpkg")

    # OSM buildings are always the land-use / DMA donor over the whole obstruction extent:
    # cityImage's classifier is OSM-vocabulary-specific, so DMA is derived here whether
    # or not official footprints are supplied.
    log.info("buildings: downloading OSM buildings for %r", args.place)
    osm = ci.buildings_from_osm(
        args.place, download_method=args.download_method, crs=args.crs, min_area=200
    )
    osm = ci.gdf_multipolygon_to_polygon(osm)
    log.info("buildings: deriving land uses (raw -> OSM groups -> DMA)")
    osm = ci.derive_land_uses_raw_fromOSM(osm)
    osm = ci.classify_land_uses_raws_into_OSMgroups(osm)
    osm = ci.classify_land_uses_intoDMAs(osm)

    if official_path is not None:
        # Official dataset: its geometries are the authoritative footprints and its own
        # 'height'/'base' columns (when present) are kept. Land use / DMA are borrowed
        # from the OSM buildings by largest overlap.
        log.info("buildings: using official footprints from %s as obstructions",
                 official_path.name)
        obstructions = gpd.read_file(official_path).to_crs(args.crs)
        obstructions = ci.gdf_multipolygon_to_polygon(obstructions)
        obstructions = obstructions[
            obstructions.geometry.notna() & ~obstructions.geometry.is_empty
        ].copy()
        obstructions = _assign_by_largest_overlap(obstructions, osm, ["land_uses", "DMA"], args.crs)
        obstructions = _normalise_official_heights(obstructions)
    else:
        # OSM footprints carry their land use / DMA directly. OSM height tags are
        # intentionally discarded: heights come only from a heights layer or rasters.
        obstructions = osm.drop(columns=[c for c in ("height", "base") if c in osm.columns])

    obstructions = obstructions.reset_index(drop=True)
    obstructions["buildingID"] = obstructions.index.astype(int)
    obstructions["area"] = obstructions.geometry.area

    # Heights on the whole obstructions set: the analysed subset inherits them, and the
    # occluders need heights for the 3D sight lines.
    obstructions = _assign_building_heights(obstructions, args, heights_path, dtm_path, surface_path)

    # Durable, reusable copy in inputData so later runs skip regeneration.
    _stringify_list_columns(obstructions).to_file(obs_path, driver="GPKG")
    log.info("buildings: obstructions written to %s (%d features)", obs_path.name, len(obstructions))
    return obstructions


def stage_buildings(args, stager: Stager) -> None:
    if stager.done("buildings_analysed", "obstructions"):
        log.info("buildings: checkpoints present, skipping")
        return

    # This stage needs no network, so it runs first: its obstructions define the extent the
    # network stage clips to. The structural score (which needs the edges) is deferred to the
    # landmarks stage.
    obstructions = _load_or_build_obstructions(args)

    # Analysed buildings: the subset (study area / radius) whose landmark scores are later
    # computed against the full obstructions, so boundary buildings keep a complete neighbourhood.
    buildings = _select_analysis_buildings(obstructions, args)

    stager.save("obstructions", obstructions)
    stager.save("buildings_analysed", buildings)


def stage_sightlines(args, stager: Stager) -> None:
    if stager.done("sight_lines"):
        log.info("sightlines: checkpoint present, skipping")
        return

    buildings = stager.load("buildings_analysed")
    if "height" not in buildings.columns or buildings["height"].isna().all():
        log.warning("sightlines: no building heights available; skipping stage")
        return

    nodes = stager.load("nodes_districts") if stager.path("nodes_districts").exists() \
        else stager.load("nodes_network")
    nodes = _with_z(nodes, stager)
    edges = stager.load("edges_network")

    buildings = buildings[pd.to_numeric(buildings["height"], errors="coerce").notna()].copy()
    buildings["height"] = buildings["height"].astype(float)
    targets = buildings[buildings["height"] >= 3.0]

    # Occluders: the full obstructions set (with heights) so sight lines to boundary
    # targets are not spuriously unobstructed. Rows without a height cannot occlude in 3D.
    obstructions = stager.load("obstructions")
    obstructions = obstructions[
        pd.to_numeric(obstructions["height"], errors="coerce").notna()
    ].copy()
    obstructions["height"] = obstructions["height"].astype(float)

    # A distance cap (0 disables it) bounds candidate pairs: sight lines longer than this
    # are dominated by rare, mostly-obstructed lines that account for most of the runtime.
    max_distance = args.max_sightline_distance if args.max_sightline_distance > 0 else None
    log.info("sightlines: computing 3D sight lines (%d nodes x %d targets, %d occluders, "
             "max distance %s) — this can take a while",
             len(nodes), len(targets), len(obstructions),
             f"{max_distance:.0f} m" if max_distance else "uncapped")
    # All our height sources (OSM tags, official layer, DEM-DTM) are above-ground heights;
    # 'base' (when the DTM provided it) carries the terrain elevation under the footprint.
    # compute_3d_sight_lines treats height as above-ground and adds it to base internally.
    start = time.perf_counter()
    sight_lines = ci.compute_3d_sight_lines(
        nodes, targets, obstructions, edges, args.city_name,
        distance_along=200, min_observer_target_distance=300,
        max_observer_target_distance=max_distance,
        # Sight lines are computed per graph node (consolidate=False). When the network
        # stage consolidated nodes (--consolidate-network yes) that already is the graph
        # resolution; consolidating again here would test visibility at a coarser position
        # than the simulation graph and then explode back, biasing the count downward.
        consolidate=False,
        num_workers=args.workers,
        verbose=True,
    )
    elapsed = time.perf_counter() - start
    log.info("sightlines: compute_3d_sight_lines wall time %.1f s (%.2f h), "
             "%d sight lines", elapsed, elapsed / 3600, len(sight_lines))
    if sight_lines.empty:
        log.warning("sightlines: no visible sight lines found")
    stager.save("sight_lines", sight_lines)

    # Persist a small completion record next to the checkpoint. The per-run console log is
    # truncated each launch, so this is what proves - after the fact - that compute_3d_sight_lines
    # returned and the merge/save completed (its presence + counts, not just the .gpkg existing).
    meta = {
        "created": datetime.now().astimezone().isoformat(timespec="seconds"),
        "wall_time_s": round(elapsed, 1),
        "wall_time_h": round(elapsed / 3600, 3),
        "n_sight_lines": int(len(sight_lines)),
        "n_observers_in": int(len(nodes)),
        "distinct_observers": (
            int(sight_lines["nodeID"].nunique()) if "nodeID" in sight_lines.columns else None
        ),
        "n_targets": int(len(targets)),
        "n_obstructions": int(len(obstructions)),
        "max_sightline_distance_m": (
            args.max_sightline_distance if args.max_sightline_distance > 0 else None
        ),
    }
    meta_path = stager.path("sight_lines").with_suffix(".meta.json")
    meta_path.write_text(json.dumps(meta, indent=2), encoding="utf-8")
    log.info("sightlines: wrote completion record %s (%d sight lines)",
             meta_path.name, len(sight_lines))

    # compute_3d_sight_lines writes per-chunk GeoPackages into ./sight_lines_tmp and merges
    # them into the returned frame, but never cleans up. The result is now finalised in the
    # stager, so the temporary chunks are dead weight — remove the folder.
    tmp_chunks = Path("sight_lines_tmp")
    if tmp_chunks.is_dir():
        try:
            shutil.rmtree(tmp_chunks)
            log.info("sightlines: removed temporary chunk folder %s", tmp_chunks)
        except OSError as e:
            log.warning("sightlines: could not remove %s (%s)", tmp_chunks, e)


def stage_landmarks(args, stager: Stager) -> None:
    if stager.done("landmarks"):
        log.info("landmarks: checkpoint present, skipping")
        return

    buildings = stager.load("buildings_analysed")
    obstructions = stager.load("obstructions")
    edges = stager.load("edges_network")

    # Structural component: computed here (not in the buildings stage) because it needs the edges.
    # The analysed buildings are scored against the full obstructions, so the 2d visibility /
    # neighbour terms are not truncated at the study-area boundary.
    # The 2D advance-visibility isovist is CPU-bound and per-building; run it across most of
    # the cores (it is the slow part of this stage) — the result is worker-count independent.
    import multiprocessing
    struct_workers = max(1, multiprocessing.cpu_count() - 1)
    log.info("landmarks: structural component (%d analysed vs %d obstructions, %d workers)",
             len(buildings), len(obstructions), struct_workers)
    start = time.perf_counter()
    buildings = ci.structural_score(
        buildings, obstructions, edges,
        advance_vis_expansion_distance=300, neighbours_radius=150,
        workers=struct_workers,
    )
    log.info("landmarks: structural component done in %.1f s", time.perf_counter() - start)

    sight_lines = (
        stager.load("sight_lines") if stager.path("sight_lines").exists() else None
    )
    log.info("landmarks: visibility component (%s sight lines)",
             "with" if sight_lines is not None else "no")
    buildings = ci.visibility_score(buildings, sight_lines=sight_lines, method="combined")

    log.info("landmarks: cultural component (historic elements from OSM)")
    try:
        historic = ci.features_from_osm(
            args.place, {"historic": True},
            download_method=args.download_method, crs=args.crs,
        )
    except Exception as e:  # no historic features is not fatal
        log.warning("landmarks: historic download failed (%s); cultural score = 0", e)
        historic = None
    buildings = ci.cultural_score(buildings, historic_elements_gdf=historic)

    # Pragmatic = land-use entropy in a search radius. Computed over the full obstructions
    # (which carry land_uses) so a boundary building still sees all its neighbours, then the
    # 'prag' value is mapped back onto the analysed buildings by buildingID.
    log.info("landmarks: pragmatic component (entropy over the obstructions)")
    obstructions = ci.pragmatic_score(obstructions, search_radius=200)
    buildings["prag"] = buildings["buildingID"].map(
        obstructions.set_index("buildingID")["prag"]
    )

    log.info("landmarks: global and local scores")
    # PedSimCity-specific weights; they deliberately differ from the cityImage defaults.
    global_index_weights = {
        "3dvis": 0.50, "fac": 0.30, "height": 0.20,
        "area": 0.30, "2dvis": 0.30, "neigh": 0.20, "road": 0.20,
    }
    global_component_weights = {"vScore": 0.45, "sScore": 0.25, "cScore": 0.10, "pScore": 0.20}
    local_index_weights = {
        "3dvis": 0.50, "fac": 0.30, "height": 0.20,
        "area": 0.40, "2dvis": 0.00, "neigh": 0.30, "road": 0.30,
    }
    # No visual component at the local level (as in the notebooks' local weighting).
    local_component_weights = {"vScore": 0.00, "sScore": 0.45, "cScore": 0.10, "pScore": 0.45}

    buildings = ci.score_buildings_global(
        buildings,
        index_weights=global_index_weights,
        component_weights=global_component_weights,
    )
    buildings = ci.score_buildings_local(
        buildings,
        index_weights=local_index_weights,
        component_weights=local_component_weights,
        rescaling_radius=args.local_radius,
    )

    # Java contract: a scalar 'land_use' string per building.
    if "land_use" not in buildings.columns and "land_uses" in buildings.columns:
        buildings["land_use"] = buildings["land_uses"].apply(
            lambda uses: uses[0] if isinstance(uses, (list, tuple)) and uses else "unclassified"
        )

    stager.save("landmarks", buildings)


# ----------------------------------------------------------------------------
# Final assembly: write the files the Java simulation reads
# ----------------------------------------------------------------------------

def finalize(args, stager: Stager) -> None:
    out = args.resources_dir
    prefix = out / args.city_name

    if not stager.path("nodes_network").exists():
        log.warning("finalize: no network checkpoints yet; nothing to write "
                    "(run the network stage first)")
        return

    nodes = stager.load("nodes_districts") if stager.path("nodes_districts").exists() \
        else stager.load("nodes_network")
    nodes = _with_z(nodes, stager)
    edges = stager.load("edges_barriers") if stager.path("edges_barriers").exists() \
        else stager.load("edges_network")

    nodes = nodes.drop(columns=["oldNodeIDs", "old_nodeIDs"], errors="ignore")
    _stringify_list_columns(nodes).to_file(f"{prefix}_nodes.gpkg", driver="GPKG")
    _stringify_list_columns(edges).to_file(f"{prefix}_edges.gpkg", driver="GPKG")

    if stager.path("nodesDual").exists() and stager.path("edgesDual").exists():
        stager.load("nodesDual").pipe(_stringify_list_columns) \
            .to_file(f"{prefix}_nodesDual.gpkg", driver="GPKG")
        stager.load("edgesDual").pipe(_stringify_list_columns) \
            .to_file(f"{prefix}_edgesDual.gpkg", driver="GPKG")

    if stager.path("barriers").exists():
        stager.load("barriers").pipe(_stringify_list_columns) \
            .to_file(f"{prefix}_barriers.gpkg", driver="GPKG")

    # The landmarks checkpoint is the full buildings layer with landmark scores; the
    # Java reader expects it as <City>_buildings.gpkg.
    if stager.path("landmarks").exists():
        stager.load("landmarks").pipe(_stringify_list_columns) \
            .to_file(f"{prefix}_buildings.gpkg", driver="GPKG")
    elif stager.path("buildings_analysed").exists():
        # No landmark stage yet: still ship the analysed buildings (DMA, use tags);
        # the sim runs with landmark navigation disabled.
        stager.load("buildings_analysed").pipe(_stringify_list_columns) \
            .to_file(f"{prefix}_buildings.gpkg", driver="GPKG")

    if stager.path("obstructions").exists():
        stager.load("obstructions").pipe(_stringify_list_columns) \
            .to_file(f"{prefix}_obstructions.gpkg", driver="GPKG")

    if stager.path("pois").exists():
        stager.load("pois").pipe(_stringify_list_columns) \
            .to_file(f"{prefix}_POIs.gpkg", driver="GPKG")

    if stager.path("sight_lines").exists():
        sight_lines = stager.load("sight_lines")
        if not sight_lines.empty:
            from shapely import force_2d
            sight_lines = sight_lines.copy()
            sight_lines["geometry"] = force_2d(sight_lines.geometry.values)
            keep = [c for c in ("buildingID", "nodeID", "length", "geometry")
                    if c in sight_lines.columns]
            sight_lines[keep].to_file(f"{prefix}_sight_lines2D.gpkg", driver="GPKG")

    log.info("final outputs written to %s with prefix %s_*", out, args.city_name)


# ----------------------------------------------------------------------------
# Entry point
# ----------------------------------------------------------------------------

CONFIG_NAME = "prep_config.json"


def _load_config(raw_dir: Path) -> dict:
    config_path = raw_dir / CONFIG_NAME
    if not config_path.exists():
        return {}
    try:
        return json.loads(config_path.read_text(encoding="utf-8"))
    except (OSError, ValueError) as e:
        log.warning("could not read %s (%s); ignoring it", config_path, e)
        return {}


def _save_config(args) -> None:
    """Persist the city's identity so stage re-runs don't re-ask (and can't mismatch)."""
    config_path = args.raw_dir / CONFIG_NAME
    config = {
        "place": args.place,
        "epsg": args.epsg,
        "download_method": args.download_method,
    }
    config_path.write_text(json.dumps(config, indent=2), encoding="utf-8")


def _yes_no(value):
    """argparse type: parse a yes/no (y/n, true/false, 1/0) string into a bool."""
    v = str(value).strip().lower()
    if v in ("yes", "y", "true", "t", "1"):
        return True
    if v in ("no", "n", "false", "f", "0"):
        return False
    raise argparse.ArgumentTypeError(f"expected yes/no, got {value!r}")


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--city", required=True,
                        help="city name: the folder under inputData/ (raw material) and "
                             "src/main/resources/ (outputs), and the <City>_* file prefix")
    parser.add_argument("--place", default=None,
                        help="OSM query for the study area, e.g. 'Torino, Italy' "
                             f"(optional once {CONFIG_NAME} exists in inputData/<City>)")
    parser.add_argument("--epsg", default=None, type=int,
                        help="projected CRS for the city, e.g. 3003 "
                             f"(optional once {CONFIG_NAME} exists in inputData/<City>)")
    parser.add_argument("--download_method", default=None,
                        help="cityImage download method (default OSMplace)")
    parser.add_argument("--consolidate-network", type=_yes_no, default=True,
                        metavar="yes/no",
                        help="consolidate nearby network nodes in the network stage "
                             "(yes/no, default yes); uses --consolidate-tolerance")
    parser.add_argument("--consolidate-tolerance", type=float, default=15.0,
                        help="node consolidation tolerance in metres, used when "
                             "--consolidate-network yes (default 15)")
    parser.add_argument("--district-min-size", type=int, default=20,
                        help="minimum edges per district partition (default 20)")
    parser.add_argument("--local-radius", type=float, default=800.0,
                        help="rescaling radius for local landmark scores (default 800)")
    parser.add_argument("--analysis-radius", type=float, default=None,
                        help="radius in metres from the obstructions' union centroid that "
                             "delimits the analysed buildings when no <City>_studyArea.gpkg "
                             "is supplied (default: analyse the whole place)")
    parser.add_argument("--network-clip-buffer", type=float, default=300.0,
                        help="margin in metres kept around the study area / obstructions hull when "
                             "clipping the network (default 300)")
    parser.add_argument("--workers", type=int, default=None,
                        help="parallel workers for the sight lines (default cpu/2)")
    parser.add_argument("--max-sightline-distance", type=float, default=2000.0,
                        help="maximum observer-target distance in metres for the 3D sight "
                             "lines (default 2000). Pairs farther apart are not considered: "
                             "in a dense city they are dominated by long, mostly-obstructed "
                             "lines that account for the bulk of the runtime. Pass 0 to "
                             "disable the cap and consider all pairs (much slower).")
    parser.add_argument("--stages", default="all",
                        help=f"comma-separated subset of {','.join(STAGES)} (default all)")
    parser.add_argument("--force", action="store_true",
                        help="recompute stages even when checkpoints exist")
    args = parser.parse_args(argv)

    args.city_name = args.city
    args.raw_dir = paths.raw_dir(args.city_name)
    args.resources_dir = paths.resources_dir(args.city_name)

    # City identity: CLI wins, then the saved per-city config; saved back for stage re-runs.
    config = _load_config(args.raw_dir)
    args.place = args.place or config.get("place")
    args.epsg = args.epsg or config.get("epsg")
    args.download_method = args.download_method or config.get("download_method") or "OSMplace"
    if not args.place or not args.epsg:
        parser.error(
            f"--place and --epsg are required on the first run (no {CONFIG_NAME} "
            f"in {args.raw_dir})"
        )
    _save_config(args)

    args.crs = f"EPSG:{args.epsg}"
    if args.workers is None:
        import multiprocessing
        args.workers = max(1, multiprocessing.cpu_count() // 2)

    requested = STAGES if args.stages == "all" else tuple(
        s.strip() for s in args.stages.split(",") if s.strip()
    )
    unknown = set(requested) - set(STAGES)
    if unknown:
        parser.error(f"unknown stages: {sorted(unknown)}; valid: {STAGES}")
    args.run_stages = requested
    return args


def main(argv=None) -> int:
    args = parse_args(argv)
    stager = Stager(args.raw_dir / "prep_staging", args.force)

    stage_functions = {
        "network": stage_network,
        "districts": stage_districts,
        "elevation": stage_elevation,
        "barriers": stage_barriers,
        "pois": stage_pois,
        "buildings": stage_buildings,
        "sightlines": stage_sightlines,
        "landmarks": stage_landmarks,
    }
    for stage in STAGES:
        if stage not in args.run_stages:
            continue
        log.info("========== stage: %s ==========", stage)
        stage_functions[stage](args, stager)

    finalize(args, stager)
    return 0


if __name__ == "__main__":
    sys.exit(main())
