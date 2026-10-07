#!/usr/bin/env python
"""Turn a Digimap OS MasterMap order into ``<City>_officialBuildings.gpkg``.

The order is OS MasterMap Topography Layer (GeoPackage, Buildings) plus OS MasterMap Building
Height Attribute (CSV tiles). Footprints are the ``Topographicarea`` polygons whose
``descriptivegroup`` names a building; heights join on the OS TOID (``fid`` in the topography,
``OS_TOPO_TOID`` in the heights).

Structures that span a street (``descriptiveterm`` Archway, Footbridge, Bridge) are left out: the
pipeline extrudes every footprint from the ground to its roof, and a solid block where one can see
underneath would cut every sight line along that street.

Where OS supplies no height, and ``<City>_DSM.tif`` (first-return surface) and ``<City>_DTM.tif``
(terrain) cover the footprint, the height is the 90th percentile of the surface inside it minus the
median terrain under it. On London that reproduces RelHMax, where OS has one, to a median of -1.0 m
(10th-90th percentile -3.0 to +0.8 m, 400 buildings over 200 m2).

Written columns, as ``00_city_preparation.py`` reads an official layer:

  toid       OS TOID
  height     RelHMax, roof top above ground (m); from the LiDAR where OS supplies none and the
             rasters cover the footprint; empty otherwise
  height_source  ``os_bha``, ``lidar_dsm`` or empty
  base       empty by default, so the pipeline samples it from ``<City>_DTM.tif``, the raster
             that also gives the nodes their z; ``--base zero`` writes 0 for a city with no DTM
             (flat, matching nodes at z 0); ``--base absolute`` writes AbsHMin
  rel_h2     RelH2, roof base / eaves above ground (m)
  abs_hmin   AbsHMin, ground level above Ordnance Datum (m)
  bha_conf   BHA_Conf, OS confidence code

Usage:
  python pipeline/os_mastermap_buildings.py --city London \\
      --order inputData/London/Download_London_pedsimcity_buildings_BHA_3015943
"""
from __future__ import annotations

import argparse
import json
import logging
import sys
from pathlib import Path

import geopandas as gpd
import numpy as np
import pandas as pd
import pyogrio
import rasterio
from rasterio import features

import paths
import provenance

log = logging.getLogger("os_mastermap_buildings")

BHA_COLUMNS = {"RelHMax": "height", "RelH2": "rel_h2", "AbsHMin": "abs_hmin", "BHA_Conf": "bha_conf"}

# Structures over a street: extruded from the ground they would block the view under them.
OVERHEAD_TERMS = ("Archway", "Footbridge", "Bridge")
ROOF_PERCENTILE = 90


def read_heights(order: Path) -> pd.DataFrame:
    """All Building Height Attribute tiles, one row per TOID (highest confidence, then latest)."""
    folder = next(order.glob("mastermap_building_heights_*"))
    header = (folder / "docs" / "osmm-building-heights-attribute-header.csv").read_text(
        encoding="utf-8-sig").strip().split(",")
    tiles = sorted((folder).rglob("*.csv"))
    tiles = [t for t in tiles if t.parent.name != "docs"]
    heights = pd.concat((pd.read_csv(t, header=None, names=header) for t in tiles), ignore_index=True)
    log.info("heights: %d rows from %d tiles", len(heights), len(tiles))
    # A TOID on a tile edge appears in both tiles.
    heights = heights.sort_values(["BHA_Conf", "BHA_ProcessDate"], ascending=[False, False])
    heights = heights.drop_duplicates("OS_TOPO_TOID")
    return heights.set_index("OS_TOPO_TOID")[list(BHA_COLUMNS)].rename(columns=BHA_COLUMNS)


def read_footprints(order: Path) -> gpd.GeoDataFrame:
    topo = next(order.glob("mastermap-topo_*.gpkg"))
    areas = pyogrio.read_dataframe(topo, layer="Topographicarea",
                                   columns=["fid", "descriptivegroup", "descriptiveterm"])
    buildings = areas[areas["descriptivegroup"].astype(str).str.contains("Building")]
    log.info("footprints: %d building polygons of %d areas", len(buildings), len(areas))
    overhead = buildings["descriptiveterm"].astype(str).str.strip().isin(OVERHEAD_TERMS)
    log.info("footprints: %d structures over a street left out (%s)", overhead.sum(),
             ", ".join(OVERHEAD_TERMS))
    return buildings[~overhead].rename(columns={"fid": "toid"})


def lidar_heights(buildings: gpd.GeoDataFrame, dsm_path: Path, dtm_path: Path) -> pd.Series:
    """Roof height above ground from the LiDAR, per footprint: the surface's ROOF_PERCENTILE
    inside it minus the median terrain under it. NaN where the rasters do not reach it, where no
    cell centre falls inside it, or where the result is not above ground."""
    with rasterio.open(dsm_path) as dsm, rasterio.open(dtm_path) as dtm:
        if dsm.crs != dtm.crs or dsm.transform != dtm.transform or dsm.shape != dtm.shape:
            raise ValueError(f"{dsm_path.name} and {dtm_path.name} are not on one grid")
        geoms = buildings.to_crs(dsm.crs).geometry
        labels = features.rasterize(
            ((g, i + 1) for i, g in enumerate(geoms) if g is not None and not g.is_empty),
            out_shape=dsm.shape, transform=dsm.transform, fill=0, dtype="int32")
        surface = dsm.read(1, masked=True)
        terrain = dtm.read(1, masked=True)
    valid = (labels > 0) & ~np.ma.getmaskarray(surface) & ~np.ma.getmaskarray(terrain)
    cells = pd.DataFrame({"label": labels[valid], "surface": surface.data[valid],
                          "terrain": terrain.data[valid]})
    grouped = cells.groupby("label")
    height = grouped["surface"].quantile(ROOF_PERCENTILE / 100) - grouped["terrain"].median()
    height = height[height > 0]
    result = pd.Series(np.nan, index=buildings.index)
    result.iloc[height.index.to_numpy() - 1] = height.to_numpy()
    return result


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--city", required=True)
    parser.add_argument("--order", required=True, type=Path,
                        help="the unzipped Digimap order folder")
    parser.add_argument("--dsm", type=Path, default=None,
                        help="first-return surface raster for heights OS does not supply "
                             "(default <City>_DSM.tif in inputData/<City>, if present)")
    parser.add_argument("--dtm", type=Path, default=None,
                        help="terrain raster on the same grid as --dsm "
                             "(default <City>_DTM.tif in inputData/<City>)")
    parser.add_argument("--base", choices=("dtm", "zero", "absolute"), default="dtm",
                        help="dtm: leave base empty for the pipeline to sample from the DTM "
                             "(default); zero: flat ground; absolute: AbsHMin")
    args = parser.parse_args(argv)
    logging.basicConfig(level=logging.INFO, format="%(asctime)s  %(levelname)s  %(message)s")

    footprints = read_footprints(args.order)
    heights = read_heights(args.order)
    buildings = footprints.join(heights, on="toid")
    buildings["height_source"] = np.where(buildings["height"].notna(), "os_bha", None)

    raw = paths.raw_dir(args.city)
    dsm = args.dsm or raw / f"{args.city}_DSM.tif"
    dtm = args.dtm or raw / f"{args.city}_DTM.tif"
    missing = buildings["height"].isna()
    if missing.any() and dsm.exists() and dtm.exists():
        filled = lidar_heights(buildings[missing], dsm, dtm)
        buildings.loc[filled.dropna().index, "height"] = filled.dropna()
        buildings.loc[filled.dropna().index, "height_source"] = "lidar_dsm"
        log.info("heights from the LiDAR: %d of %d buildings OS gives none (%s - %s)",
                 filled.notna().sum(), missing.sum(), dsm.name, dtm.name)
    elif missing.any():
        log.info("no %s and %s: %d buildings keep no height", dsm.name, dtm.name, missing.sum())

    buildings["base"] = {"dtm": float("nan"), "zero": 0.0}.get(args.base, buildings["abs_hmin"])
    buildings = buildings[["toid", "height", "height_source", "base", "rel_h2", "abs_hmin",
                           "bha_conf", "descriptivegroup", "descriptiveterm", "geometry"]]

    with_height = buildings["height"].notna()
    log.info("heights joined: %d of %d buildings (%.1f%%), %.1f%% of footprint area",
             with_height.sum(), len(buildings), 100 * with_height.mean(),
             100 * buildings.area[with_height].sum() / buildings.area.sum())

    out = raw / f"{args.city}_officialBuildings.gpkg"
    out.unlink(missing_ok=True)
    buildings.to_file(out, driver="GPKG", layer=f"{args.city}_officialBuildings")
    log.info("written %s (%d buildings)", out, len(buildings))

    # Carried into the city's provenance file by the pipeline, which lists it as an input.
    sources = buildings["height_source"].value_counts().to_dict()
    sidecar = out.with_name(out.name + ".provenance.json")
    sidecar.write_text(json.dumps({
        "written": provenance.now(),
        "command": sys.argv,
        "order": str(args.order),
        "excluded_terms": list(OVERHEAD_TERMS),
        "roof_percentile": ROOF_PERCENTILE,
        "buildings": len(buildings),
        "height_source": {str(k): int(v) for k, v in sources.items()},
        "without_height": int(buildings["height"].isna().sum()),
        "rasters": [provenance.file_info(p) for p in (dsm, dtm) if p.exists()],
        "environment": provenance.environment(),
    }, indent=2), encoding="utf-8")
    return 0


if __name__ == "__main__":
    sys.exit(main())
