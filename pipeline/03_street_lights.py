"""Step 3: compute continuous street illumination.

Inputs (searched in inputData/<City>/ then the resources folder):
- <City>_streetlights_with_radius.gpkg (step-2 output)
- <City>_edges.gpkg
- <City>_buildings.gpkg

Outputs:
- <City>_edges_illuminated_continuous.gpkg  -> src/main/resources/<City>/ (read by the sim)
- <City>_nodes_2m_densified_illuminated.gpkg -> inputData/<City>/ (intermediate for step 4)

`--city` is the city name (folder under inputData/ and resources/, and file prefix).

`--falloff-law` and `--variant` exist so the choice of physics can be *measured* rather than
argued: run the alternatives over the same edges and compare the `pct_unlit` distributions this
step prints. A `--variant` suffixes every output and diverts the edges layer to inputData/, so a
comparison run can never overwrite the layer the simulation reads. The law also sets the flux
normalisation, which step 2 applies - so a comparison re-runs step 2 with the same
`--falloff-law` and `--variant` first.

## Why this step is bulk-array work and not a loop

It is the only expensive thing in the pipeline: ~1.08M sample points on Torino, each summing
every lamp within the negligible-contribution radius, each of those pairs needing a line-of-sight
test against the building footprints. Written per pair in Python - one `LineString` constructed
and one index lookup per point-lamp pair - that is tens of millions of interpreter round trips,
and it is why nobody re-ran this step. The work is identical here; it is just handed to shapely
and scipy in blocks: one vectorised interpolation for the sample points, one KD-tree query per
block of points, and one bulk `STRtree` "crosses" query per block of sight lines.
"""

from __future__ import annotations

import argparse
import itertools
from pathlib import Path

import geopandas as gpd
import numpy as np
import shapely
from scipy.spatial import cKDTree

import lighting
import paths


SAMPLE_SPACING_M = 2.0

# A lamp whose point falls inside a building footprint, but no deeper than this, is mounted on or
# under that building - a wall bracket, or a fixture under one of Turin's arcades - and lights the
# street in front of it. The building it is attached to therefore does not occlude it. Deeper than
# this the lamp is inside the block (a courtyard, a gallery) and stays occluded, since its light
# does not reach the street. On Torino, 5,873 lamps sit inside a footprint: the median is 1.6 m in,
# 87% are within 5 m, and 316 are deeper than 10 m.
ARCADE_DEPTH_M = 5.0

# Block sizes. They are memory controls, not tuning knobs: the answer does not depend on them,
# only on how large the intermediate arrays get. A block of points produces one KD-tree query and,
# with a hundred-odd lamps in range of each, some millions of sight lines, which are then tested a
# block at a time so the geometry array never has to hold them all at once.
POINT_BLOCK = 10_000
SIGHT_LINE_BLOCK = 250_000


def remove_existing(path: Path) -> None:
    if path.exists():
        path.unlink()


def load_inputs(city: str, variant: str):
    # Require the step-2 output (*_with_radius.gpkg): it carries the per-lamp physics
    # (downward_intensity_cd) this step needs. The raw inventory is intentionally NOT accepted so
    # that step 3 can never silently fabricate uniform lamp intensity - run step 2 first.
    punti_path = paths.require_input(
        city,
        f"streetlights_with_radius{variant}.gpkg",
        "lamp inventory with radius (run step 2 first)",
    )
    edges_path = paths.require_input(city, "edges.gpkg", "edges layer")
    buildings_path = paths.require_input(city, "buildings.gpkg", "buildings layer")

    print("Loading datasets...")
    print(f"  city: {city}")
    print(f"  lamps    : {punti_path}")
    print(f"  edges    : {edges_path}")
    print(f"  buildings: {buildings_path}")

    punti = gpd.read_file(punti_path)
    edges = gpd.read_file(edges_path)
    buildings = gpd.read_file(buildings_path)

    if punti.empty:
        raise ValueError(f"Lamp layer is empty: {punti_path}")
    if edges.empty:
        raise ValueError(f"Edges layer is empty: {edges_path}")

    # Mounting height, like downward_intensity_cd below, is step 2's job. Filling a missing one
    # with 9.0 here was the same silent fabrication the intensity guard already refuses: height
    # enters the illuminance law twice, so a wall fixture credited with a pole's height lights a
    # street it does not light. Step 2 imputes by support type and records how in `altezza_source`.
    if "altezza_palo_m" not in punti.columns:
        raise KeyError(
            f"{punti_path} has no 'altezza_palo_m' column. Run step 2 to assign mounting heights "
            "before running step 3."
        )
    missing_height = punti["altezza_palo_m"].isna() | (punti["altezza_palo_m"] <= 0)
    if missing_height.any():
        raise ValueError(
            f"{punti_path} has {int(missing_height.sum())} lamps with no usable "
            "'altezza_palo_m'. Step 2 assigns a height to every lamp; re-run it rather than "
            "letting step 3 invent one."
        )

    if "altezza_source" in punti.columns:
        assumed = punti["altezza_source"].ne("measured").sum()
        print(
            f"  mounting heights: {len(punti) - assumed} measured, {assumed} imputed "
            f"({100.0 * assumed / len(punti):.1f}%)"
        )

    # downward_intensity_cd is produced by step 2. Its absence means step 3 was pointed at a
    # non-step-2 file: fail loudly rather than fabricating uniform lamp intensity.
    if "downward_intensity_cd" not in punti.columns:
        raise KeyError(
            f"{punti_path} has no 'downward_intensity_cd' column. Run step 2 "
            "(02_street_lights.py) to compute the per-lamp physics before running step 3."
        )

    if punti["downward_intensity_cd"].isnull().any():
        # Impute the occasional missing lamp value within an otherwise valid dataset (by technology
        # group, then global mean). If every value is missing there is nothing to impute from, so
        # fail rather than invent one.
        if "tecnologia" in punti.columns:
            punti["downward_intensity_cd"] = punti.groupby("tecnologia")["downward_intensity_cd"].transform(
                lambda x: x.fillna(x.mean() if not x.dropna().empty else np.nan)
            )
        mean_intensity = punti["downward_intensity_cd"].mean()
        if np.isnan(mean_intensity):
            raise ValueError(
                f"{punti_path} has no usable 'downward_intensity_cd' values (all missing)."
            )
        punti["downward_intensity_cd"] = punti["downward_intensity_cd"].fillna(mean_intensity)

    target_crs = punti.crs or edges.crs or buildings.crs
    if target_crs is None:
        raise ValueError("No CRS found in lamp/edge/building layers.")

    if punti.crs is None:
        punti = punti.set_crs(target_crs)
    if edges.crs is None:
        edges = edges.set_crs(target_crs)
    elif edges.crs != target_crs:
        edges = edges.to_crs(target_crs)
    if buildings.crs is None:
        buildings = buildings.set_crs(target_crs)
    elif buildings.crs != target_crs:
        buildings = buildings.to_crs(target_crs)

    return punti, edges, buildings


def densify_edges(edges: gpd.GeoDataFrame, spacing: float) -> gpd.GeoDataFrame:
    """Sample points every `spacing` metres along every edge, plus one at its far end.

    The sample positions are computed per edge, because `np.arange` is what decides how many
    points an edge gets and a closed-form count disagrees with it on any edge whose length is near
    a multiple of the spacing. The *interpolation* is then one vectorised call for the whole city
    rather than one `geom.interpolate` per point.
    """
    geoms = edges.geometry.to_numpy()
    index_labels = edges.index.to_numpy()
    # Stable per-edge key for the step-4 join: the edge's own edgeID when available, else the
    # index label. parent_edge_idx (the index label) is kept for this file's own aggregation.
    edge_ids = edges["edgeID"].to_numpy() if "edgeID" in edges.columns else index_labels

    usable = np.fromiter(
        ((g is not None) and (not g.is_empty) for g in geoms), dtype=bool, count=len(geoms)
    )
    positions = np.flatnonzero(usable)

    per_edge = []
    for pos in positions:
        length = geoms[pos].length
        distances = np.arange(0.0, length, spacing)
        if distances.size == 0 or distances[-1] < length:
            distances = np.append(distances, length)
        per_edge.append(distances)

    if not per_edge:
        return gpd.GeoDataFrame(
            {
                "node_id": np.empty(0, dtype=object),
                "parent_edge_idx": np.empty(0, dtype=index_labels.dtype),
                "parent_edge_id": np.empty(0, dtype=edge_ids.dtype),
                "dist_along_edge": np.empty(0, dtype="float64"),
                "x": np.empty(0, dtype="float64"),
                "y": np.empty(0, dtype="float64"),
            },
            geometry=np.empty(0, dtype=object),
            crs=edges.crs,
        )

    counts = np.fromiter((d.size for d in per_edge), dtype=np.int64, count=len(per_edge))
    distances = np.concatenate(per_edge)
    parent = np.repeat(positions, counts)

    sample_points = shapely.line_interpolate_point(geoms[parent], distances)

    return gpd.GeoDataFrame(
        {
            "node_id": np.char.add("node_2m_", np.arange(distances.size).astype(str)),
            "parent_edge_idx": index_labels[parent],
            "parent_edge_id": edge_ids[parent],
            "dist_along_edge": distances,
            "x": shapely.get_x(sample_points),
            "y": shapely.get_y(sample_points),
        },
        geometry=sample_points,
        crs=edges.crs,
    )


def mounted_building_index(punti: gpd.GeoDataFrame, buildings: gpd.GeoDataFrame) -> np.ndarray:
    """For each lamp, the positional index of the building it is mounted on, or -1.

    A lamp inside a footprint is a wall or arcade fixture; that building is not an obstacle for it,
    while every other building still is. Lamps deeper than `ARCADE_DEPTH_M` are inside the block and
    are not treated as mounted, so they stay occluded.
    """
    mounted = np.full(len(punti), -1, dtype=np.int64)
    if buildings.empty:
        return mounted

    lamps = punti[["geometry"]].reset_index(drop=True)
    blds = buildings[["geometry"]].reset_index(drop=True)
    inside = gpd.sjoin(lamps, blds, predicate="within", how="inner")
    if inside.empty:
        return mounted

    lamp_idx = inside.index.to_numpy(dtype=np.int64)
    bld_idx = inside["index_right"].to_numpy(dtype=np.int64)
    depth = shapely.distance(
        lamps.geometry.to_numpy()[lamp_idx],
        shapely.boundary(blds.geometry.to_numpy()[bld_idx]),
    )
    attached = depth <= ARCADE_DEPTH_M
    mounted[lamp_idx[attached]] = bld_idx[attached]
    print(
        f"  lamps mounted on a building (not occluded by it): "
        f"{int(np.count_nonzero(mounted >= 0))} of {len(lamps)}"
    )
    return mounted


def blocked_sight_lines(
    origins: np.ndarray,
    targets: np.ndarray,
    mounted_for_pair: np.ndarray,
    b_tree: shapely.STRtree,
) -> np.ndarray:
    """Which of these (sample point, lamp) sight lines a building stands in.

    One bulk `STRtree` query in place of a `LineString` construction and an index lookup per pair.
    `crosses`, not `intersects`: a sight line that grazes a corner or runs along a wall shares a
    boundary with the footprint without passing through it. `crosses` is symmetric, so querying
    lines against a tree of polygons asks exactly what the per-pair `polygon.crosses(line)` asked.
    """
    count = len(origins)
    coords = np.empty((2 * count, 2), dtype="float64")
    coords[0::2] = origins
    coords[1::2] = targets
    lines = shapely.linestrings(coords, indices=np.repeat(np.arange(count), 2))

    line_idx, bld_idx = b_tree.query(lines, predicate="crosses")
    # A lamp's own building is not an obstacle to it; every other one is.
    obstructing = bld_idx != mounted_for_pair[line_idx]

    blocked = np.zeros(count, dtype=bool)
    blocked[line_idx[obstructing]] = True
    return blocked


def compute_lux(points: gpd.GeoDataFrame, punti: gpd.GeoDataFrame, buildings: gpd.GeoDataFrame) -> np.ndarray:
    if points.empty:
        return np.array([], dtype="float64")

    lamp_coords = np.column_stack(
        (punti.geometry.x.to_numpy(dtype="float64"), punti.geometry.y.to_numpy(dtype="float64"))
    )
    pt_coords = np.column_stack(
        (points["x"].to_numpy(dtype="float64"), points["y"].to_numpy(dtype="float64"))
    )
    intensity = punti["downward_intensity_cd"].to_numpy(dtype="float64")
    heights = punti["altezza_palo_m"].to_numpy(dtype="float64")

    # Derived, not chosen: the distance at which the strongest lamp in this inventory falls below
    # lighting.NEGLIGIBLE_LUX. The flat 40 m this replaces was undocumented and happened to sit
    # near the 0.05-0.1 lux contour of a typical 100 W lamp at 9 m - right by luck, and wrong for
    # any city whose lamps are taller or brighter.
    search_radius = lighting.summation_radius_m(intensity, heights)
    print(f"  lamp search radius: {search_radius:.1f} m "
          f"(where the strongest lamp reaches {lighting.NEGLIGIBLE_LUX} lux)")

    lamp_tree = cKDTree(lamp_coords)

    has_buildings = not buildings.empty
    if has_buildings:
        b_tree = shapely.STRtree(buildings.geometry.to_numpy())
        mounted = mounted_building_index(punti, buildings)
    else:
        b_tree = None
        mounted = None

    total_points = len(points)
    lux = np.zeros(total_points, dtype="float64")
    pairs_seen = 0

    for start in range(0, total_points, POINT_BLOCK):
        stop = min(start + POINT_BLOCK, total_points)
        neighbours = lamp_tree.query_ball_point(
            pt_coords[start:stop], search_radius, workers=-1, return_sorted=False
        )
        counts = np.fromiter((len(n) for n in neighbours), dtype=np.int64, count=stop - start)
        pair_count = int(counts.sum())
        if pair_count == 0:
            continue
        pairs_seen += pair_count

        pair_point = np.repeat(np.arange(start, stop), counts)
        pair_lamp = np.fromiter(
            itertools.chain.from_iterable(neighbours), dtype=np.int64, count=pair_count
        )

        origins = pt_coords[pair_point]
        targets = lamp_coords[pair_lamp]
        # d is the HORIZONTAL lamp-to-point distance; the tilt is already in the formula.
        d = np.hypot(targets[:, 0] - origins[:, 0], targets[:, 1] - origins[:, 1])
        contribution = lighting.illuminance_lux(intensity[pair_lamp], heights[pair_lamp], d)

        if has_buildings:
            for lo in range(0, pair_count, SIGHT_LINE_BLOCK):
                hi = min(lo + SIGHT_LINE_BLOCK, pair_count)
                blocked = blocked_sight_lines(
                    origins[lo:hi], targets[lo:hi], mounted[pair_lamp[lo:hi]], b_tree
                )
                contribution[lo:hi][blocked] = 0.0

        lux[start:stop] = np.bincount(
            pair_point - start, weights=contribution, minlength=stop - start
        )

        if (start // POINT_BLOCK) % 10 == 0 or stop == total_points:
            print(
                f"    {stop}/{total_points} sample points, {pairs_seen} lamp sight lines",
                flush=True,
            )

    return lux


def report_distribution(edges: gpd.GeoDataFrame) -> None:
    """The numbers a falloff-law comparison is decided on, beside the law that produced them."""
    pct_unlit = edges["pct_unlit"].to_numpy(dtype="float64")
    mean_lux = edges["mean_lux"].to_numpy(dtype="float64")
    min_lux = edges["min_lux"].to_numpy(dtype="float64")
    print(f"  {lighting.describe_law()}")
    print(f"  edges: {len(edges)}")
    print(f"  pct_unlit  mean {pct_unlit.mean():6.2f}  median {np.median(pct_unlit):6.2f}")
    print(f"             fully unlit (100%): {int((pct_unlit >= 100.0).sum())} "
          f"({100.0 * (pct_unlit >= 100.0).mean():.1f}%)")
    print(f"             fully lit     (0%): {int((pct_unlit <= 0.0).sum())} "
          f"({100.0 * (pct_unlit <= 0.0).mean():.1f}%)")
    print(f"  mean_lux   mean {mean_lux.mean():8.2f}  median {np.median(mean_lux):8.2f}")
    print(f"             below the {lighting.MIN_LUX} lux service level: "
          f"{int((mean_lux < lighting.MIN_LUX).sum())} "
          f"({100.0 * (mean_lux < lighting.MIN_LUX).mean():.1f}%)")
    print(f"  min_lux    mean {min_lux.mean():8.2f}  median {np.median(min_lux):8.2f}")


def main() -> None:
    parser = argparse.ArgumentParser(description="Calculate street illumination along network edges.")
    parser.add_argument("--city", required=True,
                        help="City name: folder under inputData/ and src/main/resources/, "
                             "and the <City>_* file prefix.")
    parser.add_argument("--falloff-law", choices=lighting.LAW_CHOICES, default=lighting.FALLOFF_LAW,
                        help="Photometric falloff law for this run. See pipeline/lighting.py; "
                             "step 2 must have been run with the same one.")
    parser.add_argument("--variant", default="",
                        help="Suffix for this run's files (e.g. 'lambertian'). Reads the step-2 "
                             "output of the same name and writes the edges layer to inputData/ "
                             "rather than resources/, so a comparison run cannot overwrite the "
                             "layer the simulation reads.")
    args = parser.parse_args()
    city = args.city
    lighting.set_law(args.falloff_law)
    variant = f"_{args.variant}" if args.variant else ""

    punti, edges, buildings = load_inputs(city, variant)

    print("Densifying edges to 2 m sample points...")
    points = densify_edges(edges, SAMPLE_SPACING_M)
    print(f"  {len(points)} sample points.")

    print(f"Computing line-of-sight lux per sample point ({lighting.describe_law()})...")
    points["calculated_lux"] = compute_lux(points, punti, buildings)
    points["is_unlit"] = points["calculated_lux"] < lighting.MIN_LUX

    edge_stats = points.groupby("parent_edge_idx").agg(
        min_lux=("calculated_lux", "min"),
        mean_lux=("calculated_lux", "mean"),
        pct_unlit=("is_unlit", lambda x: (x.sum() / len(x)) * 100),
    )

    edges = edges.join(edge_stats)
    edges["min_lux"] = edges["min_lux"].fillna(0.0)
    edges["mean_lux"] = edges["mean_lux"].fillna(0.0)
    edges["pct_unlit"] = edges["pct_unlit"].fillna(100.0)

    report_distribution(edges)

    # Sim-read output goes to resources; the densified nodes are an intermediate for step 4.
    if variant:
        edges_out = paths.raw_dir(city) / f"{city}_edges_illuminated_continuous{variant}.gpkg"
    else:
        edges_out = paths.resources_dir(city) / f"{city}_edges_illuminated_continuous.gpkg"
    remove_existing(edges_out)
    edges.to_file(edges_out, driver="GPKG")
    print(f"saved: {edges_out}")

    nodes_out = paths.raw_dir(city) / f"{city}_nodes_2m_densified_illuminated{variant}.gpkg"
    remove_existing(nodes_out)
    points.drop(columns=["x", "y"]).to_file(nodes_out, driver="GPKG")
    print(f"saved: {nodes_out} ({len(points)} nodes)")


if __name__ == "__main__":
    main()
