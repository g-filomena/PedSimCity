"""Check the vectorised step 3 against the per-pair loop it replaces, on a slice of a real city.

Run from the pipeline folder. It rebuilds the OLD densify_edges and compute_lux verbatim, runs
both over the same subset of edges, and compares sample points and lux element by element.
"""

from __future__ import annotations

import argparse
import importlib.util
import sys
import time
from pathlib import Path

import geopandas as gpd
import numpy as np
from scipy.spatial import cKDTree
from shapely.geometry import LineString

import lighting
import paths

spec = importlib.util.spec_from_file_location("step3", Path(__file__).resolve().parent / "03_street_lights.py")
step3 = importlib.util.module_from_spec(spec)
sys.modules["step3"] = step3
spec.loader.exec_module(step3)


# ---------------------------------------------------------------- the code being replaced


def old_densify_edges(edges, spacing):
    has_edge_id = "edgeID" in edges.columns
    rows = []
    counter = 0
    for idx, row in edges.iterrows():
        geom = row.geometry
        if geom is None or geom.is_empty:
            continue
        edge_id = row["edgeID"] if has_edge_id else idx
        length = geom.length
        distances = np.arange(0, length, spacing)
        if len(distances) == 0 or distances[-1] < length:
            distances = np.append(distances, length)
        for dist in distances:
            pt = geom.interpolate(float(dist))
            rows.append(
                {
                    "node_id": f"node_2m_{counter}",
                    "parent_edge_idx": idx,
                    "parent_edge_id": edge_id,
                    "dist_along_edge": float(dist),
                    "geometry": pt,
                    "x": pt.x,
                    "y": pt.y,
                }
            )
            counter += 1
    return gpd.GeoDataFrame(rows, crs=edges.crs)


def old_mounting_buildings(punti, buildings):
    if buildings.empty:
        return {}
    lamps = punti[["geometry"]].reset_index(drop=True)
    blds = buildings[["geometry"]].reset_index(drop=True)
    inside = gpd.sjoin(lamps, blds, predicate="within", how="inner")
    if inside.empty:
        return {}
    boundaries = blds.geometry.boundary
    mounted = {}
    for lamp_idx, bld_idx in zip(inside.index, inside["index_right"]):
        depth = lamps.geometry.iloc[lamp_idx].distance(boundaries.iloc[bld_idx])
        if depth <= step3.ARCADE_DEPTH_M:
            mounted[int(lamp_idx)] = int(bld_idx)
    return mounted


def old_compute_lux(points, punti, buildings):
    if points.empty:
        return np.array([], dtype="float64")
    lamp_coords = np.column_stack((punti.geometry.x, punti.geometry.y))
    pt_coords = np.column_stack((points["x"], points["y"]))
    intensity = punti["downward_intensity_cd"].to_numpy(dtype="float64")
    heights = punti["altezza_palo_m"].to_numpy(dtype="float64")
    search_radius = lighting.summation_radius_m(intensity, heights)
    tree = cKDTree(lamp_coords)
    nearby = tree.query_ball_point(pt_coords, search_radius)
    has_buildings = not buildings.empty
    if has_buildings:
        blds = buildings.reset_index(drop=True)
        b_sindex = blds.sindex
        b_geom = blds.geometry.values
        mounted = old_mounting_buildings(punti, buildings)
    else:
        b_sindex = None
        b_geom = None
        mounted = {}
    lux = np.zeros(len(points), dtype="float64")
    for i, lamp_list in enumerate(nearby):
        if not lamp_list:
            continue
        px, py = pt_coords[i]
        for lamp_idx in lamp_list:
            lx, ly = lamp_coords[lamp_idx]
            blocked = False
            if has_buildings:
                sight = LineString([(px, py), (lx, ly)])
                own = mounted.get(lamp_idx)
                blocked = any(
                    b != own and b_geom[b].crosses(sight)
                    for b in b_sindex.intersection(sight.bounds)
                )
            if not blocked:
                d = np.hypot(lx - px, ly - py)
                lux[i] += lighting.illuminance_lux(intensity[lamp_idx], heights[lamp_idx], d)
    return lux


# ---------------------------------------------------------------- the comparison


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--city", required=True)
    ap.add_argument("--edges", type=int, default=400, help="How many edges to slice out.")
    ap.add_argument("--seed", type=int, default=20260916)
    args = ap.parse_args()

    punti, edges, buildings = step3.load_inputs(args.city, "")
    rng = np.random.default_rng(args.seed)
    take = rng.choice(len(edges), size=min(args.edges, len(edges)), replace=False)
    subset = edges.iloc[np.sort(take)].copy()
    print(f"subset: {len(subset)} edges")

    t0 = time.perf_counter()
    old_points = old_densify_edges(subset, step3.SAMPLE_SPACING_M)
    t_old_d = time.perf_counter() - t0

    t0 = time.perf_counter()
    new_points = step3.densify_edges(subset, step3.SAMPLE_SPACING_M)
    t_new_d = time.perf_counter() - t0

    assert len(old_points) == len(new_points), (len(old_points), len(new_points))
    for col in ("node_id", "parent_edge_idx", "parent_edge_id"):
        assert (old_points[col].to_numpy() == new_points[col].to_numpy()).all(), col
    np.testing.assert_allclose(
        old_points["dist_along_edge"].to_numpy(), new_points["dist_along_edge"].to_numpy()
    )
    np.testing.assert_allclose(old_points["x"].to_numpy(), new_points["x"].to_numpy(), atol=1e-9)
    np.testing.assert_allclose(old_points["y"].to_numpy(), new_points["y"].to_numpy(), atol=1e-9)
    print(f"densify: {len(new_points)} points identical "
          f"(loop {t_old_d:.1f}s -> vector {t_new_d:.1f}s, {t_old_d / max(t_new_d, 1e-9):.0f}x)")

    t0 = time.perf_counter()
    old_lux = old_compute_lux(old_points, punti, buildings)
    t_old_l = time.perf_counter() - t0

    t0 = time.perf_counter()
    new_lux = step3.compute_lux(new_points, punti, buildings)
    t_new_l = time.perf_counter() - t0

    diff = np.abs(old_lux - new_lux)
    worst = float(diff.max()) if diff.size else 0.0
    np.testing.assert_allclose(old_lux, new_lux, rtol=1e-9, atol=1e-9)
    print(f"lux: identical (worst abs diff {worst:.3e}) over {len(new_lux)} points")
    print(f"     loop {t_old_l:.1f}s -> vector {t_new_l:.1f}s, {t_old_l / max(t_new_l, 1e-9):.0f}x")
    print(f"     mean lux {new_lux.mean():.2f}, unlit share "
          f"{100.0 * (new_lux < lighting.MIN_LUX).mean():.1f}%")

    full = 1_081_000
    print(f"projected full-city compute_lux: loop "
          f"{t_old_l * full / len(new_lux) / 3600:.2f} h -> vector "
          f"{t_new_l * full / len(new_lux) / 60:.1f} min")


if __name__ == "__main__":
    main()
