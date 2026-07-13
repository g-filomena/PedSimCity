"""Step 3: compute continuous street illumination.

Inputs (searched in inputData/<City>/ then the resources folder):
- <City>_streetlights_with_radius.gpkg (step-2 output)
- <City>_edges.gpkg
- <City>_buildings.gpkg

Outputs:
- <City>_edges_illuminated_continuous.gpkg  -> src/main/resources/<City>/ (read by the sim)
- <City>_nodes_2m_densified_illuminated.gpkg -> inputData/<City>/ (intermediate for step 4)

`--city` is the city name (folder under inputData/ and resources/, and file prefix).
"""

from __future__ import annotations

import argparse
from pathlib import Path

import geopandas as gpd
import numpy as np
from scipy.spatial import cKDTree
from shapely.geometry import LineString

import paths


SAMPLE_SPACING_M = 2.0
LAMP_SEARCH_RADIUS_M = 40.0
UNLIT_LUX_THRESHOLD = 5.0


def remove_existing(path: Path) -> None:
    if path.exists():
        path.unlink()


def load_inputs(city: str):
    # Require the step-2 output (*_with_radius.gpkg): it carries the per-lamp physics
    # (downward_intensity_cd) this step needs. The raw inventory is intentionally NOT accepted so
    # that step 3 can never silently fabricate uniform lamp intensity — run step 2 first.
    punti_path = paths.require_input(
        city,
        "streetlights_with_radius.gpkg",
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

    if "altezza_palo_m" not in punti.columns:
        punti["altezza_palo_m"] = 9.0
    else:
        punti["altezza_palo_m"] = punti["altezza_palo_m"].fillna(9.0)

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
    # Stable per-edge key for the step-4 join: the edge's own edgeID when available, else the
    # positional index. parent_edge_idx (positional) is kept for this file's own aggregation.
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


def compute_lux(points: gpd.GeoDataFrame, punti: gpd.GeoDataFrame, buildings: gpd.GeoDataFrame) -> np.ndarray:
    if points.empty:
        return np.array([], dtype="float64")

    lamp_coords = np.column_stack((punti.geometry.x, punti.geometry.y))
    pt_coords = np.column_stack((points["x"], points["y"]))
    intensity = punti["downward_intensity_cd"].to_numpy(dtype="float64")
    heights = punti["altezza_palo_m"].to_numpy(dtype="float64")

    tree = cKDTree(lamp_coords)
    nearby = tree.query_ball_point(pt_coords, LAMP_SEARCH_RADIUS_M)

    has_buildings = not buildings.empty
    if has_buildings:
        b_sindex = buildings.sindex
        b_geom = buildings.geometry.values
    else:
        b_sindex = None
        b_geom = None

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
                blocked = any(b_geom[b].intersects(sight) for b in b_sindex.intersection(sight.bounds))
            if not blocked:
                d = np.hypot(lx - px, ly - py)
                h = heights[lamp_idx]
                lux[i] += (intensity[lamp_idx] * h) / ((h**2 + d**2) ** 1.5)
    return lux


def main() -> None:
    parser = argparse.ArgumentParser(description="Calculate street illumination along network edges.")
    parser.add_argument("--city", required=True,
                        help="City name: folder under inputData/ and src/main/resources/, "
                             "and the <City>_* file prefix.")
    args = parser.parse_args()
    city = args.city

    punti, edges, buildings = load_inputs(city)

    print("Densifying edges to 2 m sample points...")
    points = densify_edges(edges, SAMPLE_SPACING_M)
    print(f"  {len(points)} sample points.")

    print("Computing line-of-sight lux per sample point...")
    points["calculated_lux"] = compute_lux(points, punti, buildings)
    points["is_unlit"] = points["calculated_lux"] < UNLIT_LUX_THRESHOLD

    edge_stats = points.groupby("parent_edge_idx").agg(
        min_lux=("calculated_lux", "min"),
        mean_lux=("calculated_lux", "mean"),
        pct_unlit=("is_unlit", lambda x: (x.sum() / len(x)) * 100),
    )

    edges = edges.join(edge_stats)
    edges["min_lux"] = edges["min_lux"].fillna(0.0)
    edges["mean_lux"] = edges["mean_lux"].fillna(0.0)
    edges["pct_unlit"] = edges["pct_unlit"].fillna(100.0)

    # Sim-read output goes to resources; the densified nodes are an intermediate for step 4.
    edges_out = paths.resources_dir(city) / f"{city}_edges_illuminated_continuous.gpkg"
    remove_existing(edges_out)
    edges.to_file(edges_out, driver="GPKG")
    print(f"saved: {edges_out}")

    nodes_out = paths.raw_dir(city) / f"{city}_nodes_2m_densified_illuminated.gpkg"
    remove_existing(nodes_out)
    points.drop(columns=["x", "y"]).to_file(nodes_out, driver="GPKG")
    print(f"saved: {nodes_out} ({len(points)} nodes)")


if __name__ == "__main__":
    main()
