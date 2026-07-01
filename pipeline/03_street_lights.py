"""Step 3: compute continuous street illumination.

All layers are named <City>_… (City = the resources folder name = Pars.cityName).

Inputs:
- a lamp layer with radius/intensity: <City>_puntiLuce_with_radius.gpkg (Turin) or
  <City>_streetlights_with_radius.gpkg (generic)
- <City>_edges.gpkg
- <City>_buildings.gpkg

Outputs (the names the Java NightImport loader expects):
- <City>_edges_illuminated_continuous.gpkg   (read by Java)
- <City>_nodes_2m_densified_illuminated.gpkg (intermediate, consumed by step 4)
"""

from __future__ import annotations

import argparse
import os
from pathlib import Path

import geopandas as gpd
import numpy as np
from scipy.spatial import cKDTree
from shapely.geometry import LineString

SAMPLE_SPACING_M = 2.0
LAMP_SEARCH_RADIUS_M = 40.0
UNLIT_LUX_THRESHOLD = 5.0


def first_existing(paths: list[Path], label: str) -> Path:
    for path in paths:
        if path.exists():
            return path
    joined = "\n  ".join(str(p) for p in paths)
    raise FileNotFoundError(f"Could not find {label}. Tried:\n  {joined}")


def remove_existing(path: Path) -> None:
    if path.exists():
        path.unlink()


def load_inputs(base_dir: Path):
    city = base_dir.name

    punti_path = first_existing(
        [
            base_dir / f"{city}_puntiLuce_with_radius.gpkg",       # Turin adapter (02_street_lights_torino.py)
            base_dir / f"{city}_streetlights_with_radius.gpkg",    # generic adapter (02_street_lights_generic.py)
            base_dir / f"{city}_puntiLuce.gpkg",                   # fallback: raw Turin inventory
        ],
        "lamp inventory",
    )

    edges_path = first_existing(
        [base_dir / f"{city}_edges.gpkg"],
        "edges layer",
    )

    buildings_path = first_existing(
        [base_dir / f"{city}_buildings.gpkg"],
        "buildings layer",
    )

    print("Loading datasets...")
    print(f" lamps    : {punti_path}")
    print(f" edges    : {edges_path}")
    print(f" buildings: {buildings_path}")

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

    if "downward_intensity_cd" not in punti.columns:
        print("Warning: no downward_intensity_cd column; defaulting to 1000.0 cd.")
        punti["downward_intensity_cd"] = 1000.0
    else:
        if punti["downward_intensity_cd"].isnull().any():
            if "tecnologia" in punti.columns:
                punti["downward_intensity_cd"] = punti.groupby("tecnologia")["downward_intensity_cd"].transform(
                    lambda x: x.fillna(x.mean() if not x.dropna().empty else 0.0)
                )
            mean_intensity = punti["downward_intensity_cd"].mean()
            punti["downward_intensity_cd"] = punti["downward_intensity_cd"].fillna(
                mean_intensity if not np.isnan(mean_intensity) else 1000.0
            )

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

    return city, punti, edges, buildings


def densify_edges(edges: gpd.GeoDataFrame, spacing: float) -> gpd.GeoDataFrame:
    rows = []
    counter = 0

    for idx, row in edges.iterrows():
        geom = row.geometry
        if geom is None or geom.is_empty:
            continue

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
    parser.add_argument("--input_dir", required=True)
    args = parser.parse_args()

    base_dir = Path(os.path.abspath(args.input_dir))

    city, punti, edges, buildings = load_inputs(base_dir)

    print("Densifying edges to 2 m sample points...")
    points = densify_edges(edges, SAMPLE_SPACING_M)
    print(f" {len(points)} sample points.")

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

    edges_out = base_dir / f"{city}_edges_illuminated_continuous.gpkg"
    remove_existing(edges_out)
    edges.to_file(edges_out, driver="GPKG")
    print(f"saved: {edges_out}")

    nodes_out = base_dir / f"{city}_nodes_2m_densified_illuminated.gpkg"
    remove_existing(nodes_out)
    points.drop(columns=["x", "y"]).to_file(nodes_out, driver="GPKG")
    print(f"saved: {nodes_out} ({len(points)} nodes)")


if __name__ == "__main__":
    main()
