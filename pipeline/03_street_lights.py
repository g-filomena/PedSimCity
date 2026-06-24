"""
Step 3: street-light illumination.

Computes continuous illuminance (lux) along the street network from the lamp inventory using a
point-source model with building occlusion, and produces two layers from the SAME 2 m sample points
and the SAME line-of-sight computation (done once):

  <City>_edges_illuminated_continuous.gpkg   per-edge min/mean lux + % unlit   (read by Java)
  nodes_2m_densified_illuminated.gpkg        per 2 m sample-point lux          (input to step 4)

Per-point illuminance:  E = I * h / (h^2 + d^2)^1.5  -- inverse-square law with cosine-of-incidence
correction; I = downward intensity (cd), h = pole height (m), d = horizontal lamp-point distance (m).
"""

import argparse
import os

import geopandas as gpd
import numpy as np
from scipy.spatial import cKDTree
from shapely.geometry import LineString

SAMPLE_SPACING_M = 2.0
LAMP_SEARCH_RADIUS_M = 40.0
UNLIT_LUX_THRESHOLD = 5.0


def load_inputs(base_dir, prefix):
    """Loads the lamp / edge / building layers, imputes missing lamp physics, and aligns CRS."""
    city = os.path.basename(os.path.normpath(base_dir))

    punti_path = os.path.join(base_dir, f"{prefix}puntiLuce_with_radius.gpkg")
    if not os.path.exists(punti_path):
        punti_path = os.path.join(base_dir, f"{prefix}puntiLuce_updated.gpkg")
    if not os.path.exists(punti_path):
        punti_path = os.path.join(base_dir, f"{prefix}puntiLuce.gpkg")

    edges_path = os.path.join(base_dir, f"{prefix}edges.gpkg")

    # Buildings provide the occluders. Fall back to the city-folder name when the prefixed file is
    # absent (e.g. raw prefix 'Torino_' vs city folder 'TorinoCentre').
    buildings_path = os.path.join(base_dir, f"{prefix}buildings.gpkg")
    if not os.path.exists(buildings_path):
        buildings_path = os.path.join(base_dir, f"{city}_buildings.gpkg")

    print("Loading datasets...")
    punti = gpd.read_file(punti_path)
    edges = gpd.read_file(edges_path)
    buildings = gpd.read_file(buildings_path)

    # Impute missing lamp physics so the lux sum never propagates NaNs.
    if "altezza_palo_m" in punti.columns and punti["altezza_palo_m"].isnull().any():
        if "uso_ottica" in punti.columns:
            punti["altezza_palo_m"] = punti.groupby("uso_ottica")["altezza_palo_m"].transform(
                lambda x: x.fillna(x.median() if not x.dropna().empty else 9.0))
        punti["altezza_palo_m"] = punti["altezza_palo_m"].fillna(9.0)

    if "downward_intensity_cd" not in punti.columns:
        print("Warning: no downward_intensity_cd column; defaulting to 1000.0 cd.")
        punti["downward_intensity_cd"] = 1000.0
    elif punti["downward_intensity_cd"].isnull().any():
        if "tecnologia" in punti.columns:
            punti["downward_intensity_cd"] = punti.groupby("tecnologia")["downward_intensity_cd"].transform(
                lambda x: x.fillna(x.mean() if not x.dropna().empty else 0.0))
        punti["downward_intensity_cd"] = punti["downward_intensity_cd"].fillna(
            punti["downward_intensity_cd"].mean())

    # Work in the lamp layer's CRS.
    target_crs = punti.crs
    if edges.crs is None:
        edges.set_crs(target_crs, inplace=True)
    elif edges.crs != target_crs:
        edges = edges.to_crs(target_crs)
    if buildings.crs != target_crs:
        buildings = buildings.to_crs(target_crs)

    return city, punti, edges, buildings


def densify_edges(edges, spacing):
    """Sample points every `spacing` metres along each edge, tagged with their parent edge index
    and distance along it (the latter is needed by the directional-lighting step)."""
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
            pt = geom.interpolate(dist)
            rows.append({
                "node_id": f"node_2m_{counter}",
                "parent_edge_idx": idx,
                "dist_along_edge": dist,
                "geometry": pt,
                "x": pt.x,
                "y": pt.y,
            })
            counter += 1
    return gpd.GeoDataFrame(rows, crs=edges.crs)


def compute_lux(points, punti, buildings):
    """Horizontal illuminance (lux) at each sample point: sum over nearby, unobstructed lamps of the
    point-source illuminance E = I * h / (h^2 + d^2)^1.5. Building footprints block the line of
    sight."""
    lamp_coords = np.column_stack((punti.geometry.x, punti.geometry.y))
    pt_coords = np.column_stack((points["x"], points["y"]))
    intensity = punti["downward_intensity_cd"].values
    heights = punti["altezza_palo_m"].values

    tree = cKDTree(lamp_coords)
    nearby = tree.query_ball_point(pt_coords, LAMP_SEARCH_RADIUS_M)
    b_sindex = buildings.sindex
    b_geom = buildings.geometry.values

    lux = np.zeros(len(points))
    for i, lamp_list in enumerate(nearby):
        if not lamp_list:
            continue
        px, py = pt_coords[i]
        for lamp_idx in lamp_list:
            lx, ly = lamp_coords[lamp_idx]
            sight = LineString([(px, py), (lx, ly)])
            blocked = any(b_geom[b].intersects(sight)
                          for b in b_sindex.intersection(sight.bounds))
            if not blocked:
                d = np.hypot(lx - px, ly - py)
                h = heights[lamp_idx]
                lux[i] += (intensity[lamp_idx] * h) / ((h ** 2 + d ** 2) ** 1.5)
    return lux


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--input_dir", required=True)
    parser.add_argument("--prefix", required=True)
    args = parser.parse_args()

    city, punti, edges, buildings = load_inputs(args.input_dir, args.prefix)

    print("Densifying edges to 2 m sample points...")
    points = densify_edges(edges, SAMPLE_SPACING_M)
    print(f"  {len(points)} sample points.")

    print("Computing line-of-sight lux per sample point...")
    points["calculated_lux"] = compute_lux(points, punti, buildings)

    # --- Per-edge aggregation (read by Java: mean_lux) ---
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

    edges_out = os.path.join(args.input_dir, f"{city}_edges_illuminated_continuous.gpkg")
    edges.to_file(edges_out, driver="GPKG")
    print(f"saved: {edges_out}")

    # --- Per 2 m sample-point export (input to step 4, directional lighting) ---
    nodes_out = os.path.join(args.input_dir, "nodes_2m_densified_illuminated.gpkg")
    points.drop(columns=["x", "y"]).to_file(nodes_out, driver="GPKG")
    print(f"saved: {nodes_out}  ({len(points)} nodes)")


if __name__ == "__main__":
    main()
