import os
import geopandas as gpd
import numpy as np
from scipy.spatial import cKDTree
from shapely.geometry import LineString

# ------------------------------------------
# 1. Setup paths and load source layers
# ------------------------------------------
import argparse
import sys
parser = argparse.ArgumentParser()
parser.add_argument('--input_dir', required=True)
parser.add_argument('--prefix', required=True)
args = parser.parse_args()
base_dir = args.input_dir
prefix = args.prefix



punti_path = os.path.join(base_dir, f"{prefix}puntiLuce_with_radius.gpkg")
if not os.path.exists(punti_path):
    punti_path = os.path.join(base_dir, f"{prefix}puntiLuce_updated.gpkg")
if not os.path.exists(punti_path):
    punti_path = os.path.join(base_dir, f"{prefix}puntiLuce.gpkg")

edges_path = os.path.join(base_dir, f"{prefix}edges.gpkg")
buildings_path = os.path.join(base_dir, f"{prefix}buildings.gpkg")
if not os.path.exists(buildings_path):
    buildings_path = os.path.join(base_dir, "TorinoCentre_buildings.gpkg")
output_path = os.path.join(base_dir, "edges_illuminated_continuous.gpkg")

print("Loading datasets...")

punti = gpd.read_file(punti_path)
if 'radius' not in punti.columns and 'radius_m' not in punti.columns:
    print('Warning: No radius column found. Defaulting to 15m radius.')
    punti['radius'] = 15.0
if 'lux' not in punti.columns and 'intensity' not in punti.columns:
    print('Warning: No lux/intensity column found. Defaulting to 20.0.')
    punti['lux'] = 20.0

edges = gpd.read_file(edges_path)
buildings = gpd.read_file(buildings_path)

# ------------------------------------------
# 2. Align CRS and prepare building index
# ------------------------------------------
target_crs = punti.crs

if edges.crs is None: edges.set_crs(target_crs, inplace=True)
elif edges.crs != target_crs: edges = edges.to_crs(target_crs)

if buildings.crs != target_crs:
    buildings = buildings.to_crs(target_crs)

buildings_sindex = buildings.sindex
buildings_geom = buildings.geometry.values

# ------------------------------------------
# 3. Densify road edges every 2 meters
# ------------------------------------------
print("Generating 2-meter sample points along edges...")
sample_points = []

for idx, row in edges.iterrows():
    geom = row.geometry
    if geom is None or geom.is_empty:
        continue

    length = geom.length
    distances = np.arange(0, length, 2.0)
    if len(distances) == 0 or distances[-1] < length:
        distances = np.append(distances, length)

    for dist in distances:
        pt = geom.interpolate(dist)
        sample_points.append({
            "edge_idx": idx,
            "geometry": pt,
            "x": pt.x,
            "y": pt.y,
        })

sample_points = gpd.GeoDataFrame(sample_points, crs=target_crs)
print(f"Created {len(sample_points)} sample points.")

# ------------------------------------------
# 4. Find nearby lamps for each sample point
# ------------------------------------------
print("Building KDTree for lamp proximity...")
punti_coords = np.column_stack((punti.geometry.x, punti.geometry.y))
sample_coords = np.column_stack((sample_points["x"], sample_points["y"]))

tree = cKDTree(punti_coords)
search_radius = 40.0
nearby_lamps_indices = tree.query_ball_point(sample_coords, search_radius)

# ------------------------------------------
# 5. Compute Lux for each sample point with occlusion
# ------------------------------------------

if 'downward_intensity_cd' not in punti.columns:
    print('Warning: No downward_intensity_cd column found. Defaulting to 1000.0.')
    punti['downward_intensity_cd'] = 1000.0

print("Computing continuous Lux values for sample points...")
intensity = punti["downward_intensity_cd"].values
heights = punti["altezza_palo_m"].values
pt_lux_totals = np.zeros(len(sample_points))

for i, lamp_idx_list in enumerate(nearby_lamps_indices):
    if not lamp_idx_list:
        continue

    sx, sy = sample_coords[i]

    for lamp_idx in lamp_idx_list:
        lx, ly = punti_coords[lamp_idx]
        line_of_sight = LineString([(sx, sy), (lx, ly)])

        possible_blockers_idx = list(buildings_sindex.intersection(line_of_sight.bounds))
        is_blocked = False

        for b_idx in possible_blockers_idx:
            if buildings_geom[b_idx].intersects(line_of_sight):
                is_blocked = True
                break

        if not is_blocked:
            d = np.hypot(lx - sx, ly - sy)
            I = intensity[lamp_idx]
            h = heights[lamp_idx]
            pt_lux_totals[i] += (I * h) / ((h**2 + d**2) ** 1.5)

sample_points["calculated_lux"] = pt_lux_totals

# ------------------------------------------
# 6. Aggregate sample points back to street edges
# ------------------------------------------
print("Aggregating sample point metrics to edges...")
sample_points["is_unlit"] = sample_points["calculated_lux"] < 5.0

edge_stats = sample_points.groupby("edge_idx").agg(
    min_lux=("calculated_lux", "min"),
    mean_lux=("calculated_lux", "mean"),
    pct_unlit=("is_unlit", lambda x: (x.sum() / len(x)) * 100),
)

edges = edges.join(edge_stats)
edges["min_lux"] = edges["min_lux"].fillna(0.0)
edges["mean_lux"] = edges["mean_lux"].fillna(0.0)
edges["pct_unlit"] = edges["pct_unlit"].fillna(100.0)

# ------------------------------------------
# 7. Export illuminated edge network
# ------------------------------------------
print("Saving illuminated street edge layer...")
edges.to_file(output_path, driver="GPKG")
print(f"Saved illuminated edges to: {output_path}")


import os
import geopandas as gpd
import numpy as np
from scipy.spatial import cKDTree
from shapely.geometry import LineString

# ------------------------------------------
# 1. Setup paths and load source data
# ------------------------------------------
import argparse
import sys
parser = argparse.ArgumentParser()
parser.add_argument('--input_dir', required=True)
parser.add_argument('--prefix', required=True)
args = parser.parse_args()
base_dir = args.input_dir
prefix = args.prefix



punti_path = os.path.join(base_dir, f"{prefix}puntiLuce_with_radius.gpkg")
if not os.path.exists(punti_path):
    punti_path = os.path.join(base_dir, f"{prefix}puntiLuce_updated.gpkg")
if not os.path.exists(punti_path):
    punti_path = os.path.join(base_dir, f"{prefix}puntiLuce.gpkg")

edges_path = os.path.join(base_dir, f"{prefix}edges.gpkg")
buildings_path = os.path.join(base_dir, f"{prefix}buildings.gpkg")
if not os.path.exists(buildings_path):
    buildings_path = os.path.join(base_dir, "TorinoCentre_buildings.gpkg")
output_path = os.path.join(base_dir, "nodes_2m_densified_illuminated.gpkg")

print("Loading datasets...")

punti = gpd.read_file(punti_path)
if 'radius' not in punti.columns and 'radius_m' not in punti.columns:
    print('Warning: No radius column found. Defaulting to 15m radius.')
    punti['radius'] = 15.0
if 'lux' not in punti.columns and 'intensity' not in punti.columns:
    print('Warning: No lux/intensity column found. Defaulting to 20.0.')
    punti['lux'] = 20.0

edges = gpd.read_file(edges_path)
buildings = gpd.read_file(buildings_path)

# ------------------------------------------
# 2. Impute Missing Values (NaN Fail-Safe)
# ------------------------------------------
print("Checking and handling missing data fallbacks...")

# Impute pole heights using the median of their specific optics group (uso_ottica)
if punti["altezza_palo_m"].isnull().any():
    print("-> Filling missing pole heights based on optics categories...")
    punti["altezza_palo_m"] = punti.groupby("uso_ottica")["altezza_palo_m"].transform(
        lambda x: x.fillna(x.median() if not x.dropna().empty else 9.0)
    )
    # Global fallback if an entire category was completely empty
    punti["altezza_palo_m"] = punti["altezza_palo_m"].fillna(9.0)

# Impute downward intensity using the mean of their technology group (tecnologia)
if 'downward_intensity_cd' not in punti.columns:
    print('Warning: No downward_intensity_cd column found. Defaulting to 1000.0.')
    punti['downward_intensity_cd'] = 1000.0
elif punti["downward_intensity_cd"].isnull().any():
    print("-> Filling missing intensity values based on technology categories...")
    punti["downward_intensity_cd"] = punti.groupby("tecnologia")["downward_intensity_cd"].transform(
        lambda x: x.fillna(x.mean() if not x.dropna().empty else 0.0)
    )
    # Global fallback using network average intensity
    punti["downward_intensity_cd"] = punti["downward_intensity_cd"].fillna(punti["downward_intensity_cd"].mean())

# ------------------------------------------
# 3. Align CRS and prepare building index
# ------------------------------------------
target_crs = punti.crs

if edges.crs is None: edges.set_crs(target_crs, inplace=True)
elif edges.crs != target_crs: edges = edges.to_crs(target_crs)

if buildings.crs != target_crs:
    buildings = buildings.to_crs(target_crs)

buildings_sindex = buildings.sindex
buildings_geom = buildings.geometry.values

# ------------------------------------------
# 4. Generate dense 2-meter nodes along streets
# ------------------------------------------
print("Generating 2-meter nodes along the road network...")
dense_nodes = []
node_counter = 0

for idx, row in edges.iterrows():
    geom = row.geometry
    if geom is None or geom.is_empty:
        continue

    length = geom.length
    distances = np.arange(0, length, 2.0)
    if len(distances) == 0 or distances[-1] < length:
        distances = np.append(distances, length)

    for dist in distances:
        pt = geom.interpolate(dist)
        dense_nodes.append({
            "node_id": f"node_2m_{node_counter}",
            "parent_edge_idx": idx,
            "dist_along_edge": dist,
            "geometry": pt,
            "x": pt.x,
            "y": pt.y,
        })
        node_counter += 1

nodes_2m = gpd.GeoDataFrame(dense_nodes, crs=target_crs)
print(f"Created {len(nodes_2m)} dense 2-meter nodes.")

# ------------------------------------------
# 5. Find lamps near each node
# ------------------------------------------
print("Building KDTree for lamp proximity searches...")
punti_coords = np.column_stack((punti.geometry.x, punti.geometry.y))
nodes_2m_coords = np.column_stack((nodes_2m["x"], nodes_2m["y"]))

tree = cKDTree(punti_coords)
search_radius = 40.0
nearby_lamps_indices = tree.query_ball_point(nodes_2m_coords, search_radius)

# ------------------------------------------
# 6. Compute Lux at each 2m node
# ------------------------------------------
print("Computing line-of-sight Lux for each dense node...")
intensity = punti["downward_intensity_cd"].values
heights = punti["altezza_palo_m"].values
calculated_lux_array = np.zeros(len(nodes_2m))

for i, lamp_idx_list in enumerate(nearby_lamps_indices):
    if not lamp_idx_list:
        continue

    nx, ny = nodes_2m_coords[i]

    for lamp_idx in lamp_idx_list:
        lx, ly = punti_coords[lamp_idx]
        line_of_sight = LineString([(nx, ny), (lx, ly)])

        possible_blockers_idx = list(buildings_sindex.intersection(line_of_sight.bounds))
        is_blocked = False

        for b_idx in possible_blockers_idx:
            if buildings_geom[b_idx].intersects(line_of_sight):
                is_blocked = True
                break

        if not is_blocked:
            d = np.hypot(lx - nx, ly - ny)
            I = intensity[lamp_idx]
            h = heights[lamp_idx]
            calculated_lux_array[i] += (I * h) / ((h**2 + d**2) ** 1.5)

nodes_2m["calculated_lux"] = calculated_lux_array

# ------------------------------------------
# 7. Export dense illuminated nodes
# ------------------------------------------
print("Saving dense illuminated node layer...")
nodes_2m_export = nodes_2m.drop(columns=["x", "y"])
nodes_2m_export.to_file(output_path, driver="GPKG")
print(f"Saved {len(nodes_2m_export)} illuminated nodes to: {output_path}")

