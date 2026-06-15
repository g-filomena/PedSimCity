import os
import geopandas as gpd
import numpy as np
import pandas as pd

# ==========================================
# 1. SETUP PATHS & FILES
# ==========================================
import argparse
import sys
parser = argparse.ArgumentParser()
parser.add_argument('--input_dir', required=True)
parser.add_argument('--prefix', required=True)
args = parser.parse_args()
base_dir = args.input_dir
prefix = args.prefix


edges_filename = f"{prefix}edges.gpkg"
edges_path = os.path.join(base_dir, edges_filename)
nodes_2m_path = os.path.join(base_dir, "nodes_2m_densified_illuminated.gpkg")
output_path = os.path.join(base_dir, "directional_lighting_lookup.csv")

print("Loading datasets...")
edges = gpd.read_file(edges_path)
nodes_2m = gpd.read_file(nodes_2m_path)

# Define the Visibility Horizon (how many meters down the street the agent looks)
visibility_horizon_m = 12.0 

# ==========================================
# 2. MAP TOPOLOGY (Find Edge Endpoints)
# ==========================================
print("Analyzing edge geometry endpoints...")
# Assuming standard OSMnx network structure where edges have 'u' (from node) and 'v' (to node)
# If your columns have different names, change 'u' and 'v' here
has_topology = 'u' in edges.columns and 'v' in edges.columns

directional_records = []

# Group the 2m points by their parent edge for rapid slicing
grouped_points = nodes_2m.groupby('parent_edge_idx')

print("Calculating directional lightness profiles...")
for idx, row in edges.iterrows():
    if idx not in grouped_points.groups:
        continue
        
    edge_pts = grouped_points.get_group(idx)
    edge_length = row.geometry.length
    
    # Identify the node IDs at the start and end of this street
    from_node = row['u'] if has_topology else f"node_start_{idx}"
    to_node = row['v'] if has_topology else f"node_end_{idx}"
    
    # --- CHOICE A: Agent is standing at from_node, looking TOWARD to_node ---
    # Look at the beginning of the edge line geometry
    start_slice = edge_pts[edge_pts['dist_along_edge'] <= visibility_horizon_m]
    lux_looking_forward = start_slice['calculated_lux'].mean() if not start_slice.empty else 0.0
    min_lux_forward = start_slice['calculated_lux'].min() if not start_slice.empty else 0.0
    
    directional_records.append({
        'current_node_id': from_node,
        'target_node_id': to_node,
        'chosen_edge_idx': idx,
        'visibility_mean_lux': lux_looking_forward,
        'visibility_min_lux': min_lux_forward
    })
    
    # --- CHOICE B: Agent is standing at to_node, looking TOWARD from_node ---
    # Look at the tail end of the edge line geometry
    end_slice = edge_pts[edge_pts['dist_along_edge'] >= (edge_length - visibility_horizon_m)]
    lux_looking_backward = end_slice['calculated_lux'].mean() if not end_slice.empty else 0.0
    min_lux_backward = end_slice['calculated_lux'].min() if not end_slice.empty else 0.0
    
    directional_records.append({
        'current_node_id': to_node,
        'target_node_id': from_node,
        'chosen_edge_idx': idx,
        'visibility_mean_lux': lux_looking_backward,
        'visibility_min_lux': min_lux_backward
    })


# exort to CSV

lookup_df = pd.DataFrame(directional_records)

# Handle any NaN entries safely
lookup_df['visibility_mean_lux'] = lookup_df['visibility_mean_lux'].fillna(0.0)
lookup_df['visibility_min_lux'] = lookup_df['visibility_min_lux'].fillna(0.0)


lookup_df.to_csv(output_path, index=False)


