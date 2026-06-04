import os
import geopandas as gpd
import pandas as pd
import numpy as np
import osmnx as ox
from shapely.geometry import box

base_dir = r"C:\Users\sgabalog\Documents\PedSim\Working_Version_Connected_to_Gab\PedSimCity\src\main\resources\Torino"

edges_filename = "Torino_edges.gpkg"  # Your street centerline network file
edges_path = os.path.join(base_dir, edges_filename)
buildings_path = os.path.join(base_dir, "Torino_buildings.gpkg")
output_path = os.path.join(base_dir, "edges_with_frontages.gpkg")

print("Loading local street and building layers...")
edges = gpd.read_file(edges_path)
buildings = gpd.read_file(buildings_path)

# Enforce matching metric CRS (needed for accurate meter-based buffering)
target_crs = edges.crs
if buildings.crs != target_crs:
    print("Aligning building CRS to match edges...")
    buildings = buildings.to_crs(target_crs)

# Ensure the edges have a clean, explicit unique ID column for mapping back later
edges = edges.reset_index(names='edge_unique_id')

### pois from OSM
print("Extracting geographic bounding box to pull active retail POIs...")
# Project bounding box to WGS84 for OpenStreetMap queries
edges_wgs84 = edges.to_crs(epsg=4326)
minx, miny, maxx, maxy = edges_wgs84.total_bounds

# Define tags that signify active visual engagement on the ground floor
osm_tags = {
    'shop': True, 
    'amenity': ['restaurant', 'cafe', 'bar', 'pub', 'fast_food', 'bank', 'pharmacy', 'marketplace'],
    'tourism': ['museum', 'gallery']
}

print("Downloading active frontages/POIs from OpenStreetMap via OSMnx...")
try:
    # Fetch active POI points within the network boundary box
    pois = ox.features_from_bbox(bbox=(maxy, miny, maxx, minx), tags=osm_tags)
    pois = pois.to_crs(target_crs)
    print(f"Successfully retrieved {len(pois)} active POIs.")
except Exception as e:
    print(f"OSMnx query timed out or failed: {e}. Falling back to an empty POI dataframe.")
    pois = gpd.GeoDataFrame(geometry=[], crs=target_crs)


# take just STREET-FACING facades

print("Extracting building exterior walls (facades)...")
# Convert solid building polygons into thin outline boundaries (LineStrings)
buildings_lines = buildings[['geometry']].copy()
buildings_lines['geometry'] = buildings_lines.geometry.boundary

print("Isolating facades sitting close to street segments...")
# Buffer the street centerlines by 12 meters to capture the adjacent sidewalk building walls
edges_buffered = edges[['edge_unique_id', 'geometry']].copy()
edges_buffered['geometry'] = edges_buffered.geometry.buffer(12.0)

# Intersect building walls with the street buffer zones 
# This breaks building lines down and assigns each piece to its specific street ID
facades_per_edge = gpd.overlay(buildings_lines, edges_buffered, how='intersection')
facades_per_edge['total_wall_len'] = facades_per_edge.geometry.length


# MEASURE ACTIVE VS INACTIVE FRONTAGES

if not pois.empty:
    print("Mapping active POI footprints onto street-facing walls...")
    # Buffer POI points by 8 meters to account for depth inside building envelopes
    pois_buffered = gpd.GeoDataFrame(geometry=pois.geometry.buffer(8.0), crs=target_crs)
    
    # Clip the street-facing building walls strictly inside active POI footprints
    active_facades = gpd.overlay(facades_per_edge, pois_buffered, how='intersection')
    active_facades['active_wall_len'] = active_facades.geometry.length
    
    # Sum up the continuous active frontage length per unique street edge
    active_totals = active_facades.groupby('edge_unique_id')['active_wall_len'].sum().reset_index()
else:
    active_totals = pd.DataFrame(columns=['edge_unique_id', 'active_wall_len'])

# Sum up the global total facade wall length per unique street edge
total_facade_lengths = facades_per_edge.groupby('edge_unique_id')['total_wall_len'].sum().reset_index()


# AGGREGATE METRICS TO ROAD NETWORK

print("Compiling final indicators and frontage ratios onto network lines...")
# Merge metrics back onto the primary street layer
edges = edges.merge(total_facade_lengths, on='edge_unique_id', how='left')
edges = edges.merge(active_totals, on='edge_unique_id', how='left')

# Clean missing values safely (if a street has no buildings or active POIs)
edges['total_wall_len'] = edges['total_wall_len'].fillna(0.0)
edges['active_wall_len'] = edges['active_wall_len'].fillna(0.0)
edges['inactive_wall_len'] = edges['total_wall_len'] - edges['active_wall_len']

# Calculate Frontage Activity Ratio (Active Frontage / Total Built Frontage)
edges['frontage_activity_ratio'] = np.where(
    edges['total_wall_len'] > 0,
    edges['active_wall_len'] / edges['total_wall_len'],
    0.0
)

# Calculate Street Activity Index (Active Frontage / Total Potential Street Length Both Sides)
edges['street_activity_index'] = edges['active_wall_len'] / (edges.geometry.length * 2.0)
# Cap index at 1.0 in case of complex overlapping geometry clusters
edges['street_activity_index'] = edges['street_activity_index'].clip(upper=1.0)

# Clean up temporary structural helper column before export
edges = edges.drop(columns=['edge_unique_id'])

# ==========================================
# 6. EXPORT ENRICHED ROAD NETWORK
# ==========================================
print("Writing out final spatial dataset...")
edges.to_file(output_path, driver="GPKG")
print(f"Success! Network with frontage metrics saved to:\n{output_path}")