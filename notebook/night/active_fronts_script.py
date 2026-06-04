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


def download_osm_pois(tags, bbox, target_crs, tile_size_deg=0.015):
    """Query OSM in smaller tiles to avoid oversized Overpass requests."""
    minx, miny, maxx, maxy = bbox
    chunk_frames = []

    for x0 in np.arange(minx, maxx, tile_size_deg):
        x1 = min(x0 + tile_size_deg, maxx)
        if x1 <= x0:
            continue

        for y0 in np.arange(miny, maxy, tile_size_deg):
            y1 = min(y0 + tile_size_deg, maxy)
            if y1 <= y0:
                continue

            try:
                chunk = ox.features_from_bbox(bbox=(y1, y0, x1, x0), tags=tags)
                if not chunk.empty:
                    chunk_frames.append(chunk)
            except Exception as exc:
                print(f"  Chunk ({x0:.5f},{y0:.5f}) -> {exc}")

    if not chunk_frames:
        return gpd.GeoDataFrame(geometry=[], crs=target_crs)

    combined = pd.concat(chunk_frames, ignore_index=True)
    combined = gpd.GeoDataFrame(combined, geometry='geometry', crs='EPSG:4326')
    return combined.to_crs(target_crs)

# 1. Exhaustive OSM Tags for ACTIVE frontages
active_tags = {
    'shop': True,
    'amenity': ['restaurant', 'cafe', 'bar', 'pub', 'fast_food', 'ice_cream', 'food_court', 'marketplace', 
                'bank', 'pharmacy', 'post_office', 'library', 'community_centre', 'social_facility', 
                'clinic', 'dentist', 'doctors', 'theatre', 'cinema', 'arts_centre', 'nightclub'],
    'tourism': ['museum', 'gallery', 'information', 'aquarium', 'theme_park', 'zoo'],
    'leisure': ['fitness_centre', 'sports_centre', 'amusement_arcade', 'bowling_alley'],
    'office': ['estate_agent', 'travel_agent', 'insurance', 'government', 'employment_agency'],
    'craft': True
}

# 2. OSM Tags for INACTIVE / HOSTILE frontages
inactive_tags = {
    'building': ['industrial', 'warehouse', 'garage', 'garages', 'manufacture'],
    'amenity': ['parking', 'parking_entrance'],
    'barrier': ['wall', 'fence', 'noise_barrier']
}

print("Downloading active frontages/POIs from OpenStreetMap via OSMnx (tile-based)...")
try:
    active_pois = download_osm_pois(active_tags, edges_wgs84.total_bounds, target_crs)
    # Convert polygons to centroids to prevent whole building perimeters from becoming active
    active_pois['geometry'] = active_pois.geometry.centroid
    print(f"Successfully retrieved {len(active_pois)} active POIs.")
except Exception as e:
    print(f"OSMnx query timed out or failed for active POIs: {e}. Falling back to an empty POI dataframe.")
    active_pois = gpd.GeoDataFrame(geometry=[], crs=target_crs)

print("Downloading inactive/hostile POIs from OpenStreetMap via OSMnx (tile-based)...")
try:
    inactive_pois = download_osm_pois(inactive_tags, edges_wgs84.total_bounds, target_crs)
    # Convert polygons to centroids
    inactive_pois['geometry'] = inactive_pois.geometry.centroid
    print(f"Successfully retrieved {len(inactive_pois)} inactive POIs.")
except Exception as e:
    print(f"OSMnx query timed out or failed for inactive POIs: {e}. Falling back to an empty POI dataframe.")
    inactive_pois = gpd.GeoDataFrame(geometry=[], crs=target_crs)

# take just STREET-FACING facades

print("Extracting building exterior walls (facades)...")
# Convert solid building polygons into thin outline boundaries (LineStrings)
buildings_lines = buildings[['geometry']].copy()
buildings_lines['geometry'] = buildings_lines.geometry.boundary

print("Isolating facades sitting close to street segments...")
edges_buffered = edges[['edge_unique_id', 'geometry']].copy()

# Adaptive buffering if 'highway' column exists, otherwise fallback to 15m
if 'highway' in edges.columns:
    def get_buffer(hw):
        if pd.isna(hw): return 15.0
        hw = str(hw).lower()
        if 'primary' in hw or 'trunk' in hw: return 20.0
        elif 'secondary' in hw: return 15.0
        elif 'tertiary' in hw: return 12.0
        elif 'residential' in hw or 'living_street' in hw: return 10.0
        return 15.0
    edges_buffered['geometry'] = edges.apply(lambda row: row.geometry.buffer(get_buffer(row['highway'])), axis=1)
else:
    edges_buffered['geometry'] = edges_buffered.geometry.buffer(15.0)

# Intersect building walls with the street buffer zones 
# This breaks building lines down and assigns each piece to its specific street ID
facades_per_edge = gpd.overlay(buildings_lines, edges_buffered, how='intersection')
facades_per_edge['total_wall_len'] = facades_per_edge.geometry.length

# MEASURE ACTIVE VS INACTIVE FRONTAGES

def measure_frontages(poi_gdf, name_prefix):
    if not poi_gdf.empty:
        # Buffer POI points by 10 meters to capture the nearest facade wall
        pois_buffered = gpd.GeoDataFrame(geometry=poi_gdf.geometry.buffer(10.0), crs=target_crs)
        intersected = gpd.overlay(facades_per_edge, pois_buffered, how='intersection')
        intersected[f'{name_prefix}_wall_len'] = intersected.geometry.length
        return intersected.groupby('edge_unique_id')[f'{name_prefix}_wall_len'].sum().reset_index()
    else:
        return pd.DataFrame(columns=['edge_unique_id', f'{name_prefix}_wall_len'])

print("Mapping POI footprints onto street-facing walls...")
active_totals = measure_frontages(active_pois, 'active')
inactive_totals = measure_frontages(inactive_pois, 'hostile')

# Sum up the global total facade wall length per unique street edge
total_facade_lengths = facades_per_edge.groupby('edge_unique_id')['total_wall_len'].sum().reset_index()

# AGGREGATE METRICS TO ROAD NETWORK

print("Compiling final indicators and frontage ratios onto network lines...")
# Merge metrics back onto the primary street layer
edges = edges.merge(total_facade_lengths, on='edge_unique_id', how='left')
edges = edges.merge(active_totals, on='edge_unique_id', how='left')
edges = edges.merge(inactive_totals, on='edge_unique_id', how='left')

# Clean missing values safely
edges['total_wall_len'] = edges['total_wall_len'].fillna(0.0)
edges['active_wall_len'] = edges['active_wall_len'].fillna(0.0)
edges['hostile_wall_len'] = edges['hostile_wall_len'].fillna(0.0)

# Inactive frontages are whatever is left over (e.g. residential windows)
edges['inactive_wall_len'] = edges['total_wall_len'] - edges['active_wall_len'] - edges['hostile_wall_len']
edges['inactive_wall_len'] = edges['inactive_wall_len'].clip(lower=0.0)

# Calculate Frontage Activity Ratio (Active Frontage / Total Built Frontage)
edges['frontage_activity_ratio'] = np.where(
    edges['total_wall_len'] > 0,
    edges['active_wall_len'] / edges['total_wall_len'],
    0.0
)

# Calculate Blank Wall / Hostile Ratio
edges['blank_wall_ratio'] = np.where(
    edges['total_wall_len'] > 0,
    edges['hostile_wall_len'] / edges['total_wall_len'],
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