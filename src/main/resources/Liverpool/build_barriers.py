import os
import osmnx as ox
import geopandas as gpd
import pandas as pd
import json

output_dir = os.path.dirname(os.path.abspath(__file__))
boundary_file = os.path.join(output_dir, "CA boundary_disolved.shp")
edges_file = os.path.join(output_dir, "liverpool_edges.gpkg")
barriers_file = os.path.join(output_dir, "liverpool_barriers.gpkg")

print("Loading boundary...")
boundary_gdf = gpd.read_file(boundary_file)
# We need WGS84 to query OSM
if boundary_gdf.crs and boundary_gdf.crs.to_string() != "EPSG:4326":
    boundary_gdf_4326 = boundary_gdf.to_crs(epsg=4326)
else:
    boundary_gdf_4326 = boundary_gdf
    
poly = boundary_gdf_4326.geometry.unary_union

print("Querying OpenStreetMap for Parks, Rivers, and Barriers...")
try:
    parks = ox.features_from_polygon(poly, tags={"leisure": "park"})
    parks = parks[parks.geom_type.isin(['Polygon', 'MultiPolygon'])].copy()
    parks['type'] = 'park'
except Exception as e:
    print("No parks found or error:", e)
    parks = gpd.GeoDataFrame()

try:
    water = ox.features_from_polygon(poly, tags={"natural": "water", "waterway": ["river", "canal"]})
    water = water[water.geom_type.isin(['Polygon', 'MultiPolygon', 'LineString', 'MultiLineString'])].copy()
    water['type'] = 'water'
except Exception as e:
    print("No water found or error:", e)
    water = gpd.GeoDataFrame()

try:
    fences = ox.features_from_polygon(poly, tags={"barrier": True})
    fences = fences[fences.geom_type.isin(['Polygon', 'MultiPolygon', 'LineString', 'MultiLineString'])].copy()
    fences['type'] = 'p_barr' # Positive barrier (walls, fences)
except Exception as e:
    print("No fences found or error:", e)
    fences = gpd.GeoDataFrame()

# Combine them all
all_barriers = []
if not parks.empty: all_barriers.append(parks[['geometry', 'type']])
if not water.empty: all_barriers.append(water[['geometry', 'type']])
if not fences.empty: all_barriers.append(fences[['geometry', 'type']])

if len(all_barriers) > 0:
    combined_barriers = pd.concat(all_barriers, ignore_index=True)
    # Assign unique integer ID
    combined_barriers['barrierID'] = range(1, len(combined_barriers) + 1)
    
    # Save to barriers.gpkg
    print(f"Saving {len(combined_barriers)} barriers to {barriers_file}...")
    combined_barriers.to_file(barriers_file, driver="GPKG")
    
    print("Loading liverpool_edges.gpkg to tag them with barriers...")
    edges_gdf = gpd.read_file(edges_file)
    
    # Ensure CRS matches
    if edges_gdf.crs != combined_barriers.crs:
        combined_barriers = combined_barriers.to_crs(edges_gdf.crs)
        
    # We will buffer the edges slightly (e.g., 20 meters) to find adjacent parks/rivers
    # For buffering in meters, we temporarily project to EPSG:27700
    edges_27700 = edges_gdf.to_crs(epsg=27700)
    barriers_27700 = combined_barriers.to_crs(epsg=27700)
    
    # Buffer edges by 15 meters
    edges_buffered = edges_27700.copy()
    edges_buffered.geometry = edges_buffered.geometry.buffer(15)
    
    # Spatial join to find which barriers intersect the buffered edge
    joined = gpd.sjoin(edges_buffered, barriers_27700, how='left', predicate='intersects')
    
    # Create empty columns
    edges_gdf['w_parks'] = "[]"
    edges_gdf['a_rivers'] = "[]"
    edges_gdf['p_barr'] = "[]"
    
    # Group by edge index
    for edge_idx, group in joined.groupby(joined.index):
        if pd.notna(group['barrierID'].iloc[0]):
            park_ids = group[group['type'] == 'park']['barrierID'].astype(int).tolist()
            water_ids = group[group['type'] == 'water']['barrierID'].astype(int).tolist()
            p_barr_ids = group[group['type'] == 'p_barr']['barrierID'].astype(int).tolist()
            
            if park_ids: edges_gdf.at[edge_idx, 'w_parks'] = json.dumps(park_ids)
            if water_ids: edges_gdf.at[edge_idx, 'a_rivers'] = json.dumps(water_ids)
            if p_barr_ids: edges_gdf.at[edge_idx, 'p_barr'] = json.dumps(p_barr_ids)
            
    print("Saving updated edges back to liverpool_edges.gpkg...")
    # GeoPackage columns can only be basic types, so we saved lists as JSON strings.
    edges_gdf.to_file(edges_file, driver="GPKG")
    print("Done tagging edges!")
else:
    print("No barriers/parks/rivers found.")
