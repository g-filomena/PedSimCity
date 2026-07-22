import osmnx as ox
import geopandas as gpd
import os
import pandas as pd

output_dir = os.path.dirname(os.path.abspath(__file__))

# Load the user's boundary shapefile
boundary_file = os.path.join(output_dir, "CA boundary_disolved.shp")
print(f"Loading boundary from: {boundary_file}")
boundary_gdf = gpd.read_file(boundary_file)

# Make sure it's in WGS84 (EPSG:4326) which OSMnx requires
if boundary_gdf.crs and boundary_gdf.crs.to_string() != "EPSG:4326":
    boundary_gdf = boundary_gdf.to_crs(epsg=4326)

# Extract the polygon geometry
poly = boundary_gdf.geometry.unary_union

print(f"Downloading OSM data for the provided boundary polygon...")

# 1. Download Spatial Network Data (Walkable Graph)
print("\n1. Downloading walkable street network...")
try:
    G = ox.graph_from_polygon(poly, network_type="walk", retain_all=True)
    nodes, edges = ox.graph_to_gdfs(G)
    
    edges_path = os.path.join(output_dir, "liverpool_edges.gpkg")
    nodes_path = os.path.join(output_dir, "liverpool_nodes.gpkg")
    
    edges.to_file(edges_path, driver="GPKG")
    nodes.to_file(nodes_path, driver="GPKG")
    print(f"-> Saved network edges to: {edges_path}")
    print(f"-> Saved network nodes to: {nodes_path}")
except Exception as e:
    print(f"Error downloading or saving network data: {e}")

# 2. Download Urban Morphology Data (Buildings & Barriers)
print("\n2a. Downloading building footprints...")
try:
    buildings = ox.features_from_polygon(poly, tags={"building": True})
    buildings = buildings[buildings.geom_type.isin(['Polygon', 'MultiPolygon'])]
    
    buildings_path = os.path.join(output_dir, "liverpool_buildings.gpkg")
    
    # Convert list/dict columns to strings
    for col in buildings.columns:
        if buildings[col].apply(type).eq(list).any() or buildings[col].apply(type).eq(dict).any():
            buildings[col] = buildings[col].astype(str)
            
    buildings.to_file(buildings_path, driver="GPKG")
    print(f"-> Saved {len(buildings)} building footprints to: {buildings_path}")
except Exception as e:
    print(f"Error downloading or saving building data: {e}")

print("\n2b. Downloading physical barriers...")
try:
    barriers = ox.features_from_polygon(poly, tags={"barrier": True})
    barriers = barriers[barriers.geom_type.isin(['LineString', 'MultiLineString', 'Polygon', 'MultiPolygon'])]
    
    barriers_path = os.path.join(output_dir, "liverpool_barriers.gpkg")
    
    for col in barriers.columns:
        if barriers[col].apply(type).eq(list).any() or barriers[col].apply(type).eq(dict).any():
            barriers[col] = barriers[col].astype(str)
            
    barriers.to_file(barriers_path, driver="GPKG")
    print(f"-> Saved {len(barriers)} physical barriers to: {barriers_path}")
except Exception as e:
    print(f"Error downloading or saving barrier data: {e}")

print("\nFinished successfully! All requested GeoPackages are in the Liverpool folder.")
