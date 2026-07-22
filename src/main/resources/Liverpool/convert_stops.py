import os
import glob
import pandas as pd
import geopandas as gpd
from shapely.geometry import Point

# Paths
output_dir = os.path.dirname(os.path.abspath(__file__))
boundary_file = os.path.join(output_dir, "CA boundary_disolved.shp")
output_gpkg = os.path.join(output_dir, "liverpool_transit.gpkg")

# Find all NaPTAN CSV files in the folder (in case user downloads Halton)
csv_files = glob.glob(os.path.join(output_dir, "*Stops.csv"))

if not csv_files:
    print("No *Stops.csv files found!")
    exit()

print(f"Loading stops from {len(csv_files)} files: {[os.path.basename(f) for f in csv_files]}")

dfs = []
for f in csv_files:
    dfs.append(pd.read_csv(f, low_memory=False))

df = pd.concat(dfs, ignore_index=True)

# Drop rows missing Lat/Lon
df = df.dropna(subset=['Longitude', 'Latitude'])

# Filter for only 'active' stops
if 'Status' in df.columns:
    initial_count = len(df)
    df = df[df['Status'].astype(str).str.lower() == 'active']
    print(f"Dropped {initial_count - len(df)} inactive stops.")

# Convert to GeoDataFrame
geometry = [Point(xy) for xy in zip(df['Longitude'], df['Latitude'])]
gdf_stops = gpd.GeoDataFrame(df, geometry=geometry, crs="EPSG:4326")
print(f"Total active stops in CSV(s): {len(gdf_stops)}")

print(f"Loading boundary from: {boundary_file}")
boundary_gdf = gpd.read_file(boundary_file)

# Make sure boundary is WGS84
if boundary_gdf.crs and boundary_gdf.crs.to_string() != "EPSG:4326":
    boundary_gdf = boundary_gdf.to_crs(epsg=4326)

poly = boundary_gdf.geometry.unary_union

print("Filtering stops to within the boundary...")
# Keep only stops that intersect with the boundary polygon
gdf_clipped = gdf_stops[gdf_stops.intersects(poly)]
print(f"Stops remaining after clipping: {len(gdf_clipped)}")

# Convert all non-geometry columns to strings for GeoPackage compatibility
for col in gdf_clipped.columns:
    if col != 'geometry':
        gdf_clipped[col] = gdf_clipped[col].astype(str)

print(f"Saving to {output_gpkg}...")
gdf_clipped.to_file(output_gpkg, driver="GPKG")
print("Done!")
