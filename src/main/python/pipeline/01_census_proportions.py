import pandas as pd
import geopandas as gpd
from pathlib import Path
import os

# --- Portable Path Setup ---
# 1. Get the directory where the notebook is currently running
current_dir = Path(os.getcwd())

# 2. Find the project root (traverses up until it finds the folder containing 'src')
try:
    project_root = next(p for p in current_dir.parents if (p / 'src').exists() or p.name == 'PedSimCity')
except StopIteration:
    # Fallback to current directory if 'src' isn't found in parents
    project_root = current_dir

# 3. Define the relative path to your census file
file_path = project_root / "src" / "main" / "resources" / "TorinoCentre" / "Torino_censusData.gpkg"

print(f"Attempting to read file at: {file_path}")

try:
    # Read the GeoPackage file
    # Pathlib objects work directly with gpd.read_file
    gdf = gpd.read_file(file_path)

    # Inspect the columns 
    print("\nColumns available:")
    print(gdf.columns.tolist())

    # CRS Check
    print(f"\nCoordinate Reference System: {gdf.crs}")

    # Preview the data
    print("\nFirst 5 rows of data:")
    print(gdf.head())

except FileNotFoundError:
    print(f"Error: The file was not found at {file_path}. Check your folder structure.")
except Exception as e:
    print(f"Error reading the file: {e}")

# Depending on census data column names will change
# 'P1' here is the population per zone
# Calculate the % of total city residents that live in each zone
total_city_population = gdf['P1'].sum()
gdf['zone_residence_pct'] = gdf['P1'] / total_city_population


# Preview the results
print(gdf[['SEZ21_ID', 'P1', 'zone_residence_pct']].head())

# Remove all rows where zone_residence_pct is 0 as agents cannot spawn there
gdf = gdf[gdf['zone_residence_pct'] > 0].copy()

current_dir = Path(os.getcwd())
try:
    project_root = next(p for p in current_dir.parents if (p / 'src').exists() or p.name == 'PedSimCity')
except StopIteration:
    project_root = current_dir

# 1. Define the columns you want to keep
columns_to_save = ['SEZ21_ID', 'zone_residence_pct', 'geometry']

# 2. Create a new GeoDataFrame containing only those columns
zone_percentages = gdf[columns_to_save].copy()

# 3. Define the output path dynamically
output_path = project_root / "src" / "main" / "resources" / "TorinoCentre" / "TorinoCentre_censusData.gpkg"

try:
    # 4. Save to GPKG
    # Ensure the directory exists
    output_path.parent.mkdir(parents=True, exist_ok=True)
    
    # Save the specific layer
    zone_percentages.to_file(output_path)
    print(f"Successfully: {output_path}")

except Exception as e:
    print(f"An error occurred while saving: {e}")

