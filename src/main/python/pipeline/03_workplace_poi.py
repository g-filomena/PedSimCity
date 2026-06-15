import osmnx as ox
import geopandas as gpd
import pandas as pd
from pathlib import Path
import os

# --- Portable Path Setup ---
# This logic finds the project root by looking for the 'src' directory.
# It ensures the code works for any user who clones the repository.
current_path = Path(os.getcwd())
try:
    project_root = next(p for p in current_path.parents if (p / 'src').exists() or p.name == 'PedSimCity')
except StopIteration:
    project_root = current_path

census_path = project_root / "src" / "main" / "resources" / "TorinoCentre" / "Torino_censusData.gpkg"
output_path = project_root / "src" / "main" / "resources" / "TorinoCentre" / "TorinoCentre_Workplace_POI_densities.gpkg"

print(f"Project Root identified as: {project_root}")
print(f"Looking for census data at: {census_path}")

# Load Census Zones
gdf_bounds = gpd.read_file(census_path)

# Ensure we are in a metric CRS (meters) for accurate density calculations.
# EPSG:32632 is the standard UTM projection for Torino (Italy).
if gdf_bounds.crs.is_geographic:
    gdf_bounds = gdf_bounds.to_crs(epsg=32632)

# Create a unioned boundary in 4326 (lat/long) for OSMnx extraction
gdf_4326 = gdf_bounds.to_crs(epsg=4326)
boundary_poly = gdf_4326.union_all()

print("Census boundaries loaded and projected.")

## Define Workplace POIs of interest 
tags = {
    # Every office (lawyers, tech, agencies)
    "office": True, 
    
    # Every single shop (from tiny boutiques to big supermarkets)
    "shop": True, 
    
    # Every place where individuals can work are provided
    "amenity": [
        "restaurant", "cafe", "bar", "pub", "fast_food", 
        "bank", "pharmacy", "hospital", "clinic", "school", 
        "university", "police", "fire_station", "post_office"
    ],
    
    # Production and manufacturing
    "craft": True,
    "industrial": True,
    
    # Business-heavy land use
    "landuse": ["commercial", "industrial", "retail"]
}

# Increase timeout to 10 minutes (600 seconds)
ox.settings.requests_timeout = 600
def calculate_workplace_density(poly, tags, gdf_bounds, id_col, output_path):
    """
    Calculates workplace POI density per census zone and saves only 
    the ID, geometry, and density value.
    """
    print("Starting Workplace Density extraction and calculation...")
    
    # 1. Fetch features from OSM
    features = ox.features.features_from_polygon(poly, tags)
    
    if features.empty:
        print("No features found for the provided workplace tags. Check boundary or tags.")
        return None

    # 2. Match CRS to census bounds and calculate Centroids
    features = features.to_crs(gdf_bounds.crs)
    features['geometry'] = features.geometry.centroid
    points = features[features.geometry.type == 'Point'].copy()
    
    # 3. Spatial Join: Match each point to its specific Census Zone
    points_in_zones = gpd.sjoin(
        points, 
        gdf_bounds[[id_col, 'geometry']], 
        how='inner', 
        predicate='within'
    )
    
    # 4. Aggregate: Count points per Zone ID
    counts = points_in_zones.groupby(id_col).size().reset_index(name='workplace_count')
    
    # 5. Merge counts back to the census geometries
    gdf_final = gdf_bounds.copy().merge(counts, on=id_col, how='left')
    
    # 6. Calculate Density (Workplaces per 10,000 square meters / 1 Hectare)
    gdf_final['workplace_count'] = gdf_final['workplace_count'].fillna(0)
    gdf_final['work_density'] = (gdf_final['workplace_count'] / gdf_final.geometry.area) * 10000
    
    # 7. Cleanup: Keep only the ID, Geometry, and the final density column
    gdf_export = gdf_final[[id_col, 'geometry', 'work_density']]
    
    # 8. Save to File
    # Ensure output directory exists
    output_path.parent.mkdir(parents=True, exist_ok=True)
    gdf_export.to_file(output_path)
    
    print(f"Success! Workplace density saved to: {output_path}")
    return gdf_export

# --- Run Execution ---
gdf_work_result = calculate_workplace_density(
    poly=boundary_poly, 
    tags=tags, 
    gdf_bounds=gdf_bounds, 
    id_col='SEZ21_ID', 
    output_path=output_path
)

