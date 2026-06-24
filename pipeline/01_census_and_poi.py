"""
Step 1 of the PedSimCity data pipeline: build the unified census layer.

Produces ONE ``<City>_censusData.gpkg`` over the FULL census zone set, carrying every per-zone
attribute the simulation reads, with column names matching the Java side
(``pedsim.activity.engine.ActivityEnvironment``):

    residence_pct      share of city residents (home spawning)        -- 0 for non-residential zones
    workplace_poi      daytime workplace POI count (work destinations)
    night_poi          evening / leisure POI count (night destinations)
    vulnerability_pct  share of vulnerable residents [0, 1]           -- read by the night module

This replaces the four ad-hoc scripts (census proportions / vulnerability / workplace POI /
nighttime POI). No zones are dropped and there is no type-1/36 fusion: non-residential zones simply
get residence_pct = 0 and remain valid work/night destinations. POI values are raw COUNTS (numbers
of opportunities), not area-normalised densities, and are kept in the layer's native CRS so they
align with the road-network layers the simulation loads.

Usage:
    python 01_census_and_poi.py --input_dir src/main/resources/TorinoCentre --prefix Torino_
"""

import argparse
import os
from pathlib import Path

import geopandas as gpd
import pandas as pd
import osmnx as ox


WORKPLACE_TAGS = {
    "office": True,
    "shop": True,
    "amenity": ["restaurant", "cafe", "bar", "pub", "fast_food", "bank", "pharmacy", "hospital",
                "clinic", "school", "university", "police", "fire_station", "post_office"],
    "craft": True,
    "industrial": True,
    "landuse": ["commercial", "industrial", "retail"],
}

NIGHT_TAGS = {
    "amenity": ["bar", "pub", "nightclub", "restaurant", "cafe", "fast_food", "cinema", "theatre",
                "pharmacy"],
    "leisure": ["social_centre", "cultural_centre", "casino"],
    "shop": ["convenience"],
}


def count_pois_per_zone(gdf, boundary_4326, tags):
    """Count OSM POIs (by centroid) within each census zone. Returns a per-zone COUNT Series aligned
    to ``gdf.index`` -- the number of opportunities, NOT an area-normalised density."""
    feats = ox.features.features_from_polygon(boundary_4326, tags)
    if feats.empty:
        return pd.Series(0, index=gdf.index)
    feats = feats.to_crs(gdf.crs)
    feats["geometry"] = feats.geometry.centroid
    pts = feats[feats.geometry.type == "Point"].copy()
    joined = gpd.sjoin(pts, gdf[["zone_idx", "geometry"]], how="inner", predicate="within")
    counts = joined.groupby("zone_idx").size()
    return gdf["zone_idx"].map(counts).fillna(0).astype(int)


def main():
    parser = argparse.ArgumentParser(description="Build the unified census layer for PedSimCity.")
    parser.add_argument("--input_dir", required=True, help="City resources folder (e.g. src/main/resources/TorinoCentre)")
    parser.add_argument("--prefix", required=True, help="Prefix of the raw census file (e.g. Torino_)")
    args = parser.parse_args()

    input_dir = Path(os.path.abspath(args.input_dir))
    city = input_dir.name  # output city name = folder name = Pars.cityName on the Java side
    raw_census = input_dir / f"{args.prefix}censusData.gpkg"
    output_path = input_dir / f"{city}_censusData.gpkg"

    if raw_census.resolve() == output_path.resolve():
        raise SystemExit(f"Raw and output paths collide ({output_path}). Use a --prefix different "
                         f"from the city folder name '{city}'.")

    print(f"raw census : {raw_census}")
    print(f"output     : {output_path}")

    # --- Residence share (keep ALL zones, native CRS) ---
    gdf = gpd.read_file(raw_census)
    print("CRS:", gdf.crs)

    pop = pd.to_numeric(gdf["P1"], errors="coerce").fillna(0.0)
    total = pop.sum()
    gdf["residence_pct"] = (pop / total) if total > 0 else 0.0

    # Vulnerability: share of vulnerable residents (females + males 65+) / population, clipped [0, 1].
    vuln = (pd.to_numeric(gdf["P3"], errors="coerce") + pd.to_numeric(gdf["P19"], errors="coerce")) / pop
    gdf["vulnerability_pct"] = vuln.fillna(0.0).clip(0.0, 1.0)

    gdf = gdf.reset_index(drop=True)
    gdf["zone_idx"] = gdf.index
    n_res = int((gdf["residence_pct"] > 0).sum())
    print(f"{len(gdf)} zones | residents={int(total)} | residential zones={n_res}")

    # --- POI counts from OSM ---
    boundary_4326 = gdf.to_crs(epsg=4326).union_all()
    ox.settings.requests_timeout = 600
    gdf["workplace_poi"] = count_pois_per_zone(gdf, boundary_4326, WORKPLACE_TAGS)
    gdf["night_poi"] = count_pois_per_zone(gdf, boundary_4326, NIGHT_TAGS)
    print(f"workplace POIs={int(gdf['workplace_poi'].sum())} | night POIs={int(gdf['night_poi'].sum())}")

    # --- Save the unified census layer ---
    keep = ["censusZoneID", "censusZoneTypeID", "residence_pct", "vulnerability_pct",
            "workplace_poi", "night_poi", "geometry"]
    keep = [c for c in keep if c in gdf.columns]
    out = gdf[keep].copy()

    output_path.parent.mkdir(parents=True, exist_ok=True)
    out.to_file(output_path)
    print(f"saved: {output_path}")
    print(out.drop(columns="geometry").describe())


if __name__ == "__main__":
    main()
