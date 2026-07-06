"""Step 1: build the unified census layer for PedSimCity.

Reads the raw ISTAT census <city>_censusData_raw.gpkg and writes the enriched
<city>_censusData.gpkg (the file the Java side loads) with these columns:
- residence_pct
- workplace_poi
- night_poi
- vulnerability_pct  (share of residents who are female, under 15, or 65+; counted once each)

The raw file is left untouched, so this step is safe to re-run.

`--input_dir` is the resources folder path.
`--city_name` is the filename prefix before `_censusData.gpkg`, `_edges.gpkg`, etc.
If `--city_name` is omitted, the input folder name is used for backward compatibility.
"""

from __future__ import annotations

import argparse
import os
from pathlib import Path

import geopandas as gpd
import osmnx as ox
import pandas as pd


WORKPLACE_TAGS = {
    "office": True,
    "shop": True,
    "amenity": [
        "restaurant",
        "cafe",
        "bar",
        "pub",
        "fast_food",
        "bank",
        "pharmacy",
        "hospital",
        "clinic",
        "school",
        "university",
        "police",
        "fire_station",
        "post_office",
    ],
    "craft": True,
    "industrial": True,
    "landuse": ["commercial", "industrial", "retail"],
}

NIGHT_TAGS = {
    "amenity": [
        "bar",
        "pub",
        "nightclub",
        "restaurant",
        "cafe",
        "fast_food",
        "cinema",
        "theatre",
        "pharmacy",
    ],
    "leisure": ["social_centre", "cultural_centre", "casino"],
    "shop": ["convenience"],
}


def geometry_union(gdf: gpd.GeoDataFrame):
    if hasattr(gdf.geometry, "union_all"):
        return gdf.geometry.union_all()
    return gdf.geometry.unary_union


def osm_features_from_polygon(boundary_4326, tags):
    """OSMnx 2.x-compatible feature downloader."""
    if hasattr(ox, "features_from_polygon"):
        return ox.features_from_polygon(boundary_4326, tags)
    if hasattr(ox, "features") and hasattr(ox.features, "features_from_polygon"):
        return ox.features.features_from_polygon(boundary_4326, tags)
    raise AttributeError("This OSMnx version has no features_from_polygon API.")


def count_pois_per_zone(gdf: gpd.GeoDataFrame, boundary_4326, tags: dict) -> pd.Series:
    """Count OSM POIs by centroid within each census zone.

    Returns counts aligned to gdf.index. Counts are opportunities, not densities.
    """
    feats = osm_features_from_polygon(boundary_4326, tags)
    if feats.empty:
        return pd.Series(0, index=gdf.index, dtype="int64")

    feats = feats.to_crs(gdf.crs)
    feats = feats[feats.geometry.notnull() & ~feats.geometry.is_empty].copy()
    feats["geometry"] = feats.geometry.centroid
    pts = feats[feats.geometry.type == "Point"].copy()
    if pts.empty:
        return pd.Series(0, index=gdf.index, dtype="int64")

    joined = gpd.sjoin(pts, gdf[["zone_idx", "geometry"]], how="inner", predicate="within")
    counts = joined.groupby("zone_idx").size()
    return gdf["zone_idx"].map(counts).fillna(0).astype(int)


def main() -> None:
    parser = argparse.ArgumentParser(description="Build the unified census layer for PedSimCity.")
    parser.add_argument("--input_dir", required=True, help="City resources folder, e.g. src/main/resources/Torino")
    parser.add_argument(
        "--city_name",
        default=None,
        help="Filename prefix before _censusData.gpkg / _edges.gpkg. Defaults to input folder name.",
    )
    args = parser.parse_args()

    input_dir = Path(os.path.abspath(args.input_dir))
    city = args.city_name or input_dir.name

    # Raw ISTAT census in, enriched census out. Keeping them as distinct files leaves the raw
    # source (with all the P* columns) intact, so this step stays re-runnable and non-destructive.
    raw_census = input_dir / f"{city}_censusData_raw.gpkg"
    output_path = input_dir / f"{city}_censusData.gpkg"

    if not raw_census.exists():
        raise FileNotFoundError(f"Raw census file not found: {raw_census}")

    print(f"resources folder: {input_dir}")
    print(f"city file prefix: {city}")
    print(f"raw census: {raw_census}")
    print(f"output    : {output_path}")

    gdf = gpd.read_file(raw_census)
    if gdf.empty:
        raise ValueError(f"Raw census layer is empty: {raw_census}")

    print("CRS:", gdf.crs)
    if "P1" not in gdf.columns:
        raise KeyError("Required population column P1 not found in census data.")

    pop = pd.to_numeric(gdf["P1"], errors="coerce").fillna(0.0)
    total = pop.sum()
    gdf["residence_pct"] = (pop / total) if total > 0 else 0.0

    # Vulnerability: share of residents who are elderly, under 15, or female, counted once each.
    # The vulnerable set is the union: all females  U  males under 15  U  males 65+.
    # Taking every female once, then only the *male* children and *male* elderly, avoids
    # double-counting the female children / female elderly already inside "all females".
    # Column meanings (Torino_census_metaData.xlsx, sheet metaDataCensus):
    #   P3           females (all ages)
    #   P30,P31,P32  males aged <5 / 5-9 / 10-14   -> males under 15
    #   P43,P44,P45  males aged 65-69 / 70-74 / >74 -> males 65+
    # NOTE: the ISTAT census-section dataset has no disability variable, so disability cannot be
    # included here; add its column(s) to VULNERABLE_COLS if a future dataset provides them.
    VULNERABLE_COLS = ["P3", "P30", "P31", "P32", "P43", "P44", "P45"]

    missing_vulnerable_cols = sorted(set(VULNERABLE_COLS) - set(gdf.columns))
    if missing_vulnerable_cols:
        raise ValueError(
            "Missing required vulnerability census columns in "
            f"{raw_census}: {missing_vulnerable_cols}. "
            "Expected columns are: P3 all females; P30/P31/P32 male under-15; "
            "P43/P44/P45 male 65+."
        )

    vulnerable = sum(
        (
            pd.to_numeric(gdf[c], errors="coerce").fillna(0.0)
            for c in VULNERABLE_COLS
        ),
        start=pd.Series(0.0, index=gdf.index),
    )
    gdf["vulnerability_pct"] = (vulnerable / pop.replace(0, pd.NA)).fillna(0.0).clip(0.0, 1.0)

    gdf = gdf.reset_index(drop=True)
    gdf["zone_idx"] = gdf.index

    n_res = int((gdf["residence_pct"] > 0).sum())
    print(f"{len(gdf)} zones | residents={int(total)} | residential zones={n_res}")

    boundary_4326 = geometry_union(gdf.to_crs(epsg=4326))
    ox.settings.requests_timeout = 600

    gdf["workplace_poi"] = count_pois_per_zone(gdf, boundary_4326, WORKPLACE_TAGS)
    gdf["night_poi"] = count_pois_per_zone(gdf, boundary_4326, NIGHT_TAGS)

    print(f"workplace POIs={int(gdf['workplace_poi'].sum())} | night POIs={int(gdf['night_poi'].sum())}")

    keep = [
        "censusZoneID",
        "censusZoneTypeID",
        "residence_pct",
        "vulnerability_pct",
        "workplace_poi",
        "night_poi",
        "geometry",
    ]
    keep = [c for c in keep if c in gdf.columns]
    out = gdf[keep].copy()

    if output_path.exists():
        output_path.unlink()
    out.to_file(output_path, driver="GPKG")

    print(f"saved: {output_path}")
    print(out.drop(columns="geometry").describe())


if __name__ == "__main__":
    main()
