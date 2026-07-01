"""Optional prep step: street-frontage activity.

Enriches the edge network with active / hostile / inactive frontage lengths and ratios.
This is optional and not part of the census or lighting pipelines; run it directly.

All layers are named <City>_… (City = the resources folder name).

Run manually:
    python pipeline/active_frontages.py --input_dir src/main/resources/Torino
"""

from __future__ import annotations

import argparse
import os
from pathlib import Path

import geopandas as gpd
import numpy as np
import osmnx as ox
import pandas as pd

ACTIVE_TAGS = {
    "shop": True,
    "amenity": [
        "restaurant",
        "cafe",
        "bar",
        "pub",
        "fast_food",
        "ice_cream",
        "food_court",
        "marketplace",
        "bank",
        "pharmacy",
        "post_office",
        "library",
        "community_centre",
        "social_facility",
        "clinic",
        "dentist",
        "doctors",
        "theatre",
        "cinema",
        "arts_centre",
        "nightclub",
    ],
    "tourism": ["museum", "gallery", "information", "aquarium", "theme_park", "zoo"],
    "leisure": ["fitness_centre", "sports_centre", "amusement_arcade", "bowling_alley"],
    "office": ["estate_agent", "travel_agent", "insurance", "government", "employment_agency"],
    "craft": True,
}

HOSTILE_TAGS = {
    "building": ["industrial", "warehouse", "garage", "garages", "manufacture"],
    "amenity": ["parking", "parking_entrance"],
    "barrier": ["wall", "fence", "noise_barrier"],
}

POI_BUFFER_M = 10.0


def first_existing(paths: list[Path], label: str) -> Path:
    for path in paths:
        if path.exists():
            return path
    joined = "\n  ".join(str(p) for p in paths)
    raise FileNotFoundError(f"Could not find {label}. Tried:\n  {joined}")


def features_from_bbox_compat(west: float, south: float, east: float, north: float, tags: dict):
    # OSMnx 2.x accepts bbox=(left, bottom, right, top). Older versions may differ.
    if hasattr(ox, "features") and hasattr(ox.features, "features_from_bbox"):
        return ox.features.features_from_bbox((west, south, east, north), tags)
    return ox.features_from_bbox(north, south, east, west, tags)


def download_osm_pois(tags, bbox_wgs84, target_crs, tile_size_deg: float = 0.015) -> gpd.GeoDataFrame:
    minx, miny, maxx, maxy = bbox_wgs84
    frames = []

    for x0 in np.arange(minx, maxx, tile_size_deg):
        x1 = min(x0 + tile_size_deg, maxx)
        if x1 <= x0:
            continue
        for y0 in np.arange(miny, maxy, tile_size_deg):
            y1 = min(y0 + tile_size_deg, maxy)
            if y1 <= y0:
                continue
            try:
                chunk = features_from_bbox_compat(x0, y0, x1, y1, tags)
                if not chunk.empty:
                    frames.append(chunk)
            except Exception as exc:
                print(f" chunk ({x0:.5f},{y0:.5f}) -> {exc}")

    if not frames:
        return gpd.GeoDataFrame(geometry=[], crs=target_crs)

    combined = gpd.GeoDataFrame(pd.concat(frames, ignore_index=True), geometry="geometry", crs="EPSG:4326")
    return combined.to_crs(target_crs)


def edge_buffer(highway) -> float:
    if pd.isna(highway):
        return 15.0
    hw = str(highway).lower()
    if "primary" in hw or "trunk" in hw:
        return 20.0
    if "secondary" in hw:
        return 15.0
    if "tertiary" in hw:
        return 12.0
    if "residential" in hw or "living_street" in hw:
        return 10.0
    return 15.0


def measure_frontages(facades_per_edge: gpd.GeoDataFrame, poi_gdf: gpd.GeoDataFrame, name: str, target_crs) -> pd.DataFrame:
    col = f"{name}_wall_len"

    if poi_gdf.empty or facades_per_edge.empty:
        return pd.DataFrame(columns=["edge_unique_id", col])

    buffered = gpd.GeoDataFrame(geometry=poi_gdf.geometry.buffer(POI_BUFFER_M), crs=target_crs)
    hit = gpd.overlay(facades_per_edge, buffered, how="intersection")
    if hit.empty:
        return pd.DataFrame(columns=["edge_unique_id", col])

    hit[col] = hit.geometry.length
    return hit.groupby("edge_unique_id")[col].sum().reset_index()


def main() -> None:
    parser = argparse.ArgumentParser(description="Calculate active/hostile street frontages.")
    parser.add_argument("--input_dir", required=True)
    args = parser.parse_args()

    base_dir = Path(os.path.abspath(args.input_dir))
    city = base_dir.name

    edges_path = first_existing(
        [base_dir / f"{city}_edges.gpkg"],
        "edges layer",
    )
    buildings_path = first_existing(
        [base_dir / f"{city}_buildings.gpkg"],
        "buildings layer",
    )

    edges = gpd.read_file(edges_path)
    buildings = gpd.read_file(buildings_path)

    if edges.empty:
        raise ValueError(f"Edges layer is empty: {edges_path}")
    if buildings.empty:
        raise ValueError(f"Buildings layer is empty: {buildings_path}")

    target_crs = edges.crs or buildings.crs
    if target_crs is None:
        raise ValueError("No CRS found in edges/buildings layers.")

    if buildings.crs is None:
        buildings = buildings.set_crs(target_crs)
    elif buildings.crs != target_crs:
        buildings = buildings.to_crs(target_crs)

    edges = edges.reset_index(names="edge_unique_id")
    bbox_wgs84 = edges.to_crs(epsg=4326).total_bounds

    print("Downloading active / hostile POIs from OSM in tiles...")
    active = download_osm_pois(ACTIVE_TAGS, bbox_wgs84, target_crs)
    hostile = download_osm_pois(HOSTILE_TAGS, bbox_wgs84, target_crs)

    if not active.empty:
        active = active[active.geometry.notnull() & ~active.geometry.is_empty].copy()
        active["geometry"] = active.geometry.centroid
    if not hostile.empty:
        hostile = hostile[hostile.geometry.notnull() & ~hostile.geometry.is_empty].copy()
        hostile["geometry"] = hostile.geometry.centroid

    print(f" active={len(active)} hostile={len(hostile)}")

    facades = buildings[["geometry"]].copy()
    facades["geometry"] = facades.geometry.boundary

    edges_buffered = edges[["edge_unique_id", "geometry"]].copy()
    if "highway" in edges.columns:
        edges_buffered["geometry"] = edges.apply(lambda r: r.geometry.buffer(edge_buffer(r["highway"])), axis=1)
    else:
        edges_buffered["geometry"] = edges_buffered.geometry.buffer(15.0)

    facades_per_edge = gpd.overlay(facades, edges_buffered, how="intersection")
    if facades_per_edge.empty:
        total = pd.DataFrame(columns=["edge_unique_id", "total_wall_len"])
    else:
        facades_per_edge["total_wall_len"] = facades_per_edge.geometry.length
        total = facades_per_edge.groupby("edge_unique_id")["total_wall_len"].sum().reset_index()

    active_totals = measure_frontages(facades_per_edge, active, "active", target_crs)
    hostile_totals = measure_frontages(facades_per_edge, hostile, "hostile", target_crs)

    edges = edges.merge(total, on="edge_unique_id", how="left")
    edges = edges.merge(active_totals, on="edge_unique_id", how="left")
    edges = edges.merge(hostile_totals, on="edge_unique_id", how="left")

    for col in ["total_wall_len", "active_wall_len", "hostile_wall_len"]:
        edges[col] = edges[col].fillna(0.0)

    edges["inactive_wall_len"] = (
        edges["total_wall_len"] - edges["active_wall_len"] - edges["hostile_wall_len"]
    ).clip(lower=0.0)
    edges["frontage_activity_ratio"] = np.where(
        edges["total_wall_len"] > 0,
        edges["active_wall_len"] / edges["total_wall_len"],
        0.0,
    )
    edges["blank_wall_ratio"] = np.where(
        edges["total_wall_len"] > 0,
        edges["hostile_wall_len"] / edges["total_wall_len"],
        0.0,
    )
    edges["street_activity_index"] = (edges["active_wall_len"] / (edges.geometry.length * 2.0)).clip(upper=1.0)

    edges = edges.drop(columns=["edge_unique_id"])
    output_path = base_dir / f"{city}_edges_with_frontages.gpkg"
    if output_path.exists():
        output_path.unlink()
    edges.to_file(output_path, driver="GPKG")
    print(f"saved: {output_path}")


if __name__ == "__main__":
    main()
