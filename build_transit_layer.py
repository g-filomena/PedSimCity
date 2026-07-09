"""
build_transit_layer.py
One-time offline preprocessing script for PedSimCity multi-modal transit.

What it does:
1. Reads the physical street intersection nodes from Torino_simplified_nodes.gpkg (in EPSG:3003).
2. Reads the GTFS feed (stops.txt, routes.txt, trips.txt, stop_times.txt) from Torino_gtfs.
3. Maps every transit stop to the bus/tram/metro lines that serve it.
4. Transforms stop coordinates (lat/lon EPSG:4326) into EPSG:3003 (Monte Mario / Italy zone 1).
5. Snaps each transit stop to its nearest pedestrian street node (`snapped_node_id`) using spatial cKDTree index.
6. Filters for stops inside or directly adjacent to the urban simulation network (snapping distance <= 250m).
7. Exports `transit_stops.gpkg` (GeoPackage) and `transit_stops.csv` directly into `src/main/resources/Torino_simplified/` (and `src/main/resources/Torino/`).
"""

import os
import csv
import sqlite3
import numpy as np
import geopandas as gpd
from shapely.geometry import Point
from pyproj import Transformer
from scipy.spatial import cKDTree
from pathlib import Path

# Define paths
ROOT_DIR = Path(__file__).resolve().parent
GTFS_DIR = ROOT_DIR / "src" / "main" / "resources" / "Torino" / "Torino_gtfs"
SIMPLIFIED_DIR = ROOT_DIR / "src" / "main" / "resources" / "Torino_simplified"
FULL_TORINO_DIR = ROOT_DIR / "src" / "main" / "resources" / "Torino"

NODES_GPKG = SIMPLIFIED_DIR / "Torino_simplified_nodes.gpkg"

def main():
    print("=====================================================================")
    print("[Step 1] Building PedSimCity Multi-Modal Transit Layer (GTFS Snapping)")
    print("=====================================================================")

    # 1. Load Pedestrian Street Graph Nodes
    print(f"\n[1/5] Loading pedestrian graph nodes from: {NODES_GPKG.name} ...")
    conn = sqlite3.connect(NODES_GPKG)
    cursor = conn.cursor()
    cursor.execute("SELECT nodeID, x, y FROM Torino_nodes WHERE x IS NOT NULL AND y IS NOT NULL;")
    node_rows = cursor.fetchall()
    conn.close()

    node_ids = np.array([r[0] for r in node_rows], dtype=np.int64)
    node_coords = np.array([[r[1], r[2]] for r in node_rows], dtype=np.float64)
    
    min_x, max_x = np.min(node_coords[:, 0]), np.max(node_coords[:, 0])
    min_y, max_y = np.min(node_coords[:, 1]), np.max(node_coords[:, 1])
    print(f"      Loaded {len(node_ids)} street nodes.")
    print(f"      Network Bounding Box (EPSG:3003): X=[{min_x:.1f}, {max_x:.1f}], Y=[{min_y:.1f}, {max_y:.1f}]")

    # Build KD-Tree for O(log N) nearest node lookup
    kdtree = cKDTree(node_coords)

    # 2. Map Trips & Routes to identify which routes serve which stops
    print("\n[2/5] Parsing GTFS route and trip mappings...")
    routes = {} # route_id -> (short_name, route_type_str)
    route_type_lookup = {"0": "TRAM", "1": "METRO", "2": "RAIL", "3": "BUS"}
    
    with open(GTFS_DIR / "routes.txt", mode="r", encoding="utf-8-sig", errors="ignore") as f:
        reader = csv.DictReader(f)
        for row in reader:
            rid = row["route_id"]
            sname = row.get("route_short_name") or row.get("route_long_name") or rid
            rtype = route_type_lookup.get(row.get("route_type", "3"), "BUS")
            routes[rid] = (sname, rtype)
    print(f"      Loaded {len(routes)} routes from routes.txt")

    # Map trip_id -> route_id
    trip_to_route = {}
    with open(GTFS_DIR / "trips.txt", mode="r", encoding="utf-8-sig", errors="ignore") as f:
        reader = csv.DictReader(f)
        for row in reader:
            trip_to_route[row["trip_id"]] = row["route_id"]
    print(f"      Loaded {len(trip_to_route)} trips from trips.txt")

    # Map stop_id -> set of routes served
    print("      Scanning stop_times.txt to link stops to routes (this takes ~3-5 seconds)...")
    stop_routes = {} # stop_id -> set of route_ids
    with open(GTFS_DIR / "stop_times.txt", mode="r", encoding="utf-8-sig", errors="ignore") as f:
        reader = csv.DictReader(f)
        for row in reader:
            sid = row["stop_id"]
            tid = row["trip_id"]
            rid = trip_to_route.get(tid)
            if rid:
                if sid not in stop_routes:
                    stop_routes[sid] = set()
                stop_routes[sid].add(rid)
    print(f"      Linked {len(stop_routes)} active stops to transit lines.")

    # 3. Read & Transform Transit Stops from stops.txt
    print("\n[3/5] Transforming coordinates & snapping transit stops to street graph...")
    transformer = Transformer.from_crs("EPSG:4326", "EPSG:3003", always_xy=True)
    
    all_stops_read = 0
    snapped_stops = []

    with open(GTFS_DIR / "stops.txt", mode="r", encoding="utf-8-sig", errors="ignore") as f:
        reader = csv.DictReader(f)
        for row in reader:
            all_stops_read += 1
            sid = row["stop_id"]
            sname = row["stop_name"]
            lat_str = row["stop_lat"]
            lon_str = row["stop_lon"]
            
            try:
                lat = float(lat_str)
                lon = float(lon_str)
            except (ValueError, TypeError):
                continue
                
            # Transform to EPSG:3003
            x_stop, y_stop = transformer.transform(lon, lat)
            
            # Check rough bounding box with 500m buffer
            if not (min_x - 500 <= x_stop <= max_x + 500 and min_y - 500 <= y_stop <= max_y + 500):
                continue
                
            # Query KD-Tree for nearest pedestrian network node
            dist, idx = kdtree.query([x_stop, y_stop])
            
            # Filter for stops that snap reasonably close to our walkable network (within 250 meters)
            if dist <= 250.0:
                nearest_node_id = int(node_ids[idx])
                
                # Determine lines served
                rids = stop_routes.get(sid, set())
                served_names = sorted(list(set([routes[r][0] for r in rids if r in routes])))
                served_modes = sorted(list(set([routes[r][1] for r in rids if r in routes])))
                
                routes_str = ", ".join(served_names) if served_names else "N/A"
                modes_str = ", ".join(served_modes) if served_modes else "BUS"
                
                snapped_stops.append({
                    "stop_id": sid,
                    "stop_name": sname,
                    "stop_lat": lat,
                    "stop_lon": lon,
                    "x": round(x_stop, 2),
                    "y": round(y_stop, 2),
                    "snapped_node_id": nearest_node_id,
                    "snap_dist_m": round(float(dist), 1),
                    "modes_served": modes_str,
                    "routes_served": routes_str,
                    "geometry": Point(x_stop, y_stop)
                })

    print(f"      Total stops in GTFS: {all_stops_read}")
    print(f"      Stops successfully snapped to urban pedestrian graph: {len(snapped_stops)}")

    # Summary by mode
    metro_stops = [s for s in snapped_stops if "METRO" in s["modes_served"]]
    tram_stops = [s for s in snapped_stops if "TRAM" in s["modes_served"]]
    bus_stops = [s for s in snapped_stops if "BUS" in s["modes_served"]]
    print(f"      -> Metro Stations: {len(metro_stops)}")
    print(f"      -> Tram Stops:     {len(tram_stops)}")
    print(f"      -> Bus Stops:      {len(bus_stops)}")

    # 4. Export as GeoPackage and CSV
    print("\n[4/5] Exporting GIS GeoPackage and CSV layers...")
    gdf = gpd.GeoDataFrame(snapped_stops, crs="EPSG:3003")
    
    # Export to Torino_simplified
    gpkg_out_simp = SIMPLIFIED_DIR / "transit_stops.gpkg"
    csv_out_simp = SIMPLIFIED_DIR / "transit_stops.csv"
    
    # Save GeoPackage (remove geometry column for CSV export)
    if gpkg_out_simp.exists():
        gpkg_out_simp.unlink()
    gdf.to_file(gpkg_out_simp, layer="transit_stops", driver="GPKG")
    
    gdf_csv = gdf.drop(columns=["geometry"])
    gdf_csv.to_csv(csv_out_simp, index=False, encoding="utf-8")
    print(f"      Saved: {gpkg_out_simp.name} ({gpkg_out_simp.stat().st_size / 1024:.1f} KB)")
    print(f"      Saved: {csv_out_simp.name} ({csv_out_simp.stat().st_size / 1024:.1f} KB)")

    # 5. Copy to full Torino resources directory as well
    print("\n[5/5] Mirroring transit layer to full Torino resources...")
    gpkg_out_full = FULL_TORINO_DIR / "transit_stops.gpkg"
    csv_out_full = FULL_TORINO_DIR / "transit_stops.csv"
    
    if gpkg_out_full.exists():
        gpkg_out_full.unlink()
    gdf.to_file(gpkg_out_full, layer="transit_stops", driver="GPKG")
    gdf_csv.to_csv(csv_out_full, index=False, encoding="utf-8")
    print(f"      Saved: {gpkg_out_full.name}")
    print(f"      Saved: {csv_out_full.name}")

    print("\n=====================================================================")
    print("SUCCESS: Step 1 complete! Transit stops are snapped and ready for MASON runtime loading.")
    print("=====================================================================")

if __name__ == "__main__":
    main()
