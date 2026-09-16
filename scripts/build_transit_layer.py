"""
build_transit_layer.py
One-time offline preprocessing script for PedSimCity multi-modal transit.

What it does:
1. Reads the physical street intersection nodes from `<City>_nodes.gpkg`.
2. Reads the GTFS feed (stops.txt, routes.txt, trips.txt, stop_times.txt) from `<City>_gtfs/`,
   searched under `inputData/<City>/` then `src/main/resources/<City>/`.
3. Maps every transit stop to the bus/tram/metro lines that serve it.
4. Transforms stop coordinates (lat/lon EPSG:4326) into the network's own projected CRS.
5. Snaps each transit stop to its nearest pedestrian street node (`snapped_node_id`) via a cKDTree.
6. Keeps stops inside or directly adjacent to the network (snapping distance <= 250 m).
7. Exports `transit_stops.gpkg` and `transit_stops.csv` into `src/main/resources/<City>/`.

`--city` names the folder under `src/main/resources/` and the `<City>_*` file prefix, the same
contract every pipeline script follows. No city is named in this file: the launcher
(`scripts/build_transit_layer.bat`) prompts for it and passes it through.
"""

import argparse
import os
import csv
import sqlite3
import numpy as np
import geopandas as gpd
from shapely.geometry import Point
from pyproj import Transformer
from scipy.spatial import cKDTree
from pathlib import Path

# This script lives in scripts/; the repo root is one level up.
ROOT_DIR = Path(__file__).resolve().parent.parent


def find_gtfs_dir(city: str) -> Path:
    """The extracted GTFS feed, searching inputData/ then resources/ as the pipeline does.

    Raw material lives in `inputData/<City>/` by convention, and this script used to look only in
    `src/main/resources/<City>/`. It is shipped as `<City>_gtfs.zip`, so the common failure is
    having the archive and not the folder; that gets its own message rather than a bare "not found".
    """
    candidates = [
        ROOT_DIR / "inputData" / city / f"{city}_gtfs",
        ROOT_DIR / "src" / "main" / "resources" / city / f"{city}_gtfs",
    ]
    for candidate in candidates:
        if candidate.is_dir():
            return candidate

    for archive in (c.with_suffix(".zip") for c in candidates):
        if archive.exists():
            raise FileNotFoundError(
                f"The GTFS feed for {city} is still an archive: {archive}\n"
                f"Extract it next to itself, so that {archive.with_suffix('')}/stops.txt exists."
            )

    searched = "\n  ".join(str(c) for c in candidates)
    raise FileNotFoundError(f"No GTFS feed for {city}. Searched:\n  {searched}")


def nodes_layer_crs(gpkg_path: Path, layer: str) -> str:
    """The projected CRS the node layer declares, as an `EPSG:<code>` string.

    Read rather than assumed: this script hardcoded `EPSG:3003` throughout, which is Monte Mario /
    Italy zone 1 and correct for exactly one of the bundled cities. The stops are snapped to these
    nodes by plane distance, so a transform into the wrong CRS does not fail - it just snaps every
    stop to the wrong node.
    """
    conn = sqlite3.connect(gpkg_path)
    try:
        row = conn.execute(
            "SELECT g.srs_id FROM gpkg_geometry_columns g WHERE g.table_name = ?", (layer,)
        ).fetchone()
    finally:
        conn.close()
    if not row or not row[0] or int(row[0]) in (0, 99999):
        raise ValueError(
            f"{gpkg_path.name} layer {layer!r} declares no usable CRS (srs_id={row and row[0]}). "
            "Re-export it with its projection declared; see pipeline/README.md on srs_id 99999."
        )
    return f"EPSG:{int(row[0])}"


def nodes_layer_name(gpkg_path: Path, expected: str) -> str:
    """The node layer inside the GeoPackage: `expected` when present, else its single table.

    A GeoPackage names its layers in `gpkg_contents`, so the layer need not be guessed from the
    filename - which is how this script came to read a table called `Torino_nodes` out of a file
    called `Torino_simplified_nodes.gpkg`.
    """
    conn = sqlite3.connect(gpkg_path)
    try:
        layers = [r[0] for r in conn.execute("SELECT table_name FROM gpkg_contents").fetchall()]
    finally:
        conn.close()
    if expected in layers:
        return expected
    if len(layers) == 1:
        print(f"      (reading layer {layers[0]!r}, not the expected {expected!r})")
        return layers[0]
    raise ValueError(
        f"{gpkg_path.name} holds {layers}; none is {expected!r} and there is no single obvious "
        "choice. Rename the layer, or point this script at the right file."
    )


def main():
    parser = argparse.ArgumentParser(
        description="Snap a GTFS feed's stops to a city's pedestrian street nodes."
    )
    parser.add_argument("--city", required=True,
                        help="City name: folder under src/main/resources/ and the <City>_* prefix.")
    args = parser.parse_args()
    city = args.city

    CITY_DIR = ROOT_DIR / "src" / "main" / "resources" / city
    NODES_GPKG = CITY_DIR / f"{city}_nodes.gpkg"

    if not NODES_GPKG.exists():
        raise FileNotFoundError(f"No node layer for {city}: {NODES_GPKG}")
    GTFS_DIR = find_gtfs_dir(city)
    print("=====================================================================")
    print("[Step 1] Building PedSimCity Multi-Modal Transit Layer (GTFS Snapping)")
    print("=====================================================================")

    # 1. Load Pedestrian Street Graph Nodes
    print(f"\n[1/4] Loading pedestrian graph nodes from: {NODES_GPKG.name} ...")
    layer = nodes_layer_name(NODES_GPKG, f"{city}_nodes")
    network_crs = nodes_layer_crs(NODES_GPKG, layer)
    conn = sqlite3.connect(NODES_GPKG)
    cursor = conn.cursor()
    cursor.execute(
        f'SELECT nodeID, x, y FROM "{layer}" WHERE x IS NOT NULL AND y IS NOT NULL;'
    )
    node_rows = cursor.fetchall()
    conn.close()

    node_ids = np.array([r[0] for r in node_rows], dtype=np.int64)
    node_coords = np.array([[r[1], r[2]] for r in node_rows], dtype=np.float64)
    
    min_x, max_x = np.min(node_coords[:, 0]), np.max(node_coords[:, 0])
    min_y, max_y = np.min(node_coords[:, 1]), np.max(node_coords[:, 1])
    print(f"      Loaded {len(node_ids)} street nodes.")
    print(f"      Network Bounding Box ({network_crs}): X=[{min_x:.1f}, {max_x:.1f}], Y=[{min_y:.1f}, {max_y:.1f}]")

    # Build KD-Tree for O(log N) nearest node lookup
    kdtree = cKDTree(node_coords)

    # 2. Map Trips & Routes to identify which routes serve which stops
    print("\n[2/4] Parsing GTFS route and trip mappings...")
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
    print("\n[3/4] Transforming coordinates & snapping transit stops to street graph...")
    transformer = Transformer.from_crs("EPSG:4326", network_crs, always_xy=True)
    
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
                
            # Transform into the network's own CRS
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
    print("\n[4/4] Exporting GIS GeoPackage and CSV layers...")
    gdf = gpd.GeoDataFrame(snapped_stops, crs=network_crs)
    
    # One city, one folder - the same contract the rest of the pipeline follows. This used to write
    # the same layer into two cities' folders because the stops were snapped to one network and read
    # by another; that is exactly how a derived layer ends up keyed on another graph's node IDs.
    gpkg_out = CITY_DIR / "transit_stops.gpkg"
    csv_out = CITY_DIR / "transit_stops.csv"

    # Save GeoPackage (remove geometry column for CSV export)
    if gpkg_out.exists():
        gpkg_out.unlink()
    gdf.to_file(gpkg_out, layer="transit_stops", driver="GPKG")

    gdf_csv = gdf.drop(columns=["geometry"])
    gdf_csv.to_csv(csv_out, index=False, encoding="utf-8")
    print(f"      Saved: {gpkg_out} ({gpkg_out.stat().st_size / 1024:.1f} KB)")
    print(f"      Saved: {csv_out} ({csv_out.stat().st_size / 1024:.1f} KB)")

    print("\n=====================================================================")
    print(f"SUCCESS: transit stops snapped to {city} and ready for MASON runtime loading.")
    print("=====================================================================")

if __name__ == "__main__":
    main()
