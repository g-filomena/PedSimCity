"""
build_transit_layer.py
One-time offline preprocessing script for PedSimCity multi-modal transit.

What it does:
1. Reads the physical street intersection nodes from `<City>_nodes.gpkg`.
2. Reads the GTFS feed (stops.txt, routes.txt, trips.txt, stop_times.txt, calendar.txt,
   calendar_dates.txt) from `<City>_gtfs/` or `<City>_gtfs.zip`, searched under `inputData/<City>/`
   then `src/main/resources/<City>/`.
3. Maps every transit stop to the bus/tram/metro lines that serve it.
4. Counts each stop's departures on one weekday (`--date`, by default the Wednesday in the feed
   with the most trips running), so that a stop can be weighed by how much service it has.
5. Transforms stop coordinates (lat/lon EPSG:4326) into the network's own projected CRS.
6. Snaps each transit stop to its nearest pedestrian street node (`snapped_node_id`) via a cKDTree.
7. Keeps stops inside or directly adjacent to the network (snapping distance <= 250 m).
8. Exports `<City>_transit_stops.gpkg` and `<City>_transit_stops.csv` into
   `src/main/resources/<City>/`.

`--city` names the folder under `src/main/resources/` and the `<City>_*` file prefix, the same
contract every pipeline script follows. No city is named in this file: the launcher
(`scripts/build_transit_layer.bat`) prompts for it and passes it through.
"""

import argparse
import collections
import csv
import datetime
import io
import sqlite3
import zipfile
import numpy as np
import geopandas as gpd
from shapely.geometry import Point
from pyproj import Transformer
from scipy.spatial import cKDTree
from pathlib import Path

# This script lives in scripts/; the repo root is one level up.
ROOT_DIR = Path(__file__).resolve().parent.parent


class GtfsFeed:
    """A GTFS feed read from its folder or straight from its zip archive."""

    def __init__(self, path: Path):
        self.path = path
        self.archive = zipfile.ZipFile(path) if path.suffix == ".zip" else None

    def rows(self, name: str):
        if self.archive is not None:
            raw = self.archive.open(name)
            handle = io.TextIOWrapper(raw, encoding="utf-8-sig", errors="ignore")
        else:
            handle = open(self.path / name, mode="r", encoding="utf-8-sig", errors="ignore")
        with handle:
            yield from csv.DictReader(handle)


def mode_of(route_type: str) -> str:
    """The mode of a GTFS route_type, basic (0-12) or extended (100-1700)."""
    try:
        code = int(str(route_type).strip())
    except ValueError:
        return "BUS"
    if code in (0, 5) or 900 <= code <= 999:
        return "TRAM"
    if code in (1, 12) or 400 <= code <= 499:
        return "METRO"
    if code == 2 or 100 <= code <= 199:
        return "RAIL"
    if code in (3, 11) or 200 <= code <= 299 or 700 <= code <= 899:
        return "BUS"
    return "OTHER"


def find_gtfs(city: str) -> GtfsFeed:
    """The GTFS feed, a folder or a zip, searching inputData/ then resources/ as the pipeline does."""
    candidates = [
        ROOT_DIR / "inputData" / city / f"{city}_gtfs",
        ROOT_DIR / "src" / "main" / "resources" / city / f"{city}_gtfs",
    ]
    for candidate in candidates:
        if candidate.is_dir():
            return GtfsFeed(candidate)
        if candidate.with_suffix(".zip").exists():
            return GtfsFeed(candidate.with_suffix(".zip"))
    searched = "\n  ".join(str(c) for c in candidates)
    raise FileNotFoundError(f"No GTFS feed for {city} (folder or .zip). Searched:\n  {searched}")


def services_on(feed: GtfsFeed, day: datetime.date) -> set:
    """The service_ids running on `day`: calendar.txt by weekday and range, then calendar_dates.txt
    exceptions (1 added, 2 removed)."""
    weekday = day.strftime("%A").lower()
    stamp = day.strftime("%Y%m%d")
    running = {r["service_id"] for r in feed.rows("calendar.txt")
               if r[weekday] == "1" and r["start_date"] <= stamp <= r["end_date"]}
    for r in feed.rows("calendar_dates.txt"):
        if r["date"] == stamp:
            if r["exception_type"] == "1":
                running.add(r["service_id"])
            elif r["exception_type"] == "2":
                running.discard(r["service_id"])
    return running


def reference_weekday(feed: GtfsFeed, trip_service: dict) -> datetime.date:
    """The Wednesday inside the feed's validity with the most trips running."""
    info = next(feed.rows("feed_info.txt"), {})
    start = datetime.datetime.strptime(info.get("feed_start_date", ""), "%Y%m%d").date()
    end = datetime.datetime.strptime(info.get("feed_end_date", ""), "%Y%m%d").date()
    trips_per_service = collections.Counter(trip_service.values())
    best, best_trips = None, -1
    day = start + datetime.timedelta(days=(2 - start.weekday()) % 7)
    while day <= end:
        trips = sum(trips_per_service[s] for s in services_on(feed, day))
        if trips > best_trips:
            best, best_trips = day, trips
        day += datetime.timedelta(days=7)
    return best


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
    parser.add_argument("--gtfs", default=None,
                        help="A GTFS folder or zip; by default <City>_gtfs(.zip) under inputData/<City>/ "
                             "or src/main/resources/<City>/.")
    parser.add_argument("--date", default=None,
                        help="Day whose departures are counted, YYYY-MM-DD; by default the "
                             "Wednesday in the feed with the most trips running.")
    args = parser.parse_args()
    city = args.city

    CITY_DIR = ROOT_DIR / "src" / "main" / "resources" / city
    NODES_GPKG = CITY_DIR / f"{city}_nodes.gpkg"

    if not NODES_GPKG.exists():
        raise FileNotFoundError(f"No node layer for {city}: {NODES_GPKG}")
    feed = GtfsFeed(Path(args.gtfs)) if args.gtfs else find_gtfs(city)
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
    
    for row in feed.rows("routes.txt"):
        rid = row["route_id"]
        sname = row.get("route_short_name") or row.get("route_long_name") or rid
        rtype = mode_of(row.get("route_type", "3"))
        routes[rid] = (sname, rtype)
    print(f"      Loaded {len(routes)} routes from routes.txt")

    # Map trip_id -> route_id and service_id
    trip_to_route = {}
    trip_service = {}
    for row in feed.rows("trips.txt"):
        trip_to_route[row["trip_id"]] = row["route_id"]
        trip_service[row["trip_id"]] = row["service_id"]
    print(f"      Loaded {len(trip_to_route)} trips from trips.txt")

    day = (datetime.date.fromisoformat(args.date) if args.date
           else reference_weekday(feed, trip_service))
    running = services_on(feed, day)
    print(f"      Departures counted on {day} ({day:%A}): {len(running)} services running")

    # Map stop_id -> set of routes served, and count the stop's departures on that day
    print("      Scanning stop_times.txt to link stops to routes...")
    stop_routes = {} # stop_id -> set of route_ids
    departures = collections.Counter()
    for row in feed.rows("stop_times.txt"):
        sid = row["stop_id"]
        tid = row["trip_id"]
        rid = trip_to_route.get(tid)
        if rid:
            if sid not in stop_routes:
                stop_routes[sid] = set()
            stop_routes[sid].add(rid)
            if trip_service.get(tid) in running:
                departures[sid] += 1
    print(f"      Linked {len(stop_routes)} active stops to transit lines; "
          f"{sum(departures.values()):,} stop departures on {day}.")

    # 3. Read & Transform Transit Stops from stops.txt
    print("\n[3/4] Transforming coordinates & snapping transit stops to street graph...")
    transformer = Transformer.from_crs("EPSG:4326", network_crs, always_xy=True)
    
    all_stops_read = 0
    snapped_stops = []

    for row in feed.rows("stops.txt"):
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
                "departures": departures.get(sid, 0),
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
    gpkg_out = CITY_DIR / f"{city}_transit_stops.gpkg"
    csv_out = CITY_DIR / f"{city}_transit_stops.csv"

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
