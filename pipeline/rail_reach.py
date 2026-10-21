"""The city's railway stations, and which of them trains from each origin reach, for PedSimCity.

Generic: any city, any number of GTFS feeds. Writes two files into ``src/main/resources/<City>/``:

- ``<City>_rail_stations.csv``: the rail stops of the feeds inside the street network's extent,
  merged when closer than ``--merge-metres`` and snapped to the nearest street node (``name``,
  ``x``, ``y``, ``snapped_node_id``, ``snap_dist_m``). A stop further than ``--max-snap-metres``
  from any node is left out.
- ``<City>_rail_reach.csv``: for every origin in ``--origins`` (by default the inbound commuters,
  ``<City>_inbound_commuters.csv``), the city stations that trains from the origin's own stations
  call at afterwards, and how many trains do (``origin_id``, ``station_name``, ``snapped_node_id``,
  ``trains``). An origin's stations are those within ``--home-radius`` of it, or the nearest one
  within ``--max-home-metres`` when none is that close.

An inbound commuter who comes by train then arrives at the reachable station nearest its
workplace: it reaches the city on its own line, not at whichever station is closest in a straight
line.

The feeds are ``--gtfs`` (repeatable), by default every ``*.zip`` in ``inputData/<City>/rail_gtfs/``.
A route is rail when its ``route_type`` is 2 or an extended railway type (100-199). Where a feed
came from belongs in ``--source`` (repeatable, recorded in the provenance); for Torino:

    python rail_reach.py --city Torino \\
        --source "Regione Piemonte regional trains GTFS (CSI Piemonte, CC-BY 2.5), GTFS_RAIL_IT_PIE.zip" \\
        --source "SFM Torino GTFS 2014-2016 (5T/GTT; Mobility Database tfs-664), sfm_2016.zip"
"""

from __future__ import annotations

import argparse
import collections
import io
import zipfile
from pathlib import Path

import geopandas as gpd
import numpy as np
import pandas as pd
from scipy.spatial import cKDTree

import paths
import provenance


def read(archive: zipfile.ZipFile, name: str) -> pd.DataFrame:
    return pd.read_csv(io.BytesIO(archive.read(name)), dtype=str, encoding="utf-8-sig")


def is_rail(route_type: str) -> bool:
    try:
        code = int(str(route_type).strip())
    except ValueError:
        return False
    return code == 2 or 100 <= code <= 199


def feed_tables(path: Path, crs):
    """The feed's stops (geometry in ``crs``) and the stop sequence of every rail trip."""
    with zipfile.ZipFile(path) as archive:
        stops = read(archive, "stops.txt")
        routes = read(archive, "routes.txt")
        trips = read(archive, "trips.txt")
        times = read(archive, "stop_times.txt")
    rail_routes = set(routes.route_id[routes.route_type.map(is_rail)])
    rail_trips = set(trips.trip_id[trips.route_id.isin(rail_routes)])
    times = times[times.trip_id.isin(rail_trips)]
    used = set(times.stop_id)
    stops = stops[stops.stop_id.isin(used)]
    stops = gpd.GeoDataFrame(
        stops[["stop_id", "stop_name"]],
        geometry=gpd.points_from_xy(stops.stop_lon.astype(float), stops.stop_lat.astype(float)),
        crs=4326).to_crs(crs)
    times = times.assign(seq=times.stop_sequence.astype(int)).sort_values(["trip_id", "seq"])
    return stops, times.groupby("trip_id").stop_id.apply(list)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--city", required=True, help="folder and file prefix, e.g. Torino")
    parser.add_argument("--gtfs", action="append", type=Path,
                        help="a GTFS zip; repeatable (default: inputData/<City>/rail_gtfs/*.zip)")
    parser.add_argument("--source", action="append", default=[],
                        help="where a feed came from, for the provenance; repeatable")
    parser.add_argument("--origins", type=Path,
                        help="CSV of origins (default: <City>_inbound_commuters.csv)")
    parser.add_argument("--origin-id", default="origin_id")
    parser.add_argument("--origin-x", default="origin_x")
    parser.add_argument("--origin-y", default="origin_y")
    parser.add_argument("--home-radius", type=float, default=5000.0,
                        help="metres from an origin to the stations that serve it")
    parser.add_argument("--max-home-metres", type=float, default=15000.0,
                        help="beyond this no station serves the origin")
    parser.add_argument("--merge-metres", type=float, default=300.0,
                        help="city stops closer than this are one station")
    parser.add_argument("--max-snap-metres", type=float, default=250.0)
    args = parser.parse_args()

    resources = paths.resources_dir(args.city)
    feeds = args.gtfs or sorted((paths.raw_dir(args.city) / "rail_gtfs").glob("*.zip"))
    if not feeds:
        raise SystemExit("no GTFS feed given or found in inputData/<City>/rail_gtfs/")
    nodes = gpd.read_file(resources / f"{args.city}_nodes.gpkg")
    extent = nodes.union_all().convex_hull
    origins_file = args.origins or resources / f"{args.city}_inbound_commuters.csv"
    origins = pd.read_csv(origins_file, dtype={args.origin_id: str})
    origins = (origins.dropna(subset=[args.origin_x])
               .groupby(args.origin_id)[[args.origin_x, args.origin_y]].first())

    # Every rail stop of every feed, and every rail trip as a sequence of feed-qualified stop IDs.
    all_stops, sequences = [], []
    for n, feed in enumerate(feeds):
        stops, trips = feed_tables(feed, nodes.crs)
        stops["key"] = f"{n}:" + stops.stop_id
        all_stops.append(stops)
        sequences += [[f"{n}:{s}" for s in seq] for seq in trips]
    stops = pd.concat(all_stops, ignore_index=True)

    # The city's stations: stops inside the extent, merged by proximity across feeds.
    inside = stops[stops.within(extent)].reset_index(drop=True)
    station_of, stations = {}, []
    for _, row in inside.iterrows():
        for i, s in enumerate(stations):
            if row.geometry.distance(s["geometry"]) < args.merge_metres:
                station_of[row.key] = i
                break
        else:
            station_of[row.key] = len(stations)
            stations.append({"name": row.stop_name, "geometry": row.geometry})
    stations = gpd.GeoDataFrame(stations, crs=nodes.crs)
    tree = cKDTree(np.column_stack([nodes.geometry.x, nodes.geometry.y]))
    dist, idx = tree.query(np.column_stack([stations.geometry.x, stations.geometry.y]))
    stations["snapped_node_id"] = nodes.nodeID.to_numpy()[idx].astype(int)
    stations["snap_dist_m"] = dist.round(1)
    stations["x"] = stations.geometry.x.round(1)
    stations["y"] = stations.geometry.y.round(1)
    kept = stations.snap_dist_m <= args.max_snap_metres
    stations_out = resources / f"{args.city}_rail_stations.csv"
    stations[kept][["name", "x", "y", "snapped_node_id", "snap_dist_m"]].sort_values(
        "name").to_csv(stations_out, index=False)

    # Trains from each stop outside the city to each city station they call at later.
    reach = collections.defaultdict(collections.Counter)
    for seq in sequences:
        for i, key in enumerate(seq):
            if key in station_of:
                continue
            for station in {station_of[k] for k in seq[i + 1:] if k in station_of}:
                reach[key][station] += 1

    outside = stops[~stops.key.isin(station_of)].reset_index(drop=True)
    stop_tree = cKDTree(np.column_stack([outside.geometry.x, outside.geometry.y]))
    rows = []
    for origin, where in origins.iterrows():
        xy = [where[args.origin_x], where[args.origin_y]]
        near = stop_tree.query_ball_point(xy, args.home_radius)
        if not near:
            d, i = stop_tree.query(xy)
            near = [i] if d <= args.max_home_metres else []
        trains = collections.Counter()
        for i in near:
            trains.update(reach.get(outside.key.iloc[i], {}))
        for station, n in sorted(trains.items()):
            if kept.iloc[station]:
                rows.append({"origin_id": origin, "station_name": stations.name.iloc[station],
                             "snapped_node_id": int(stations.snapped_node_id.iloc[station]),
                             "trains": n})
    out = pd.DataFrame(rows, columns=["origin_id", "station_name", "snapped_node_id", "trains"])
    target = resources / f"{args.city}_rail_reach.csv"
    out.to_csv(target, index=False)

    served = out.origin_id.nunique()
    print(f"{stations_out.name}: {int(kept.sum())} stations; {target.name}: {served} of "
          f"{len(origins)} origins reach at least one")
    print(stations[kept][["name", "snap_dist_m"]].to_string(index=False))
    provenance.update(args.city, "rail_reach", {
        "written": provenance.now(),
        "sources": args.source,
        "settings": provenance.settings(args),
        "environment": provenance.environment(),
        "inputs": [provenance.file_info(f) for f in feeds] + [provenance.file_info(origins_file)],
        "outputs": [provenance.file_info(stations_out), provenance.file_info(target)],
        "stations": int(kept.sum()),
        "origins_served": int(served),
    })


if __name__ == "__main__":
    main()
