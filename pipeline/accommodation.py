"""Tourist accommodation and its beds, for PedSimCity.

Generic: any city. Writes ``src/main/resources/<City>/<City>_accommodation.csv``: one row per
establishment, snapped to the nearest street node (``name``, ``type``, ``beds``, ``x``, ``y`` in
the CRS of ``<City>_nodes.gpkg``, ``snapped_node_id``, ``snap_dist_m``). The model places the
night's tourists at these nodes in proportion to their beds.

Establishments come from ``--accommodation``: a CSV with ``lat``/``lon`` (WGS84), or any vector
file geopandas reads, with a beds column (``--beds-column``, default ``beds``) and optionally
``name`` and ``type``. Without it they are downloaded from OpenStreetMap: ``tourism`` = hotel,
hostel, guest_house, motel or apartment, with their ``beds`` tag, twice their ``rooms`` tag when
that is all they carry, and otherwise ``--default-beds``.

    python accommodation.py --city Torino \\
        --accommodation ../inputData/Torino/accommodation/accommodation.csv \\
        --source "Regione Piemonte register of accommodation, via Comune di Torino open data (CC-BY)"
"""

from __future__ import annotations

import argparse
from pathlib import Path

import geopandas as gpd
import numpy as np
import osmnx as ox
import pandas as pd
from scipy.spatial import cKDTree

import paths
import provenance

OSM_TYPES = ["hotel", "hostel", "guest_house", "motel", "apartment"]


def from_file(path: str, beds_column: str) -> gpd.GeoDataFrame:
    if str(path).lower().endswith(".csv"):
        table = pd.read_csv(path)
        places = gpd.GeoDataFrame(
            table, geometry=gpd.points_from_xy(table["lon"], table["lat"]), crs=4326)
    else:
        places = gpd.read_file(path)
    places["beds"] = pd.to_numeric(places[beds_column], errors="coerce")
    for column in ("name", "type"):
        if column not in places:
            places[column] = ""
    return places


def from_osm(nodes: gpd.GeoDataFrame, default_beds: float) -> gpd.GeoDataFrame:
    import cityImage as ci

    area = gpd.GeoSeries([nodes.union_all().convex_hull.buffer(500.0)], crs=nodes.crs)
    print(f"downloading accommodation from {ox.settings.overpass_url}")
    places = ci.features_from_osm(area.to_crs(4326).iloc[0], {"tourism": OSM_TYPES},
                                  download_method="polygon", crs=nodes.crs)
    beds = pd.to_numeric(places.get("beds"), errors="coerce")
    rooms = pd.to_numeric(places.get("rooms"), errors="coerce")
    places["beds"] = beds.fillna(2.0 * rooms).fillna(default_beds)
    places["type"] = places["tourism"]
    places["name"] = places.get("name", "")
    return places


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--city", required=True, help="folder and file prefix, e.g. Torino")
    parser.add_argument("--accommodation", help="CSV (lat, lon) or vector file of establishments; "
                                                "OpenStreetMap when omitted")
    parser.add_argument("--beds-column", default="beds")
    parser.add_argument("--default-beds", type=float, default=20.0,
                        help="beds of an OpenStreetMap establishment tagged with neither beds nor "
                             "rooms; unsourced")
    parser.add_argument("--source", action="append", default=[],
                        help="where an input came from, for the provenance; repeatable")
    parser.add_argument("--max-snap-metres", type=float, default=250.0)
    parser.add_argument("--overpass-url", default="https://maps.mail.ru/osm/tools/overpass/api",
                        help="Overpass endpoint when establishments come from OpenStreetMap")
    args = parser.parse_args()
    ox.settings.overpass_url = args.overpass_url
    ox.settings.overpass_rate_limit = False

    resources = paths.resources_dir(args.city)
    nodes = gpd.read_file(resources / f"{args.city}_nodes.gpkg")
    places = (from_file(args.accommodation, args.beds_column) if args.accommodation
              else from_osm(nodes, args.default_beds))
    places = places[places.beds > 0].to_crs(nodes.crs)
    places["geometry"] = places.geometry.centroid
    tree = cKDTree(np.column_stack([nodes.geometry.x, nodes.geometry.y]))
    dist, idx = tree.query(np.column_stack([places.geometry.x, places.geometry.y]))
    places["snapped_node_id"] = nodes.nodeID.to_numpy()[idx].astype(int)
    places["snap_dist_m"] = dist.round(1)
    dropped = places[places.snap_dist_m > args.max_snap_metres]
    places = places[places.snap_dist_m <= args.max_snap_metres]
    places["x"] = places.geometry.x.round(1)
    places["y"] = places.geometry.y.round(1)
    out = resources / f"{args.city}_accommodation.csv"
    places[["name", "type", "beds", "x", "y", "snapped_node_id", "snap_dist_m"]].to_csv(
        out, index=False)
    print(f"{out.name}: {len(places)} establishments, {places.beds.sum():.0f} beds on "
          f"{places.snapped_node_id.nunique()} nodes; {len(dropped)} beyond "
          f"{args.max_snap_metres:.0f} m of the network dropped ({dropped.beds.sum():.0f} beds)")

    provenance.update(args.city, "accommodation", {
        "written": provenance.now(),
        "sources": args.source or (["OpenStreetMap"] if not args.accommodation else []),
        "settings": provenance.settings(args),
        "environment": provenance.environment(),
        "inputs": [provenance.file_info(Path(p)) for p in (args.accommodation,)
                   if p and Path(p).exists()],
        "outputs": [provenance.file_info(out)],
    })


if __name__ == "__main__":
    main()
