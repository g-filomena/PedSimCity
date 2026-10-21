"""Car parks and the zone where a car is left in one, for PedSimCity.

Generic: any city. Writes two files into ``src/main/resources/<City>/``:

- ``<City>_parking.csv``: parking structures with their capacity, each snapped to the nearest
  street node (``name``, ``capacity``, ``x``, ``y`` in the CRS of ``<City>_nodes.gpkg``,
  ``snapped_node_id``, ``snap_dist_m``).
- ``<City>_restricted_zone_nodes.csv``: the street nodes inside ``--zone``, one ``nodeID`` per row.

The model reads them together: someone who drives to a destination inside the zone leaves the car
in a structure and walks from it, within the structures' capacity; elsewhere the car is left at
the door. A city with no zone file has no such zone.

Structures come from ``--parking``: a CSV with ``lat``/``lon`` (WGS84), or any vector file
geopandas reads, with a capacity column (``--capacity-column``, default ``capacity``) and
optionally a name column. Without ``--parking`` they are downloaded from OpenStreetMap: features
tagged ``amenity=parking`` with ``parking=multi-storey`` or ``underground`` and a numeric
``capacity``. The zone is any polygon layer geopandas reads (``zip://`` archives included), for
instance a limited-traffic zone or a pedestrianised centre.

    python parking.py --city Torino --parking ../inputData/Torino/parking/parking.csv \\
        --zone "zip://../inputData/Torino/parking/ztl_geo.zip" --source "5T get_pk (CC-BY)" \\
        --source "Comune di Torino geoportale, ztl_geo"
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

STRUCTURES = {"multi-storey", "underground"}


def from_file(path: str, capacity_column: str, name_column: str) -> gpd.GeoDataFrame:
    if str(path).lower().endswith(".csv"):
        table = pd.read_csv(path)
        parks = gpd.GeoDataFrame(
            table, geometry=gpd.points_from_xy(table["lon"], table["lat"]), crs=4326)
    else:
        parks = gpd.read_file(path)
    parks["capacity"] = pd.to_numeric(parks[capacity_column], errors="coerce")
    parks["name"] = parks[name_column] if name_column in parks else ""
    return parks


def from_osm(nodes: gpd.GeoDataFrame) -> gpd.GeoDataFrame:
    import cityImage as ci

    area = gpd.GeoSeries([nodes.union_all().convex_hull.buffer(500.0)], crs=nodes.crs)
    print(f"downloading parking structures from {ox.settings.overpass_url}")
    parks = ci.features_from_osm(area.to_crs(4326).iloc[0], {"amenity": "parking"},
                                 download_method="polygon", crs=nodes.crs)
    parks = parks[parks.get("parking").isin(STRUCTURES)].copy()
    parks["capacity"] = pd.to_numeric(parks.get("capacity"), errors="coerce")
    parks["name"] = parks.get("name", "")
    return parks


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--city", required=True, help="folder and file prefix, e.g. Torino")
    parser.add_argument("--parking", help="CSV (lat, lon) or vector file of parking structures; "
                                          "OpenStreetMap when omitted")
    parser.add_argument("--capacity-column", default="capacity")
    parser.add_argument("--name-column", default="name")
    parser.add_argument("--zone", help="polygon layer of the zone where cars are left in a car park")
    parser.add_argument("--source", action="append", default=[],
                        help="where an input came from, for the provenance; repeatable")
    parser.add_argument("--max-snap-metres", type=float, default=250.0)
    parser.add_argument("--overpass-url", default="https://maps.mail.ru/osm/tools/overpass/api",
                        help="Overpass endpoint when structures come from OpenStreetMap")
    args = parser.parse_args()
    ox.settings.overpass_url = args.overpass_url
    # Mirrors answer OSMnx's slot check with an error page, after which it sleeps 60 s each time.
    ox.settings.overpass_rate_limit = False

    resources = paths.resources_dir(args.city)
    nodes = gpd.read_file(resources / f"{args.city}_nodes.gpkg")
    parks = (from_file(args.parking, args.capacity_column, args.name_column) if args.parking
             else from_osm(nodes))
    parks = parks[parks.capacity > 0].to_crs(nodes.crs)
    parks["geometry"] = parks.geometry.centroid
    tree = cKDTree(np.column_stack([nodes.geometry.x, nodes.geometry.y]))
    dist, idx = tree.query(np.column_stack([parks.geometry.x, parks.geometry.y]))
    parks["snapped_node_id"] = nodes.nodeID.to_numpy()[idx].astype(int)
    parks["snap_dist_m"] = dist.round(1)
    parks = parks[parks.snap_dist_m <= args.max_snap_metres]
    parks["x"] = parks.geometry.x.round(1)
    parks["y"] = parks.geometry.y.round(1)
    parking_out = resources / f"{args.city}_parking.csv"
    parks[["name", "capacity", "x", "y", "snapped_node_id", "snap_dist_m"]].to_csv(
        parking_out, index=False)
    outputs = [parking_out]
    print(f"{parking_out.name}: {len(parks)} structures, {int(parks.capacity.sum())} spaces")

    zone_out = resources / f"{args.city}_restricted_zone_nodes.csv"
    if args.zone:
        zone = gpd.read_file(args.zone).to_crs(nodes.crs)
        inside = gpd.sjoin(nodes[["nodeID", "geometry"]], zone[["geometry"]], predicate="within")
        zone_nodes = sorted(set(inside.nodeID.astype(int)))
        pd.DataFrame({"nodeID": zone_nodes}).to_csv(zone_out, index=False)
        outputs.append(zone_out)
        print(f"{zone_out.name}: {len(zone_nodes)} nodes in {len(zone)} polygons "
              f"({zone.area.sum() / 1e6:.2f} km2)")
    elif zone_out.exists():
        zone_out.unlink()
        print(f"no --zone: removed {zone_out.name}")

    provenance.update(args.city, "parking", {
        "written": provenance.now(),
        "sources": args.source or (["OpenStreetMap"] if not args.parking else []),
        "settings": provenance.settings(args),
        "environment": provenance.environment(),
        "inputs": [provenance.file_info(Path(p)) for p in (args.parking,)
                   if p and Path(p).exists()],
        "outputs": [provenance.file_info(p) for p in outputs],
    })


if __name__ == "__main__":
    main()
