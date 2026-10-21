"""Which street each edge of a city's network belongs to.

Writes ``src/main/resources/<City>/<City>_streets.csv``: one row per edge of ``<City>_edges.gpkg``,
with the street it is part of, from cityImage's ``street_corridors``. A street in OSM is often
several parallel edges (a corso and its controviali, a pedestrian street and its arcades, a road
and its separately mapped sidewalks); counting volumes or lighting per street sums them.

- ``edgeID``       the edge
- ``street_id``    the street, empty for an edge in none (unnamed roads, crossings, paths)
- ``street_name``  the street's name, also on the unnamed sidewalks that joined it
- ``street_role``  ``carriageway``, ``side_road``, ``pedestrian_street``, ``sidewalk``,
  ``crossing`` or ``path``

The network itself is read, never rewritten, so its ``edgeID``s and every layer keyed on them stay
valid. Re-run after any network rebuild.

A side road is told by the higher-class road of its name beside it, and that road may not be in a
pedestrian network at all: Corso Regina Margherita's central carriageway is ``foot=no``. So the
named roads of the area, walkable or not, are downloaded from OSM once into
``inputData/<City>/<City>_osm_roads.gpkg`` and passed as ``reference_roads``.

    python street_corridors.py --city Torino
"""

from __future__ import annotations

import argparse

import cityImage as ci
import geopandas as gpd
import osmnx as ox

import paths
import provenance


# Road classes that can have a side road beside them.
ROAD_CLASSES = ["motorway", "trunk", "primary", "secondary", "tertiary", "unclassified",
                "residential"]


def osm_roads(edges: gpd.GeoDataFrame, target) -> gpd.GeoDataFrame:
    """The named roads over the network's extent, downloaded once and kept in ``target``."""
    if target.exists():
        return gpd.read_file(target)
    area = edges.union_all().convex_hull.buffer(100.0)
    area = gpd.GeoSeries([area], crs=edges.crs).to_crs(4326).iloc[0]
    tags = {"highway": ROAD_CLASSES + [f"{c}_link" for c in ROAD_CLASSES]}
    print(f"downloading named roads over the network's extent from {ox.settings.overpass_url}")
    features = ci.features_from_osm(area, tags, download_method="polygon", crs=edges.crs)
    features = features[features.geometry.geom_type.isin(["LineString", "MultiLineString"])]
    roads = features[features["name"].notna()].reset_index()
    roads = roads[["id", "highway", "name"] + (["foot"] if "foot" in roads else []) + ["geometry"]]
    roads.to_file(target, layer=target.stem, driver="GPKG")
    return roads


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--city", required=True, help="folder and file prefix, e.g. Torino")
    parser.add_argument("--max-width", type=float, default=40.0,
                        help="metres between a side road and the higher-class road it runs beside")
    parser.add_argument("--max-gap", type=float, default=250.0,
                        help="metres between two pieces of one named street")
    parser.add_argument("--max-offset", type=float, default=25.0,
                        help="metres from an unnamed walkway to the street it runs alongside")
    parser.add_argument("--max-angle", type=float, default=20.0,
                        help="degrees between two edges taken as parallel")
    parser.add_argument("--overpass-url", default="https://maps.mail.ru/osm/tools/overpass/api",
                        help="Overpass endpoint for the named roads (see 00_city_preparation.py)")
    args = parser.parse_args()
    ox.settings.overpass_url = args.overpass_url
    # Mirrors answer OSMnx's slot check with an error page, after which it sleeps 60 s each time.
    ox.settings.overpass_rate_limit = False

    source = paths.resources_dir(args.city) / f"{args.city}_edges.gpkg"
    edges = gpd.read_file(source)
    roads_file = paths.raw_dir(args.city) / f"{args.city}_osm_roads.gpkg"
    roads = osm_roads(edges, roads_file)
    streets = ci.street_corridors(edges, max_width=args.max_width, max_gap=args.max_gap,
                                  max_offset=args.max_offset, max_angle=args.max_angle,
                                  reference_roads=roads)
    table = streets.assign(edgeID=edges["edgeID"].values)[
        ["edgeID", "street_id", "street_name", "street_role"]]

    target = paths.resources_dir(args.city) / f"{args.city}_streets.csv"
    table.to_csv(target, index=False)

    km = edges.length.groupby(streets.street_role.values).sum() / 1000
    print(f"{target.name}: {len(table)} edges, {streets.street_id.nunique()} streets")
    for role, value in km.round(1).items():
        print(f"  {role:18} {value:8.1f} km")
    provenance.update(args.city, "streets", {
        "written": provenance.now(),
        "method": "cityImage.street_corridors",
        "cityImage": ci.__version__,
        "settings": provenance.settings(args),
        "environment": provenance.environment(),
        "inputs": [provenance.file_info(source), provenance.file_info(roads_file)],
        "output": provenance.file_info(target),
        "streets": int(streets.street_id.nunique()),
        "km_by_role": {role: float(value) for role, value in km.round(1).items()},
    })


if __name__ == "__main__":
    main()
