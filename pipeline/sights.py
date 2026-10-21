"""The sights tourists visit, weighted by how many visit them, for PedSimCity.

Generic: any city. Writes ``src/main/resources/<City>/<City>_sights.csv``: one row per sight,
snapped to the nearest street node (``name``, ``visitors``, ``x``, ``y`` in the CRS of
``<City>_nodes.gpkg``, ``snapped_node_id``, ``snap_dist_m``). The model reads ``visitors`` as the
sight's attraction for sightseeing.

The sights are the city's POIs (``<City>_POIs.gpkg``) whose ``tourism`` tag is one of
``--tourism-values``. Where ``--visitors`` gives a site's yearly visitors (a CSV with ``name``,
``lat``, ``lon``, ``visitors``), that site replaces every POI within ``--merge-metres`` of it, so a
museum mapped as a building, an entrance and a viewpoint counts once, with its own figure. Every
other POI counts ``--default-visitors``: a figure for a sight nobody counts, which no source sets,
and which decides how much the uncounted sights draw against the counted ones.

    python sights.py --city Torino --visitors ../inputData/Torino/sights/visitors.csv \\
        --source "Osservatorio Culturale del Piemonte, Relazione annuale 2023-2024"
"""

from __future__ import annotations

import argparse
from pathlib import Path

import geopandas as gpd
import numpy as np
import pandas as pd
from scipy.spatial import cKDTree

import paths
import provenance

TOURISM_VALUES = "attraction,museum,gallery,viewpoint,zoo,theme_park"


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--city", required=True, help="folder and file prefix, e.g. Torino")
    parser.add_argument("--visitors", help="CSV of counted sites: name, lat, lon, visitors")
    parser.add_argument("--tourism-values", default=TOURISM_VALUES,
                        help="comma-separated tourism tag values that make a POI a sight")
    parser.add_argument("--default-visitors", type=float, default=10000.0,
                        help="yearly visitors of a sight with no count; unsourced")
    parser.add_argument("--merge-metres", type=float, default=150.0)
    parser.add_argument("--source", action="append", default=[],
                        help="where an input came from, for the provenance; repeatable")
    parser.add_argument("--max-snap-metres", type=float, default=250.0)
    args = parser.parse_args()

    resources = paths.resources_dir(args.city)
    nodes = gpd.read_file(resources / f"{args.city}_nodes.gpkg")
    pois_path = resources / f"{args.city}_POIs.gpkg"
    pois = gpd.read_file(pois_path).to_crs(nodes.crs)
    values = {v.strip() for v in args.tourism_values.split(",") if v.strip()}
    sights = pois[pois.get("tourism").isin(values)].copy()
    sights["geometry"] = sights.geometry.centroid
    sights["name"] = sights.get("name", "").fillna("")
    sights["visitors"] = args.default_visitors
    sights = sights[["name", "visitors", "geometry"]]

    counted = 0
    replaced = 0
    if args.visitors:
        table = pd.read_csv(args.visitors)
        sites = gpd.GeoDataFrame(table, geometry=gpd.points_from_xy(table["lon"], table["lat"]),
                                 crs=4326).to_crs(nodes.crs)
        near = np.zeros(len(sights), dtype=bool)
        if len(sights):
            site_tree = cKDTree(np.column_stack([sites.geometry.x, sites.geometry.y]))
            dist, _ = site_tree.query(np.column_stack([sights.geometry.x, sights.geometry.y]))
            near = dist <= args.merge_metres
        replaced = int(near.sum())
        sights = pd.concat([sights[~near], sites[["name", "visitors", "geometry"]]],
                           ignore_index=True)
        sights = gpd.GeoDataFrame(sights, geometry="geometry", crs=nodes.crs)
        counted = len(sites)

    tree = cKDTree(np.column_stack([nodes.geometry.x, nodes.geometry.y]))
    dist, idx = tree.query(np.column_stack([sights.geometry.x, sights.geometry.y]))
    sights["snapped_node_id"] = nodes.nodeID.to_numpy()[idx].astype(int)
    sights["snap_dist_m"] = dist.round(1)
    outside = int((sights.snap_dist_m > args.max_snap_metres).sum())
    sights = sights[sights.snap_dist_m <= args.max_snap_metres].copy()
    sights["x"] = sights.geometry.x.round(1)
    sights["y"] = sights.geometry.y.round(1)
    out = resources / f"{args.city}_sights.csv"
    sights[["name", "visitors", "x", "y", "snapped_node_id", "snap_dist_m"]].to_csv(
        out, index=False)
    print(f"{out.name}: {len(sights)} sights ({counted} counted, replacing {replaced} POIs; "
          f"{outside} beyond {args.max_snap_metres:.0f} m of the network dropped), "
          f"{sights.visitors.sum():.0f} visitors a year")

    provenance.update(args.city, "sights", {
        "written": provenance.now(),
        "sources": args.source,
        "settings": provenance.settings(args),
        "environment": provenance.environment(),
        "inputs": [provenance.file_info(Path(p)) for p in (pois_path, args.visitors)
                   if p and Path(p).exists()],
        "outputs": [provenance.file_info(out)],
    })


if __name__ == "__main__":
    main()
