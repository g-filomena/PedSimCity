"""A validation run against the Vodafone passing presence: rank correlation per time band.

The simulated side is street crossings (edge_volume.csv), split over the 216 m quadkey cells an edge
runs through in proportion to its length, folded into the API's nine time bands. A weekday and a
Sunday of the same month are weighted five to two, as the month's mean day is.

    python vodafone_compare.py --run <weekday run> --weekend <Sunday run> --season winter \
        --resources ../../../src/main/resources/Torino_sidewalks --out out.csv

Seasons are the Vodafone windows: winter Jan-Feb 2026, spring Apr 2026, summer Jul 2026, autumn
Oct-Nov 2025. The rank correlation is taken over the cells that contain street.
"""
import argparse
from pathlib import Path

import geopandas as gpd
import pandas as pd

HERE = Path(__file__).resolve().parent
BANDS = {"06_08": range(6, 8), "08_10": range(8, 10), "10_12": range(10, 12),
         "12_14": range(12, 14), "14_16": range(14, 16), "16_18": range(16, 18),
         "18_20": range(18, 20), "20_24": range(20, 24), "00_06": range(0, 6)}


def crossings(run):
    volume = pd.read_csv(Path(run) / "data" / "edge_volume.csv")
    volume["n"] = volume.vuln_count + volume.norm_count
    return volume


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--run", required=True)
    parser.add_argument("--weekend")
    parser.add_argument("--season", required=True, choices=["winter", "spring", "summer", "autumn"])
    parser.add_argument("--resources", required=True)
    parser.add_argument("--city", default="Torino_sidewalks")
    parser.add_argument("--out")
    args = parser.parse_args()

    edges = gpd.read_file(Path(args.resources) / f"{args.city}_edges.gpkg")
    edges = edges[["edgeID", "geometry"]].copy()
    edges["length"] = edges.geometry.length
    vf = gpd.read_file(HERE / "data" / "torino_presence.gpkg", layer="quadkeys_long")
    obs = vf[vf.season == args.season].set_index("quadkey")
    cells = vf[vf.season == "winter"][["quadkey", "geometry"]].reset_index(drop=True)
    pieces = gpd.overlay(edges, cells, how="intersection", keep_geom_type=True)
    pieces["w"] = pieces.geometry.length / pieces["length"]

    weekday = crossings(args.run)
    sunday = crossings(args.weekend) if args.weekend else None
    rows = []
    for band, hours in BANDS.items():
        def per_edge(volume):
            return volume[volume.hour.isin(list(hours))].groupby("edgeID").n.sum()
        total = per_edge(weekday) * (5 if sunday is not None else 1)
        if sunday is not None:
            total = total.add(per_edge(sunday) * 2, fill_value=0)
        sim = (pieces.edgeID.map(total).fillna(0) * pieces.w).groupby(pieces.quadkey).sum()
        both = pd.DataFrame({"sim": sim, "obs": obs[f"passing_{band}"]}).dropna()
        rows.append({"band": band, "cells": len(both),
                     "rho": both.sim.corr(both.obs, method="spearman")})
    table = pd.DataFrame(rows)
    print(table.round(3).to_string(index=False))
    print(f"mean over the bands: {table.rho.mean():.3f}")
    if args.out:
        table.to_csv(args.out, index=False)


if __name__ == "__main__":
    main()
