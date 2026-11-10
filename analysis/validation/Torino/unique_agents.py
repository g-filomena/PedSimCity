#!/usr/bin/env python3
"""Count distinct agents per cell per band, not edge crossings.

The two sides of the validation count different things. The model reports edge
crossings — an agent who walks a street three times in a band is three. The API
reports unique visitors — that same agent is one. Where the model sends people
back and forth, or where a cell holds many edges, the simulated side inflates
against the observed one by construction, and no amount of rank-correlating
fixes it.

`trips.csv` carries an agent id per trip, so the per-person count is
recoverable. This writes it into the sim GeoPackage as `agents_<band>` beside
the existing `band_<band>` crossings, and reports what changes.

A trip is assigned whole to the band of its start step: trips run a few steps
(a few minutes) against 2-hour bands, so the misassignment is small and only
affects trips straddling a boundary.

    python unique_agents.py
    python unique_agents.py --min-piece-m 20   # ignore edges clipping a corner
"""

from __future__ import annotations

import argparse
import logging
from pathlib import Path

import geopandas as gpd
import numpy as np
import pandas as pd

from vfgrid import BANDS

log = logging.getLogger("agents")

BAND_KEYS = [b[0] for b in BANDS]
BAND_HOURS = {
    "00_06": range(0, 6), "06_08": range(6, 8), "08_10": range(8, 10),
    "10_12": range(10, 12), "12_14": range(12, 14), "14_16": range(14, 16),
    "16_18": range(16, 18), "18_20": range(18, 20), "20_24": range(20, 24),
}
HOUR_BAND = {h: k for k, hours in BAND_HOURS.items() for h in hours}
STEP_SECONDS = 300


def edge_cell_pieces(edges: gpd.GeoDataFrame, cells: gpd.GeoDataFrame,
                     min_piece_m: float) -> pd.DataFrame:
    """Which cells each edge touches, and for how many metres."""
    pieces = gpd.overlay(edges[["edgeID", "geometry"]], cells[["quadkey", "geometry"]],
                         how="intersection", keep_geom_type=True)
    pieces["piece_m"] = pieces.length
    if min_piece_m > 0:
        before = len(pieces)
        pieces = pieces[pieces["piece_m"] >= min_piece_m]
        log.info("dropped %d edge-cell pieces under %.0f m", before - len(pieces), min_piece_m)
    return pieces[["edgeID", "quadkey", "piece_m"]]


def agents_per_cell(trips: pd.DataFrame, pieces: pd.DataFrame) -> pd.DataFrame:
    """Distinct agent ids per (quadkey, band)."""
    trips = trips.copy()
    trips["band"] = (((trips["start_step"] - 1) * STEP_SECONDS // 3600)
                     .clip(0, 23).map(HOUR_BAND))

    # one row per (trip, edge)
    long = trips[["agent_id", "band", "edge_ids"]].assign(
        edgeID=trips["edge_ids"].str.split(";")).explode("edgeID")
    long["edgeID"] = pd.to_numeric(long["edgeID"], errors="coerce")
    long = long.dropna(subset=["edgeID"])
    long["edgeID"] = long["edgeID"].astype(np.int64)
    log.info("%d trip-edge rows from %d trips", len(long), len(trips))

    hit = long.merge(pieces[["edgeID", "quadkey"]], on="edgeID", how="inner")
    hit = hit[["agent_id", "band", "quadkey"]].drop_duplicates()
    log.info("%d distinct agent-band-cell presences", len(hit))

    counts = hit.groupby(["quadkey", "band"]).size().unstack(fill_value=0)
    counts = counts.reindex(columns=BAND_KEYS, fill_value=0)
    counts.columns = [f"agents_{b}" for b in BAND_KEYS]
    return counts.reset_index()


def compare(cells: gpd.GeoDataFrame, api: gpd.GeoDataFrame) -> pd.DataFrame:
    """Spearman against observed passing presence, crossings vs unique agents."""
    street = cells[cells["street_m"] > 0]
    rows = []
    for season, obs in api.groupby("season"):
        merged = street.merge(obs[["quadkey"] + [f"passing_{b}" for b in BAND_KEYS]],
                              on="quadkey", how="inner")
        for b in BAND_KEYS:
            o = merged[f"passing_{b}"]
            rows.append({
                "season": season, "band": b,
                "rho_crossings": merged[f"band_{b}"].corr(o, method="spearman"),
                "rho_agents": merged[f"agents_{b}"].corr(o, method="spearman"),
                "sim_crossings": merged[f"band_{b}"].sum(),
                "sim_agents": merged[f"agents_{b}"].sum(),
            })
    out = pd.DataFrame(rows)
    out["gain"] = out["rho_agents"] - out["rho_crossings"]
    return out


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--trips", default="sim/trips.csv")
    p.add_argument("--edges", default="sim/edges.gpkg")
    p.add_argument("--sim", default="data/torino_simulation.gpkg")
    p.add_argument("--api", default="data/torino_presence.gpkg")
    p.add_argument("--min-piece-m", type=float, default=0.0)
    p.add_argument("--report", default="agents_vs_crossings.csv")
    args = p.parse_args()

    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(message)s",
                        datefmt="%H:%M:%S")

    edges = gpd.read_file(args.edges)
    if "edgeID" not in edges.columns:
        edges = edges.rename(columns={c: "edgeID" for c in edges.columns
                                      if c.lower() in ("edgeid", "id")})
    cells = gpd.read_file(args.sim, layer="sim_cells")
    trips = pd.read_csv(args.trips)
    log.info("%d edges, %d cells, %d trips, %d agents",
             len(edges), len(cells), len(trips), trips["agent_id"].nunique())

    pieces = edge_cell_pieces(edges.to_crs(cells.crs), cells, args.min_piece_m)
    counts = agents_per_cell(trips, pieces)

    cells = cells.drop(columns=[c for c in cells.columns if c.startswith("agents_")])
    cells = cells.merge(counts, on="quadkey", how="left")
    for b in BAND_KEYS:
        cells[f"agents_{b}"] = cells[f"agents_{b}"].fillna(0).astype(int)
        # crossings per person: how much the model repeats itself in this cell
        cells[f"repeat_{b}"] = np.where(cells[f"agents_{b}"] > 0,
                                        cells[f"band_{b}"] / cells[f"agents_{b}"],
                                        np.nan).round(3)
    cells.to_file(args.sim, layer="sim_cells", driver="GPKG")
    log.info("wrote agents_* and repeat_* into %s", args.sim)

    api = gpd.read_file(args.api, layer="quadkeys_long")
    table = compare(cells, api)
    table.to_csv(args.report, index=False)

    print("\n=== Spearman against observed passing presence, street cells only ===")
    for season in table["season"].unique():
        s = table[table["season"] == season]
        print(f"\n{season}")
        print(f"{'band':>7} {'crossings':>10} {'agents':>8} {'gain':>7} "
              f"{'sim cross':>11} {'sim agents':>11} {'repeat':>7}")
        for _, r in s.iterrows():
            rep = r["sim_crossings"] / r["sim_agents"] if r["sim_agents"] else float("nan")
            print(f"{r['band']:>7} {r['rho_crossings']:10.3f} {r['rho_agents']:8.3f} "
                  f"{r['gain']:+7.3f} {r['sim_crossings']:11,.0f} {r['sim_agents']:11,.0f} "
                  f"{rep:7.2f}")
    print(f"\nmean gain across all bands and seasons: {table['gain'].mean():+.3f}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
