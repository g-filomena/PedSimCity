#!/usr/bin/env python3
"""What the observed side contains that the model does not, and what it drops.

Three things the headline Spearman cannot show, all computed from data already
on disk - no new API calls.

1. TEMPORAL PROFILE. The model's evening over-representation is partly a unit
   artefact. Crossings per person vary by band, so counting events instead of
   people inflates exactly the bands where agents move most. Reported against
   the observed profile both ways.

2. DWELL SENSITIVITY. `passing` is a 0-30 min dwell. That keeps every motorised
   traveller - a car crosses a 216 m cell in ~26 s - and drops pedestrians who
   linger, since a market, a piazza or a park bench land in 30-60 min. Both
   errors are real and they push opposite ways, so the cut is tested rather
   than assumed.

3. WHERE PEDESTRIANS CANNOT BE. Cells holding no walkable street at all in the
   full 44,278-edge pedestrian network, yet recording passing presence: that
   presence is not someone walking the network, so it measures the floor of
   non-pedestrian contamination directly instead of estimating it. The same
   question is then asked of the model's own railway and road barriers.

Deliberately NOT here: road class as a covariate. In Turin the corsi are both
the car arteries and the busiest pedestrian streets, so class correlates with
both and discriminates neither - and the model routes on the same network, so
class is collinear with the very predictor being tested.

    python sensitivity.py
"""

from __future__ import annotations

import argparse
import logging
from pathlib import Path

import geopandas as gpd
import pandas as pd

from vfgrid import BANDS

log = logging.getLogger("sens")

BAND_KEYS = [b[0] for b in BANDS]
BAND_HOURS = {b[0]: b[2] for b in BANDS}

# the full city network, three levels up from analysis/validation/Torino
RESOURCES = Path(__file__).resolve().parents[3] / "src" / "main" / "resources" / "Torino"


def temporal(cells: gpd.GeoDataFrame, api: gpd.GeoDataFrame) -> pd.DataFrame:
    """Share of the day in each band: observed, sim crossings, sim unique agents.

    Bands are unequal in length, so each is put per hour before being shared out.
    """
    street = cells[cells["street_m"] > 0]
    obs = api.groupby("season")[[f"passing_{b}" for b in BAND_KEYS]].sum().mean()
    rows = []
    for b in BAND_KEYS:
        h = BAND_HOURS[b]
        rows.append({"band": b, "hours": h,
                     "obs_per_h": obs[f"passing_{b}"] / h,
                     "cross_per_h": street[f"band_{b}"].sum() / h,
                     "agents_per_h": street[f"agents_{b}"].sum() / h})
    t = pd.DataFrame(rows)
    for c in ("obs", "cross", "agents"):
        total = (t[f"{c}_per_h"] * t["hours"]).sum()
        t[f"{c}_share"] = t[f"{c}_per_h"] * t["hours"] / total
    return t


def dwell_sensitivity(cells: gpd.GeoDataFrame, api: gpd.GeoDataFrame) -> pd.DataFrame:
    """Spearman against three cuts of the observed data, per band and season."""
    street = cells[cells["street_m"] > 0]
    rows = []
    for season, obs in api.groupby("season"):
        m = street.merge(obs, on="quadkey", how="inner", suffixes=("", "_api"))
        for b in BAND_KEYS:
            sim = m[f"agents_{b}"]
            passing = m[f"passing_{b}"]
            mid = m[f"mid_{b}"]
            everyone = m[f"band_{b}_api"]
            rows.append({
                "season": season, "band": b,
                "rho_passing": sim.corr(passing, method="spearman"),
                "rho_0_60": sim.corr(passing + mid, method="spearman"),
                "rho_all": sim.corr(everyone, method="spearman"),
                "mid_share": mid.sum() / max(everyone.sum(), 1),
            })
    return pd.DataFrame(rows)


def line_metres(lines: gpd.GeoDataFrame, cells: gpd.GeoDataFrame) -> pd.Series:
    """Metres of a line layer falling inside each cell."""
    piece = gpd.overlay(lines[["geometry"]].reset_index(names="fid"),
                        cells[["quadkey", "geometry"]],
                        how="intersection", keep_geom_type=True)
    if piece.empty:
        return pd.Series(dtype=float)
    return piece.groupby("quadkey").apply(lambda d: d.length.sum(), include_groups=False)


def exposure(cells: gpd.GeoDataFrame) -> pd.DataFrame:
    """Walkable metres per cell in the full network, plus barrier exposure."""
    net = gpd.read_file(RESOURCES / "Torino_edges.gpkg").to_crs(cells.crs)
    log.info("full pedestrian network: %d edges, classes %s", len(net),
             sorted(net["highway"].dropna().unique())[:10])

    out = pd.DataFrame({"quadkey": cells["quadkey"]})
    out["walk_m"] = out["quadkey"].map(line_metres(net, cells)).fillna(0.0)

    bar = gpd.read_file(RESOURCES / "Torino_barriers.gpkg").to_crs(cells.crs)
    for kind in ("railway", "road", "water"):
        sub = bar[bar["barrier_type"] == kind]
        m = line_metres(sub, cells) if not sub.empty else pd.Series(dtype=float)
        out[f"{kind}_m"] = out["quadkey"].map(m).fillna(0.0)
        log.info("%s barriers: %d features, %d cells touched",
                 kind, len(sub), int((out[f"{kind}_m"] > 0).sum()))
    return out


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--sim", default="data/torino_simulation.gpkg")
    p.add_argument("--api", default="data/torino_presence.gpkg")
    p.add_argument("--report", default="sensitivity.csv")
    args = p.parse_args()
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(message)s",
                        datefmt="%H:%M:%S")

    cells = gpd.read_file(args.sim, layer="sim_cells")
    api = gpd.read_file(args.api, layer="quadkeys_long")
    if "agents_06_08" not in cells.columns:
        log.error("run unique_agents.py first - the agents_* columns are missing")
        return 1
    if "mid_06_08" not in api.columns:
        log.error("rebuild the presence GeoPackage - the mid_* columns are missing")
        return 1

    # ---- 1. temporal profile -------------------------------------------------
    t = temporal(cells, api)
    print("\n=== 1. Share of the day, per band ===")
    print(f"{'band':>7} {'h':>2} {'observed':>9} {'crossings':>10} {'agents':>8}")
    for _, r in t.iterrows():
        print(f"{r['band']:>7} {r['hours']:2.0f} {r['obs_share']:8.1%} "
              f"{r['cross_share']:9.1%} {r['agents_share']:7.1%}")
    ev = t[t["band"].isin(["18_20", "20_24"])]
    o, c, a = ev["obs_share"].sum(), ev["cross_share"].sum(), ev["agents_share"].sum()
    print(f"\nafter 18:00 - observed {o:.1%}, crossings {c:.1%}, agents {a:.1%}")
    print(f"the evening gap falls from {c - o:+.1%} to {a - o:+.1%}: "
          f"{1 - (a - o) / (c - o):.0%} of it was the unit mismatch, not the model")

    # ---- 2. dwell sensitivity ------------------------------------------------
    d = dwell_sensitivity(cells, api)
    print("\n=== 2. Which cut of the observed data the model matches best ===")
    print(f"{'band':>7} {'0-30 (used)':>12} {'0-60':>8} {'everyone':>9} {'30-60 of all':>13}")
    for b in BAND_KEYS:
        s = d[d["band"] == b]
        print(f"{b:>7} {s['rho_passing'].mean():12.3f} {s['rho_0_60'].mean():8.3f} "
              f"{s['rho_all'].mean():9.3f} {s['mid_share'].mean():12.1%}")
    print(f"\nmean over all bands and seasons: 0-30 {d['rho_passing'].mean():.3f}, "
          f"0-60 {d['rho_0_60'].mean():.3f}, everyone {d['rho_all'].mean():.3f}")

    # ---- 3. where pedestrians cannot be --------------------------------------
    m = cells.merge(exposure(cells), on="quadkey", how="left")
    obs = api.groupby("quadkey")[[f"passing_{b}" for b in BAND_KEYS]].mean().reset_index()
    m = m.merge(obs, on="quadkey", how="left")
    m["obs_passing"] = m[[f"passing_{b}" for b in BAND_KEYS]].sum(axis=1)

    nowalk = m["walk_m"] < 1.0
    print("\n=== 3. Observed presence where nobody can walk ===")
    print(f"cells with no walkable street in the full network: {nowalk.sum()} of {len(m)}")
    print(f"  of those, recording passing presence: {(nowalk & (m['obs_passing'] > 0)).sum()}")
    print(f"  share of all observed passing presence sitting there: "
          f"{m.loc[nowalk, 'obs_passing'].sum() / m['obs_passing'].sum():.2%}")
    print(f"  median passing, no-street cell {m.loc[nowalk, 'obs_passing'].median():.0f}"
          f"  vs walkable cell {m.loc[~nowalk, 'obs_passing'].median():.0f}")
    print(f"  cells the simulation network reaches but the full network calls "
          f"unwalkable: {((m['street_m'] > 0) & nowalk).sum()}")

    walk = m[m["walk_m"] >= 1.0].copy()
    walk["per_walk_m"] = walk["obs_passing"] / walk["walk_m"]
    print("\npassing presence per walkable metre, median:")
    for kind in ("railway", "road"):
        hit = walk[f"{kind}_m"] > 0
        on, off = walk.loc[hit, "per_walk_m"].median(), walk.loc[~hit, "per_walk_m"].median()
        print(f"  {kind:>8} cells {on:6.2f}  vs elsewhere {off:6.2f}   "
              f"ratio {on / off:.2f}x   (n={int(hit.sum())})")

    # Both ratios above are confounded, and in opposite directions to the naive
    # reading: road barriers are Turin's corsi, which carry the traffic AND the
    # pedestrians, and railway cells include Porta Nuova and Porta Susa, which
    # are among the largest pedestrian generators in the city. Stratifying by
    # how much walkable street a cell holds separates them: stations sit in the
    # densest quartile, open rail corridors in the sparsest. A contamination
    # signal should be strongest where there is least street, a pedestrian-
    # generator signal strongest where there is most.
    walk["band_q"] = pd.qcut(walk["walk_m"], 4, labels=["Q1 sparsest", "Q2", "Q3", "Q4 densest"])
    print("\nsame ratio, stratified by walkable street in the cell:")
    print(f"{'quartile':>12} {'walk_m':>9} {'railway':>9} {'other':>8} {'ratio':>7} {'n':>5}")
    for q, sub in walk.groupby("band_q", observed=True):
        hit = sub["railway_m"] > 0
        if hit.sum() < 5:
            continue
        on, off = sub.loc[hit, "per_walk_m"].median(), sub.loc[~hit, "per_walk_m"].median()
        print(f"{str(q):>12} {sub['walk_m'].median():9.0f} {on:9.2f} {off:8.2f} "
              f"{on / off:7.2f} {int(hit.sum()):5d}")

    d.to_csv(args.report, index=False)
    keep = ["quadkey", "walk_m", "railway_m", "road_m", "water_m", "street_m", "obs_passing"]
    m[keep].to_csv("cell_exposure.csv", index=False)
    log.info("wrote %s and cell_exposure.csv", args.report)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
