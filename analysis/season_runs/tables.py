"""Season tables from seasonal Torino night runs: five seeds, four seasons, three days each.

Inputs: the per-run summaries summarise_run.py wrote on gdsl1 (one set per run, in --runs), the
network and lighting layers of --city in src/main/resources, the circoscrizioni in
inputData/Torino, and the Vodafone presence layer in analysis/validation/Torino/data.
Every figure is a mean and standard deviation across the five seeds of a season; each seed's value
is computed over its three simulated days.
"""

import argparse
import glob
from pathlib import Path

import geopandas as gpd
import numpy as np
import pandas as pd

REPO = Path(__file__).resolve().parents[2]
SEASONS = ["winter", "spring", "summer", "autumn"]
BANDS = {"06_08": range(6, 8), "08_10": range(8, 10), "10_12": range(10, 12),
         "12_14": range(12, 14), "14_16": range(14, 16), "16_18": range(16, 18),
         "18_20": range(18, 20), "20_24": range(20, 24), "00_06": range(0, 6)}
VODAFONE_WINDOW = {"winter": "Jan-Feb 2026", "spring": "Apr 2026", "summer": "Jul 2026",
                   "autumn": "Oct-Nov 2025"}
# Official composition of the eight circoscrizioni (City of Turin, reorganisation of 2016).
CIRC = {1: "1 Centro, Crocetta",
        2: "2 Santa Rita, Mirafiori Nord, Mirafiori Sud",
        3: "3 San Paolo, Cenisia, Pozzo Strada, Cit Turin, Borgata Lesna",
        4: "4 San Donato, Campidoglio, Parella",
        5: "5 Borgo Vittoria, Madonna di Campagna, Lucento, Vallette",
        6: "6 Barriera di Milano, Regio Parco, Barca, Bertolla, Falchera, Rebaudengo, Villaretto",
        7: "7 Aurora, Vanchiglia, Sassi, Madonna del Pilone",
        8: "8 San Salvario, Cavoretto, Borgo Po, Nizza Millefonti, Lingotto, Filadelfia"}
GROUPS = ("vulnerable", "nonVulnerable")
PERIODS = ("day", "dark")
METRE_COLS = ["vuln_day_m", "vuln_dark_m", "non_day_m", "non_dark_m"]


def ms(values, digits=1, suffix=""):
    v = np.asarray(values, dtype=float)
    return f"{v.mean():.{digits}f} ± {v.std(ddof=1):.{digits}f}{suffix}"


def season_seed(tag):
    *_, season, seed = tag.split("_")
    return season, seed[1:]


def load_edges(city):
    res = REPO / "src/main/resources" / city
    edges = gpd.read_file(res / f"{city}_edges.gpkg", columns=["edgeID", "name"])
    edges["edgeID"] = edges.edgeID.astype(int)
    edges["length"] = edges.geometry.length
    lux = gpd.read_file(res / f"{city}_edges_illuminated_continuous.gpkg", ignore_geometry=True)
    lux = lux[["edgeID", "mean_lux"]].astype({"edgeID": int})
    edges = edges.merge(lux, on="edgeID", how="left")
    iso = pd.read_csv(res / f"{city}_street_isovists.csv")[["edgeID", "iso_area_min"]]
    edges = edges.merge(iso, on="edgeID", how="left")
    # The street each edge belongs to (pipeline/street_corridors.py): a corso with its controviali
    # and sidewalks, or Via Roma with its portici, is one street; same-named streets elsewhere are not.
    streets = pd.read_csv(res / f"{city}_streets.csv", usecols=["edgeID", "street_id", "street_name"])
    edges = edges.merge(streets.astype({"edgeID": int}), on="edgeID", how="left")
    circ = gpd.read_file(REPO / "inputData/Torino/Torino_circoscrizioni.geojson").to_crs(edges.crs)
    mid = edges[["geometry"]].copy()
    mid["geometry"] = edges.geometry.interpolate(0.5, normalized=True)
    hit = gpd.sjoin(mid, circ[["N_circ", "geometry"]], how="left", predicate="within")
    edges["circ"] = hit.groupby(level=0)["N_circ"].first()
    return edges


def shares(df, key):
    p = df.groupby([key, "seed"])[METRE_COLS].sum()
    p["share_day"] = 100 * p.vuln_day_m / (p.vuln_day_m + p.non_day_m)
    p["share_dark"] = 100 * p.vuln_dark_m / (p.vuln_dark_m + p.non_dark_m)
    p["change"] = p.share_dark - p.share_day
    p["dark_km"] = (p.vuln_dark_m + p.non_dark_m) / 1000
    return p.reset_index()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--runs", type=Path, required=True)
    ap.add_argument("--out", type=Path, default=Path(__file__).resolve().parent)
    ap.add_argument("--city", default="Torino", help="the network the runs were made on")
    ap.add_argument("--tags", default="*", help="glob over the run tags in --runs")
    args = ap.parse_args()
    edges = load_edges(args.city)
    E = edges.set_index("edgeID")
    tags = sorted(Path(p).name[: -len("_trips.csv")]
                  for p in glob.glob(str(args.runs / f"{args.tags}_trips.csv")))
    out = [f"Runs: {len(tags)} ({tags[0].split('_')[0]}, 20% of residents, 3 days, 5 seeds per "
           "season). Enclosed streets: smallest isovist under 5,000 m2.", ""]

    # 1. trip length by group and period
    rows = []
    for t in tags:
        s, seed = season_seed(t)
        d = pd.read_csv(args.runs / f"{t}_trips.csv").groupby(["group", "period"])[["legs", "metres"]].sum()
        r = {"season": s, "seed": seed}
        for g in GROUPS:
            for p in PERIODS:
                r[f"{g}_{p}"] = d.metres[(g, p)] / d.legs[(g, p)]
                r[f"n_{g}_{p}"] = d.legs[(g, p)]
        rows.append(r)
    t1 = pd.DataFrame(rows)
    t1["gap_day"] = 100 * (t1.vulnerable_day / t1.nonVulnerable_day - 1)
    t1["gap_dark"] = 100 * (t1.vulnerable_dark / t1.nonVulnerable_dark - 1)
    t1.to_csv(args.out / "t1_trip_length_by_seed.csv", index=False)
    out += ["## 1. Mean route length (m), mean ± sd across 5 seeds", "",
            "| season | vulnerable, day | non-vuln., day | vulnerable, dark | non-vuln., dark "
            "| vuln. vs non-vuln. after dark | same, by day | legs after dark per seed |",
            "|---|---|---|---|---|---|---|---|"]
    for s in SEASONS:
        x = t1[t1.season == s]
        out.append(f"| {s} | {ms(x.vulnerable_day, 0)} | {ms(x.nonVulnerable_day, 0)} "
                   f"| {ms(x.vulnerable_dark, 0)} | {ms(x.nonVulnerable_dark, 0)} "
                   f"| {ms(x.gap_dark, 2, '%')} | {ms(x.gap_day, 2, '%')} "
                   f"| {(x.n_vulnerable_dark + x.n_nonVulnerable_dark).mean():,.0f} |")

    # 2 and 3 share the per-edge counts
    rows2, circ_rows, street_rows = [], [], []
    for t in tags:
        s, seed = season_seed(t)
        e = pd.read_csv(args.runs / f"{t}_edges.csv").set_index("edgeID")
        e = e.join(E[["length", "mean_lux", "circ", "street_id", "iso_area_min"]], how="left")
        if e.length.isna().any():
            raise SystemExit(f"{t}: {e.length.isna().sum()} edge IDs not in the network")
        for c in ("vuln_day", "vuln_dark", "non_day", "non_dark"):
            e[c + "_m"] = e[c] * e.length
        r = {"season": s, "seed": seed}
        for g, col in (("vulnerable", "vuln_dark_m"), ("nonVulnerable", "non_dark_m")):
            r[f"lux_{g}"] = (e[col] * e.mean_lux).sum() / e[col].sum()
            r[f"lt10_{g}"] = 100 * e.loc[e.mean_lux < 10, col].sum() / e[col].sum()
            r[f"lt5_{g}"] = 100 * e.loc[e.mean_lux < 5, col].sum() / e[col].sum()
            r[f"encl_{g}"] = 100 * e.loc[e.iso_area_min < 5000, col].sum() / e[col].sum()
        rows2.append(r)
        cm = e.groupby("circ")[METRE_COLS].sum()
        cm.index = cm.index.map(lambda c: str(int(c)))
        cm.loc["city"] = e[METRE_COLS].sum()
        cm.index.name = "circ"
        cm["season"], cm["seed"] = s, seed
        circ_rows.append(cm.reset_index())
        sm = e[e.street_id.notna()].groupby("street_id")[METRE_COLS].sum()
        sm["season"], sm["seed"] = s, seed
        street_rows.append(sm.reset_index())
    t2 = pd.DataFrame(rows2)
    t2.to_csv(args.out / "t2_lux_by_seed.csv", index=False)
    out += ["", "## 2. Illuminance of the street segments walked after dark, metre-weighted, "
            "mean ± sd across 5 seeds", "",
            "| season | mean lux, vulnerable | mean lux, non-vuln. | % metres < 10 lx, vuln. "
            "| % metres < 10 lx, non-vuln. | % metres < 5 lx, vuln. | % metres < 5 lx, non-vuln. "
            "| % metres on enclosed streets, vuln. | non-vuln. |",
            "|---|---|---|---|---|---|---|---|---|"]
    for s in SEASONS:
        x = t2[t2.season == s]
        out.append(f"| {s} | {ms(x.lux_vulnerable, 2)} | {ms(x.lux_nonVulnerable, 2)} "
                   f"| {ms(x.lt10_vulnerable, 2)} | {ms(x.lt10_nonVulnerable, 2)} "
                   f"| {ms(x.lt5_vulnerable, 2)} | {ms(x.lt5_nonVulnerable, 2)} "
                   f"| {ms(x.encl_vulnerable, 2)} | {ms(x.encl_nonVulnerable, 2)} |")

    # 3. vulnerable share of walked metres, day vs dark, the four seasons pooled per seed
    cs = shares(pd.concat(circ_rows), "circ")
    cs.to_csv(args.out / "t3_circoscrizioni_by_seed.csv", index=False)
    city = cs[cs.circ == "city"]
    city_change = city.change.mean()
    agg = (cs[cs.circ != "city"].groupby("circ")
           .agg(day=("share_day", "mean"), dark=("share_dark", "mean"),
                change=("change", "mean"), sd=("change", lambda v: v.std(ddof=1)),
                dark_km=("dark_km", "mean"))
           .sort_values("change"))
    out += ["", "## 3a. Vulnerable share of walked metres, by day and after dark, by circoscrizione "
            "(four seasons pooled), mean across 5 seeds", "",
            f"City-wide: {city.share_day.mean():.2f}% by day, {city.share_dark.mean():.2f}% after "
            f"dark, change {ms(city.change, 2)} points.", "",
            "| circoscrizione | share by day | share after dark | change, points (± sd) "
            "| change relative to the city | km walked after dark per seed |",
            "|---|---|---|---|---|---|"]
    for c, r in agg.iterrows():
        out.append(f"| {CIRC[int(c)]} | {r.day:.2f}% | {r.dark:.2f}% | {r.change:+.2f} ± {r.sd:.2f} "
                   f"| {r.change - city_change:+.2f} | {r.dark_km:,.0f} |")

    streets = pd.concat(street_rows)
    st = shares(streets, "street_id")
    in_street = edges[edges.street_id.notna()]
    length = in_street.groupby("street_id").length.sum()
    st["length_m"] = st.street_id.map(length)
    sagg = (st.groupby("street_id")
            .agg(day=("share_day", "mean"), dark=("share_dark", "mean"),
                 change=("change", "mean"), sd=("change", lambda v: v.std(ddof=1)),
                 below=("change", lambda v: int((v < city_change).sum())),
                 dark_km=("dark_km", "mean"), length_m=("length_m", "first")))
    sagg["street"] = in_street.groupby("street_id").street_name.first()
    sagg["lux"] = (in_street.mean_lux * in_street.length).groupby(in_street.street_id).sum() / length
    sagg["lt10"] = 100 * in_street[in_street.mean_lux < 10].groupby("street_id").length.sum().reindex(sagg.index).fillna(0) / sagg.length_m
    sagg = sagg[(sagg.length_m >= 500) & (sagg.dark_km >= 20)].sort_values("change")
    sagg.to_csv(args.out / "t3_streets.csv")
    out += ["", f"## 3b. Streets with the largest drop ({len(sagg)} streets with at least "
            "500 m of network - carriageways, side roads and sidewalks together - and 20 km "
            "walked after dark per seed)", "",
            "| street | share by day | share after dark | change, points (± sd) "
            "| seeds below the city-wide change | km walked after dark per seed "
            "| street's mean lux | % of its length < 10 lx |",
            "|---|---|---|---|---|---|---|---|"]
    for _, r in sagg.head(8).iterrows():
        out.append(f"| {r.street} | {r.day:.2f}% | {r.dark:.2f}% | {r.change:+.2f} ± {r.sd:.2f} "
                   f"| {int(r.below)}/5 | {r.dark_km:,.0f} | {r.lux:.1f} | {r.lt10:.0f}% |")

    # 4. evening departures after sunset
    rows4 = []
    for t in tags:
        s, seed = season_seed(t)
        ev = pd.read_csv(args.runs / f"{t}_evening.csv")[["legs_18_24", "after_sunset", "model_dark"]].sum()
        rows4.append({"season": s, "seed": seed, "evening_legs": ev.legs_18_24,
                      "after_sunset": 100 * ev.after_sunset / ev.legs_18_24,
                      "model_dark": 100 * ev.model_dark / ev.legs_18_24})
    t4 = pd.DataFrame(rows4)
    t4.to_csv(args.out / "t4_evening_by_seed.csv", index=False)
    out += ["", "## 4. Departures between 18:00 and 24:00 that set off after sunset, "
            "mean ± sd across 5 seeds", "",
            "| season | after sunset | in the model's darkness (sunset + 30 min) | evening legs per seed |",
            "|---|---|---|---|"]
    for s in SEASONS:
        x = t4[t4.season == s]
        out.append(f"| {s} | {ms(x.after_sunset, 1, '%')} | {ms(x.model_dark, 1, '%')} "
                   f"| {x.evening_legs.mean():,.0f} |")

    # 5. validation against Vodafone passing presence, 216 m quadkeys, per band
    vf = gpd.read_file(REPO / "analysis/validation/Torino/data/torino_presence.gpkg",
                       layer="quadkeys_long")
    cells = vf[vf.season == "winter"][["quadkey", "geometry"]].reset_index(drop=True)
    pieces = gpd.overlay(edges[["edgeID", "length", "geometry"]], cells, how="intersection",
                         keep_geom_type=True)
    pieces["w"] = pieces.geometry.length / pieces["length"]
    rows5 = []
    for t in tags:
        s, seed = season_seed(t)
        v = pd.read_csv(args.runs / f"{t}_edge_volume.csv")
        v["n"] = v.vuln_count + v.norm_count
        obs = vf[vf.season == s].set_index("quadkey")
        for band, hours in BANDS.items():
            per_edge = v[v.hour.isin(list(hours))].groupby("edgeID").n.sum()
            sim = (pieces.edgeID.map(per_edge).fillna(0) * pieces.w).groupby(pieces.quadkey).sum()
            both = pd.DataFrame({"sim": sim, "obs": obs[f"passing_{band}"]}).dropna()
            rows5.append({"season": s, "seed": seed, "band": band, "cells": len(both),
                          "rho": both.sim.corr(both.obs, method="spearman")})
    t5 = pd.DataFrame(rows5)
    t5.to_csv(args.out / "t5_validation_by_seed_band.csv", index=False)
    per_seed = t5.groupby(["season", "seed"]).rho.mean().reset_index()
    out += ["", "## 5. Spearman correlation, simulated street crossings vs Vodafone passing presence "
            "(0-30 min dwell), per time band, 216 m cells containing street", "",
            "| season (Vodafone window) | mean over the 9 bands (± sd across seeds) | lowest band "
            "| highest band | cells |", "|---|---|---|---|---|"]
    for s in SEASONS:
        x = t5[t5.season == s].groupby("band").rho.mean()
        out.append(f"| {s} ({VODAFONE_WINDOW[s]}) | {ms(per_seed[per_seed.season == s].rho, 3)} "
                   f"| {x.idxmin()}: {x.min():.3f} | {x.idxmax()}: {x.max():.3f} "
                   f"| {int(t5[t5.season == s].cells.median())} |")

    (args.out / "tables.md").write_text("\n".join(out) + "\n", encoding="utf-8")
    print("\n".join(out))


if __name__ == "__main__":
    main()
