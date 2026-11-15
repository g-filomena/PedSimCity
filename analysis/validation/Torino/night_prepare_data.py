"""Reads the night runs and the Vodafone data once and writes what every figure needs.

    python night_prepare_data.py                          # the eight runs of the first seed
    python night_prepare_data.py --replicates s20261111 s20261112   # average them with the replicate seeds

Writes data/night_cache.pkl (read by the other scripts) and, in figures/night/tables/, validation_indicators.csv,
volumes_by_band_sex.csv and volumes_by_band_sex_all_seasons.csv.

The simulated side is street crossings (edge_volume.csv) by vulnerable agents (women) and the others (men); a weekday and
a Sunday are weighted five to two, and with replicates the runs of every seed are averaged. Each edge's count is split over
the 216 m cells it runs through, weighted by the metres of it in the cell (in units of the mean street). The observed side is Vodafone's passers-by (dwell under 30 min,
so residents and workers are left out) per cell, women and men apart.
"""
from __future__ import annotations

import argparse
import pickle

import geopandas as gpd
import numpy as np
import pandas as pd

from night_common import (BAND_HOURS, BAND_KEYS, BAND_LABEL, CACHE, CENSUS, CITY, HOUR_BAND, RESOURCES, SEASON_ORDER,
                          SEASONS, SUNDAY_WEIGHT, TABLES, VODAFONE, WEEKDAY_WEIGHT, RUNS, cell_frame, indicators)


def run_tags(month: str, day: str, replicates: list[str]) -> list[str]:
    tags = [f"val_{month}_{day}"] + [f"val_{month}_{day}_{r}" for r in replicates]
    return [t for t in tags if (RUNS / t / "data" / "edge_volume.csv").exists()]


def combined_volume(season: str, replicates: list[str]) -> tuple[pd.DataFrame, int]:
    """Edge x hour crossings of the month's mean day, averaged over the seeds that exist."""
    month = SEASONS[season]
    per_seed = []
    n_seeds = 0
    for r in [""] + replicates:
        parts = []
        for day, weight in (("wed", WEEKDAY_WEIGHT), ("sun", SUNDAY_WEIGHT)):
            tag = f"val_{month}_{day}" + (f"_{r}" if r else "")
            path = RUNS / tag / "data" / "edge_volume.csv"
            if not path.exists():
                parts = []
                break
            volume = pd.read_csv(path)
            volume[["vuln_count", "norm_count"]] *= weight
            parts.append(volume)
        if parts:
            per_seed.append(pd.concat(parts).groupby(["edgeID", "hour"], as_index=False)[["vuln_count", "norm_count"]].sum())
            n_seeds += 1
    if not per_seed:
        raise SystemExit(f"no run found for {season} ({month}) under {RUNS}")
    volume = pd.concat(per_seed).groupby(["edgeID", "hour"], as_index=False)[["vuln_count", "norm_count"]].mean()
    volume["band"] = volume.hour.map(HOUR_BAND)
    return volume, n_seeds


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--replicates", nargs="*", default=[], help="seed suffixes of replicate runs, e.g. s20261111")
    args = parser.parse_args()
    TABLES.mkdir(parents=True, exist_ok=True)

    edges = gpd.read_file(RESOURCES / f"{CITY}_edges.gpkg")[["edgeID", "geometry"]]
    edges["length_m"] = edges.length
    grid = gpd.read_file(VODAFONE, layer="quadkeys_seasons")[["quadkey", "geometry"]]
    assert edges.crs == grid.crs, (edges.crs, grid.crs)
    pieces = gpd.overlay(edges, grid, how="intersection", keep_geom_type=True)
    pieces["w"] = pieces.length / pieces.edgeID.map(edges.set_index("edgeID").length_m)
    # a cell's simulated value is the metres walked in it, in units of the mean street (a crossing of a mean street counts 1), so that
    # a cell of many short streets does not count a person once per street
    pieces["wm"] = pieces.length / edges.length_m.mean()
    street_m = pieces.groupby("quadkey").apply(lambda d: d.length.sum(), include_groups=False).reindex(grid.quadkey).fillna(0.0)

    census = gpd.read_file(CENSUS).to_crs(grid.crs)
    census["zone_area"] = census.area
    census_pieces = gpd.overlay(census[["residents", "zone_area", "geometry"]], grid, how="intersection", keep_geom_type=True)
    census_pieces["share"] = census_pieces.residents * census_pieces.area / census_pieces.zone_area
    residents = census_pieces.groupby("quadkey").share.sum().reindex(grid.quadkey).fillna(0.0)

    sim, seeds_used = {}, {}
    for season in SEASONS:
        volume, seeds_used[season] = combined_volume(season, args.replicates)
        out = {"volume": volume}
        for kind in ("vuln_count", "norm_count"):
            per_edge = volume.pivot_table(index="edgeID", columns="band", values=kind, aggfunc="sum", fill_value=0)
            per_edge = per_edge.reindex(columns=BAND_KEYS, fill_value=0.0)
            joined = pieces[["edgeID", "quadkey", "wm"]].join(per_edge, on="edgeID")
            for band in BAND_KEYS:
                joined[band] = joined[band].fillna(0.0) * joined.wm
            short = "vuln" if kind == "vuln_count" else "norm"
            out[f"edge_{short}"] = per_edge
            out[f"cell_{short}"] = joined.groupby("quadkey")[BAND_KEYS].sum().reindex(grid.quadkey, fill_value=0.0)
        sim[season] = out
        print(f"{season}: {seeds_used[season]} seed(s)")

    observed_long = gpd.read_file(VODAFONE, layer="quadkeys_long")
    obs = {s: observed_long[observed_long.season == s].drop(columns="geometry").set_index("quadkey").reindex(grid.quadkey) for s in SEASONS}

    cache = {"grid": grid, "edges": edges, "pieces": pieces, "street_m": street_m, "residents": residents, "sim": sim,
             "obs": obs, "seeds": seeds_used}
    women_share = sum(sim[s]["volume"].vuln_count.sum() for s in sim) / sum((sim[s]["volume"].vuln_count + sim[s]["volume"].norm_count).sum() for s in sim)
    obs_f = sum(np.nansum(obs[s][[f"passF_{b}" for b in BAND_KEYS]].to_numpy()) for s in obs)
    obs_m = sum(np.nansum(obs[s][[f"passM_{b}" for b in BAND_KEYS]].to_numpy()) for s in obs)
    cache["women_share_sim"], cache["women_share_obs"] = float(women_share), float(obs_f / (obs_f + obs_m))
    print(f"share of women: simulated {women_share:.1%}, observed among passers-by {obs_f / (obs_f + obs_m):.1%}")

    # ---- indicators per season, band and scale
    rows = []
    for season in SEASONS:
        for band in BAND_KEYS:
            for level, scale in ((None, "216 m cells"), (15, "864 m blocks")):
                df = cell_frame(cache, season, band, "total", level)
                model = indicators(df)
                street = indicators(df.assign(sim=df.street))
                people = indicators(df.assign(sim=df.residents))
                rows.append({"season": season, "band": band, "scale": scale, **model,
                             "rho_street": street["rho"], "rho_residents": people["rho"],
                             "misallocated_street": street["misallocated"], "misallocated_residents": people["misallocated"],
                             "skill_street": 1 - model["misallocated"] / street["misallocated"],
                             "skill_residents": 1 - model["misallocated"] / people["misallocated"]})
    pd.DataFrame(rows).to_csv(TABLES / "validation_indicators.csv", index=False)

    # ---- volumes recorded, by season, band and sex
    def total(frame, prefix, band):
        column = f"{prefix}_{band}"
        return float(np.nansum(frame[column].to_numpy())) if column in frame else float("nan")

    vol_rows = []
    for season in SEASON_ORDER:
        for band in BAND_KEYS:
            pf, pm = total(obs[season], "passF", band), total(obs[season], "passM", band)
            af, am = total(obs[season], "bandF", band), total(obs[season], "bandM", band)
            sf, sm = float(sim[season]["edge_vuln"][band].sum()), float(sim[season]["edge_norm"][band].sum())
            vol_rows.append({"season": season, "band": BAND_LABEL[band], "hours": BAND_HOURS[band],
                             "obs_passers_women": pf, "obs_passers_men": pm, "obs_all_women": af, "obs_all_men": am,
                             "sim_women": sf, "sim_men": sm})
    volumes = pd.DataFrame(vol_rows)
    for side in ("obs_passers", "obs_all", "sim"):
        f, m = f"{side}_women", f"{side}_men"
        volumes[f"{side}_pct_women"] = 100 * volumes[f] / (volumes[f] + volumes[m])
    for side in ("obs_passers", "sim"):
        tot = volumes[[f"{side}_women", f"{side}_men"]].sum(axis=1)
        volumes[f"{side}_share_of_day_pct"] = 100 * tot / tot.groupby(volumes.season).transform("sum")
    volumes.to_csv(TABLES / "volumes_by_band_sex.csv", index=False)

    whole = volumes.groupby("band", sort=False)[["obs_passers_women", "obs_passers_men", "obs_all_women", "obs_all_men", "sim_women", "sim_men"]].sum()
    whole.loc["all bands"] = whole.sum()
    for side in ("obs_passers", "obs_all", "sim"):
        whole[f"{side}_pct_women"] = 100 * whole[f"{side}_women"] / (whole[f"{side}_women"] + whole[f"{side}_men"])
    whole.to_csv(TABLES / "volumes_by_band_sex_all_seasons.csv")

    with open(CACHE, "wb") as handle:
        pickle.dump(cache, handle)
    print("wrote", CACHE)


if __name__ == "__main__":
    main()
