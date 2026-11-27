"""Scores night runs against Vodafone passing presence and Cuebiq walking pings, one row per run set.

A run set is the month's mean day: a Wednesday and a Sunday weighted five to two (or a Wednesday
alone). The measures are those of src/main/java/pedsim/night/CALIBRATION_VALIDATION.md:

    M1  Spearman's rho of cell shares per Vodafone band, total, women and men; misallocated share
    M2  Spearman's rho of the nine band shares (per hour), citywide
    M3  women's share of passing presence, night bands (20-06) against day bands (06-20)
    M4  per-cell presence within 1 km of Piazza Castello over the 4-5 km ring
    C   Cuebiq: Spearman's rho per hour of tile shares against walking devices, and of the 24 hourly totals
    L   the women-men difference in the share of night-band metres on streets under 5 and 10 lx

    python night_score.py prepare --resources <.../resources/Torino_sidewalks> --vodafone torino_presence.gpkg \\
        --cuebiq <torino_cuebiq/data> --out geometry.pkl
    python night_score.py score geometry.pkl --runs <outputs/PedSimCityNight> --sets sets.csv --out scores.csv

`sets.csv` has the columns set_id, season, wed, sun (sun may be empty) and any design columns, which
are copied to the output.
"""

from __future__ import annotations

import argparse
import pickle
from pathlib import Path

import geopandas as gpd
import numpy as np
import pandas as pd
from shapely.geometry import Point

BANDS = [("06_08", 2), ("08_10", 2), ("10_12", 2), ("12_14", 2), ("14_16", 2), ("16_18", 2),
         ("18_20", 2), ("20_24", 4), ("00_06", 6)]
BAND_KEYS = [b for b, _ in BANDS]
BAND_HOURS = dict(BANDS)
NIGHT_BANDS = ["20_24", "00_06"]
DAY_BANDS = [b for b in BAND_KEYS if b not in NIGHT_BANDS]
WEEKDAY_WEIGHT, SUNDAY_WEIGHT = 5, 2
CENTRE_LONLAT = (7.6869, 45.0711)          # Piazza Castello
CENTRE_M, RING_M = 1000.0, (4000.0, 5000.0)
CUEBIQ_MONTH = {"winter": "202502", "spring": "202504", "summer": "202507"}
LUX_LEVELS = (5.0, 10.0)


def band_of(hour: int) -> str:
    if hour < 6:
        return "00_06"
    if hour >= 20:
        return "20_24"
    return f"{hour - hour % 2:02d}_{hour - hour % 2 + 2:02d}"


HOUR_BAND = {h: band_of(h) for h in range(24)}


def rho(a, b) -> float:
    a, b = pd.Series(np.asarray(a, float)), pd.Series(np.asarray(b, float))
    ok = a.notna() & b.notna()
    if ok.sum() < 5 or a[ok].sum() <= 0 or b[ok].sum() <= 0:
        return np.nan
    return float(a[ok].rank().corr(b[ok].rank()))


def misallocated(sim, obs) -> float:
    sim, obs = np.asarray(sim, float), np.asarray(obs, float)
    ok = ~np.isnan(sim) & ~np.isnan(obs)
    s, o = sim[ok] / sim[ok].sum(), obs[ok] / obs[ok].sum()
    return float(0.5 * np.abs(s - o).sum())


# ------------------------------------------------------------------------------- prepare ----

def prepare(resources: Path, vodafone: Path, cuebiq: Path, out: Path) -> None:
    city = resources.name
    edges = gpd.read_file(resources / f"{city}_edges.gpkg")[["edgeID", "geometry"]]
    edges["length_m"] = edges.length
    lux = gpd.read_file(resources / f"{city}_edges_illuminated_continuous.gpkg", ignore_geometry=True)
    edges = edges.merge(lux[["edgeID", "mean_lux"]], on="edgeID", how="left")

    grid = gpd.read_file(vodafone, layer="quadkeys_seasons")[["quadkey", "geometry"]].to_crs(edges.crs)
    pieces = gpd.overlay(edges[["edgeID", "length_m", "geometry"]], grid, how="intersection",
                         keep_geom_type=True)
    # a cell's value is the metres walked in it in units of the mean street, as night_prepare_data
    pieces["wm"] = pieces.length / edges.length_m.mean()
    street_m = pieces.assign(m=pieces.length).groupby("quadkey").m.sum()
    centre = gpd.GeoSeries([Point(*CENTRE_LONLAT)], crs="EPSG:4326").to_crs(edges.crs).iloc[0]
    distance = grid.set_index("quadkey").geometry.centroid.distance(centre)

    observed = gpd.read_file(vodafone, layer="quadkeys_long", ignore_geometry=True)
    obs = {s: g.set_index("quadkey") for s, g in observed.groupby("season")}

    tiles = gpd.read_file(cuebiq / "geohash6_tiles.gpkg")[["geohash6", "geometry"]].to_crs(edges.crs)
    tile_pieces = gpd.overlay(edges[["edgeID", "geometry"]], tiles, how="intersection",
                              keep_geom_type=True)
    tile_pieces["m"] = tile_pieces.length
    cuebiq_obs = {}
    for season, month in CUEBIQ_MONTH.items():
        table = pd.read_csv(cuebiq / f"geohash6_hour_stats_{month}.csv")
        cuebiq_obs[season] = table.pivot_table(index="geohash6", columns="hour",
                                               values="n_devices_mean", aggfunc="sum")

    geometry = {"city": city, "edges": pd.DataFrame(edges.drop(columns="geometry")),
                "pieces": pd.DataFrame(pieces[["edgeID", "quadkey", "wm"]]),
                "street_m": street_m, "distance": distance, "obs": obs,
                "tile_pieces": pd.DataFrame(tile_pieces[["edgeID", "geohash6", "m"]]),
                "cuebiq": cuebiq_obs}
    with open(out, "wb") as handle:
        pickle.dump(geometry, handle)
    print(f"{city}: {len(edges)} edges, {pieces.quadkey.nunique()} cells, "
          f"{tile_pieces.geohash6.nunique()} tiles -> {out}")


# --------------------------------------------------------------------------------- score ----

def month_volume(runs: Path, wed: str, sun: str | None) -> pd.DataFrame:
    parts = []
    for tag, weight in ((wed, WEEKDAY_WEIGHT), (sun, SUNDAY_WEIGHT)):
        if not isinstance(tag, str) or not tag:
            continue
        volume = pd.read_csv(runs / tag / "data" / "edge_volume.csv")
        volume[["vuln_count", "norm_count"]] *= weight
        parts.append(volume)
    return pd.concat(parts).groupby(["edgeID", "hour"], as_index=False)[["vuln_count", "norm_count"]].sum()


def score_set(geo: dict, volume: pd.DataFrame, season: str) -> dict:
    out = {}
    volume = volume.assign(band=volume.hour.map(HOUR_BAND))
    obs = geo["obs"][season]
    cells = geo["street_m"][geo["street_m"] > 0].index

    per_band = {}
    for kind, short in (("vuln_count", "women"), ("norm_count", "men")):
        edge_band = volume.pivot_table(index="edgeID", columns="band", values=kind, aggfunc="sum",
                                       fill_value=0).reindex(columns=BAND_KEYS, fill_value=0.0)
        joined = geo["pieces"].join(edge_band, on="edgeID").fillna(0.0)
        for band in BAND_KEYS:
            joined[band] *= joined.wm
        per_band[short] = joined.groupby("quadkey")[BAND_KEYS].sum().reindex(cells, fill_value=0.0)
    per_band["total"] = per_band["women"] + per_band["men"]
    obs_col = {"total": "passing", "women": "passF", "men": "passM"}

    # M1
    for variant in ("total", "women", "men"):
        rhos = {}
        for band in BAND_KEYS:
            o = obs.reindex(cells)[f"{obs_col[variant]}_{band}"]
            rhos[band] = rho(per_band[variant][band], o)
            if variant == "total":
                out[f"mis_{band}"] = misallocated(per_band["total"][band][o.notna()], o.dropna())
        for band, value in rhos.items():
            out[f"rho_{variant}_{band}"] = value
        out[f"rho_{variant}_day"] = float(np.nanmean([rhos[b] for b in DAY_BANDS]))
        out[f"rho_{variant}_night"] = float(np.nanmean([rhos[b] for b in NIGHT_BANDS]))
    out["mis_day"] = float(np.mean([out[f"mis_{b}"] for b in DAY_BANDS]))
    out["mis_night"] = float(np.mean([out[f"mis_{b}"] for b in NIGHT_BANDS]))

    # M2
    sim_band = np.array([per_band["total"][b].sum() / BAND_HOURS[b] for b in BAND_KEYS])
    obs_band = np.array([np.nansum(obs[f"passing_{b}"]) / BAND_HOURS[b] for b in BAND_KEYS])
    out["rho_diurnal"] = rho(sim_band, obs_band)
    out["diurnal_mis"] = misallocated(sim_band, obs_band)

    # M3
    def women_share(women, men):
        return float(women / (women + men))
    for side, women, men in (
            ("sim", lambda bs: sum(per_band["women"][b].sum() for b in bs),
             lambda bs: sum(per_band["men"][b].sum() for b in bs)),
            ("obs", lambda bs: sum(np.nansum(obs[f"passF_{b}"]) for b in bs),
             lambda bs: sum(np.nansum(obs[f"passM_{b}"]) for b in bs))):
        day = women_share(women(DAY_BANDS), men(DAY_BANDS))
        night = women_share(women(NIGHT_BANDS), men(NIGHT_BANDS))
        out[f"women_day_{side}"], out[f"women_night_{side}"] = day, night
        out[f"women_change_{side}"] = night - day
    out["women_change_error"] = out["women_change_sim"] - out["women_change_obs"]
    night_w = sum(per_band["women"][b] for b in NIGHT_BANDS)
    night_t = sum(per_band["total"][b] for b in NIGHT_BANDS)
    o = obs.reindex(cells)
    of = sum(o[f"passF_{b}"] for b in NIGHT_BANDS)
    ot = of + sum(o[f"passM_{b}"] for b in NIGHT_BANDS)
    enough = (night_t >= night_t.quantile(0.5)) & (ot >= ot.quantile(0.5))
    out["rho_women_share_night"] = rho((night_w / night_t)[enough], (of / ot)[enough])

    # M4
    distance = geo["distance"].reindex(cells)
    centre, ring = distance < CENTRE_M, (distance >= RING_M[0]) & (distance < RING_M[1])
    for group, bands in (("day", DAY_BANDS), ("night", NIGHT_BANDS)):
        s = sum(per_band["total"][b] for b in bands)
        ob = sum(o[f"passing_{b}"] for b in bands)
        out[f"centre_ratio_{group}_sim"] = float(s[centre].mean() / s[ring].mean())
        out[f"centre_ratio_{group}_obs"] = float(ob[centre].mean() / ob[ring].mean())

    # C: Cuebiq
    if season in geo["cuebiq"]:
        cq = geo["cuebiq"][season]
        hourly = volume.assign(total=volume.vuln_count + volume.norm_count).merge(
            geo["edges"][["edgeID", "length_m"]], on="edgeID")
        tp = geo["tile_pieces"].merge(hourly[["edgeID", "hour", "total", "length_m"]], on="edgeID")
        tp["metres"] = tp.total * tp.m
        sim_tile = tp.pivot_table(index="geohash6", columns="hour", values="metres", aggfunc="sum",
                                  fill_value=0.0).reindex(index=cq.index, columns=range(24), fill_value=0.0)
        rhos = [rho(sim_tile[h], cq[h]) for h in range(24)]
        out["cuebiq_rho_day"] = float(np.nanmean(rhos[6:20]))
        out["cuebiq_rho_night"] = float(np.nanmean(rhos[20:] + rhos[:6]))
        out["cuebiq_rho_diurnal"] = rho(sim_tile.sum().values, cq.sum().values)

    # L: lux
    night_hours = volume[volume.band.isin(NIGHT_BANDS)].merge(
        geo["edges"][["edgeID", "length_m", "mean_lux"]], on="edgeID")
    for level in LUX_LEVELS:
        dim = night_hours.mean_lux < level
        shares = {}
        for kind in ("vuln_count", "norm_count"):
            metres = night_hours[kind] * night_hours.length_m
            shares[kind] = float(metres[dim].sum() / metres.sum())
        out[f"dim{int(level)}_women"] = shares["vuln_count"]
        out[f"dim{int(level)}_men"] = shares["norm_count"]
        out[f"dim{int(level)}_gap"] = shares["vuln_count"] - shares["norm_count"]
    return out


def score(geometry: Path, runs: Path, sets: Path, out: Path) -> None:
    with open(geometry, "rb") as handle:
        geo = pickle.load(handle)
    design = pd.read_csv(sets, dtype={"sun": str}).fillna({"sun": ""})
    done = pd.read_csv(out) if out.exists() else pd.DataFrame(columns=["set_id"])
    rows = [r for r in done.to_dict("records")]
    for row in design.itertuples(index=False):
        if row.set_id in set(done.set_id):
            continue
        try:
            volume = month_volume(runs, row.wed, row.sun or None)
        except FileNotFoundError as missing:
            print(f"{row.set_id}: skipped, {missing.filename} not found")
            continue
        rows.append({**row._asdict(), **score_set(geo, volume, row.season)})
        print(f"{row.set_id}: scored", flush=True)
    pd.DataFrame(rows).to_csv(out, index=False)


def main(argv=None) -> None:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    sub = parser.add_subparsers(dest="cmd", required=True)
    p = sub.add_parser("prepare")
    p.add_argument("--resources", type=Path, required=True)
    p.add_argument("--vodafone", type=Path, required=True)
    p.add_argument("--cuebiq", type=Path, required=True)
    p.add_argument("--out", type=Path, required=True)
    s = sub.add_parser("score")
    s.add_argument("geometry", type=Path)
    s.add_argument("--runs", type=Path, required=True)
    s.add_argument("--sets", type=Path, required=True)
    s.add_argument("--out", type=Path, required=True)
    args = parser.parse_args(argv)
    if args.cmd == "prepare":
        prepare(args.resources, args.vodafone, args.cuebiq, args.out)
    else:
        score(args.geometry, args.runs, args.sets, args.out)


if __name__ == "__main__":
    main()
