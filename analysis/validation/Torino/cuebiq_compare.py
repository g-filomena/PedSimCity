"""A run against the Cuebiq walking-ping export: geohash6 tile by hour of the day.

The observed side is the mean daily count of pings whose speed to the previous ping, within 15
minutes, was under 8 km/h: no vehicle, but also anyone standing still (two pings of a phone that
does not move have a speed of 0). It is time spent by phones that are not in vehicles, which is what
the model's walked metres are proportional to when speed is constant, so the simulated side is
metres walked: crossings of an edge times its length, split over the tiles it runs through.

Only shares and ranks are compared: the sim is a sample of the population and the export a sample
of phones. Per hour, the rank correlation over tiles of the simulated metres with the observed
pings; over the day, the shape of the city's profile; and the ratio of the centre's tiles to the
ring's, hour by hour.

    python cuebiq_compare.py --run <run folder with data/edge_volume.csv> \
        --resources ../../../src/main/resources/Torino_sidewalks --cuebiq <torino_cuebiq/data> \
        --month 202507 [--weekend <run folder>] --out <folder>

With --weekend the day compared is five weekdays to two Sundays, as the month's mean is.
"""
import argparse
import csv
from pathlib import Path

import geopandas as gpd
import numpy as np
import pandas as pd

CRS = 3003
CENTRE = (7.6869, 45.0711)  # Piazza Castello, lon/lat
CENTRE_KM, RING_KM = 1.0, 3.0


def edge_tiles(resources, city, tiles):
    edges = gpd.read_file(Path(resources) / f"{city}_edges.gpkg")[["edgeID", "geometry"]].to_crs(CRS)
    pieces = gpd.overlay(edges, tiles[["geohash6", "geometry"]], how="intersection",
                         keep_geom_type=True)
    pieces["metres"] = pieces.geometry.length
    return pieces[["edgeID", "geohash6", "metres"]]


def simulated(run, pieces):
    volume = pd.read_csv(Path(run) / "data" / "edge_volume.csv")
    volume["crossings"] = volume.vuln_count + volume.norm_count
    merged = volume.merge(pieces, on="edgeID")
    merged["walked"] = merged.crossings * merged.metres
    return merged.groupby(["geohash6", "hour"]).walked.sum().rename("sim")


def observed(cuebiq, month):
    stats = pd.read_csv(Path(cuebiq) / f"geohash6_hour_stats_{month}.csv")
    return stats.set_index(["geohash6", "hour"]).n_pings_mean.rename("obs")


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--run", required=True)
    parser.add_argument("--weekend")
    parser.add_argument("--resources", required=True)
    parser.add_argument("--city", default="Torino_sidewalks")
    parser.add_argument("--cuebiq", required=True)
    parser.add_argument("--month", required=True)
    parser.add_argument("--out", required=True)
    args = parser.parse_args()
    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)

    tiles = gpd.read_file(Path(args.cuebiq) / "geohash6_tiles.gpkg").to_crs(CRS)
    pieces = edge_tiles(args.resources, args.city, tiles)
    sim = simulated(args.run, pieces)
    if args.weekend:
        sim = (5 * sim.reindex(sim.index, fill_value=0)
               + 2 * simulated(args.weekend, pieces).reindex(sim.index, fill_value=0)) / 7
    obs = observed(args.cuebiq, args.month)
    index = pd.MultiIndex.from_product([tiles.geohash6, range(24)], names=["geohash6", "hour"])
    table = pd.concat([sim, obs], axis=1).reindex(index).fillna(0.0)
    # Only tiles the network reaches can carry simulated walking; the others are left out of ranks.
    reach = set(pieces.geohash6)
    table = table[table.index.get_level_values(0).isin(reach)]
    table.to_csv(out / f"tile_hour_{args.month}.csv")

    rows = []
    for hour in range(24):
        t = table.xs(hour, level="hour")
        rows.append({"hour": hour, "tiles": len(t),
                     "rho": t.sim.rank().corr(t.obs.rank()),
                     "sim_share_pct": 100 * t.sim.sum() / table.sim.sum(),
                     "obs_share_pct": 100 * t.obs.sum() / table.obs.sum()})
    hours = pd.DataFrame(rows)

    point = gpd.GeoSeries(gpd.points_from_xy([CENTRE[0]], [CENTRE[1]]), crs=4326).to_crs(CRS).iloc[0]
    tiles["km"] = tiles.geometry.centroid.distance(point) / 1000
    ring = {"centre": tiles[tiles.km <= CENTRE_KM].geohash6, "ring": tiles[tiles.km >= RING_KM].geohash6}
    for hour_row in rows:
        t = table.xs(hour_row["hour"], level="hour")
        for key in ("sim", "obs"):
            c = t.loc[t.index.isin(ring["centre"]), key].mean()
            r = t.loc[t.index.isin(ring["ring"]), key].mean()
            hour_row[f"{key}_centre_ring"] = c / r if r > 0 else np.nan
    hours = pd.DataFrame(rows)
    hours.to_csv(out / f"hourly_{args.month}.csv", index=False)

    profile = hours[["sim_share_pct", "obs_share_pct"]]
    summary = {
        "month": args.month,
        "tiles": int(table.index.get_level_values(0).nunique()),
        "mean_rho_over_hours": float(hours.rho.mean()),
        "median_rho_over_hours": float(hours.rho.median()),
        "profile_pearson": float(profile.sim_share_pct.corr(profile.obs_share_pct)),
        "profile_spearman": float(profile.sim_share_pct.rank().corr(profile.obs_share_pct.rank())),
        "share_after_18_sim": float(hours[hours.hour >= 18].sim_share_pct.sum()),
        "share_after_18_obs": float(hours[hours.hour >= 18].obs_share_pct.sum()),
    }
    with open(out / f"summary_{args.month}.csv", "w", newline="") as handle:
        writer = csv.writer(handle)
        writer.writerows(summary.items())
    print(hours.round(3).to_string(index=False))
    print(summary)


if __name__ == "__main__":
    main()
