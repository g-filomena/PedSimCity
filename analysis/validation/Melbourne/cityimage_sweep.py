"""Sensitivity, calibration and validation of the cityImage route-choice models against Melbourne's
pedestrian counters.

Each run walks one shared set of origin-destination trips with one agent per route-choice scenario
(`--scenarios`), writes per-edge volumes to `outputs/PedSimCityImage/<run_id>/`, and is scored by
how well each scenario's volumes on the counter streets reproduce the counters' shares of the
weekday total. Shares and ranks, never magnitudes: the model walks a chosen number of trips while a
counter sees every pedestrian.

    python cityimage_sweep.py blocks  --sensors sensor_locations.csv --out sensor_blocks.csv
    python cityimage_sweep.py design  convergence|elements|morris --out design.csv
    python cityimage_sweep.py run     design.csv --repo <PedSimCity checkout> --parallel 20
    python cityimage_sweep.py score   design.csv --repo ... --targets sensor_targets_2025.csv \\
                                      --link sensor_edge_link.csv --blocks sensor_blocks.csv \\
                                      --out scores.csv
    python cityimage_sweep.py morris  design.csv scores.csv --out morris.csv

**Scoring.** For each run and scenario: Spearman, Kendall's tau-b and Pearson correlation between
model and observed shares over the usable counters, the RMSE of shares, and coverage (the share of
counters the model reaches at all), over all counters and with each spatial block held out
(`block_<b>_*` scores the held-out block alone, `train_<b>_*` the rest). Counters the model never
reaches tie at zero; tau-b handles those ties, and coverage reports how many there are. Calibration
fits on the training blocks and reports the held-out one, so a parameter cannot be tuned to the
counters it is judged on.

**Controls built in.** `ROAD_DISTANCE` is the null model: least distance over the same trips, i.e.
shortest-path betweenness of the trip sample. An element earns its place only by beating it.
Scenario runs within a Morris trajectory share one seed, so the trips are identical and an
elementary effect is the parameter's, not the dice's.

Standard library, numpy and pandas only: it runs on the server where the simulations run.
"""

from __future__ import annotations

import argparse
import concurrent.futures
import json
import os
import subprocess
import sys
from pathlib import Path

import numpy as np
import pandas as pd

CITY = "Melbourne"
MODE = "Testing Specific Route Choice Models"
LAUNCHER = "pedsim.cityimage.launcher.CityImageLauncher"

# One scenario per element family, each against the distance baseline; angular variants of the
# element scenarios are added by the `elements` design.
CORE_SCENARIOS = [
    "ROAD_DISTANCE",
    "ANGULAR_CHANGE",
    "REGION_DISTANCE",
    "BARRIER_DISTANCE",
    "LANDMARKS_DISTANCE",
    "LOCAL_LANDMARKS_DISTANCE",
    "DISTANT_LANDMARKS_DISTANCE",
]
ALL_SCENARIOS = CORE_SCENARIOS + [
    "LANDMARKS_ANGULAR",
    "LOCAL_LANDMARKS_ANGULAR",
    "DISTANT_LANDMARKS_ANGULAR",
    "DISTANT_LANDMARKS",
    "REGION_ANGULAR",
    "BARRIER_ANGULAR",
    "REGION_BARRIER_DISTANCE",
    "REGION_BARRIER_ANGULAR",
]

# Parameters screened by Morris: (field, low, high). Ranges span plausible values either side of
# the defaults in RouteChoicePars; they are the space searched, not claims about the city.
MORRIS_PARAMETERS = [
    ("salientNodesPercentile", 0.80, 0.97),
    ("globalLandmarkThresholdCommunity", 0.10, 0.60),
    ("localLandmarkThresholdCommunity", 0.10, 0.60),
    ("distanceNodeLandmark", 20.0, 100.0),
    ("wayfindingEasinessThresholdCommunity", 0.70, 0.99),
    ("globalLandmarknessWeightDistanceCommunity", 0.50, 1.00),
    ("regionNavActivationThreshold", 200.0, 1500.0),
    ("wayfindingEasinessThresholdRegionsCommunity", 0.60, 0.99),
    ("perceptionErrorSD", 0.00, 0.30),
    ("minRouteLength", 400.0, 1200.0),
]

# Counters left out of scoring, with the reason. `reliable == False` in the link (a snap over 25 m)
# is excluded as well.
EXCLUDED = {
    103: "Kensington Town Hall: outside the study area, 19 m inside the network's clipped edge",
    71: "Westwood Place: a service lane the cityImage pedestrian filter drops",
    93: "East Melbourne Library: on Eastcourt Lane, an alley the cityImage pedestrian filter drops",
}


# --------------------------------------------------------------------------- designs ----


def design_convergence(seeds: int) -> pd.DataFrame:
    """Trips per scenario against seed-to-seed spread: how many trips before the shares settle."""
    rows = []
    for trips in (250, 500, 1000, 2000, 4000):
        for seed in range(1, seeds + 1):
            rows.append(run_row(f"conv_t{trips}_s{seed}", seed, trips, CORE_SCENARIOS, {}))
    return pd.DataFrame(rows)


def design_elements(seeds: int, trips: int) -> pd.DataFrame:
    """Every scenario at default parameters, with and without floor-area-weighted trips."""
    rows = []
    for weighted in (True, False):
        for seed in range(1, seeds + 1):
            tag = "w" if weighted else "u"
            rows.append(run_row(f"elem_{tag}_s{seed}", seed, trips, ALL_SCENARIOS, {},
                                weighted=weighted))
    return pd.DataFrame(rows)


def design_morris(trajectories: int, trips: int, levels: int = 4, seed: int = 1) -> pd.DataFrame:
    """Morris one-at-a-time trajectories over MORRIS_PARAMETERS, all at one seed."""
    rng = np.random.default_rng(20260926)
    k = len(MORRIS_PARAMETERS)
    delta = levels / (2.0 * (levels - 1))
    grid = np.arange(levels) / (levels - 1)
    rows = []
    for t in range(trajectories):
        start = rng.choice(grid[grid <= 1.0 - delta + 1e-9], size=k)
        order = rng.permutation(k)
        signs = rng.choice([-1.0, 1.0], size=k)
        point = start.copy()
        # A step goes up by delta unless that would leave [0, 1].
        points = [point.copy()]
        changed = [None]
        for j in order:
            step = delta if (signs[j] > 0 and point[j] + delta <= 1.0 + 1e-9) or point[j] - delta < -1e-9 else -delta
            point[j] = point[j] + step
            points.append(point.copy())
            changed.append(j)
        for i, (x, j) in enumerate(zip(points, changed)):
            params = {name: round(low + xi * (high - low), 4)
                      for (name, low, high), xi in zip(MORRIS_PARAMETERS, x)}
            rows.append(run_row(f"morris_t{t}_p{i}", seed, trips, CORE_SCENARIOS, params,
                                trajectory=t, point=i,
                                changed="" if j is None else MORRIS_PARAMETERS[j][0]))
    return pd.DataFrame(rows)


def run_row(run_id, seed, trips, scenarios, params, weighted=True, **extra):
    return {"run_id": run_id, "seed": seed, "trips": trips, "scenarios": ",".join(scenarios),
            "weighted": weighted, "params": json.dumps(params, sort_keys=True), **extra}


# ------------------------------------------------------------------------------ runs ----


def command(row, repo: Path, heap: str) -> list[str]:
    java = os.environ.get("JAVA", "java")
    cp = f"{repo / 'target' / 'classes'}{os.pathsep}{(repo / 'cp.txt').read_text().strip()}"
    args = [java, f"-Xmx{heap}", "-cp", cp, LAUNCHER, "--headless", f"--cityName={CITY}",
            f"--stringMode={MODE}", f"--scenarios={row.scenarios}",
            f"--numberTripsPerAgent={int(row.trips)}", "--jobs=1", f"--seed={int(row.seed)}",
            f"--weightODByFloorArea={str(bool(row.weighted)).lower()}",
            f"--outputTag={row.run_id}", "--stepDelayMs=0", "--exportHtmlDashboard=false"]
    for key, value in json.loads(row.params).items():
        args.append(f"--{key}={value}")
    return args


def volumes_path(repo: Path, run_id: str) -> Path | None:
    folder = repo / "outputs" / "PedSimCityImage" / run_id / "streetVolumes"
    files = sorted(folder.glob("*.csv")) if folder.is_dir() else []
    return files[-1] if files else None


def execute(row, repo: Path, heap: str) -> tuple[str, int]:
    if volumes_path(repo, row.run_id) is not None:
        return row.run_id, 0
    log = repo / "outputs" / "sweep_logs"
    log.mkdir(parents=True, exist_ok=True)
    with (log / f"{row.run_id}.log").open("w") as handle:
        code = subprocess.run(command(row, repo, heap), cwd=repo, stdout=handle,
                              stderr=subprocess.STDOUT).returncode
    return row.run_id, code


def run_design(design: pd.DataFrame, repo: Path, parallel: int, heap: str) -> None:
    rows = list(design.itertuples(index=False))
    with concurrent.futures.ThreadPoolExecutor(max_workers=parallel) as pool:
        for n, (run_id, code) in enumerate(
                pool.map(lambda r: execute(r, repo, heap), rows), start=1):
            print(f"[{n}/{len(rows)}] {run_id}: {'ok' if code == 0 else f'exit {code}'}",
                  flush=True)


# --------------------------------------------------------------------------- scoring ----


def usable_counters(targets: pd.DataFrame, link: pd.DataFrame, blocks: pd.DataFrame,
                    daytype: str = "weekday", hours: str = "7-19") -> pd.DataFrame:
    """Counters with a target in the chosen day type and time window, a reliable snap, and no
    listed exclusion. 07-19 on weekdays by default: the daytime pattern a model without a clock
    implicitly represents."""
    t = targets[(targets.daytype == daytype) & (targets.hours == hours)][["location_id", "share"]]
    if t.location_id.duplicated().any():
        raise SystemExit(f"more than one target per counter for {daytype} {hours}")
    lk = link[["location_id", "edgeID", "reliable"]]
    table = t.merge(lk, on="location_id").merge(blocks, on="location_id")
    table = table[table.reliable.astype(str).str.lower().eq("true")]
    table = table[~table.location_id.isin(EXCLUDED)]
    return table.reset_index(drop=True)


def kendall_tau_b(x, y) -> float:
    """Kendall's tau-b: concordant minus discordant pairs over the pairs untied in each variable.
    pandas' own Kendall needs scipy, which the server lacks."""
    x, y = np.asarray(x, dtype=float), np.asarray(y, dtype=float)
    iu = np.triu_indices(len(x), k=1)
    dx = np.sign(np.subtract.outer(x, x))[iu]
    dy = np.sign(np.subtract.outer(y, y))[iu]
    denominator = np.sqrt(np.abs(dx).sum() * np.abs(dy).sum())
    return float((dx * dy).sum() / denominator) if denominator > 0 else np.nan


def metrics(model: pd.Series, observed: pd.Series) -> dict:
    """Spearman ranks ties at their average, which is correct but lets the counters the model never
    reaches (all tied at zero) flatten the bottom of the ranking. Kendall's tau-b counts tied pairs
    as neither agreeing nor disagreeing, and coverage says how many counters the model reached."""
    if model.sum() <= 0 or len(model) < 5:
        return {"spearman": np.nan, "kendall": np.nan, "pearson": np.nan, "rmse": np.nan,
                "coverage": np.nan}
    m, o = model / model.sum(), observed / observed.sum()
    return {"spearman": m.rank().corr(o.rank()), "kendall": kendall_tau_b(m, o),
            "pearson": m.corr(o), "rmse": float(np.sqrt(((m - o) ** 2).mean())),
            "coverage": float((m > 0).mean())}


def score_run(row, repo: Path, counters: pd.DataFrame) -> list[dict]:
    path = volumes_path(repo, row.run_id)
    if path is None:
        return []
    volumes = pd.read_csv(path)
    merged = counters.merge(volumes, on="edgeID", how="left").fillna(0)
    out = []
    for scenario in row.scenarios.split(","):
        if scenario not in merged or merged[scenario].sum() == 0:
            continue
        record = {"run_id": row.run_id, "scenario": scenario, "counters": len(merged),
                  "zero_counters": int((merged[scenario] == 0).sum())}
        for key, value in metrics(merged[scenario], merged.share).items():
            record[f"all_{key}"] = value
        for block in sorted(merged.block.unique()):
            held = merged[merged.block == block]
            train = merged[merged.block != block]
            for part, rows in (("block", held), ("train", train)):
                scores = metrics(rows[scenario], rows.share)
                for key in ("spearman", "kendall", "coverage"):
                    record[f"{part}_{block}_{key}"] = scores[key]
        out.append(record)
    return out


# -------------------------------------------------------------------------- analysis ----


def morris_effects(design: pd.DataFrame, scores: pd.DataFrame, metric: str) -> pd.DataFrame:
    """Elementary effects of each parameter on `metric`, per scenario: mu* and sigma."""
    merged = scores.merge(design, on="run_id")
    names = [p[0] for p in MORRIS_PARAMETERS]
    rows = []
    for scenario, g in merged.groupby("scenario"):
        effects = {n: [] for n in names}
        for _, traj in g.sort_values("point").groupby("trajectory"):
            traj = traj.reset_index(drop=True)
            for i in range(1, len(traj)):
                name = traj.loc[i, "changed"]
                before = json.loads(traj.loc[i - 1, "params"])[name]
                after = json.loads(traj.loc[i, "params"])[name]
                low, high = next((lo, hi) for n, lo, hi in MORRIS_PARAMETERS if n == name)
                step = (after - before) / (high - low)
                if step != 0:
                    effects[name].append((traj.loc[i, metric] - traj.loc[i - 1, metric]) / step)
        for name, values in effects.items():
            v = np.array(values, dtype=float)
            v = v[~np.isnan(v)]
            rows.append({"scenario": scenario, "parameter": name, "n": len(v),
                         "mu_star": np.abs(v).mean() if len(v) else np.nan,
                         "mu": v.mean() if len(v) else np.nan,
                         "sigma": v.std(ddof=1) if len(v) > 1 else np.nan})
    return pd.DataFrame(rows).sort_values(["scenario", "mu_star"], ascending=[True, False])


def make_blocks(sensors_csv: Path, splits: int = 2) -> pd.DataFrame:
    """Spatial blocks of counters of near-equal size, for held-out validation: the counters are
    split at the median of the wider axis, and each half again, `splits` times, giving 2**splits
    blocks. Clustering would follow the counters into the CBD and leave blocks of a handful."""
    s = pd.read_csv(sensors_csv, sep=";", encoding="utf-8-sig")
    lat0 = np.radians(s.latitude.mean())
    s["x"] = s.longitude * 111320 * np.cos(lat0)
    s["y"] = s.latitude * 110574
    groups = [s.index.to_numpy()]
    for _ in range(splits):
        halves = []
        for idx in groups:
            part = s.loc[idx]
            axis = "x" if np.ptp(part.x) >= np.ptp(part.y) else "y"
            ordered = part.sort_values([axis, "location_id"]).index.to_numpy()
            halves.extend([ordered[: len(ordered) // 2], ordered[len(ordered) // 2:]])
        groups = halves
    block = pd.Series(0, index=s.index)
    for b, idx in enumerate(groups):
        block.loc[idx] = b
    return pd.DataFrame({"location_id": s.location_id, "block": block.values})


# ------------------------------------------------------------------------------- cli ----


def main(argv=None) -> None:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    sub = parser.add_subparsers(dest="cmd", required=True)
    b = sub.add_parser("blocks")
    b.add_argument("--sensors", type=Path, required=True)
    b.add_argument("--splits", type=int, default=2)
    b.add_argument("--out", type=Path, required=True)
    d = sub.add_parser("design")
    d.add_argument("kind", choices=["convergence", "elements", "morris"])
    d.add_argument("--seeds", type=int, default=4)
    d.add_argument("--trips", type=int, default=1000)
    d.add_argument("--trajectories", type=int, default=10)
    d.add_argument("--out", type=Path, required=True)
    r = sub.add_parser("run")
    r.add_argument("design", type=Path)
    r.add_argument("--repo", type=Path, required=True)
    r.add_argument("--parallel", type=int, default=8)
    r.add_argument("--heap", default="8g")
    s = sub.add_parser("score")
    s.add_argument("design", type=Path)
    s.add_argument("--repo", type=Path, required=True)
    s.add_argument("--targets", type=Path, required=True)
    s.add_argument("--link", type=Path, required=True)
    s.add_argument("--blocks", type=Path, required=True)
    s.add_argument("--out", type=Path, required=True)
    s.add_argument("--daytype", default="weekday")
    s.add_argument("--hours", default="7-19")
    m = sub.add_parser("morris")
    m.add_argument("design", type=Path)
    m.add_argument("scores", type=Path)
    m.add_argument("--metric", default="all_spearman")
    m.add_argument("--out", type=Path, required=True)
    args = parser.parse_args(argv)

    if args.cmd == "blocks":
        blocks = make_blocks(args.sensors, args.splits)
        blocks.to_csv(args.out, index=False)
        print(blocks.block.value_counts().sort_index().to_string())
    elif args.cmd == "design":
        design = {"convergence": lambda: design_convergence(args.seeds),
                  "elements": lambda: design_elements(args.seeds, args.trips),
                  "morris": lambda: design_morris(args.trajectories, args.trips)}[args.kind]()
        design.to_csv(args.out, index=False)
        print(f"{len(design)} runs -> {args.out}")
    elif args.cmd == "run":
        run_design(pd.read_csv(args.design), args.repo, args.parallel, args.heap)
    elif args.cmd == "score":
        counters = usable_counters(pd.read_csv(args.targets), pd.read_csv(args.link),
                                   pd.read_csv(args.blocks), args.daytype, args.hours)
        print(f"{len(counters)} counters scored; excluded: "
              + "; ".join(f"{k} ({v.split(':')[0]})" for k, v in EXCLUDED.items()))
        records = []
        for row in pd.read_csv(args.design).itertuples(index=False):
            records.extend(score_run(row, args.repo, counters))
        scores = pd.DataFrame(records)
        scores.to_csv(args.out, index=False)
        print(f"{len(scores)} run-scenario scores -> {args.out}")
    elif args.cmd == "morris":
        effects = morris_effects(pd.read_csv(args.design), pd.read_csv(args.scores), args.metric)
        effects.to_csv(args.out, index=False)
        print(effects.to_string(index=False))


if __name__ == "__main__":
    sys.exit(main())
