"""Designs, runs and scores the night module's sensitivity, calibration and validation series.

    python night_sweep.py design noise|morris|lhs|validate [options] --out design.csv
    python night_sweep.py run design.csv --repo <checkout> --parallel 8 --heap 32g
    python night_sweep.py sets design.csv --out sets.csv          # pair Wednesdays with Sundays
    python night_sweep.py morris scores.csv --out effects.csv

One row of a design is one simulated day. Factors are stated on the scale of src/main/java/pedsim/night/CALIBRATION_VALIDATION.md: each
vulnerable weight, and the non-vulnerable weight as a ratio of it. Runs are resumable: a run whose
edge_volume.csv exists is skipped. Standard library, numpy and pandas only.
"""

from __future__ import annotations

import argparse
import concurrent.futures
import json
import os
import subprocess
import time
from pathlib import Path

import numpy as np
import pandas as pd

CITY = "Torino_sidewalks"
LAUNCHER = "pedsim.night.launcher.NightLauncher"
DATES = {"winter": ("2025-02-12", "2025-02-16"), "spring": ("2025-04-09", "2025-04-13"),
         "summer": ("2025-07-09", "2025-07-13"), "autumn": ("2025-10-29", "2025-11-02")}

# (name, low, high, default)
FACTORS = [
    ("darknessWeight", 0.0, 4.0, 1.0),
    ("parkWaterWeight", 0.0, 4.0, 1.0),
    ("enclosureWeight", 0.0, 4.0, 1.0),
    ("emptyStreetWeight", 0.0, 4.0, 1.0),
    ("darknessRatio", 0.0, 1.0, 0.5),
    ("parkWaterRatio", 0.0, 1.0, 0.5),
    ("enclosureRatio", 0.0, 1.0, 0.5),
    ("emptyStreetRatio", 0.0, 1.0, 0.5),
    ("minReassuranceLux", 5.0, 15.0, 8.9),
    ("maxReassuranceLux", 15.0, 40.0, 26.0),
    ("distanceWeight", 0.0006, 0.0024, 0.0012),
    ("walkShareHalfDistance", 1000.0, 2300.0, 1500.0),
]
NAMES = [f[0] for f in FACTORS]
DEFAULTS = {f[0]: f[3] for f in FACTORS}
TERMS = ["darkness", "parkWater", "enclosure", "emptyStreet"]


def model_arguments(factors: dict) -> list[str]:
    """Factors on the protocol's scale -> the model's command-line parameters."""
    f = {**DEFAULTS, **factors}
    args = []
    for term in TERMS:
        vulnerable = f[f"{term}Weight"]
        args += [f"--{term}WeightVulnerable={vulnerable:.6g}",
                 f"--{term}WeightNonVulnerable={vulnerable * f[f'{term}Ratio']:.6g}"]
    args += [f"--minReassuranceLux={f['minReassuranceLux']:.6g}",
             f"--maxReassuranceLux={f['maxReassuranceLux']:.6g}",
             f"--distanceWeight={f['distanceWeight']:.6g}",
             f"--walkShareHalfDistance={f['walkShareHalfDistance']:.6g}"]
    return args


def row(run_id, season, day, seed, share, factors, **extra) -> dict:
    date = DATES[season][0 if day == "wed" else 1]
    return {"run_id": run_id, "season": season, "day": day, "date": date, "seed": seed,
            "share": share, "factors": json.dumps(factors, sort_keys=True), **extra}


# ------------------------------------------------------------------------------- designs ----

def design_noise(seeds: int, shares: list[float]) -> pd.DataFrame:
    return pd.DataFrame([row(f"noise_p{int(share * 100):02d}_s{s}", "winter", "wed", 20261115 + s, share, {})
                         for share in shares for s in range(seeds)])


def design_morris(trajectories: int, share: float, levels: int = 4) -> pd.DataFrame:
    rng = np.random.default_rng(20261115)
    k = len(FACTORS)
    delta = levels / (2.0 * (levels - 1))
    grid = np.arange(levels) / (levels - 1)
    rows = []
    for t in range(trajectories):
        point = rng.choice(grid[grid <= 1.0 - delta + 1e-9], size=k)
        signs = rng.choice([-1.0, 1.0], size=k)
        points, changed = [point.copy()], [""]
        for j in rng.permutation(k):
            up = (signs[j] > 0 and point[j] + delta <= 1 + 1e-9) or point[j] - delta < -1e-9
            point[j] += delta if up else -delta
            points.append(point.copy())
            changed.append(NAMES[j])
        for i, (x, name) in enumerate(zip(points, changed)):
            factors = {n: round(lo + xi * (hi - lo), 6) for (n, lo, hi, _), xi in zip(FACTORS, x)}
            rows.append(row(f"morris_t{t}_p{i}", "winter", "wed", 20261200 + t, share, factors,
                            trajectory=t, point=i, changed=name))
    return pd.DataFrame(rows)


def design_lhs(points: int, share: float, names: list[str], seasons: list[str]) -> pd.DataFrame:
    """Latin hypercube over `names` (the rest at their defaults), each point run on a Wednesday and
    a Sunday of each season, one seed per point."""
    rng = np.random.default_rng(20261116)
    chosen = [f for f in FACTORS if f[0] in names]
    cube = np.column_stack([(rng.permutation(points) + rng.random(points)) / points for _ in chosen])
    rows = []
    for i in range(points):
        factors = {n: round(lo + cube[i, j] * (hi - lo), 6) for j, (n, lo, hi, _) in enumerate(chosen)}
        for season in seasons:
            for day in ("wed", "sun"):
                rows.append(row(f"lhs_{i:03d}_{season}_{day}", season, day, 20261300 + i, share, factors,
                                point=i))
    return pd.DataFrame(rows)


def design_validate(seeds: int, share: float, calibrated: dict) -> pd.DataFrame:
    rows = []
    for arm, factors in (("default", {}), ("calibrated", calibrated)):
        for season in DATES:
            for day in ("wed", "sun"):
                for s in range(seeds):
                    rows.append(row(f"val_{arm}_{season}_{day}_s{s}", season, day, 20261400 + s, share,
                                    factors, arm=arm))
    return pd.DataFrame(rows)


# ---------------------------------------------------------------------------------- runs ----

def volumes(repo: Path, run_id: str) -> Path:
    return repo / "outputs" / "PedSimCityNight" / run_id / "data" / "edge_volume.csv"


def command(r, repo: Path, heap: str) -> list[str]:
    java = os.environ.get("JAVA", "java")
    cp = f"{repo / 'target' / 'classes'}:{(repo / 'cp.txt').read_text().strip()}"
    return [java, f"-Xmx{heap}", "-cp", cp, LAUNCHER, "--headless", f"--cityName={CITY}",
            f"--percentage={r.share}", "--days=1", "--jobs=1", "--stepDelayMs=0",
            "--exportHtmlDashboard=false", f"--SIMULATION_START_DATE={r.date}", f"--seed={int(r.seed)}",
            f"--outputTag={r.run_id}"] + model_arguments(json.loads(r.factors))


def execute(r, repo: Path, heap: str, logs: Path) -> tuple[str, int, float]:
    if volumes(repo, r.run_id).exists():
        return r.run_id, 0, 0.0
    start = time.time()
    with (logs / f"{r.run_id}.log").open("w") as handle:
        code = subprocess.run(command(r, repo, heap), cwd=repo, stdout=handle,
                              stderr=subprocess.STDOUT).returncode
    return r.run_id, code, time.time() - start


def run_design(design: pd.DataFrame, repo: Path, parallel: int, heap: str, logs: Path) -> None:
    logs.mkdir(parents=True, exist_ok=True)
    rows = list(design.itertuples(index=False))
    with concurrent.futures.ThreadPoolExecutor(max_workers=parallel) as pool:
        futures = [pool.submit(execute, r, repo, heap, logs) for r in rows]
        for n, future in enumerate(concurrent.futures.as_completed(futures), start=1):
            run_id, code, seconds = future.result()
            print(f"{time.strftime('%d %H:%M:%S')} [{n}/{len(rows)}] {run_id} rc={code} {seconds:.0f}s",
                  flush=True)


def sets(design: pd.DataFrame) -> pd.DataFrame:
    """One scoring set per Wednesday, with the Sunday of the same point and season if there is one."""
    out = []
    keys = [c for c in ("point", "trajectory", "arm") if c in design]
    for r in design[design.day == "wed"].itertuples(index=False):
        sunday = design[(design.day == "sun") & (design.season == r.season) & (design.seed == r.seed)
                        & (design.factors == r.factors)]
        out.append({"set_id": r.run_id.replace("_wed", ""), "season": r.season, "wed": r.run_id,
                    "sun": sunday.run_id.iloc[0] if len(sunday) else "", "seed": r.seed,
                    "factors": r.factors, **{k: getattr(r, k) for k in keys},
                    **({"changed": r.changed} if "changed" in design else {})})
    return pd.DataFrame(out)


def morris_effects(scores: pd.DataFrame, metrics: list[str]) -> pd.DataFrame:
    """mu*, mu and sigma of each factor's elementary effects on each metric. `scores` carries the
    trajectory, point, changed factor and factors of each set, as `sets` wrote them."""
    span = {n: hi - lo for n, lo, hi, _ in FACTORS}
    rows = []
    for metric in metrics:
        effects = {n: [] for n in NAMES}
        for _, traj in scores.sort_values("point").groupby("trajectory"):
            traj = traj.reset_index(drop=True)
            for i in range(1, len(traj)):
                name = traj.loc[i, "changed"]
                step = (json.loads(traj.loc[i, "factors"])[name]
                        - json.loads(traj.loc[i - 1, "factors"])[name]) / span[name]
                if step:
                    effects[name].append((traj.loc[i, metric] - traj.loc[i - 1, metric]) / step)
        for name, values in effects.items():
            v = np.array(values, float)
            v = v[~np.isnan(v)]
            rows.append({"metric": metric, "factor": name, "n": len(v),
                         "mu_star": np.abs(v).mean() if len(v) else np.nan,
                         "mu": v.mean() if len(v) else np.nan,
                         "sigma": v.std(ddof=1) if len(v) > 1 else np.nan})
    return pd.DataFrame(rows).sort_values(["metric", "mu_star"], ascending=[True, False])


def main(argv=None) -> None:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    sub = parser.add_subparsers(dest="cmd", required=True)
    d = sub.add_parser("design")
    d.add_argument("kind", choices=["noise", "morris", "lhs", "validate"])
    d.add_argument("--seeds", type=int, default=5)
    d.add_argument("--shares", default="0.05,0.10,0.20")
    d.add_argument("--share", type=float, default=0.10)
    d.add_argument("--trajectories", type=int, default=10)
    d.add_argument("--points", type=int, default=80)
    d.add_argument("--factors", default="", help="comma-separated factor names for lhs")
    d.add_argument("--seasons", default="winter,summer")
    d.add_argument("--calibrated", default="{}", help="JSON of calibrated factors for validate")
    d.add_argument("--out", type=Path, required=True)
    r = sub.add_parser("run")
    r.add_argument("design", type=Path)
    r.add_argument("--repo", type=Path, required=True)
    r.add_argument("--parallel", type=int, default=8)
    r.add_argument("--heap", default="32g")
    r.add_argument("--logs", type=Path, default=Path("logs"))
    s = sub.add_parser("sets")
    s.add_argument("design", type=Path)
    s.add_argument("--out", type=Path, required=True)
    m = sub.add_parser("morris")
    m.add_argument("scores", type=Path)
    m.add_argument("--metrics", default="rho_total_day,rho_total_night,rho_diurnal,women_change_sim,"
                                         "centre_ratio_day_sim,cuebiq_rho_day,dim10_gap,dim5_gap")
    m.add_argument("--out", type=Path, required=True)
    args = parser.parse_args(argv)

    if args.cmd == "design":
        design = {
            "noise": lambda: design_noise(args.seeds, [float(x) for x in args.shares.split(",")]),
            "morris": lambda: design_morris(args.trajectories, args.share),
            "lhs": lambda: design_lhs(args.points, args.share, args.factors.split(","), args.seasons.split(",")),
            "validate": lambda: design_validate(args.seeds, args.share, json.loads(args.calibrated)),
        }[args.kind]()
        design.to_csv(args.out, index=False)
        print(f"{len(design)} runs -> {args.out}")
    elif args.cmd == "run":
        run_design(pd.read_csv(args.design), args.repo, args.parallel, args.heap, args.logs)
    elif args.cmd == "sets":
        out = sets(pd.read_csv(args.design))
        out.to_csv(args.out, index=False)
        print(f"{len(out)} sets -> {args.out}")
    else:
        effects = morris_effects(pd.read_csv(args.scores), args.metrics.split(","))
        effects.to_csv(args.out, index=False)
        print(effects.to_string(index=False))


if __name__ == "__main__":
    main()
