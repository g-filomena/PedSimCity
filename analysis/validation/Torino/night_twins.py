"""Runs the A/B twins on Torino_sidewalks: zero, default and calibrated weights, winter and summer (B4).

    python night_twins.py run --repo <checkout> --calibrated calib_B3/calibrated.json --parallel 4
    python night_twins.py gap --repo <checkout> --resources <.../resources/Torino_sidewalks> --out twins.csv

Each run has 4,000 pairs of agents who are one person, vulnerable and not, and the perception error
at 0, so at zero weights both twins walk the same routes. Pairs are agents 2p (vulnerable) and 2p+1;
their legs are matched in order of start, keeping those with the same origin and destination. `gap`
writes one row per run: the legs of each twin, the share of matched legs on the same streets, the
extra metres of the vulnerable twin, and each twin's metres below 5 and 10 lx and along parks or water.
"""

from __future__ import annotations

import argparse
import concurrent.futures
import csv
import json
import os
import subprocess
from collections import defaultdict
from pathlib import Path

import geopandas as gpd
import numpy as np
import pandas as pd

from night_sweep import CITY, LAUNCHER, TERMS, model_arguments

PAIRS, SEEDS = 4000, range(20261501, 20261506)
DATES = {"winter": "2025-12-21", "summer": "2025-06-21"}
ZERO = {f"{t}Weight": 0.0 for t in TERMS}


def arms(calibrated: dict) -> dict[str, dict]:
    """The zero arm keeps the calibrated activity parameters: only the night weights are zero in it."""
    return {"zero": {**calibrated, **ZERO}, "default": {}, "calibrated": calibrated}


def runs(calibrated: dict) -> list[tuple[str, str, int, dict]]:
    return [(f"twins_{arm}_{season}_s{seed}", DATES[season], seed, factors)
            for arm, factors in arms(calibrated).items() for season in DATES for seed in SEEDS]


def diagnostic(repo: Path, tag: str) -> Path:
    return repo / "outputs" / tag / "trip_diagnostic.csv"


def execute(item, repo: Path, heap: str, logs: Path) -> tuple[str, int]:
    tag, date, seed, factors = item
    if diagnostic(repo, tag).exists():
        return tag, 0
    cp = f"{repo / 'target' / 'classes'}:{(repo / 'cp.txt').read_text().strip()}"
    command = [os.environ.get("JAVA", "java"), f"-Xmx{heap}", "-cp", cp, LAUNCHER, "--headless",
               f"--cityName={CITY}", "--enableLightABTesting=true", f"--abTestPairs={PAIRS}", "--days=1",
               "--jobs=1", "--stepDelayMs=0", "--exportHtmlDashboard=false", "--exportRoutes=false",
               "--perceptionErrorSD=0", f"--SIMULATION_START_DATE={date}", f"--seed={seed}",
               f"--outputTag={tag}"] + model_arguments(factors)
    with (logs / f"{tag}.log").open("w") as handle:
        return tag, subprocess.run(command, cwd=repo, stdout=handle, stderr=subprocess.STDOUT).returncode


def flagged(value) -> bool:
    return str(value).strip() not in ("", "[]", "None", "nan", "0")


def exposure_table(resources: Path) -> dict[int, np.ndarray]:
    """Edge -> its metres, in total, below 5 lx, below 10 lx and along parks or water."""
    city = resources.name
    edges = gpd.read_file(resources / f"{city}_edges.gpkg")
    lux = gpd.read_file(resources / f"{city}_edges_illuminated_continuous.gpkg", ignore_geometry=True)
    lux = dict(zip(lux.edgeID.astype(int), lux.mean_lux.astype(float)))
    park_water = np.zeros(len(edges), bool)
    for name in ("w_parks", "a_rivers"):
        if name in edges.columns:
            park_water |= edges[name].map(flagged).to_numpy()
    out = {}
    for edge, metres, along in zip(edges.edgeID.astype(int), edges.length, park_water):
        light = lux.get(edge, np.inf)
        out[edge] = metres * np.array([1.0, light < 5, light < 10, along])
    return out


def gap(path: Path, exposure: dict[int, np.ndarray]) -> dict:
    csv.field_size_limit(10**9)
    legs = defaultdict(list)
    with path.open(newline="") as handle:
        for r in csv.DictReader(handle):
            nodes = r["nodes_walked"].split(";")
            edges = [int(e) for e in r["edges_walked"].split(";") if e]
            legs[int(r["agent_id"])].append((r["start_time"], nodes[0], nodes[-1], edges))
    metres = {side: np.zeros(4) for side in ("v", "n")}
    count = {"v": 0, "n": 0}
    matched, same, extra = 0, 0, 0.0
    measure = lambda edges: sum((exposure[e] for e in edges if e in exposure), np.zeros(4))
    for pair in {a // 2 for a in legs}:
        v, n = sorted(legs.get(2 * pair, [])), sorted(legs.get(2 * pair + 1, []))
        count["v"] += len(v)
        count["n"] += len(n)
        for a, b in zip(v, n):
            if a[1:3] != b[1:3]:
                continue
            matched += 1
            same += a[3] == b[3]
            ma, mb = measure(a[3]), measure(b[3])
            metres["v"] += ma
            metres["n"] += mb
            extra += ma[0] - mb[0]
    row = {"legs_v": count["v"], "legs_n": count["n"], "matched": matched, "same_route": same / max(matched, 1),
           "extra_m": extra / max(matched, 1)}
    for i, name in enumerate(("metres", "lt5", "lt10", "park_water")):
        row[f"{name}_v"], row[f"{name}_n"] = metres["v"][i], metres["n"][i]
    for name in ("lt5", "lt10", "park_water"):
        row[f"{name}_gap_pct"] = 100 * (row[f"{name}_v"] / max(row[f"{name}_n"], 1e-9) - 1)
    return row


def main(argv=None) -> None:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    sub = parser.add_subparsers(dest="cmd", required=True)
    r = sub.add_parser("run")
    r.add_argument("--repo", type=Path, required=True)
    r.add_argument("--calibrated", type=Path, required=True)
    r.add_argument("--parallel", type=int, default=4)
    r.add_argument("--heap", default="16g")
    r.add_argument("--logs", type=Path, default=Path("logs"))
    g = sub.add_parser("gap")
    g.add_argument("--repo", type=Path, required=True)
    g.add_argument("--resources", type=Path, required=True)
    g.add_argument("--out", type=Path, required=True)
    args = parser.parse_args(argv)

    if args.cmd == "run":
        args.logs.mkdir(parents=True, exist_ok=True)
        items = runs(json.loads(args.calibrated.read_text()))
        with concurrent.futures.ThreadPoolExecutor(args.parallel) as pool:
            for tag, code in pool.map(lambda i: execute(i, args.repo, args.heap, args.logs), items):
                print(tag, f"rc={code}", flush=True)
        return
    exposure = exposure_table(args.resources)
    rows = []
    for arm in arms({}):
        for season in DATES:
            for seed in SEEDS:
                path = diagnostic(args.repo, f"twins_{arm}_{season}_s{seed}")
                if path.exists():
                    rows.append({"arm": arm, "season": season, "seed": seed, **gap(path, exposure)})
    table = pd.DataFrame(rows)
    table.to_csv(args.out, index=False)
    print(table.groupby(["arm", "season"])[["same_route", "extra_m", "lt5_gap_pct", "lt10_gap_pct",
                                            "park_water_gap_pct"]].agg(["mean", "sem"]).round(3).to_string())


if __name__ == "__main__":
    main()
