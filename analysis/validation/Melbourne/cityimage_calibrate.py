"""Calibrate one cityImage scenario on a Latin hypercube of runs, leaving one spatial block out at
a time (stage A4).

    python cityimage_calibrate.py design_A4.csv scores_A4.csv --scenario LANDMARKS_DISTANCE \\
        --parameters salientNodesPercentile,perceptionErrorSD --out calib/

For each fold - every spatial block in turn, and "all" - a Gaussian-process emulator (Matern 5/2
plus white noise, on [0, 1]) is fitted to the fold's training score (tau-b over the counters
outside the held-out block, every replicate as its own observation), and the calibrated point is
the emulator's maximum over a uniform sample of the space. The fold's held-out score is never
used to choose it. Writes, per scenario, the points into `calibrated.json` (merged with any
scenario already there), the emulators' 10-fold cross-validation, and the emulated training and
held-out score at each fold's point beside the defaults' measured scores from A2.

Sobol indices come from the "all" emulator's mean, not from runs: first-order (Saltelli 2010) and
total (Jansen 1999) indices of the expected score over the hypercube, on 2^13 base samples, with a
5-95% interval from refitting the emulator on design points resampled with replacement. They
decompose the variance of the score's expectation; the seed noise the white-noise term absorbs is
reported beside them as `noise_share`, the share of the score's variance across runs it accounts
for. Written to `sobol.csv`.
"""

from __future__ import annotations

import argparse
import json
import warnings
from pathlib import Path

import numpy as np
import pandas as pd
from sklearn.gaussian_process import GaussianProcessRegressor
from sklearn.gaussian_process.kernels import ConstantKernel, Matern, WhiteKernel
from sklearn.model_selection import KFold

from cityimage_sweep import parameter_bounds

warnings.filterwarnings("ignore", module="sklearn.gaussian_process")
SAMPLE = 100_000


def emulator(x: np.ndarray, y: np.ndarray) -> GaussianProcessRegressor:
    kernel = (ConstantKernel(1.0, (1e-3, 1e3)) * Matern(length_scale=np.full(x.shape[1], 0.5),
                                                         length_scale_bounds=(0.05, 20.0), nu=2.5)
              + WhiteKernel(1e-3, (1e-8, 1e1)))
    gp = GaussianProcessRegressor(kernel, normalize_y=True, n_restarts_optimizer=5, random_state=0)
    return gp.fit(x, y)


def cross_validate(x: np.ndarray, y: np.ndarray) -> dict:
    """10-fold, by design point, so a point's replicates are never split between train and test."""
    points = np.unique(x, axis=0)
    z, err = [], []
    for train, test in KFold(10, shuffle=True, random_state=0).split(points):
        held = (x[:, None, :] == points[test][None, :, :]).all(axis=2).any(axis=1)
        gp = emulator(x[~held], y[~held])
        mean, sd = gp.predict(x[held], return_std=True)
        z.extend((y[held] - mean) / sd)
        err.extend(y[held] - mean)
    z, err = np.array(z), np.array(err)
    return {"cv_rmse": float(np.sqrt((err ** 2).mean())), "cv_sd_z": float(z.std()),
            "cv_within_2": float((np.abs(z) < 2).mean()), "score_sd": float(y.std())}


def sobol_indices(gp: GaussianProcessRegressor, k: int, rng: np.random.Generator,
                  n: int = 8192) -> tuple[np.ndarray, np.ndarray]:
    """First-order and total Sobol indices of the emulator's mean over [0, 1]^k."""
    a, b = rng.random((n, k)), rng.random((n, k))
    fa, fb = gp.predict(a), gp.predict(b)
    var = np.var(np.concatenate([fa, fb]))
    first, total = np.empty(k), np.empty(k)
    for i in range(k):
        ab = a.copy()
        ab[:, i] = b[:, i]
        fab = gp.predict(ab)
        first[i] = np.mean(fb * (fab - fa)) / var
        total[i] = 0.5 * np.mean((fa - fab) ** 2) / var
    return first, total


def sobol_report(x: np.ndarray, y: np.ndarray, names: list[str], scenario: str,
                 boots: int) -> pd.DataFrame:
    """Sobol indices at the full fit, with an interval over emulators refitted on design points
    resampled with replacement (a point's replicates travel together)."""
    rng = np.random.default_rng(20261120)
    gp = emulator(x, y)
    first, total = sobol_indices(gp, len(names), rng)
    noise = float(gp.kernel_.k2.noise_level) / float(np.var(y)) if np.var(y) > 0 else np.nan
    points, index = np.unique(x, axis=0, return_inverse=True)
    index = index.ravel()
    draws = []
    for _ in range(boots):
        chosen = rng.integers(0, len(points), len(points))
        rows = np.concatenate([np.flatnonzero(index == c) for c in chosen])
        draws.append(np.concatenate(sobol_indices(emulator(x[rows], y[rows]), len(names), rng)))
    draws = np.array(draws) if draws else np.full((1, 2 * len(names)), np.nan)
    lo, hi = np.nanquantile(draws, 0.05, axis=0), np.nanquantile(draws, 0.95, axis=0)
    k = len(names)
    return pd.DataFrame({"scenario": scenario, "parameter": names,
                         "S1": first, "S1_q05": lo[:k], "S1_q95": hi[:k],
                         "ST": total, "ST_q05": lo[k:], "ST_q95": hi[k:],
                         "noise_share": noise})


def main(argv=None) -> None:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("design", type=Path)
    parser.add_argument("scores", type=Path)
    parser.add_argument("--scenario", required=True)
    parser.add_argument("--parameters", required=True)
    parser.add_argument("--defaults", type=Path, default=None,
                        help="A2 design and scores as design.csv,scores.csv: the defaults' scores")
    parser.add_argument("--sobol-boot", type=int, default=20,
                        help="emulator refits for the Sobol intervals")
    parser.add_argument("--out", type=Path, required=True)
    args = parser.parse_args(argv)
    args.out.mkdir(parents=True, exist_ok=True)

    names = args.parameters.split(",")
    bounds = parameter_bounds(names)
    design = pd.read_csv(args.design)
    scores = pd.read_csv(args.scores).merge(design[["run_id", "params"]], on="run_id")
    scores = scores[scores.scenario == args.scenario].reset_index(drop=True)
    params = pd.DataFrame([json.loads(p) for p in scores.params])
    x = np.column_stack([(params[n] - lo) / (hi - lo) for n, (lo, hi) in bounds.items()])
    blocks = sorted({int(c.split("_")[1]) for c in scores if c.startswith("block_")
                     and c.endswith("_kendall")})

    folds = [(str(b), f"train_{b}_kendall", f"block_{b}_kendall") for b in blocks]
    folds.append(("all", "all_kendall", None))
    rng = np.random.default_rng(20261119)
    sample = rng.random((SAMPLE, len(names)))

    default = {}
    if args.defaults is not None:
        dd, ds = (pd.read_csv(p) for p in str(args.defaults).split(","))
        ds = ds.merge(dd[["run_id", "weighted"]], on="run_id")
        ds = ds[(ds.scenario == args.scenario) & ds.weighted.astype(str).str.lower().eq("true")]
        default = ds.mean(numeric_only=True).to_dict()

    points, report = {}, []
    for fold, fit_col, held_col in folds:
        ok = scores[fit_col].notna().to_numpy()
        gp = emulator(x[ok], scores.loc[ok, fit_col].to_numpy())
        mean = gp.predict(sample)
        best = sample[mean.argmax()]
        points[fold] = {n: round(float(lo + best[i] * (hi - lo)), 4)
                        for i, (n, (lo, hi)) in enumerate(bounds.items())}
        # Near-optimal: within one seed SD of the best. A parameter the counters do not identify
        # spans most of its range here, whatever its calibrated value.
        seed_sd = float(scores[ok].groupby("params")[fit_col].std().mean())
        near = sample[mean >= mean.max() - seed_sd]
        ranges = {}
        for i, (n, (lo, hi)) in enumerate(bounds.items()):
            q = np.quantile(near[:, i], [0.05, 0.95])
            ranges[f"{n}_q05"], ranges[f"{n}_q95"] = (round(float(lo + v * (hi - lo)), 4) for v in q)
        row = {"scenario": args.scenario, "fold": fold, **points[fold], **ranges,
               "seed_sd": seed_sd, "near_optimal_share": len(near) / SAMPLE,
               "fit": fit_col, "fit_emulated": float(mean.max()),
               "fit_default": default.get(fit_col, np.nan), "kernel": str(gp.kernel_),
               **cross_validate(x[ok], scores.loc[ok, fit_col].to_numpy())}
        if held_col is not None:
            ok_h = scores[held_col].notna().to_numpy()
            held = emulator(x[ok_h], scores.loc[ok_h, held_col].to_numpy())
            row["held_emulated"] = float(held.predict(best[None, :])[0])
            row["held_default"] = default.get(held_col, np.nan)
        report.append(row)
        print(f"{args.scenario} fold {fold}: {points[fold]}  fit {row['fit_emulated']:.3f}"
              f" (default {row['fit_default']:.3f})"
              + (f"  held out {row['held_emulated']:.3f} (default {row['held_default']:.3f})"
                 if held_col else ""))

    ok = scores["all_kendall"].notna().to_numpy()
    sobol = sobol_report(x[ok], scores.loc[ok, "all_kendall"].to_numpy(), names, args.scenario,
                         args.sobol_boot)
    print(sobol.round(3).to_string(index=False))
    table = args.out / "sobol.csv"
    old = pd.read_csv(table) if table.exists() else pd.DataFrame()
    if len(old):
        old = old[old.scenario != args.scenario]
    pd.concat([old, sobol], ignore_index=True).to_csv(table, index=False)

    path = args.out / "calibrated.json"
    calibrated = json.loads(path.read_text()) if path.exists() else {}
    calibrated[args.scenario] = points
    path.write_text(json.dumps(calibrated, indent=1))
    table = args.out / "calibration_report.csv"
    old = pd.read_csv(table) if table.exists() else pd.DataFrame()
    if len(old):
        old = old[old.scenario != args.scenario]
    pd.concat([old, pd.DataFrame(report)], ignore_index=True).to_csv(table, index=False)


if __name__ == "__main__":
    main()
