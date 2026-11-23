"""History matching of the night model against Vodafone, on a Latin hypercube of runs (stage B3).

    python night_calibrate.py scores_B3.csv --factors darknessWeight,emptyStreetWeight,... --out calib/

For each season and each matched output it fits a Gaussian-process emulator over the factors (on
[0, 1], Matern 5/2 plus a white-noise term that absorbs the seed variance), checks it by
leave-one-out, and computes the implausibility

    I(x) = |E[f(x)] - z| / sqrt(Var_emulator(x) + Var_seed + Var_discrepancy)

over a large uniform sample of the factor space. Matched outputs, each with its observed value z:
the women's change after dark, and the centre-ring ratio by day and by night (as logs). The
discrepancy is 10% of the observed value. Points whose largest implausibility is under 3 are not
ruled out; among them the calibrated point maximises the emulated mean of the fit measures (cell
rho by day and by night, diurnal rho). Writes the not-ruled-out sample, per-factor ranges, the
emulators' leave-one-out checks and the calibrated point.
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
from sklearn.model_selection import LeaveOneOut

from night_sweep import FACTORS

warnings.filterwarnings("ignore", module="sklearn.gaussian_process")

MATCHED = {  # output -> (simulated column, observed column, transform)
    "women_change": ("women_change_sim", "women_change_obs", "identity"),
    "centre_day": ("centre_ratio_day_sim", "centre_ratio_day_obs", "log"),
    "centre_night": ("centre_ratio_night_sim", "centre_ratio_night_obs", "log"),
}
FIT = ["rho_total_day", "rho_total_night", "rho_diurnal"]
CUTOFF = 3.0
DISCREPANCY = 0.10
SAMPLE = 200_000


def transform(values, kind):
    return np.log(values) if kind == "log" else values


def emulator(x: np.ndarray, y: np.ndarray) -> GaussianProcessRegressor:
    kernel = (ConstantKernel(1.0, (1e-3, 1e3)) * Matern(length_scale=np.full(x.shape[1], 0.5),
                                                         length_scale_bounds=(0.05, 20.0), nu=2.5)
              + WhiteKernel(1e-3, (1e-8, 1e1)))
    gp = GaussianProcessRegressor(kernel, normalize_y=True, n_restarts_optimizer=5, random_state=0)
    return gp.fit(x, y)


def loo_check(x: np.ndarray, y: np.ndarray) -> dict:
    z = []
    for train, test in LeaveOneOut().split(x):
        gp = emulator(x[train], y[train])
        mean, sd = gp.predict(x[test], return_std=True)
        z.append(float((y[test][0] - mean[0]) / sd[0]))
    z = np.array(z)
    return {"loo_mean_z": float(z.mean()), "loo_sd_z": float(z.std()),
            "loo_within_2": float((np.abs(z) < 2).mean())}


def main(argv=None) -> None:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("scores", type=Path)
    parser.add_argument("--factors", required=True)
    parser.add_argument("--seed-sd", type=Path, default=None,
                        help="B1 scores at the chosen share, for the seed variance of each output")
    parser.add_argument("--out", type=Path, required=True)
    args = parser.parse_args(argv)
    args.out.mkdir(parents=True, exist_ok=True)

    names = args.factors.split(",")
    bounds = {n: (lo, hi) for n, lo, hi, _ in FACTORS}
    scores = pd.read_csv(args.scores)
    unit = lambda frame: np.column_stack([(frame[n] - bounds[n][0]) / (bounds[n][1] - bounds[n][0])
                                          for n in names])
    factors = pd.DataFrame([json.loads(f) for f in scores.factors])
    scores = pd.concat([scores.reset_index(drop=True), factors[names]], axis=1)

    seed_var = {}
    if args.seed_sd is not None:
        noise = pd.read_csv(args.seed_sd)
        for key, (sim, _, kind) in MATCHED.items():
            seed_var[key] = float(np.var(transform(noise[sim], kind), ddof=1))
        for key in FIT:
            seed_var[key] = float(np.var(noise[key], ddof=1))

    rng = np.random.default_rng(20261117)
    sample = rng.random((SAMPLE, len(names)))
    worst = np.zeros(SAMPLE)
    fit_mean = np.zeros(SAMPLE)
    checks = []
    for season, g in scores.groupby("season"):
        x = unit(g)
        for key, (sim, obs, kind) in MATCHED.items():
            y = transform(g[sim].to_numpy(), kind)
            z = float(transform(g[obs].iloc[0], kind))
            gp = emulator(x, y)
            mean, sd = gp.predict(sample, return_std=True)
            variance = sd ** 2 + seed_var.get(key, 0.0) + (DISCREPANCY * abs(z)) ** 2
            implausibility = np.abs(mean - z) / np.sqrt(variance)
            worst = np.maximum(worst, implausibility)
            checks.append({"season": season, "output": key, "observed": z, **loo_check(x, y),
                           "kernel": str(gp.kernel_)})
        for key in FIT:
            gp = emulator(x, g[key].to_numpy())
            fit_mean += gp.predict(sample) / (len(FIT) * scores.season.nunique())
            checks.append({"season": season, "output": key, "observed": np.nan,
                           **loo_check(x, g[key].to_numpy()), "kernel": str(gp.kernel_)})

    pd.DataFrame(checks).to_csv(args.out / "emulator_checks.csv", index=False)
    real = pd.DataFrame({n: bounds[n][0] + sample[:, i] * (bounds[n][1] - bounds[n][0])
                         for i, n in enumerate(names)})
    real["max_implausibility"], real["fit_mean"] = worst, fit_mean
    kept = real[real.max_implausibility < CUTOFF]
    kept.to_csv(args.out / "not_ruled_out.csv", index=False)
    ranges = pd.DataFrame({"low": kept[names].min(), "q05": kept[names].quantile(0.05),
                           "median": kept[names].median(), "q95": kept[names].quantile(0.95),
                           "high": kept[names].max(), "prior_low": [bounds[n][0] for n in names],
                           "prior_high": [bounds[n][1] for n in names]})
    ranges.to_csv(args.out / "ranges.csv")
    best = kept.loc[kept.fit_mean.idxmax()] if len(kept) else real.loc[real.max_implausibility.idxmin()]
    calibrated = {n: round(float(best[n]), 6) for n in names}
    (args.out / "calibrated.json").write_text(json.dumps(calibrated, indent=1))
    print(f"not ruled out: {len(kept)} of {SAMPLE} ({len(kept) / SAMPLE:.1%})")
    print(ranges.round(4).to_string())
    print("calibrated:", calibrated, f"max implausibility {best.max_implausibility:.2f}, fit {best.fit_mean:.3f}")


if __name__ == "__main__":
    main()
