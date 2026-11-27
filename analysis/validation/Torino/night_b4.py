"""Compares the default and the calibrated night model on every season (stage B4).

    python night_b4.py scores_B4.csv --seed-sd scores_seed.csv --out b4/

`scores_B4.csv` is the output of `night_score.py score` on the sets of both arms (columns season, arm
and seed). Writes, per season and arm, the mean and the seed standard deviation of each measure
(`summary.csv`, and `table.tex` for the paper), and the comparisons of PROTOCOL.md's rule
(`rule.csv`): on the held-out seasons and on Cuebiq, the calibrated mean is not worse than the
default's by more than the default's seed standard deviation, and the implausibility of M3 and M4 is
under the cutoff at the calibrated mean.
"""

from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np
import pandas as pd

SEASONS = ["winter", "summer", "spring", "autumn"]
HELD_OUT = ["spring", "autumn"]
CUTOFF, DISCREPANCY = 3.0, 0.10
MATCHED = {"M3 women's change": ("women_change_sim", "women_change_obs", "identity"),
           "M4 centre, day": ("centre_ratio_day_sim", "centre_ratio_day_obs", "log"),
           "M4 centre, night": ("centre_ratio_night_sim", "centre_ratio_night_obs", "log")}
# measure -> (column, whether more is better)
MEASURES = {"M1 rho, day": ("rho_total_day", True), "M1 rho, night": ("rho_total_night", True),
            "M2 rho, diurnal": ("rho_diurnal", True),
            "M3 |error|": ("m3_error", False), "M4 |log error|, day": ("m4_day_error", False),
            "M4 |log error|, night": ("m4_night_error", False),
            "C rho, day": ("cuebiq_rho_day", True), "C rho, night": ("cuebiq_rho_night", True)}
REPORTED = list(MEASURES) + list(MATCHED)


def with_errors(scores: pd.DataFrame) -> pd.DataFrame:
    return scores.assign(
        m3_error=(scores.women_change_sim - scores.women_change_obs).abs(),
        m4_day_error=np.log(scores.centre_ratio_day_sim / scores.centre_ratio_day_obs).abs(),
        m4_night_error=np.log(scores.centre_ratio_night_sim / scores.centre_ratio_night_obs).abs())


def column(name: str) -> str:
    return MEASURES[name][0] if name in MEASURES else MATCHED[name][0]


def summarise(scores: pd.DataFrame) -> pd.DataFrame:
    rows = []
    for (season, arm), g in scores.groupby(["season", "arm"]):
        for name in REPORTED:
            values = g[column(name)]
            if values.notna().any():
                observed = g[MATCHED[name][1]].iloc[0] if name in MATCHED else np.nan
                rows.append({"season": season, "arm": arm, "measure": name, "n": values.count(),
                             "mean": values.mean(), "sd": values.std(ddof=1), "observed": observed,
                             "held_out": season in HELD_OUT or name.startswith("C ")})
    return pd.DataFrame(rows)


def seed_variance(seeds: pd.DataFrame, sim: str, kind: str) -> float:
    """As `night_calibrate.py`: between replicates within a season, averaged over the seasons."""
    values = np.log(seeds[sim]) if kind == "log" else seeds[sim]
    return float(values.groupby(seeds.season).var(ddof=1).mean())


def rule(scores: pd.DataFrame, summary: pd.DataFrame, seeds: pd.DataFrame) -> pd.DataFrame:
    table = summary.set_index(["season", "arm", "measure"])
    rows = []
    for season in SEASONS:
        for name, (_, more_is_better) in MEASURES.items():
            if not (season in HELD_OUT or name.startswith("C ")) or (season, "default", name) not in table.index:
                continue
            default, calibrated = table.loc[(season, "default", name)], table.loc[(season, "calibrated", name)]
            worse = (default["mean"] - calibrated["mean"]) * (1 if more_is_better else -1)
            rows.append({"season": season, "check": "not worse than the default", "measure": name,
                         "default": default["mean"], "calibrated": calibrated["mean"], "worse_by": worse,
                         "limit": default["sd"], "holds": bool(worse <= default["sd"])})
        if season in HELD_OUT:
            g = scores[(scores.season == season) & (scores.arm == "calibrated")]
            for name, (sim, obs, kind) in MATCHED.items():
                f = np.log if kind == "log" else (lambda v: v)
                z, mean = float(f(g[obs].iloc[0])), float(f(g[sim]).mean())
                discrepancy = DISCREPANCY if kind == "log" else DISCREPANCY * abs(z)
                value = abs(mean - z) / np.sqrt(seed_variance(seeds, sim, kind) + discrepancy ** 2)
                rows.append({"season": season, "check": "implausibility", "measure": name, "worse_by": value,
                             "limit": CUTOFF, "holds": bool(value < CUTOFF)})
    return pd.DataFrame(rows)


def latex(summary: pd.DataFrame) -> str:
    table = summary.set_index(["measure", "season", "arm"])
    digits = {"M4 centre, day": 1, "M4 centre, night": 1}
    lines = []
    for name in REPORTED:
        for arm in ("default", "calibrated"):
            cells = []
            for season in SEASONS:
                if (name, season, arm) not in table.index:
                    cells.append("--")
                    continue
                r, d = table.loc[(name, season, arm)], digits.get(name, 3)
                cells.append(f"{r['mean']:.{d}f} ({r['sd']:.{d}f})")
            label = name.replace("rho", r"$\rho$").replace("|error|", r"$|$error$|$").replace(
                "|log error|", r"$|\log$ error$|$")
            lines.append(f"    {label if arm == 'default' else ''} & {arm} & " + " & ".join(cells) + r" \\")
        if name in MATCHED:
            observed = [table.loc[(name, s, "default")]["observed"] for s in SEASONS]
            d = digits.get(name, 3)
            lines.append("     & observed & " + " & ".join(f"{o:.{d}f}" for o in observed) + r" \\")
    head = " & ".join(s + (r"$^{*}$" if s in HELD_OUT else "") for s in SEASONS)
    return ("% generated by night_b4.py; mean (seed sd), * held out\n"
            "\\begin{tabular}{@{}llcccc@{}}\n    \\toprule\n"
            f"    Measure & Model & {head} \\\\\n    \\midrule\n" + "\n".join(lines)
            + "\n    \\bottomrule\n\\end{tabular}\n")


def main(argv=None) -> None:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("scores", type=Path)
    parser.add_argument("--seed-sd", type=Path, required=True)
    parser.add_argument("--out", type=Path, required=True)
    args = parser.parse_args(argv)
    args.out.mkdir(parents=True, exist_ok=True)

    scores = with_errors(pd.read_csv(args.scores))
    counts = scores.groupby(["season", "arm"]).seed.nunique().unstack()
    if counts.shape != (len(SEASONS), 2) or counts.isna().any(axis=None) or counts.nunique().max() != 1:
        raise SystemExit(f"the arms are not complete:\n{counts}")
    summary = summarise(scores)
    checks = rule(scores, summary, pd.read_csv(args.seed_sd))
    summary.to_csv(args.out / "summary.csv", index=False)
    checks.to_csv(args.out / "rule.csv", index=False)
    (args.out / "table.tex").write_text(latex(summary))
    print(checks.round(4).to_string(index=False))
    print(f"holds: {int(checks.holds.sum())} of {len(checks)} checks ->",
          "the rule is met" if checks.holds.all() else "the rule is not met")


if __name__ == "__main__":
    main()
