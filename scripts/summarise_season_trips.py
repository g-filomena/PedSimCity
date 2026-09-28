"""Per-population trip summary for one night run, for the results site.

Reads every ``trip_diagnostic*.csv`` a run wrote (one per job; job 0's has no suffix) and
reduces them to the figures the seasons page shows for each population: legs, mean route length
and mean route lux overall, and legs and mean route lux by the hour the leg set off.

    python summarise_season_trips.py <run>/outputs --season winter --out winter_trips.json

``start_time`` is written as ``DayN hh:mm``. ``--max-day`` drops legs from later days, so a
season can be cut back to the days its siblings share. Standard library only: it runs on gdsl1,
where the data is and numpy is not.
"""

from __future__ import annotations

import argparse
import csv
import json
import re
from pathlib import Path

HOURS = 24
START = re.compile(r"^Day(\d+) (\d{1,2}):(\d{2})$")
GROUPS = ("vulnerable", "nonVulnerable")


def summarise(outputs: Path, max_day: int | None) -> dict:
    files = sorted(outputs.glob("trip_diagnostic*.csv"))
    if not files:
        raise SystemExit(f"no trip_diagnostic*.csv in {outputs}")
    legs = {g: [0] * HOURS for g in GROUPS}
    metres = {g: 0.0 for g in GROUPS}
    lux_sum = {g: [0.0] * HOURS for g in GROUPS}
    lux_legs = {g: [0] * HOURS for g in GROUPS}
    skipped = 0
    for path in files:
        with path.open(newline="") as handle:
            for row in csv.DictReader(handle):
                match = START.match(row["start_time"].strip())
                if not match:
                    skipped += 1
                    continue
                day, hour = int(match.group(1)), int(match.group(2)) % HOURS
                if max_day is not None and day > max_day:
                    continue
                group = "vulnerable" if row["vulnerable"].strip().lower() == "true" else "nonVulnerable"
                legs[group][hour] += 1
                metres[group] += float(row["distance_m"] or 0.0)
                lux = row.get("mean_lux", "").strip()
                if lux and lux.lower() != "nan":
                    lux_sum[group][hour] += float(lux)
                    lux_legs[group][hour] += 1

    total = {g: sum(legs[g]) for g in GROUPS}
    all_legs = sum(total.values())
    return {
        "legs": all_legs,
        "files": len(files),
        "skippedRows": skipped,
        "vulnerableLegShare": total["vulnerable"] / all_legs if all_legs else None,
        "legsBy": total,
        "meanMetres": {g: metres[g] / total[g] if total[g] else None for g in GROUPS},
        "meanLux": {
            g: sum(lux_sum[g]) / sum(lux_legs[g]) if sum(lux_legs[g]) else None for g in GROUPS
        },
        "legsByHour": legs,
        "meanLuxByHour": {
            g: [
                round(lux_sum[g][h] / lux_legs[g][h], 3) if lux_legs[g][h] else None
                for h in range(HOURS)
            ]
            for g in GROUPS
        },
    }


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("outputs", type=Path, help="the run's outputs/ directory")
    parser.add_argument("--season", required=True)
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument("--max-day", type=int, default=None)
    args = parser.parse_args()
    result = summarise(args.outputs, args.max_day)
    result["season"] = args.season
    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(json.dumps(result, separators=(",", ":")))
    print(
        f"{args.season}: {result['legs']} legs from {result['files']} files, vulnerable share "
        f"{result['vulnerableLegShare']:.3f}, {result['skippedRows']} rows skipped -> {args.out}"
    )


if __name__ == "__main__":
    main()
