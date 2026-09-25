"""Reduce a night run's per-day street volumes to one compact file per season.

A run writes ``streetVolumes/<date>_<job>_<day>.csv``, one row per edge and one column per
hour and population, for every day of every job: 7.4 MB x 21 files for a 7-day, 3-job season.
That is far too much to serve, and almost all of it is zero - roughly 19,800 of Torino's
44,278 edges carry any flow on a given day.

This sums the hourly columns over every day-job of one season, drops the edges that carry
nothing, and writes a single JSON file holding flat integer arrays. Gzipped by the web server
it lands in the low hundreds of kB.

    python aggregate_season_volumes.py <run_dir> --season autumn --out autumn.json

``run_dir`` is the directory holding ``streetVolumes/`` and ``daySummary/`` - for a night run,
``<run>/outputs/PedSimCityNight``.

Standard library only: gdsl1 has no numpy, and this has to run where the data is.
"""

from __future__ import annotations

import argparse
import csv
import json
import re
import sys
from collections import defaultdict
from pathlib import Path

HOURS = 24
# Column names are matched rather than positional: the exporter writes LIGHT/DARK splits and
# totals alongside the hourly columns, and a module may add its own.
VULN_HOUR = re.compile(r"^VULNERABLE_h(\d+)$")
NONVULN_HOUR = re.compile(r"^NON_VULNERABLE_h(\d+)$")
# <date>_<job>_<day>.csv
VOLUME_NAME = re.compile(r"^(?P<date>\d+)_(?P<job>\d+)_(?P<day>\d+)\.csv$")


def hour_columns(header: list[str]) -> tuple[dict[int, int], dict[int, int]]:
    """Maps hour (0-23) to column index, for each population."""
    vuln: dict[int, int] = {}
    nonvuln: dict[int, int] = {}
    for idx, name in enumerate(header):
        match = VULN_HOUR.match(name)
        if match:
            vuln[int(match.group(1)) - 1] = idx
            continue
        match = NONVULN_HOUR.match(name)
        if match:
            nonvuln[int(match.group(1)) - 1] = idx
    return vuln, nonvuln


def read_day_summary(run_dir: Path, max_day: int | None = None) -> list[dict[str, str]]:
    """Every row of every daySummary file, in file order, up to ``max_day``."""
    rows: list[dict[str, str]] = []
    for path in sorted((run_dir / "daySummary").glob("*.csv")):
        with path.open(newline="") as handle:
            for row in csv.DictReader(handle):
                # A concatenated file repeats its header; skip those rows.
                if not row.get("day") or row["day"] == "day":
                    continue
                if max_day is not None and int(row["day"]) > max_day:
                    continue
                rows.append(row)
    return rows


def aggregate(run_dir: Path, max_day: int | None = None) -> dict:
    volume_dir = run_dir / "streetVolumes"
    files = sorted(p for p in volume_dir.glob("*.csv") if VOLUME_NAME.match(p.name))
    if max_day is not None:
        files = [p for p in files if int(VOLUME_NAME.match(p.name).group("day")) <= max_day]
    if not files:
        sys.exit(f"no street volume files under {volume_dir}")

    vuln_totals: dict[int, list[int]] = defaultdict(lambda: [0] * HOURS)
    nonvuln_totals: dict[int, list[int]] = defaultdict(lambda: [0] * HOURS)
    jobs: set[str] = set()
    days: set[str] = set()

    for path in files:
        meta = VOLUME_NAME.match(path.name)
        jobs.add(meta.group("job"))
        days.add(meta.group("day"))
        with path.open(newline="") as handle:
            reader = csv.reader(handle)
            header = next(reader)
            vuln_cols, nonvuln_cols = hour_columns(header)
            if not vuln_cols or not nonvuln_cols:
                sys.exit(f"{path.name}: no hourly columns found")
            for row in reader:
                edge_id = int(row[0])
                vuln_row = None
                nonvuln_row = None
                for hour, col in vuln_cols.items():
                    value = int(row[col])
                    if value:
                        if vuln_row is None:
                            vuln_row = vuln_totals[edge_id]
                        vuln_row[hour] += value
                for hour, col in nonvuln_cols.items():
                    value = int(row[col])
                    if value:
                        if nonvuln_row is None:
                            nonvuln_row = nonvuln_totals[edge_id]
                        nonvuln_row[hour] += value

    edges = sorted(set(vuln_totals) | set(nonvuln_totals))
    zero = [0] * HOURS
    vuln_flat: list[int] = []
    nonvuln_flat: list[int] = []
    for edge_id in edges:
        vuln_flat.extend(vuln_totals.get(edge_id, zero))
        nonvuln_flat.extend(nonvuln_totals.get(edge_id, zero))

    vuln_sum = sum(vuln_flat)
    nonvuln_sum = sum(nonvuln_flat)
    total = vuln_sum + nonvuln_sum

    return {
        "edges": edges,
        "vulnerable": vuln_flat,
        "nonVulnerable": nonvuln_flat,
        "dayJobs": len(files),
        "jobs": sorted(jobs),
        "days": len(days),
        "maxDay": max_day,
        "hours": HOURS,
        # The city-wide share is what a per-edge vulnerable share has to be read against: the
        # population is not evenly split, so parity is not the neutral point.
        "vulnerableShare": vuln_sum / total if total else 0.0,
        "traversals": {"vulnerable": vuln_sum, "nonVulnerable": nonvuln_sum},
    }


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("run_dir", type=Path, help="directory holding streetVolumes/ and daySummary/")
    parser.add_argument("--season", required=True, help="label for this run, e.g. autumn")
    parser.add_argument("--out", type=Path, required=True, help="JSON file to write")
    parser.add_argument(
        "--max-day",
        type=int,
        default=None,
        help="ignore days after this one. Seasons are compared on absolute traversals, so a "
        "run that got further than its siblings has to be cut back to the days they all share.",
    )
    args = parser.parse_args()

    result = aggregate(args.run_dir, args.max_day)
    summary = read_day_summary(args.run_dir, args.max_day)
    result["season"] = args.season
    result["dates"] = sorted({row["date"] for row in summary if row.get("date")})
    result["daySummary"] = summary

    args.out.parent.mkdir(parents=True, exist_ok=True)
    with args.out.open("w") as handle:
        json.dump(result, handle, separators=(",", ":"))

    size_mb = args.out.stat().st_size / 1e6
    print(
        f"{args.season}: {len(result['edges'])} edges with flow, "
        f"{result['dayJobs']} day-jobs, "
        f"{result['traversals']['vulnerable'] + result['traversals']['nonVulnerable']} traversals, "
        f"vulnerable share {result['vulnerableShare']:.3f} -> {args.out} ({size_mb:.1f} MB)"
    )


if __name__ == "__main__":
    main()
