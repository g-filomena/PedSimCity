"""Quick post-hoc summary of trip_diagnostic.csv (written by the Java sim to the repo root)."""

import csv
from collections import Counter
from pathlib import Path

# Repo root is two levels up (analysis/ -> repo root). The Java engine writes trip_diagnostic.csv
# into its working directory, i.e. the repo root.
csv_path = Path(__file__).resolve().parent.parent / "trip_diagnostic.csv"

rows = []
with open(csv_path) as f:
    reader = csv.DictReader(f)
    for row in reader:
        rows.append(row)

print(f"Total trips: {len(rows)}")

if len(rows) > 0:
    dists = [float(r["distance_m"]) for r in rows]
    durs = [int(r["duration_min"]) for r in rows]
    starts = [r["start_time"] for r in rows]

    print(f"Distance range: {min(dists):.1f} - {max(dists):.1f} m")
    print(f"Distance avg: {sum(dists) / len(dists):.1f} m")
    print(f"Duration range: {min(durs)} - {max(durs)} min")
    print(f"Duration avg: {sum(durs) / len(durs):.1f} min")

    start_counts = Counter(starts)
    print(f"Unique start times: {len(start_counts)}")
    print("Start time distribution (top 10):")
    for t, c in start_counts.most_common(10):
        print(f"  {t}: {c}")

    print("\nSample of first 5 rows (agent_id, start, end, dur, dist):")
    for r in rows[:5]:
        print(f"Agent {r['agent_id']}: {r['start_time']} -> {r['end_time']}, "
              f"{r['duration_min']}m, {float(r['distance_m']):.1f}m")
