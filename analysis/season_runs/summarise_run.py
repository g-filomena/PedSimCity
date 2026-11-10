"""Per-run summary of a seasonal night run, for the season tables.

Reads <outputs>/<tag>/trip_diagnostic.csv and the run's daySummary, and writes beside each other
in <out>/<tag>_*.csv. The first simulated day is the season's Monday for a tag of the form
season7_<season>_s<seed>, or the ISO date given as a fourth argument:

    python summarise_run.py <outputs> <out> <tag> [YYYY-MM-DD]

  trips   one row per (day, group, period): legs and summed route metres; period is the model's
          own darkness (Daylight.isDark: before sunrise - 30 min or after sunset + 30 min)
  evening one row per (day, group): legs departing 18:00-24:00, and how many of them after sunset
          (no buffer) and in the model's darkness
  edges   per edgeID, how many legs of each group walked it in each period
  check   the model's legs_dark per day against the count reproduced here
"""
import csv, math, sys
from collections import defaultdict
from datetime import date, timedelta
from pathlib import Path

LAT, LON, BUFFER_H = 45.0708, 7.6690, 0.5
START = {"winter": date(2026, 12, 21), "spring": date(2026, 3, 16),
         "summer": date(2026, 6, 15), "autumn": date(2026, 9, 21)}


def rome_offset(d):
    """Europe/Rome: CET +1, CEST +2 from the last Sunday of March to the last Sunday of October."""
    def last_sunday(m):
        x = date(d.year, m, 31)
        return x - timedelta(days=(x.weekday() + 1) % 7)
    return 2 if last_sunday(3) <= d < last_sunday(10) else 1


def sun(d):
    """Daylight.sunriseSunsetHours, line for line."""
    n = d.timetuple().tm_yday
    decl = math.radians(23.44) * math.sin(2 * math.pi * (284 + n) / 365)
    lat = math.radians(LAT)
    cos_h = (math.sin(math.radians(-0.833)) - math.sin(lat) * math.sin(decl)) / (math.cos(lat) * math.cos(decl))
    half = math.degrees(math.acos(cos_h)) / 15
    b = 2 * math.pi * (n - 81) / 364
    eot = 9.87 * math.sin(2 * b) - 7.53 * math.cos(b) - 1.5 * math.sin(b)
    noon = 12 - (4 * (LON - 15 * rome_offset(d)) + eot) / 60
    return noon - half, noon + half


def main(outputs, out, tag, start=None):
    first_day = date.fromisoformat(start) if start else START[tag.split("_")[1]]
    trips = defaultdict(lambda: [0, 0.0])
    evening = defaultdict(lambda: [0, 0, 0])
    edges = defaultdict(lambda: [0, 0, 0, 0])  # vuln_day, vuln_dark, non_day, non_dark
    dark_by_day = defaultdict(int)
    csv.field_size_limit(10**9)
    with open(Path(outputs) / tag / "trip_diagnostic.csv", newline="") as fh:
        for r in csv.DictReader(fh):
            day_s, clock = r["start_time"].split()
            day = int(day_s[3:])
            h, m = clock.split(":")
            hour = int(h) + int(m) / 60
            rise, sset = sun(first_day + timedelta(days=day - 1))
            dark = hour < rise - BUFFER_H or hour > sset + BUFFER_H
            group = "vulnerable" if r["vulnerable"] == "true" else "nonVulnerable"
            period = "dark" if dark else "day"
            t = trips[(day, group, period)]
            t[0] += 1
            t[1] += float(r["distance_m"])
            if dark:
                dark_by_day[day] += 1
            if hour >= 18:
                e = evening[(day, group)]
                e[0] += 1
                e[1] += hour > sset
                e[2] += dark
            col = (0 if group == "vulnerable" else 2) + (1 if dark else 0)
            for eid in r["edges_walked"].split(";"):
                if eid:
                    edges[int(eid)][col] += 1
    out = Path(out)
    with open(out / f"{tag}_trips.csv", "w", newline="") as fh:
        w = csv.writer(fh); w.writerow(["tag", "day", "group", "period", "legs", "metres"])
        for (d, g, p), (n, mtr) in sorted(trips.items()):
            w.writerow([tag, d, g, p, n, round(mtr, 1)])
    with open(out / f"{tag}_evening.csv", "w", newline="") as fh:
        w = csv.writer(fh); w.writerow(["tag", "day", "group", "legs_18_24", "after_sunset", "model_dark"])
        for (d, g), v in sorted(evening.items()):
            w.writerow([tag, d, g, *v])
    with open(out / f"{tag}_edges.csv", "w", newline="") as fh:
        w = csv.writer(fh); w.writerow(["edgeID", "vuln_day", "vuln_dark", "non_day", "non_dark"])
        for eid in sorted(edges):
            w.writerow([eid, *edges[eid]])
    model = {}
    for f in (Path(outputs) / "PedSimCityNight" / tag / "daySummary").glob("*.csv"):
        for r in csv.DictReader(open(f)):
            model[int(r["day"])] = (int(r["legs"]), int(r["legs_dark"]))
    with open(out / f"{tag}_check.csv", "w", newline="") as fh:
        w = csv.writer(fh); w.writerow(["tag", "day", "model_legs", "model_legs_dark", "diag_legs", "diag_dark"])
        for d in sorted(model):
            n = sum(v[0] for (dd, _, _), v in trips.items() if dd == d)
            w.writerow([tag, d, *model[d], n, dark_by_day[d]])


if __name__ == "__main__":
    main(*sys.argv[1:5])
