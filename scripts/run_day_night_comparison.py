"""
PedSimCity paired night-module comparison.

Runs the night module twice on one city and diffs the trip diagnostics:

- Average route distance (m) and detour %
- Average travel duration (min)
- Average illuminance (mean_lux) experienced
- Disparity between vulnerable and non-vulnerable agents

**What the two arms are is an argument now, because the old ones stopped existing.** This script
used to switch "day mode" and "night mode" with `--DAY_START_HOUR` and `--NIGHT_START_HOUR`, plus
`--enableAB`. None of those three is a parameter any more:

- `TimePars.DAY_START_HOUR` / `NIGHT_START_HOUR` are constants nothing branches on. Darkness is
  `Daylight.isDark(time)`, from the date and the city's measured latitude, and there is no switch.
- The A/B flag is `enableLightABTesting`; `enableAB` never matched a field.

`ParameterManager` ignores an unknown key silently, so the old invocation ran **two identical
simulations** and reported the difference between them as a day/night effect. The arms are stated
explicitly now and go straight onto the command line.

The default baseline is `--maxKnownDarkEdgeCostMultiplier=1.0`, which `CLAUDE.md` and
`night/TODO.md` both name as the control for the planning-cost mechanism - 1.0 is the behaviour
before darkness reached route planning. Pass your own arms if you are isolating something else.

`--city` names the folder under `src/main/resources/`; no city is named in this file.
"""

import argparse
import os
import sys
import shutil
import subprocess
import csv
from pathlib import Path

# Paths
# This script lives in scripts/; the repo root is one level up.
ROOT_DIR = Path(__file__).resolve().parent.parent
OUTPUTS_DIR = ROOT_DIR / "outputs"
OUTPUTS_DIR.mkdir(exist_ok=True)

DIAGNOSTIC_CSV = OUTPUTS_DIR / "trip_diagnostic.csv"
BASELINE_CSV = OUTPUTS_DIR / "trip_diagnostic_baseline.csv"
TREATMENT_CSV = OUTPUTS_DIR / "trip_diagnostic_treatment.csv"
REPORT_TXT = OUTPUTS_DIR / "night_comparison_report.txt"

# Filled by main() from the command line, and read by the report so it always states its own arms.
CONFIG = {"city": "", "percentage": 0.0, "days": 0, "baseline_args": "", "treatment_args": ""}

def run_simulation(arm: str, target_csv: Path, extra_args: str):
    print("\n==========================================================")
    print(f"[{arm}] night module on {CONFIG['city']} "
          f"({CONFIG['percentage']:.1%} of census, {CONFIG['days']} day(s))")
    print(f"       arm args: {extra_args or '(model defaults)'}")
    print("==========================================================")

    exec_args = (
        f"--headless --cityName={CONFIG['city']} --percentage={CONFIG['percentage']}"
        f" --days={CONFIG['days']}"
    )
    if extra_args:
        exec_args += " " + extra_args
    cmd = ["mvn", "compile", "exec:java@night", f"-Dexec.args={exec_args}"]

    # Set MAVEN_OPTS to provide sufficient heap memory for a large agent count.
    env = os.environ.copy()
    env["MAVEN_OPTS"] = "-Xms2g -Xmx8g"

    result = subprocess.run(cmd, cwd=ROOT_DIR, shell=True, env=env)
    if result.returncode != 0:
        print(f"ERROR: simulation failed for arm {arm} with exit code {result.returncode}")
        sys.exit(1)

    if not DIAGNOSTIC_CSV.exists():
        print(f"ERROR: expected output file {DIAGNOSTIC_CSV} was not created!")
        sys.exit(1)

    shutil.copy(DIAGNOSTIC_CSV, target_csv)
    print(f"[{arm}] saved results to: {target_csv.name}")

def analyze_csv(filepath: Path):
    if not filepath.exists():
        return None
    
    trips = []
    with open(filepath, mode="r", encoding="utf-8") as f:
        reader = csv.DictReader(f)
        for row in reader:
            trips.append({
                "agent_id": int(row["agent_id"]),
                "duration_min": float(row["duration_min"]),
                "distance_m": float(row["distance_m"]),
                "vulnerable": row["vulnerable"].strip().lower() == "true",
                "mean_lux": float(row.get("mean_lux", 0.0))
            })
            
    if not trips:
        return None
        
    vuln_trips = [t for t in trips if t["vulnerable"]]
    norm_trips = [t for t in trips if not t["vulnerable"]]
    
    def avg(lst, key):
        return sum(t[key] for t in lst) / len(lst) if lst else 0.0
        
    return {
        "total_trips": len(trips),
        "vuln_count": len(vuln_trips),
        "norm_count": len(norm_trips),
        "vuln_dist": avg(vuln_trips, "distance_m"),
        "norm_dist": avg(norm_trips, "distance_m"),
        "vuln_dur": avg(vuln_trips, "duration_min"),
        "norm_dur": avg(norm_trips, "duration_min"),
        "vuln_lux": avg(vuln_trips, "mean_lux"),
        "norm_lux": avg(norm_trips, "mean_lux"),
        "all_dist": avg(trips, "distance_m"),
        "all_dur": avg(trips, "duration_min"),
        "all_lux": avg(trips, "mean_lux")
    }

def generate_report(baseline_stats, treatment_stats):
    if not baseline_stats or not treatment_stats:
        print("ERROR: Could not load stats for comparison.")
        return
        
    lines = []
    lines.append("=========================================================================")
    lines.append("           PEDSIMCITY PAIRED NIGHT-MODULE COMPARISON            ")
    lines.append("=========================================================================")
    lines.append(f"City Network       : {CONFIG['city']}")
    lines.append(f"Population Share   : {CONFIG['percentage']:.1%} (Census-derived)")
    lines.append(f"Baseline arm       : {CONFIG['baseline_args'] or '(model defaults)'}")
    lines.append(f"Treatment arm      : {CONFIG['treatment_args'] or '(model defaults)'}")
    lines.append(f"Total Trips        : Baseline = {baseline_stats['total_trips']} | Treatment = {treatment_stats['total_trips']}")
    lines.append(f"Vulnerable Agents  : Baseline = {baseline_stats['vuln_count']} | Treatment = {treatment_stats['vuln_count']}")
    lines.append("-------------------------------------------------------------------------")
    lines.append(f"{'METRIC':<30} | {'BASELINE':<15} | {'TREATMENT':<16} | {'CHANGE (%)':<10}")
    lines.append("-------------------------------------------------------------------------")
    
    def row(label, d_val, n_val, unit=""):
        diff = ((n_val - d_val) / d_val * 100.0) if d_val > 0 else 0.0
        sign = "+" if diff > 0 else ""
        lines.append(f"{label:<30} | {d_val:>11.1f} {unit:<3} | {n_val:>12.1f} {unit:<3} | {sign}{diff:>7.1f}%")

    row("All Agents - Avg Distance", baseline_stats["all_dist"], treatment_stats["all_dist"], "m")
    row("Vulnerable - Avg Distance", baseline_stats["vuln_dist"], treatment_stats["vuln_dist"], "m")
    row("Normal     - Avg Distance", baseline_stats["norm_dist"], treatment_stats["norm_dist"], "m")
    lines.append("-------------------------------------------------------------------------")
    row("All Agents - Avg Duration", baseline_stats["all_dur"], treatment_stats["all_dur"], "min")
    row("Vulnerable - Avg Duration", baseline_stats["vuln_dur"], treatment_stats["vuln_dur"], "min")
    row("Normal     - Avg Duration", baseline_stats["norm_dur"], treatment_stats["norm_dur"], "min")
    lines.append("-------------------------------------------------------------------------")
    row("All Agents - Avg Illuminance", baseline_stats["all_lux"], treatment_stats["all_lux"], "lx")
    row("Vulnerable - Avg Illuminance", baseline_stats["vuln_lux"], treatment_stats["vuln_lux"], "lx")
    row("Normal     - Avg Illuminance", baseline_stats["norm_lux"], treatment_stats["norm_lux"], "lx")
    lines.append("=========================================================================")
    
    # Key insights
    vuln_detour = ((treatment_stats["vuln_dist"] - baseline_stats["vuln_dist"]) / baseline_stats["vuln_dist"] * 100.0) if baseline_stats["vuln_dist"] > 0 else 0.0
    vuln_lux_gain = ((treatment_stats["vuln_lux"] - baseline_stats["vuln_lux"]) / baseline_stats["vuln_lux"] * 100.0) if baseline_stats["vuln_lux"] > 0 else 0.0
    sign_detour = "+" if vuln_detour > 0 else ""
    sign_lux = "+" if vuln_lux_gain > 0 else ""
    
    lines.append("")
    lines.append("VULNERABLE AGENTS, TREATMENT AGAINST BASELINE:")
    lines.append(f"1. Distance  : {sign_detour}{vuln_detour:.1f}% on average.")
    lines.append(f"2. Route lux : {sign_lux}{vuln_lux_gain:.1f}% on average.")
    lines.append("")
    lines.append("What either number MEANS depends on what the two arms were; this script does")
    lines.append("not know and does not guess. Read them against the arm arguments printed")
    lines.append("above, and against a replicate: on a one-day night run the replicate sd is")
    lines.append("about 6.5% of planned metres, so a smaller difference is not a result.")
    lines.append("=========================================================================\n")
    
    report_text = "\n".join(lines)
    print(report_text)
    
    with open(REPORT_TXT, "w", encoding="utf-8") as f:
        f.write(report_text)
    print(f"Report saved to: {REPORT_TXT}")

def main():
    parser = argparse.ArgumentParser(
        description="Run the night module twice on one city and diff the trip diagnostics."
    )
    parser.add_argument("--city", required=True,
                        help="City name: folder under src/main/resources/ and the <City>_* prefix.")
    parser.add_argument("--percentage", type=float, default=0.02,
                        help="Share of the census resident total to release as agents.")
    parser.add_argument("--days", type=int, default=1, help="Simulated days per arm.")
    parser.add_argument("--baseline-args", default="--maxKnownDarkEdgeCostMultiplier=1.0",
                        help="Extra command-line args for the baseline arm. The default is the "
                             "documented control for the planning-cost mechanism; pass an empty "
                             "string to run the model's defaults on both arms.")
    parser.add_argument("--treatment-args", default="",
                        help="Extra command-line args for the treatment arm (default: none, i.e. "
                             "the model's own defaults).")
    args = parser.parse_args()

    CONFIG.update(
        city=args.city,
        percentage=args.percentage,
        days=args.days,
        baseline_args=args.baseline_args.strip(),
        treatment_args=args.treatment_args.strip(),
    )

    if CONFIG["baseline_args"] == CONFIG["treatment_args"]:
        print("REFUSING: both arms carry the same arguments, so this would compare a run with "
              "itself and report the seed noise as an effect. Give --baseline-args and "
              "--treatment-args that differ.")
        sys.exit(2)

    print(f"Starting paired night comparison on {CONFIG['city']}...")
    run_simulation("BASELINE", BASELINE_CSV, CONFIG["baseline_args"])
    run_simulation("TREATMENT", TREATMENT_CSV, CONFIG["treatment_args"])

    print("\nAnalyzing trip results...")
    generate_report(analyze_csv(BASELINE_CSV), analyze_csv(TREATMENT_CSV))


if __name__ == "__main__":
    main()
