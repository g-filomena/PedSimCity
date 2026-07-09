"""
PedSimCity Automated Day vs. Night Comparison Script
Runs the night module on Torino_simplified with 2% census population in both:
1. Daytime mode (shortest path, no lighting penalties)
2. Nighttime mode (lighting avoidance, vulnerable agents seek lit roads)

Calculates and reports key statistics:
- Average route distance (m) & detour %
- Average travel duration (min)
- Average illuminance (mean_lux) experienced
- Disparity between Vulnerable and Non-Vulnerable agents
"""

import os
import sys
import shutil
import subprocess
import csv
from pathlib import Path

# Paths
ROOT_DIR = Path(__file__).resolve().parent
OUTPUTS_DIR = ROOT_DIR / "outputs"
OUTPUTS_DIR.mkdir(exist_ok=True)

DIAGNOSTIC_CSV = OUTPUTS_DIR / "trip_diagnostic.csv"
DAY_CSV = OUTPUTS_DIR / "trip_diagnostic_day.csv"
NIGHT_CSV = OUTPUTS_DIR / "trip_diagnostic_night.csv"
REPORT_TXT = OUTPUTS_DIR / "day_night_comparison_report.txt"

def run_simulation(mode_name: str, day_start_hour: int, night_start_hour: int):
    print(f"\n==========================================================")
    print(f"[{mode_name}] Running PedSimCity Simulation (2% Census Pop)...")
    print(f"==========================================================")
    
    cmd = [
        "mvn", "compile", "exec:java@night",
        f"-Dexec.args=--headless --cityName=Torino_simplified --percentage=0.02 --percentagePopulationAgent=0.02 --enableAB=false --durationDays=1 --DAY_START_HOUR={day_start_hour} --NIGHT_START_HOUR={night_start_hour}"
    ]
    
    # Set MAVEN_OPTS to provide sufficient heap memory (up to 8GB) for 16,931 agents
    env = os.environ.copy()
    env["MAVEN_OPTS"] = "-Xms2g -Xmx8g"
    
    # Run maven command
    result = subprocess.run(cmd, cwd=ROOT_DIR, shell=True, env=env)
    if result.returncode != 0:
        print(f"ERROR: Simulation failed for mode {mode_name} with exit code {result.returncode}")
        sys.exit(1)
        
    if not DIAGNOSTIC_CSV.exists():
        print(f"ERROR: Expected output file {DIAGNOSTIC_CSV} was not created!")
        sys.exit(1)
        
    target_csv = DAY_CSV if mode_name == "DAYTIME" else NIGHT_CSV
    shutil.copy(DIAGNOSTIC_CSV, target_csv)
    print(f"[{mode_name}] Saved results to: {target_csv.name}")

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

def generate_report(day_stats, night_stats):
    if not day_stats or not night_stats:
        print("ERROR: Could not load stats for comparison.")
        return
        
    lines = []
    lines.append("=========================================================================")
    lines.append("           PEDSIMCITY DAY vs. NIGHT COMPARISON REPORT            ")
    lines.append("=========================================================================")
    lines.append(f"City Network       : Torino_simplified")
    lines.append(f"Population Share   : 2% (Census-derived)")
    lines.append(f"Total Trips        : Day = {day_stats['total_trips']} | Night = {night_stats['total_trips']}")
    lines.append(f"Vulnerable Agents  : Day = {day_stats['vuln_count']} | Night = {night_stats['vuln_count']}")
    lines.append("-------------------------------------------------------------------------")
    lines.append(f"{'METRIC':<30} | {'DAY (Baseline)':<15} | {'NIGHT (Lit Mode)':<16} | {'CHANGE (%)':<10}")
    lines.append("-------------------------------------------------------------------------")
    
    def row(label, d_val, n_val, unit=""):
        diff = ((n_val - d_val) / d_val * 100.0) if d_val > 0 else 0.0
        sign = "+" if diff > 0 else ""
        lines.append(f"{label:<30} | {d_val:>11.1f} {unit:<3} | {n_val:>12.1f} {unit:<3} | {sign}{diff:>7.1f}%")

    row("All Agents - Avg Distance", day_stats["all_dist"], night_stats["all_dist"], "m")
    row("Vulnerable - Avg Distance", day_stats["vuln_dist"], night_stats["vuln_dist"], "m")
    row("Normal     - Avg Distance", day_stats["norm_dist"], night_stats["norm_dist"], "m")
    lines.append("-------------------------------------------------------------------------")
    row("All Agents - Avg Duration", day_stats["all_dur"], night_stats["all_dur"], "min")
    row("Vulnerable - Avg Duration", day_stats["vuln_dur"], night_stats["vuln_dur"], "min")
    row("Normal     - Avg Duration", day_stats["norm_dur"], night_stats["norm_dur"], "min")
    lines.append("-------------------------------------------------------------------------")
    row("All Agents - Avg Illuminance", day_stats["all_lux"], night_stats["all_lux"], "lx")
    row("Vulnerable - Avg Illuminance", day_stats["vuln_lux"], night_stats["vuln_lux"], "lx")
    row("Normal     - Avg Illuminance", day_stats["norm_lux"], night_stats["norm_lux"], "lx")
    lines.append("=========================================================================")
    
    # Key insights
    vuln_detour = ((night_stats["vuln_dist"] - day_stats["vuln_dist"]) / day_stats["vuln_dist"] * 100.0) if day_stats["vuln_dist"] > 0 else 0.0
    vuln_lux_gain = ((night_stats["vuln_lux"] - day_stats["vuln_lux"]) / day_stats["vuln_lux"] * 100.0) if day_stats["vuln_lux"] > 0 else 0.0
    sign_detour = "+" if vuln_detour > 0 else ""
    sign_lux = "+" if vuln_lux_gain > 0 else ""
    
    lines.append("\nKEY BEHAVIOURAL INSIGHTS:")
    lines.append(f"1. Safety Detour Tax : Vulnerable pedestrians walk an average of {sign_detour}{vuln_detour:.1f}% farther at night")
    lines.append(f"                       to remain on well-lit corridors and avoid dark/park areas.")
    lines.append(f"2. Lighting Seeking  : At night, vulnerable pedestrians experienced {sign_lux}{vuln_lux_gain:.1f}% change in average")
    lines.append(f"                       route illuminance compared to daytime shortest paths.")
    lines.append("=========================================================================\n")
    
    report_text = "\n".join(lines)
    print(report_text)
    
    with open(REPORT_TXT, "w", encoding="utf-8") as f:
        f.write(report_text)
    print(f"Report saved to: {REPORT_TXT}")

def main():
    print("Starting Day vs Night Comparison Pipeline...")
    
    # 1. Run Daytime Baseline (DAY_START_HOUR=0, NIGHT_START_HOUR=24 => isDark is always false)
    run_simulation("DAYTIME", day_start_hour=0, night_start_hour=24)
    
    # 2. Run Nighttime Lit Environment (DAY_START_HOUR=24, NIGHT_START_HOUR=0 => isDark is always true)
    run_simulation("NIGHTTIME", day_start_hour=24, night_start_hour=0)
    
    # 3. Analyze and compare
    print("\nAnalyzing trip results...")
    day_stats = analyze_csv(DAY_CSV)
    night_stats = analyze_csv(NIGHT_CSV)
    
    generate_report(day_stats, night_stats)

if __name__ == "__main__":
    main()
