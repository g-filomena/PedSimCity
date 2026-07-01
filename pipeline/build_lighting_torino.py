"""PedSimCity street-lighting pipeline - Turin `puntiLuce` inventory.

For cities whose lamp inventory follows the Turin open-data "Punti Luce" schema:
Italian column names (potenza_w_max, altezza_palo_m, tecnologia, uso_ottica, ...)
carrying real per-lamp technical data. Runs step 02 (Turin) then the shared
illumination steps 03 and 04. Every layer is named <City>_… (City = folder name).

For cities with only bare lamp/pole locations, use build_lighting.py instead.
"""

from __future__ import annotations

import argparse
import subprocess
import sys
from pathlib import Path

PIPELINE_DIR = Path(__file__).resolve().parent

# (script, primary output used for the skip check; {city} filled per run)
STEPS = [
    ("02_street_lights_torino.py", "{city}_puntiLuce_with_radius.gpkg"),
    ("03_street_lights.py", "{city}_edges_illuminated_continuous.gpkg"),
    ("04_directional_lighting.py", "{city}_directional_lighting_lookup.csv"),
]


def main() -> None:
    parser = argparse.ArgumentParser(description="Build street-lighting data from a Turin puntiLuce inventory.")
    parser.add_argument("--input_dir", required=True, help="City folder, e.g. src/main/resources/Torino")
    parser.add_argument("--force", action="store_true", help="Re-run steps even if their output exists.")
    args = parser.parse_args()

    input_dir = Path(args.input_dir).resolve()
    if not input_dir.exists():
        raise FileNotFoundError(f"Input directory not found: {input_dir}")
    city = input_dir.name

    for script, output in STEPS:
        out = input_dir / output.format(city=city)
        if out.exists() and not args.force:
            print(f"[SKIP] {script} ({out.name} exists)")
            continue
        print(f"[RUN] {script}")
        subprocess.run([sys.executable, str(PIPELINE_DIR / script), "--input_dir", str(input_dir)], check=True)

    print(f"Done. Lighting data ready in {input_dir}")


if __name__ == "__main__":
    main()
