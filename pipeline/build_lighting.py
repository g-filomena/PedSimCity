"""PedSimCity street-lighting pipeline.

Runs step 02 — picking the adapter by the lamp inventory the city provides: the Turin
`puntiLuce` adapter (rich per-lamp attributes) when <City>_puntiLuce.gpkg exists, the
generic adapter (assumed physics on bare points) when <City>_streetlights.gpkg does —
then the shared illumination steps 03 and 04. Raw inputs are read from
inputData/<City>/, sim-read outputs land in src/main/resources/<City>/.

`--city` is the city name (folder under inputData/ and resources/, and file prefix).
"""

from __future__ import annotations

import argparse
import subprocess
import sys
from pathlib import Path

import paths


PIPELINE_DIR = Path(__file__).resolve().parent

# Shared steps after the adapter: (script, primary output suffix used for the skip check).
SHARED_STEPS = [
    ("03_street_lights.py", "edges_illuminated_continuous.gpkg"),
    ("04_directional_lighting.py", "directional_lighting_lookup.csv"),
]


def pick_adapter(city: str) -> str:
    """The step-2 adapter matching the lamp inventory the city provides."""
    if paths.find_input(city, "puntiLuce.gpkg") is not None:
        return "02_street_lights_torino.py"
    if paths.find_input(city, "streetlights.gpkg") is not None:
        return "02_street_lights_generic.py"
    raise FileNotFoundError(
        f"No lamp inventory ({city}_puntiLuce.gpkg or {city}_streetlights.gpkg) in "
        f"inputData/{city} or the resources folder."
    )


def run_step(script: str, city: str) -> None:
    print(f"[RUN] {script}")
    subprocess.run(
        [sys.executable, str(PIPELINE_DIR / script), "--city", city],
        check=True,
    )


def main() -> None:
    parser = argparse.ArgumentParser(description="Build street-lighting data from a lamp inventory.")
    parser.add_argument("--city", required=True,
                        help="City name: folder under inputData/ and src/main/resources/, "
                             "and the <City>_* file prefix.")
    parser.add_argument("--force", action="store_true", help="Re-run steps even if their output exists.")
    args = parser.parse_args()

    city = args.city
    print(f"city: {city}")

    # Step 2: adapter chosen by inventory; both write <City>_streetlights_with_radius.gpkg.
    existing = paths.find_input(city, "streetlights_with_radius.gpkg")
    if existing is not None and not args.force:
        print(f"[SKIP] step 2 ({existing.name} exists)")
    else:
        run_step(pick_adapter(city), city)

    for script, output_suffix in SHARED_STEPS:
        existing = paths.find_input(city, output_suffix)
        if existing is not None and not args.force:
            print(f"[SKIP] {script} ({existing.name} exists)")
            continue
        run_step(script, city)

    print(f"Done. Lighting data ready in {paths.resources_dir(city)}")


if __name__ == "__main__":
    main()
