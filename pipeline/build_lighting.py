"""PedSimCity street-lighting pipeline - generic lamp-point locations.

For cities where only street-light pole / lamp-point LOCATIONS are available, with no technical attributes.
Runs the generic step 02, then the shared illumination steps 03 and 04.

`--input_dir` is the resources folder path.
`--city_name` is the filename prefix; if omitted, the input folder name is used.
"""

from __future__ import annotations

import argparse
import subprocess
import sys
from pathlib import Path


PIPELINE_DIR = Path(__file__).resolve().parent

# (script, primary output used for the skip check; {city} filled per run)
STEPS = [
    ("02_street_lights_generic.py", "{city}_streetlights_with_radius.gpkg"),
    ("03_street_lights.py", "{city}_edges_illuminated_continuous.gpkg"),
    ("04_directional_lighting.py", "{city}_directional_lighting_lookup.csv"),
]


def main() -> None:
    parser = argparse.ArgumentParser(description="Build street-lighting data from bare lamp-point locations.")
    parser.add_argument("--input_dir", required=True, help="Resources folder, e.g. src/main/resources/Manchester")
    parser.add_argument(
        "--city_name",
        default=None,
        help="Filename prefix before _edges.gpkg / _streetlights.gpkg. Defaults to input folder name.",
    )
    parser.add_argument("--force", action="store_true", help="Re-run steps even if their output exists.")
    args = parser.parse_args()

    input_dir = Path(args.input_dir).resolve()
    if not input_dir.exists():
        raise FileNotFoundError(f"Input directory not found: {input_dir}")

    city = args.city_name or input_dir.name

    print(f"resources folder: {input_dir}")
    print(f"city file prefix: {city}")

    for script, output in STEPS:
        out = input_dir / output.format(city=city)
        if out.exists() and not args.force:
            print(f"[SKIP] {script} ({out.name} exists)")
            continue

        print(f"[RUN] {script}")
        subprocess.run(
            [
                sys.executable,
                str(PIPELINE_DIR / script),
                "--input_dir",
                str(input_dir),
                "--city_name",
                city,
            ],
            check=True,
        )

    print(f"Done. Lighting data ready in {input_dir}")


if __name__ == "__main__":
    main()
