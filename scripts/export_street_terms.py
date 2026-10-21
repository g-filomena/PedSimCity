"""Every street's night terms, for the results site's street map.

Reads the table ``NightCostTermRoutes.java --streets`` writes (lux, park and water flags, smallest
isovist, venues open by hour) and the edge layer's names and classes, and writes one JSON the page
joins to ``<City>_edges.geojson`` by ``edgeID``:

    python export_street_terms.py streets.csv --hour 22 \
        --out ../inclusivestreets/torino-di-notte/data/Torino/streets/streets.json

The page computes darkness, enclosure, eyes and the night cost from these with the weights stored
here, so the file carries measurements rather than one model's verdict on them.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import geopandas as gpd
import pandas as pd

REPO_ROOT = Path(__file__).resolve().parent.parent

# NightPars at the time of the season9 runs. Each agent draws its reassurance level uniformly in
# [minReassuranceLux, maxReassuranceLux]; the map shows the person in the middle of that range.
WEIGHTS = {"vulnerable": [1.0, 1.0, 1.0, 1.0], "nonVulnerable": [0.5, 0.5, 0.5, 0.5]}
REASSURANCE_LUX = (8.9 + 26.0) / 2
ENCLOSED_AREA_M2 = 5000.0
EYES_SATURATION_PER_100M = 2.0
EYES_MIN_LENGTH = 50.0


def first(value):
    if isinstance(value, (list, tuple)):
        return value[0] if value else None
    if isinstance(value, str) and value.startswith("["):
        inner = value.strip("[]").split(",")[0].strip().strip("'\"")
        return inner or None
    return None if pd.isna(value) else value


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("streets", type=Path, help="CSV from NightCostTermRoutes --streets")
    parser.add_argument("--city", default="Torino")
    parser.add_argument("--hour", type=int, default=22)
    parser.add_argument("--out", type=Path, required=True)
    args = parser.parse_args()

    terms = pd.read_csv(args.streets)
    layer = REPO_ROOT / "src" / "main" / "resources" / args.city / f"{args.city}_edges.gpkg"
    edges = gpd.read_file(layer)[["edgeID", "name", "highway", "length"]]
    table = terms.merge(edges, on="edgeID", how="left").sort_values("edgeID")

    names, kinds = [], []
    name_index, kind_index = {}, {}

    def index_of(value, values, lookup):
        if value is None:
            return -1
        if value not in lookup:
            lookup[value] = len(values)
            values.append(value)
        return lookup[value]

    name_ids = [index_of(first(v), names, name_index) for v in table["name"]]
    kind_ids = [index_of(first(v), kinds, kind_index) for v in table["highway"]]
    flags = (table["park"].astype(int) + 2 * table["water"].astype(int)).tolist()

    data = {
        "city": args.city,
        "hour": args.hour,
        "weights": WEIGHTS,
        "reassuranceLux": REASSURANCE_LUX,
        "enclosedAreaM2": ENCLOSED_AREA_M2,
        "eyesSaturationPer100m": EYES_SATURATION_PER_100M,
        "eyesMinLength": EYES_MIN_LENGTH,
        "names": names,
        "kinds": kinds,
        "edges": table["edgeID"].astype(int).tolist(),
        "lux": [round(float(v), 1) for v in table["lux"]],
        "flags": flags,
        "iso": [int(v) for v in table["iso_min"]],
        "open": table[f"open{args.hour}"].astype(int).tolist(),
        "name": name_ids,
        "kind": kind_ids,
    }
    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(json.dumps(data, separators=(",", ":"), ensure_ascii=False), encoding="utf-8")
    print(f"{len(table)} streets, {len(names)} names, {sum(v > 0 for v in data['open'])} with "
          f"something open at {args.hour}:00 -> {args.out} ({args.out.stat().st_size / 1e6:.1f} MB)")


if __name__ == "__main__":
    main()
