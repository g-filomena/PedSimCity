"""Walked route lengths for Melbourne's OD matrix, from the VISTA travel survey.

Victorian Integrated Survey of Travel and Activity (Department of Transport and Planning, CC BY 4.0),
https://opendata.transport.vic.gov.au/dataset/victorian-integrated-survey-of-travel-and-activity-vista
The `stops` tables hold one row per trip stage, with its mode (`fullmode`) and length
(`vistadist`, km). Kept: walking stages that start and end in the City of Melbourne, on weekdays,
departing 07:00-18:59 - the day type, hours and area the counter targets are built on. Walks to and
from public transport are stages of their own and are kept, because the counters see them.

`vistadist` is taken as a walked (network) length. The explanatory document does not define it; the
median implied walking speed of the kept stages, `vistadist` over the reported travel time, is about
4.0 km/h, which a straight-line distance would put well below normal walking pace.

Writes src/main/resources/Melbourne/Melbourne_walkLengths.csv (`metres,weight`, one row per stage,
weight = `stoppoststratweight`) and a provenance file beside it.

    python pipeline/vista_walk_lengths.py inputData/Melbourne/VISTA
"""

import argparse
import hashlib
import json
from datetime import datetime, timezone
from pathlib import Path

import numpy as np
import pandas as pd

ROOT = Path(__file__).resolve().parent.parent
WAVES = ("2022_2023", "2023_2024", "2024_2025")
LGA = "Melbourne (C)"
HOURS = (7, 19)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("vista", type=Path, help="folder holding stops_vista_<wave>.csv")
    parser.add_argument("--out", type=Path,
                        default=ROOT / "src/main/resources/Melbourne/Melbourne_walkLengths.csv")
    args = parser.parse_args()

    files = [args.vista / f"stops_vista_{wave}.csv" for wave in WAVES]
    stops = pd.concat([pd.read_csv(f, low_memory=False) for f in files], ignore_index=True)
    stops["metres"] = pd.to_numeric(stops.vistadist, errors="coerce") * 1000.0
    stops["weight"] = pd.to_numeric(stops.stoppoststratweight, errors="coerce")
    stops["hour"] = pd.to_numeric(stops.starthour, errors="coerce")

    kept = stops[(stops.fullmode == "Walking")
                 & (stops.origlga == LGA) & (stops.destlga == LGA)
                 & (stops.dayType == "Weekday")
                 & stops.hour.between(HOURS[0], HOURS[1] - 1)
                 & (stops.metres > 0) & (stops.weight > 0)]
    out = kept[["metres", "weight"]].sort_values("metres")
    out["metres"] = out.metres.round(1)
    args.out.parent.mkdir(parents=True, exist_ok=True)
    out.to_csv(args.out, index=False)

    order = np.argsort(out.metres.values)
    cumulative = np.cumsum(out.weight.values[order]) / out.weight.sum()
    quantiles = {f"p{int(p * 100)}": float(out.metres.values[order][np.searchsorted(cumulative, p)])
                 for p in (0.1, 0.25, 0.5, 0.75, 0.9, 0.95)}
    speed = (kept.metres / 1000.0) / (pd.to_numeric(kept.travtime, errors="coerce") / 60.0)
    provenance = {
        "written": datetime.now(timezone.utc).isoformat(timespec="seconds"),
        "source": "VISTA stops tables, waves " + ", ".join(WAVES),
        "licence": "CC BY 4.0, Department of Transport and Planning, Victoria",
        "files": {f.name: hashlib.sha256(f.read_bytes()).hexdigest() for f in files},
        "filter": f"fullmode == Walking, origlga == destlga == {LGA}, dayType == Weekday, "
                  f"starthour {HOURS[0]}-{HOURS[1] - 1}, vistadist > 0",
        "stages": int(len(out)),
        "weighted_quantiles_metres": quantiles,
        "median_implied_speed_kmh": float(speed.replace(np.inf, np.nan).median()),
    }
    args.out.with_suffix(".provenance.json").write_text(json.dumps(provenance, indent=2) + "\n")
    print(json.dumps(provenance, indent=2))


if __name__ == "__main__":
    main()
