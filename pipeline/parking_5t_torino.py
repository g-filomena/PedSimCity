"""Turin adapter for ``parking.py``: the 5T parking feed and the Comune's limited-traffic zones.

Downloads, into ``inputData/<City>/parking/``:

- ``parking.csv`` from 5T's ``https://opendata.5t.torino.it/get_pk`` (CC-BY): the parking
  structures' names, positions and capacity (``Total``), as ``parking.py`` reads them (``name``,
  ``lat``, ``lon``, ``capacity``). The feed's occupancy is a snapshot and is not kept.
- ``ztl_geo.zip``, the ZTL perimeters, from the Comune di Torino geoportal.

Then run ``parking.py`` on them, as its docstring shows.

    python parking_5t_torino.py --city Torino
"""

from __future__ import annotations

import argparse
import urllib.request
import xml.etree.ElementTree as ET

import pandas as pd

import paths

PARKING_URL = "https://opendata.5t.torino.it/get_pk"
ZTL_URL = "http://geoportale.comune.torino.it/geodati/zip/ztl_geo.zip"


def fetch(url: str) -> bytes:
    print(f"downloading {url}")
    request = urllib.request.Request(url, headers={"User-Agent": "Mozilla/5.0"})
    with urllib.request.urlopen(request, timeout=300) as response:
        return response.read()


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--city", required=True, help="folder and file prefix, e.g. Torino")
    args = parser.parse_args()

    folder = paths.raw_dir(args.city) / "parking"
    folder.mkdir(parents=True, exist_ok=True)
    root = ET.fromstring(fetch(PARKING_URL))
    rows = [{"name": e.attrib["Name"], "lat": float(e.attrib["lat"]),
             "lon": float(e.attrib["lng"]), "capacity": int(e.attrib["Total"])}
            for e in root.iter() if e.tag.endswith("PK_data")]
    pd.DataFrame(rows).to_csv(folder / "parking.csv", index=False)
    (folder / "ztl_geo.zip").write_bytes(fetch(ZTL_URL))
    print(f"{folder}: parking.csv ({len(rows)} structures), ztl_geo.zip")


if __name__ == "__main__":
    main()
