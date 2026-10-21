"""Turin adapter for ``accommodation.py``: the Regione Piemonte register of accommodation.

Downloads the register's six tables (hotels, hostels and holiday homes for groups, bed and
breakfasts, rooms to let, holiday homes and flats, holiday lets), as published by the Comune di
Torino's open-data portal (CC-BY, 2020 edition), into ``inputData/<City>/accommodation/register/``
- a table already there is read rather than downloaded again - keeps the establishments in the
comune, and writes ``inputData/<City>/accommodation/accommodation.csv`` as ``accommodation.py`` reads it
(``name``, ``type``, ``lat``, ``lon``, ``beds``).

About a third of the register's beds have no coordinates. They are not dropped: each type's
located establishments are scaled up to that type's total, so the beds keep the city total and
take the located establishments' distribution. The register holds no short-term lets
(``locazioni turistiche``), so their geography is not represented.

    python accommodation_torino.py --city Torino --comune TORINO
"""

from __future__ import annotations

import argparse
import urllib.request

import pandas as pd

import paths

BASE = "https://risorse.comune.torino.it/opendata//turismo/"
TABLES = ["regpie-Albergo_3764-all.csv", "regpie-Ostello_3769-all.csv",
          "regpie-Bed_and_brea_3766-all.csv", "regpie-Affittacamer_3763-all.csv",
          "regpie-Casa_vacanze_3768-all.csv", "regpie-Alloggio_vac_3765-all.csv"]


def fetch(url: str) -> bytes:
    print(f"downloading {url}")
    request = urllib.request.Request(url, headers={"User-Agent": "Mozilla/5.0"})
    with urllib.request.urlopen(request, timeout=300) as response:
        return response.read()


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--city", required=True, help="folder and file prefix, e.g. Torino")
    parser.add_argument("--comune", default="TORINO", help="the register's Comune value to keep")
    args = parser.parse_args()

    folder = paths.raw_dir(args.city) / "accommodation"
    register_dir = folder / "register"
    register_dir.mkdir(parents=True, exist_ok=True)
    rows = []
    for table in TABLES:
        local = register_dir / table
        if not local.exists():
            local.write_bytes(fetch(BASE + table))
        frame = pd.read_csv(local, sep=";", encoding="latin-1", dtype=str, on_bad_lines="skip")
        frame = frame[frame.Comune.str.upper().str.strip() == args.comune.upper()]
        rows.append(frame[["Tipologia", "Denominazione", "Latitudine", "Longitudine",
                           "Posti_letto"]])
    register = pd.concat(rows)
    register["beds"] = pd.to_numeric(register.Posti_letto, errors="coerce")
    register["lat"] = pd.to_numeric(register.Latitudine.str.replace(",", "."), errors="coerce")
    register["lon"] = pd.to_numeric(register.Longitudine.str.replace(",", "."), errors="coerce")
    register = register[register.beds > 0]
    total = register.groupby("Tipologia").beds.sum()
    located = register[register.lat.notna() & register.lon.notna()].copy()
    scale = total / located.groupby("Tipologia").beds.sum()
    located["beds"] = located.beds * located.Tipologia.map(scale)
    out = located.rename(columns={"Denominazione": "name", "Tipologia": "type"})[
        ["name", "type", "lat", "lon", "beds"]]
    out.to_csv(folder / "accommodation.csv", index=False)
    lost = total.sum() - out.beds.sum()
    print(f"{folder / 'accommodation.csv'}: {len(out)} establishments, {out.beds.sum():.0f} beds "
          f"(register {total.sum():.0f}; {lost:.0f} in types with no located establishment)")
    for kind, factor in scale.items():
        print(f"  {kind}: located beds scaled x{factor:.2f}")


if __name__ == "__main__":
    main()
