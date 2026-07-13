"""Step 2 (Turin `puntiLuce` adapter): per-lamp physics from a rich lamp inventory.

For inventories following the Turin open-data "Punti Luce" schema (`potenza_w_max`,
`altezza_palo_m`, `braccio_l_m_max`, `tecnologia`, `uso_ottica`, …). Real attributes
drive the physics; missing values are filled by the median of the lamp's technology /
optics group, then sensible defaults.

Reads <City>_puntiLuce.gpkg (from inputData/<City>/ or the resources folder) and
writes the intermediate <City>_streetlights_with_radius.gpkg to inputData/<City>/
(the single step-2 output name step 3 consumes, whichever adapter produced it).

`--city` is the city name (folder under inputData/ and resources/, and file prefix).
"""

from __future__ import annotations

import argparse
from pathlib import Path

import geopandas as gpd
import numpy as np
import pandas as pd

import paths


# Defaults for lamps whose group medians cannot be computed.
DEFAULT_POWER_W = 100.0
DEFAULT_HEIGHT_M = 9.0
DEFAULT_EFFICACY_LM_W = 70.0
DEFAULT_UTILIZATION = 0.4
E_MIN_LUX = 5.0

LUMINOUS_EFFICACY_MAP = {
    "LED / probabile LED": 120,
    "scarica / HID-CDM-HQL": 90,
    "non determinata": 70,
}
UTILIZATION_FACTOR_MAP = {
    "stradale": 0.6,
    "viali": 0.6,
    "sospeso": 0.4,
    "storico/decorativo; sospeso": 0.35,
    "storico/decorativo": 0.3,
    "non determinato": 0.4,
}


def remove_existing(path: Path) -> None:
    if path.exists():
        path.unlink()


def numeric_column(frame: gpd.GeoDataFrame, column: str, default: float) -> pd.Series:
    if column not in frame.columns:
        return pd.Series(default, index=frame.index, dtype="float64")
    return pd.to_numeric(frame[column], errors="coerce")


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Calculate per-lamp street-light physics from a puntiLuce inventory."
    )
    parser.add_argument("--city", required=True,
                        help="City name: folder under inputData/ and src/main/resources/, "
                             "and the <City>_* file prefix.")
    args = parser.parse_args()
    city = args.city

    punti_path = paths.require_input(city, "puntiLuce.gpkg", "puntiLuce lamp inventory")
    output_path = paths.raw_dir(city) / f"{city}_streetlights_with_radius.gpkg"

    print(f"city: {city}")
    print(f"Loading puntiLuce inventory: {punti_path.name}")

    punti = gpd.read_file(punti_path)
    if punti.empty:
        raise ValueError(f"Lamp inventory is empty: {punti_path}")
    punti = punti[punti.geometry.notnull() & ~punti.geometry.is_empty].copy()

    print("Cleaning data...")
    punti["potenza_w_max"] = numeric_column(punti, "potenza_w_max", np.nan)
    punti["altezza_palo_m"] = numeric_column(punti, "altezza_palo_m", np.nan)
    punti["braccio_l_m_max"] = numeric_column(punti, "braccio_l_m_max", 0.0)

    punti.loc[punti["potenza_w_max"] > 500, "potenza_w_max"] = np.nan

    if "tecnologia" in punti.columns:
        punti["potenza_w_max"] = punti["potenza_w_max"].fillna(
            punti.groupby("tecnologia")["potenza_w_max"].transform("median")
        )

    global_power_median = punti["potenza_w_max"].median()
    punti["potenza_w_max"] = punti["potenza_w_max"].fillna(
        global_power_median if not pd.isna(global_power_median) else DEFAULT_POWER_W
    )

    if "uso_ottica" in punti.columns:
        punti["altezza_palo_m"] = punti["altezza_palo_m"].fillna(
            punti.groupby("uso_ottica")["altezza_palo_m"].transform("median")
        )

    punti["altezza_palo_m"] = punti["altezza_palo_m"].fillna(DEFAULT_HEIGHT_M)
    punti["braccio_l_m_max"] = punti["braccio_l_m_max"].fillna(0.0)

    if "tecnologia" in punti.columns:
        punti["luminous_efficacy"] = (
            punti["tecnologia"].map(LUMINOUS_EFFICACY_MAP).fillna(DEFAULT_EFFICACY_LM_W)
        )
    else:
        punti["luminous_efficacy"] = DEFAULT_EFFICACY_LM_W

    if "uso_ottica" in punti.columns:
        punti["utilization_factor"] = (
            punti["uso_ottica"].map(UTILIZATION_FACTOR_MAP).fillna(DEFAULT_UTILIZATION)
        )
    else:
        punti["utilization_factor"] = DEFAULT_UTILIZATION

    print("Calculating physics: lumens, intensity, radius...")
    punti["total_lumens"] = punti["potenza_w_max"] * punti["luminous_efficacy"]
    punti["downward_intensity_cd"] = (punti["total_lumens"] * punti["utilization_factor"]) / np.pi

    intensity = punti["downward_intensity_cd"]
    height = punti["altezza_palo_m"]
    radius_term = np.power((intensity * height) / E_MIN_LUX, 2.0 / 3.0) - np.power(height, 2.0)
    punti["radius_m"] = np.sqrt(np.clip(radius_term, a_min=0.0, a_max=None)).fillna(0.0)

    punti["radius"] = punti["radius_m"]
    # Luminous INTENSITY in candela. Illuminance is computed per 2 m
    # sample point in step 03 (calculated_lux); this per-lamp value is intensity.
    punti["intensity_cd"] = punti["downward_intensity_cd"]

    print(f"Saving to {output_path}...")
    remove_existing(output_path)
    punti.to_file(output_path, driver="GPKG")
    print("Done.")


if __name__ == "__main__":
    main()
