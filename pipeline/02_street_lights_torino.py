"""Step 2 (Turin): per-lamp physics from the `puntiLuce` inventory.

Reads <city>_puntiLuce.gpkg and writes <city>_puntiLuce_with_radius.gpkg.
`--input_dir` is the resources folder path.
`--city_name` is the filename prefix; if omitted, the input folder name is used.
"""

from __future__ import annotations

import argparse
import os
from pathlib import Path

import geopandas as gpd
import numpy as np
import pandas as pd


def remove_existing(path: Path) -> None:
    if path.exists():
        path.unlink()


def numeric_column(frame: gpd.GeoDataFrame, column: str, default: float) -> pd.Series:
    if column not in frame.columns:
        return pd.Series(default, index=frame.index, dtype="float64")
    return pd.to_numeric(frame[column], errors="coerce")


def main() -> None:
    parser = argparse.ArgumentParser(description="Calculate base street-light physics.")
    parser.add_argument("--input_dir", required=True)
    parser.add_argument(
        "--city_name",
        default=None,
        help="Filename prefix before _puntiLuce.gpkg / _edges.gpkg. Defaults to input folder name.",
    )
    args = parser.parse_args()

    input_dir = Path(os.path.abspath(args.input_dir))
    city = args.city_name or input_dir.name

    punti_path = input_dir / f"{city}_puntiLuce.gpkg"
    output_path = input_dir / f"{city}_puntiLuce_with_radius.gpkg"

    if not punti_path.exists():
        raise FileNotFoundError(f"Raw lamp inventory not found: {punti_path}")

    print(f"resources folder: {input_dir}")
    print(f"city file prefix: {city}")
    print("Loading raw puntiLuce data...")

    punti = gpd.read_file(punti_path)
    if punti.empty:
        raise ValueError(f"Lamp inventory is empty: {punti_path}")

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
        global_power_median if not pd.isna(global_power_median) else 100.0
    )

    if "uso_ottica" in punti.columns:
        punti["altezza_palo_m"] = punti["altezza_palo_m"].fillna(
            punti.groupby("uso_ottica")["altezza_palo_m"].transform("median")
        )

    punti["altezza_palo_m"] = punti["altezza_palo_m"].fillna(9.0)
    punti["braccio_l_m_max"] = punti["braccio_l_m_max"].fillna(0.0)

    luminous_efficacy_map = {
        "LED / probabile LED": 120,
        "scarica / HID-CDM-HQL": 90,
        "non determinata": 70,
    }
    utilization_factor_map = {
        "stradale": 0.6,
        "viali": 0.6,
        "sospeso": 0.4,
        "storico/decorativo; sospeso": 0.35,
        "storico/decorativo": 0.3,
        "non determinato": 0.4,
    }

    if "tecnologia" in punti.columns:
        punti["luminous_efficacy"] = punti["tecnologia"].map(luminous_efficacy_map).fillna(70)
    else:
        punti["luminous_efficacy"] = 70

    if "uso_ottica" in punti.columns:
        punti["utilization_factor"] = punti["uso_ottica"].map(utilization_factor_map).fillna(0.4)
    else:
        punti["utilization_factor"] = 0.4

    print("Calculating physics: lumens, intensity, radius...")
    punti["total_lumens"] = punti["potenza_w_max"] * punti["luminous_efficacy"]
    punti["downward_intensity_cd"] = (punti["total_lumens"] * punti["utilization_factor"]) / np.pi

    e_min = 5.0
    intensity = punti["downward_intensity_cd"]
    height = punti["altezza_palo_m"]
    radius_term = np.power((intensity * height) / e_min, 2.0 / 3.0) - np.power(height, 2.0)
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
