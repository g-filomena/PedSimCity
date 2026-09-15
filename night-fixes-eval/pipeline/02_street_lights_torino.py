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

import lighting
import paths


# Defaults for lamps whose group medians cannot be computed.
DEFAULT_POWER_W = 100.0
DEFAULT_HEIGHT_M = 9.0
DEFAULT_EFFICACY_LM_W = 70.0
DEFAULT_UTILIZATION = 0.4

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

# tipo_supporto value that means "a pole physically exists at this fixture's location". Every
# other value means altezza_palo_m (pole height) is a category error for that record -- there is
# no pole to have measured a height of, which is exactly why it is 100% absent for all of them in
# this inventory (register finding A2).
POLE_SUPPORT_TYPE = "palo presente sulla stessa sede"

# Assumed mounting height (m) for non-pole support types, since altezza_palo_m carries no signal
# for any of them (0% coverage in this inventory) and there is nothing to derive a height from.
# Unsourced -- stated as an assumption, not a measurement, which is the point of height_source
# below. "solo apparecchio in distinta" (fixture recorded with no pole) is read as facade/bracket
# mounting: lower than a dedicated street pole, consistent with why no pole was needed. The other
# two categories carry no signal to differentiate from the pole population at all (13 records
# combined for "tesata/sostegno..."), so they take the same global pole fallback as a pole with no
# measurement, rather than inventing a second unsourced number for a negligible group.
NON_POLE_HEIGHT_M = {
    "solo apparecchio in distinta": 4.0,
}


def remove_existing(path: Path) -> None:
    if path.exists():
        path.unlink()


def numeric_column(frame: gpd.GeoDataFrame, column: str, default: float) -> pd.Series:
    if column not in frame.columns:
        return pd.Series(default, index=frame.index, dtype="float64")
    return pd.to_numeric(frame[column], errors="coerce")


def assign_heights(punti: gpd.GeoDataFrame) -> gpd.GeoDataFrame:
    """Assigns altezza_palo_m and height_source, grouped by tipo_supporto (register finding A2).

    Grouping by uso_ottica (optics type) -- what this replaces -- is a category error: pole height
    is a property of poles, and altezza_palo_m is 100% absent for every non-pole tipo_supporto.
    Assigning a pole-height median to a fixture that has no pole (uso_ottica cuts across both) is
    what let the 9.0 m DEFAULT_HEIGHT_M literal look reachable when the true fallback the data
    supports -- a per-street pole median -- was never consulted.

    Poles: measured height where recorded (88.3% of poles in this inventory); missing ->
    same-street (`via`) median of other measured poles; still missing (no other measured pole on
    that street) -> the global pole median.

    Non-poles: no altezza_palo_m signal exists at all, so height is assumed by support type from
    NON_POLE_HEIGHT_M, falling back to the global pole median for support types with no assumption
    entered (see that dict's comment).

    height_source records provenance for every row, so it survives into the physics and beyond
    rather than being indistinguishable from a real measurement.
    """
    has_support_type = "tipo_supporto" in punti.columns
    is_pole = (
        punti["tipo_supporto"] == POLE_SUPPORT_TYPE if has_support_type else pd.Series(True, index=punti.index)
    )

    height = pd.Series(np.nan, index=punti.index, dtype="float64")
    source = pd.Series("", index=punti.index, dtype="object")

    # Poles: measured value first.
    measured = is_pole & punti["altezza_palo_m"].notna()
    height.loc[measured] = punti.loc[measured, "altezza_palo_m"]
    source.loc[measured] = "measured"

    # Poles missing a height: same-street median among OTHER measured poles.
    pole_missing = is_pole & ~measured
    if pole_missing.any() and "via" in punti.columns:
        street_median = (
            punti.loc[measured].groupby(punti.loc[measured, "via"])["altezza_palo_m"].median()
        )
        # .map on the full via column, not just the missing rows: keeps the result aligned to
        # punti's own index throughout, so the pole_missing mask can be combined with it directly.
        mapped = punti["via"].map(street_median)
        got_street = pole_missing & mapped.notna()
        height.loc[got_street] = mapped.loc[got_street]
        source.loc[got_street] = "same_street_median"

    global_pole_median = punti.loc[measured, "altezza_palo_m"].median()
    if pd.isna(global_pole_median):
        global_pole_median = DEFAULT_HEIGHT_M

    # Poles still missing (unique/unmeasured street): global pole median.
    still_missing_pole = is_pole & height.isna()
    height.loc[still_missing_pole] = global_pole_median
    source.loc[still_missing_pole] = "global_pole_median"

    # Non-poles: assumed by support type, else the same global pole fallback.
    non_pole = ~is_pole
    if has_support_type:
        assumed = punti.loc[non_pole, "tipo_supporto"].map(NON_POLE_HEIGHT_M)
        height.loc[non_pole] = assumed.fillna(global_pole_median)
        source.loc[non_pole] = np.where(
            assumed.notna(), "support_type_assumed", "global_pole_median"
        )
    else:
        height.loc[non_pole] = global_pole_median
        source.loc[non_pole] = "global_pole_median"

    n_total = len(punti)
    print(f"  height_source: "
          f"{(source == 'measured').sum()} measured, "
          f"{(source == 'same_street_median').sum()} same_street_median, "
          f"{(source == 'support_type_assumed').sum()} support_type_assumed, "
          f"{(source == 'global_pole_median').sum()} global_pole_median "
          f"(of {n_total}); global pole median = {global_pole_median:.2f} m")

    punti["altezza_palo_m"] = height
    punti["height_source"] = source
    return punti


def main() -> None:
    # potenza_w_max is a MAXIMUM rated power, used below as-is for downward_intensity_cd. There is
    # no dimming or maintenance-factor field anywhere in this inventory to derive a real-world
    # discount from, and Turin's LED programme is known to include smart (dimmable) control -- so
    # every lamp here is modelled as new and at full rated power, which this inventory cannot
    # correct (register finding A7). Not fixable from the data available; stated here, and in the
    # paper's methods, rather than silently assumed.
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

    print("Assigning mounting heights (grouped by tipo_supporto)...")
    punti = assign_heights(punti)
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

    print("Calculating physics: lumens and downward intensity...")
    punti["total_lumens"] = punti["potenza_w_max"] * punti["luminous_efficacy"]
    # See the generic adapter: intensity and pole height are all step 3 reads.
    punti["downward_intensity_cd"] = lighting.downward_intensity_cd(
        punti["potenza_w_max"], punti["luminous_efficacy"], punti["utilization_factor"]
    )

    print(f"Saving to {output_path}...")
    remove_existing(output_path)
    punti.to_file(output_path, driver="GPKG")
    print("Done.")


if __name__ == "__main__":
    main()
