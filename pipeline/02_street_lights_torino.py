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

# Mounting height where the inventory gives none, and the support type says there is no pole to
# give one. An ASSUMPTION, not a measurement: a wall bracket or one of Turin's arcade fixtures
# hangs at roughly this height. It is stated here rather than inherited from the pole median
# because height enters the illuminance law twice - once in the numerator and once inside the
# slant distance - so putting a wall fixture nine metres up is not a small error.
#
# **It still wants a source, and this is what was checked on 16 September 2026 so nobody checks it
# twice.** EN 13201 and UNI 11248 classify roads and set maintained illuminance; neither prescribes
# a mounting height. CEI 64-8/7 section 714 (replacing CEI 64-7) gives a floor rather than a value:
# a luminaire whose lamp is reachable without a tool must be above 2.8 m. Regione Piemonte
# L.R. 31/2000 Allegato A punto 1(d) constrains the spacing-to-height RATIO (> 3.7 for new
# installations) and so says nothing about either term alone. Turin's own PRIC publishes a
# 79.9 MB "fascicolo completo apparecchi" that plausibly settles it and has not been read. The
# inventory cannot answer it either: every one of the three support types with no measured height
# has NO measured height at all, so there is nothing to impute from.
#
# What there is instead is a bound. `--no-pole-height` re-runs the pipeline at another value, and
# the 16 September sweep over Torino is reported in pipeline/README.md: it says how much of the
# city's darkness rests on this number.
NO_POLE_HEIGHT_M = 4.0

LUMINOUS_EFFICACY_MAP = {
    "LED / probabile LED": 120,
    "scarica / HID-CDM-HQL": 90,
    "non determinata": 70,
}
# Downward light output ratio: the share of LAMP lumens the luminaire emits downward. See
# pipeline/lighting.py for why the intensity formula wants DLOR and not the utilisation factor that
# used to sit here, and for the two facts that bound it - Regione Piemonte L.R. 31/2000 Allegato A
# punto 1(a) holding ULOR to ~0, and a LED luminaire's rated efficacy already being the luminaire's
# rather than a bare source's, which makes its LOR 1.0 by convention.
#
# Keyed on technology first, because that is what decides whether `luminous_efficacy` above is a
# luminaire figure or a lamp figure, then on whether the fixture is a decorative lantern, whose
# optical control is much worse than a road reflector's. In this inventory the two axes are nearly
# the same fact - 33,124 of 33,175 `stradale` lamps are LED, 8,550 of 8,921 `storico/decorativo` are
# discharge - but they are different properties and a retrofitted city will separate them.
#
# THE FOUR VALUES ARE ENGINEERING JUDGEMENT, not measurements. What a per-luminaire IES/LDT
# photometric file would give is the real DLOR, and the same file would settle the falloff law; both
# are waiting on the same thing. What has changed is that these are judgements about a quantity
# EN 13032-1 defines and Piemonte law bounds on one side, rather than numbers attached to a label.
DLOR_BY_CLASS = {
    ("led", "road"): 1.00,
    ("led", "decorative"): 0.85,
    ("discharge", "road"): 0.80,
    ("discharge", "decorative"): 0.55,
}


def remove_existing(path: Path) -> None:
    if path.exists():
        path.unlink()


def text_column(frame: gpd.GeoDataFrame, column: str) -> pd.Series:
    """A lower-cased, never-null view of a free-text column; empty strings where it is absent."""
    if column not in frame.columns:
        return pd.Series("", index=frame.index, dtype="object")
    return frame[column].astype("object").fillna("").astype(str).str.strip().str.lower()


def downward_light_output_ratios(punti: gpd.GeoDataFrame) -> pd.Series:
    """DLOR per lamp, from its technology and whether its optic is a decorative lantern.

    A technology this inventory does not determine is treated as discharge: it is the older half of
    Turin's stock, and crediting an unknown fixture with a LED luminaire's LOR would raise the lux
    of exactly the lamps least likely to deserve it.
    """
    technology = np.where(text_column(punti, "tecnologia").str.contains("led"), "led", "discharge")
    optic = np.where(
        text_column(punti, "uso_ottica").str.contains("storico|decorativo", regex=True),
        "decorative",
        "road",
    )
    dlor = pd.Series(
        [DLOR_BY_CLASS[(t, o)] for t, o in zip(technology, optic)],
        index=punti.index,
        dtype="float64",
    )
    print("  downward light output ratio:")
    for value, count in dlor.value_counts().sort_index(ascending=False).items():
        print(f"    {value:.2f} {count:>17} ({100.0 * count / len(dlor):.1f}%)")
    print(f"    mean {dlor.mean():.3f}")
    return dlor


def numeric_column(frame: gpd.GeoDataFrame, column: str, default: float) -> pd.Series:
    if column not in frame.columns:
        return pd.Series(default, index=frame.index, dtype="float64")
    return pd.to_numeric(frame[column], errors="coerce")


def impute_mounting_height(punti: gpd.GeoDataFrame, no_pole_height: float) -> None:
    """Fill a missing `altezza_palo_m` from the lamp's SUPPORT type, and record how.

    The inventory splits cleanly by `tipo_supporto` and not at all by `uso_ottica`, which is what
    this used to group on. On Turin, height is missing for every lamp whose support is
    `solo apparecchio in distinta` (a fixture with no pole of its own) and every `non determinato`,
    against 11.7% of the lamps that do stand on a pole - 45% of the inventory running on an imputed
    height. Grouping by optics mixed those two populations and handed the pole-less ones a pole's
    height.

    The rule is derived from the data rather than from a list of Italian labels, so it carries to
    another city's inventory:

    - a support type that has SOME measured heights takes that type's median (`support_median`);
    - a support type with NO measured height at all has no pole to measure, so it takes the stated
      `NO_POLE_HEIGHT_M` assumption (`no_pole_assumed`);
    - with no `tipo_supporto` column there is nothing to group on, so the global median of the
      measured heights stands in (`global_median`), and with nothing measured anywhere either,
      `DEFAULT_HEIGHT_M` (`default`).

    **The last resort fills rather than fails, deliberately.** A pipeline that refuses to run on a
    thin inventory produces no lighting layer at all, which is worse than a declared assumption: the
    9 m is the same figure the generic point-only adapter applies to every lamp, and it is reported
    here the same way. What made the old behaviour a defect was not the number but that it was
    invisible and applied to 45% of Turin's lamps through the wrong grouping. Both halves of that
    are closed above, and `altezza_source` counts whatever is left, so a run that leans on this says
    so in its own output — and step 3 prints the imputed share again before it computes anything.
    """
    measured = punti["altezza_palo_m"].copy()
    punti["altezza_source"] = np.where(measured.notna(), "measured", None)

    if "tipo_supporto" in punti.columns:
        support = punti["tipo_supporto"].astype("object").fillna("").astype(str).str.strip().str.lower()
        # Medians come from the measured values only, so an imputed height can never seed another.
        group_median = measured.groupby(support).transform("median")

        from_group = measured.isna() & group_median.notna()
        punti.loc[from_group, "altezza_palo_m"] = group_median[from_group]
        punti.loc[from_group, "altezza_source"] = "support_median"

        no_pole = measured.isna() & group_median.isna()
        punti.loc[no_pole, "altezza_palo_m"] = no_pole_height
        punti.loc[no_pole, "altezza_source"] = "no_pole_assumed"

    still_missing = punti["altezza_palo_m"].isna()
    if still_missing.any():
        global_median = measured.median()
        if pd.isna(global_median):
            punti.loc[still_missing, "altezza_palo_m"] = DEFAULT_HEIGHT_M
            punti.loc[still_missing, "altezza_source"] = "default"
        else:
            punti.loc[still_missing, "altezza_palo_m"] = global_median
            punti.loc[still_missing, "altezza_source"] = "global_median"

    counts = punti["altezza_source"].value_counts()
    total = len(punti)
    print("  mounting height:")
    for source, count in counts.items():
        note = f"  at the stated {no_pole_height} m" if source == "no_pole_assumed" else ""
        print(f"    {source:<16} {count:>7} ({100.0 * count / total:.1f}%){note}")
    invented = int(counts.get("default", 0))
    if invented:
        print(
            f"    WARNING: {invented} lamps have no measured height and no support type to impute "
            f"from; they are running on the flat {DEFAULT_HEIGHT_M} m assumption."
        )


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Calculate per-lamp street-light physics from a puntiLuce inventory."
    )
    parser.add_argument("--city", required=True,
                        help="City name: folder under inputData/ and src/main/resources/, "
                             "and the <City>_* file prefix.")
    parser.add_argument("--falloff-law", choices=lighting.LAW_CHOICES, default=lighting.FALLOFF_LAW,
                        help="Photometric falloff law for this run. It sets the flux "
                             "normalisation, so step 3 must be given the same one. "
                             "See pipeline/lighting.py.")
    parser.add_argument("--variant", default="",
                        help="Suffix for this run's output (e.g. 'lambertian'), so a falloff-law "
                             "comparison does not overwrite the canonical step-2 file.")
    parser.add_argument("--no-pole-height", type=float, default=NO_POLE_HEIGHT_M,
                        help="Mounting height (m) for lamps whose support type has no measured "
                             "height anywhere. It has no source, so it exists as a flag: run the "
                             "pipeline at two values and report how far the answer moves.")
    args = parser.parse_args()
    city = args.city
    lighting.set_law(args.falloff_law)
    variant = f"_{args.variant}" if args.variant else ""

    punti_path = paths.require_input(city, "puntiLuce.gpkg", "puntiLuce lamp inventory")
    output_path = paths.raw_dir(city) / f"{city}_streetlights_with_radius{variant}.gpkg"

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

    impute_mounting_height(punti, args.no_pole_height)
    punti["braccio_l_m_max"] = punti["braccio_l_m_max"].fillna(0.0)

    if "tecnologia" in punti.columns:
        punti["luminous_efficacy"] = (
            punti["tecnologia"].map(LUMINOUS_EFFICACY_MAP).fillna(DEFAULT_EFFICACY_LM_W)
        )
    else:
        punti["luminous_efficacy"] = DEFAULT_EFFICACY_LM_W

    punti["dlor"] = downward_light_output_ratios(punti)

    print(f"Calculating physics: lumens and downward intensity ({lighting.describe_law()})...")
    punti["total_lumens"] = punti["potenza_w_max"] * punti["luminous_efficacy"]
    # See the generic adapter: intensity and pole height are all step 3 reads.
    punti["downward_intensity_cd"] = lighting.downward_intensity_cd(
        punti["potenza_w_max"], punti["luminous_efficacy"], punti["dlor"]
    )

    print(f"Saving to {output_path}...")
    remove_existing(output_path)
    punti.to_file(output_path, driver="GPKG")
    print("Done.")


if __name__ == "__main__":
    main()
