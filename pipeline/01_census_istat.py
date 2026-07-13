"""Step 1: build the census layer for PedSimCity — ISTAT adapter (Italian cities).

This script is **ISTAT-specific**: it reads the raw Italian census-section layer
<city>_censusData_raw.gpkg (sezioni di censimento with the ISTAT ``P*`` population
variables) and writes the enriched <city>_censusData.gpkg the Java side loads.

The census is **population structure only** — it drives home spawning, the absolute
population size and the night module's vulnerability sampling. Destination
attraction (work/leisure/night places) is *not* a census matter: it comes from the
OSM use tags on `<City>_POIs.gpkg` and the buildings layer, both produced by
`00_city_preparation.py`.

The *output* schema is country-neutral — it is the contract any national census
adapter must produce (a future UK adapter, e.g. 01_census_uk.py from ONS output
areas, would emit the same columns from different inputs):
- residence_pct       share of the city's residents living in the zone
- residents           absolute per-zone headcount
- vulnerability_pct   share of residents who are female, under 15, or 65+ (counted once each)
- retiree_pct         share of the zone's ADULT (15+) residents aged 65+   (optional)
- student_pct         share of the zone's ADULT (15+) residents aged 15-24 (optional)

The last two condition the activity model's persona mix on where agents live (elderly
neighbourhoods spawn more retirees); they are emitted only when the raw census carries
the ISTAT age bands (P14-P29) and the Java side falls back to global persona shares
without them.

The raw census is read from `inputData/<City>/` (or the resources folder) and the
enriched output is written to `src/main/resources/<City>/`; the raw file is left
untouched, so this step is safe to re-run.

`--city` is the city name (folder under inputData/ and resources/, and file prefix).
"""

from __future__ import annotations

import argparse

import geopandas as gpd
import pandas as pd
from shapely.geometry import MultiPolygon

import paths


# ---------------------------------------------------------------------------
# ISTAT-specific input contract: the raw layer must carry these census variables
# (see <City>_census_metaData.xlsx, sheet metaDataCensus).
# ---------------------------------------------------------------------------

# Total resident population of the census section.
ISTAT_POPULATION_COL = "P1"

# Vulnerability: the vulnerable set is the union  all females U males under 15 U males 65+.
# Taking every female once, then only the *male* children and *male* elderly, avoids
# double-counting the female children / female elderly already inside "all females".
#   P3           females (all ages)
#   P30,P31,P32  males aged <5 / 5-9 / 10-14   -> males under 15
#   P43,P44,P45  males aged 65-69 / 70-74 / >74 -> males 65+
# NOTE: the ISTAT census-section dataset has no disability variable, so disability cannot
# be included here; add its column(s) if a future dataset provides them.
ISTAT_VULNERABLE_COLS = ["P3", "P30", "P31", "P32", "P43", "P44", "P45"]

# Age structure (both sexes, 5-year bands): P14 <5 ... P29 75+. Used to condition the
# activity model's persona mix per zone; optional — skipped when the bands are absent.
ISTAT_UNDER15_COLS = ["P14", "P15", "P16"]   # 0-4, 5-9, 10-14
ISTAT_STUDENT_AGE_COLS = ["P17", "P18"]      # 15-19, 20-24
ISTAT_RETIREE_AGE_COLS = ["P27", "P28", "P29"]  # 65-69, 70-74, 75+


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Build the unified census layer for PedSimCity from raw ISTAT "
                    "census sections (Italian cities only)."
    )
    parser.add_argument("--city", required=True,
                        help="City name: folder under inputData/ and src/main/resources/, "
                             "and the <City>_* file prefix.")
    args = parser.parse_args()
    city = args.city

    # Raw ISTAT census in, enriched census out. Keeping them as distinct files leaves the raw
    # source (with all the P* columns) intact, so this step stays re-runnable and non-destructive.
    raw_census = paths.require_input(city, "censusData_raw.gpkg", "raw ISTAT census")
    output_path = paths.resources_dir(city) / f"{city}_censusData.gpkg"

    print(f"city      : {city}")
    print(f"raw census: {raw_census}")
    print(f"output    : {output_path}")

    gdf = gpd.read_file(raw_census)
    if gdf.empty:
        raise ValueError(f"Raw census layer is empty: {raw_census}")

    print("CRS:", gdf.crs)
    if ISTAT_POPULATION_COL not in gdf.columns:
        raise KeyError(
            f"Required ISTAT population column {ISTAT_POPULATION_COL} not found in census "
            "data. This adapter expects Italian census sections; other countries need "
            "their own adapter producing the same output schema."
        )

    pop = pd.to_numeric(gdf[ISTAT_POPULATION_COL], errors="coerce").fillna(0.0)
    total = pop.sum()
    gdf["residence_pct"] = (pop / total) if total > 0 else 0.0
    # Absolute per-zone residents (P1). Java sums this on import to drive the population when present,
    # so the sampling fraction applies to the real census headcount instead of a user-entered number.
    gdf["residents"] = pop.round().astype("int64")

    missing_vulnerable_cols = sorted(set(ISTAT_VULNERABLE_COLS) - set(gdf.columns))
    if missing_vulnerable_cols:
        raise ValueError(
            "Missing required ISTAT vulnerability census columns in "
            f"{raw_census}: {missing_vulnerable_cols}. "
            "Expected columns are: P3 all females; P30/P31/P32 male under-15; "
            "P43/P44/P45 male 65+."
        )

    vulnerable = sum(
        (
            pd.to_numeric(gdf[c], errors="coerce").fillna(0.0)
            for c in ISTAT_VULNERABLE_COLS
        ),
        start=pd.Series(0.0, index=gdf.index),
    )
    gdf["vulnerability_pct"] = (vulnerable / pop.replace(0, pd.NA)).fillna(0.0).clip(0.0, 1.0)

    # Optional persona shares from the age bands: share of the zone's ADULT (15+) residents
    # who are 65+ (retirees) or 15-24 (student age). Children are excluded from the
    # denominator because the simulation population is adults.
    age_cols = ISTAT_UNDER15_COLS + ISTAT_STUDENT_AGE_COLS + ISTAT_RETIREE_AGE_COLS
    if all(c in gdf.columns for c in age_cols):
        def _col_sum(cols):
            return sum(
                (pd.to_numeric(gdf[c], errors="coerce").fillna(0.0) for c in cols),
                start=pd.Series(0.0, index=gdf.index),
            )

        adults = (pop - _col_sum(ISTAT_UNDER15_COLS)).clip(lower=0.0)
        adults_safe = adults.replace(0, pd.NA)
        gdf["retiree_pct"] = (
            (_col_sum(ISTAT_RETIREE_AGE_COLS) / adults_safe).fillna(0.0).clip(0.0, 1.0)
        )
        gdf["student_pct"] = (
            (_col_sum(ISTAT_STUDENT_AGE_COLS) / adults_safe).fillna(0.0).clip(0.0, 1.0)
        )
        print("age structure found (P14-P29): per-zone retiree_pct / student_pct emitted")
    else:
        print("no full age structure (P14-P29): persona shares stay global in the model")

    gdf = gdf.reset_index(drop=True)

    n_res = int((gdf["residence_pct"] > 0).sum())
    print(f"{len(gdf)} zones | residents={int(total)} | residential zones={n_res}")

    keep = [
        "censusZoneID",
        "censusZoneTypeID",
        "residence_pct",
        "residents",
        "vulnerability_pct",
        "retiree_pct",
        "student_pct",
        "geometry",
    ]
    keep = [c for c in keep if c in gdf.columns]
    out = gdf[keep].copy()

    # Repair invalid geometries. geopandas/GDAL tolerate them, but GeoMason's GeoPackage importer
    # returns a null geometry for an invalid feature and then NPEs on read — which drops the whole
    # census layer in the Java sim. buffer(0) fixes invalid polygons in place, keeping them polygonal.
    invalid_mask = ~out.geometry.is_valid
    n_invalid = int(invalid_mask.sum())
    if n_invalid:
        fixed = out.loc[invalid_mask, "geometry"].buffer(0)
        # buffer(0) can return a single Polygon; keep the layer uniformly MultiPolygon.
        fixed = fixed.apply(lambda g: MultiPolygon([g]) if g.geom_type == "Polygon" else g)
        out.loc[invalid_mask, "geometry"] = fixed
        print(f"repaired {n_invalid} invalid geometr{'y' if n_invalid == 1 else 'ies'} for MASON")

    if output_path.exists():
        output_path.unlink()
    out.to_file(output_path, driver="GPKG")

    print(f"saved: {output_path}")
    print(out.drop(columns="geometry").describe())


if __name__ == "__main__":
    main()
