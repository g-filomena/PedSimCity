"""Step 2 (generic): assumed physics for bare lamp-point locations.

For cities where only street-light pole / lamp-point LOCATIONS are available,
with no technical attributes. Reads <City>_streetlights.gpkg (City = the resources
folder name) and writes <City>_streetlights_with_radius.gpkg carrying the columns
step 03 needs (downward_intensity_cd, altezza_palo_m, radius_m), computed from
fixed assumed defaults applied uniformly to every lamp.

Assumptions (edit the constants below to retune). These mirror the global
fallback path of the Turin adapter, so downstream steps 03/04 are identical for
both pipelines:
    power       = 100 W
    height      = 9 m
    efficacy    = 70 lm/W
    utilization = 0.4
"""

from __future__ import annotations

import argparse
import os
from pathlib import Path

import geopandas as gpd
import numpy as np

# Assumed lamp physics, applied identically to every point (no per-lamp data).
ASSUMED_POWER_W = 100.0
ASSUMED_HEIGHT_M = 9.0
ASSUMED_EFFICACY_LM_W = 70.0
ASSUMED_UTILIZATION = 0.4
E_MIN_LUX = 5.0


def first_existing(paths: list[Path], label: str) -> Path:
    for path in paths:
        if path.exists():
            return path
    joined = "\n  ".join(str(p) for p in paths)
    raise FileNotFoundError(f"Could not find {label}. Tried:\n  {joined}")


def to_points(gdf: gpd.GeoDataFrame) -> gpd.GeoDataFrame:
    """Reduce any input geometry to representative points (centroids for non-points)."""
    gdf = gdf[gdf.geometry.notnull() & ~gdf.geometry.is_empty].copy()
    non_point = gdf.geometry.type != "Point"
    if non_point.any():
        gdf.loc[non_point, "geometry"] = gdf.loc[non_point, "geometry"].centroid
    return gdf


def main() -> None:
    parser = argparse.ArgumentParser(description="Assign assumed physics to bare lamp-point locations.")
    parser.add_argument("--input_dir", required=True)
    args = parser.parse_args()

    input_dir = Path(os.path.abspath(args.input_dir))
    city = input_dir.name

    lamps_path = first_existing(
        [input_dir / f"{city}_streetlights.gpkg"],
        "lamp-point locations",
    )
    output_path = input_dir / f"{city}_streetlights_with_radius.gpkg"

    print(f"Loading lamp-point locations: {lamps_path}")
    lamps = gpd.read_file(lamps_path)
    if lamps.empty:
        raise ValueError(f"Lamp-point layer is empty: {lamps_path}")

    lamps = to_points(lamps)
    if lamps.empty:
        raise ValueError(f"No usable point geometries in: {lamps_path}")

    print(
        f"Applying assumed physics to {len(lamps)} lamps "
        f"(power={ASSUMED_POWER_W} W, height={ASSUMED_HEIGHT_M} m, "
        f"efficacy={ASSUMED_EFFICACY_LM_W} lm/W, utilization={ASSUMED_UTILIZATION})..."
    )

    lamps["potenza_w_max"] = ASSUMED_POWER_W
    lamps["altezza_palo_m"] = ASSUMED_HEIGHT_M
    lamps["luminous_efficacy"] = ASSUMED_EFFICACY_LM_W
    lamps["utilization_factor"] = ASSUMED_UTILIZATION

    lamps["total_lumens"] = lamps["potenza_w_max"] * lamps["luminous_efficacy"]
    lamps["downward_intensity_cd"] = (lamps["total_lumens"] * lamps["utilization_factor"]) / np.pi

    intensity = lamps["downward_intensity_cd"]
    height = lamps["altezza_palo_m"]
    radius_term = np.power((intensity * height) / E_MIN_LUX, 2.0 / 3.0) - np.power(height, 2.0)
    lamps["radius_m"] = np.sqrt(np.clip(radius_term, a_min=0.0, a_max=None))

    # Backward compatibility with the Turin adapter's output columns.
    lamps["radius"] = lamps["radius_m"]
    lamps["lux"] = lamps["downward_intensity_cd"]

    if output_path.exists():
        output_path.unlink()
    print(f"Saving to {output_path}...")
    lamps.to_file(output_path, driver="GPKG")
    print("Done.")


if __name__ == "__main__":
    main()
