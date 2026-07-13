"""Step 2 (generic adapter): assumed physics for bare lamp-point locations.

For cities where only street-light pole / lamp-point LOCATIONS are available, with no
technical attributes: every lamp gets the same assumed physics (constants below).

Reads <City>_streetlights.gpkg (from inputData/<City>/ or the resources folder) and
writes the intermediate <City>_streetlights_with_radius.gpkg to inputData/<City>/
(the single step-2 output name step 3 consumes, whichever adapter produced it).

`--city` is the city name (folder under inputData/ and resources/, and file prefix).
"""

from __future__ import annotations

import argparse

import geopandas as gpd
import numpy as np

import paths


# Assumed lamp physics, applied identically to every point (no per-lamp data).
ASSUMED_POWER_W = 100.0
ASSUMED_HEIGHT_M = 9.0
ASSUMED_EFFICACY_LM_W = 70.0
ASSUMED_UTILIZATION = 0.4
E_MIN_LUX = 5.0


def to_points(gdf: gpd.GeoDataFrame) -> gpd.GeoDataFrame:
    """Reduce any input geometry to points (centroids for non-points), dropping empties."""
    gdf = gdf[gdf.geometry.notnull() & ~gdf.geometry.is_empty].copy()
    non_point = gdf.geometry.type != "Point"
    if non_point.any():
        gdf.loc[non_point, "geometry"] = gdf.loc[non_point, "geometry"].centroid
    return gdf


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Assign assumed physics to bare lamp-point locations."
    )
    parser.add_argument("--city", required=True,
                        help="City name: folder under inputData/ and src/main/resources/, "
                             "and the <City>_* file prefix.")
    args = parser.parse_args()
    city = args.city

    lamps_path = paths.require_input(city, "streetlights.gpkg", "lamp-point locations")
    output_path = paths.raw_dir(city) / f"{city}_streetlights_with_radius.gpkg"

    print(f"city: {city}")
    print(f"Loading lamp-point locations: {lamps_path.name}")

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

    lamps["radius"] = lamps["radius_m"]
    # Luminous INTENSITY in candela. Illuminance is computed per 2 m
    # sample point in step 03 (calculated_lux); this per-lamp value is intensity.
    lamps["intensity_cd"] = lamps["downward_intensity_cd"]

    if output_path.exists():
        output_path.unlink()

    print(f"Saving to {output_path}...")
    lamps.to_file(output_path, driver="GPKG")
    print("Done.")


if __name__ == "__main__":
    main()
