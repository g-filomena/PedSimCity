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

import lighting
import paths


# Assumed lamp physics, applied identically to every point (no per-lamp data).
ASSUMED_POWER_W = 100.0
ASSUMED_HEIGHT_M = 9.0
ASSUMED_EFFICACY_LM_W = 70.0
# Downward light output ratio, not a utilisation factor - see pipeline/lighting.py. 0.80 is the rung
# the Turin adapter gives a road fixture whose technology its inventory does not determine, which is
# the same state of knowledge a bare lamp point leaves you in.
ASSUMED_DLOR = 0.80


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
    parser.add_argument("--falloff-law", choices=lighting.LAW_CHOICES, default=lighting.FALLOFF_LAW,
                        help="Photometric falloff law for this run. It sets the flux "
                             "normalisation, so step 3 must be given the same one. "
                             "See pipeline/lighting.py.")
    parser.add_argument("--variant", default="",
                        help="Suffix for this run's output (e.g. 'lambertian'), so a falloff-law "
                             "comparison does not overwrite the canonical step-2 file.")
    args = parser.parse_args()
    city = args.city
    lighting.set_law(args.falloff_law)
    variant = f"_{args.variant}" if args.variant else ""

    lamps_path = paths.require_input(city, "streetlights.gpkg", "lamp-point locations")
    output_path = paths.raw_dir(city) / f"{city}_streetlights_with_radius{variant}.gpkg"

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
        f"efficacy={ASSUMED_EFFICACY_LM_W} lm/W, DLOR={ASSUMED_DLOR})..."
    )

    lamps["potenza_w_max"] = ASSUMED_POWER_W
    lamps["altezza_palo_m"] = ASSUMED_HEIGHT_M
    # Every height here is assumed; the Turin adapter distinguishes measured from imputed, and
    # step 3 reads this column to say how much of a city's lighting rests on an assumption.
    lamps["altezza_source"] = "assumed"
    lamps["luminous_efficacy"] = ASSUMED_EFFICACY_LM_W
    lamps["dlor"] = ASSUMED_DLOR
    lamps["total_lumens"] = lamps["potenza_w_max"] * lamps["luminous_efficacy"]
    # The only per-lamp quantity step 3 reads, along with the pole height. The radius columns this
    # step used to emit were read by nothing: step 3 sums illuminance and so needs a negligible-
    # contribution cutoff, not a per-lamp service-level reach. See pipeline/lighting.py.
    print(f"  {lighting.describe_law()}")
    lamps["downward_intensity_cd"] = lighting.downward_intensity_cd(
        lamps["potenza_w_max"], lamps["luminous_efficacy"], lamps["dlor"]
    )

    if output_path.exists():
        output_path.unlink()

    print(f"Saving to {output_path}...")
    lamps.to_file(output_path, driver="GPKG")
    print("Done.")


if __name__ == "__main__":
    main()
