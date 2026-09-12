"""Street-lighting photometry, in one place.

Both step-2 adapters computed the same three lines of physics from their own copies of the
same constants, and step 3 carried a third copy of the threshold. This is that physics, once.

## The law

A street lamp is treated as a point source above a horizontal surface. The illuminance it
produces at a point `d` metres away horizontally, from a height `h`, is the inverse-square law
with the cosine correction for the surface tilt:

    E = I * cos(theta) / D**2,  where D**2 = h**2 + d**2 and cos(theta) = h / D

    => E = I * h / (h**2 + d**2)**1.5

`d` is the **horizontal** lamp-to-point distance, not the slant distance; the geometry is already
in the formula and applying it to a slant distance would double-count the tilt.

Intensity comes from the lamp's rated power:

    lumens    = power_W * efficacy_lm_per_W
    I_down_cd = lumens * utilization_factor / pi

The division by pi treats the downward hemisphere as a Lambertian emitter, for which the on-axis
intensity of a flux `F` spread over a hemisphere is `F / pi`. That is an approximation - a real
luminaire has a photometric distribution - and it is the approximation that most deserves replacing
if per-luminaire IES/LDT files ever become available.

## The two thresholds do different jobs

They were the same number written three times, which hid the fact that they answer different
questions:

- `MIN_LUX` is a **service level**: is this stretch of pavement adequately lit? It is not a
  modelling choice. EN 13201-2 sets maintained average horizontal illuminance by pedestrian class
  P1 15 lx, P2 10, P3 7.5, **P4 5**, P5 3, P6 2. 5 lux is class P4, the usual residential /
  pedestrian-street value, and is what `is_unlit` is measured against.
- `NEGLIGIBLE_LUX` is a **computational cutoff**: how far away can a lamp be before leaving it out
  of the sum changes nothing? This must be far smaller than `MIN_LUX`, because illuminance adds -
  ten lamps contributing 1 lux each make 10 lux, and a cutoff set at the service level would throw
  all ten away.

Using the service level as the summation cutoff is the mistake this module exists to prevent. The
old code avoided it by accident, with an undocumented flat 40 m search radius that happens to sit
near the 0.1 lux contour of a typical 100 W lamp at 9 m.
"""

from __future__ import annotations

import numpy as np


# Maintained average horizontal illuminance for pedestrian class P4, EN 13201-2 (and UNI 11248,
# which adopts it for Italy). The level at which a footway counts as lit.
MIN_LUX = 5.0

# Below this, one more lamp in the sum does not change the answer. Two orders of magnitude under
# the service level, so no realistic pile-up of distant lamps is lost.
NEGLIGIBLE_LUX = 0.05


def downward_intensity_cd(power_w, efficacy_lm_per_w, utilization_factor):
    """Downward luminous intensity in candela, from rated power."""
    return (power_w * efficacy_lm_per_w * utilization_factor) / np.pi


def illuminance_lux(intensity_cd, height_m, horizontal_distance_m):
    """Illuminance on a horizontal surface, point source, inverse square with cosine correction."""
    return (intensity_cd * height_m) / (
        (height_m**2 + horizontal_distance_m**2) ** 1.5
    )


def distance_at_lux(intensity_cd, height_m, target_lux):
    """Horizontal distance at which one lamp's illuminance falls to `target_lux`.

    Inverts `illuminance_lux` for `d`. Returns 0 where the lamp never reaches the target even
    directly underneath it.
    """
    term = np.power((intensity_cd * height_m) / target_lux, 2.0 / 3.0) - np.power(height_m, 2.0)
    return np.sqrt(np.clip(term, a_min=0.0, a_max=None))


def summation_radius_m(intensity_cd, height_m):
    """How far to look for lamps when summing illuminance at a point.

    The distance at which the strongest lamp in the inventory drops below `NEGLIGIBLE_LUX`. One
    radius for the whole city rather than one per lamp: the search is a spatial query that has to
    be issued at a single radius, and using the strongest lamp's reach means no lamp is ever
    truncated while it still contributes.
    """
    reach = distance_at_lux(intensity_cd, height_m, NEGLIGIBLE_LUX)
    return float(np.nanmax(reach)) if np.size(reach) else 0.0
