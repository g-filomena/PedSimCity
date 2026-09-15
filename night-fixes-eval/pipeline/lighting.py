"""Street-lighting photometry, in one place.

Both step-2 adapters computed the same three lines of physics from their own copies of the
same constants, and step 3 carried a third copy of the threshold. This is that physics, once.

## The law

A street lamp is treated as a **Lambertian** point source above a horizontal surface: intensity
falls off the axis as I(theta) = I0 * cos(theta), where I0 is the on-axis (straight-down)
intensity. Combined with the inverse-square law and the cosine tilt correction that converts
intensity along the ray into illuminance on a horizontal surface:

    E = I(theta) * cos(theta) / D**2,  where D**2 = h**2 + d**2 and cos(theta) = h / D

    => E = I(theta) * h / (h**2 + d**2)**1.5,  and I(theta) = I0 * cos(theta) = I0 * h / D

    => E = I0 * h**2 / (h**2 + d**2)**2

`d` is the **horizontal** lamp-to-point distance, not the slant distance; the geometry is already
in the formula and applying it to a slant distance would double-count the tilt.

Intensity comes from the lamp's rated power:

    lumens     = power_W * efficacy_lm_per_W
    I0_down_cd = lumens * utilization_factor / pi

The division by pi is exact for a Lambertian hemisphere: the on-axis intensity of a flux `F`
spread Lambertian-fashion over a hemisphere is `F / pi`. This module's falloff law
(`illuminance_lux`, below) previously applied the *isotropic* falloff (E = I*h/(h^2+d^2)^1.5,
`I` held constant across angle) to an intensity that was already the Lambertian I0 -- the two
have to agree on one emission model, and they didn't. The isotropic law's angular term
underestimates fall-off with angle (cos^1 total vs. the Lambertian cos^2 in the combined formula
above, since the tilt correction contributes a second cosine on top of the source's own), so it
overstates illuminance away from directly underneath a lamp. Fixed here; same ~4.6x divergence at
the tail this replaces (register finding A3). A per-luminaire IES/LDT photometric file, if one
ever becomes available, is still the more accurate replacement for either approximation.

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
    """On-axis (straight-down) Lambertian intensity in candela, from rated power."""
    return (power_w * efficacy_lm_per_w * utilization_factor) / np.pi


def illuminance_lux(intensity_cd, height_m, horizontal_distance_m):
    """Illuminance on a horizontal surface from a Lambertian point source.

    `intensity_cd` is I0, the on-axis intensity from `downward_intensity_cd` (see module
    docstring for the derivation of E = I0 * h^2 / (h^2+d^2)^2 -- the Lambertian cos(theta)
    falloff combined with the inverse-square law and the cosine tilt correction).
    """
    return (intensity_cd * height_m**2) / (
        (height_m**2 + horizontal_distance_m**2) ** 2
    )


def distance_at_lux(intensity_cd, height_m, target_lux):
    """Horizontal distance at which one lamp's illuminance falls to `target_lux`.

    Inverts `illuminance_lux` for `d`: E = I0*h^2/(h^2+d^2)^2 => h^2+d^2 = sqrt(I0*h^2/E) =>
    d = sqrt(sqrt(I0*h^2/E) - h^2). Returns 0 where the lamp never reaches the target even
    directly underneath it.
    """
    term = np.sqrt((intensity_cd * height_m**2) / target_lux) - np.power(height_m, 2.0)
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
