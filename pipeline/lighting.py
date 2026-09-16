"""Street-lighting photometry, in one place.

Both step-2 adapters computed the same three lines of physics from their own copies of the
same constants, and step 3 carried a third copy of the threshold. This is that physics, once.

## The law

A street lamp is treated as a point source above a horizontal surface. For an emitter whose
intensity does not vary with angle, the illuminance at a point `d` metres away horizontally, from
a height `h`, is the inverse-square law with the cosine correction for the surface tilt:

    E = I * cos(theta) / D**2,  where D**2 = h**2 + d**2 and cos(theta) = h / D

    => E = I * h / (h**2 + d**2)**1.5

`d` is the **horizontal** lamp-to-point distance, not the slant distance; the geometry is already
in the formula and applying it to a slant distance would double-count the tilt.

Intensity comes from the lamp's rated power:

    lumens    = power_W * efficacy_lm_per_W
    I_down_cd = lumens * DLOR / normalisation

## What belongs in that middle factor, and what used to be there

**`DLOR`, the downward light output ratio**: the fraction of *lamp* lumens that leaves the
*luminaire* downward. It is a defined photometric quantity - EN 13032-1 measures the luminaire
under standardised conditions, and LOR = luminaire flux / lamp flux, split into DLOR and ULOR by
hemisphere - so a number in this slot can be right or wrong about a real fixture.

What used to be there was a **utilisation factor**, 0.3-0.6 by free-text optics label. That is a
different quantity: utilance is the fraction of luminaire flux landing on the *carriageway*, and it
depends on mounting height, road width and overhang. The propagation law above already computes how
much light reaches a given point, so multiplying by a utilance as well charges the geometry twice
and under-states every lux value by roughly the ratio of the two. The slot wants an emission
property, not a delivery one.

Two things fix DLOR for Turin rather than leaving it to judgement:

- **Upward flux is regulated to nothing.** Regione Piemonte L.R. 31/2000, Allegato A punto 1(a) (as
  amended by L.R. 3/2018) requires maximum intensity between 0 and 0.49 cd per 1000 lm at gamma
  >= 90 degrees, so ULOR ~ 0 for any compliant installation and DLOR ~ LOR.
- **A LED luminaire's photometry is the luminaire's.** Its rated efficacy is already a
  luminaire efficacy, so LOR is 1.0 by convention and DLOR ~ 1.0. A discharge lamp's efficacy is the
  bare lamp's, and the optic around it is what costs: a road reflector behind flat glass keeps most
  of it, a decorative lantern much less.

**The per-class values are still engineering judgement**, and step 2 says so where it sets them.
What changed is that they are judgements about a defined quantity with a regulatory bound on one
side, instead of a label-to-number table with no quantity behind it at all.

## The falloff law is a choice, and the three options do not agree

The normalisation and the propagation exponent belong together and for a long time did not.
`FALLOFF_LAW` names the pairing in force, so a lux value always says which physics produced it:

- `"mixed"` (the default, and what every lighting layer in the repo was built with) divides the
  flux by `pi` - the on-axis intensity of a **Lambertian** emitter - and then propagates it with
  the **isotropic** law `E = I h / D**3`. Those two do not belong together: it uses a Lambertian's
  peak intensity in every direction, so it over-states illuminance away from directly underneath.
- `"lambertian"` is that emitter carried through: `E = I0 h**2 / D**4`, one cosine for the
  emitter's own falloff and one for the surface tilt. It narrows the summation radius sharply.
- `"isotropic"` keeps `E = I h / D**3` and pays for it with the honest `F / (2 pi)` normalisation.
  A uniform halving, which still moves what counts as lit against the fixed `MIN_LUX` line.

**None of the three is correct**, because a real cobra-head luminaire is neither a Lambertian nor
an isotropic point source; a per-luminaire IES/LDT photometric file is what would settle it.

**The three were run over Torino's 44,278 edges on 16 September 2026 and `"isotropic"` was chosen.**

| law | pct_unlit mean / median | fully unlit | fully lit | mean_lux | edges under 5 lux | radius |
|---|---|---|---|---|---|---|
| mixed | 11.1 / 0.0 | 7.1% | 81.6% | 25.57 | 9.4% | 112.5 m |
| isotropic | 24.3 / 0.0 | 11.4% | 56.3% | 12.72 | 18.5% | 88.7 m |
| lambertian | 29.3 / 12.1 | 13.1% | 44.5% | 14.80 | 20.5% | 67.7 m |

Two things the table says that an argument would not have. **`isotropic` is `mixed` with the honest
divisor and nothing else**: same propagation, so the same spatial pattern, and `mean_lux` exactly
halved - the difference between them is a level, not a shape. **`lambertian` is a different
shape**: it has the highest `mean_lux` of the two consistent laws and still the worst `pct_unlit`,
because `cos(theta)` piles light under the pole and takes it from the mid-span between two poles -
which is precisely what `min_lux` and `pct_unlit` exist to measure, and backwards for a cobra-head,
whose whole design is lateral throw.

So `isotropic` keeps the shape the repo's layers already had and pays the honest `F / (2 pi)` for
it. The level it gives up was never the falloff law's to set: that is `DLOR` above, which moved in
the opposite direction in the same change.

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


# Which pairing of normalisation and propagation exponent is in force. Chosen 16 September 2026 by
# running all three over Torino and comparing pct_unlit; see the table in the module docstring. It
# is "isotropic" because that is the consistent version of the propagation every layer in this repo
# was already built with - the same shape, with the normalisation that shape actually implies.
FALLOFF_LAW = "isotropic"

# divisor turning luminous flux into on-axis intensity, and the exponent of D in the propagation.
# mixed:      Lambertian normalisation (F/pi) with isotropic propagation - physically inconsistent.
# lambertian: Lambertian throughout,  E = I0 * h**2 / D**4.
# isotropic:  isotropic throughout,   E = I  * h    / D**3, flux spread over the lower hemisphere.
_LAWS = {
    "mixed": (np.pi, 3),
    "lambertian": (np.pi, 4),
    "isotropic": (2.0 * np.pi, 3),
}


LAW_CHOICES = sorted(_LAWS)


def set_law(name: str) -> None:
    """Put a falloff law in force for this process.

    The point of being able to say it at the command line is that the choice between the three is
    to be settled by running them over the same edges and comparing the `pct_unlit` distributions.
    The committed `FALLOFF_LAW` is still what a run without the flag uses, so changing the physics
    for good remains an edit with a re-run behind it. Note that the law sets the flux
    normalisation as well as the propagation exponent, so step 2 and step 3 must be told the same
    one: a step-3 run over a step-2 output built under another law mixes two physics.
    """
    global FALLOFF_LAW
    if name not in _LAWS:
        raise ValueError(f"Unknown falloff law {name!r}; expected one of {LAW_CHOICES}.")
    FALLOFF_LAW = name


def _law():
    """The (normalisation, exponent) pair for the configured law."""
    try:
        return _LAWS[FALLOFF_LAW]
    except KeyError:
        raise ValueError(
            f"Unknown FALLOFF_LAW {FALLOFF_LAW!r}; expected one of {sorted(_LAWS)}."
        ) from None


def describe_law() -> str:
    """One line naming the physics in force, for a step to print beside its numbers."""
    normalisation, exponent = _law()
    divisor = "pi" if abs(normalisation - np.pi) < 1e-12 else "2*pi"
    if exponent == 4:
        formula = "E = I0 * h**2 / D**4"
    else:
        formula = "E = I * h / D**3"
    return f"falloff law: {FALLOFF_LAW} (I = F/{divisor}, {formula})"


def downward_intensity_cd(power_w, efficacy_lm_per_w, dlor):
    """Downward (on-axis) luminous intensity in candela, from rated power.

    `dlor` is the downward light output ratio - the share of lamp lumens the luminaire emits
    downward - and NOT a utilisation factor. See the module docstring: the propagation law already
    works out how much light reaches a point, so a factor describing how much lands on the road
    would charge the same geometry twice.
    """
    normalisation, _ = _law()
    return (power_w * efficacy_lm_per_w * dlor) / normalisation


def illuminance_lux(intensity_cd, height_m, horizontal_distance_m):
    """Illuminance on a horizontal surface from one lamp, under the configured falloff law."""
    _, exponent = _law()
    slant_squared = height_m**2 + horizontal_distance_m**2
    if exponent == 4:
        # Lambertian: one cosine for the emitter's own falloff, one for the surface tilt.
        return (intensity_cd * height_m**2) / slant_squared**2
    return (intensity_cd * height_m) / slant_squared**1.5


def distance_at_lux(intensity_cd, height_m, target_lux):
    """Horizontal distance at which one lamp's illuminance falls to `target_lux`.

    Inverts `illuminance_lux` for `d`. Returns 0 where the lamp never reaches the target even
    directly underneath it.
    """
    _, exponent = _law()
    if exponent == 4:
        slant_squared = np.sqrt((intensity_cd * np.power(height_m, 2.0)) / target_lux)
    else:
        slant_squared = np.power((intensity_cd * height_m) / target_lux, 2.0 / 3.0)
    return np.sqrt(np.clip(slant_squared - np.power(height_m, 2.0), a_min=0.0, a_max=None))


def summation_radius_m(intensity_cd, height_m):
    """How far to look for lamps when summing illuminance at a point.

    The distance at which the strongest lamp in the inventory drops below `NEGLIGIBLE_LUX`. One
    radius for the whole city rather than one per lamp: the search is a spatial query that has to
    be issued at a single radius, and using the strongest lamp's reach means no lamp is ever
    truncated while it still contributes.
    """
    reach = distance_at_lux(intensity_cd, height_m, NEGLIGIBLE_LUX)
    return float(np.nanmax(reach)) if np.size(reach) else 0.0
