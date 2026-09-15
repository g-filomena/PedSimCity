# Python pipeline fixes: A1, A2, A3, A4 (occlusion half), A7, A8, B1 (horizon half)

Until 15 Sep this environment had no Python interpreter, so every pipeline-tier finding in the
register (A1–A5, A7, A8, B1's horizon half) could only ever be a code-review recommendation, not a
tested fix — unlike the Java changes above, which were shadow-compiled and, for six of them, run
end to end on real Torino data (see `VALIDATION.md`). Python is now installed
(`C:\Users\tripo\AppData\Local\Programs\Python\Python314`, not yet on this session's PATH — call
it by full path or refresh the shell), and `pipeline/` is a real, runnable Python project
(`geopandas`, `pandas`, `numpy`, `shapely`, `scipy`), so these are now proposed **and tested
against real Torino data**, the same way the Java fixes were.

## What's here

| Finding | File | What changed |
|---|---|---|
| **A3** | `lighting.py` | The falloff law now matches the Lambertian intensity it's already computing. `illuminance_lux()` was `E = I·h/(h²+d²)^1.5` (isotropic — I constant across angle) applied to an intensity that's already `I0 = F/π` (the Lambertian on-axis value). Fixed to `E = I0·h²/(h²+d²)²`, the correct combination of the source's own `cos(θ)` falloff and the surface-tilt cosine correction. `distance_at_lux()` (and therefore `summation_radius_m()`) re-derived to match. Verified by round-trip: `distance_at_lux` → `illuminance_lux` reproduces the target lux exactly. |
| **A2** | `02_street_lights_torino.py` | Mounting height now grouped by `tipo_supporto` (support type), not `uso_ottica` (optics type) — the category error the register flagged: `altezza_palo_m` is 100% absent for every non-pole support type, so a pole-height median was being assigned to fixtures with no pole. New `assign_heights()`: poles get their measured height, else the same-street (`via`) median of other measured poles, else the global pole median; non-poles get an assumed height by support type (stated as an assumption, not derived — see the code's own comment on `NON_POLE_HEIGHT_M`), else the same global pole fallback. Every row gets a `height_source` column recording which. |
| **A1** | `03_street_lights.py` | A lamp inside a building's own footprint no longer occludes itself. New `lamp_own_building()` finds which building (if any) each lamp sits inside via `sjoin(predicate="within")` and excludes exactly that building from the occlusion test for sight lines from that lamp. Also switched the occlusion test from `.intersects()` to `.crosses()`, so a sight line that only grazes a building's edge or corner (shares boundary, not interior) no longer counts as blocked. |
| **A4** (occlusion half) | `03_street_lights.py` | Occlusion now checks height, not just the 2D footprint. For a building whose footprint the sight line crosses, the ray's height at the *nearest* crossing point (height rises monotonically from 0 at the sample point to the lamp's height) is compared against the building's real `height`; the building only blocks if the ray is still below the roofline there. Canopy half (street trees) is **not** fixed — no canopy data exists for Torino; documented as a stated limitation in the function's own docstring, per the register's own recommendation not to approximate it. |
| **A7** | `02_street_lights_torino.py` | Docs only. `potenza_w_max` is a maximum rated power; no dimming/maintenance-factor field exists anywhere in the inventory to discount it with, so every lamp is modelled new and at full power. Not fixable from this data — stated explicitly rather than silently assumed. |
| **A8** | `03_street_lights.py` | Missing/absent `altezza_palo_m` now fails loudly (`KeyError`/`ValueError`, naming the count of affected rows) instead of silently filling `9.0`, matching the guard immediately below it for `downward_intensity_cd` — the inconsistency the register flagged. Safe now that A2 guarantees step 2 never emits a null height; this guard only fires for a file that skipped or predates that guarantee. |
| **B1** (horizon half) | `04_directional_lighting.py` | `VISIBILITY_HORIZON_M`: `12.0` → `15.0`, matching Fotios, Yang & Uttley (2015)'s recommended observation distance exactly (10.3 m measured, 15 m recommended). The noisy-statistic half of B1 (`MIN` → `MEAN`) is the Java-side fix already on this branch (`NightPars.directionalLuxStatistic`). |

**Not attempted**: A5 (lamp-arm offset — only 7.2% of lamps have `braccio_l_m_max` recorded, and
computing a real per-lamp offset needs a street-relative bearing this data doesn't carry cleanly)
and A4's canopy half (no data exists). Both are the register's own **low** priority tier ("document,
or tidy when convenient" / "not fixable from the data available") — deferred, not overlooked.

## How this was tested

Isolated from the tracked repo, the same way the Java fixes were validated in `VALIDATION.md`: a
sibling directory, `PedSimCity-pipeline-test/`, with its own `inputData/Torino/` and
`src/main/resources/Torino/` populated from **copies** of the real tracked inputs (`paths.py`
resolves everything relative to wherever the script file lives, so this is a complete, genuine
isolation — nothing in the tracked repo's resources was read from or written to).

- **A2** (`assign_heights`): run against the full real 99,742-lamp `Torino_puntiLuce.gpkg`.
  Output: 54,395 measured, 6,220 same-street median, 31,435 support-type assumed, 7,692 global
  pole median. Global pole median computed as exactly 9.0 m — matching the register's own cited
  figure for this inventory.
- **A1** (`lamp_own_building`): 5,873 of 99,742 lamps (5.9%) found to sit inside a building
  footprint via real polygon containment — lower than the register's bounding-box estimate
  (4–26%), as expected: a real polygon test is more conservative than a bbox approximation.
- **A3**: round-trip verified numerically (`distance_at_lux(illuminance_lux(x)) == x` to 6 decimal
  places) rather than run at scale — it's pure arithmetic, not spatial computation.
- **A1 + A3 + A4 combined**, smoke-tested on a real 3,000-point subset before committing to a
  full run: 152.9 points/sec, no errors, sensible output (max 206.5 lux, 156/3000 points at 0,
  matching what a real dense/sparse mix of Torino streets should look like).
- **Full-city run**: a genuine baseline-vs-fixed comparison, the same shape as `VALIDATION.md`'s
  Java one, is running in the background as this is written — ~1,076,786 sample points at 2 m
  spacing across all 44,278 Torino edges, baseline (unmodified pipeline) and fixed (this file's
  changes) both computing concurrently. At smoke-tested throughput this is on the order of two
  hours; real before/after lux numbers will be added here once both finish, the same way the Java
  validation was reported.

## Search radius, already a visible real effect

Confirmed live in both runs' own diagnostic line before either finished: baseline (isotropic
falloff) computes a 112.5 m summation radius; fixed (Lambertian) computes 67.7 m — a real ~40%
narrowing, consistent with the register's flagged "~4.6x divergence at the tail" between the two
falloff shapes. This alone confirms A3 is doing something substantial, independent of the full
run's eventual lux numbers.
