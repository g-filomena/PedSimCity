# Walking after dark in Turin: sensitivity, calibration and validation protocol

Fixed before any run of this series is scored. A change after scoring starts is recorded at the end
with its date and reason.

## Questions

1. How well does the model reproduce where and when people are on foot in Turin, by day and after
   dark, and how the share of women changes after dark?
2. Which parameters drive those outputs, and which drive the gap between women and men in how much
   of their walking is on dim streets or in parks and along water?
3. Fitted to one set of observations, does the model hold on observations it was not fitted to?

## Data

- **Calibration: Vodafone Smart Locator, February and July** (winter, summer). Passing presence
  (dwell 0-30 min) per 216 m cell and time band, by sex. All modes and indoors, so it constrains
  *shape* - diurnal, spatial, women's share - never pedestrian volume.
- **Held out, same source: Vodafone April and October** (spring, autumn).
- **Independent validation: Cuebiq walking-filtered GPS pings**, February, April and July 2025,
  geohash6 tile x hour. A different source, filtered to walking speeds, never used in fitting.
- Model inputs are never derived from either source.

## Simulated

Night module on `Torino_sidewalks` (the network with footways, as the 10 Nov 2026 validation), a
Wednesday and a Sunday per month weighted 5:2 (the month's mean day): 12 and 16 February, 9 and 13
April, 9 and 13 July, 15 and 19 October 2025. `--perceptionErrorSD` at its default (it is part of
the model here). Code: the commit and the GeoMason-light jar recorded in each stage's `PROVENANCE`;
runs from a clean worktree, no overlay. Scripts: `night_sweep.py` (designs, runs, Morris) and
`night_score.py` (all measures).

## Measures

Per season, per band, over cells with street and an observed value (`night_common.indicators`):

- **M1 spatial**: Spearman's rho of cell shares, total and by sex; misallocated share; error factor.
- **M2 diurnal**: Spearman's rho of the nine band shares, citywide.
- **M3 women after dark**: the change in women's share of passing presence from the day bands
  (06-20) to the night bands (20-06), citywide, observed against simulated. Its rho across cells is
  reported but not fitted: on the 10 Nov runs at the defaults it was ~0, noise at the cell scale.
- **M4 centre**: presence per cell within 1 km of Piazza Castello over the 4-5 km ring.
- **C Cuebiq**: Spearman's rho of tile shares per hour (day 06-20, night 20-06) and of the 24 hourly
  totals.
- Outcomes that no data can check, reported as model behaviour: the women-men gap in dark-hour
  metres under 5 and 10 lx and in parks or along water; legs per person.

## Parameters

| parameter | default | range | why it is free |
|---|---|---|---|
| `darknessWeightVulnerable` | 1.0 | 0-4 | no source sets it |
| `parkWaterWeightVulnerable` | 1.0 | 0-4 | no source |
| `enclosureWeightVulnerable` | 1.0 | 0-4 | no source |
| `emptyStreetWeightVulnerable` | 1.0 | 0-4 | no source |
| ratio non-vulnerable / vulnerable, one per term (4) | 0.5 | 0-1 | the women-men fear ratio is an assumption |
| `minReassuranceLux`, `maxReassuranceLux` | 8.9, 26 | 5-15, 15-40 | the source gives a range between locations |
| `distanceWeight` | 0.0012 | 0.0006-0.0024 | not calibrated against a trip-length distribution |
| `walkShareHalfDistance` | 1500 | 1000-2300 | English curve moved towards an Italian level |

The ratios parametrise each non-vulnerable weight as a fraction of the vulnerable one, so the
screening can tell the level of a term from the difference between the groups.

## Stages

| | design | size | decides |
|---|---|---|---|
| B1 noise | defaults, winter Wednesday, 5 seeds at 5%, 10% and 20% of residents (and the 10 Nov runs: 3 seeds at 50%) | 15 runs | the scale for B2-B3: the smallest share at which the seed SD is under 0.002 for the women's change (M3) and under 0.01 for the night rho (M1) |
| B2 screening | Morris, 12 factors, 10 trajectories, 4 levels, winter Wednesday, common seed per trajectory | 130 runs | influential parameters per measure, read as mu* x 2/3 over seed SD (> 2 counts) |
| B3 calibration | Latin hypercube over the influential parameters (<= 6), 80 points, February and July, Wednesday and Sunday; Gaussian-process emulator per target; history matching (implausibility < 3; variance = seed variance + emulator variance + a model discrepancy of 10% of the observed value) | 320 runs, + 1 wave | the non-implausible region and the calibrated point |
| B4 validation | calibrated and default, 4 months x Wed+Sun x 5 seeds at 20% | 80 runs | M1-M4 on April and October (held out) and against Cuebiq; the twins' A/B gap at the calibrated weights |

**Calibration targets (B3), fixed now:** M1 rho, day and night, total; M2 rho; M3 women's change;
M4 centre ratio (as log ratio simulated over observed); each for February and July.

## What is reported

M1-M4 by season for default and calibrated parameters, held-out months marked; the Cuebiq
comparison; Morris mu*-sigma plots per measure with the noise line; the non-implausible region as
pairwise projections; the women-men gap at the calibrated weights with seed intervals. Once fitted
to Vodafone, Vodafone February and July are no longer a check; the paper says so and rests the
validation on the held-out months and on Cuebiq.

## Threats to validity

Vodafone counts every mode and indoor presence, and its sex split is the subscriber's, not the
walker's. Cuebiq is a panel of app users and its walking filter is a speed threshold. Neither sees
a street, only a cell or a tile. The night weights can only be identified through what they change
in where and when women are present; if that change is small against noise, the calibration will
leave them wide, and that is a result.

## Changes after scoring began

Before this series, `night_score.py` was tested on the 10 Nov runs at the defaults, which is how the
cell-level women's share was found to be noise; no parameter was chosen from them.

**15 Nov 2026, B1: no share met the noise rule; B2 and B3 run at 20%.** Seed SD over 5 seeds
(winter Wednesday, defaults):

| share | women's change | night rho | day rho | centre ratio, day | dim-10 lx gap |
|---|---|---|---|---|---|
| 5% | 0.0018 | 0.022 | 0.0032 | 0.18 | 0.0042 |
| 10% | 0.0056 | 0.018 | 0.0018 | 0.14 | 0.0038 |
| 20% | 0.0050 | 0.012 | 0.0013 | 0.14 | 0.0044 |

The women's change rule (< 0.002) is not met at 20%; the 5% figure is a five-seed estimate and
not credible against the two larger shares. The 10 Nov runs at 50% gave ~0.0015 over three seeds,
but at 50% B2 and B3 (450 runs) would take over a week of the shared server. 20% is the largest
share affordable, as the rule's fallback stated. What follows: the Morris reading is already
relative to seed SD, so the screening stays valid with a wider noise line; in B3 the seed variance
measured here enters the implausibility, so the women's change will constrain less (a set within
about 2 points of the observed change is not ruled out). Measured, not assumed.

**The cell rho rises with the share** (night 0.58, 0.63, 0.68 at 5, 10, 20%; 0.72 at 50%): with
fewer agents a cell's simulated share is noisier and the rank correlation is attenuated. Rho is
therefore compared only between runs at one share, and B4 reports it at 20% beside the 50% runs.

**24 Nov 2026, B2 scored (17 Nov); B3 wave 1 launched.** Effects as mu* x 2/3 over B1's seed SD at
20% (five seeds); a factor counts when it exceeds 2 on a calibration target (M1 rho day and night,
M3 women's change, M4 centre ratio):

| factor | centre ratio, day | rho total, day | rho total, night | women's change |
|---|---|---|---|---|
| `distanceWeight` | **3.9** | **5.2** | 1.0 | 1.7 |
| `walkShareHalfDistance` | **2.7** | 1.9 | 0.9 | 1.9 |
| `emptyStreetWeight` | 0.6 | 0.6 | **2.0** | **7.5** |
| `emptyStreetRatio` | 0.1 | 0.4 | 0.6 | **8.1** |

The other eight stay between 0 and 1.4 on every target. Off the targets, `darknessRatio` moves the
twins' dim-10 lx gap (3.7) and `minReassuranceLux` reaches 2.0 there. No factor moves the diurnal
rho or Cuebiq. Signs are stable (|mu| = mu*) except `emptyStreetWeight` on the women's change
(mu -0.027 against mu* 0.056), an interaction with its ratio; on night rho it is stable.

So the lighting, park-and-water and enclosure terms are not identified by Vodafone: they change
where the twins walk relative to each other, not the pattern a cell count sees, as anticipated
under *Threats to validity*. The women's change Vodafone does see is carried by the empty-street
term, which has no source.

B3 therefore runs on these four (`stage_b3.sh`): Latin hypercube, 80 points x {February, July} x
{Wednesday, Sunday} = 320 runs at 20%, one seed per point (20261300 + point), the other eight at
their defaults; `~/pub` at `e4187e6`, B2's tree, checked unchanged (no class file newer than
`PROVENANCE`). History matching by `night_calibrate.py` with B1's five 20% seeds as the seed
variance. `darknessRatio` is not added: no target constrains it, and B4 reports the twins' gap at
the calibrated point with the night weights at their defaults. Started 24 Nov 11:16 UTC, alongside
Melbourne A3-LL, as B2 ran alongside A2; at B2's pace about five days. The second wave is decided
on wave 1's non-implausible region.

**25 Nov 2026, B3 wave 1 continued on faster code.** At 13:55 UTC, with 75 of the 320 runs done and
8 in progress (which finish on `~/pub`), the 237 left moved to `~/perf/b3`: `e4187e6` plus
`DestinationIndex` (destination choice from per-purpose arrays) and `--exportRoutes=false`, with
B3's own dependencies and GeoMason jar (`~/pub/cp.txt`) and resources compared equal. Reason: run
time only. Before the change `lhs_016_summer_wed` was repeated on the new tree, and its
`edge_volume.csv`, `trips.csv` and street volumes are byte-identical to the original's. Design,
seeds, factors and scoring are unchanged; scoring reads only `edge_volume.csv`. Driver:
`stage_b3_fast.sh`; ends with `DONE_B3` in `SUMMARY` as before.

**25 Nov 2026, B3's calibration code corrected before any B3 run was scored.** Three points in
`night_calibrate.py`, found while writing it up; no B3 score existed when they were changed, and the
runs are not affected.

1. *Seed variance.* The emulator's predictive variance included its white-noise term, which is the
   seed variance as the emulator estimates it, and B1's was added to it. The emulator's variance is
   now that of its mean, so the seed enters once, as B1's variance, which is what *Stages* states.
   B1's is the variance of one Wednesday, while B3 scores a Wednesday and a Sunday weighted 5:2;
   the white noise is written beside it in `emulator_checks.csv`, so the two can be compared.
2. *Discrepancy of the centre ratio.* It was 10% of the logarithm of the observed ratio, 17-19% of
   the ratio. It is now 0.10 on the log, 10% of the observed ratio.
3. *Targets.* M3 and M4 are matched. M1 and M2 were listed among the targets, but a rank correlation
   has no observed value to match, and its ceiling depends on the share simulated (B1). They rank
   the points that are not ruled out, as the code already did; they rule none out.

With the first two the non-implausible region can only be smaller.

**25 Nov 2026, B3 wave 1 moved to faster code a second time.** At 15:54 UTC, with 91 of the 320 runs
done and 5 in progress (which finish on the trees they started on), the 224 left moved to
`~/perf/b3w`: `~/perf/b3` plus `WorkplaceChoice` reading its weights from arrays; same dependencies
and resources, compared equal. Run time only: `lhs_016_summer_wed` repeated on it is byte-identical
to the original in `edge_volume.csv`, `trips.csv` and street volumes. Driver: `stage_b3_w.sh`, which
scores and calibrates with the corrected `night_calibrate.py` and ends with `DONE_B3` in `SUMMARY`.

**25 Nov 2026, B4's default arm started** (16:08 UTC, `stage_b4_default.sh`, `~/perf/b3w`, 40 runs: 4
months x Wed+Sun x seeds 20261400-20261404 at 20%). It does not depend on B3. It is run and not
scored until the calibrated point is fixed, so the held-out months stay unseen.

**25 Nov 2026, B3's seed variance: B4's default sets instead of B1.** Decided before any B3 or B4 run
was scored. B1's is the variance of one winter Wednesday; a B3 score combines a Wednesday and a
Sunday, in winter and in summer. The implausibility takes the variance between the five seeds of
B4's default arm in winter and in summer, scored as B3's sets are, averaged over the two seasons
(8 degrees of freedom). Those are calibration months; B4's spring and autumn stay unscored. Driver:
`stage_b3_finish.sh` (the runs go on unchanged), seed scores in `scores_seed.csv`. This replaces
what point 1 above says about B1.

**25 Nov 2026, autumn is 29 October and 2 November 2025 instead of 15 and 19 October.** Two thirds
of the observed window (15 October - 15 November 2025) follow the end of summer time, with sunset
around 17:10; on the days first planned it is 18:35. The new Wednesday and Sunday are at the middle
of the window, with sunset around 17:15. No autumn run of B3 or B4 had started, and Cuebiq has no
autumn. The other seasons keep their days: in spring and summer the sunset is inside the window's
range; in winter (12 and 16 February, window 15 January - 15 February) it is about 20 minutes later
than at the window's middle. The year does not matter otherwise: the model uses a date for
daylight, the weekday, the month's tourist nights and the draw of rain. B4's default arm was
relaunched with the new design.
