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
