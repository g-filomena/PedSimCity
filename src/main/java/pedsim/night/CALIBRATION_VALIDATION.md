# Walking after dark in Turin: sensitivity, calibration and validation protocol

Fixed before any run of this series is scored. A change after scoring starts is recorded at the end
with its date and reason.

## Stages and their names

Two phases: **calibration** - Variability (seed noise and scale), Sensitivity (Morris screening), Calibration - and **validation**, against data the
calibration never saw. Until 27 Nov 2026 the stages were called B1-B4, and the files of their runs
on `gdsl1` (`~/runs/pub_night/`) keep those prefixes: Variability = `B1` (`B1_noise.txt`), Sensitivity =
`B2`, Calibration = `B3` (`design_B3.csv`, `calib_B3/`, `stage_b3*.sh`), Validation = `B4`
(`design_B4_*.csv`, `b4/`, `night_b4.py`). `paper/main-night.tex` still uses the codes. The scripts
are in `analysis/validation/Torino/`.

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
| Variability | defaults, winter Wednesday, 5 seeds at 5%, 10% and 20% of residents (and the 10 Nov runs: 3 seeds at 50%) | 15 runs | the scale for Sensitivity and Calibration: the smallest share at which the seed SD is under 0.002 for the women's change (M3) and under 0.01 for the night rho (M1) |
| Sensitivity | Morris, 12 factors, 10 trajectories, 4 levels, winter Wednesday, common seed per trajectory | 130 runs | influential parameters per measure, read as mu* x 2/3 over seed SD (> 2 counts) |
| Calibration | Latin hypercube over the influential parameters (<= 6), 80 points, February and July, Wednesday and Sunday; Gaussian-process emulator per target; history matching (implausibility < 3; variance = seed variance + emulator variance + a model discrepancy of 10% of the observed value) | 320 runs, + 1 wave | the non-implausible region and the calibrated point |
| Validation | calibrated and default, 4 months x Wed+Sun x 5 seeds at 20% | 80 runs | M1-M4 on April and October (held out) and against Cuebiq; the twins' A/B gap at the calibrated weights |

**Calibration targets, fixed now:** M1 rho, day and night, total; M2 rho; M3 women's change;
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

**15 Nov 2026, Variability: no share met the noise rule; Sensitivity and Calibration run at 20%.** Seed SD over 5 seeds
(winter Wednesday, defaults):

| share | women's change | night rho | day rho | centre ratio, day | dim-10 lx gap |
|---|---|---|---|---|---|
| 5% | 0.0018 | 0.022 | 0.0032 | 0.18 | 0.0042 |
| 10% | 0.0056 | 0.018 | 0.0018 | 0.14 | 0.0038 |
| 20% | 0.0050 | 0.012 | 0.0013 | 0.14 | 0.0044 |

The women's change rule (< 0.002) is not met at 20%; the 5% figure is a five-seed estimate and
not credible against the two larger shares. The 10 Nov runs at 50% gave ~0.0015 over three seeds,
but at 50% Sensitivity and Calibration (450 runs) would take over a week of the shared server. 20% is the largest
share affordable, as the rule's fallback stated. What follows: the Morris reading is already
relative to seed SD, so the screening stays valid with a wider noise line; in Calibration the seed variance
measured here enters the implausibility, so the women's change will constrain less (a set within
about 2 points of the observed change is not ruled out). Measured, not assumed.

**The cell rho rises with the share** (night 0.58, 0.63, 0.68 at 5, 10, 20%; 0.72 at 50%): with
fewer agents a cell's simulated share is noisier and the rank correlation is attenuated. Rho is
therefore compared only between runs at one share, and Validation reports it at 20% beside the 50% runs.

**24 Nov 2026, Sensitivity scored (17 Nov); Calibration wave 1 launched.** Effects as mu* x 2/3 over Variability's seed SD at
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

Calibration therefore runs on these four (`stage_b3.sh`): Latin hypercube, 80 points x {February, July} x
{Wednesday, Sunday} = 320 runs at 20%, one seed per point (20261300 + point), the other eight at
their defaults; `~/pub` at `e4187e6`, Sensitivity's tree, checked unchanged (no class file newer than
`PROVENANCE`). History matching by `night_calibrate.py` with Variability's five 20% seeds as the seed
variance. `darknessRatio` is not added: no target constrains it, and Validation reports the twins' gap at
the calibrated point with the night weights at their defaults. Started 24 Nov 11:16 UTC, alongside
Melbourne's Local-Landmark Sensitivity, as Sensitivity ran alongside Melbourne's Comparison; at Sensitivity's pace about five days. The second wave is decided
on wave 1's non-implausible region.

**25 Nov 2026, Calibration wave 1 continued on faster code.** At 13:55 UTC, with 75 of the 320 runs done and
8 in progress (which finish on `~/pub`), the 237 left moved to `~/perf/b3`: `e4187e6` plus
`DestinationIndex` (destination choice from per-purpose arrays) and `--exportRoutes=false`, with
Calibration's own dependencies and GeoMason jar (`~/pub/cp.txt`) and resources compared equal. Reason: run
time only. Before the change `lhs_016_summer_wed` was repeated on the new tree, and its
`edge_volume.csv`, `trips.csv` and street volumes are byte-identical to the original's. Design,
seeds, factors and scoring are unchanged; scoring reads only `edge_volume.csv`. Driver:
`stage_b3_fast.sh`; ends with `DONE_B3` in `SUMMARY` as before.

**25 Nov 2026, Calibration's code corrected before any Calibration run was scored.** Three points in
`night_calibrate.py`, found while writing it up; no Calibration score existed when they were changed, and the
runs are not affected.

1. *Seed variance.* The emulator's predictive variance included its white-noise term, which is the
   seed variance as the emulator estimates it, and Variability's was added to it. The emulator's variance is
   now that of its mean, so the seed enters once, as Variability's variance, which is what *Stages* states.
   Variability's is the variance of one Wednesday, while Calibration scores a Wednesday and a Sunday weighted 5:2;
   the white noise is written beside it in `emulator_checks.csv`, so the two can be compared.
2. *Discrepancy of the centre ratio.* It was 10% of the logarithm of the observed ratio, 17-19% of
   the ratio. It is now 0.10 on the log, 10% of the observed ratio.
3. *Targets.* M3 and M4 are matched. M1 and M2 were listed among the targets, but a rank correlation
   has no observed value to match, and its ceiling depends on the share simulated (Variability). They rank
   the points that are not ruled out, as the code already did; they rule none out.

With the first two the non-implausible region can only be smaller.

**25 Nov 2026, Calibration wave 1 moved to faster code a second time.** At 15:54 UTC, with 91 of the 320 runs
done and 5 in progress (which finish on the trees they started on), the 224 left moved to
`~/perf/b3w`: `~/perf/b3` plus `WorkplaceChoice` reading its weights from arrays; same dependencies
and resources, compared equal. Run time only: `lhs_016_summer_wed` repeated on it is byte-identical
to the original in `edge_volume.csv`, `trips.csv` and street volumes. Driver: `stage_b3_w.sh`, which
scores and calibrates with the corrected `night_calibrate.py` and ends with `DONE_B3` in `SUMMARY`.

**25 Nov 2026, Validation's default arm started** (16:08 UTC, `stage_b4_default.sh`, `~/perf/b3w`, 40 runs: 4
months x Wed+Sun x seeds 20261400-20261404 at 20%). It does not depend on Calibration. It is run and not
scored until the calibrated point is fixed, so the held-out months stay unseen.

**25 Nov 2026, Calibration's seed variance: Validation's default sets instead of Variability.** Decided before any Calibration or Validation run
was scored. Variability's is the variance of one winter Wednesday; a Calibration score combines a Wednesday and a
Sunday, in winter and in summer. The implausibility takes the variance between the five seeds of
Validation's default arm in winter and in summer, scored as Calibration's sets are, averaged over the two seasons
(8 degrees of freedom). Those are calibration months; Validation's spring and autumn stay unscored. Driver:
`stage_b3_finish.sh` (the runs go on unchanged), seed scores in `scores_seed.csv`. This replaces
what point 1 above says about Variability.

**25 Nov 2026, autumn is 29 October and 2 November 2025 instead of 15 and 19 October.** Two thirds
of the observed window (15 October - 15 November 2025) follow the end of summer time, with sunset
around 17:10; on the days first planned it is 18:35. The new Wednesday and Sunday are at the middle
of the window, with sunset around 17:15. No autumn run of Calibration or Validation had started, and Cuebiq has no
autumn. The other seasons keep their days: in spring and summer the sunset is inside the window's
range; in winter (12 and 16 February, window 15 January - 15 February) it is about 20 minutes later
than at the window's middle. The year does not matter otherwise: the model uses a date for
daylight, the weekday, the month's tourist nights and the draw of rain. Validation's default arm was
relaunched with the new design.

**25 Nov 2026, two rules fixed before any Calibration or Validation run was scored.**

- *Second wave of Calibration.* It is run when the points not ruled out are more than none and under 25% of
  the space, and each of the six matched emulators has at least 90% of its leave-one-out errors
  within two standard deviations. At 25% or more the data constrain too little for a second wave to
  help, and the region is reported as it is. With none left, the point of lowest implausibility is
  reported and the model is said not to match the observations. The 25% is a choice of ours.
- *Outcome of Validation.* The calibrated model holds outside the data it was fitted to if, for every
  measure on the held-out seasons (M1 by day and night, M2, the error of M3, the absolute log error
  of M4 by day and night) and on Cuebiq (rho by day and by night), its mean over five seeds is not
  worse than the default's by more than the default's seed standard deviation, and if the
  implausibility of M3 and M4 at the calibrated runs' mean, with Calibration's seed and discrepancy variances
  and no emulator variance, stays under 3. Each comparison is reported, not the verdict alone.
  `night_b4.py` computes it.

**26 Nov 2026, Calibration wave 1 scored; no second wave.** All 320 runs and the 20 seed runs ended by 01:45
UTC; scored and calibrated at 09:32 (`calib_B3.txt`, `calib_B3/`). Not ruled out: 13,692 of 200,000
points, **6.8%** of the space. Leave-one-out, share of errors within two standard deviations, for
the six matched emulators: summer M3 0.925, M4 day 0.95, M4 night 0.95; winter M3 **0.8875** (71 of
80), M4 day 0.925, M4 night 0.9125. The winter M3 emulator misses the 90% by one point, so the rule
above is not met and no second wave is run; the region is reported as it is. Its LOO z-scores have
an SD of 1.13, so that emulator is slightly overconfident and the region it constrains is, if
anything, slightly too small. Per output, the share not ruled out is 42% (summer) and 50% (winter)
for M3, 88-100% for M4; M3 is what constrains. Region (5-95%): `distanceWeight` 0.00082-0.00229,
`walkShareHalfDistance` 1,038-2,125 m, `emptyStreetWeight` 0.25-3.85, `emptyStreetRatio` 0.04-0.47.
Calibrated point, the best M1/M2 fit among the points not ruled out (fit 0.753): `distanceWeight`
0.000968, `walkShareHalfDistance` 2,142 m, `emptyStreetWeight` 0.510, `emptyStreetRatio` 0.362;
its largest implausibility is 2.97, at the edge of the region. Validation's calibrated arm (40 runs) and
the twins (30 runs) started at 10:32 UTC on `~/perf/b3w` (`stage_b4_calibrated.sh`), which scores
both arms, applies the Validation rule and writes `DONE_B4` in `SUMMARY`.

**26 Nov 2026, Validation scored: the rule is not met (18 of 24 checks).** The calibrated arm and the twins
ended by 13:07 UTC (`b4.txt`, `b4/rule.csv`, `b4/summary.csv`, `b4/table.tex`, `twins.txt`,
`twins.csv`). Mean over five seeds at 20%, default against calibrated, and the checks that fail:

| season | check | default | calibrated | worse by | limit |
|---|---|---|---|---|---|
| winter | C rho, day | 0.9419 | 0.9407 | 0.0012 | 0.0010 |
| spring | C rho, day | 0.9377 | 0.9365 | 0.0012 | 0.0009 |
| spring | M4 abs. log error, night | 0.052 | 0.103 | 0.051 | 0.021 |
| autumn | M4 abs. log error, night | 0.089 | 0.204 | 0.115 | 0.028 |
| autumn | M3 abs. error | 0.0150 | 0.0192 | 0.0041 | 0.0009 |
| autumn | M3 implausibility | | 3.90 | | 3.0 |

The other 18 hold. What the calibrated point does, read off `b4/table.tex`:

- *Better everywhere.* M2 diurnal rho (winter 0.86 to 0.91, summer 0.58 to 0.66, spring 0.80 to
  0.82, autumn 0.80 to 0.85) and the daytime centre ratio (default 6.7-7.1, calibrated 6.4-6.9,
  observed 5.2-5.7), which stays above the observed in every season, as Calibration found (never below 5.9).
- *Worse everywhere.* The night centre ratio: default 4.6-4.8, calibrated 4.1-4.6, observed 4.8-5.4.
  The point that spreads the day out of the centre spreads the night too, and the night was already
  too little concentrated. It fails on both held-out seasons; winter, a calibration month, moves
  from 0.04 to 0.14 in log error as well.
- *The women's change moves the same way in every season*, 0.3-0.5 points more negative (winter
  -3.8 to -4.3, summer -1.2 to -1.4, spring -2.1 to -2.5, autumn -4.1 to -4.5; observed -3.2,
  -2.7, -3.3, -2.6). The default already amplifies the seasons, too strong in winter and autumn and
  too weak in spring and summer, and a shift common to all seasons cannot correct that: it helps
  spring and summer and harms autumn, where the implausibility crosses 3.
- *M1 and Cuebiq unchanged.* M1 rho moves by +0.001 to +0.004, Cuebiq by -0.001 by day and +0.004
  to +0.008 by night. The two Cuebiq failures are a difference of 0.001 against a seed SD of 0.001.

Two checks are degenerate: the default's five autumn seeds give the same diurnal rho (0.800), and
the calibrated's five spring seeds do too (0.817), because a rank correlation over nine bands takes
few values. The autumn M2 limit is therefore 0, and any worsening would have failed; the calibrated
is better there, so it holds.

*Twins* (`night_twins.py`, `~/perf/b3w`, Torino_sidewalks, 21 December and 21 June, 4,000 pairs,
seeds 20261501-20261505, perception error 0). The vulnerable twin relative to the other, mean and
standard error over seeds:

| arm | season | same route % | length % | legs % | < 5 lx % | < 10 lx % | parks, water % |
|---|---|---|---|---|---|---|---|
| zero | winter | 100.0 | 0 | -1.7 +/- 0.4 | 0 | 0 | 0 |
| zero | summer | 100.0 | 0 | -0.7 +/- 0.8 | 0 | 0 | 0 |
| default | winter | 81.4 +/- 0.3 | +1.3 | -5.8 +/- 1.4 | -7.0 +/- 1.6 | -5.9 +/- 0.2 | -8.1 +/- 1.1 |
| default | summer | 82.7 +/- 0.3 | +1.2 | -4.2 +/- 0.8 | -6.7 +/- 1.0 | -5.3 +/- 0.4 | -11.1 +/- 1.2 |
| calibrated | winter | 74.4 +/- 0.5 | +1.4 | -4.4 +/- 0.6 | -5.6 +/- 0.7 | -6.1 +/- 0.4 | -10.4 +/- 1.4 |
| calibrated | summer | 77.4 +/- 0.5 | +1.4 | -5.1 +/- 0.8 | -7.3 +/- 1.1 | -5.7 +/- 0.5 | -11.6 +/- 1.3 |

The zero arm keeps the calibrated activity parameters with the four night weights at 0; the
calibrated arm sets `emptyStreetWeight` 0.510 and `emptyStreetRatio` 0.362 and leaves the other night
weights at their defaults. At the calibrated point the twins' gap on lighting and on parks and water
is the default's within about one standard error; fewer matched routes coincide because matched legs
are longer (846 m against 600 m in winter: the lower `distanceWeight`), while the relative extra
length is the same. The gaps are measured on matched legs, which are 41% of the vulnerable twin's
legs at zero weights, 26% at the defaults and 35% at the calibrated point, so each arm is measured
on a different subset of legs. The zero arm's routes are identical, as the check requires, but in
winter the vulnerable twin makes 1.7% fewer legs (summer -0.7 +/- 0.8; the centreline network gave
+1.3 +/- 0.7). To be checked before the twins enter the paper as a controlled experiment.

**27 Nov 2026, what the zero arm's leg gap is made of.** In pairs where both twins walk, they make
the same number of legs (winter 2.370 and 2.373 a pair). The gap is in the pairs where only one
twin walks at all, 38% of pairs: in winter it is the non-vulnerable twin 3,617 times and the
vulnerable one 3,402 (z = 2.6 against an even split), in summer 3,451 and 3,328 (z = 1.5), together
z = 2.9. So the 4.6 standard errors of the leg counts over five seeds overstated it; it is weaker
than that, and it may be chance. It does not follow the step order: whichever twin steps first in a
tick (`scheduleKey`), the non-vulnerable twin walks alone more often. At zero weights the code reads
vulnerability only in `NightRouteCost` (multiplied by zero), in the volume labels and in the trip
record, so what is left to separate the twins is how they are built: the vulnerable twin through
`defineHomeWorkLocations`, the other by copying. Test (diagnostic, not a stage): 10 more winter
seeds (20261506-20261515) at zero weights, as built and with the labels swapped (`~/perf/swap`,
`~/perf/b3w` with the twin built first made non-vulnerable; only `NightPopulate.class` differs),
in `~/runs/pub_night/twin_check/`. Separately, the shared destination is copied from the twin only
while the twin's current destination is set and is not home, so after the first leg the twins
mostly choose apart: 41% of legs are matched at zero weights.

**27 Nov 2026, which parameters the paper reports.** Validation's rule is not met, so the model the paper
describes is the default; the calibrated point is reported as a sensitivity case, with its Validation
result and its twins, beside it. Decided after Validation was scored, on that result.

**27 Nov 2026, the zero arm's leg gap was chance.** On the 10 new winter seeds the twin built first
and the other walk alone in 7,137 and 7,051 pairs (z = 0.7) and make legs within 0.03% +/- 0.37%;
with the labels swapped the leg counts are identical, leg for leg. At zero weights vulnerability
changes nothing, as the code says, and the Validation zero arm's -1.7% in winter was the draw of five seeds.

**27 Nov 2026, the twins are one person for the whole day, and their runs are repeated.** Until
now each twin drew its own agenda, stays and walk decisions from its own generator, and the
destination of a leg was copied from the other only while the other's was set and not home; at
zero weights 41% of legs were matched. Now both twins of a pair take one seed for the generator
their day is drawn from and one for a separate generator their route searches draw from
(`NightAgent.shareRandomStreams`, `Agent.getRouteRandom`, `NightPopulate`), the destination is no
longer copied, and the non-vulnerable twin takes the vulnerable one's commute as car parks left
it (`ActivityAgent.copyCommuteFrom`). Every other agent draws as before (its route generator is its
own). So at equal weights the twins live the same day, and with the weights on they part only where
the weights make them: a leg one of them does not walk, or a different route. The validation is now
strict: at zero weights every leg must be matched, on the same streets. Tree `~/perf/twins`
(`~/perf/b3w`, Validation's code, with only these changes; `PROVENANCE_twins`); driver
`~/runs/pub_night/twins_v2.sh`: `mvn test`, one zero-arm smoke run that must pass, then
`night_twins.py`'s 30 runs (same arms, dates and seeds as Validation's twins), log `SUMMARY_twins2`, gaps
in `twins2.csv` / `twins2.txt`. They replace Validation's twins (`twins.csv`), which compared partly
different days. The 16 arms on the centreline network in the paper's twin table have the same
defect and are not repeated yet.

**27 Nov 2026, a statement in the paper to correct.** `paper/main-night.tex`, which still names the stages B1-B4 (the history-matching
paragraph, around line 971) says the seed variance in the implausibility is B1's (Variability's). Since the
25 Nov entry it is the variance between the five seeds of Validation's default arm in winter and summer,
averaged over the two seasons, which is what Calibration used.

**27 Nov 2026, Validation's twins with shared streams.** `twins_v2.sh` ended at 11:41 UTC (`mvn test` 140
green; smoke 6,793 legs each, all matched, same streets). Vulnerable twin against the other, mean
+/- SE over seeds 20261501-20261505, `twins2.csv` / `twins2.txt`:

| arm | day | matched % | same route % | length % | legs % | < 5 lx % | < 10 lx % | parks, water % |
|---|---|---|---|---|---|---|---|---|
| zero | 21 Dec | 100 | 100 | 0 | 0 | 0 | 0 | 0 |
| zero | 21 Jun | 100 | 100 | 0 | 0 | 0 | 0 | 0 |
| default | 21 Dec | 87.8 +/- 0.3 | 80.5 +/- 0.3 | +1.4 | -5.1 +/- 0.2 | -4.3 +/- 0.2 | -4.5 +/- 0.3 | -10.4 +/- 0.8 |
| default | 21 Jun | 89.0 +/- 0.4 | 83.2 +/- 0.3 | +1.1 | -5.4 +/- 0.3 | -4.3 +/- 0.3 | -3.7 +/- 0.2 | -8.4 +/- 0.4 |
| calibrated | 21 Dec | 88.6 +/- 0.4 | 74.2 +/- 0.4 | +1.5 | -4.6 +/- 0.2 | -6.1 +/- 0.9 | -5.7 +/- 0.5 | -14.1 +/- 0.8 |
| calibrated | 21 Jun | 89.4 +/- 0.3 | 78.2 +/- 0.3 | +1.2 | -5.1 +/- 0.4 | -5.4 +/- 0.4 | -4.9 +/- 0.3 | -12.0 +/- 0.8 |

At zero weights the twins make the same legs (7,047 in winter, 6,587 in summer) on the same
streets: the validation holds exactly. With the weights on, 88-89% of legs stay matched, against
26-35% before, and the standard errors shrink by 2 to 5 times. Two readings change against the
26 Nov table. At the default weights the lighting gap is smaller than the old twins showed (under
10 lx -4.5% / -3.7% against -5.9% / -5.3%, under 5 lx -4.3% against -7.0% / -6.7%), and the leg
gap is the same (-5.1% / -5.4%). And the calibrated point separates the twins more than the
default weights on exposure (under 10 lx 2.3 and 3.3 standard errors, parks and water 3.3 and 4.2),
not equally as the old noise suggested; legs are unchanged. It lowers both empty-street weights and
lengthens legs (matched legs 885 m against 657 m in winter); which of the two widens the gap is not
separated by these runs. The 16 arms on the centreline network follow, on the same tree
(`twins16.sh` after `twins_v2.sh`, `twins16.py`, same arms, dates 21 December / 21 June 2026, seeds
20261007-20261011 and 4,000 pairs as `~/runs/twins3`), log `SUMMARY_twins16`, table `twins16.txt`.

**27 Nov 2026, the 16 twin arms on the centreline network, with shared streams.** 80 runs on
`~/perf/twins` with Torino's resources smudged from `e4187e6` (a first attempt found LFS pointers
there and failed at populate), 11:45-13:03 UTC, all exit 0, no enclosure warning; `twins16.txt`,
`twins16.csv`, full table in `src/main/java/pedsim/night/TODO.md`. Both zero arms match every leg on
the same streets. The findings stated in the paper hold: darkness alone separates the twins on
lighting as much as all four terms (under 10 lx -5.1 +/- 0.2% against -4.7 +/- 0.2%) and saturates
(-6.8% at 4 / 2); empty streets remove legs (-3.3%, -5.2%; -4.8% with all terms); park and water
saturate (-15.8%, -5.5%, -57.9% at 1 / 0). Three numbers change beyond their old errors: every
lighting gap is smaller (all terms -4.7% against -6.1%), darkness at 4 / 2 removes legs too (-1.9 +/-
0.1%), and enclosure pulls the vulnerable twin towards parks twice as strongly (+3.0% and +7.0%) and
at 4 / 2 no longer lowers its dim metres (+0.5% against -2.0%).
