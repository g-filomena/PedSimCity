# Route choice against Melbourne's pedestrian counters: protocol

Fixed before any run of this series is scored. A change after scoring starts is recorded at the end
with its date and reason.

## Stages and their names

Two phases: **calibration** - Variability (seed noise and trips per run), Comparison (with its VISTA Comparison and Barrier
Comparison arms), Sensitivity (Morris screening; and the Local-Landmark Sensitivity), Calibration - and **validation**,
against blocks, days and a year the calibration never saw. Until 27 Nov 2026 the stages were called
A1-A5, and the files of their runs on `gdsl1` (`~/runs/pub_melbourne/`) keep those prefixes:
Variability = `A1`, Comparison = `A2` (900-2,700 m lengths), VISTA Comparison = `A2L`, Barrier
Comparison = `A2b`, Sensitivity = `A3`, Local-Landmark Sensitivity = `A3LL`, Calibration = `A4`
(`stage_a4.sh`, `SUMMARY_A4`, `calib_A4/`), Validation = `A5` (`stage_a5.sh`, `SUMMARY_A5`). The
scripts are in `analysis/validation/Melbourne/`.

## Question

Which route-choice models - shortest path, least angular change, and the element-based models
(landmarks, regions, barriers, alone and combined) - reproduce how pedestrian flow is distributed
across Melbourne's footpaths, and does any of them do so better than the shortest path?

## Data

- **Observed.** City of Melbourne Pedestrian Counting System (CC BY 4.0), hourly counts. Target per
  counter: the mean weekday count between 07:00 and 19:00 in 2025 (`sensor_targets_2025.csv`,
  `daytype=weekday`, `hours=7-19`), as a share of the total over the counters scored. Daytime,
  because a model without a clock represents the daytime pattern.
- **Counters scored.** Those linked to an edge within 25 m of the 5 Oct 2026 network
  (`sensor_edge_link.csv`, cityImage 2.2.0), minus 103 (outside the analysed area).
- **Spatial blocks.** Counters grouped into four blocks by their position (`sensor_blocks.csv`,
  `cityimage_sweep.py blocks`), so a calibrated parameter is judged on counters it was not fitted
  to.
- **Simulated.** cityImage module, *Testing Specific Route Choice Models*, Melbourne layers of
  5 Oct 2026, one shared set of origin-destination trips per seed walked once by each scenario.
  Code: the commit and GeoMason-light jar recorded in each stage's `PROVENANCE`.

## Measures

- **Primary: Kendall's tau-b** between simulated and observed shares across counters. Tie-aware,
  because counters a model never reaches tie at zero.
- Secondary: Spearman's rho, Pearson's r of shares, RMSE of shares, coverage (share of counters
  with any simulated flow).
- Every score is reported with its seed spread; a difference between two models is the paired
  difference over common seeds (same trips), with a 95% bootstrap interval over seeds.
- A model **beats the shortest path** when the paired difference in tau-b is positive and its 95%
  interval excludes zero, overall and in at least three of the four held-out blocks.

## Stages

| | design | runs | decides |
|---|---|---|---|
| Variability | 7 core scenarios x {1,000, 2,000, 4,000, 8,000} trips x 5 seeds | 20 | trips per run: the smallest N at which the seed SD of tau-b is under a third of the smallest scenario gap of interest (0.02) |
| Comparison | 15 scenarios x {uniform, floor-area} origin-destination weighting x 10 seeds at N | 20 | which models beat the null; whether OD weighting changes the ranking (it is an input, never tuned) |
| Sensitivity | Morris, 10 parameters (`MORRIS_PARAMETERS`), 20 trajectories, 4 levels, one seed per trajectory; seed noise from Comparison | 220 | which parameters move tau-b beyond seed noise (mu* x 2/3 over seed SD > 2) |
| Calibration | for the two best element models: Latin hypercube over the influential parameters (<= 4), 100 points x 3 seeds, fitted leave-one-block-out | 300 per model | calibrated values and their out-of-block score |
| Validation | default, calibrated and null models, 20 seeds, scored on: held-out blocks; weekday all-day, Saturday and Sunday targets; 2026 targets (Jan-Sep, not used before) | 60 | the reported result |

Morris ranges are the plausible span either side of each default; no source sets them and the
paper says so.

## What is reported

A table of tau-b (and rho) by model with intervals; the paired difference against the shortest path
overall and by block; Morris mu* and sigma per parameter against the seed-noise line; calibrated
values with the out-of-block score; residual maps (log ratio of shares per counter) for the null and
the best model; and the Validation robustness scores. Scripts: `cityimage_sweep.py` (design, run, score,
morris) and the stage launchers under `gdsl1:~/runs/pub_melbourne/`.

## Threats to validity

The model has no clock, so only the spatial pattern is tested, never volume or time. The counters
sit in and around the CBD, so a model good there may not be good elsewhere. Origins and destinations
are synthetic (uniform or by floor area), and where trips start shapes the flows as much as how
they are routed; that is why OD weighting is varied and reported, not fitted.

## Changes after scoring began

**26 Nov 2026, before any Calibration run: what Calibration calibrates, and the re-run before it.** Local-Landmark Sensitivity ended on
25 Nov. Its effects as mu* x 2/3 over the VISTA Comparison seed SD of tau-b (floor-area OD, 10 seeds):

| scenario | seed SD | factor | ratio | sign |
|---|---|---|---|---|
| `LOCAL_LANDMARKS_ANGULAR` | 0.0079 | `localLandmarkThresholdCommunity` | **4.0** | 0.75 (+) |
| | | `distanceNodeLandmark` | **3.7** | 0.15 |
| | | `salientNodesPercentile` | **3.0** | 0.20 |
| | | `wayfindingEasinessThresholdCommunity` | **2.6** | 0.86 (+) |
| | | `perceptionErrorSD` | 1.7 | 0.83 |
| `LOCAL_LANDMARKS_DISTANCE` | 0.0172 | `perceptionErrorSD` | **3.3** | 0.94 (+) |
| | | `salientNodesPercentile` | 1.7 | 0.77 |
| | | `distanceNodeLandmark` | 1.2 | 0.27 |
| | | `localLandmarkThresholdCommunity` | 1.0 | 0.26 |
| | | `wayfindingEasinessThresholdCommunity` | 0.8 | 0.45 |

So Calibration's hypercubes are four-dimensional for `LOCAL_LANDMARKS_ANGULAR` (the four over 2) and
one-dimensional for `LOCAL_LANDMARKS_DISTANCE` (`perceptionErrorSD` alone). Both keep the design
as specified, 100 points x seeds 101-103, ranges as in Sensitivity, VISTA lengths, floor-area OD,
N = 8,000. A one-dimensional hypercube is denser than it needs to be; it is kept, not cut, so
that the two models are calibrated on the same number of runs.

The pair is the VISTA Comparison floor-area ranking (`LOCAL_LANDMARKS_DISTANCE` 0.347, SD 0.017;
`LOCAL_LANDMARKS_ANGULAR` 0.333, SD 0.008; paired difference against `ROAD_DISTANCE` +0.068 and
+0.054, SD over seeds 0.023 and 0.015). As the 23 Nov entry requires, bare `DISTANT_LANDMARKS` is
re-run first on the length-neutral cost: VISTA Comparison's 20 runs (both weightings, seeds 1-10), that scenario
alone, from `~/pub3`, which is `~/pub2` with only `DijkstraGlobalLandmarks` replaced by its
`6dba442` version (one class differs; `~/runs/pub/PROVENANCE_pub3`). Calibration starts only if the two
best floor-area models are still the pair above; otherwise the stage stops with `HELD`. Calibration itself
runs from `~/pub2`, the code of VISTA Comparison and Local-Landmark Sensitivity, which the local-landmark models share with `~/pub3`.
Launcher `stage_a4.sh`, log `SUMMARY_A4`, calibration in `calib_A4/`; it ends with `DONE_A4`.

The re-run ended the same evening: bare `DISTANT_LANDMARKS` scores tau-b 0.026 (SD 0.014) with
floor-area OD and 0.012 with uniform OD on the length-neutral cost, against 0.0004 and 0.019 on
the old one, still last of the fifteen; Calibration started at 21:07 UTC on the pair above. Validation is chained
to it (`stage_a5.sh`, from `~/pub2`, `design validate` with 20 seeds and 5 per fold, log
`SUMMARY_A5`, ends `DONE_A5`), scored on the 2025 targets (weekday 07-19, which carries the held-out
blocks, weekday all day, Saturday, Sunday) and on the same four from 2026.

**24 Nov 2026, Sensitivity scored (240 of 240 runs); Local-Landmark Sensitivity extended to both local-landmark models.**
Effects as mu* x 2/3 over the seed SD of tau-b in VISTA Comparison (10 seeds, floor-area OD, VISTA lengths,
default parameters); `sign` is |mu| / mu*, near 1 when the effect keeps its sign. Every factor over
2, and the largest below it where none is:

| scenario | seed SD | factor | ratio | sign |
|---|---|---|---|---|
| `ROAD_DISTANCE` | 0.017 | `perceptionErrorSD` | **2.8** | 0.97 (+) |
| `ANGULAR_CHANGE` | 0.011 | `perceptionErrorSD` | 1.2 | 0.75 |
| `LANDMARKS_DISTANCE` | 0.026 | `globalLandmarknessWeightDistanceCommunity` | 1.2 | 1.00 (-) |
| | | `salientNodesPercentile` | 1.2 | 0.57 |
| `LANDMARKS_ANGULAR` | 0.006 | `localLandmarkThresholdCommunity` | **6.1** | 0.91 (+) |
| | | `wayfindingEasinessThresholdCommunity` | **5.6** | 0.95 (+) |
| | | `distanceNodeLandmark` | **4.2** | 0.70 (-) |
| | | `salientNodesPercentile` | **2.5** | 0.32 |
| `REGION_BARRIER_DISTANCE` | 0.018 | `salientNodesPercentile` | **2.1** | 0.67 |
| `REGION_BARRIER_ANGULAR` | 0.007 | `salientNodesPercentile` | **3.0** | 0.02 |
| | | `regionNavActivationThreshold` | **2.5** | 0.20 |
| | | `perceptionErrorSD` | **2.3** | 0.82 (+) |
| | | `severingBarrierFactor` | **2.3** | 0.73 (+) |

`morris_A3.csv` on gdsl1 holds every factor; a factor that a scenario's model does not read has
mu* = 0 exactly there, which is the check that the screen moved only what it should. Two readings:
the perception error *raises* the shortest path's tau-b (noise spreads flow over parallel streets,
which the counters seem to prefer), and `LANDMARKS_DISTANCE` has no factor over 2 because its seed
SD (0.026) is the largest of the six, not because nothing acts on it.

*Local-Landmark Sensitivity widened.* The VISTA Comparison ranking (floor-area OD) puts both local-landmark models first,
`LOCAL_LANDMARKS_DISTANCE` 0.347 and `LOCAL_LANDMARKS_ANGULAR` 0.333, then `LANDMARKS_DISTANCE`
0.307 and `ROAD_DISTANCE` 0.279; Calibration calibrates the two best, and Local-Landmark Sensitivity as first launched screened
only the first. It was stopped after 18 minutes (no run finished) and restarted with both
scenarios, same design points and seeds. This was decided after Sensitivity's effects were computed, on the
VISTA Comparison ranking, which was known before; the factors and ranges are unchanged from the entry below.

**24 Nov 2026, during Sensitivity (235 of 240 runs done, no Sensitivity effect computed): a second screen, Local-Landmark Sensitivity.**
The best model in VISTA Comparison, `LOCAL_LANDMARKS_DISTANCE` (tau-b 0.316), is not among Sensitivity's scenarios; Sensitivity
screens the local-landmark parameters only inside `LANDMARKS_*`, where distant landmarks also act,
so an effect there may not be the effect in the local-landmark model. Local-Landmark Sensitivity is a Morris screen of
that scenario alone (`design morris --scenarios LOCAL_LANDMARKS_DISTANCE --prefix morrisLL`), over
the five parameters that act on it: `salientNodesPercentile`, `localLandmarkThresholdCommunity`,
`distanceNodeLandmark`, `wayfindingEasinessThresholdCommunity`, `perceptionErrorSD`, ranges as in
Sensitivity, the other six at their defaults (the model has no distant landmarks, no regions and no barrier
terms). 20 trajectories, 4 levels, 120 runs, seed t + 1 for trajectory t as in Sensitivity, VISTA lengths,
floor-area OD, N = 8,000; same effect rule. Calibration takes that model's parameters from Local-Landmark Sensitivity.
Launched by `stage_a3LL.sh` once Sensitivity ends.

**23 Nov 2026, after VISTA Comparison and Barrier Comparison were scored, during Sensitivity: the cost of bare `DISTANT_LANDMARKS`.**
Its search charged a segment `(1 - landmarkness) / length`, so long segments were cheap and
routes ran 2.1x the shortest path at the median (90th percentile 5x, maximum 90x); it scored
tau-b 0.010 in VISTA Comparison. The cost is now `1 - landmarkness` per segment, independent of length. Only
that scenario changes, and it is not in the Sensitivity Morris set. Its Variability and VISTA Comparison scores are on the old
cost and are reported as such; VISTA Comparison is re-run for it before Calibration.

**15 Nov 2026, during Variability (no Comparison-to-Validation score seen): how Calibration and Validation are carried out.**

- Calibration: one hypercube per model (`design lhs`), seeds 101-103 at every point, so the trips are
  common across points. Per fold, a Gaussian-process emulator of the training tau-b (every
  replicate an observation); the fold's point is its maximum, and the held-out block is never used
  to choose it (`cityimage_calibrate.py`). Reported with each point: the near-optimal range, the
  sample within one seed SD of the best, so a parameter the counters do not identify shows as
  wide rather than as a value; and the emulators' 10-fold cross-validation.
- Validation (`design validate`, seeds from 201): the 60 runs above, plus each model at each fold's point
  for 5 seeds, so the held-out-block score is measured at a point that block did not choose
  (2 models x 4 folds x 5 = 40 runs).
- 2026 targets built with `build_calibration.py profiles --year 2026` (101 counters, 252 days to
  2026-09-09). How far the targets themselves move, tau-b against the 2025 weekday 07-19 target:
  2026 weekday 07-19 0.925, 2025 weekday all day 0.964, Saturday 0.756, Sunday 0.741. The 2026
  test is therefore mostly one of stability; the weekend one is a different pattern.

**15 Nov 2026, after Variability was scored and before any Comparison-to-Validation score: route lengths, barriers, Sobol.**

- *Variability outcome.* No N met the rule: the largest seed SD of tau-b across the seven scenarios was
  0.038, 0.033, 0.019 and 0.017 at 1,000-8,000 trips, against 0.0067. The stage took the largest,
  **N = 8,000**. A single run therefore cannot resolve a difference below about 0.05; every
  comparison below is the paired difference over the common seeds, as already specified.
- *Route lengths are an input, fixed from a source.* The OD matrix drew walked lengths between 900
  and 2,700 m, a range no Melbourne source sets. VISTA (2022-25, weekday walking stages starting
  and ending in the City of Melbourne, departing 07:00-18:59, n = 2,308, weighted) has a median of
  416 m and a 90th percentile of 1,052 m, so that range left out about three quarters of the
  walking the counters see. The primary arm now draws each trip's walked length from that
  distribution (`--routeLengths=walkLengths`, `pipeline/vista_walk_lengths.py`; the destination is
  drawn from the 30 nodes whose shortest-path distance from the origin is nearest the length, so
  no circuity factor enters). `vistadist` is taken as a
  walked length: the document does not define it, and the median implied speed is 4.0 km/h.
  The 900-2,700 m arm is kept as the comparison with earlier work: Comparison at 900-2,700 m had started
  from `~/pub` and runs to completion; Comparison with VISTA lengths runs from `~/pub2`. Both are reported;
  route length, like OD weighting, is never tuned. `minRouteLength` leaves the Morris screen.
  The seed SD at N = 8,000 under VISTA lengths is read from Comparison's ten seeds and reported.
- *Barriers.* The cost factors and the barrier types were constants; they are now parameters
  (`RouteChoicePars.barrierType`, `preferenceNaturalBarriers`, `aversionSeveringBarriers`,
  `naturalBarrierFactor` 0.70, `severingBarrierFactor` 1.30), defaults unchanged.
  New stage **Barrier Comparison** (`design barriers`, 60 runs, VISTA lengths, floor-area OD, seeds 1-10 as in
  Comparison): the four barrier scenarios with sub-goals from positive, negative or separating barriers
  only (ALL is Comparison), and, with ALL, with neither cost term, only the park-and-water discount, or
  only the road-and-rail premium (both is Comparison). It separates what sub-goals do from what the cost
  terms do.
- *Sensitivity.* Morris over 11 parameters: the ten minus `minRouteLength`, plus `naturalBarrierFactor`
  (0.3-1.0) and `severingBarrierFactor` (1.0-2.0), the ends of the survey mapping used by the
  learning module. 20 trajectories, 240 runs, VISTA lengths. Scenarios: `ROAD_DISTANCE`,
  `ANGULAR_CHANGE`, and `LANDMARKS_*` and `REGION_BARRIER_*` under both heuristics, so every
  parameter acts on a scenario under each heuristic; all fifteen would double the cost of a
  240-run screen. Calibration calibrates a model on the parameters influential in the screened scenario
  holding its mechanisms.
- *Sobol.* First-order and total indices for each Calibration model, computed on the "all" emulator's mean
  (2^13 base samples, Saltelli 2010 / Jansen 1999), with a 5-95% interval over 20 emulators
  refitted on resampled design points, and the white-noise share of the score's variance beside
  them (`cityimage_calibrate.py`, `sobol.csv`). No runs beyond Calibration's.
- *Lengths realised* (1,000 trips, seed 1, `ROAD_DISTANCE` at `perceptionErrorSD=0`, shortest
  path in m, p10/p25/p50/p75/p90/p95): uniform OD 131/239/427/696/995/1,393; floor-area OD
  198/294/469/704/1,059/1,355; VISTA 101/223/416/696/1,052/1,373. Under floor-area weighting the
  shortest trips come out longer, because the nearest 30 nodes with buildings can lie well off
  the drawn length; above the median both match. Code: `~/pub2` at `2779e93`
  (`~/runs/pub/PROVENANCE_pub2`); VISTA Comparison, Barrier Comparison and Sensitivity run in sequence from `stage_a2L.sh`.
