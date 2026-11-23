# Route choice against Melbourne's pedestrian counters: protocol

Fixed before any run of this series is scored. A change after scoring starts is recorded at the end
with its date and reason.

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
| A1 convergence | 7 core scenarios x {1,000, 2,000, 4,000, 8,000} trips x 5 seeds | 20 | trips per run: the smallest N at which the seed SD of tau-b is under a third of the smallest scenario gap of interest (0.02) |
| A2 elements | 15 scenarios x {uniform, floor-area} origin-destination weighting x 10 seeds at N | 20 | which models beat the null; whether OD weighting changes the ranking (it is an input, never tuned) |
| A3 screening | Morris, 10 parameters (`MORRIS_PARAMETERS`), 20 trajectories, 4 levels, one seed per trajectory; seed noise from A2 | 220 | which parameters move tau-b beyond seed noise (mu* x 2/3 over seed SD > 2) |
| A4 calibration | for the two best element models: Latin hypercube over the influential parameters (<= 4), 100 points x 3 seeds, fitted leave-one-block-out | 300 per model | calibrated values and their out-of-block score |
| A5 validation | default, calibrated and null models, 20 seeds, scored on: held-out blocks; weekday all-day, Saturday and Sunday targets; 2026 targets (Jan-Sep, not used before) | 60 | the reported result |

Morris ranges are the plausible span either side of each default; no source sets them and the
paper says so.

## What is reported

A table of tau-b (and rho) by model with intervals; the paired difference against the shortest path
overall and by block; Morris mu* and sigma per parameter against the seed-noise line; calibrated
values with the out-of-block score; residual maps (log ratio of shares per counter) for the null and
the best model; and the A5 robustness scores. Scripts: `cityimage_sweep.py` (design, run, score,
morris) and the stage launchers under `gdsl1:~/runs/pub_melbourne/`.

## Threats to validity

The model has no clock, so only the spatial pattern is tested, never volume or time. The counters
sit in and around the CBD, so a model good there may not be good elsewhere. Origins and destinations
are synthetic (uniform or by floor area), and where trips start shapes the flows as much as how
they are routed; that is why OD weighting is varied and reported, not fitted.

## Changes after scoring began

**15 Nov 2026, during A1 (no A2-A5 score seen): how A4 and A5 are carried out.**

- A4: one hypercube per model (`design lhs`), seeds 101-103 at every point, so the trips are
  common across points. Per fold, a Gaussian-process emulator of the training tau-b (every
  replicate an observation); the fold's point is its maximum, and the held-out block is never used
  to choose it (`cityimage_calibrate.py`). Reported with each point: the near-optimal range, the
  sample within one seed SD of the best, so a parameter the counters do not identify shows as
  wide rather than as a value; and the emulators' 10-fold cross-validation.
- A5 (`design validate`, seeds from 201): the 60 runs above, plus each model at each fold's point
  for 5 seeds, so the held-out-block score is measured at a point that block did not choose
  (2 models x 4 folds x 5 = 40 runs).
- 2026 targets built with `build_calibration.py profiles --year 2026` (101 counters, 252 days to
  2026-09-09). How far the targets themselves move, tau-b against the 2025 weekday 07-19 target:
  2026 weekday 07-19 0.925, 2025 weekday all day 0.964, Saturday 0.756, Sunday 0.741. The 2026
  test is therefore mostly one of stability; the weekend one is a different pattern.

**15 Nov 2026, after A1 was scored and before any A2-A5 score: route lengths, barriers, Sobol.**

- *A1 outcome.* No N met the rule: the largest seed SD of tau-b across the seven scenarios was
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
  The 900-2,700 m arm is kept as the comparison with earlier work: A2 at 900-2,700 m had started
  from `~/pub` and runs to completion; A2 with VISTA lengths runs from `~/pub2`. Both are reported;
  route length, like OD weighting, is never tuned. `minRouteLength` leaves the Morris screen.
  The seed SD at N = 8,000 under VISTA lengths is read from A2's ten seeds and reported.
- *Barriers.* The cost factors and the barrier types were constants; they are now parameters
  (`RouteChoicePars.barrierType`, `preferenceNaturalBarriers`, `aversionSeveringBarriers`,
  `naturalBarrierFactor` 0.70, `severingBarrierFactor` 1.30), defaults unchanged.
  New stage **A2b** (`design barriers`, 60 runs, VISTA lengths, floor-area OD, seeds 1-10 as in
  A2): the four barrier scenarios with sub-goals from positive, negative or separating barriers
  only (ALL is A2), and, with ALL, with neither cost term, only the park-and-water discount, or
  only the road-and-rail premium (both is A2). It separates what sub-goals do from what the cost
  terms do.
- *A3.* Morris over 11 parameters: the ten minus `minRouteLength`, plus `naturalBarrierFactor`
  (0.3-1.0) and `severingBarrierFactor` (1.0-2.0), the ends of the survey mapping used by the
  learning module. 20 trajectories, 240 runs, VISTA lengths. Scenarios: `ROAD_DISTANCE`,
  `ANGULAR_CHANGE`, and `LANDMARKS_*` and `REGION_BARRIER_*` under both heuristics, so every
  parameter acts on a scenario under each heuristic; all fifteen would double the cost of a
  240-run screen. A4 calibrates a model on the parameters influential in the screened scenario
  holding its mechanisms.
- *Sobol.* First-order and total indices for each A4 model, computed on the "all" emulator's mean
  (2^13 base samples, Saltelli 2010 / Jansen 1999), with a 5-95% interval over 20 emulators
  refitted on resampled design points, and the white-noise share of the score's variance beside
  them (`cityimage_calibrate.py`, `sobol.csv`). No runs beyond A4's.
- *Lengths realised* (1,000 trips, seed 1, `ROAD_DISTANCE` at `perceptionErrorSD=0`, shortest
  path in m, p10/p25/p50/p75/p90/p95): uniform OD 131/239/427/696/995/1,393; floor-area OD
  198/294/469/704/1,059/1,355; VISTA 101/223/416/696/1,052/1,373. Under floor-area weighting the
  shortest trips come out longer, because the nearest 30 nodes with buildings can lie well off
  the drawn length; above the median both match. Code: `~/pub2` at `2779e93`
  (`~/runs/pub/PROVENANCE_pub2`); A2L, A2b and A3 run in sequence from `stage_a2L.sh`.
