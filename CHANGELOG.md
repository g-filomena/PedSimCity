# Changelog

What changed in PedSimCity and when, most recent first. This is the summary; `bug_changelog.md` is
the companion record of individual defects — symptom, root cause, fix, severity — and indexes the
pre-September history by area rather than by date. Open work is in the per-module `TODO.md` files,
and `CLAUDE.md` explains how the code works now.

## Results these changes invalidate

Read this before trusting an older figure.

| from | what is void | why |
|---|---|---|
| any run before **12 Sep 2026** | every run-output comparison | `Engine(StateFactory)` seeded from the clock, so no two headless runs shared a seed |
| **12–13 Sep 2026**, `useDestinationChoice=false` | any A/B using it | nothing wrote `distanceNextDestination`, so every agent walked to one of the thirty nodes nearest home |
| before **13 Sep 2026** | night A/B comparisons | twins no longer share a drawn trip length — they share a destination — and an A/B twin always chooses by utility |
| before **13 Sep 2026** | cityImage and empirical per-edge volumes | `edgesWalkedSoFar` accumulated across an agent's trips, so each trip re-counted every earlier one: volumes were cumulative, not per trip |
| since **March 2026** | region- and landmark-based *distance* route comparisons | `roadDistanceSequence` returned its last leg instead of the whole sequence |

---

## September 2026

### 13 September — release, trip distance, configuration, and the GUI

**Release is a count of departures, everywhere.** The metres budget is gone from every tier including
core, and with it `releaseAgentsMeters`, `sampleTripMeters`, `capResidual`, the carried residual,
`Engine.calculateMetersCurrentDay` (and its unsourced ±10% Gaussian), `Pars.metersPerDay` and the
`TripDistanceBands` class. `TravelDemand` lost four seams no implementation needed —
`expectedTripChainLegs`, `measuredLegMetres`, `tripAcceptanceProbability`, `usesCountBasedRelease` —
and `unscheduledChainsPerPerson` became `unscheduledDeparturesPerPerson`, since a trip chain is an
activity programme's idea and core has no agenda. `metersPerDayPerPerson` is deleted: once the budget
went it had no readers at all.

**Trip distance is a *walked route length*, and node lookup is Euclidean.** `Pars.minRouteLength` /
`maxRouteLength` (900/2700) replace `min/avg/maxTripDistance` and are set directly —
`setMinMaxTripDistance()` and its invented ×0.5/×1.5 are gone, along with the dead literals they
overwrote. Every caller that picks a destination by distance now converts through
`NetworkCircuity.straightLineFor()`, the one place the division happens; core divided and the two
OD-generating modules did not, so one shared field briefly meant two different quantities.
`networkCircuityFactor` and `NetworkCircuity` moved from `ActivityPars` to core and are measured in
`Environment.prepare()`, so a bare core run measures them (1.331 Torino_simplified, 1.292 Torino).

**Per-city configuration, owned by the activity module.** `activity.parameters.CityConfig` reads
`src/main/resources/<City>/<City>.properties` before the command line; `Torino.properties` applies 24
`ActivityPars` keys. Core reaches it only through `SimulationModule.loadCityConfig`, a default no-op —
core is the machinery and has no behaviour to configure. Run switches (`useDestinationChoice`,
`seed`, `jobs`, `parallel`) are **refused with a warning**: they choose a model or an experiment, not
a place, and a city file that set one would let two cities differ in mechanism while appearing to
differ in geography. Every key is reported as applied, refused or unplaced.

**One list of parameter classes.** `SimulationModule.parameterClasses()` is consulted by both the
command line and the configuration file. Previously `initFromArgs` named core's three classes inline,
so every module key had to be picked up a second time by hand or be dropped in silence —
`--useDestinationChoice=true` was dropped that way, turning an afternoon of comparison runs into two
runs of the same code. The single-argument `initFromArgs` is deprecated.

**The Java (AWT) GUI is removed.** Fifteen classes: six `PedSimCity*Applet` frames, six panels,
`SimulationViewer` and `PedSimCityActionHandler`. Entry points are now
`pedsim.<module>.launcher.<X>Launcher`, none importing AWT, all delegating to one `ModuleLauncher`;
`pom.xml`, the READMEs and `CLAUDE.md` follow. Three things fell out with it: `ParameterManager` no
longer takes the GUI class as a parameter type (`collectParameters(PedSimCityApplet, …)` is gone);
`PedSimCity.currentInstance`, a mutable static shared across simulation threads whose only reader was
the viewer, is gone; and `ServerLauncherApplet` became `pedsim.core.server.RemoteLauncher`, taking an
argument string rather than reading a window's text fields.

**A headless run no longer builds a GUI.** `PedSimCityApplet.main` called
`new PedSimCityApplet().coreLauncher()`, and `coreLauncher()` is static — so the whole frame was
constructed to call a static method through an instance. On a JVM started with
`-Djava.awt.headless=true` the `Frame` constructor throws, so the documented headless invocation could
not have run on a display-less server.

**cityImage and empirical became real modules**, with `SimulationModule` implementations,
`parameterClasses()` and launchers going through `ModuleLauncher`. Their parameters had been
unreachable from the command line, and empirical had no headless entry point at all — its applet
opened a window and nothing else. Four defects surfaced in the process; see `bug_changelog.md`.

**Routing.** Angular-change now tries every incident dual centroid rather than the single
best-aligned one (0 fallbacks and 0 unknown dual endpoints on a Torino day, against 92/420);
an unroutable trip widens to the full network rather than being lost, counted in
`RunLedger.fullNetworkEscalations`; and `roadDistanceSequence` returns the whole sequence again
rather than its last leg. The night module was audited: the bypass cache removed, shadowed statics
removed, `roadDistanceNight` repaired, the vulnerability gating documented.

**Documentation.** Javadoc and comments across both repos were rewritten to describe current
behaviour rather than the history of how the code got there; that history lives here and in
`CLAUDE.md`. The per-module `TODO.md` files replace a single root `NEXT.md`, and the activity
module's `FUTURE_WORK.md` was folded into its `TODO.md`.

### 12 September — circuity, reproducibility, lighting, performance, module boundaries

**Network circuity was never measured.** `NetworkCircuity` discards a sampled pair whose route
reports a non-positive length, and `Route.getLength()` returned zero for every route — the field
behind it was declared and never assigned, in every version of GeoMason-light back to July 2024. So
the measurement bailed out on every run and the hardcoded fallback stood. Measured now: Barcelona
1.174, Paris 1.174, Muenster 1.231, **Torino 1.292**, Torino_simplified 1.331, Melbourne 1.543. The
fallback moved from 1.41 to 1.23, the middle of that spread. **`distanceWeight` was calibrated while
the factor sat at 1.41, so it wants re-deriving** — both under one convention, or the correction is
applied twice. Passing the factor now implies `measureNetworkCircuity = false`.

Two consequences of the zero-length routes went with it: `RouteNovelty` divided by zero and returned
infinite novelty on the normal path, and `RouteComplexity` returned zero for every route. The day
ledger also stopped swallowing unusable lengths silently — that is how a run reported "planned 0 m,
walked 0 m" every day for as long as the bug lasted.

**Runs were never reproducible.** `Engine(StateFactory)` seeded from `System.currentTimeMillis()`,
and that is the constructor every headless run reaches, so the work that derived every generator from
a base seed was deriving them from the clock. `Pars.seed` is now a real parameter, fixed at 20260912,
`--seed=-1` for a clock seed, logged at startup. One unseeded draw survived and did not look like
one: `Engine.calculateMetersCurrentDay` passed `null` as `Utilities.fromDistribution`'s *direction*
argument, reaching an overload that drew from `ThreadLocalRandom`. Verified afterwards: two runs on
one seed, byte-identical trip files.

**The activity module's 25 minutes a day was `sim.graph.Islands`, not the activity code.** Reached
once per agent from `NetworkBuilder.buildKnownNetwork`, it was quadratic in three separate places —
`findClosestPairAcrossAllIslands` enumerated every cross-island pair twice through nested
`parallelStream`s; `findConnectingBridge` built the same doubled cross product and called
`getEdgeBetween` on all of it; `findDisconnectedIslands` was a `parallelStream` whose whole body sat
inside one `synchronized` block; `dfs` built and discarded a list per visit. `mergeConnectedIslands`
was also unbounded: if A* cannot route between the closest pair, the island count never falls and the
loop never terminates — a hang rather than a crash. The activity module paid it and night barely did
because its richer `cognitiveAnchors` fragments known space into more islands.

**Street lighting became one photometry module.** `pipeline/lighting.py` holds the point-source law
once (`E = I·h/(h²+d²)^1.5`); both step-2 adapters had carried byte-identical copies. The 5-lux
constant has a source: EN 13201-2 pedestrian class **P4**, adopted in Italy by UNI 11248. The
substantive fix is that one constant was doing two jobs — `MIN_LUX` is a service level ("is this
pavement lit?"), `NEGLIGIBLE_LUX` a summation cutoff ("how far can a lamp be before omitting it
changes nothing?"). Illuminance adds, so a cutoff at the service level discards ten lamps
contributing a lux each. Derived from the inventory, the cutoff is **112.5 m**, not the undocumented
flat 40 m: over 40,000 sample points, mean illuminance 14.38 → 15.55 lux (**+13.7% where lit**) and
**1.88% of points change side of the lit/unlit threshold**. The night module routes on `mean_lux`, so
this corrects its input data.

**The city's latitude comes from the census**, not a hardcoded 53.4 — Liverpool, applied to every
city. Turin is 45.069, an 8.3° error:

| date | daylight at 53.4 (used) | at 45.069 (correct) | darkness error |
|---|---|---|---|
| 21 Jun | 03:07–20:53 | 03:47–20:13 | **1 h 20 m too little** |
| 21 Dec | 07:53–16:07 | 07:13–16:47 | 1 h 20 m too much |

The default start date is a June Monday, so **night results produced before this are short of
1 h 20 m of darkness a day** — that is the behavioural `isDark` flag, which drives the vulnerability
response, lighting-aware routing and park avoidance. Exporters aggregate on the fixed window and are
unaffected.

**Education got its own decay and curve.** `educationDistanceDecay` 1.5 against work's 1.0, and a
student commute curve of 1,200 m / 0.0019, fitted to the *study* rows of the ISTAT matrix (38.0%
walked; 87.5 / 11.1 / 0.9 / 0.5). A school catchment is not a labour market. Students came out at
**40.1%** against 38.0%, from 15.6%.

**Module boundaries.** `LearningPars` moved to `learning/parameters`; the one constant core read
became `CognitiveMap.MEAN_SPATIAL_ABILITY`, since it seeds a trait every agent has.
`CoreTravelDemand` became `BaselineTravelDemand` — it is the base class, not "the core one".
`Agent`'s working day moved to `activity.agents.CommuterAgent`: employment was a property of every
pedestrian the model can represent, including `OdAgent`, which has no day at all. The workplace
ladder existed twice and had drifted; core now exposes `residenceLadder()` / `workplaceLadder()` and a
module prepends its own rung. Dead code removed: `BarrierPreference`, `PopulationPars`, and the
unread `Pars.javaProject`, `Pars.localPath`, `RouteChoicePars.{usingDMA, maxTripsPerDay, originsTmp,
destinationsTmp}` — the last four shadowing live fields of the same name in `EmpiricalPars` and
`TestPars`.

**Two modules did not compile, and no ordinary build said so.** `cityImage` passed `random::nextInt`
to ten `NodesLookup` lookups taking a `MersenneTwisterFast`; `social`'s `Group`/`Grouping` still
declared `java.util.Random` after the import had gone. The default `night` profile excludes both
packages, so the breakage was one `-P` away. `mvn -Pall-modules compile` passes again. Separately,
`Environment` no longer re-attaches junction geometries by hand — GeoMason-light 2.2.0 does it in the
library — which also closed two holes: a junction with no segment threw, and a node with no matching
junction kept id 0 and overwrote whatever `nodesMap` held there.

**Angular-change routing now reports substitutions.** The daily line splits fallbacks by cause; on
full Torino it read **92 of 420 angular routes served as shortest path**, a rate never seen before
because the path crashed until that week.

**Verified:** a 7-day, 8,465-agent (1%) run on full Torino in 2 h 31 m, fixed seed, with walked
commute shares of 16.6% (workers) and 37.3% (students).

### 11–12 September — travel demand, and the commute calibrated against ISTAT

**Travel demand left the state class.** `AgentReleaseManager` had asked `PedSimCity` fourteen
questions — nine about travel demand, five about the run's own measurements. They became
`TravelDemand` and `RunLedger`, reached through `state.travelDemand()` and `state.ledger()`. No
behaviour changed; what changed is that the release manager can no longer reach a calibration anchor,
which is how `metersPerDayPerPerson` came to steer the departure profile months after it stopped
being the anchor.

**The commute is generated, not drawn.** `planMandatoryDeparture` gives each agent a departure minute
inside its persona's start window if it has a workplace, walks to it, and its persona attends that
day. `commuteShareOfTripChains` — which told the departure profile how often chance had obliged — is
deleted, and with it the last live reader of `metersPerDayPerPerson`.
`DepartureProfile.discretionary` lost its day-of-week parameter and takes the realised,
census-conditioned persona mix, so layer 1's data reaches the day's shape.
`walkedTripsPerPersonPerDay = 0.51` replaced `tripChainsPerPersonPerDay = 0.21`, which was the same
survey figure with a chain length of 2.4 already divided into it.

**The commute was calibrated against the ISTAT commuting matrix.** The census sections carry no
commuting variable; the *Matrice del pendolarismo* (2011) does, including flows within a
municipality. `walkShareCommuteWorker` 0.120 → **0.163** and `walkShareCommuteStudent` 0.279 →
**0.380**, Turin-specific. The substantive change is that **the walk-share curve split in two**: a
commute is not a discretionary trip and is walked far less at the same distance, and one pooled curve
for both is why the model walked 47% of work commutes where Turin walks 16.3% — no workplace
distribution could repair it, because across the whole decay range the share and the length
distribution moved in opposite directions. `CommuteCalibration` scores candidates against that data
over 20,000 census homes without simulating anything. Fitted: **β = 1.0** (was 2.0), floor **0 m**
(was 540), commute curve **800 m / 0.0015**; misfit 2.8 against 13.8 and 16.7 one step either side.
Commutes then took 404 legs of an 863-leg budget instead of exceeding it, so discretionary travel
went from *zero* to 53% of the day's trips.

## July 2026

### Fixed
- **Spatial indexing & trajectory recording** — corrected indexing/recording
  issues that produced wrong trajectories and lookups (`daf61ab`).
- **Optional dual graph & landmark data** — import now degrades gracefully when a
  city ships no dual graph or no landmark scores instead of failing (`da6a161`).
- **A/B light-testing visuals** — restored day/night sky colour, fixed lux-metric
  IDs, trail fade-out, camera-follow panning math and auto-cancel on manual
  zoom/pan, tether rendering, and neutral road colouring when lights are off
  (`bc97fc7`, `c95e331`, `7bcadcd`, `edfb26f`).
- **HTML export** — fixed a Java string-literal size-limit compilation error in
  `HtmlExporter` (`033e275`) and a template syntax error that broke the map
  canvas (`3fc6e6e`).
- **Dashboard** — removed the stale weather control and fixed blank results pages
  caused by a missing `<style>` tag (`a0ea093`).

### Changed / Performance
- **Dijkstra pathfinding** — performance improvements and RNG cleanup (`ba8ac99`).
- **Time model** — day-of-week traffic patterns added; simulation step reduced to
  5 minutes (`d519901`).
- **Per-run GIS loading** — GIS preload deferred to each run; day/night hours made
  configurable (`4938c4c`).
- **Census-driven population** — population sourcing wired through the modules
  (`895200c`).
- **A/B testing support** and a refactor of agent movement creation (`82f7364`).

### Data & build
- **`inputData/` reorganisation** — raw source data consolidated under
  `inputData/` (`2859e8e`).
- **Pipeline refactor** — added step 0, centralised paths, consolidated the
  city/census adapters; new city-preparation launcher and refactored Windows
  build scripts (`e055461`, `0442c48`, `503895c`, `db74723`).
- **Census ISTAT alignment** — the raw census layer now keeps the original ISTAT
  field names (`P*`, `SEZ21_ID`, `COD_TIPO_S`, …); the ISTAT→friendly translation
  lives in the `<City>_census_metaData.xlsx` workbook (an `EnglishFieldName` column
  on both sheets), and `01_census_istat.py` reads the raw by its ISTAT names and
  applies that mapping — so the enriched census the sim loads carries friendly names
  only, with no ISTAT codes leaking through.

### Visualisation & publishing
- **Results site on Cloudflare Pages** — `publish_site.py` stages the self-contained
  result pages into a per-city static site (a PedSimCity overview plus one `/<City>`
  sub-page each) and deploys it to Cloudflare Pages, live at
  [pedsimcity.inclusivestreets.org](https://pedsimcity.inclusivestreets.org). It
  rebuilds the staging folder each run (so removed runs drop off the site), works
  before the first run (empty-state landing), and keeps model content off the
  `inclusivestreets.org` apex.

---

## June 2026 — module restructure & night simulation

The largest month of the cycle: the codebase was split into a reusable **core**
plus runnable **modules**, the night-lighting simulation matured, and the
visualisation stack was rebuilt.

### Added / Changed (architecture)
- **Module architecture** — core decoupled from activity/night datasets; added a
  `SimulationModule` abstraction, a REST API, and a `SimulationLauncher`; core
  clearly separated from runnable simulations, with extensibility hooks and
  `NightEngine` overrides (`c0b2dea`, `20b1205`, `cbd473d`, `6bee7ac`, `da6a161`).
- **Night module** — hourly scenarios and night detection, per-edge measured lux
  and counts, directional entrance-lighting, and an illuminated-edges → graph
  join; lighting logic and lux tracking refined throughout.
- **Data pipeline** — restructured with dedicated lighting steps, Windows build
  scripts, and per-module READMEs.
- **Dashboard** — full redesign to a light "SaaS" theme with summary tabs; the
  laggy Streamlit dashboard was replaced by a fast Leaflet HTML dashboard served
  over HTTP.

### Fixed (critical)
- **OOM / memory leak** — route caches bounded with an LRU policy to stop
  out-of-memory crashes (`4ff3699`).
- **Thread-safety** — fixed static-state races from concurrent runs, ensured
  `AgentReleaseManager` is closed in a `finally`, and synchronised environment
  initialisation (`20b1205`, `e8419d4`, `9ad733e`, `e5e0afb`).
- **NullPointerExceptions** — `buildResidenceProbabilities` (`7fcc4fb`),
  `AgentMovement.edgesToAvoid` during night rerouting (`813af96`), `Agent`
  constructor light-sensitivity init (`da4504b`), and `Environment` (`28eaae3`).
- **Dijkstra / routing** — NPEs, out-of-bounds in `cleanDualPath` for short
  sequences (`676f2c5`), compilation mismatch on returned barriers type
  (`685f4b1`), night trip-skipping, and original-route-aware bypass rerouting.
- **Agents** — corrected origin placement and destination selection (`7a45519`),
  release logic (`c97790b`), and day/night travel durations plus stuck
  visualisation markers (`4a39d60`); a batch of "4 critical simulation bugs"
  (`61b88dd`).
- **Dashboard accuracy** — zero-volume bug fixed by snapshotting `volumesMap`
  before clearance (`e53e27e`), agent colouring and live-lux display corrected
  (`793a6ac`), road-layer overlap ordering (`edf1e90`), and constant road
  thickness to avoid distortion (`3653de6`).
- **Live telemetry** — served via HTTP with CORS and load-time polling
  (`75a8f5a`).
- **GUI parameters** — days/population/% now actually applied on Run (`e843c12`,
  `ae40df6`).
- **Torino paths** — city name corrected to `Torino` and illuminated-edges path
  moved to the standard `cityName` prefix (`c185d9e`); Torino GIS restored to Git
  LFS with a pre-commit guard (`170c21c`).

---

## March–May 2026 — earlier stabilisation

Groundwork that made the Torino night runs usable and the engine
concurrency-safe.

### Fixed
- **Landmark navigation** — initialise heuristics for safe access and skip local
  landmark logic when a city (e.g. Torino) has no landmark setup, which had been
  crashing navigation (`474f9dc`, `334f9b9`).
- **Census & centroids** — hardened census loading and made centroid handling
  null-safe (`5684b42`, `edc5367`).
- **Concurrency** — fixed static state being overwritten by concurrent runs and
  added Engine/Environment synchronisation; default city set to `TorinoCentre`
  (`9041d85`, `300196c`, `7fd2097`).
- **Building data guard** — prevented simulation freezes by skipping DMA
  building-location searches when no building layer is loaded (`e9e63e7`).
- **Input data** — fixed data problems, zone/node overlaps, and multi-layer input
  loading errors (`3db1bb2`); resolved layer issues (`7f9465a`).
- **Crashes** — fixed a critical crash on empty routes (`4a16827`) and an NPE in
  Dijkstra (`ae00a97`).
- **Applets** — fixed parameter mapping and compilation errors across the applets;
  preloaded map data at startup for immediate rendering.

### Added
- Night applet runnable for Torino (`629fd3b`, `#5`); REST API + dashboard
  groundwork and map preloading.
