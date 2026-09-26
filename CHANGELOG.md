# Changelog

Notable changes to PedSimCity, most recent first, dated by the day they landed. The format
follows [Keep a Changelog](https://keepachangelog.com); the project is not yet versioned.

## 2026-09-25

### Fixed
- **`min_lux` is joined onto graph edges, so the dark-spot half of the lighting gate can fire.**
  `NightLighting.isLit` has always tested it, but `NightEnvironment.joinIlluminatedEdges` attached
  only `mean_lux`, and the test passes when the attribute is absent — the degradation a city with
  no lighting pipeline needs. Every edge took that pass, so the gate was `mean_lux` alone and a
  street bright at both ends with a black middle read as lit. The join now carries both, counts
  them separately, and warns when `mean_lux` lands without `min_lux`. On Torino 8,671 of 44,278
  edges (19.6%) hold a sub-5-lux gap and 1,118 (2.5%) flip from lit to unlit at a 15-lux threshold.
  Reported as C1 by Marcin Wozniak's audit of the lighting pipeline and night layer.
- **A run no longer grows its heap with every simulated day.** `FlowHandler.routesData` was
  appended to on every leg and never cleared, so every route of the run stayed in memory and each
  day's GeoPackage re-exported all earlier days — per job, 374 MB on day 1 rising to 1,971 MB on
  day 5. It is cleared once the day holding it has been written. `TripRouteRecorder` held the same
  geometry a second time in `TripRecord.pathCoords`; the walked length is now measured once at
  capture into `distanceMetres`, and the coordinates are kept only when the HTML dashboard will
  draw them. Removes two duplicated length helpers with it. Four 20% Torino seasonal runs had died
  of `OutOfMemoryError` on day 4–5 of 5.

### Changed
- `aggregate_season_volumes.py --max-day` cuts a run back to the days its siblings also reached.
  Seasons are compared on absolute traversals, so a run that got further would otherwise read as
  busier.
- `publish_site.py` names the Pages branch explicitly (default `main`). Left to itself wrangler
  takes the current git branch, and a feature branch deploys to a preview URL — succeeding, printing
  a link, and leaving the live domain untouched. It also merges `trips_summary.json` with
  `setdefault`, so a trips file covering fewer jobs or days cannot overwrite a day-summary total.

## 2026-09-20

### Added
- **Mode choice per trip.** `Agent.walksLeg(origin, destination)` is asked once a destination is
  known and before any route exists: core walks everything, `ActivityAgent` draws against a
  walk-share curve at the distance the leg turned out to be. Both trip paths cross it, `planTrip`
  and `startChainedTrip`, so a chained leg is no longer walked whatever its length. A leg that is
  not walked produces no pedestrian metres and the agent never sets off; the commute is exempt,
  its mode having been settled by `decideCommuteMode` against the ISTAT curve.
- `ActivityPars.walkShareHalfDistance` / `walkShareSteepness` restored as the general curve, behind
  `TravelDemand.walkProbability` — an interface method that until now had no caller.
- `DaySummary` reports the realised walking share whole, by distance band and by ring out from the
  city centre, each with its count and the ring's mean leg length, plus a `discretionary_walk_share`
  column. The walking share is now something the model produces and is checked on.

### Changed
- **The trip budget counts every mode.** `ActivityPars.tripsPerPersonPerDay = 2.04` replaces
  `walkedTripsPerPersonPerDay = 0.51` — the same ISFORT figure without the walking share divided
  into it. Commute legs are charged to it whatever mode they are made by (`ActivityAgent.commutesOn`),
  since subtracting only the walked ones spent a driver's commute a second time as somebody else's
  discretionary walk.
- `walkShareHalfDistance` set to 1,500 m rather than the English NTS fit's 2,290 m. Interim: the
  level moves with it, but no half-distance fixes the level while the NTS *slope* stays shallower
  than the ISTAT-fitted Italian ones.

### Measured
- One Torino day, 3,386 agents: 65.8% of discretionary trips walked at 2,290 m, falling with
  distance as fitted (87.8% under 500 m, 43.9% between 2 and 5 km). **The periphery gradient does
  not follow**: mean leg length is 1,266 m in the centre, 1,484 m mid and 1,375 m in the outer ring,
  so peripheral agents are not offered longer trips and a distance-based mode choice has nothing to
  bite on. The cause is `ActivityPars.choiceSetRadiusMetres = 3000`, which makes the trip a
  peripheral resident would really drive unrepresentable — structurally the 2,700 m commute cap
  again. Layer 3, not another mode parameter, is where the periphery is decided.
- Audimob excludes walking trips under five minutes from its trip universe, so both figures the
  model takes from ISFORT — 2.04 trips per resident and the 20.6% walking share — omit exactly the
  trips a pedestrian model produces most of. Comparisons have to drop sub-420 m legs on both sides;
  see `references/README.md`.

## 2026-09-19

### Added
- Seasonal results page, published per city as `<City>/seasons`: metres walked per street, the
  split between vulnerable and non-vulnerable pedestrians, an hour scrubber and an hour player
  (`scripts/site/seasons.html`).
- `scripts/publish_site.py` stages `outputs/site_data/<City>/` beside the result pages and writes
  the index the page reads first (`data/seasons/summary.json`).
- `scripts/export_network_geojson.py` carries each edge's `length`, so the map can show metres
  walked rather than edges entered.

### Changed
- **An agent has a sex, drawn from the census in layer 1.** `female_pct` — women aged 15+ over the
  zone's adults — replaces `vulnerability_pct` on the census layer, and `ActivityPopulate` draws
  each agent's sex from its home zone alongside the persona. *Vulnerable* is then the night
  module's judgement about that fact, made in `NightPopulate.assignVulnerabilityStatus` and nowhere
  else: this module's vulnerable group is women. The census says who someone is; a model says what
  it makes of that.

  It replaces a set that was all females, males under 15 and males 65+, taken over every resident:
  68.9% of Turin against 52.5% now. The denominator was the deeper error — persona shares are drawn
  on the 15+ base, so no agent can be a child, while 11.7% of Turin's residents are.

  The realised share is logged at startup against the census expectation
  (`women: 52.3% of agents (census 52.5%)`), as the walked-commute shares already were.
- The results site and the `inclusivestreets.org` umbrella are two Cloudflare Pages projects, since
  one project serves the same deployment on every domain attached to it. `--project` defaults to
  `pedsimcity`, and `$WRANGLER` locates wrangler when it is not on `PATH`.
- Planning notes (`TODO.md`, the per-module `TODO.md` files, `bug_changelog.md`) are no longer
  tracked; they are working notes rather than user documentation.

### Fixed
- **A per-zone rate reached an agent through the node it was standing on, not the zone it was drawn
  from.** `NightEnvironment.deriveVulnerability` broadcast each zone's rate to every node within
  50 m and merged with `Double::max`; Turin's census sections are city blocks, so a node was claimed
  by a median of 5 zones and took the highest of their rates. The realised vulnerable share was
  74.1% against a census 68.9%. Sex is now read from the home zone the agent was actually sampled
  from, like the persona shares, and the node broadcast is gone — along with
  `PedSimCityNight.nodesVulnerabilityWeight`.

  The four seasonal Torino runs of 19 September were stopped part-way and relaunched on the
  corrected construct; every night figure computed before it overstates the vulnerable share.

## 2026-09-17

### Fixed
- Night reroute could send a vulnerable agent back along the route it had already walked:
  `NightAgentMovement.computeAlternativeRoute` did not advance `originalRouteIndex` when it applied
  a bypass, and the earliest-reachable-re-entry rule then preferred positions behind the agent.
  3.9% of trips had been carrying about 60% of all walked metres; walked distance now sits about
  1.4% above planned.
- The final edge of a route could never be bypassed, because the candidate loop read each remaining
  edge's *from*-node. The destination is now a re-entry candidate.

### Added
- `NightPars.maxReroutesPerLeg` (10) bounds applied bypasses per leg.
- `RouteTrace` reports a revisit factor — edge traversals against distinct edges — with the day's
  worst single leg beside it.

### Changed
- GeoMason-light 2.2.2, for the `Astar` predicate and multi-target search the night reroute needs.

### Performance
- One December night day at 423 agents: 571.9 s → 47.1 s, with identical output across the library
  step. The park-and-water edge union is held rather than rebuilt per edge relaxation; unlit-edge
  sets are cached per light-sensitivity threshold, quantised by
  `NightPars.lightSensitivityQuantumLux`; A* takes a predicate instead of a materialised avoid-set;
  and candidate re-entry points are narrowed rather than searched one at a time.

## 2026-09-16

### Added
- Darkness reaches route planning: `DijkstraRoadDistanceNight.lightingCostMultiplier` scales a
  **known** edge's cost from 1.0 at the agent's own sensitivity threshold toward
  `NightPars.maxKnownDarkEdgeCostMultiplier` (1.5) at total darkness. Unknown edges are left to the
  situated gate. 1.0 restores the previous behaviour and is the control.
- `NightLighting.darknessDepth` is the single measurement behind both the planning cost and the
  situated reroute-or-speed-up probability.

### Changed
- The vulnerable avoid-set is what is neither lit nor familiar, rather than whatever the agent does
  not know. This is the A/B experiment's manipulated arm.
- A/B twin pairs are released only into dark events; `NightTravelDemand` counts the date's dark-event
  capacity and warns when the experiment is larger than its own night.
- `routing/pathfinder` and `routing/pathfinding` became `routing/routers` and `routing/search`, each
  with a `package-info.java`. Class names are unchanged.
- `core.cognition.cityimage` became `core.cognition.elements`.
- Lighting pipeline: `FALLOFF_LAW = "isotropic"`, chosen by running all three laws over the same
  44,278 edges, and the uncited utilisation factor replaced by the downward light output ratio
  (mean 0.898 against 0.470). The two changes nearly cancel on `mean_lux`. Step 3 is vectorised, so
  a full Torino run takes about ten minutes rather than half an hour; Torino's lighting layer was
  rebuilt. The other five cities were not.
- All logging goes through `LoggerUtil`; the 24 `System.out` / `System.err` calls are gone.
- `scripts/build_transit_layer.py`, `scripts/run_day_night_comparison.py` and `scripts/dashboard.py`
  take the city from the caller instead of a constant.

### Fixed
- `DijkstraRoadDistanceNight` drew its own hard-coded 0.10 perception sigma, so `--perceptionErrorSD=0`
  did not pin the night router. It calls `costPerceptionError` now.
- `Torino_directional_lighting_lookup.csv` had been generated against `Torino_simplified` — 29,062
  rows where the full graph needs 88,556 — so most edge entrances fell back to the OSM `lit` tag and
  the rest matched by node-ID collision between two graphs. Regenerated.
- `DijkstraRoadDistanceNight` now applies the three relaxation skips its parent applies, and the
  known-network sets load through `initialiseKnownNetwork()` on both entry paths; an individualised
  agent on the three-argument path would otherwise have been returned no route at all.
- `run_day_night_comparison.py` compared a run with itself: all three flags distinguishing its arms
  were unknown keys, which `ParameterManager` drops silently. The arms are explicit arguments now,
  and identical arms are refused.
- `build_transit_layer.py` hard-coded EPSG:3003 and read its layer name off the filename.

### Removed
- `pedsim.transit` and the per-leg mode split that consumed it. Neither was wired into mode choice,
  and the walking legs around a stop — the part that would matter — did not exist.
- The `Torino_simplified` city.

## 2026-09-15

### Added
- A test suite: 74 fast tests under `mvn test` (about a second), plus `-Pslow-tests` for the three
  that need a real city. They pin what a simulation cannot check about itself.
- `RouteChoicePars.perceptionErrorSD` replaces a hard-coded 0.10, so `--perceptionErrorSD=0` pins
  the cost multiplier for a model comparison without a source edit.
- `-Dpedsim.trace=<file>` writes one line per planned leg, with no timestamps, so two runs are
  byte-comparable between machines.
- `CityLocation` measures the city's position from the street network's minimum bounding circle and
  the CRS the node layer declares; `ActivityPars.timeZoneId` states the time zone, which geometry
  cannot supply.

### Changed
- Route choice is a value. `RouteChoiceModel` is an immutable record of what an agent's route choice
  *is*; `AgentProperties` is the working copy one trip is planned against. `Heuristics` returns a
  model rather than overwriting one that cityImage or empirical assigned.
- Parameters have a declared precedence: module defaults → city file → command line → derived. The
  three passes that existed only to undo an earlier stage are gone.
- Sunrise and sunset are computed in clock time — longitude against the standard meridian, the
  equation of time, and the conventional −0.833° altitude. Turin's sunset had been 19:43 against a
  real 21:18 CEST.
- One definition of darkness, `Daylight.isDark(time)`. The fixed 20:00–06:00 window and
  `ActivityPars.useSeasonalDaylight` are removed.
- The HTML dashboard moved to the night module; `Pars.isNight` is removed.
- Core no longer has a word for darkness, lighting or vulnerability: each moved to the module that
  owns it, behind a seam core already had.
- The empirical cluster is re-sampled per trip rather than once per agent. This changes empirical
  results by design.
- Nine build and publish scripts moved to `scripts/`.

### Fixed
- Region navigation had returned the shortest path on every OD pair, barrier sub-goals were never
  generated, and on-route marks had never been inserted by any version of this code. Each gated on
  an `agentKnown*` set that is empty for an agent whose cognitive map is never individualised;
  `CognitiveMap` now answers with the community map for those agents.
- `CognitiveMap.getWayfindingEasinessThreshold` was an unimplemented stub returning 0, so the
  on-route-mark loop ended before its first iteration.
- Two cityImage scenarios were one configuration: `activateLandmarks()` matched the substring
  `LANDMARKS` before the `LOCAL` / `DISTANT` qualifier.
- `SharedCognitiveMap.edgesWithinParks` and `edgesAlongWater` had no writer anywhere, so five night
  mechanisms read empty sets. They are derived from the per-edge attributes `BarrierIntegration`
  already writes.
- Region navigation latched off for the rest of an agent's life, because three sites disabled it by
  writing to properties that outlive the trip.
- `EmpiricalAgent` assigned its route field directly, so no empirical run recorded planned metres.
- `RegionBasedNavigation.findNextGateway` collected into a `ConcurrentHashMap` and broke ties on
  identity-hash order, so one seed gave two answers.
- `RouteProperties.cumulativeLandmarkness` summed two parallel streams, and floating-point addition
  is not associative.
- `SharedCognitiveMap` held 17 static mutable collections and reset none, which the REST dashboard's
  repeated runs need.
- `PedSimCity.sightLines` was set to null to free memory and never restored, so a second run in one
  JVM failed before importing anything.
- `-Pslow-tests` did not exist, although the pom's own comment named it.
- `PathFinder.routeSequence` holds the sub-goal loop once. The three copies had drifted; one
  corrected edge directions from the wrong end of the leg.
- Lighting pipeline: a lamp up to 5 m inside a building footprint is mounted on it and is no longer
  occluded by it — 5,092 of Torino's 99,742 lamps, which had contributed nothing.

## 2026-09-14

### Changed
- `LearningAgent` extends `ActivityAgent` and `CommuterAgent` is folded in, so learners get the
  persona, the agenda and the commute mode choice. This changes learning results.
- Pedestrian-volume exports aggregate on the date's own sunrise and sunset; the columns are `LIGHT`
  and `DARK` rather than `DAY` and `NIGHT`.
- `TimePars.SIMULATION_START_DATE` takes a value on the command line. Every run before this was
  1 June, the shortest night of the year, in a module about darkness.
- Formatting is enforced by spotless with google-java-format, applied by `pre-commit` and checked by
  `pre-push`.
- Opening windows may come from a city file (`purpose.<NAME>.open`, `.close`, `.stayMinutes`,
  `.staySigma`).

### Fixed
- A seed did not reproduce a run across machines. `NodeGraph` and `EdgeGraph` override no
  `hashCode`, so hash-keyed collections iterated in identity-hash order and `WorkplaceChoice.draw`,
  which picks by position, chose a different workplace per JVM. Three sites became `LinkedHashMap` /
  `LinkedHashSet`.
- The learning module had never completed a day: seed memory was angular-routed whatever the agent's
  route choice; `applyDecay` took its percentile threshold before decaying and compared it after;
  and `buildBasicMemory` discarded four of every five cognitive-map rebuilds.
- Commuting was refused after dark, which deleted Turin's winter commute — the most routine walking
  there is, for the population most exposed to unlit streets.
- A fleeing vulnerable agent did not avoid the edge it was fleeing, so a bypass could run down it.
- `LocalHeuristicMode.NONE` resolved as angular, through a final ternary that tested "is it
  distance".
- `Persona.sample` thins the student share by `ActivityPars.youthEmploymentRate` instead of counting
  the employed young twice.

### Added
- `--jobs=N` reports replicate mean, sample sd and range. The first measurement put the sd at 6.5%
  of planned metres, which is the floor any claimed effect has to clear.
- `RemoteLauncher` has a `main`, so remote runs are reachable as a command.

### Performance
- `CognitiveMap.readjustCognitiveMap` walks the network's nodes once against an STRtree of prepared
  polygons: 16 learning agents went 4 m 10 s → 2 m 45 s.

## 2026-09-13

### Changed
- Release is a count of departures in every tier. The metres budget is gone, with
  `Pars.metersPerDay`, `TripDistanceBands`, the carried residual and four `TravelDemand` seams no
  implementation needed.
- Trip distance is a walked route length (`Pars.minRouteLength` / `maxRouteLength`), converted to the
  straight line a node lookup needs through `NetworkCircuity.straightLineFor()` — the one place that
  division happens.
- `NetworkCircuity` moved to core and is measured in `Environment.prepare()`, so a bare core run
  measures it.
- Per-city configuration: `activity.parameters.CityConfig` reads `<City>.properties` before the
  command line. Run switches are refused with a warning, since they choose an experiment rather than
  a place, and every key is reported as applied, refused or unplaced.
- `SimulationModule.parameterClasses()` is one list, consulted by both the command line and the city
  file. Module parameters had previously to be picked up a second time by hand or be dropped in
  silence.
- cityImage and empirical became modules, with launchers and parameters reachable from the command
  line.

### Fixed
- Angular-change routing iterates every incident dual centroid rather than the single best-aligned
  one: 0 fallbacks and 0 unknown dual endpoints on a Torino day, against 92 of 420.
- An unroutable trip widens the search to the full network instead of being lost, counted in
  `RouteTrace.fullNetworkEscalations`.
- `roadDistanceSequence` returned its last leg rather than the whole sequence, which had been true
  since March 2026.
- A headless run constructed an AWT frame, so the documented headless invocation threw under
  `-Djava.awt.headless=true`.

### Removed
- The AWT GUI, 15 classes. Entry points are `pedsim.<module>.launcher.<X>Launcher`.

## 2026-09-12

### Fixed
- Runs were never reproducible: `Engine(StateFactory)` seeded from `System.currentTimeMillis()`, and
  that is the constructor every headless run reaches. `Pars.seed` is a parameter now, fixed at
  20260912, with `--seed=-1` for a clock seed, logged at startup.
- Network circuity was never measured. `NetworkCircuity` discards a pair whose route reports a
  non-positive length, and `Route.getLength()` returned zero in every released GeoMason-light, so
  the measurement bailed out and the fallback stood. Measured: Barcelona 1.174, Paris 1.174,
  Muenster 1.231, Torino 1.292, Melbourne 1.543.
- The city's latitude was a hard-coded 53.4 — Liverpool — applied to every city. Turin is 45.069, an
  error of 1 h 20 m of darkness a day.
- `cityImage` and `social` had not compiled for some time; the default profile excludes both, so no
  ordinary build noticed.
- `RouteNovelty` divided by zero and `RouteComplexity` returned zero for every route, both
  consequences of the same zero-length routes.

### Performance
- An activity day at 423 agents went from about 25 minutes to about 150 seconds. The cost was
  `sim.graph.Islands`, quadratic in three separate places and unbounded in `mergeConnectedIslands`,
  reached once per agent from `NetworkBuilder.buildKnownNetwork`.

### Changed
- Street lighting is one photometry module (`pipeline/lighting.py`). The summation cutoff, derived
  from the inventory at 112.5 m, is separated from the 5-lux service level (EN 13201-2 class P4, via
  UNI 11248); illuminance adds, so a cutoff at the service level discarded ten lamps contributing a
  lux each.
- Education has its own distance decay and commute curve, fitted to the study rows of the ISTAT
  matrix.
- Module boundaries: `LearningPars` moved to `learning/parameters`, `CoreTravelDemand` became
  `BaselineTravelDemand`, and the working day moved to the activity tier — employment had been a
  property of every pedestrian the model can represent. `BarrierPreference` and `PopulationPars`
  removed as dead; the survey quantities they held are in `Muenster_clusters.csv` per cluster.

## 2026-09-11

### Changed
- Travel demand left the state class. `AgentReleaseManager` had asked `PedSimCity` fourteen
  questions about two unrelated things; they are `TravelDemand` and `RouteTrace` now, reached
  through `state.travelDemand()` and `state.trace()`.
- The commute is generated rather than drawn: `planMandatoryDeparture` gives each agent a departure
  minute inside its persona's start window when it has a workplace, walks to it and attends that
  day. `ActivityPars.walkedTripsPerPersonPerDay = 0.51` replaces `tripChainsPerPersonPerDay = 0.21`,
  which was the same survey figure with a chain length already divided into it.
- The walk-share curve split in two, commuting and discretionary: a commute is walked far less at
  the same distance, and one pooled curve is why the model walked 47% of work commutes where Turin
  walks 16.3%. Fitted against the ISTAT *Matrice del pendolarismo*: `walkShareCommuteWorker` 0.163,
  `walkShareCommuteStudent` 0.380, workplace decay β = 1.0, distance floor 0 m.

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
  rebuilds the staging folder each run (so removed runs drop off the site) and works
  before the first run (empty-state landing).

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
