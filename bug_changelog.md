# Bug Fix Log

Fixed defects, March–September 2026. Each entry gives the **symptom**, the **root cause** and the
**fix**. September is chronological; earlier history is grouped by area, so "what has ever been
wrong with routing?" is answerable. `CHANGELOG.md` is the higher-level summary and carries the
table of which results these defects invalidate.

_Excludes the learning module and the activity persona/realism work._

Legend: 🔴 crash/data-corrupting · 🟠 wrong results · 🟡 performance/UX

---

## September 2026 — destination distance, reproducibility, instrumentation

Uncommitted at the time of writing; hashes to be filled in when this lands.

### 🟠 A seed reproduced a run only on the machine that produced it (2026-09-14)
- **Symptom:** the identical command on `gdsl1` and on the Windows laptop gave 1163 against 1143
  mandatory legs, 2064 against 2049 trips, and 2,746,498 against 2,663,226 planned metres — same seed
  (20260912), same code, same resources, same `GeoMason-light-2.2.0.jar` at 117,232 bytes. Each
  machine replayed *itself* exactly (three runs and two runs respectively), so nothing looked wrong
  from either one alone.
- **Cause:** not floating point — `Math.exp`, `log`, `pow`, `sin` and `sqrt` were checked over
  200,000 inputs and agree bit-for-bit across the two. `NodeGraph` and `EdgeGraph` override neither
  `hashCode` nor `equals`, so a `HashMap`/`HashSet` keyed on one iterates in identity-hash order, and
  HotSpot derives identity hashes from a per-JVM generator whose values differ between JVM builds
  (21.0.1 locally, 21.0.6 on the server). `PoiClassifier` built the per-purpose attraction maps as
  `HashMap<NodeGraph, Double>`, and `WorkplaceChoice.draw` walks their entries into a cumulative
  distribution and then picks by position — so one random number selected a different workplace on
  each machine, which moved the commute distance, which moved `walksToWork`.
- **Fix:** `LinkedHashMap` for those maps, whose insertion order follows the buildings and POI layers
  and is therefore stable. `NetworkBuilder`'s known-network edge sets and
  `CognitiveMap.deriveOtherKnownRegions`'s per-region buckets became `LinkedHashSet` for the same
  reason, since `Islands.mergeConnectedIslands` iterates them to decide which islands to join — that
  reaches the activity and learning tiers, not night, whose agents build a simple bone.
- **Result:** the population layer is now identical across the two machines — mandatory legs, both
  walked commute shares, the length bands. **Trips and metres still differ by about 1%** (2036 against
  2058); the remaining path is downstream in destination choice or routing and has not been found.
  Until it is, a comparison must be run entirely on one machine, with the machine recorded beside the
  seed. **This changes results**: a Torino day now reports workers 16.8% / students 38.3%, where it
  reported 14.9% / 38.8% before.

### 🔴 `RoutePlanner` routed angular with no dual graph to route on (2026-09-14)
- **Symptom:** `NullPointerException: Cannot read field "primalEdge" because "dualNode" is null`,
  from `NodeGraph.checkPreviousJunction` via `getDualNodes`, on any primal-only city.
- **Cause:** `definePath()` tests `isLocalHeuristicDistance()`, so every mode that is *not* DISTANCE —
  including the constructed default `LocalHeuristicMode.NONE` — resolves as angular, and the
  minimisation branch above it has the same shape. `Heuristics.constrainLocalHeuristic` already
  demotes angular to distance when the dual graph is absent, but only for callers that reach it
  through `Agent.planRoute()`; `RoutePlanner` is also constructed directly.
- **Fix:** both branches check `PedSimCity.dualGraphLoaded` before choosing angular. The
  NONE-reads-as-angular asymmetry is left as it was and flagged in place — changing it would move
  every module's behaviour. Verified no-op for night on full Torino, which has a dual graph: a day
  reproduced byte-for-byte against the pre-change baseline.

### 🔴 The OD modules never terminated, and their volumes were cumulative (2026-09-13)
- **Symptom:** cityImage ran 300 s on 2 models × 4 trips without finishing; 8 × 6 exhausted a 4 GB
  heap. Empirical had the same loop.
- **Cause:** two faults. `startMovingAgents()` scheduled the spatial-index updater repeating and
  never stopped it, so `schedule.step()` stayed true after the last agent left — harmless to engines
  that run a fixed number of days, fatal to the two that loop on schedule emptiness. And
  `AgentMovement.initialisePath()` never cleared `edgesWalkedSoFar`, while an `OdAgent` reuses one
  movement handler across trips, so trip *n* reported the edges of trips 1…*n*.
- **Fix:** the updater's `Stoppable` is stopped in `Agent.removeAgent()`; `initialisePath()` clears
  the list. **The second fault corrupted output** — `updateFlowsData` re-counted every earlier trip
  into the current one, so per-edge volumes were cumulative. Any cityImage or empirical volumes from
  before this date are wrong. Core and activity agents build a fresh handler per trip and were
  unaffected.

### 🟠 Neither OD module exported anything (2026-09-13)
- **Symptom:** a city-image run compared its route-choice models and wrote nothing.
- **Cause:** both engines override `executeJob`, so neither reached the `exportFlowsData` core's
  `Engine` performs per day.
- **Fix:** both export after their job loop — `streetVolumes/*.csv` (one column per model) and
  `routes/*.gpkg`.

### 🟠 `getHeuristics()` was null for any agent overriding `planRoute()` (2026-09-13)
- **Symptom:** empirical threw `NullPointerException` in `Dijkstra.computeTentativeCostDual` as soon
  as it had a headless entry point.
- **Cause:** core's `planRoute()` builds *and stores* the heuristics; `EmpiricalAgent` and
  `CityImageAgent` overrode it and dropped that, and `NightAgent` built one without keeping it.
  `Dijkstra` dereferences it unguarded on a landmark-weighted cost, so cityImage survived only
  because road distance and angular change never consult landmarkness.
- **Fix:** one `Agent.initialiseHeuristics(boolean)` that stores as well as builds, called from all
  four.

### 🟠 `TestPars.defineMode()` overwrote the command line (2026-09-13)
- **Symptom:** `--numberTripsPerAgent=8 --jobs=1` logged "8 trips each, 1 job(s)" and then walked
  2,000 trips per agent across 10 jobs.
- **Cause:** `defineMode()` also sets `numberTripsPerAgent` and `jobs` to the design's defaults, and
  was called twice — once in the module's `applyMode()`, again in `CityImageImport.importFiles()`,
  which `Engine.runJobs` reaches after the arguments are applied.
- **Fix:** the import no longer calls it; the module resolves the design once, re-applies the
  overrides, and logs what it resolved.

### 🟠 A headless run constructed the entire AWT GUI (2026-09-13)
- **Symptom:** none locally; with `-Djava.awt.headless=true` the documented headless invocation would
  have thrown in the `Frame` constructor before reaching the simulation.
- **Cause:** `PedSimCityApplet.main` called `new PedSimCityApplet().coreLauncher()`, and
  `coreLauncher()` is `static` — the frame was built to call a static method through an instance.
- **Fix:** moot; the AWT GUI was removed the same day and launching moved to
  `pedsim.<module>.launcher.<X>Launcher`.

### 🟠 Dead GUI fields, and a reflective reach into a module (2026-09-13)
- **Symptom:** `ParsPanel`'s `avgTripDistance` row, `RouteChoiceParametersPanel`'s "local path" field
  and `dashboard.html`'s `maxTripDistance` all collected values that reached nothing.
- **Cause:** the panel applied its fields to `Pars` while the parameter lived on `RouteChoicePars`;
  `Pars.localPath` had been removed as unread; the dashboard field was never renamed with the
  parameter. Separately `HtmlExporter` reached the night module by string
  (`Class.forName("pedsim.night.parameters.NightPars")`), so a rename would have made the dashboard
  report no A/B test forever.
- **Fix:** the panels went with the GUI; the dashboard field is `maxRouteLength`; `HtmlExporter` asks
  `SimulationStateStore.moduleFlag(...)`.

### 🟠 GeoMason-light 2.2.0 is two different builds (2026-09-13)
- **Symptom:** night on Torino died on `gdsl1` with
  `NullPointerException: getIntegerAttribute("nodeID") is null` in `Environment.prepareGraph`, while
  identical code and data completed cleanly locally.
- **Cause:** the server's jar was 115,882 bytes against 117,232 locally — an older build under the
  same version number, predating the `Islands` work and the `Route.length` fix. 2.2.0 was never
  published, so nothing detects the difference.
- **Fix:** current jar shipped. **Not resolved at source:** publish 2.2.0 or stamp the rebuild. If a
  server run fails in graph preparation and a local one does not, compare jar sizes first.

### 🟠 Destination selection inherited the geometry of its own choice set (2026-09-09)
- **Symptom:** the straight-line origin–destination distance ran 1.20× the distance
  the release budget had charged, on the first leg of a trip chain with a fresh draw and a
  band that had not been widened. Three explanations were tested and rejected.
- **Cause:** candidates come from an annulus, so their number grows with the radius.
  A draw proportional to attraction inherits that growth, and the realised radius
  sits above the sampled distance even when every weight is equal.
- **Fix:** `Agent.selectWeightedDestination` divides each candidate's weight by the
  number of candidates sharing its radial shell — the standard correction for a
  sampled choice set. Attraction still decides which node at a given distance; it no
  longer decides the distance. **Effect not yet measured.**

### 🟠 Two different distance bands for the same quantity (2026-09-10)
- **Symptom:** core and night runs produced systematically different leg lengths.
- **Cause:** `Agent` asked for `[0.9d, d]` — one-sided, so it could never exceed the
  charged distance — while `NightAgent` asked for `[0.9d, 1.1d]`. About 5% apart.
- **Fix:** the band is gone. `Agent.candidatesNearDistance` starts a search interval
  at one metre and doubles until it is non-empty, returning the nodes nearest the
  distance actually asked for. A fixed ±10% also widened the choice set where the
  network is dense and narrowed it where it is sparse, which is backwards.

### 🟠 Purpose scaling moved the calibrated distance distribution (2026-09-10)
- **Symptom:** the released trip-length aggregate did not match the band it was
  drawn from.
- **Cause:** `ActivityPurpose.tripDistanceFactor` multiplied each draw by a
  purpose-specific factor. That preserves the aggregate only if the factors average
  exactly 1.0 over the purpose mix realised, and that mix shifts with hour, persona
  and agenda. The coded factors averaged above 1.0.
- **Fix:** removed. Watson et al. (2021), 2017 NHTS, report walking distances as not
  significantly different by purpose — a 1.15× spread visible only in duration, and
  in the opposite order to the one coded.

### 🟠 A park/water rejection pushed the destination further away (2026-09-10)
- **Symptom:** night trips were longer than the distance drawn for them.
- **Cause:** when a dark-hours candidate landed on a park or waterside edge,
  `NightAgent` answered by widening the *distance* interval and searching again.
- **Fix:** drop that candidate and redraw from the same set. A constraint unrelated
  to distance no longer moves the destination outward.

### 🟠 Habitual reuse was a flat 0.70, and the favourite set froze (2026-09-09)
- **Symptom:** 70% of discretionary legs went to a remembered place regardless of
  the distance drawn, from the agent's first release onwards; and an agent that had
  filled its favourites never learned another place for that purpose again.
- **Cause:** `habitualDestinationProbability` was a constant with no source, and
  `rememberFavourite` returned early once a purpose held `maxFavouritesPerPurpose`
  entries. Since the reuse probability reads the size of that set, the freeze fed
  itself.
- **Fix:** the exploration / preferential-return law of Song, Koren, Wang &
  Barabási (2010) — `P_new = 0.6 · S^(−0.21)` over every remembered place — plus a
  familiar-set capacity of 25 with turnover of the least-visited member
  (Alessandretti et al. 2018), and a return drawn in proportion to past visits
  rather than uniformly.

### 🔴 The activity module crashed on every city, and had for some time (2026-09-11)
- **Symptom:** `NullPointerException` from `CognitiveMap.buildActivityBone` as soon as agents began
  to step. Not intermittent: any run of `PedSimCityActivityApplet` died.
- **Cause:** the activity bone was built from `{agent.getHome(), agent.getWork()}`, and personas
  without a mandatory activity - retirees and flex adults - are deliberately given no work node. The
  null then reached `getRegionID()`, a `HashMap` key, and A*'s heuristic, in three separate places.
  The night module hid it: its agents come through the same populate, so the crash should have been
  visible there too and was not, which is worth understanding.
- **Fix:** `Agent.cognitiveAnchors()`, a seam returning the places a given agent is anchored on -
  home plus the mandatory activity where there is one. `ActivityAgent` overrides it so that someone
  with no workplace is anchored on plausible destinations for the persona's two strongest purposes
  instead of being left with a known world one neighbourhood wide. Connectivity paths are traced
  from the first anchor to each of the others rather than from home to work.

### 🟠 Module parameters given on the command line were silently ignored (2026-09-11)
- **Symptom:** `--useDestinationChoice=true` changed nothing, and said nothing. Two runs meant to
  compare the old destination mechanism against the new one were two runs of the old one, and the
  differences between them — which were run-to-run variance — were read as the cost of the new
  mechanism and written down as a 5× slowdown per leg. The claim was false and nothing in the output
  revealed it.
- **Cause:** `ParameterManager.initFromArgs` applies command-line overrides to `Pars`, `TimePars`
  and `RouteChoicePars`, and to nothing else. Module parameters have to be picked up by that
  module's `applyParameters`, and `ActivityPars` was not wired into
  `ActivitySimulationModule.applyParameters`.
- **Fix:** the activity module's switches and coefficients are applied there now.
- **Still open:** an unrecognised parameter is still accepted without a warning, so the same trap is
  open for the next class added.

### 🟠 The night module never used any of the destination work (2026-09-11)
- **Symptom:** a day of grounding and correcting destination choice produced no change whatsoever in
  the Turin night run — the one run the model is validated against. Improvements that had been
  written up as fixing the 1.20× selection excess had not touched it.
- **Cause:** `NightAgent` overrode `defineRandomDestination` outright, with its own band, its own
  search and its own park/water avoidance. Habitual reuse, the shell correction, the doubling search
  and later the utility model all sat in code the night module never reached.
- **Fix:** `NightAgent.defineRandomDestination` delegates to the shared choice and applies the
  park/water rule to the result, refusing a candidate and redrawing rather than widening the
  interval. It stays a refusal rather than becoming a utility term, because after dark it is a hard
  avoidance and not a preference to be traded off against attraction.

### 🟠 `Route.getLength()` returned zero, always (2026-09-11)
- **Symptom:** the release manager's day summary reported `planned 0 m, walked 0 m` after a day in
  which 2,207 trips were walked.
- **Cause:** `Route.length` is a private field with a getter, and `computeRouteSequences()` built
  the node sequence, the edge sequence and the geometry but never assigned it. Every caller asking a
  route how long it was got zero, and had done since the class was written - which means
  `PedSimCity.plannedRouteMeters()`, described at length in `CLAUDE.md` as the measurement that must
  not be fed back, had never measured anything.
- **Fix:** in GeoMason-light, sum the edge lengths in `computeRouteSequences()`, so the value
  follows the route and is recomputed when `resetRoute` cuts it back to what was walked.

### 🟠 Every worker walked the whole way to work (2026-09-10)
- **Symptom:** walking commutes were roughly eight times too many and twice too long, and the
  departure curve disagreed with the agents it was shaping.
- **Cause:** `ActivityAgent.shouldGoToWork` gated on persona, day and hour, and on nothing else, so
  every agent with a work node walked its commute. `ActivityPars.walkShareCommuteWorker` (12.0%)
  and `walkShareCommuteStudent` (27.9%), both from ISTAT 2017, existed but were read only by
  `DepartureProfile` to size the commute share of the day.
- **Fix:** the walking commute is drawn once per agent, when the persona is assigned, against that
  same ISTAT share, and `shouldGoToWork` now respects it. The agents and the departure curve
  finally assume the same thing.
- **Since (2026-09-11):** the flat share became a function of distance.
  `ActivityAgent.decideCommuteMode` draws `walksToWork` against `state.walkProbability(d)` for the
  agent's own home–work distance, because a 400 m commute and a 6 km one are not walked with the
  same probability and a flat share says they are. The ISTAT figures still drive `DepartureProfile`,
  which needs a population-level share rather than a per-agent decision. And the 2,700 m cap is gone:
  `Populate.selectWorkNode` no longer places the workplace inside the discretionary trip range.
- **Still open:** the commuters who do not walk make no commute leg at all. Their access and egress
  walks around transit stops are pedestrian metres the model does not produce yet. Workplaces are
  drawn with equal weight wherever they are, with no distance decay, so commutes come out longer
  than they should — and `ActivityPopulate.selectWorkNodeFromPurposeWeights` and
  `sampleEducationNode` still apply the old band, so the cap survives on those two routes.

### 🟠 `getPOIWeight` took a day/night flag that no implementation read (2026-09-10)
- **Symptom:** attraction weights documented as differing between day and night were
  identical in both.
- **Cause:** the `isDark` parameter was threaded through
  `selectWeightedDestination` and ignored by every override; there is one attraction
  table per purpose.
- **Fix:** parameter removed. What changes after dark is which purposes are open,
  which `ActivityPurpose` opening windows already decide.

### 🔴 No run was reproducible from its seed (2026-09-10)
- **Symptom:** two runs of the same model with the same seed produced different
  agents, different homes and different trips.
- **Cause:** `Agent.random`, `Populate.random`, `AgentReleaseManager.random`,
  `Group`, `Grouping`, `Heuristics`, `NightBehaviour` and
  `EmpiricalAgentProperties` were all `new Random()` — seeded from the clock. The
  walk-share acceptance draw and `CognitiveMap` used `ThreadLocalRandom`. `ba8ac99`
  (July) had seeded the route-choice draw only, which is a different generator.
- **Fix:** a generator per agent — a MASON `MersenneTwisterFast`, seeded from
  `state.nextAgentSeed()`, a counter off the model seed rather than the construction
  order, so the stream an agent gets does not depend on how the jobs interleaved; `Populate.seedFrom(state)` at every
  entry point; the release manager seeded from seed and day; collaborators draw from
  the agent's generator. `PedSimCity.acceptTripDistance` became
  `tripAcceptanceProbability` so the draw happens in the release manager against a
  seeded generator instead of in the state against a thread-local one.
- **Closed (2026-09-11):** GeoMason-light 2.2.0 gives every `NodesLookup` drawing method a
  `MersenneTwisterFast` overload, with the old signatures delegating to a `ThreadLocal` fallback so
  existing callers still compile, and `Populate` and the agents pass their seeded generator.
  `java.util.Random` is gone from the simulation path — a second, slower, differently-distributed
  source of randomness sitting next to the one MASON already guarantees reproducible.
- **Still open:** 2.2.0 is not published to Maven Central, so `pedsimcity` resolves it from the local
  `.m2` only and a fresh clone elsewhere will not build until the release workflow is run.

### 🟠 The planned-metres ledger counted legs that were never finished (2026-09-10)
- **Symptom:** planned metres exceeded walked metres by more than destination
  selection and circuity could explain.
- **Cause:** `Agent.setRoute` records a leg when its route is laid out. A trip chain still
  under way when the day ends has its last leg counted in full and walked in part.
- **Fix:** `PedSimCity.walkedRouteMeters()`, recorded in `AgentMovement.updateData`
  at the point the route is replaced by the edges actually covered, so the
  difference is visible rather than buried.

### 🟡 Band widening and the any-node fallback were silent (2026-09-10)
- **Symptom:** a leg could come from a band several times wider than the one
  requested, or from a uniform draw over the whole network, and a run had no way to
  say which.
- **Fix:** `PedSimCity.destinationWidenings()` and `destinationFallbacks()`, both
  reported per day when the release manager closes, alongside planned and walked
  metres.

### 🟡 `expectedTripChainLegs(Agent, int hour)` promised an hour it never used (2026-09-10)
- **Cause:** neither implementation read `hour`, and `AgentReleaseManager` passed a
  hardcoded `0` in the empty-candidates branch while passing the real hour
  elsewhere. Harmless, until someone made it hour-dependent.
- **Fix:** parameter removed; `ActivityAgent.expectedTripChainLegs()` already derives the
  hour from `shouldGoToWork()`.

### Found in the lighting chain, not fixed (2026-09-10)
An audit of the street-lighting pipeline for Turin turned up four defects that are
recorded here so they are not rediscovered. None is fixed.
- **Lux does not enter route planning.** `DijkstraRoadDistanceNight` avoids parks,
  water and unknown regions and never reads illuminance, so an agent cannot prefer a
  lit route in advance — only react once it is on a dark edge.
- **`min_lux` and `pct_unlit` are computed per edge and never read.** The model
  thresholds on `mean_lux`, so a street that is bright at both ends and black in the
  middle passes.
- **`NightAgent.getTripMeanLux` cannot fall.** It averages over lit edges only, so a
  darker route shrinks the denominator instead of lowering the value. The `mean_lux`
  column in `trip_diagnostic.csv` is not the mean illuminance along the path.
- **`braccio_l_m_max` is read, cleaned, zero-filled and ignored**, and 45.5% of
  Turin's poles have no height while 49.0% have no optic class, so both fall through
  to flat defaults for roughly half the inventory.

---

## Correctness — metrics & data

### 🔴 Trip distances inflated ~111,320× (`61b88dd`, 2026-06-11)
- **Symptom:** every reported trip distance was absurdly large; distance-based
  metrics were meaningless.
- **Cause:** `TripDiagnostic` treated projected metre coordinates (EPSG:3003) as
  geographic degrees and applied a `METRES_PER_DEGREE * cos(lat)` conversion.
- **Fix:** removed the conversion; distances now use raw Euclidean
  `sqrt(dx² + dy²)` on the projected coordinates.

### 🟠 "Least-walked" agent selection had no effect (`61b88dd`, 2026-06-11)
- **Symptom:** the intended priority (release the least-walked agents first) never
  actually influenced selection.
- **Cause:** `AgentReleaseManager.selectRandomAgents` called `sorted()` (stream)
  but never assigned the result, so the underlying list stayed unsorted.
- **Fix:** sort the list in place (`agents.sort(...)`) and select via a while-loop.

### 🟠 Agent release skipped by floating-point equality (`61b88dd`, 2026-06-11)
- **Symptom:** agents occasionally were not released on the intended step.
- **Cause:** release timing used exact double equality
  (`nextAgentRelease == steps`), fragile under floating-point accumulation.
- **Fix:** changed to a `>=` comparison.

### 🟠 Dashboard showed zero volumes (`e53e27e`, 2026-06-29)
- **Symptom:** volume tabs rendered empty / all-zero even after a run produced
  traffic.
- **Cause:** `volumesMap` was cleared before the dashboard read it.
- **Fix:** snapshot `volumesMap` before clearance; also raised base road
  visibility.

### 🟠 Wrong Torino illuminated-edges path & city name (`c185d9e`, 2026-06-10)
- **Symptom:** night lighting data failed to load / mismatched for Torino.
- **Cause:** a non-standard city name and a bespoke illuminated-edges path.
- **Fix:** corrected the city name to `Torino` and moved the illuminated-edges
  file to the standard `<cityName>_` prefix convention.

### 🟠 GUI run parameters ignored (`e843c12`, 2026-06-10; `ae40df6`, 2026-06-15)
- **Symptom:** changing days / population / percentage in the GUI had no effect on
  the run.
- **Cause:** the GUI values were not propagated into the run configuration;
  duration parameter processing was also broken.
- **Fix:** apply the GUI parameters on Run (defaults set to 1 day / 10 agents for
  fast testing) and fix duration parameter handling.

---

## Null-safety, crashes & import robustness

### 🔴 NPE from concurrent runs overwriting static state (`9041d85`, 2026-05-09)
- **Symptom:** intermittent `NullPointerException` when simulations ran
  concurrently.
- **Cause:** overlapping runs mutated shared static state, tearing down data another
  run was still using.
- **Fix:** guarded/synchronised the shared static state so concurrent execution no
  longer corrupts it (followed by broader Engine/Environment synchronisation in
  `300196c`).

### 🔴 Crash on empty routes (`4a16827`, 2026-04-30)
- **Symptom:** simulation crashed when a route came back empty.
- **Cause:** empty-route case was unguarded.
- **Fix:** added handling for empty routes; set `TorinoCentre` as the default city.

### 🔴 Freeze when no building data loaded (`e9e63e7`, 2026-04-22)
- **Symptom:** simulation hung for cities without a building layer.
- **Cause:** DMA (building-related) location searches ran unconditionally,
  spinning with no building data.
- **Fix:** skip building-related location searches when no building layer is
  loaded.

### 🔴 Landmark navigation crash on landmark-less cities (`474f9dc` / `334f9b9`, 2026-03)
- **Symptom:** navigation failed for Torino, which has no landmark setup.
- **Cause:** `LandmarkNavigation` invoked landmark logic even when no usable
  landmark data existed, and heuristics were used before initialisation.
- **Fix:** initialise heuristics for safe access and guard local-landmark usage on
  the presence of an actual landmark setup.

### 🔴 NPE: `AgentMovement.edgesToAvoid` during night rerouting (`813af96`, 2026-06-12)
- **Symptom:** `NullPointerException` while rerouting at night.
- **Cause:** the `edgesToAvoid` set was never initialised before use.
- **Fix:** initialise `edgesToAvoid` in `AgentMovement`.

### 🔴 NPE in `Agent` constructor (`da4504b`, 2026-06-10)
- **Symptom:** agent construction threw `NullPointerException`.
- **Cause:** light-sensitivity was initialised too early, before its dependencies
  existed.
- **Fix:** defer light-sensitivity initialisation.

### 🔴 NPE in `buildResidenceProbabilities` (`7fcc4fb`, 2026-06-15)
- **Symptom:** `NullPointerException` while building residence probabilities.
- **Fix:** null-guarded the probability construction.

### 🔴 Robustness when dual-graph / landmark / barrier / region data absent (`da6a161`, 2026-07-06)
- **Symptom:** route choice failed on cities that ship only a primal graph or lack
  landmark/barrier/region layers.
- **Cause:** these optional layers were treated as required, and heuristics assumed
  their presence.
- **Fix:** import reads the primal graph as required and the rest optionally with
  logging; added `dualGraphLoaded` / `landmarksLoaded` flags (reset on teardown).
  Heuristics fall back from angular to distance routing when the dual graph is
  absent, skip landmark/barrier/region mechanisms when their data is missing, and
  sample sensible defaults; null `localLandmarkness` attributes are guarded.

### 🔴 Centroid NPE & fragile census loading (`edc5367` / `5684b42`, 2026-05-09)
- **Symptom:** crashes during census import / centroid handling.
- **Fix:** made census import more robust and centroid handling null-safe.

---

## Routing & pathfinding

### 🔴 IndexOutOfBounds in `cleanDualPath` for short sequences (`676f2c5`, 2026-06-14)
- **Symptom:** `IndexOutOfBoundsException` during dual-path cleaning.
- **Cause:** `cleanDualPath` accessed `get(0)`/`get(1)` on sequences shorter than 2.
- **Fix:** early-return when `partialSequence.size() < 2`.

### 🟠 Night trip-skipping (`2e57195`, 2026-04-30; `7927414`, 2026-06-13)
- **Symptom:** night agents skipped trips / failed to route.
- **Fix:** stabilised night pathfinding and adopted original-route-aware bypass
  rerouting so detours respect the intended route.

### 🟡 Dijkstra performance & non-deterministic RNG (`ba8ac99`, 2026-07-11)
- **Symptom:** slow pathfinding and non-reproducible per-job results.
- **Cause:** repeated `getEdgeBetween`/`getDirectedEdgeBetween` lookups and
  allocations; `Utilities.fromDistribution` used a shared RNG; `reconstructSequence`
  was worse than linear.
- **Fix:** iterate `getOutDirectedEdges`, early-exit when the destination is polled,
  reuse primal junctions, replace the RNG call with a seeded per-agent
  `drawFromDistribution`, and make `reconstructSequence` O(n) via append-then-reverse.

---

## Concurrency & memory

### 🔴 Out-of-memory from unbounded route caches (`4ff3699`, 2026-06-16)
- **Symptom:** long / large runs crashed with `OutOfMemoryError`.
- **Cause:** route caches grew without bound.
- **Fix:** bounded the caches with an LRU eviction policy.

### 🔴 `AgentReleaseManager` not closed on failure (`e8419d4`, 2026-06-15)
- **Symptom:** leaked resources when a run threw mid-way.
- **Fix:** close `AgentReleaseManager` in a `finally` block.

### 🟠 Stale spatial indexes after teardown (`daf61ab`, 2026-07-11)
- **Symptom:** stale geometry lookups; wasted work rebuilding indexes.
- **Cause:** `clearStaticData()` called `getGeometries().clear()`, which only cleared
  a defensive copy and left the layer's spatial index intact; the spatial-index
  updater was also scheduled inside the per-agent loop (N rebuilds per step).
- **Fix:** call `layer.clear()` directly, and move index-updater scheduling outside
  the agent loop (one rebuild per step). Snapshot interval raised 1 → 5 steps to cut
  memory/HTML size.

---

## Visualisation & dashboard

### 🟠 Road lines overlapping / distorted (`edf1e90`, `3653de6`, 2026-06-14)
- **Symptom:** high-volume roads hidden behind low-volume ones; thickness
  distortion.
- **Fix:** sort road layers ascending by volume so heavier roads draw on top; keep
  road thickness constant on volumes tabs.

### 🟠 Live telemetry not reaching the dashboard (`75a8f5a`, `78dc460`, 2026-06-29 / 06-10)
- **Symptom:** dashboard didn't update live; blank or stale data.
- **Cause:** no HTTP serving / CORS and no load-time polling; laggy Streamlit
  dashboard.
- **Fix:** serve the dashboard over HTTP with CORS and poll on load; replaced
  Streamlit with a fast Leaflet HTML dashboard and split roads into a separate
  `/api/roads` endpoint.

### 🟠 Incorrect agent colouring & live lux display (`793a6ac`, 2026-06-10)
- **Symptom:** agents mis-coloured; live lux shown incorrectly; A/B tethers visible
  when A/B testing was off.
- **Fix:** corrected agent colouring and live-lux rendering; hide A/B tethers when
  A/B testing is disabled.

### 🟠 A/B testing visual defects (`bc97fc7`, `c95e331`, `7bcadcd`, 2026-07-07)
- **Symptom:** wrong sky colour, broken lux-metric IDs, trails not fading, camera
  follow drifting, tethers mis-rendered, roads mis-coloured when lights off.
- **Fix:** restored day/night `getSkyColor`, fixed lux-metric IDs and trail
  fade-out, corrected camera-follow panning math with auto-cancel on manual
  zoom/pan, and neutral road colouring when lights are off.

---

## Build & compilation

### 🔴 HtmlExporter string-literal size-limit compile error (`033e275`, 2026-07-02)
- **Symptom:** build failed — a generated Java string literal exceeded the 64 KB
  class-file limit.
- **Fix:** split the oversized literal so it compiles.

### 🔴 HTML export template syntax error broke map canvas (`3fc6e6e`, 2026-06-29)
- **Symptom:** exported results page rendered no map.
- **Cause:** a syntax error in the HTML export template.
- **Fix:** corrected the template.

### 🟡 Assorted compile fixes
- Engine compilation error & scope (`dd08980`), Dijkstra return-type mismatch on
  barriers (`685f4b1`), `Exporter.java` variable conflict (`4104f74`), javadoc
  parsing errors (`a855e53`), BOM encoding / Jackson imports (`c3789ac`).

---

## geomason-light substrate — shared & overwritten lists

The model sits on **GeoMason-light** (`uk.ac.liv.gdsl:GeoMason-light:2.1.0`, GitHub
Packages) for its GIS/graph layer: `VectorLayer`, `NodeGraph`/`EdgeGraph`/`Graph`,
`NodesLookup`, `MasonGeometry`, `Route`, etc. Several bugs stem from **list/collection
sharing semantics** in that substrate — a returned list being a live internal
reference (mutating it corrupts library state) or, conversely, a *defensive copy*
(mutating it is a silent no-op). Both directions caused **one agent's operation to
shape another agent's lists, candidate nodes, or the shared layer state.** These are
grouped here because the root behaviour is library-side; the PedSimCity fixes below
are the model-side defenses.

> ⚠️ Root causes partly live in GeoMason-light, which is a Maven dependency (not in
> this repo), so line-level detail below is for the PedSimCity call sites. Where a fix
> is a workaround around library semantics rather than a library fix, it is marked.

### 🟠 `VectorLayer.getGeometries()` returns a defensive copy — `.clear()` is a no-op (`daf61ab`, 2026-07-11)
- **Symptom:** layers appeared "cleared" but retained stale features / spatial
  indexes; re-reads accumulated duplicate features across runs.
- **Cause:** `getGeometries()` returns a **copy** of the internal list; calling
  `.clear()` / mutating it leaves the underlying `VectorLayer` (and its quadtree
  spatial index) untouched.
- **Fix:** `clearStaticData()` now calls `layer.clear()` on the layer itself. The
  same guidance is documented at every teardown site (`Import.java:164/219`,
  `PedSimCity.java:233`, `PedSimCityActivity.java:103`). _(Model-side workaround
  around library semantics.)_
- **Follow-up (2026-07-13):** the two remaining night call sites
  (`NightImport` illuminated-edges load, `PedSimCityNight.clearNightStaticData`)
  were switched from `illuminatedEdges.getGeometries().clear()` to
  `illuminatedEdges.clear()`, so illuminated-edge state (and its spatial index) no
  longer persists across re-initialisations.

### 🔴 Agents cross-registered / double-registered in shared agent lists (`01fdcb5`, `d49a003`, `b7acd3c`, 2026-06)
- **Symptom:** `state.agentsList` / `agentsAtHome` held duplicate or missing agents;
  per-agent status changes were not reflected consistently, so operations iterating
  these shared lists acted on the wrong population.
- **Cause:** `updateAgentLists` was invoked at the wrong point in the lifecycle —
  during `createAgent`/`assignHomeNode` and again during initialization — so agents
  were added before their geometry/location existed, or added twice.
- **Fix:** register agents only in the sequential `registerAgent` step, **after**
  `state.agents.addGeometry` sets the location (`01fdcb5`); remove premature
  additions during creation (`d49a003`); ensure `updateAgentLists` (re)adds to
  `state.agentsList` when walking/home status changes (`b7acd3c`).

### 🔴 Empty candidate-node list → NPE in destination selection (`b7acd3c`, 2026-04-13)
- **Symptom:** `NullPointerException` when choosing a destination.
- **Cause:** `selectWeightedDestination` assumed the candidate-node list (built via
  the `NodesLookup` substrate) was non-empty.
- **Fix:** added a null/empty guard before weighting candidates. See also the
  `randomDestination` hardening in `7a45519` (retry avoiding gateway nodes, fall
  back to a random node when no candidate qualifies).

### 🟠 Shared maps recreated on re-init, losing live references (`e5e0afb`, 2026-06-14)
- **Symptom:** volume and cognitive-map structures (`volumesMap`, `knownEdgesMap`,
  `knownLandmarksMap`) lost their references on re-initialization, so writers held
  stale map instances while readers saw a new empty one.
- **Cause:** initialization unconditionally recreated the inner maps.
- **Fix:** preserve existing map instances — if the outer map already exists, reset
  inner values to 0 via `Map.replaceAll` instead of reallocating.

### 🟠 Shared cognitive state scattered across statics (`3c6adcb`, 2026-06-15)
- **Symptom:** shared cognitive-map state (known junctions/landmarks, gateways) was
  reached through disparate statics, inviting cross-run/cross-agent contamination.
- **Fix:** made `SharedCognitiveMap` a proper singleton (`getInstance()`), added a
  `gatewaysMap`, and converted the relevant utilities to instance methods so shared
  state is centralised and reset coherently on teardown.

### 🟡 Concurrency containers around shared agent state (`a00558e`, 2026-06-13)
- **Context:** `SimulationStateStore` used a `CopyOnWriteArrayList` (O(N) array
  clone on every agent update) and `TrajectoryRecorder` did linear scans.
- **Fix:** switched to `ConcurrentHashMap` + plain `ArrayList`, removing per-update
  array clones on the shared agent-state path. (Performance, but it also removes a
  class of copy-on-write aliasing surprises.)

---

## Provenance vs. published releases (v1.01 / v1.09 / v1.11)

Mapping of the paper-linked releases and whether the bugs above could have shaped
their results:

| Release | Tag | Date |
|---|---|---|
| urban-subdivisions | `v1.01` | 2020-11-30 |
| modelling-landmarks | `v1.09` | 2020-12-06 |
| empirical | `v1.11` | 2022-05-23 |

**Structural caveat:** all three predate a full rewrite — version numbers drop from
`v1.11` (May 2022) to `v0.90` (Nov 2023, ~130 commits that month) before climbing
again. Every bug in this document lives in the **post-rewrite** lineage; most have no
counterpart in the release code.

**Not applicable** — the module/class did not exist at those tags (verified against
the `v1.11` tree): trip-distance CRS bug (`TripDiagnostic`), agent-release bugs
(`AgentReleaseManager`), night lighting / `edgesToAvoid`, census/activity imports,
OOM/LRU cache, dashboard/HTML/REST, A/B testing, the geomason-light agent-list and
`getGeometries()` fixes above (those layers/agent-registration paths are all
post-rewrite). The landmark-navigation guard (`474f9dc`) fires only when a city has
**no** landmark setup (Torino) — the landmarks/empirical papers used Muenster *with*
landmarks, so it would never have triggered there.

**One carry-over — `cleanDualPath` out-of-bounds.** The unguarded `partialPath.get(0)`
/ `.get(1)` (fixed in `676f2c5`) is present **verbatim** at `v1.01`/`v1.09`/`v1.11`
(`RoutePlanner.java:500`), inside angular-change / dual-graph routing used by all
three studies. Worst case is an `IndexOutOfBoundsException` on a too-short sub-path
(e.g. a single-edge segment between adjacent gateways), which would **drop/error that
individual trip** rather than bias aggregates — and it is **unverified** whether those
short-sequence conditions actually arose in the study runs.

**Reproducibility, not correctness.** Those releases used unseeded `new Random()` /
`Utilities.fromDistribution` (confirmed in `AgentGroupProperties.java`,
`DijkstraAngularChange.java`), later replaced by a seeded per-agent RNG (`ba8ac99`).
This makes individual old runs non-reproducible byte-for-byte but **does not
invalidate Monte-Carlo aggregate statistics**; the other `ba8ac99` changes are pure
performance and do not alter chosen paths.

**Bottom line:** no evidence of systematic bias in the v1.01 / v1.09 / v1.11 results.
The only substantive carry-over (`cleanDualPath`) can at worst drop occasional
dual-graph trips, and its triggering in those runs is unverified.
