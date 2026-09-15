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
| before **14 Sep 2026** | any figure depending on where workplaces are | the per-purpose attraction maps iterated in identity-hash order, so the workplace draw picked differently per JVM build; a Torino day now reports workers 16.8% / students 38.3% |
| any comparison spanning **two machines** | all of it | still open: trips and metres differ across machines by about 1% on one seed |
| any learning run before **14 Sep 2026** | all of it — there are none | the module threw during agent creation without a dual graph, and routed seed memory by angular change regardless of route choice where it did not throw |
| any cityImage or empirical result before **15 Sep 2026**, including the tables dated 14 Sep | all of it | region navigation returned the shortest path on 100% of OD pairs, barrier sub-goals were never generated, and on-route marks had never been inserted by any version of this code |
| any `LOCAL_LANDMARKS_*` figure before **15 Sep 2026** | the local-only claim | a substring test turned distant landmarks on as well, so those scenarios were byte-identical to their bare `LANDMARKS_*` siblings |
| every empirical run's **totals** | planned metres only | `EmpiricalAgent` assigned `route` directly instead of through `setRoute()`, so the ledger recorded nothing; walked volumes and exports are unaffected |
| any cityImage or empirical run using local landmarks | routes and volumes | `getWayfindingEasinessThreshold` returned 0, so on-route marks were never inserted |
| any **night** run before **15 Sep 2026** | park and waterside behaviour | `edgesWithinParks` and `edgesAlongWater` were never filled, so destination refusal, vulnerable-agent avoidance, the preference and its cost term all read empty sets |
| any distant-landmark route over a sub-goal sequence | the route | `globalLandmarksPathSequence` corrected edge directions from the leg's far end |

---

## September 2026

### 15 September (after the refactor) — an audit of the eight modules

Three questions, asked across every module: is it modular, is anything duplicated, is it
reproducible. What it found, most serious first.

**The three sub-goal routers were one loop written three times, and had drifted into a bug.**
`checkEdgesSequence` walks a leg forward from the node it is given and flips edges the search
returned reversed; `globalLandmarksPathSequence` advanced `tmpOrigin` to the leg's destination
*before* calling it, so it corrected from the wrong end. It also advanced `tmpOrigin` on every
`moveOn`, where backtracking sets that flag both for "found a direct edge" and for "gave up and
skipped this sub-goal" - only the first means the agent moved. Both are gone: `PathFinder
.routeSequence` holds the loop once and takes the per-leg routing as a lambda. Angular keeps its own,
deliberately. `RoadDistancePathFinder` 172 → 134 lines, `GlobalLandmarksPathFinder` 112 → 80.

**Park and waterside edges were never collected.** `SharedCognitiveMap.edgesWithinParks` and
`edgesAlongWater` had no writer anywhere in the tree, so five night mechanisms read empty sets:
destination refusal after dark, the vulnerable agent's avoidance, the non-vulnerable agent's
preference and its cost term. Derived now from the per-edge `parks` / `waterBodies` attributes
`BarrierIntegration` already writes - no second copy of the same fact. Torino has 2,500 park edges
and 647 waterside; a night day goes from 205 legs / 313,334 m planned to 204 / 311,671 m.

**Parallel floating-point sums.** `RouteProperties.cumulativeLandmarkness` summed two
`parallelStream()`s; `DoubleStream.sum()` adds in split order and floating-point addition is not
associative, so the same route could score differently run to run - and route memorability is what
the learning module learns from. Sequential now.

**Static state that never reset.** `SharedCognitiveMap` held 17 static mutable collections and
cleared none, while `PedSimCity.clearStaticData()` cleared its own 22. `communityKnownNodes` was only
ever added to, and `routesSubNetwork`, `cachedHeuristics` and `roadTypeMap` are keyed on graph
objects from a previous city. Harmless for one run per JVM; wrong for the REST dashboard, which runs
the engine lifecycle repeatedly. `SharedCognitiveMap.clearStaticData()` is called from
`PedSimCity.clearStaticData()`, before the import and prepare that rebuild every one of them.

**Two smaller reproducibility items.** `HtmlExporter` picked its follow-agent with `Math.random()`,
so two renders of one run differed. And `keepValidSubGoals` returned a `HashMap<EdgeGraph, Double>`
that the caller stable-sorts and takes the first of - equal distances along a barrier are common, so
the tie fell to `EdgeGraph`'s hash, reproducible only because GeoMason-light 2.2.1 happens to give it
a value-based one. A `LinkedHashMap` in candidate order removes the dependency rather than
documenting it. The comment in `NetworkBuilder` that asserted the opposite - that `EdgeGraph`
overrides no `hashCode` - was checked against the resolved jar and corrected.

**Dead:** `EmpiricalPars.applyDefaults()` and `SharedCognitiveMap.buildCommunityKnownNetwork()`, an
empty method whose commented-out body describes work `NetworkBuilder` already does per agent.

**Clean:** no core → module imports anywhere; module scaffolding is proportionate. One finding is
left open - `activity` and `transit` import each other, and untangling that belongs with wiring
transit into layer 4.

**Verified on gdsl1**, every module, against the traces from earlier the same day: London
subdivisions (1,200 legs), London landmarks (2,295) and Muenster empirical (2,709) all **identical**;
activity unchanged to the metre (58 legs, 80,499 m); core clean (49 legs, 85,161 m). Night is the one
that moves, and by design - 205 legs / 313,334 m planned becomes 204 / 311,671, because park and
waterside avoidance now has edges to act on.

### 15 September (last) — route choice becomes a value, and the ledger becomes the trace

Four of the day's defects were the same design: `AgentProperties` was a mutable bag that mixed what
an agent's route choice *is* with what a planner had decided *for one trip*, and the two had the same
lifetime as the agent.

**`RouteChoiceModel`** is now the decision - an immutable record carrying the strategy, the
minimisation mode or local heuristic, the elements, the landmark type and the barrier preferences.
**`AgentProperties`** is the working copy one trip is planned against, rebuilt from the model at the
start of every trip. What that makes unrepresentable:

- a planner disabling region navigation for the rest of an agent's life rather than for one trip
- `Heuristics` overwriting a model that cityImage or empirical assigned - it returns a model now, and
  an agent that has one never asks; `isRouteChoiceAssigned()` is gone
- a model falling into pure minimisation by acquiring a mode. That a pure minimisation ignores
  every element is intended and old - v1.11's `onlyMinimising` - but it used to be *inferred* from
  the mode being set, so `Heuristics` writing a sampled mode over an assigned model made the rule
  discard elements the agent was built to use, and nine cityImage scenarios came out as two routes.
  `shouldOnlyUseMinimization()` reads a declared `Strategy` now, and the record's constructor refuses
  a model that is neither kind, so the unconfigured state that needed a warning cannot be built

**The cityImage `RouteChoice` enum declares each scenario** in its constructor instead of having its
meaning parsed out of its name by substring - the defect that made `LOCAL_LANDMARKS_*` identical to
its bare sibling. `CityImageAgentProperties` had nothing left to do and is gone.

**The empirical cluster is re-sampled per trip again.** `randomizeRouteChoiceParameters()` was called
once in the constructor, so an agent drew one way of getting somewhere and kept it for life; v1.11
drew per trip, from `Pedestrian.findNewAStarPath`. A cluster is a distribution over ways of getting
somewhere rather than a label fixed to a person, and drawing once per agent makes a group's realised
mix N draws instead of N x trips. This is the one change of the day that alters empirical results by
design.

**`RouteTrace` and the per-leg tracer are one class, `RouteTrace`.** They share the one hook that
matters, `Agent.setRoute()`, and they fail together: when `EmpiricalAgent` assigned its route field
directly, the totals read zero and the per-leg file held only its header, with nothing to say which
was wrong. The counters answer how much; the per-leg record answers which route this model took for
this OD pair. `state.ledger()` is `state.trace()`.

Verified by re-running every measurement of the day and comparing traces byte for byte: London
subdivisions (1,200 legs), London landmarks (2,295), Muenster empirical (2,709) all **identical**,
and night and activity days unchanged to the metre. A refactor that changes no behaviour is the only
kind worth making here, and the traces are how that was shown rather than asserted.

### 15 September (later) — four more mechanisms that ran without effect

Found by pinning the perception error and diffing per-leg traces, after the route-choice assignment
fix below made a real comparison possible for the first time. Each one reported a number while doing
nothing, and each is a regression against the 2020 `RegionBased`/`LandmarkBased` branches, v1.11 and
the 2024 `pre-subs` branch, all three of which were checked and all three of which agree.

**Region navigation returned the shortest path on 100% of OD pairs.**
`RegionBasedNavigation.getKnownGateways()` kept only gateways whose entry and exit were in
`CognitiveMap.getNodesInKnownNetwork()` — empty for any agent whose map was never individualised, so
no gateway survived anywhere, the sequence collapsed to origin-plus-destination and routed as
distance. This is the sixth instance of the invariant `CLAUDE.md` already states: a gate on an
`agentKnown*` set needs a defined answer for an agent that has no cognitive map.
`getNodesInKnownNetwork()` and `getEdgesInKnownNetwork()` now answer with the **community cognitive
map** for such an agent, which is the same formula an individualised map uses with the personal half
empty: `fuseBoneWithCommunityNetwork` is *own bone + community known network*. It is a subset of the
city — main roads, city centre, salient junctions — and deliberately so, because these agents route
over the full network but plan among the places everyone knows. Night agents keep their simple bone,
which is non-empty.

**And it latched off.** Three sites disable region navigation mid-trip by writing to the agent's
properties, which outlive the trip; a cityImage agent is built once and walks its whole OD matrix, so
the first within-region pair disabled it for the rest of the run. `RoutePlanner.definePath()` now
restores the entry value in a `finally`.

**Barrier sub-goals were never generated** — the same empty set, filtering a barrier's `edgesAlong`
down to nothing. What separated `BARRIER_*` from its sibling until now was only the cost multiplier
in `Dijkstra.costPerceptionError`. On London/subdivisions the barrier effect goes from 1.033x to
**1.574x** on distance.

**On-route marks have never been inserted by this code.**
`CognitiveMap.getWayfindingEasinessThreshold` was an unimplemented stub returning 0, and
`LandmarkNavigation` looks for sub-goals only *while* easiness is below it. v1.11's two values had
already been carried into `RouteChoicePars` and left with no reader; core returns them now. The tell
had been visible and was read as agreement: three differently-configured landmark models scored
1.0740 on 255 ODs, to four decimal places. Two further gates on the same path were closed with it,
neither sufficient alone — `findSalientJunctions` filtered against an unsatisfiable known-node set
(and its recovery loop then silently ran at a looser percentile, for community and individualised
agents alike), and `findKnownLocalLandmarks` collected nothing, leaving local landmarkness at 0.0 for
every node of every route.

**The threshold fix reaches only the OD modules.** `setUsingLocalLandmarks` is called from
`CityImageAgentProperties` and `EmpiricalAgentProperties` and nowhere else, so activity, night and
learning agents never reach `LandmarkNavigation` and no figure from those tiers is affected.

**Two of cityImage's nine scenarios were the same configuration.**
`activateLandmarks()` tested for the substring `"LANDMARKS"` before the `LOCAL`/`DISTANT` qualifiers,
so `LOCAL_LANDMARKS_*` had distant landmarks on as well. Nine named models, seven distinct
configurations. All nine are now distinct, and local-only is a smaller detour than local-plus-distant
on both heuristics.

**`EmpiricalAgent` bypassed `setRoute()`**, assigning the field directly, so `RouteTrace` saw no
planned metres from any empirical run. `CityImageAgent` always called it.

**Working region navigation then exposed a reproducibility defect underneath it.** Two runs of one
seed disagreed on 18-20 of 150 ODs per `REGION_*` model while the non-region models were identical:
`findNextGateway` collected candidates into a `ConcurrentHashMap<Gateway, Double>` from a
`parallelStream` and `Utilities.sortByValue` is stable, so ties were broken by identity-hash
iteration order, which HotSpot varies between runs. `Gateway` overrides neither `hashCode` nor
`equals`. It is a `LinkedHashMap` filled by an ordinary loop now, and two runs are byte-identical.
Third instance of this class in this repository, after `NodeGraph`/`EdgeGraph` and `Agent`.

Measured: on London/subdivisions every `REGION_*` model was byte-identical to its non-region sibling
on 150 of 150 ODs before, and on 32.7%/22.7% after. On Muenster, 39-61% of empirical legs changed
route and the population walked 11.4% further, with a 4x spread across the survey clusters. `ROAD_DISTANCE` is beaten on 0 of
150 and 0 of 255 ODs throughout, so the distance baseline stays minimal and the comparisons hold.
Per-module detail in `src/main/java/pedsim/cityimage/TODO.md` item 5 and
`src/main/java/pedsim/empirical/TODO.md`.

**Two pieces of tooling, because none of this was measurable before.**
`RouteChoicePars.perceptionErrorSD` replaces the hard-coded 0.10 in `Dijkstra.costPerceptionError`,
so `--perceptionErrorSD=0` pins the multiplier to 1.0 and no longer needs a source edit before every
model comparison. The draw is still made, so the random stream is unchanged. And
`-Dpedsim.trace=<file>` writes one line per planned leg from `Agent.setRoute()` — scenario, agent,
trip, OD, node and edge counts, length — with no timestamps, so two files are byte-comparable between
runs and between machines. Every table above was computed from it.

### 15 September — cityImage was never running the models it reported

**Every route-choice comparison this module has produced is void.** `CityImageAgent.planRoute()`
called `initialiseHeuristics(false)`, which let `Heuristics` re-decide route choice and overwrite the
model the agent was constructed with — a coin flip between shortest path and simplest path, since no
caller of `setActivationProbabilities` exists and the probability-driven branch is unreachable. Nine
models were two routes in varying proportions, and the landmark, region and barrier branches of
`definePath()` were never reached. The tell was that `ROAD_DISTANCE` was beaten on 125 of 255 OD
pairs, by up to 2,850 m, which a distance-minimising search cannot be.

Five fixes. `AgentProperties.isRouteChoiceAssigned()` (false by default, true for cityImage and
empirical) stops `Heuristics` re-deciding a model that was assigned, while `initialiseHeuristics`
still builds and stores the object the routing code dereferences. `Landmarkness` skips zero-distance
anchors and admits only finite scores into `Math.max` — a refactor from `if (score > best)` to
`Math.max(best, score)` had turned NaN from silently discarded into propagated, which made a node's
landmarkness NaN, its cost `(1 - NaN)/length`, and every comparison against it false, so the node was
never relaxed and a destination carrying such an anchor was unreachable. `GlobalLandmarksPathFinder`
falls back to road distance on both exits instead of calling `computeRouteSequences()` on an empty
sequence, and `RunLedger.landmarkFallbacks` counts it. `CognitiveMap` answers `getAgentKnownRegions`,
`getAgentKnownBarriers` and `isRegionKnown` for agents that never individualise a map.

**Region navigation remains inert** — `REGION_DISTANCE` is byte-identical to the shortest path with
both region gates open. Open, with the next step, in `cityimage/TODO.md` item 5.

Verified deterministically on London/landmarks: `DISTANT_LANDMARKS` 1.632x the shortest path,
angular-family landmarks 1.28x, distance-family 1.076x, and 0-2% of routes identical to shortest
where before every landmark model matched it to the metre. Barriers reach 1.033x on distance.

**Two claims were made and retracted the same day** because the +/-10% perception error was read as
signal. Any model comparison here needs that draw pinned first; `CLAUDE.md` says how and why.

### 14 September (last) — one activity agent, and less machinery around it

**`CommuterAgent` is gone; `LearningAgent` extends `ActivityAgent`.** The learning module already ran
on `ActivityEngine`, `ActivityPopulate` and `ActivityTravelDemand`; only its agent class stood
outside the activity tier, and `ActivityPopulate` gates the persona block on
`instanceof ActivityAgent`, so learners had no persona, no agenda and no commute mode while the
travel demand computed a budget from personas and agendas — hence `0 mandatory legs` and `1.00 legs
each` on every learning run. Learners now hold the full programme, and the working day (the
worked-today latch, the work-targeting rule, the stay) is part of `ActivityAgent`, which was
`CommuterAgent`'s only remaining subclass. **Changes learning results**, of which there were none
older than 14 September.

**Core no longer counts darkness.** The per-leg dark/light tally came out of `Agent.setRoute` and
`RouteTrace`: darkness is a night-module concern, and the volume exports already carry it per edge per
day in the `LIGHT` / `DARK` columns. `PedSimCity.isDarkHour(hour, day)` stays, because the exporter
needs it.

**The replicate summary lost its class.** `ReplicateSummary` (125 lines) is a dozen lines inside
`Engine`: one row per finished job, and the mean and sample sd across them when there is more than
one. The numbers it reports are the same.

**Formatting became automatic** (`b6c7529`, `4bfa34e`). `pom.xml` gained spotless-maven-plugin 3.6.0
driving google-java-format 1.28.0 over `src/main/java/**`, `src/test/java/**` and root `*.java`, and
both git hooks now act on it: `pre-commit` applies the formatting and re-stages the Java files that
were already staged, `pre-push` checks and aborts on a violation. Commit time is where the fix
belongs — a pre-push hook can only stop you, because the commits it is about to push already hold the
unformatted code. **No behaviour change**, and the hooks are copied per clone rather than wired
through `core.hooksPath`, which would bypass Git LFS's own hooks.

Two consequences worth knowing, both documented in `CLAUDE.md` and `.githooks/README.md`:
`spotless:apply` formats the **whole tree** while the hook re-stages only what you staged, so files
you never opened come back reformatted and dirty — `<ratchetFrom>` would confine it and is not set
yet; and `SKIP_SPOTLESS=1` is read by *both* hooks, so exporting it in a shell turns formatting off
at commit and push together.

**The tree-wide sweep itself is not committed yet.** Running the formatter over the repo touches 58
Java files, essentially all of it whitespace and google-java-format's line re-wrapping; the plan is
one isolated formatting commit plus a `.git-blame-ignore-revs`, so it costs one diff rather than
polluting every future one. See `TODO.md`.

### 14 September (later still) — what the fixed night window misses, and three routing fixes

**The exporters' night is not the model's night, and in winter the gap is most of the walking.**
Mandatory legs run 06:30-19:30, so removing the darkness guard did not push commutes into the fixed
`[20:00, 06:00)` aggregation window — it pushed them into *behavioural* darkness, which the window
does not see. `RouteTrace` counts both now and the day's line reports them. On `Torino_simplified` at
338 agents: **1 June, 29/166 legs begun in darkness and 0 outside the window; 7 December, 90/172 in
darkness and 63 outside it** — 37% of the day's walking reported as daytime volume while the agents
walked it in the dark and behaved accordingly, since lighting-aware routing, park and water refusal
and vulnerable detours all key off `isDark`. Whether to aggregate on the window, on darkness, or on
both is now a stated decision rather than an assumption nobody had tested.

**Night means dark for that date.** The pedestrian-volume exports aggregate the hourly columns on
the seasonal sunrise and sunset of the day being exported instead of a fixed clock window, and those
two columns are now `LIGHT` and `DARK`. `PedSimCity.isDarkHour(hour, day)` is the seam: core answers
with the fixed `TimePars` window, the activity tier with the daylight model at the city's latitude.
The same `Torino_simplified` day reports 8% of volume in the dark on 1 June and 56% on 7 December.

**Every run this repo has ever produced was 1 June** — the shortest night of the year, in a model
about darkness — because `TimePars.SIMULATION_START_DATE` could only be changed by editing the class.
`ParameterManager` parses `LocalDate` now, so `--SIMULATION_START_DATE=2026-12-07` works.

**A fleeing night agent avoids the edge it is fleeing.** `NightAgentMovement.defineEdgesToAvoid`
added the current edge in the non-vulnerable branch only. A vulnerable agent's avoid-set is the whole
city minus what it knows, and the problematic edge is normally a street it or the community knows, so
it was subtracted straight back out and A* could return a "bypass" that ran down it. The one
population the module is about was the one that could not get away from what frightened it.
**Changes vulnerable-agent results**, which is the A/B's manipulated arm.

**`LocalHeuristicMode.NONE` means unset, and unset routes by distance.** The final ternary in
`RoutePlanner.definePath` tested "is it distance", so every mode that was not DISTANCE - the
constructed default included - resolved as angular; it is a positive `isLocalHeuristicAngular()` test
now. Nothing that configures itself could reach it with NONE (`Heuristics` always sets a mode,
cityImage's route-choice names all carry DISTANCE or ANGULAR except `DISTANT_LANDMARKS`, which
returns earlier, and empirical always sets one), so no configured module's behaviour moves - only the
callers that never chose a model. Those now also get a warning: `AgentProperties.isConfigured()` is
false when nothing is set, and `RoutePlanner` says so once per run, naming the agent and pointing at
`Agent.planRoute()`.

### 14 September (later) — the winter commute, replicate variance, and four settled questions

**People commute in the dark again.** `CommuterAgent.shouldGoToWork`, `ActivityAgent.shouldGoToWork`
and `ActivityAgent.planMandatoryDeparture` all consulted darkness: the first two refused to set off
after dark and the third refused to draw a departure into it, so that the leg budget was not charged
for a commute that would not happen. Turin's sunset is before 17:00 through December, so between
them they deleted the winter commute — the most routine walking there is, made by the population most
exposed to unlit streets, in the one module built to study exactly that. The persona's start window
decides when somebody leaves; the season decides whether it is light when they do. **Changes
results**: more of the day's legs go to commuting in winter and fewer discretionary chains are
bought, and night aggregates over `[20:00, 06:00)` now contain commutes. A summer day is unchanged,
which is why nothing showed in the June runs.

**A run reports its own variance.** `--jobs=N` always gave N replicates (job *n* uses `seed + n`) but
nothing ever compared them, so a single run's number carried no error bar and a difference between
two conditions could not be told from a difference between two seeds. `ReplicateSummary` now prints
each job's legs, planned and walked metres and metres per agent, then the mean, sample sd and range
across them; `RouteTrace` gained job totals that the daily reset leaves alone, so a multi-day run
reports the run rather than its last day. First measurement, two jobs on `Torino_simplified` at 169
agents: **sd 6.5% of planned metres, 12.9% of legs** — several times the cross-machine disagreement,
and the floor any claimed effect has to clear.

**The student/worker overlap is an assumption instead of a silent double count.** The census gives no
enrolment variable at section level, so students were the 15-24 age band while P101 counted everyone
employed at 15-64: the employed young were both, and the residual borrowed them from flex.
`Persona.sample` thins the student share by the new `ActivityPars.youthEmploymentRate`, leaving them
among the workers, where a commute belongs. The default 0.18 is a national order of magnitude and not
a Turin figure, and says so.

**Opening windows can now come from the city.** `ActivityPurpose`'s 32 numbers are defaults rather
than facts: a city file may set `purpose.<NAME>.open` / `.close` / `.stayMinutes` / `.staySigma`, and
`CityConfig` applies and reports them like any other key, resetting the enum first so one JVM can run
two cities. `Torino.properties` carries the block commented out — nothing in the pipeline reads OSM
`opening_hours`, and inventing Italian-sounding hours would only move the invention somewhere that
looks sourced.

**`Pars.departuresPerPersonPerDay` has a source**: 0.255, from ISFORT's 0.51 walked legs per resident
per day halved, because a core agent's departure is an out-and-back. It stays in core, which was the
open question — core has to answer `TravelDemand` on its own — while the activity taxonomy stays
behind the interface.

**`RemoteLauncher` has a `main`**, so the remote-run capability orphaned by the AWT GUI's removal is
reachable again as a command rather than a class nothing calls. The deprecated
`ParameterManager.initFromArgs(String[])` is deleted: both remaining callers had already moved to
`ModuleLauncher`, and with it goes the way a module parameter could be accepted and then ignored.

### 14 September — the learning module runs, and a seed stops depending on the machine

**A seed did not reproduce a run across machines.** `gdsl1` and the Windows laptop each replayed
themselves exactly and disagreed with each other by about 1% of trips, on the same seed, code, data
and jar. Not floating point: `Math.exp`, `log`, `pow`, `sin` and `sqrt` were checked over 200,000
inputs and agree bit-for-bit across the two. The cause is that **`NodeGraph` and `EdgeGraph` override
neither `hashCode` nor `equals`**, so a `HashMap`/`HashSet` keyed on one iterates in identity-hash
order, which HotSpot derives from a per-JVM generator that differs between JVM builds.
`PoiClassifier` built the per-purpose attraction maps that way and `WorkplaceChoice.draw` walks their
entries into a cumulative distribution and picks by position, so one random number chose a different
workplace on each machine, which moved the commute distance and then `walksToWork`. Now
`LinkedHashMap`; `NetworkBuilder`'s known-network edge sets and
`CognitiveMap.deriveOtherKnownRegions`'s per-region buckets became `LinkedHashSet` for the same
reason, since `Islands.mergeConnectedIslands` iterates them. **The whole population layer is now
identical across the two machines** — mandatory legs, both walked commute shares, the length bands.
Trips and metres still differ by about 1%; the remaining path is downstream, in destination choice or
routing, and is not found. Until it is, a comparison must be run entirely on one machine.

**The learning module completed a day for the first time.** Three defects, none of which could be
seen because the first one crashed the run:

- `IncrementalLearning.buildBasicMemory()` plans through a `RoutePlanner` it builds itself rather
  than `Agent.planRoute()`, so `initialiseHeuristics()` had never run and the agent's properties sat
  at `MinimisationMode.NONE` / `LocalHeuristicMode.NONE` — which `RoutePlanner`'s final ternary reads
  as angular. Seed memory was therefore angular-routed regardless of the agent's route choice, and
  threw inside `NodeGraph.getDualNodes` on a primal-only city. It now samples route choice per seed
  route, which is what the `randomizeRouteChoiceParameters` TODO asked for and what makes the loop
  mean anything: origin and destination are fixed, so without it all five iterations recomputed one
  path. `RoutePlanner` additionally refuses angular routing when no dual graph is loaded.
- `applyDecay` took the percentile threshold **before** the decay and compared it with the values
  **after**. A uniform multiplication preserves ranking and so cannot move cells across a percentile;
  worse, with `usingMeaningfulness` off every cell sits at exactly 1.0 and the 15th percentile lands
  there, so the *entire* active set "dropped" on every step of every learner, each time rebuilding
  the cognitive map. Taken from the decayed grid now. **The modelling question — absolute threshold
  or percentile — is still open** and is recorded in the learning TODO.
- `buildBasicMemory` rebuilt the cognitive map once per seed route while `readjustCognitiveMap`
  re-derives everything from the current grid, so four of five rebuilds were discarded. Accumulates
  now, rebuilds once.

**Learning performance**, all behaviour-preserving and confirmed against identical run output:
`CognitiveMap.readjustCognitiveMap` walks the network's nodes once against an STRtree of prepared
collage polygons instead of calling `Graph.getNodesWithinPolygon` per polygon (that method scans
every node and builds a fresh JTS geometry graph per test, so cost was nodes × polygons) — 16 agents
went 4 m 10 s → 2 m 45 s; and the landmark-membership filters in
`RouteProperties.computeRouteProperties` hoist `getGeometries()` out of the per-building lambda into
a `HashSet`, which was handing back a fresh `Bag` and scanning it linearly once per building.
`LearningPars.cognitiveMapRebuildFraction` throttles rebuilds to a material change in the remembered
space. It remains slow — about 10 s per agent, nearly all one-off seeding, with
`LearningPars.cellSize` the lever.

**Verified on `gdsl1`.** The server tree was carrying **July's `Torino_censusData.gpkg`** — the
vintage drift the night TODO warns about, and it matters here specifically because the current census
carries `centroid_lat`, so the run takes Turin's latitude (45.0691°) instead of falling back to
`ActivityPars.latitudeDegrees = 53.4`, Liverpool's, which sets every sunset in a model about
darkness. With it shipped: a Torino day at 4,232 agents in 1 m 25 s, clean ledger; the night A/B path
spawning 72 pairs and writing 158 paired trips, twins sharing origin and destination with the
vulnerable one detouring.

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
`TravelDemand` and `RouteTrace`, reached through `state.travelDemand()` and `state.ledger()`. No
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
