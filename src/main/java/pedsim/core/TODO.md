# Core — what is left to do

Split out of `/TODO.md` on 13 September 2026. Core is the machinery every module inherits: the primal
and dual graph, regions, barriers, landmarks, the cognitive map, route choice and path finding,
movement, the day loop, flow accumulation and export. It models no behaviour, and nothing here should
acquire any.

---

## 1. ~~The remote-run route has no caller~~ — it has a `main` now, 14 September 2026

`pedsim.core.server.RemoteLauncher` could launch and stop a run over SSH but nothing called it: its
only caller was `PedSimCityActionHandler`, deleted with the AWT GUI, and the dashboard's REST surface
has no remote-run endpoint. Of the three ways out — a dashboard endpoint, a CLI entry point, or
deleting `pedsim.core.server` — it took the middle one, so a remote run is a command like every other
run and is reproducible from what is written down:

```bash
java -cp "target/classes;<deps>" pedsim.core.server.RemoteLauncher     --remoteMainClass=pedsim.night.launcher.NightLauncher     [--remoteProjectDir=...] [--sshKey=...] [--server=user@host]     -- --headless --cityName=Torino --days=1

java ... pedsim.core.server.RemoteLauncher --stop --remoteMainClass=...
```

Defaults come from `server.properties`; anything after `--` goes to the remote run; `runOnServer`
returns the process so the CLI can wait on it rather than exiting before the first line of output.

**Two things it does not fix.** `server.properties` still points `ssh.key` at a `C:` path that no
longer exists — pass `--sshKey=id_ed25519`. And the remote command still does `git pull` and compiles
there, so it runs *committed* code: uncommitted work has to be shipped by hand, as in `CLAUDE.md`.

## 2. Size the length error for full-network escalations

An agent planning against a route through streets it has never walked carries the same ±10%
perception error as one on streets it knows. On a Torino day, 34 of 197 trips took that path, so it is
not an edge case. Flagged at each site in `RoadDistancePathFinder` and `AngularChangePathFinder`;
sizing it needs a source.

## 3. ~~`Pars.departuresPerPersonPerDay = 0.25` is invented~~ — sourced, 14 September 2026

It is **0.255** now, and it comes from the same place the activity tier's figure does: ISFORT's 2.53
trips a day for the mobile 80.8% of the population → 2.04 per resident across all modes → above a
quarter walked in a large north-western city → **0.51 walked legs per resident per day**. A core
agent's day is one departure — out to somewhere it knows, then home — so a departure is two legs, and
0.51 / 2 = 0.255.

**It stays in core**, which was the open question. Core has to be runnable on its own:
`AgentReleaseManager` asks `TravelDemand` how many people set off and `BaselineTravelDemand` has to
answer. What would not belong here is the activity taxonomy — chains, agendas, personas — which is
why the legs-to-departures conversion is stated in the javadoc rather than parameterised. The
activity tier converts the same 0.51 using the chain length its realised persona mix produces.

## 4. Publish GeoMason-light 2.2.0, or stamp the rebuild

One version number currently covers two different builds — 117,232 bytes in the local `.m2`, 115,882
on `gdsl1` before it was replaced. Nothing detects the difference. On 13 Sep it presented as a
`NullPointerException` on a null `nodeID` inside `Environment.prepareGraph`, which points nowhere near
the cause; what isolated it was running the same city locally, where identical code and data completed
cleanly.

Until it is published, a fresh clone elsewhere will not resolve the dependency at all.

## 5. Find the rest of the cross-machine divergence

The population layer stopped depending on the machine on 14 September: `NodeGraph` and `EdgeGraph`
override neither `hashCode` nor `equals`, so a `HashMap`/`HashSet` keyed on one iterates in
identity-hash order, and HotSpot derives identity hashes from a per-JVM generator whose values
differ between JVM builds. `PoiClassifier`'s attraction maps, `NetworkBuilder`'s known-network edge
sets and `CognitiveMap.deriveOtherKnownRegions`'s per-region buckets are insertion-ordered now.
**Trips and metres still differ across the two machines by about 1%**, so something downstream still
walks such a collection into an ordered decision.

Checked since, and clean — the order reaches no decision:

- `ActivityAgent.favouritePlaces` is already an `EnumMap` of `LinkedHashMap`s, so preferential
  return builds its cumulative distribution in insertion order, and `evictLeastVisited` breaks ties
  on the place learned earliest rather than on a hash;
- `PedSimCityActivity.nodesPurposeWeight` is an `EnumMap` of those `LinkedHashMap`s;
- `PedSimCityNight.nodesVulnerabilityWeight` is a plain `HashMap<NodeGraph, Double>`, but it is only
  ever `getOrDefault`-ed per home node, never iterated;
- `Agent.defineRandomDestination` filters a `NodesLookup` list with `retainAll` against a `HashSet`,
  which removes elements without reordering the list, and `selectWeightedDestination` then walks the
  list by index;
- `Dijkstra`'s `knownEdges`, `knownNodes`, `visitedNodes`, `nodeWrappersMap` and the region-subgraph
  sets are membership and lookup only. Its priority queue compares cost alone, so equal-cost ties
  break on insertion order — which follows each node's out-edges, and those come from the graph.

**Still suspect, and all three are in GeoMason-light rather than here:**

- ~~`Islands.findDisconnectedIslands` undoes the fix at its own door.~~ **Fixed in GeoMason-light on
  14 Sep 2026.** It used to do `new HashSet<>(GraphUtils.nodesFromEdges(edges))` on whatever set it
  was handed — and `nodesFromEdges` was a `Collectors.toSet()` — so the island *list order* was
  identity-hash ordered however carefully the caller ordered its edges, and `findConnectingBridge`
  returns the **first** edge it meets between two islands scanning in that order. Which street joined
  an agent's known network therefore depended on the JVM build, and every route planned on it
  followed. Those collections are `LinkedHashSet`s now, so `NetworkBuilder`'s ordering survives into
  the island search. It reached the activity and learning tiers and not night, so it was never the
  night module's remaining 1% — **that is still open.**
- `NodeGraph.getDualNodes` builds a `HashMap` and hands it to `Utilities.sortByValue`, whose sort is
  stable, so dual nodes at exactly equal cost come back in identity-hash order. Ties only, but it
  reaches every angular route.
- `Graph.salientNodes` is a `HashMap`, and `GlobalLandmarkNavigation.findOnRouteMark` lists its keys,
  sorts them stably by score and takes the last. Ties only, again.

**The general fix does not work, and here is why — do not try it again.** The obvious move is to give
`NodeGraph` and `EdgeGraph` a `hashCode()` derived from their id, leaving `equals` as identity; that
is legal and would make every hash-ordered iteration deterministic at once. It would also break
routing. `Graph.generateAdjacencyMatrix()` runs inside `fromStreetJunctionsSegments`, keying
`adjacencyMatrix` and `adjacencyMatrixDirected` on `Pair<NodeGraph, NodeGraph>` — and javatuples'
`Tuple.hashCode()` is `final` and derived from its elements' hash codes. PedSimCity assigns node ids
afterwards, in `Environment.prepareGraph`, so every key would be hashed at id 0 and then looked up at
its real id: `getEdgeBetween` would start missing. An id-based hash needs the ids assigned before the
graph is built, or the adjacency matrix keyed on ids rather than on objects. A hash over the node's
coordinate would be stable — it is fixed at construction — but that is a bigger change than the one
call site that was actually costing anything.

## 6. ~~`RoutePlanner` can be built on unconfigured properties~~ — both halves fixed, 14 Sep 2026

**NONE now means unset, and unset routes by distance.** The final ternary tested
`isLocalHeuristicDistance()`, so every mode that was not DISTANCE — including the constructed default
`LocalHeuristicMode.NONE` — resolved as *angular*. It is written as a positive test now,
`isLocalHeuristicAngular() && angularAvailable()`: angular is a stated preference, and shortest path
is what is left when there is none.

**The blast radius was smaller than it looked, and worth recording.** Nothing that configures itself
could reach that ternary with NONE: `Heuristics` always sets a minimisation mode or samples a local
heuristic (never NONE); cityImage's every `Scenario` name contains DISTANCE or ANGULAR except
`DISTANT_LANDMARKS`, which returns from an earlier branch; empirical always sets one of the two. So
the only callers this changes are the ones that never chose a model at all — which is exactly the
bug, and no configured module's behaviour moves.

**And a planner built on unconfigured properties now says so.** `AgentProperties.isConfigured()` is
false when no minimisation mode, local heuristic or element is set; `RoutePlanner`'s constructor logs
one warning per run naming the agent and pointing at `Agent.planRoute()`. That is the seam that runs
`initialiseHeuristics()` first — the learning module's seed memory was routed for its whole life by a
model none of its agents held, and nothing in the output said so.

## 7. Smaller

- **The day's ledger line now carries darkness exposure** — legs begun while the state considered it
  dark, and how many of those fall outside the fixed `[20:00, 06:00)` window the exporters aggregate
  on. Core answers `isDark()` with the fixed window, so for a bare core run the second number is
  always zero; the activity tier overrides it with the seasonal flag, where it is not.
- **`TimePars.SIMULATION_START_DATE` is settable from the command line**, now that
  `ParameterManager` parses `LocalDate`. Before this it could only be changed by editing the class,
  so every run ever made in this repo was 1 June.
- **`agent_release_day_N.csv` lost two columns.** It was
  `step,datetime,meters_to_allocate,meters_spent,agents_released` and is now
  `step,datetime,agents_released`. Nothing in this repo reads it; check `../vodafoneAPI/` and the
  analysis notebooks.
- ~~**The deprecated `initFromArgs(String[])` still has callers.**~~ It had none: both the cityImage
  and empirical launchers already went through `ModuleLauncher`. The overload is deleted (14 Sep
  2026), so the only way in is `initFromArgs(String[], Class[])` with the module's
  `parameterClasses()`, and a module key can no longer be accepted and silently ignored.
- **`RouteChoicePars` and `Pars` share a copy-pasted class javadoc** ("contains global parameters and
  settings…"), which describes neither.

---

## Invariants — do not break these

- **The departure share is a density integrating to 1.0 over the day**, so it must be integrated over
  the interval between release events (`TimePars.releaseAgentsEveryMinutes`), never over
  `STEP_DURATION`. Integrating over the wrong one scales the whole day by their ratio with no error to
  show for it. Any change to the step size or the release cadence must keep this correct.
- **A single job is not a result.** A run is deterministic from `Pars.seed`, so one job per condition
  has no error bar; job *n* uses `seed + n`, and `Engine` logs each replicate's totals plus the mean
  and sd across them at the end of a run. First measurement: sd 6.5% of planned metres over two jobs
  at 169 agents. Compare effects against that spread, not against zero.
- **One sub-goal loop.** `PathFinder.routeSequence` holds the walk over a sub-goal sequence and takes
  the per-leg routing as a lambda; road-distance and global-landmark routing go through it. Angular
  keeps its own, because it searches the dual graph over two candidate centroid lists and corrects
  edge directions with `cleanDualPath`. A fourth sub-goal router goes through `routeSequence` — the
  three copies that existed had drifted into a direction-correction bug and an off-by-one on
  `tmpOrigin`.

- **`SharedCognitiveMap` clears its own statics.** `PedSimCity.clearStaticData()` calls
  `SharedCognitiveMap.clearStaticData()`, which empties the road classification, the community
  network, the lit/park/water sets and the route caches. Everything it clears is rebuilt by
  `Environment.prepare()` → `setCommunityCognitiveMap()`, which runs after the import. Anything added
  as static state in that class belongs in both.

- **`RouteTrace` is measurement, never an input.** Feeding the planned-versus-walked gap back into
  allocation runs away: the measurement lags, the difference stays negative, and subtracting a
  negative raises the allocation.
- **Trip lengths are walked metres; node lookup is Euclidean.** Everything that picks a destination by
  distance converts through `NetworkCircuity.straightLineFor()`. Dividing at the call site instead is
  how one field came to mean two different quantities.
- **A collection keyed on a graph object must be insertion-ordered wherever its iteration reaches a
  decision.** `NodeGraph` and `EdgeGraph` inherit identity hashing, and HotSpot's identity hashes
  differ between JVM builds, so a `HashMap` or `HashSet` of them iterates one way here and another
  way on the server. Membership tests and lookups are safe; cumulative draws, first-match-wins scans
  and stable sorts are not. Use `LinkedHashMap`/`LinkedHashSet` wherever the insertion order is
  itself deterministic, and see item 5 for what is not yet covered.
- **`SimulationModule.parameterClasses()` is the single list** consulted by the command line and by a
  module's city configuration. A parameter class omitted from it is unreachable from both, and a key
  naming one of its fields is accepted and then ignored.
