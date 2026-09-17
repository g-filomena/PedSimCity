# What to work on next

Open work lives per module. This file is the index plus the few things that belong to no single one.
What has already been done is in `CHANGELOG.md`; how the code works is in `CLAUDE.md`.

Each module file leads with what invalidates earlier output from that module — read that before
trusting an old result.

| module | what is in it |
|---|---|
| [core](src/main/java/pedsim/core/TODO.md) | the remote-run route has no caller; escalation length error; `departuresPerPersonPerDay`; the rest of the cross-machine divergence; `RoutePlanner` on unconfigured properties; the invariants not to break |
| [activity](src/main/java/pedsim/activity/TODO.md) | **its commute figures are superseded** — layer 2's invented interior and two microdata requests to start now; `distanceWeight`; `choiceSetRadiusMetres`; the lost commuting check; the long commute tail |
| [night](src/main/java/pedsim/night/TODO.md) | **the four seasonal runs are void** — vulnerable agents looped until 17 Sep; A/B design, threats to validity, void comparisons |
| [cityImage](src/main/java/pedsim/cityimage/TODO.md) | **regions, barrier sub-goals and on-route marks all ran without effect until 15 Sep** and are now measured; cumulative landmarkness still unimplemented; London's two networks |
| [empirical](src/main/java/pedsim/empirical/TODO.md) | **the clusters walk disjoint OD sets, so they cannot be compared to each other**; only Muenster has cluster data; what not to re-add after `PopulationPars` |
| [learning](src/main/java/pedsim/learning/TODO.md) | **it ran for the first time on 14 Sep** — three fixes, the decay-threshold decision still open, and what makes it slow |

Social has no list of its own; it inherits the activity tier's.

## Cross-cutting

- **`gdsl1` cannot build the current tree.** It holds GeoMason-light 2.2.1 and a pre-17-September
  checkout; the night reroute needs 2.2.2's `Astar` predicate and multi-target searches. After
  fetching 2.2.2, delete the version folder from that `~/.m2` first and compare `sha256sum` — same
  version string, two builds is the failure mode this project has already been bitten by.

- **The four seasonal night runs have to be redone**; see the night list.

- **The per-agent memory slope has never been measured**, so what a 50% population would need is
  unknown. At 3,386 agents the live heap is ~4.5 GB and the histogram says most of that is fixed
  graph cost — 5.9M `AttributeValue`, 4.8M `Coordinate`, 2.35M adjacency-matrix `Pair`s, plus the
  spatial indices. Every run so far has used the same agent count, so the per-agent part cannot be
  differenced out. Two short runs at different `--percentage` settle it, and give the CPU slope too.
  Do that before quoting a figure for 423,283 agents. Routing is no longer the constraint.

- **Two modules are not in the repository.** `.gitignore` excludes
  `/src/main/java/pedsim/social/` and `/src/main/java/pedsim/learning/`, so both exist only on the
  machine that wrote them - while `pom.xml` carries `-Plearning` and `-Psocial` profiles that
  compile them, `TODO.md` indexes their module files, and `CLAUDE.md` describes the learning
  module's first successful run. A fresh clone cannot build either profile. Decide which is true:
  they belong here and should be tracked, or they belong in `PedSimCityLearning` and the profiles,
  the index and the documentation should say so.

- **~~Name the two routing packages apart.~~ Done 16 Sep 2026.** `pathfinder` and `pathfinding`
  differed by two letters and neither said which was which. They are now `routing/routers` (the
  route builders — `PathFinder` and its subclasses) and `routing/search` (the graph searches —
  `Dijkstra` and its subclasses), in core and in night alike, which are the words `CLAUDE.md`
  already used for the two tiers. Each carries a `package-info.java` naming its tier and the ones
  above and below it, and the locals that held a `Dijkstra*` under the name `pathfinder` are called
  `search`.

- **`learning/routeMemorability` is the last camelCase package.** `empirical/agent` became
  `agents` and `learning`/`social`'s `cognitiveMap` became `cognitivemap`; this one needs a word
  rather than a lowercasing (`routememorability` is unreadable). `learning/memorability`?

- **~~Decide the console-versus-log policy.~~ Settled 16 Sep 2026: `LoggerUtil` takes
  precedence.** A run's own account of itself is a log record, so the 24 `System.out`/`System.err`
  calls are gone. The two that were tables — `CommuteCalibration`'s parameter sweep and
  `PedSimCityActivity`'s transit summary — are assembled into one string and logged once, because a
  table read line by line through a formatter that prefixes each row with its level is not a table.
  `ParameterManager`'s three `System.err` writes became `warning`/`severe`.

  **One exception, stated on the method: `RemoteLauncher.USAGE`.** Help text is the program's
  output, not a record of what it did, so `--help` still prints; the two failures that accompany it
  are logged. Also removed: four commented-out debug `println`s in `BarrierBasedNavigation`.
  (`IncrementalLearning` still carries one inside a fully commented-out method body — that is dead
  code to delete, not a logging decision.)

- **~~Decide on lux in the night route cost.~~ Landed 16 Sep 2026.**
  `DijkstraRoadDistanceNight.lightingCostMultiplier` raises a **known** edge's Dijkstra cost as its
  `mean_lux` falls below the travelling agent's threshold, toward
  `NightPars.maxKnownDarkEdgeCostMultiplier` (1.5) at total darkness, so an agent can prefer a lit
  way round before setting off instead of only reacting once on a dark edge. Unknown edges are
  untouched, so planning and situated reaction cannot charge for the same darkness twice. It is the
  one change in the module that moves the *plan*. **Unmeasured**: the last figures on it are from a
  December Torino day that planned 242,419 m without it against 241,963 m with the other three night
  changes also reverted, and the module has changed twice since. Set the parameter to 1.0 for the
  control.

- **~~Decide on `ActivityPars.distanceWeight` against the measured circuity.~~ Decided 16 Sep 2026:
  leave it at 0.0012.** Only the product of it and `Pars.networkCircuityFactor` sets any choice
  probability, and measuring the factor from the network (1.292 on Torino, against the hardcoded
  1.41 it was fitted under) drifted that product by 8.4%. Holding it invariant at 0.0013096 would
  re-import through the coefficient exactly the circularity that measuring circuity removed — 1.41
  was itself measured on the old mechanism's trips. An 8.4% drift in a coefficient that has never
  been fitted is not a result; inheriting a superseded mechanism's trip lengths would be. Recorded on
  the field, and note that `Torino.properties` sets the key, so the Java default alone does nothing
  for Turin. Fitting it properly still needs the Audimob microdata.

- **The lighting pipeline is re-run, the falloff law is decided and the utilisation factor is gone
  (16 Sep 2026).** Torino's layer is rebuilt; **the five other cities are not**, and their lighting
  layers are now the odd ones out. Derivations and every number are in `pipeline/README.md`.

  - **Step 3 is ten minutes, not half an hour.** The per-pair `LineString` loop is bulk array work —
    one vectorised interpolation, one KD-tree query per block of points, one `STRtree` `crosses`
    query per block of sight lines. `pipeline/verify_step3.py` runs the old loop beside it on a real
    slice of the city: worst difference 4.5e-12 lux over 59,643 points. That is what made everything
    below possible in an afternoon.
  - **`FALLOFF_LAW = "isotropic"`, chosen by running all three over the same 44,278 Torino edges.**
    `mixed` 11.1 mean `pct_unlit` and 81.6% of edges fully lit; `isotropic` 24.3 / 56.3%;
    `lambertian` 29.3 / 44.5%. `isotropic` is `mixed` with the honest `F/(2 pi)` divisor and the same
    propagation, so it keeps the spatial pattern and only halves a level; `lambertian` changes the
    shape, and in the wrong direction for a cobra-head.
  - **The utilisation factor was the wrong quantity.** `I_down = lamp_lumens * X / norm` wants DLOR
    (EN 13032-1), the share of lamp lumens the luminaire emits downward — not the share landing on
    the carriageway, which the propagation law already works out. 0.3-0.6 by optics label became
    1.00 / 0.85 / 0.80 / 0.55 by technology and fixture class, bounded by L.R. Piemonte 31/2000
    holding ULOR to ~0. Mean 0.898 against 0.470, and the two changes cancel to within 3% on
    `mean_lux`.

  - **A lamp inside a building no longer occludes itself** up to `ARCADE_DEPTH_M` (5 m) inside the
    footprint — the arcade and wall-bracket case, which is 5,092 of Turin's 5,873 inside-footprint
    lamps. Deeper than that it is in the block and stays occluded. **The 5 m is a judgement**, from
    the observed depth distribution (median 1.6 m in, 87% within 5 m) and Turin's portico geometry;
    another city needs another number and nothing here says what.
  - **Mounting height is imputed from `tipo_supporto`, not `uso_ottica`**, by a rule rather than a
    list of Italian labels: a support type with some measured heights supplies its own median, and a
    support type with none has no pole to measure and takes `NO_POLE_HEIGHT_M`. Every lamp now
    carries an `altezza_source` column, so the 45% of the inventory running on an assumption is
    counted in the output and printed by steps 2 and 3.
  - **`NO_POLE_HEIGHT_M = 4.0` still wants a source, and is now bounded instead.** Nothing fixes it:
    EN 13201 and UNI 11248 prescribe no mounting height, CEI 64-8/7 section 714 gives only a 2.8 m
    reachability floor, and L.R. 31/2000 constrains the spacing-to-height *ratio*. Turin's PRIC
    "fascicolo completo apparecchi" (79.9 MB PDF) is the one source that plausibly settles it and
    has not been read. Measured instead, via the new `--no-pole-height` flag: 3.0 m -> 6.0 m, for
    the 38.2% of lamps this reaches, moves edges below the service level 9.8% -> 9.3% and mean
    `pct_unlit` 11.93 -> 10.79. A higher lamp has a lower peak and a wider spread and the two
    nearly cancel. Worth a source; not worth blocking a run for.
  - **A missing height now fails** in `03_street_lights.py` instead of becoming 9.0, matching the
    guard already on `downward_intensity_cd`. Step 2 keeps `DEFAULT_HEIGHT_M` as the last rung of
    its ladder, so a thin inventory still produces a layer — counted in `altezza_source` and warned
    about by name, which is what the old silent 9.0 was not.
  - **The directional visibility horizon is 15 m**, per Fotios, Yang & Uttley (2015).
  - **Still open: the four DLOR values and the 5 m arcade depth are engineering judgement.** A
    per-luminaire IES/LDT photometric file is what would replace the first, and it would settle the
    falloff law in the same stroke — none of the three laws describes a real cobra-head. Both wait
    on the same thing.
  - **Next: re-run the lighting pipeline for the other five cities**, or state in any cross-city
    comparison that only Torino's layer is built under the current physics.

- **Cross-machine reproducibility is CLOSED (14 Sep 2026).** A seed replays on any machine: core,
  night and activity give byte-identical per-leg traces between `gdsl1` and the Windows laptop on
  seed 20260912. It needed two things and neither is sufficient alone — core's release draw and agent
  scheduling ordered deterministically, *and* GeoMason-light 2.2.1's `hashCode` on `NodeGraph` and
  `EdgeGraph`. Mechanism in `CLAUDE.md`. **Every run made before that date is superseded.**

- **Give ordinary agents local landmarks, or decide not to.** `setUsingLocalLandmarks` is reached
  only from the cityImage scenario map and `EmpiricalAgentProperties`, so activity, night and
  learning agents never enter `LandmarkNavigation` at all. That is a gap in the model rather than a
  defect: the wayfinding-easiness threshold, on-route marks and landmark-weighted costs exist and no
  activity-tier agent can use them.

- **Layer 4 still has no access and egress walking, and now has no transit code either.**
  `pedsim.transit` was moved to `obsolete/transit/` (gitignored) on 16 Sep 2026: `TransitStop`,
  `TransitVehicle`, `TransitLoader`, the per-leg mode split in `ActivityAgent`, the vehicle fleets
  scheduled by `PedSimCityActivity`, the `tripsByMode` counter and `RouteChoicePars
  .usePublicTransport`. It had never been wired into layer 4 — **the pedestrian legs around a stop
  were the whole point and were the part that did not exist** — and it carried a cycle
  (`TransitLoader` and `TransitVehicle` took `PedSimCityActivity`, which held the stop lists).
  `scripts/build_transit_layer.py` stays and still builds `transit_stops.gpkg` from a GTFS feed, so
  the data side is intact.

  When it comes back: it takes the core state or a narrow interface rather than
  `PedSimCityActivity`; the mode split goes on a seam **both** `planTrip` and `startChainedTrip`
  cross, or it is a split on first legs only; and per-leg transit state needs a writer that clears
  it. Both of those last two were real defects, and both are in `CLAUDE.md` under *Layer 4*.

- **~~Extend the tests upward.~~ Written 16 Sep 2026, and they found two things.** The three that
  needed a city now exist, tagged `slow` and run with **`mvn test -Pslow-tests -Pall-modules`**
  (~60 s on Muenster). `mvn test` is unchanged at 32 fast tests in about two seconds.

  - `RouteChoiceOnACityTest.roadDistanceIsMinimalOnEveryOdPair` — with `perceptionErrorSD` pinned
    to 0, no model may beat the distance baseline. **Verified red** by unpinning it, which is also
    the cleanest demonstration of why the pin matters: at the default 0.10, five models "beat" the
    shortest path by 4–54 m on the same OD pairs, which is exactly the noise two retracted claims
    were read out of on 15 September.
  - `RouteChoiceOnACityTest.eachElementScenarioDiffersFromItsSibling` — all eight
    element-versus-sibling pairs on Muenster: region and barrier in both forms, the two combined, and
    the three landmark pairs. **Verified red** by pointing a scenario at itself. It asserts a floor of
    eight compared pairs, so a city or a test design that quietly stops loading a layer turns it red
    rather than green.
  - `SeedReproducesRunTest` — two runs of one seed write an identical per-leg trace, *and* a third
    run on a different seed writes a different one. The second assertion is the control: without it
    the first passes on a trace that records nothing a decision can move, which is how `RouteTrace`
    once reported "planned 0 m, walked 0 m" for every run while looking healthy.

  **`-Pslow-tests` did not exist until now**, though `pom.xml`'s own comment told you to use it:
  `<excludedGroups>slow</excludedGroups>` was a literal, and `-DexcludedGroups` cannot override a
  literal, so no `@Tag("slow")` test could be selected by any command. It is a property now.

  **Two findings from writing them**, both in the CHANGELOG. `PedSimCity.sightLines` was set to
  `null` to free memory and never restored, so any **second run in one JVM** died in
  `clearStaticData()` — that is the REST dashboard's normal path. And `CityImageImport.importFiles()`
  reads a different city per test design — the subdivisions design loads no buildings and no sight
  lines — so a landmark scenario run under it routes with no landmarks and reports a route rather
  than failing. Detail in `cityimage/TODO.md`.

- **`dashboard.html` and `bg.png` are still at the repo root.** `SimulationRestApi` serves them by
  filename from the working directory, so they could not move with the other nine scripts. Resolving
  them from the classpath would let them join `scripts/` — or better, a `web/` folder of their own.

- **Three methods to keep, not conclusions.**
  Comparing route-choice models needs `--perceptionErrorSD=0`, a parameter rather than a source edit.
  `-Dpedsim.trace=<file>` writes a per-leg trace with no timestamps, so `cmp` answers "did this change
  anything" directly. And **re-run one configuration twice before reading anything into it** — a
  mechanism that has just started working is exactly where non-reproducibility surfaces, as region
  navigation did the moment it began producing routes.

- **The server is one git checkout now.** `/mnt/home/gabriele/PedSimCity` builds itself with maven
  and git-lfs, both installed 14 Sep; `PedSimCity-wip` and its hand-shipped classes, jars and 77 runs
  are deleted. `mvn -Pall-modules compile` is green there. Setup and its three traps — `~/.mavenrc`
  pinning JDK 21 over the system's OpenJDK 11, conda missing from non-interactive `PATH`, and maven
  caching a failed lookup until `-U` — are in `CLAUDE.md`. **`pedsim-deps/lib` (55 MB) is the last
  hand-shipped thing left and is now redundant**; its GeoMason jar is the superseded pre-fix build,
  so it is worth deleting rather than leaving to be picked up by mistake.

- **Nothing from 12–14 September 2026 is committed.** Both trees compile; `mvn -Pall-modules compile`
  and `-Pcityimage-empirical` are clean here, and GeoMason-light's 171 tests pass. The CHANGELOG
  follows the natural commit seams.
- **Commit the spotless sweep as its own commit, then add `.git-blame-ignore-revs`.** The tree is
  formatted and `HEAD` is not, so the formatting sits uncommitted across 58 Java files, mixed into
  the working tree. Leaving it there does not avoid the noise — it makes it permanent, because every
  `git status` and `git diff` carries it and the pre-commit hook re-applies it after each checkout.
  Committing it once ends it: `spotless:check` already reports 158 files clean, so no later commit
  has any formatting left to add. Of the 58 files exactly **one** — `TransitVehicle.java`, the
  `countTrip` refactor — also carries a real change, so the split is nearly free. A worktree at
  `E:/tmp/fmt` is already formatted and ready to commit for this; the sequence is in the session
  notes, and **the push is the user's to make, not an agent's**.
- **Set `<ratchetFrom>origin/main</ratchetFrom>` in the spotless config.** `spotless:apply` formats
  the whole tree while the pre-commit hook re-stages only what you staged, so a single new
  non-conforming file drags every other file it touches into your working tree as whitespace churn.
  That is what produced the 58-file spread above. The ratchet confines it to files that actually
  changed. Do it before the next new file lands, or this recurs.
- **Prune the stale worktree.** `git worktree list` shows
  `C:/Users/gfilo/OneDrive - .../pedsimcity-clean` marked `prunable`, left from the
  `rest-module-restructure-integrated` branch. `git worktree prune` once it is confirmed dead.
- **GeoMason-light 2.2.1 is published** (14 Sep 2026): the `hashCode` above, the
  `intersectingFeatures` ordering fix, `Route.computeRouteSequences`'s guard and the idempotent
  `edgeSequence()`, and the new tests. 171 tests pass. **117,397 bytes, sha256 `65849579…`** — record
  it, because a `.m2` holding a different 2.2.1 under the same version string is exactly the hazard
  2.2.0 had, and it happened again the same day: the local copy was the 117,431-byte pre-fix build
  until it was deleted and re-fetched. After any publish, clear the version folder in every `.m2` and
  compare hashes. Anything further is 2.2.2; Central is immutable.
- **Consolidate London's two networks**, or flatten them into two cities. Detail in the cityImage
  file; it is a data decision, not a code one.
