# Night module — before you run experiments on Turin

Written 13 September 2026, after the first clean full-Torino run of the post-refactor code.
Repo-wide state is in `/TODO.md`; this is what bears on the night module specifically.

---

## Park and waterside avoidance was reading empty sets — fixed 15 September 2026

`SharedCognitiveMap.edgesWithinParks` and `edgesAlongWater` had no writer anywhere in the tree, so
every mechanism in this module that consults them did nothing:

| site | what it does |
|---|---|
| `NightAgent` (two branches) | refuse a park or waterside destination after dark |
| `NightAgentMovement` | vulnerable agents avoid those edges while routing |
| `NightBehaviour` | non-vulnerable agents prefer them |
| `DijkstraRoadDistanceNight` | the cost term behind that preference |

They are derived now from the per-edge `parks` and `waterBodies` attributes that
`BarrierIntegration.setEdgeGraphBarriers` already writes, so there is one copy of the fact rather
than two. Torino carries 2,500 park edges and 647 waterside ones.

**Every night figure this module has produced predates the mechanism running.** A 423-agent
Torino_simplified day moves from 205 legs / 313,334 m planned / 332,136 m walked to 204 / 311,671 /
327,246 — a small aggregate shift, but the A/B experiment's manipulated variable is exactly this
avoidance, so the comparison is what has to be redone, not the total.

The distinction the two gates encode is unchanged and still deliberate: refusing to *spend an
evening* in an unlit park applies to every night agent, avoiding those edges while *walking past*
applies to vulnerable agents only.

## Where it stands

Verified on 13 Sep. **The commute figures in this table are superseded** — the workplace draw changed
on 14 Sep when the attraction maps stopped iterating in identity-hash order, and a Torino day now
reports `workers 16.8% (ISTAT 16.3%), students 38.3% (ISTAT 38.0%)`. See the cross-machine section
below.

| | |
|---|---|
| Full Torino, 4,232 agents (0.5%), one day | clean: 0 unusable route lengths, 0 destination fallbacks, 0 band widenings, 0 angular fallbacks, 0 full-network escalations, 0 known networks left in pieces |
| walked commute share | workers 16.4% (ISTAT 16.3%), students 36.9% (38.0%) |
| walked commute length bands | 78.2 / 18.5 / 3.1 / 0.2 against ISTAT 76.2 / 19.0 / 3.4 / 1.3 |
| metres per agent per day | 662 m, inside the 600–1000 m band |
| mean walked leg | 1,364 m |
| circuity, measured | 1.292 (Torino), 1.331 (Torino_simplified) |
| A/B path | 40 pairs → 93 paired trips; twins share origin *and* destination, routes differ |

Full write-up of that run: `analysis/validation/Torino/night_torino_2026-09-13.md`.

---

## Verified working on `gdsl1`, 14 September 2026

The server tree is current again: `classes_new` under `/mnt/home/gabriele/PedSimCity-wip` carries the
12-14 Sep code and the 12 Sep `Torino_censusData.gpkg`. **The census there had been July's**, which is
the vintage drift this file warns about below, and it matters to this module specifically: the current
one carries `centroid_lat`, so the run logs `city latitude from census: 45.0691 degrees` instead of
falling back to `ActivityPars.latitudeDegrees = 53.4` - Liverpool's - which sets every sunset in a
model about darkness.

- one day, 4,232 agents (0.5%): **1 m 25 s**, clean ledger;
- the A/B path: 72 pairs spawned, 158 paired trips written to `ab_test_comparison.csv`, twins sharing
  `start_node` and `dest_node` with routes that differ and the vulnerable twin detouring (1311.6 m
  against 1140.9 m on the first pair);
- runs live under `/mnt/home/gabriele/PedSimCity-wip/runs/`.

```bash
ssh -i id_ed25519 gabriele@gdsl1.liv.ac.uk
cd /mnt/home/gabriele/PedSimCity-wip && export PATH=/usr/local/software/java/jdk-21.0.6/bin:$PATH
java -Xmx32g -cp "classes_new:lib/*" pedsim.night.launcher.NightLauncher   --headless --cityName=Torino --percentage=0.005   --days=1 --jobs=1 --stepDelayMs=0 --exportHtmlDashboard=false
```

`ssh.key` in `server.properties` still points at a `C:` path that no longer exists; pass
`-i id_ed25519` from the repo root, as above.

## Pick the date. It has always been 1 June

`TimePars.SIMULATION_START_DATE` was only reachable by editing the class and rebuilding, so every run
this repo has ever produced is **1 June 2026 — the shortest night of the year**, in a module about
darkness. It takes a value on the command line now: `--SIMULATION_START_DATE=2026-12-07`. Seasonal
daylight, the day of week, persona attendance and the whole darkness exposure above all follow from
it. A lighting study that reports a June day is reporting the easiest case there is.

## Do these before any run you intend to keep

1. **Commit.** Around a hundred files are uncommitted, and they include release, destination choice,
   routing and the A/B itself. A result that cannot be tied to a revision is not a result.
2. **Settle GeoMason-light 2.2.0.** Two different builds currently share that version number — 117,232
   bytes locally, 115,882 on `gdsl1` before it was replaced. Nothing detects the difference, and it
   surfaced as a `NullPointerException` on a null `nodeID` deep in `Environment.prepareGraph`. Bump
   and publish, or record the jar's checksum beside every result.
3. **If running on `gdsl1`, ship the current `Torino_censusData.gpkg`.** The 13 Sep run used an older
   copy, so its persona mix was not the current one. Also check `Torino_edges_illuminated_continuous.gpkg`:
   resource vintages there drift silently and only a size comparison catches it.
4. **Rebuild after touching anything under `src/main/resources/`.** `mvn compile` re-copies; running
   `java` against an unrebuilt `target/classes` silently uses the old data.

---

## Decide these before running, not after

- **Replicates — now measured, and bigger than you would guess.** Runs are deterministic from
  `Pars.seed`, so one run per condition still gives no variance estimate; job *n* uses `seed + n`, so
  `--jobs=N` gives N replicates from one base seed, and `--seed=-1` gives a clock seed for
  independent bases. The end of a run prints one line per replicate — legs, planned and walked
  metres, agents — and the mean and sd across them. First
  measurement, two jobs on `Torino_simplified` at 169 agents: **sd 6.5% of planned metres, 12.9% of
  legs**. Any effect smaller than that needs more replicates before it is an effect — and note it is
  several times the ~1% cross-machine disagreement, which puts that in proportion.
- **How many A/B pairs, and when they depart — decided: size the experiment on the pairs that
  actually depart at night, and leave the release mechanism as it is.** Pairs are released **one per
  release event**, and a day holds 72 events (`releaseAgentsEveryMinutes = 20`). Two consequences:
  - `abTestPairs > 72` leaves the extra pairs at home all day;
  - the night window is `[20:00, 06:00)`, which is about **31 of those 72 events**, so with
    `abTestPairs = 72` roughly 43% of pairs depart in darkness and the rest depart into daylight,
    where the lighting manipulation does nothing. Size the experiment on the pairs that actually
    depart at night, not on `abTestPairs`.
  - A/B releases run on **day 1 only** (`releaseAgentsOverride` returns −1 for any other day).
- **Sample size against the effect you want to detect.** 0.5% of Torino is 4,232 agents and ~2,054
  trips a day. How many of those cross unlit edges decides whether a lighting difference is
  measurable at all.
- **`--exportHtmlDashboard=false`** for anything large: the dashboard embeds every trip path (~29 MB
  at 5.8k trips). The CSV and GeoPackage exports are unaffected.

---

## Threats to validity — state these in any write-up

**1. ~~Nobody commutes in the dark.~~ Settled 14 Sep 2026 — the guard is gone.** `CommuterAgent`
and `ActivityAgent.shouldGoToWork` no longer consult `isDark()`, and `planMandatoryDeparture` no
longer refuses to draw a departure into darkness. The persona's start window says when somebody sets
off; the season says whether it is light when they do. Turin's sunset is before 17:00 through
December, so the guard had been deleting the winter commute — the most routine walking there is,
by the population most exposed to unlit streets. **Two consequences for this module:** mandatory
legs now fall inside the `[20:00, 06:00)` aggregation window in winter, which is what threat 6 below
was asking about; and the daylight-only assumption no longer has to appear in a write-up.

**2. Relative comparisons only — decided 14 Sep 2026.** The A/B design reports
vulnerable-vs-non-vulnerable and scenario-vs-scenario differences, not absolute exposure.
`ActivityPars.distanceWeight` is uncalibrated and *is* the discretionary trip-length distribution,
and trip length sets how much lit and unlit street an agent crosses, so "agents cross X metres of
unlit street per night" inherits that coefficient whole while a difference between two arms largely
cancels it. `choiceSetRadiusMetres` stays at 3,000 m, truncating the mean leg by a measured ~4%,
which is reported with any trip-length figure rather than tuned away. State both in the write-up.

**3. Barrier preferences are absent after dark.** `roadDistanceNight` reaches the three-argument
`dijkstraAlgorithm`, which does not call `initialisePrimal`, so no region subgraph is built and
`directedEdgesToAvoid` is not consulted. Night also replaces `costPerceptionError` with a plain draw.
The first is a decision; the second is undecided rather than intended. Either way, night routing is
not day routing plus lighting.

**4. Night agents are not individualised.** `NightAgent.step` builds
`CognitiveMap.buildSimpleActivityBone()`, so `individualised` stays false: their known edges are a
*preference* signal — what `NightBehaviour` scores, what a vulnerable agent avoids — and not a
statement about what is reachable. The full-network escalation therefore never fires for them.
Anything that flips that flag for a simple-bone agent breaks destination choice too.

**5. A/B vulnerability proportions are not population shares.** With `enableLightABTesting` the twins
are spawned by construction, not sampled from the census `vulnerability_pct`. The run logs this
itself; do not read the outputs as a population.

**6. ~~Unmeasured: whether structural commutes disturb night statistics.~~ Measured 14 Sep 2026 —
and the answer is the other way round.** Commutes do not pollute the night window; the night window
misses the dark. Mandatory legs run 06:30-19:30 (worker start window 6.5-10.5, a 6-9 h stay), so
they never fall inside `[20:00, 06:00)`. What they do now fall inside, with the darkness guard gone,
is *behavioural* darkness — and in winter that is most of them. The day's ledger line reports it, on
`Torino_simplified` at 338 agents:

| date | legs begun in darkness | of those, outside the fixed night window |
|---|---|---|
| 1 June | 29 / 166 (17%) | **0** |
| 7 December | 90 / 172 (52%) | **63** — 37% of the day's legs |

In June the seasonal and fixed definitions agree exactly, which is why nothing ever showed. In
December the exporters report more than a third of the day's walking as daytime volume when the
agents walked it in the dark and behaved accordingly — lighting-aware routing, park and water
refusal, vulnerable detours all key off `isDark`, not off the window.

**Decided: night means dark for that date.** The volume exports aggregate on the seasonal
sunrise/sunset of the day being exported, and the columns are `LIGHT` / `DARK` rather than
`DAY` / `NIGHT`. Someone walking at a time that is dark for that season is counted as walking in the
dark, whether or not a June evening at the same clock hour would have been. Two dates are therefore
not comparable hour-for-hour on those two columns — the per-hour columns are still there for anyone
who wants a fixed window — and the same day on `Torino_simplified` reports **8% of volume in the dark
on 1 June and 56% on 7 December**.

---

## Do not change these casually — they define what the A/B measures

- **Park and water avoidance has one rule and two gates, deliberately.** Refusing a park or waterside
  *destination* after dark applies to **every** night agent; avoiding those edges while *routing*
  applies to **vulnerable agents only**. Spending an evening in an unlit park and walking past one
  are different decisions. Changing either changes the A/B's one manipulated variable.
- **There is no bypass cache, and a `(routeOrigin, reentryNode)` key cannot bring one back.** The
  avoid-set is built per agent from its known edges, the edge it is fleeing and its destination, so
  that key hands agents each other's routes — across the vulnerable/non-vulnerable split included. A
  correct key carries the destination and the current edge, which vary per trip, so it would almost
  never hit. Measure the hit rate of a correct key before reintroducing one.
- ~~**Open question:** whether a fleeing agent should avoid the edge it is fleeing.~~ **Settled
  14 Sep 2026: it should, and now does.** The current edge used to be added in the non-vulnerable
  branch only. A vulnerable agent's avoid-set is "the whole city minus what I know", and the
  problematic edge is normally a street it or the community knows, so it was subtracted straight back
  out — leaving A* free to return a "bypass" down the very edge being fled. The one population this
  module is about was the one that could not get away. `canReroute()` already guarantees the edge is
  not incident to the destination, so the destination-edge exemption cannot reinstate it.
  **This changes vulnerable-agent results**, which is the A/B's manipulated arm.

---

## A seed does not fully reproduce a run across machines — partly fixed 14 Sep 2026

Established by running the identical command on both machines. **Each machine replays itself
exactly**; the two did not agree with each other.

| | mandatory legs | workers walking | trips | planned metres |
|---|---|---|---|---|
| before, `gdsl1` (×3 runs) | 1163 | 14.9% | 2064 | 2,746,498 |
| before, Windows laptop (×2 runs) | 1143 | 15.0% | 2049 | 2,663,226 |
| **after the fix, both machines** | **1254** | **16.8%** | 2036 / 2058 | 2,649,251 / 2,733,620 |

Same seed (20260912), same code, same resources, same `GeoMason-light-2.2.0.jar` at 117,232 bytes.

**The cause is identity hash codes, not floating point.** A probe running `Math.exp`, `log`, `pow`,
`sin` and `sqrt` over 200,000 inputs produced bit-identical results on both machines, so the
transcendental functions are not responsible. What is: `NodeGraph` and `EdgeGraph` override neither
`hashCode` nor `equals`, so any `HashMap` or `HashSet` keyed on them iterates in identity-hash order —
and HotSpot generates identity hashes from a per-JVM generator whose values differ between JVM
builds. Stable within a machine, different across them, exactly as observed.

`PoiClassifier` built the per-purpose attraction maps as `HashMap<NodeGraph, Double>`, and
`WorkplaceChoice.draw` walks those entries into a cumulative distribution and then picks by
position — so the same random number selected a different workplace on each machine, which moved the
commute distance, which moved `walksToWork`. Those maps are now `LinkedHashMap`, and **the whole
population layer is now identical across the two machines**: mandatory legs, walked commute share for
both workers and students, and the length bands. The same change was made to the known-network edge
sets in `NetworkBuilder` and `CognitiveMap.deriveOtherKnownRegions`, which feed
`Islands.mergeConnectedIslands`; that matters to the activity and learning tiers rather than to night,
whose agents build a simple bone and never reach it.

**What is still open.** Trips and metres still differ by about 1% (2036 against 2058). The remaining
path is downstream of population, in destination choice or routing, and has not been found. Candidate
shape: another object-keyed collection whose iteration order reaches a draw. `DestinationChoice`
itself indexes lists and is clean.

Swept since, with the night path specifically in mind (the full list, and the proposed one-line fix
in GeoMason-light, are in `../core/TODO.md` item 5):

- `PedSimCityNight.nodesVulnerabilityWeight` is a `HashMap<NodeGraph, Double>` and looks like exactly
  the defect that was just fixed, but it is only ever `getOrDefault`-ed per home node and never
  iterated. Clean — **do not "fix" it and expect the 1% to move.**
- `Agent.defineRandomDestination`, `selectWeightedDestination` and `Dijkstra`'s known/visited sets are
  clean for the same kind of reason: filtering without reordering, indexing by position, membership
  and lookup only.
- **`Islands.mergeConnectedIslands` is genuinely order-dependent and does not reach this module.** The
  library re-wraps the caller's carefully ordered edge set into a `HashSet`, so which bridge joins two
  islands is machine-dependent — but night agents build a simple bone and never call `Islands`. It
  bears on the activity and learning tiers instead.
- The one open candidate that *does* reach night is `NodeGraph.getDualNodes`, which builds a
  `HashMap` and sorts it stably, so dual nodes at exactly equal cost come back in identity-hash order.
  Ties only, and only on angular routes.

**What follows for an experiment, until that is closed:** every run of a comparison has to come from
one machine, and the machine belongs in the write-up beside the seed and the jar. A condition run on
the laptop against a control run on `gdsl1` differs by roughly the size of the effects this module
looks for.

Note that the fix **changes results** on both machines — the workplace draw now selects differently.
Any figure in this file measured before 14 Sep 2026 is superseded; the current Torino day reports
`workers 16.8% (ISTAT 16.3%), students 38.3% (ISTAT 38.0%)`.

## Comparisons that are already void

- **Any run-output comparison from before 12 Sep 2026.** `Engine(StateFactory)` seeded from the clock,
  so no two headless runs shared a seed.
- **Any A/B run with `useDestinationChoice=false` between 12 and 13 Sep 2026.** Nothing wrote
  `distanceNextDestination` in that window, so the "legacy" destination path searched around zero and
  every agent walked to one of the thirty nodes nearest home.
- **Any A/B run from before 13 Sep 2026.** The twins no longer share a drawn trip length — they share
  a destination, which is the stronger control — and an A/B twin now always chooses by utility
  regardless of `useDestinationChoice`.

---

## Validation data, and what each can and cannot say

- **Turin, Vodafone Smart Locator** (`analysis/validation/Torino/`, acquisition in `../vodafoneAPI/`).
  The metric is `visitors`: unique SIM presence, all modes, indoors included. It validates **shape** —
  diurnal, spatial, compositional — and never absolute pedestrian volume. Restricting to a 0–30 min
  dwell removes residents and workers but **cannot separate mode**: a car crosses a 216 m cell in
  ~26 s against a pedestrian's ~2.8 min, and the dwell dimension has three buckets.
- **`DepartureProfile` has never been compared against that diurnal series.** It is a prediction, not
  a fit, which makes the comparison a single pre-registered step and the most valuable cheap check
  available.
- **Melbourne pedestrian counts** measure the same quantity the model produces, but against the
  city-image module, which has no clock — spatial pattern only.

---

## Reference invocation

```bash
java -Xmx64g -cp "classes:lib/*" pedsim.night.launcher.NightLauncher \
  --headless --cityName=Torino --percentage=0.005 \
  --days=1 --jobs=1 --stepDelayMs=0 --exportHtmlDashboard=false

# add --enableLightABTesting=true --abTestPairs=40 for the paired experiment
```

A day at 4,232 agents takes about a minute on `gdsl1`. A 7-day, 8,465-agent (1%) run on full Torino
completed in 2 h 31 m.
