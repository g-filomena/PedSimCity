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

## Darkness reaches the plan, the avoid-set reads light, and pairs depart at night — 16 September 2026

Four changes, and every one of them makes something in this module mean what it already said.
**All four need a run before any of the numbers in this file can be trusted.** The lighting layer
they read was rebuilt on 16 September — see below — so that part is no longer blocking.

- **A known dark edge now costs more to plan through.** `DijkstraRoadDistanceNight` scales a known
  edge's cost from 1.0 at the agent's own sensitivity threshold toward
  `NightPars.maxKnownDarkEdgeCostMultiplier` (1.5, a starting value) at total darkness, so an agent
  can prefer a lit way round *before setting off* instead of only reacting once it is standing on a
  dark street. This is the audit's finding C2, which sat unreviewed in `obsolete/`; it is the only
  change in the module that moves the **plan** rather than the reaction. **Known edges only** — an
  unknown dark street is already handled by the situated gate, and charging for it here as well
  would price the same darkness twice. A night agent's known edges are its simple activity bone, so
  it bites in the home and work regions and nowhere else. Set the parameter to 1.0 to get the old
  planning back, which is the control this change needs.
- **The vulnerable avoid-set reads light, not only knowledge.** It used to be "the whole city minus
  what I know", so an agent frightened by darkness detoured toward *familiarity* and any improvement
  in the light it ended up under was a coincidence. It is now **what is neither lit nor familiar**:
  of the edges outside the community-known network, those that also read as unlit at the agent's own
  drawn threshold, less the streets it knows itself. One rule at two thresholds — the non-vulnerable
  branch already asked exactly this question with the community as its familiar set. **This changes
  the A/B's manipulated arm**, so it supersedes every paired result.
- **`darknessDepth` is one measurement with two readers.** The situated reroute-or-speed-up
  probability and the new planning cost grade darkness identically, from `NightLighting`, rather
  than from two copies of the same four lines.
- **A/B pairs depart into darkness only.** One pair per release event across the whole day meant
  that with `abTestPairs = 72` and a 20-minute cadence, only the ~31 dark events of a Turin June day
  carried a manipulated pair and the other 41 departed into daylight, where the manipulation does
  nothing. The experiment's real size was therefore not `abTestPairs` but whatever fraction of it
  fell after sunset — and it moved with the date with nothing saying so. Light events are now
  skipped, so `abTestPairs` means what it says up to the number of dark events the date allows, and
  `NightTravelDemand` counts those events for the date and **logs the capacity at the start of the
  day**, warning when the experiment is bigger than its own night. On a December date the capacity
  roughly doubles, which is another reason to run this module on a December date.

### What is still not done

- **Nothing is measured.** No run has been made with any of this. The first one to make is the
  control — `maxKnownDarkEdgeCostMultiplier=1.0` against the default, one date, several jobs —
  because planned metres is the quantity these changes are supposed to move and the replicate sd is
  6.5% of it.
- **The lighting layer underneath was rebuilt on 16 September, and everything before it is
  superseded.** `03_street_lights.py` is vectorised (a full Torino run is about ten minutes, not
  hours), the four pipeline fixes have taken effect, `FALLOFF_LAW` is `"isotropic"` — chosen by
  running all three over the same 44,278 edges — and the uncited utilisation factor is now DLOR. The
  last two cancel to within 3% on `mean_lux`, so **`NightPars.darkSpotLuxThreshold` and the drawn
  `lightSensitivityThreshold` still sit where they sat relative to the city** and do not need
  re-tuning. `NO_POLE_HEIGHT_M` remains unsourced but is now bounded: 3 m to 6 m moves the share of
  edges below the service level by half a point. Numbers in `pipeline/README.md`.
- **Only Torino has been re-run.** Any night result on another city is on a layer built under the
  old physics, and the two are not comparable.
- **The directional lookup shipped for Torino was `Torino_simplified`'s** — 29,062 rows, which is
  14,531 x 2, against the full city's 44,278 edges. `directionalEntranceLuxOrNull` returns null on a
  miss and the gate falls back to the binary OSM `lit` tag, so on full Torino about two thirds of
  edge entrances never reached a measured lux value and the rest matched by node-ID collision
  between two unrelated graphs. Regenerating step 4 on 16 September fixed it (88,556 rows).
  **Every full-Torino night run this repo has made predates that.** The general lesson: a derived
  layer keyed on node IDs is silently wrong when it comes from another network, and the row count is
  the cheapest check.

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
the vintage drift this file warns about below, and it matters to this module specifically: the census
vintage no longer reaches darkness at all, since 15 Sep 2026: the city's position is measured from
the street network (`city position from the network (EPSG:3003): 45.0634, 7.6768 degrees`), not read
from the census `centroid_lat` column.

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

## A night run's cost is a function of how dark the period is

Measured 16 September 2026, on the first runs this module has made away from 1 June: four weeks on
full Torino, 3,386 agents, three replicates each, each week containing its own solstice or equinox.

| week | legs departing in darkness | simulated days per hour |
|---|---|---|
| 15 June (solstice) | ~75 / 1,500 — 5% | 3.3 |
| 16 March (equinox) | ~180 / 1,500 — 12% | 0.9 |
| 21 September (equinox) | ~188 / 1,650 — 11% | 0.9 |
| 21 December (solstice) | ~400 / 1,490 — 27% | 0.7 |

**A December week costs about five times a June week**, and the ordering follows the dark share
exactly. `jstack` says where it goes and it is not a defect: `NightAgent.step` ->
`NightAgentMovement.keepWalking` -> `transitionToNextEdge` -> `setupEdge` -> `checkLightLevel` ->
`whenLitVulnerable` -> `computeAlternativeRoute` -> `Astar.astarRoute`. Every entry onto a dark edge
runs the lighting gate, and a darkness-graded fraction of those buys a fresh A* bypass. There is no
bypass cache, deliberately — a key wide enough to be correct carries the destination and the current
edge, so it would almost never hit.

**Budget by the date, not by the agent count.** A seven-day December run at 3,386 agents with three
replicates is roughly a day and a half of wall time; the same run in June is six hours. This was
invisible while every run was 1 June, the brightest week of the year and the cheapest.

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
- **How many A/B pairs — settled 16 Sep 2026 in the release mechanism rather than in the
  arithmetic.** Pairs used to be released one per release event across the whole day, so with a day
  of 72 events (`releaseAgentsEveryMinutes = 20`) and roughly 31 of them dark, more than half of
  `abTestPairs` departed into daylight where the manipulation does nothing. Light events are now
  skipped, so a pair always departs after dark and `abTestPairs` is the experiment's size — capped
  by the dark events the date allows, which is counted and logged at the start of the day. Two
  things still hold: `abTestPairs` above that capacity leaves the extra pairs at home, and A/B
  releases run on **day 1 only** (`releaseAgentsOverride` returns −1 for any other day).
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
legs now fall inside the `[20:00, 06:00)` aggregation window in winter, which is what threat 7 below
was asking about; and the daylight-only assumption no longer has to appear in a write-up.

**2. Relative comparisons only — decided 14 Sep 2026.** The A/B design reports
vulnerable-vs-non-vulnerable and scenario-vs-scenario differences, not absolute exposure.
`ActivityPars.distanceWeight` is uncalibrated and *is* the discretionary trip-length distribution,
and trip length sets how much lit and unlit street an agent crosses, so "agents cross X metres of
unlit street per night" inherits that coefficient whole while a difference between two arms largely
cancels it. `choiceSetRadiusMetres` stays at 3,000 m, truncating the mean leg by a measured ~4%,
which is reported with any trip-length figure rather than tuned away. State both in the write-up.

**3. Darkness now reaches planning, for known edges only.** Until 16 Sep 2026 no lighting rule in
this module touched route choice: agents reacted to a dark street they were standing on and planned
as though light did not exist. `DijkstraRoadDistanceNight.lightingCostMultiplier` changes that for
the edges an agent knows, which for a night agent is its home and work regions. State which value of
`NightPars.maxKnownDarkEdgeCostMultiplier` a run used; 1.0 is the old behaviour and is the control.

**4. Barrier preferences are not part of night route choice, and that is now a decision — 16 Sep
2026.** `roadDistanceNight` reaches the three-argument `dijkstraAlgorithm`, which does not call
`initialisePrimal`, so no region subgraph is built and `directedEdgesToAvoid` is not consulted; that
half was already deliberate. The other half — "night replaces `costPerceptionError` with a plain
draw" — was never a removal. `Heuristics` builds every core, activity, night and learning model with
`BarrierPreferences.NONE`, so the parent's barrier branch is unreachable for a night agent and
`costPerceptionError` already returns the plain perception error. Barrier perception is survey-derived
route-choice data and belongs to the two modules that hold it.

What the hand-written draw *did* do was hardcode a 0.10 sigma, so `--perceptionErrorSD=0` did not
pin the night router. It calls `costPerceptionError` now. **Any night A/B run before 16 Sep 2026
carried perception noise that a pinned run was meant to have removed** — and a paired experiment is
exactly where that noise does the most damage. Either way, night routing is not day routing plus
lighting.

**5. Night agents are not individualised.** `NightAgent.step` builds
`CognitiveMap.buildSimpleActivityBone()`, so `individualised` stays false: their known edges are a
*preference* signal — what `NightBehaviour` scores, what a vulnerable agent avoids — and not a
statement about what is reachable. The full-network escalation therefore never fires for them.
Anything that flips that flag for a simple-bone agent breaks destination choice too.

**6. A/B vulnerability proportions are not population shares.** With `enableLightABTesting` the twins
are spawned by construction, not sampled from the census `vulnerability_pct`. The run logs this
itself; do not read the outputs as a population.

**7. ~~Unmeasured: whether structural commutes disturb night statistics.~~ Measured 14 Sep 2026 —
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

- **The vulnerable avoid-set is now lighting-aware, and that is the manipulated variable.** "Avoid
  what is neither lit nor familiar" is what makes a vulnerable agent's detour a detour *toward
  light*. Reverting it to the knowledge-only set turns the A/B back into a comparison of how much
  unfamiliar ground two agents cover.
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

## Cross-machine reproducibility is CLOSED — 14 September 2026

The long section that stood here, tracking a ~1% disagreement between `gdsl1` and the laptop on one
seed, is gone because the disagreement is gone. A seed replays byte-for-byte on both machines: core,
night and activity give identical per-leg traces on seed 20260912. It needed two things and neither
was sufficient alone — core's release draw and agent scheduling ordered deterministically, *and*
GeoMason-light 2.2.1's `hashCode` on `NodeGraph` and `EdgeGraph`. The mechanism is in `CLAUDE.md`;
the general rule it left behind is in `../core/TODO.md` under the invariants.

**What survives from it, because it is still true:** every figure this module measured before
14 September 2026 is superseded, since the workplace draw moved when the attraction maps stopped
iterating in identity-hash order. The machine still belongs in a write-up beside the seed and the
jar — not because they disagree now, but because that is how you would notice if they ever did
again.


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
