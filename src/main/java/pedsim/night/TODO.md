# Night module — before you run experiments on Turin

Written 13 September 2026, after the first clean full-Torino run of the post-refactor code.
Repo-wide state is in `/TODO.md`; this is what bears on the night module specifically.

---

## Where it stands

Verified on 13 Sep:

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

- **Replicates.** Runs are deterministic from `Pars.seed`, so one run per condition gives you no
  variance estimate at all. Job *n* uses `seed + n`, so `--jobs=N` gives N replicates from one base
  seed; `--seed=-1` gives a clock seed if you want independent bases.
- **How many A/B pairs, and when they depart.** Pairs are released **one per release event**, and a
  day holds 72 events (`releaseAgentsEveryMinutes = 20`). Two consequences:
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

**1. Nobody commutes in the dark.** `CommuterAgent.shouldGoToWork` carries an `!isDark()` guard, so
mandatory travel happens in daylight only. Turin's sunset is before 17:00 through December and people
walk home from work in the dark all winter — exactly the population most exposed to dark streets, on
a routine, non-discretionary trip. The guard is unsourced, and `planMandatoryDeparture` was made to
agree with it so the leg budget is not mischarged; agreeing with a rule is not the rule being right.
**For a lighting study this is the single most load-bearing unsourced assumption in the module.**

**2. Prefer relative comparisons to absolute ones.** `ActivityPars.distanceWeight` is uncalibrated and
*is* the discretionary trip-length distribution; trip length sets how much lit and unlit street an
agent crosses. Vulnerable-vs-non-vulnerable, or scenario-vs-scenario, are robust to that.
"Agents cross X metres of unlit street per night" inherits it directly. `choiceSetRadiusMetres = 3000`
additionally truncates the mean leg by a measured ~4%.

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

**6. Unmeasured: whether structural commutes disturb night statistics.** The module generates
mandatory commutes, in daylight only, and the exporters aggregate night on the fixed
`[20:00, 06:00)` window, so night figures *should* be unaffected. That is an expectation, not a
measurement. One before/after on a night run settles it, and it is cheap.

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
- **Open question:** for a vulnerable agent the avoid-set is the whole city minus what it knows, so
  the problematic edge it is fleeing is itself avoided only when it falls outside both the
  community-known and agent-known networks. Explicit in `NightAgentMovement.defineEdgesToAvoid`;
  whether a fleeing agent should avoid the edge it is fleeing is undecided.

---

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
