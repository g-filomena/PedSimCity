# Empirical validation, 15 Sep

**Update, 18 Sep:** see [Empirical validation, 18 Sep](#empirical-validation-18-sep--isolating-the-c1-join-fix)
at the end of this file. `main` has since independently absorbed C1's gate logic, C2, C3 and C5
(and fixed an unrelated reroute-loop bug that inflated detour numbers on both sides of the run
below) — so the comparison below no longer reflects current upstream behaviour. The 18-Sep run
isolates the one thing that's still actually unmerged: the C1 `min_lux` join.

The six fixes on this branch (C1, C5, C6, B1, C2, C3) were shadow-compiled
clean against the real classpath while being written, but shadow-compiling
only proves the code builds — it says nothing about what it actually does at
runtime. This is that check: both sides actually built and run, headless, on
real Torino data.

## Method

Two isolated `git worktree`s off `main` @ `08063cf` (both clean, no
commits):

- **baseline** — untouched.
- **fixed** — the five proposed-fix files from this folder copied onto
  their real source paths (`src/main/java/pedsim/night/...`), uncommitted,
  local only. Never pushed; exists only to run.

Both compiled with `mvn -o compile` (a real compile, not the shadow-compile
classpath trick used earlier) and run with `mvn exec:java@night`:

```
--cityName=Torino --durationDays=7
```

Same fixed seed (`20260912`, the project default) and the same 846-agent
draw (0.1% of Torino's census population, also the default) on both sides,
so daytime behaviour, population, and commute stats came out byte-identical
between the two runs — the only variable was the code. Neither run produced
any warning beyond the expected `transit_stops.csv not found` (consistent
with C6), and both exited 0.

An earlier 1-day pass (`--durationDays=1`) is not reported here — the
night-window sample (33-39 legs) was too small to say anything with; the
7-day numbers below (365-375 night legs) are the ones worth reading.

## Results

| | Baseline | Fixed |
|---|---|---|
| Total legs, 7 days | 2,847 | 2,840 |
| Planned → walked distance | 3.88M → 4.46M m (**+15.0%**) | 3.78M → 4.44M m (**+17.5%**) |
| Night-window legs (before 6am / after 8pm) | 375 | 365 |
| Night avg `mean_lux`, all agents | 22.28 | 21.63 |
| Night avg `mean_lux`, **non-vulnerable** | 19.01 | **20.66** (+8.7%) |
| Night avg `mean_lux`, **vulnerable** | 23.44 | **21.88** (−6.7%) |
| Night legs under 10 lux | 13 (3.5%) | 10 (2.7%) |

`mean_lux` is the per-trip metric from `getTripMeanLux()` (length-weighted,
register finding D1 — already resolved upstream). Night-window = legs
starting before 06:00 or at/after 20:00.

## Reading it

**The detour overhead rising (15.0% → 17.5%) is the expected fingerprint of
the fixes actually engaging** — C1's stricter gate, C2's lux-aware route
cost, and C5's darkness-scaled reroute probability all push agents toward
more active avoidance than before. Not noise: it shows up consistently.

**Non-vulnerable agents got measurably better-lit routes** — the direct,
intended effect of C3: their avoid-set is lux-based now instead of the raw
OSM `lit` tag, and the number backs it up (+8.7% average night `mean_lux`).

**Vulnerable agents got measurably *worse* — and this is the finding worth
Gabriele's attention, not a bug in what's here.** C1 makes the lit/non-lit
gate stricter (an edge bright on average but dark in the middle now
correctly fails), so *more* edges register as dark to a vulnerable agent
than before the fix. But the vulnerable-agent reroute avoid-set
(`edgesOutsideCommunityKnown()` minus the agent's own known edges, in
`NightAgentMovement.defineEdgesToAvoid()`) has **nothing to do with
lighting** — it's built purely from community-vs-personal knowledge. That
gap is not something this branch introduced or touched: C3's own scoping
note says explicitly *"whether knowledge-based avoidance is the intended
design... belongs with the code owner, not this register."* This run turns
that from a theoretical gap into a measured one: C1 now correctly flags more
genuinely-patchy streets as dark, more vulnerable agents get triggered into
rerouting more often, but the mechanism they reroute *through* doesn't
target brightness at all — so on average they don't end up on better-lit
streets, and here they measurably ended up on slightly worse ones.

That's real, run-derived evidence for making the vulnerable-agent avoid-set
question an actual decision rather than an open one.

## Caveats

One seed, one city, 7 days, 846 agents. This is directional evidence that
the fixes behave as designed and that the numbers move in the direction the
Javadoc claims, not a calibration-grade result — no multi-seed replication,
no significance testing, no comparison against the Torino Vodafone
validation data. Worth more days and/or multiple seeds before leaning on the
magnitudes specifically; the *direction* of each effect (more detour
overhead, non-vulnerable better lit, vulnerable's avoid-set gap surfacing) is
what this run actually supports.

## Reproducing

```bash
# from a clean checkout of main:
git worktree add ../PedSimCity-baseline main
git worktree add --detach ../PedSimCity-fixed main

# overlay this folder's proposed files onto their real paths in -fixed:
for f in agents/NightBehaviour.java agents/NightAgent.java \
         agents/NightAgentMovement.java parameters/NightPars.java \
         routing/pathfinding/DijkstraRoadDistanceNight.java; do
  cp "night-fixes-eval/src/main/java/pedsim/night/$f" \
     "../PedSimCity-fixed/src/main/java/pedsim/night/$f"
done

cd ../PedSimCity-baseline && mvn -o compile && \
  mvn -o exec:java@night -Dexec.args="--cityName=Torino --durationDays=7"
cd ../PedSimCity-fixed   && mvn -o compile && \
  mvn -o exec:java@night -Dexec.args="--cityName=Torino --durationDays=7"

# trip-level lux lands in outputs/trip_diagnostic.csv on each side
# (agent_id,start_time,end_time,duration_min,distance_m,nodes_walked,edges_walked,vulnerable,mean_lux)
```

Both worktrees are local-only (the `-fixed` one has five uncommitted file
overlays by design) — nothing here was pushed.

---

# Empirical validation, 18 Sep — isolating the C1 join fix

## Why a new run, not a repeat of the one above

Between 15 and 18 Sep, `main` independently absorbed C1's gate logic (`NightLighting.isLit()`),
C2 (`DijkstraRoadDistanceNight.lightingCostMultiplier()`), C3
(`NightAgentMovement.defineEdgesToAvoid()`, lux-based on *both* the vulnerable and non-vulnerable
branches — going further than this branch's own C3 proposed) and C5
(`NightBehaviour.rerouteOrIncreaseSpeed()`), plus fixed a real reroute-loop bug where vulnerable
agents backtracked and re-walked their own routes (3.9% of trips carrying ~60% of all walked
metres in the affected runs). Overlaying this branch's original five proposal files — written
against the pre-`main`-reimplementation codebase — onto current `main` would silently regress all
of that. See the README's Status table for the finding-by-finding detail.

The one thing not independently fixed upstream: `NightEnvironment.joinIlluminatedEdges()` never
attached `min_lux` to graph edges, only `mean_lux`, so every reader of `min_lux` (`NightLighting.isLit()`
included) fell through its `minLuxAttr == null` fallback on every edge, unconditionally. This run
isolates that one fix.

## Method

Two worktrees off `main` @ `5b9dea1`:

- **baseline** — untouched.
- **fixed** — `night-fixes-eval/src/main/java/pedsim/night/engine/NightEnvironment.java` copied
  onto its real path, nothing else.

Both compiled with `mvn compile` (online — this environment's `.m2` cache had neither
`GeoMason-light:2.2.2`, which `main`'s reroute rewrite now needs, nor the test-scope JUnit
dependencies `exec:java` also resolves; `-o` fails on both without a prior online compile) and run
with `mvn exec:java@night -Dexec.args="--cityName=Torino --durationDays=7"`. Same fixed seed
(`20260912`) and the same 846-agent draw as the 15-Sep run.

## Results

| | Baseline (`main`, unmodified) | Fixed (`main` + C1 join) |
|---|---|---|
| Total legs, 7 days | 2,847 | 2,838 |
| Planned → walked distance | 3,803,328 → 3,935,169 m (**+3.5%**) | 3,779,856 → 3,897,531 m (**+3.1%**) |
| Night-window legs | 419 | 415 |
| Night avg `mean_lux`, all agents | 22.05 | **23.85** (+8.2%) |
| Night avg `mean_lux`, non-vulnerable | 21.01 | **22.88** (+8.9%) |
| Night avg `mean_lux`, vulnerable | 22.46 | **24.27** (+8.1%) |
| Night legs under 10 lux | 8 (1.9%) | 7 (1.7%) |

## Reading it

**Detour overhead is far lower on both sides than the 15-Sep run's (15.0% / 17.5%).** Consistent
with `main`'s reroute-loop fix (landed 17 Sep, unrelated to anything on this branch) actually
working network-wide — not an artefact of this particular comparison.

**The join fix lifts realized night-lux exposure for *both* populations this time (+8-9% each),
not just non-vulnerable like the 15-Sep run found.** That run's headline finding was that C3 helped
non-vulnerable agents but left vulnerable agents worse off, because the vulnerable-agent avoid-set
wasn't lighting-based at all back then. `main` has since closed that gap independently (C3, beyond
what this branch proposed). With both avoid-sets now lighting-based, correctly gating out
patchy-but-bright-on-average edges via the join benefits whichever population walks them —
vulnerable and non-vulnerable alike.

**Total legs differ by 9 (2,847 vs 2,838, ~0.3%) despite an identical seed and population draw.**
Not a daytime regression — both logs show identical walked-commute-share and commute-length
figures against the ISTAT targets on every one of the 7 days. The join only touches night-specific
lighting attributes, so a handful of night reroute/bypass decisions landing differently, cascading
into a different number of completed legs for a few agents, is the plausible mechanism.

## Caveats

Same as the 15-Sep run: one seed, one city, 7 days, 846 agents — directional evidence, not a
calibration-grade or significance-tested result. The effect size here is smaller than the 15-Sep
bundle, which is expected (one join fix, not six behavioural changes stacked together), but it's
consistent in direction across both vulnerable and non-vulnerable populations, which the 15-Sep
run's finding was not.

## Reproducing

```bash
# from a clean checkout of main:
git worktree add ../PedSimCity-baseline main
git worktree add --detach ../PedSimCity-fixed main

cp night-fixes-eval/src/main/java/pedsim/night/engine/NightEnvironment.java \
   ../PedSimCity-fixed/src/main/java/pedsim/night/engine/NightEnvironment.java

cd ../PedSimCity-baseline && mvn compile && \
  mvn exec:java@night -Dexec.args="--cityName=Torino --durationDays=7"
cd ../PedSimCity-fixed   && mvn compile && \
  mvn exec:java@night -Dexec.args="--cityName=Torino --durationDays=7"

# trip-level lux lands in outputs/trip_diagnostic.csv on each side, same columns as the 15-Sep run
```

`mvn compile` (online, not `-o`) only needs to run once per worktree to populate the local `.m2`
cache with `GeoMason-light:2.2.2` and the test-scope JUnit dependencies; nothing else needs
network access, and neither worktree's changes were pushed.
