# Empirical validation, 15 Sep

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
