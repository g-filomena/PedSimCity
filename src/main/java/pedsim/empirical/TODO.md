# Empirical module — what is left to do

Split out of `/TODO.md` on 13 September 2026. This module walks a shared origin-destination matrix
with one agent group per survey-derived cluster (`Muenster_clusters.csv`), so the groups differ in the
route-choice preferences the clusters describe.

> **Output from before 13 September 2026 is wrong**, for the same reasons as cityImage: per-edge
> volumes were cumulative rather than per trip, and the module exported nothing. It also had no
> headless entry point at all before that date. Details in `bug_changelog.md`.
>
> **And barrier avoidance inside a region subgraph did not happen at all** until 14 September 2026.
> That bears directly on what this module measures — see item 4, which is the first thing to run.

---

## 1. `PopulationPars` is gone, and this module is why that is safe

`PopulationPars` held 24 survey-derived route-choice probabilities — `probUsingElements = 0.63`,
`probRoadDistance = 0.22`, `naturalBarriers = 0.49` and their standard deviations. It was removed as
dead code (its only reader was `BarrierPreference`, which had no readers).

What makes that safe rather than a loss is that **`Muenster_clusters.csv` carries the same quantities
per cluster**: `usingElements_mean/std`, `onlyDistance_*`, `onlyAngular_*`, `regions_*`, `barriers_*`,
`distantLandmarks_*`, `preferenceNatural_*`. `PopulationPars` was the population-level aggregate of
what the CSV holds per group — GROUP2's `usingElements_mean` is 0.680 against the aggregate 0.63.

**Do not re-add the aggregate.** If a population-level figure is wanted, derive it from the clusters
weighted by group size, so there is one source.

## 2. Only Muenster has cluster data

`Muenster_clusters.csv` is the only `*_clusters.csv` in the repo, and the module's defaults —
Muenster, 301 agents, 10 jobs — are that study's. Running it on another city falls back to whatever
`EmpiricalGroup` provides without cluster-specific preferences. Worth a check that it fails visibly
rather than quietly running undifferentiated groups.

## 3. Smaller

- **The deprecated `initFromArgs(String[])` still has a caller here.**
- **No per-city configuration.** `loadCityConfig` stays core's no-op, deliberately: those files
  configure activity behaviour and this module models none. The cluster CSV is the equivalent here.
- **`EmpiricalPars.applyDefaults()` is no longer called** from the launch path; the module applies its
  Muenster/301/10 defaults in `applyParameters`, and only where the command line is silent. The method
  still exists and sets `Pars.cityName` unconditionally — check for other callers before keeping it.

## 4. Run it — the region-subgraph avoid-set fix is unexercised

**`Dijkstra.subGraphInitialisation` discarded the primal edge-avoid set** instead of mapping it onto
the subgraph: the condition tested the wrong field, in the wrong sense, so avoidance inside a region
subgraph silently did not happen whenever there was anything to avoid. Fixed 14 Sep 2026.

This module is outside the default build profile and was not run, so the fix compiles under
`-Pcityimage-empirical` and nothing more has been checked.

**Why it matters more here than almost anywhere.** The clusters this module exists to contrast are
partly *defined* by barrier preference — `barriers_mean/std` and `preferenceNatural_mean/std` in
`Muenster_clusters.csv`. If avoidance was being dropped whenever a region subgraph was in play, then
the groups were differentiated on paper and not in the routes, and any earlier comparison between
them understates the separation by an unknown amount.

**What is needed:** a Muenster run (the only city with cluster data) with region-based navigation on
and the barrier-preference groups active, checked against the same seed before the fix — so the
question of how much it moved is answered with a number rather than an argument.

A second fix the same day, `DijkstraGlobalLandmarks.findBestLandmarkness`, also touches this module
wherever `distantLandmarks_*` drives the route choice; the full description is in
[../cityimage/TODO.md](../cityimage/TODO.md), item 5.
