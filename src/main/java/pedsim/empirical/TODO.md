# Empirical module — what is left to do

Split out of `/TODO.md` on 13 September 2026. This module walks a shared origin-destination matrix
with one agent group per survey-derived cluster (`Muenster_clusters.csv`), so the groups differ in the
route-choice preferences the clusters describe.

It got a `SimulationModule` and a real headless entry point on 13 Sep. Before that its applet opened a
window and nothing else, and the only display-less route was a `main` on the state class that took the
city as a bare positional argument with no parameter handling at all.

---

## Read this before trusting any earlier result

**Per-edge volumes produced before 13 September 2026 are wrong**, for the same reason as cityImage:
`AgentMovement.initialisePath()` never cleared `edgesWalkedSoFar`, and an `OdAgent` reuses one movement
handler across all its trips, so each trip re-counted every earlier one. See `../cityimage/TODO.md`.

Also fixed the same day, and both would have bitten this module first:

- **It exported nothing.** `EmpiricalEngine` overrides `executeJob` and never reached core's export.
- **`getHeuristics()` was null**, because `EmpiricalAgent.planRoute()` overrode core's and dropped the
  heuristics initialisation. This module threw on it outright as soon as it had a headless entry to be
  run from — its clusters use landmark-weighted models, which dereference it.

And from an earlier session: **any comparison of a region- or landmark-based *distance* model made
since March 2026 is void** — `roadDistanceSequence` returned the last leg rather than the whole
sequence. This module reaches that path through `EmpiricalAgentProperties`.

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
