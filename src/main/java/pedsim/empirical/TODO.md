# Empirical module — what is left to do

This module walks a shared origin-destination matrix with one agent group per survey-derived cluster
(`Muenster_clusters.csv`), so the groups differ in the route-choice preferences the clusters describe.
What the module *is* is in [README.md](README.md); what changed and when is in the root `CHANGELOG.md`.
This file is only the open work.

> **Nothing this module produced before 15 September 2026 is usable**, for the reasons in
> `../cityimage/TODO.md` — the cluster-derived route choice was overwritten at plan time, and the
> region, barrier and landmark mechanisms it contrasts were all returning the plain minimisation
> route.

---

## 1. The clusters walk disjoint OD sets, so they cannot be compared to each other

`EmpiricalPopulate.assignODMatrixToEmpiricalGroups` hands POPULATION and NULLGROUP the **whole** OD
matrix and gives each cluster a disjoint `subList` of it. That is right for assigning aggregate
volumes — each cluster is a share of the population making its share of the trips — but it means a
difference between GROUP1 and GROUP2 confounds route choice with geography, and no amount of seeding
separates them.

**So the measurement this module exists for cannot currently be made.** Two ways to get it:

- **Cheap and sufficient:** a diagnostic mode giving every cluster the same OD matrix, as cityImage
  already does for its scenarios. The groups then differ in nothing but preference. It changes no
  production path.
- **What exists instead:** each group compared *against itself* before and after a change, on one
  seed. That shows the preferences reach the routes; it cannot show how far apart the groups are.

## 2. Only Muenster has cluster data

`Muenster_clusters.csv` is the only `*_clusters.csv` in the repo, and the module's defaults — Muenster,
301 agents, 10 jobs — are that study's. Running it on another city falls back to whatever
`EmpiricalGroup` provides without cluster-specific preferences. **Worth a check that it fails visibly
rather than quietly running undifferentiated groups.**

## 3. `NULLGROUP` is a uniform prior, not an absence of mechanism

`EmpiricalAgentsGroup.setGroup` returns early for it and `randomizeRouteChoiceParameters` then calls
`initialiseUniform` over every route-choice property, so roughly half its agents are region-based and
many use barrier sub-goals. The name invites the opposite reading, and a benchmark meant to be "no
elements" is not what this is. It is also why NULLGROUP moves least under a change: a uniform draw
over every mechanism dilutes each one.

## 4. Do not re-add the population-level route-choice constants

`PopulationPars` held 24 survey-derived probabilities — `probUsingElements = 0.63`,
`probRoadDistance = 0.22`, `naturalBarriers = 0.49` and their standard deviations — and was removed.
What makes that safe is that **`Muenster_clusters.csv` carries the same quantities per cluster**:
`usingElements_mean/std`, `onlyDistance_*`, `onlyAngular_*`, `regions_*`, `barriers_*`,
`distantLandmarks_*`, `preferenceNatural_*`. The aggregate was the population-level summary of what
the CSV holds per group (GROUP2's `usingElements_mean` is 0.680 against the aggregate 0.63). If a
population-level figure is wanted, derive it from the clusters weighted by group size, so there is one
source.

## 5. Smaller

- **No per-city configuration, deliberately.** `loadCityConfig` stays core's no-op: those files
  configure activity behaviour and this module models none. The cluster CSV is the equivalent here.
- **The cluster is re-sampled per trip**, from `EmpiricalAgent.assignedRouteChoice()`. A cluster is a
  distribution over ways of getting somewhere, not a label fixed to a person — drawing once per agent
  makes a group's realised mix N draws instead of N × trips.
