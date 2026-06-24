# Empirical module (`pedsim.empirical`)

An **empirically-grounded ABM**: pedestrians are drawn from empirical agent *groups* (behavioural
clusters with measured route-choice parameters) and routed over an OD matrix, so simulated flows can
be benchmarked against population and null activitys.

This is a legacy module that extends `core` directly. Like `cityimage` it is **not** an activity-based
module — it has no census home/work routine and no 24h clock; demand is an OD matrix, not a
daily schedule.

## Relationship to core

| Concern | Core class | Empirical class |
|---|---|---|
| Simulation state | `PedSimCity` | `PedSimCityEmpirical` (default scenario = `EmpiricalGroup.values()`) |
| Engine | `Engine` | `EmpiricalEngine` |
| Import | `Import` | `EmpiricalImport` |
| Population | `Populate` | `EmpiricalPopulate` (OD matrix → empirical groups) |
| Agent | `Agent` | `EmpiricalAgent` (+ `EmpiricalAgentProperties`, `EmpiricalAgentsGroup`, `EmpiricalGroup`) |

## How it works

1. **OD matrix** — `EmpiricalPopulate` builds `numAgents × numberTripsPerAgent` OD pairs, either
   randomly or **DMA-weighted** (`work` 0.30 / `visit` 0.46 / `random` 0.24) when `usingDMA` is set.
2. **Group allocation** — the OD matrix is split across the configured `empiricalGroups`. Optional
   `POPULATION` and `NULLGROUP` benchmarks are created first (when enabled), then the remaining
   population is divided among the empirical clusters by their `share`.
3. **Agents** — each `EmpiricalAgent` is assigned its slice of the OD matrix and its group's
   route-choice parameters, then walks its trips through the core engine loop.

## Parameters (`parameters/EmpiricalPars`)

| Field | Purpose |
|---|---|
| `empiricalGroups` | behavioural clusters with route-choice parameters and population `share` |
| `numberTripsPerAgent` | trips per agent |
| `usingDMA` | DMA-weighted vs random OD generation |
| `includePopulationBenchmark` / `includeNullBenchmark` | add population / null reference groups |

## Running

Build with the `cityimage-empirical` profile (empirical builds alongside the legacy `cityimage`;
**not** in the default build). `PedSimCityEmpirical` has a `main` entry point:

```bash
mvn -Pcityimage-empirical compile
mvn -Pcityimage-empirical compile exec:java \
  -Dexec.mainClass=pedsim.empirical.engine.PedSimCityEmpirical -Dexec.args="Torino"
```

(The first argument overrides `Pars.cityName`.)

## Notes

- Intentionally depends only on `core` and its own `empirical.*` classes — not on `cityimage` —
  despite sharing a build profile.
