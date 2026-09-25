# Night module (`pedsim.night`)

A night-time pedestrian model. On top of the inherited activity 24-hour routine, it adds a runtime perception/safety layer: pedestrians have a vulnerability status and evaluate lighting, parks/water proximity, crowding and local knowledge after dark.

The split is deliberate:

- **Activity** decides where people go over 24 hours: home, workplace by day, night POIs after dark.
- **Night** decides how agents react while moving through a dark or unevenly lit city.

## Relationship to activity

Night extends the activity tier and layers vulnerability + lighting on top.

| Concern | Activity class | Night class |
|---|---|---|
| Simulation state | `PedSimCityActivity` | `PedSimCityNight` — vulnerability, illuminated edges, directional lux, route caches |
| Engine | `ActivityEngine` | `NightEngine` — night import/environment, A/B export, night diagnostics |
| Import | `ActivityImport` | `NightImport` — illuminated edges + directional lighting |
| Environment | `ActivityEnvironment` | `NightEnvironment` — `mean_lux` edge join |
| Population | `ActivityPopulate` | `NightPopulate` — per-agent vulnerability + optional A/B twins |
| Agent | `ActivityAgent` | `NightAgent` — night route planning and runtime lighting behaviour |

The 24h clock (`isDark`), workplace/night-POI data and time-of-day destination selection are inherited from activity.

## Night-specific layer

| Class | Role |
|---|---|
| `agents/NightAgent` | inherits the 24h activity pattern; after dark plans a night trip and filters out park/water destinations |
| `agents/NightAgentMovement` | movement layer; records measured lux, binary-lit fallback use and missing lighting data separately |
| `agents/NightBehaviour` | vulnerability/light-sensitivity behaviour; evaluates mean edge lux and directional entrance lux at runtime |
| `routing/routers/RoadDistancePathFinder`, `routing/search/DijkstraRoadDistanceNight` | night route planning based on road distance, with vulnerable-agent avoidance of parks/water and unknown regions |
| `parameters/NightPars` | light-sensitivity thresholds, crowdedness percentile, A/B-testing flag, directional lux statistic |

## Data layers

| Layer file (`<City>_...`) | Field | Purpose |
|---|---|---|
| `edges_illuminated_continuous.gpkg` | `mean_lux` | measured per-edge mean illuminance used by runtime situated behaviour |
| `directional_lighting_lookup.csv` | `visibility_min_lux`, `visibility_mean_lux` | directional entrance lighting per `current_node_id` / `target_node_id` pair |
| `censusData.gpkg` | `female_pct` | per-zone share of adults who are women; the activity tier draws each agent's sex from it |

**Who is vulnerable: women.** Sex is a census fact, drawn per agent from the home zone's `female_pct` by the activity tier alongside the persona; *vulnerable* is this module's judgement about it, made in `NightPopulate.assignVulnerabilityStatus` and nowhere else. The mechanism the module models — avoiding streets that are neither lit nor familiar after dark — is the one the evidence on fear of crime after dark supports for women. Age does not enter it: every agent the model builds is an adult, which is also why `female_pct` is a share of the zone's adults rather than of its residents. On Turin that is 52.5%.

Under A/B testing the split is experimental and set by construction, so it does not read sex at all.

## How it works

1. **Clock** — `isDark` comes from `Daylight.isDark(time)`: sunrise and sunset for the simulated date at the city's own latitude and longitude, measured from the street network.
2. **Vulnerability** — when `enableLightABTesting = false`, `NightPopulate.assignVulnerabilityStatus` makes every woman vulnerable; `NightAgent.initSensitivity` sets the light-sensitivity threshold.
3. **Destinations** — workplace POIs by day, night POIs after dark.
4. **Route planning** — when dark, `NightAgent` plans a `roadDistanceNight` route. A **known** edge's cost rises as it darkens, from 1.0 at the agent's own sensitivity threshold toward `NightPars.maxKnownDarkEdgeCostMultiplier` at total darkness; unknown edges are left to the situated gate below, so the same darkness is not charged twice. Vulnerable agents also avoid parks and water, and edges that are neither lit nor familiar.
5. **Runtime situated behaviour** — while walking after dark, agents evaluate measured `mean_lux`, directional entrance lux and binary `lit` fallback. Dark/unsafe edges can trigger local rerouting or speed-up behaviour.
6. **A/B testing** — with `enableLightABTesting = true`, `NightPopulate` spawns identical vulnerable/non-vulnerable twin pairs. In this mode the vulnerable/non-vulnerable split is experimental and does not read the agent's sex.

## Lighting semantics

`mean_lux` and directional lux are measured illuminance metrics.

The binary `lit` flag is only a pass/fail fallback when measured lux is missing. It is not treated as measured lux and is tracked separately from measured-lux exposure metrics.

`directionalLuxStatistic` selects which directional CSV column is used at runtime:

| Value | CSV column | Interpretation |
|---|---|---|
| `MIN` | `visibility_min_lux` | conservative default: minimum lux near the edge entrance |
| `MEAN` | `visibility_mean_lux` | average lux near the edge entrance |

Mean-light passes if measured `mean_lux` exists and exceeds the agent threshold, or if the binary `lit` fallback says the edge is lit. Entrance-light passes if directional lux exists and exceeds the threshold, or if the binary `lit` fallback says the edge is lit. Otherwise the check fails closed.

By default `nonVulnerableLightSensitivity = 5.0`, so non-vulnerable agents also respond to darkness, treating an edge below 5 lux as dark (the same unlit threshold as the vulnerable-agent minimum). Set `nonVulnerableLightSensitivity = 0.0` to make non-vulnerable agents insensitive to darkness — lux-driven behaviour then applies only to vulnerable agents, apart from parks/water logic.

## Diagnostics

The night module logs lighting coverage and lookup diagnostics, including:

- `mean_lux joined: X / Y illuminated records`
- `graph edges with mean_lux: A / B`
- `Directional lighting rows loaded: X`
- `directional lookup misses: M`
- `binary lit fallback used: N`
- `edges without any lighting data: K`

A near-zero directional lookup hit rate usually means the CSV node IDs do not match `NodeGraph.getID()`.

## Running

Night is the default Maven profile:

```bash
mvn compile exec:java@night-website
mvn compile exec:java@night
```

Start a run via REST:

```bash
curl -X POST http://localhost:8081/api/start \
  -H "Content-Type: application/json" \
  -d '{"module":"night","cityName":"Torino","days":7,"jobs":1}'
```

## Parameters

Module-specific REST parameters handled by `NightSimulationModule`:

| Key | Type | Default / field |
|---|---|---|
| `enableLightABTesting` | boolean | `NightPars.enableLightABTesting` |
| `crowdednessPercentile` | double | `NightPars.crowdednessPercentile` |
| `directionalLuxStatistic` | `MIN` \| `MEAN` | default `MIN` |
| `nonVulnerableLightSensitivity` | double | `NightPars.nonVulnerableLightSensitivity` |
| `useGravityModel` | boolean | `RouteChoicePars.useGravityModel` |

## Open items

