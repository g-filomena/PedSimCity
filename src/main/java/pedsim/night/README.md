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
| `agents/NightAgentMovement` | walks the route; looks at each street it did not know, re-plans when one is darker than assumed, walks faster on unlit streets; records the lux walked |
| `routing/NightRouteCost` | what a street costs after dark: length raised by darkness and by parks or water, weighted by vulnerability |
| `routing/routers/RoadDistancePathFinder`, `routing/search/DijkstraRoadDistanceNight` | night route planning: least cost under `NightRouteCost` after dark, road distance by day |
| `engine/NightLighting` | whether a street reads as lit, how dark an illuminance is, and the typical lux of a street class |
| `parameters/NightPars` | darkness and park/water weights, the reassurance level, light-sensitivity thresholds, crowdedness percentile, A/B flag |

## Data layers

| Layer file (`<City>_...`) | Field | Purpose |
|---|---|---|
| `edges_illuminated_continuous.gpkg` | `mean_lux`, `min_lux` | measured per-edge illuminance: the route cost, the lit gate and the lux walked |
| `directional_lighting_lookup.csv` | `visibility_min_lux`, `visibility_mean_lux` | directional entrance lighting per `current_node_id` / `target_node_id` pair |
| `censusData.gpkg` | `female_pct` | per-zone share of adults who are women; the activity tier draws each agent's sex from it |

**Who is vulnerable: women.** Sex is a census fact, drawn per agent from the home zone's `female_pct` by the activity tier alongside the persona; *vulnerable* is this module's judgement about it, made in `NightPopulate.assignVulnerabilityStatus` and nowhere else. The mechanism the module models — avoiding streets that are neither lit nor familiar after dark — is the one the evidence on fear of crime after dark supports for women. Age does not enter it: every agent the model builds is an adult, which is also why `female_pct` is a share of the zone's adults rather than of its residents. On Turin that is 52.5%.

Under A/B testing the split is experimental and set by construction, so it does not read sex at all.

## How it works

1. **Clock** — `isDark` comes from `Daylight.isDark(time)`: sunrise and sunset for the simulated date at the city's own latitude and longitude, measured from the street network.
2. **Vulnerability** — when `enableLightABTesting = false`, `NightPopulate.assignVulnerabilityStatus` makes every woman vulnerable; `NightAgent.initSensitivity` sets the light-sensitivity threshold.
3. **Destinations** — workplace POIs by day, night POIs after dark.
4. **Route planning** — when dark, `NightAgent` plans the least-cost route under `NightRouteCost`: `length × (1 + darknessWeight × darkness(lux)) × (1 + parkWaterWeight)`, the park term only on park or waterside streets. `darkness` is 1 at 0 lux and falls concavely to 0 at `reassuranceLux` (10 lx, where reassurance plateaus). Both weights are higher for vulnerable agents. The agent plans with the measured lux of the streets it knows and the typical lux of their class for the rest.
5. **Walking** — arriving at a street it did not know, the agent sees its real lighting and whether it is busy (a busy street costs no darkness). If the street is darker than assumed, it plans again from where it stands and takes the new route only if that is cheaper than finishing the old one. A street seen once is known for the rest of the trip, so each re-plan lowers the cost of what is left and none can repeat. On a street that reads as unlit at its own sensitivity and is not busy, the agent walks faster.
6. **A/B testing** — with `enableLightABTesting = true`, `NightPopulate` spawns identical vulnerable/non-vulnerable twin pairs. In this mode the vulnerable/non-vulnerable split is experimental and does not read the agent's sex.

## Lighting semantics

`mean_lux` and directional lux are measured illuminance metrics.

The binary `lit` flag is only a pass/fail fallback when measured lux is missing. It is not treated as measured lux and is tracked separately from measured-lux exposure metrics.

`directionalLuxStatistic` selects which directional CSV column is used at runtime:

| Value | CSV column | Interpretation |
|---|---|---|
| `MIN` | `visibility_min_lux` | minimum lux near the edge entrance; about one lamp spacing wide, so it measures distance from the nearest lamp rather than how the street ahead is lit |
| `MEAN` | `visibility_mean_lux` | average lux near the edge entrance — the default |

Mean-light passes if measured `mean_lux` exists and exceeds the agent threshold, or if the binary `lit` fallback says the edge is lit. It then also has to clear the dark-spot test: `min_lux`, the darkest sample point on the edge, against `NightPars.darkSpotLuxThreshold` (5 lux, the pipeline's service level) rather than against the agent's own threshold, which a minimum over a whole edge would almost never clear. An edge carrying no `min_lux` passes that test, so a city without the lighting pipeline is gated on the mean alone. Entrance-light passes if directional lux exists and exceeds the threshold, or if the binary `lit` fallback says the edge is lit. Otherwise the check fails closed.

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
| `directionalLuxStatistic` | `MIN` \| `MEAN` | default `MEAN` |
| `nonVulnerableLightSensitivity` | double | `NightPars.nonVulnerableLightSensitivity` |
| `useGravityModel` | boolean | `RouteChoicePars.useGravityModel` |

## Open items

