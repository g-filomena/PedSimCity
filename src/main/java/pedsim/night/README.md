# Night module (`pedsim.night`)

A night-time pedestrian model. On top of the activity 24-hour activity routine it adds the
**perception/safety layer**: pedestrians have a **vulnerability** status and route through the city in
a **light-aware** way after dark, preferring illuminated streets and avoiding parks/water.

The split is deliberate: *where* people go over 24 hours (workplace by day, night POIs after dark) is
**activity**; *how* a vulnerable pedestrian routes through a dark, unevenly-lit city is **night**.

## Relationship to activity

Night extends the activity tier — it is an activity-based module — and layers vulnerability + lighting
on top:

| Concern | Activity class | Night class |
|---|---|---|
| Simulation state | `PedSimCityActivity` | `PedSimCityNight` (vulnerability, illuminated edges, directional lux, route caches) |
| Engine | `ActivityEngine` | `NightEngine` (night import/environment, A/B export, night data clearing) |
| Import | `ActivityImport` | `NightImport` (vulnerability census + illuminated edges) |
| Environment | `ActivityEnvironment` | `NightEnvironment` (vulnerability join + `mean_lux` edge join) |
| Population | `ActivityPopulate` | `NightPopulate` (per-agent vulnerability + A/B twins) |
| Agent | `ActivityAgent` | `NightAgent` (light-aware routing, park/water avoidance after dark) |

The 24h clock (`isDark`), workplace/night-POI data and time-of-day destination selection are all
**inherited** from activity; night does not redefine them.

## Night-specific layer

| Class | Role |
|---|---|
| `agents/NightAgent` | inherits the 24h activity pattern; overrides routing to use `roadDistanceNight`, and after dark plans a night trip and filters out park/water destinations |
| `agents/NightAgentMovement` | movement that accumulates `mean_lux` exposure per edge walked |
| `agents/NightBehaviour` | vulnerability/light-sensitivity behaviour |
| `routing/pathfinder/RoadDistancePathFinder`, `routing/pathfinding/DijkstraRoadDistanceNight` | lighting/vulnerability-weighted shortest path |
| `parameters/NightPars` | light-sensitivity thresholds, crowdedness percentile, A/B-testing flag |

## Data layers (night-specific)

| Layer file (`<City>_<name>.gpkg` / `.csv`) | Field | Purpose |
|---|---|---|
| `edges_illuminated_continuous` | `mean_lux` | per-edge illumination for light-aware routing |
| `directional_lighting_lookup` (CSV) | `visibility_min_lux` | directional entrance lighting per node pair |

Vulnerability is **not** a separate layer: it travels as the `vulnerability_pct` column on the
unified `censusData` layer (loaded by the inherited `ActivityImport`) and is resolved per node by
`NightEnvironment` from the shared `CensusZone` model. The activity columns (`residence_pct`,
`workplace_poi`, `night_poi`, `vulnerability_pct`) all live in that one census GeoPackage. Every
layer is optional and degrades gracefully when absent (no vulnerability → all agents non-vulnerable,
`mean_lux` defaults to 0).

## How it works

1. **Clock** (activity) — `isDark` flips between ~20:00 and ~06:00.
2. **Vulnerability** — `NightPopulate.assignVulnerabilityStatus` samples each agent's vulnerability
   from its home zone's `vulnerability_pct`; `NightAgent.initSensitivity` sets a light-sensitivity
   threshold (vulnerable agents draw from a higher range).
3. **Destinations** (activity) — workplace POIs by day, night POIs after dark.
4. **Routing** (night) — when dark, `NightAgent` plans a `roadDistanceNight` route that trades off
   distance against illumination/safety and avoids parks/water.
5. **A/B testing** — with `enableLightABTesting`, `NightPopulate` spawns identical vulnerable/normal
   twin pairs and `NightEngine` exports `ab_test_comparison.csv` at job end.

## Running

Night is the **default** Maven profile (core + activity + night):

```bash
mvn compile exec:java@night-website     # REST API + browser dashboard (default profile)
mvn compile exec:java@night             # GUI (PedSimCityNightApplet)
```

Start a run via REST:

```bash
curl -X POST http://localhost:8081/api/start \
  -H "Content-Type: application/json" \
  -d '{"module":"night","cityName":"Torino","days":7,"jobs":1}'
```

## Parameters

Module-specific (`NightSimulationModule`):

| Key | Type | `NightPars` field |
|---|---|---|
| `enableAB` | boolean | `enableLightABTesting` (spawn vulnerable/normal twins) |
| `crowdednessPercentile` | double | `crowdednessPercentile` |
