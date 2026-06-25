# Core module (`pedsim.core`)

The shared foundation every simulation module builds on. Core owns the **infrastructure**: the
street/graph model, the cognitive-map and route-choice machinery, the day-based engine loop, agent
movement, and the REST/dashboard layer. It knows nothing about census data, activity patterns, night
lighting, vulnerability or learning — those live in the modules that extend core.

## What core provides

| Package | Responsibility |
|---|---|
| `engine` | `PedSimCity` (simulation state, GIS layers, scheduling), `Engine` (job loop, day/agent-release, flow & cognitive-map export), `Import` (graph/landmark/barrier GPKG loading), `Environment` (graph/buildings/gateways/dual-graph/barriers/regions preparation), `Populate` (agent creation + home/work assignment), `FlowHandler`, `ScenarioConfig`, `SimulationModule`, `SimulationLauncher`, recorders/exporters |
| `agents` | `Agent` (trip planning, movement lifecycle), `AgentMovement`, `AgentProperties`, `Heuristics`, `OdAgent`, `BarrierPreference` |
| `cognition.cognitivemap` | `CognitiveMap`, `SharedCognitiveMap` (the community/primal network) |
| `cognition.cityimage` | `Barrier`, `Gateway`, `Region` (the city-image elements) |
| `cognition.metrics` | `Landmarkness`, `LandmarkIntegration`, `BarrierIntegration`, `Complexity` |
| `cognition.network` | `NetworkBuilder` |
| `routing` | `RoutePlanner`, navigation `elements/*`, `pathfinder/*`, `pathfinding/*` (Dijkstra variants) |
| `parameters` | `Pars`, `RouteChoicePars`, `PopulationPars`, `TimePars`, `LearningPars` |
| `utilities` | `RobustVectorLayer`, `RouteData`, `StringEnum`, `LoggerUtil` |
| `website` | `SimulationRestApi`, `GeoJsonExporter`, `HtmlExporter` (dashboard) |
| `applet` | Swing GUI + REST launcher (`PedSimCityApplet`, `SimulationViewer`, panels) |

## The extension pattern

Every module specialises the same four core seams. Core stays census/activity-agnostic; modules add
their layer by subclassing:

| Core class | Overridable seams | Example override |
|---|---|---|
| `PedSimCity` | `populateEnvironment()` | choose the module's `Populate` |
| `Engine` | `createImporter()`, `prepareEnvironment()`, `clearStaticData()`, `onStepUpdate()`, `onJobFinished()` | wire module Import/Environment, clear module statics |
| `Populate` | `createAgent()`, `defineHomeWorkLocations()`, `assignHomeNode()`, `assignWorkNode()` | module-specific population |
| `Agent` | `getPOIWeight()`, `isDark()`, `planTrip()`, `planRoute()`, `defineRandomDestination()` | module-specific behaviour |

`Environment.prepare()` is called via `Engine.prepareEnvironment()` so a module can swap in its own
environment preparation (e.g. `ActivityEnvironment`, `NightEnvironment`) without touching core.

`getPOIWeight(node, isDark)` returns `0.0` in core (uniform destination selection) and `isDark()`
returns `false` (no day/night cycle) — both are activated by the **activity** layer.

## Module map

```
core  ──►  activity  ──►  night          (24h activity → + vulnerability/lighting)
                     ──►  learning        (24h activity → + incremental cognitive map)
core  ──►  empirical                      (empirical ABM: behavioural groups vs. empirical data)
core  ──►  cityimage                      (route-choice experiments: landmarks / regions / barriers)
```

## Running

Core is not run on its own; you run a module. The default Maven profile bundles core + night:

```bash
mvn compile exec:java@night-website     # REST + browser dashboard
mvn compile exec:java                    # GUI (PedSimCityApplet)
```

`Pars.cityName` defaults to `Torino`. Any city works provided its network/landmark/barrier GPKG
layers are present under `src/main/resources/<City>/`.
