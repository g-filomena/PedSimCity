# City-image module (`pedsim.cityimage`)

A **route-choice testing harness** (not a daily-routine model). It reproduces the historical
city-image experiments: generate one shared OD matrix, then run the *same* trips under several
route-choice strategies in parallel so their resulting flows can be compared on identical demand.

This is a legacy module: it extends `core` directly (it is **not** an activity-based module — there
is no census, no 24h clock, no home/work routine).

## Relationship to core

| Concern | Core class | City-image class |
|---|---|---|
| Simulation state | `PedSimCity` | `PedSimCityImage` (default scenario = `RouteChoice.values()`) |
| Engine | `Engine` | `CityImageEngine` |
| Import | `Import` | `CityImageImport` |
| Population | `Populate` | `CityImagePopulate` (builds the OD matrix, one agent per route-choice model) |
| Agent | `Agent` | `CityImageAgent` (runs a fixed OD list with an assigned route-choice strategy) |

`CityImagePopulate.populateTests` ignores the census/home-work path entirely: it generates an OD
matrix (generic, landmark, subdivision, or manual specific-OD mode per `TestPars`) and instantiates
one `CityImageAgent` per route-choice model so each walks the identical OD set.

## Testing modes (`parameters/TestPars`)

| Mode | OD generation |
|---|---|
| generic | random origins from `startingNodes`, destinations in `[minTripDistance, maxTripDistance]` |
| landmarks | single origin, 255 destinations from a fixed distance set |
| subdivisions | origins from starting nodes, destinations in 1000–3000 m |
| specific OD | manual origin/destination node-ID pairs (`originsTmp` / `destinationsTmp`) |

Route-choice models default to `ROAD_DISTANCE` and `ANGULAR_CHANGE` (configurable via
`TestPars.routeChoiceModels`).

## Running

Build with the `cityimage-empirical` profile (kept together with `empirical`; **not** in the default
build):

```bash
mvn -Pcityimage-empirical compile
mvn -Pcityimage-empirical compile exec:java -Dexec.mainClass=pedsim.cityimage.applet.PedSimCityImageApplet
```

## Notes

- Legacy harness retained for reproducing earlier route-choice/landmark experiments; it shares the
  current core engine and agent lifecycle but its own OD/test plumbing.
- Has its own `utilities/StringEnum` (`RouteChoice`) and `agents/AgentProperties` specialisation.
