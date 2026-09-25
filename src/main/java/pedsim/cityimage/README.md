# City-image module (`pedsim.cityimage`)

A **route-choice experiment harness**. It reproduces the published city-image studies — *Testing
Landmarks* and *Testing Urban Subdivisions* — comparing the effect of urban elements (landmarks,
regions, barriers) on pedestrian route choice against minimisation-based models (distance shortest
path, least cumulative angular change): one shared OD matrix is run under several route-choice
strategies in parallel so their flows can be compared on identical demand.

It extends `core` directly (it is **not** an activity-based module — there is no census, no 24h
clock, no home/work routine).

## Relationship to core

| Concern | Core class | City-image class |
|---|---|---|
| Simulation state | `PedSimCity` | `PedSimCityImage` (default scenario = `Scenario.values()`) |
| Engine | `Engine` | `CityImageEngine` |
| Import | `Import` | `CityImageImport` |
| Population | `Populate` | `CityImagePopulate` (builds the OD matrix, one agent per scenario) |
| Agent | `Agent` | `CityImageAgent` (runs a fixed OD list with an assigned route-choice strategy) |

`CityImagePopulate.populateTests` ignores the census/home-work path entirely: it generates an OD
matrix (generic, landmark, subdivision, or manual specific-OD mode per `TestPars`) and instantiates
one `CityImageAgent` per scenario so each walks the identical OD set.

## Testing modes (`parameters/TestPars`)

| Mode | OD generation |
|---|---|
| generic | random origins from `startingNodes`, destinations in `[Pars.minRouteLength, maxRouteLength]` (900–2700 walked m) |
| landmarks | single origin, 255 destinations from a fixed distance set |
| subdivisions | origins from starting nodes, destinations in 1000–3000 m |
| specific OD | manual origin/destination node-ID pairs (`originsTmp` / `destinationsTmp`) |

Route-choice models default to `ROAD_DISTANCE` and `ANGULAR_CHANGE` (configurable via
`TestPars.scenarios`).

## Running

Build with the `cityimage-empirical` profile (kept together with `empirical`; **not** in the default
build):

```bash
mvn -Pcityimage-empirical compile
mvn -Pcityimage-empirical compile exec:java \
  -Dexec.mainClass=pedsim.cityimage.launcher.CityImageLauncher \
  -Dexec.args="--headless --cityName=Muenster --jobs=1 --stringMode='Testing Urban Subdivisions'"
```

`--stringMode` selects the test design; `--numberTripsPerAgent` and `--jobs` override that design's
own defaults. **"Testing Landmarks", the default design, needs `<City>_distances.csv`, which no
bundled city ships.**

## Notes

- Harness for reproducing the route-choice / landmark experiments from the papers; it shares the
  current core engine and agent lifecycle but keeps its own OD/test plumbing.
- Has its own `utilities/StringEnum` (`Scenario`); each scenario's route-choice model is stated in
  `CityImageAgent.modelFor`, an exhaustive switch.
