# PedSimCity #

## An Agent-Based Model for simulating pedestrian movement in large urban areas ##

[![GitHub CI](https://github.com/g-filomena/PedSimCity/actions/workflows/build.yaml/badge.svg)](https://github.com/g-filomena/PedSimCity/actions/workflows/build.yaml)
[![License](https://img.shields.io/badge/License-GPLv3-blue.svg)](https://www.gnu.org/licenses/gpl-3.0.en.html)

This model simulates the movement of pedestrians across the street network of large urban areas. The novelty of the model lies on the inclusion of cognitive representations of space (cognitive maps) in the behavioural architecture of the pedestrian agents.

More specifically, a computational approach to Kevin Lynch's The Image of the City (see related paper in [Cities](https://www.sciencedirect.com/science/article/pii/S0264275118309776)) is employed to incorporate salient urban elements in the cognitive maps of the agents - alongside perception of distances and angular relationships between road segments. 
It is argued that the include of certain urban elements in one’s cognitive map shapes their route choice behaviour, that is how they formulate a route between an origin and a destination. 

The ABM has been built following a stepwise approach, so as to explore and assess the effect of the inclusion in the cognitive map of the agents of different urban elements (1. landmarks, 2. regions and barriers). 

The impact of element-based route choice models within the model was assessed in comparison with minimisation-based route choice models (i.e. distance shortest path, least cumulative angular change). 
The inclusion of these urban elements has been tested, in combination with existing route choice models:
* Landmark-based navigation: London - Methods, results and evaluation are documented in *Modelling the effect of landmarks on pedestrian dynamics*, published in [Computers, Environment and Urban Systems](https://doi.org/10.1016/j.compenvurbsys.2020.101573).
* Region- and barrier-based navigation: London and Paris - Methods and results, along with a validation are documented in *Perception of urban subdivisions in pedestrian movement simulation*, published in [PLoS ONE](https://doi.org/10.1371/journal.pone.0244099).

The ABM allows executing these experiments *Testing Landmarks* and *Testing Urban Subdivisions*.

In addition, the ABM, can be run as an empirical-based model where the interaction between the effects of the different urban elements is regulated and calibrated on the basis of empirical data (“Empirical ABM”). 
The ABM, the qualitative study conducted to calibrate it, and its evaluation are documented in *Empirical characterisation of agents’ spatial behaviour in pedestrian movement simulation*, 
published in [Journal of Environmental Psychology](https://www.sciencedirect.com/science/article/pii/S0272494422000524).

![](figures/urbanSub.png)
*Testing Urban Subdivisions: Pedestrian Volumes after elaboration in Python (London, UK)*

PedSimCity is built on:
* [JTS](https://github.com/locationtech/jts)
* [Mason](https://cs.gmu.edu/~eclab/projects/mason/extensions/geomason/)
* [GeoMason-light](https://github.com/g-filomena/GeoMason-light)

Along with:
* [Apache Commons Lang](https://commons.apache.org/proper/commons-lang/download_lang.cgi)
* [OpenCsv](http://opencsv.sourceforge.net)
* [Java Tuples](https://www.javatuples.org)
* [SLF4J](https://www.slf4j.org)

**How to run:**
1. Install Java (JDK 21) on your machine.
2. Download the jar file *pedsimcity1.23-jar-with-dependencies.jar* wherever it is convenient.
3. Open the command prompt in the directory where the .jar file is placed.
4. Run `java -jar pedsimcity1.23-jar-with-dependencies.jar --cityName=Torino --days=1`.
5. The run starts immediately and logs to the terminal. Add `--website` for the browser dashboard.

**This is the recommended option for running PedSimCity and it does not require the user to take any other step or to manually install the dependencies.**

> **The Java (AWT) GUI was removed on 13 September 2026.** There is no window and no
> *Run Simulation* button. A run is configured by two things, both of which leave a record it can be
> reproduced from: `src/main/resources/<City>/<City>.properties` for per-city parameters, and
> `--key=value` on the command line for everything else. The step that used to ask for the
> `src/main/resources/` path by hand is gone too — resources are read from the classpath.

To work on the sources in an IDE instead:

1. Clone the repository (it uses Git LFS for the GIS data; `git lfs pull` if the .gpkg files look like 133-byte text).
2. Open the folder as a Maven project; the dependencies in `pom.xml` resolve automatically.
3. Run `mvn -Pall-modules compile` to build every module — the default profile builds fewer than half.
4. Run any launcher's `main`: `pedsim.night.launcher.NightLauncher`, `pedsim.activity.launcher.ActivityLauncher`, `pedsim.core.launcher.CoreLauncher`, and so on.

**How to run in an editor such as Cursor or VS Code:**

1. Ensure **Maven** and **Java** (JDK 21) are installed on your computer.
2. Open the project folder in your editor (e.g. VS Code or Cursor).
3. Open the terminal and ensure you are in the project folder with the `pom.xml` file.
4. The fastest way to run the simulation after making changes is:
   `mvn compile exec:java` (core) or `mvn compile exec:java@night` (night module). Both run
   headless; add `@night-website` for the browser dashboard.
   *Note: Only use `mvn clean compile exec:java` if you are experiencing caching issues or have changed your dependencies in `pom.xml`. Skipping `clean` makes incremental builds much faster.*
5. Alternatively, use your IDE's run button on `NightLauncher.java` (`pedsim.night.launcher`).

The city-image module runs the simulation with three different configurations:
1. Testing Landmarks (London, Muenster).
2. Testing Urban Subdivisions (London, Paris, Muenster).
3. Testing Specific Route Choice Models (Muenster).
4. Empirical ABM (Muenster).

Each design carries the `jobs`, `numAgents` and `numberTripsPerAgent` used to produce the results in
the papers above, and `--stringMode` selects between them:

```bash
mvn -Pcityimage-empirical compile exec:java \
  -Dexec.mainClass=pedsim.cityimage.launcher.CityImageLauncher \
  -Dexec.args="--headless --cityName=Muenster --stringMode='Testing Urban Subdivisions'"
```

`--numberTripsPerAgent` and `--jobs` override the design's own defaults. For specific
origin-destination pairs, pass `--testingSpecificOD=true` with `--originsTmp` and `--destinationsTmp`
as comma-separated node IDs. Under "Testing Specific Route Choice Models" one agent walks the matrix
per route-choice model.

**"Testing Landmarks" needs `<City>_distances.csv`, which no bundled city currently ships** — see
`src/main/java/pedsim/cityimage/TODO.md`.

## Architecture: modules

`core` is shared infrastructure: the engine and day loop, routing and path finding, the
cognitive-map machinery, the REST server and dashboard state. It models no behaviour — no census, no
personas, no agendas — so a bare core run releases agents at a flat rate and sends them somewhere
they know; it exists as the skeleton the domain models extend, and as a smoke test. Each module has
its own README and `TODO.md` under `src/main/java/pedsim/<module>/`:

| Module | Role |
|---|---|
| [`core`](src/main/java/pedsim/core/README.md) | shared infrastructure: engine, routing, cognition, REST/dashboard |
| [`activity`](src/main/java/pedsim/activity/README.md) | activity-based **24h foundation**: census home/work, workplace/night POI destinations, day/night clock |
| [`night`](src/main/java/pedsim/night/README.md) | extends `activity` — vulnerability + lighting-aware night routing |
| [`learning`](src/main/java/pedsim/learning/README.md) | extends `activity` — incremental, decaying cognitive map |
| [`empirical`](src/main/java/pedsim/empirical/README.md) | extends `core` — **empirical ABM**: behaviour calibrated on empirical data |
| [`cityImage`](src/main/java/pedsim/cityimage/README.md) | extends `core` — **route-choice experiments**: effect of landmarks / regions / barriers |

The dependency layering is `core → activity → {night, learning}`; `empirical` and `cityImage` extend
`core` directly. The REST API/dashboard live in `core`, but `/api/modules` lists only registered
runnable modules and `/api/start` routes to them.

### Running a module

Each runnable module has Maven exec profiles (headless run, and REST + browser dashboard); see its README for parameters:

| Module | headless | REST + dashboard |
|---|---|---|
| night (default) | `mvn compile exec:java@night` | `mvn compile exec:java@night-website` |
| activity | `mvn compile exec:java@activity` | `mvn compile exec:java@activity-website` |
| learning | `mvn -Plearning compile exec:java@learning` | `mvn -Plearning compile exec:java@learning-website` |
| cityImage / empirical | `mvn -Pcityimage-empirical compile …` | — |

## Repository layout

| Path | Contents |
|---|---|
| `src/main/java/pedsim/<module>/` | Java source, one folder per module (each with a README) |
| `src/main/resources/<City>/` | per-city GIS input layers read by the simulation (`<City>_*.gpkg`) |
| `inputData/<City>/` | raw preparation material (DTM/DEM rasters, detailed building layers, source notes); `00_city_preparation.py` searches it automatically after the resources folder |
| `pipeline/` | Python data-prep scripts + `build_lighting*.py` orchestrators — see [`pipeline/README.md`](pipeline/README.md) |
| `analysis/` | Jupyter notebooks + scripts for post-hoc analysis of results |
| `outputs/` | all simulation results (gitignored) |
| `.githooks/` | Git LFS guard + spotless formatting hooks — **copy them in once per clone**, see [`.githooks/README.md`](.githooks/README.md) |

**Code formatting is automatic, once the hooks are installed.** Java is formatted with
[spotless](https://github.com/diffplug/spotless) + google-java-format; `pre-commit` applies it to
what you staged and `pre-push` refuses a push that is not clean. `mvn spotless:apply` does it by
hand. The hooks are not installed by cloning — `cp .githooks/pre-commit .git/hooks/` and likewise for
`pre-push`.

**Preparing a city's data** — on Windows double-click `build_city.bat` (base layers +
POIs), `build_census.bat` (ISTAT census) or `build_lighting.bat` (street lighting from
the lamp inventory) and enter the city; or run e.g.
`python pipeline/build_lighting.py --city <City>`. Raw material goes in
`inputData/<City>/`; the pipeline writes what the sim reads to `src/main/resources/<City>/`.
See [`pipeline/README.md`](pipeline/README.md) for the steps and the `<City>_…` filename
conventions.

**Results** — every run writes under `outputs/`: structured exports (volumes, routes, cognitive
maps) under `outputs/<appName>/`, with trip diagnostics and the HTML dashboard at the `outputs/`
root.

**Publishing results** — the result pages under `outputs/results/` are self-contained HTML.
`python publish_site.py` (or `publish_site.bat`) stages them into `outputs/site/` — an overview
page plus one sub-page per city — and deploys to Cloudflare Pages (project `inclusivestreets`),
served at [pedsimcity.inclusivestreets.org](https://pedsimcity.inclusivestreets.org), with each
city at `pedsimcity.inclusivestreets.org/<City>`. The site lives at the root of its own
subdomain; the `inclusivestreets.org` apex is a separate umbrella and is not managed by this
script. Use `--no-deploy` to only stage the folder (drag-and-drop it in the Pages dashboard
instead), or `--open` to preview it locally. One-time toolchain setup (per machine):
`npm install -g wrangler`, `wrangler login`, `wrangler pages project create inclusivestreets`;
the `pedsimcity` subdomain is then attached once in the Cloudflare dashboard
(Workers & Pages → inclusivestreets → Custom domains).

**How to use the Web Dashboard:**

PedSimCity features a browser-based dashboard for real-time simulation monitoring via a REST API.
The REST server runs on `http://localhost:8081`; the dashboard is opened automatically as a local
HTML file (`dashboard.html`).

To start the night-module REST server and dashboard:

```bash
mvn compile exec:java@night-website
```

When the startup menu appears, select **Option 2 (Browser/HTML Dashboard)**. The REST API starts
on port 8081 and `dashboard.html` opens in your browser automatically.

Endpoints available once the server is running:

| Endpoint | Description |
|---|---|
| `GET /api/state` | Live simulation state (agents, stats, module info) |
| `GET /api/roads` | GeoJSON road network |
| `GET /api/modules` | Registered runnable modules with parameter schemas |
| `POST /api/start` | Start a simulation run (body: `{"module":"night","cityName":"Torino",…}`) |

To start a simulation from the command line once the server is running:

```bash
curl -X POST http://localhost:8081/api/start \
  -H "Content-Type: application/json" \
  -d '{"module":"night","cityName":"Torino","days":1}'
```
