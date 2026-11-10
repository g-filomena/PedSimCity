# Learning module (`pedsim.learning`)

An incremental spatial-learning pedestrian model. Agents do **not** know the whole city: each
navigates within a personal cognitive map that **grows** from the routes it actually walks and
**fades** over time, and whose recognition of landmarks/easiness **adapts** with accumulated
experience (novice → expert).

## Relationship to core

The module reuses core for everything generic and only adds the learning-specific layer. It mirrors
the `night` module's "extend core" pattern.

| Concern | Provided by |
|---|---|
| Engine loop, day/agent-release, flow & cognitive-map export | `core.engine.Engine` (via `LearningEngine`) |
| Simulation state, GIS data, scheduling | `core.engine.PedSimCity` (via `PedSimCityLearning`) |
| Agent movement, trip planning, home/work fields | `core.agents.Agent` (via `LearningAgent`) |
| Population, home/work assignment (census → DMA → uniform) | `core.engine.Populate` (via `LearningPopulate`) |
| Routing, routers, graph search, landmark/region/barrier navigation | `core.routing.*` |
| Activity bone, known-network, community cognitive map | `core.cognition.cognitivemap.*`, `core.cognition.network.NetworkBuilder` |
| REST/dashboard | `core.website.*` |

The learning-specific classes are:

| Class | Role |
|---|---|
| `cognitiveMap/CognitiveMap` | subclass of `core` `CognitiveMap`; seeds initial memory in `formCognitiveMap()` and makes the local-landmark and wayfinding-easiness thresholds **vividness-driven** |
| `cognitiveMap/IncrementalLearning` | seeds, reinforces and decays route memory; builds the `VividnessGrid` collage that re-shapes the known space |
| `cognitiveMap/VividnessGrid` | decaying raster of how strongly each area is remembered, fading as one scale factor; produces the collage polygons and the vividness between two places |
| `cognitiveMap/ElementsSensitivity` | maps vividness → local-landmark recognition and wayfinding-easiness thresholds, per space |
| `agents/RouteChoiceSurvey` | each learner's route-choice probabilities, drawn from the survey's population row |
| `routeMemorability/RouteProperties` | the visibility space of a walked route: what the walker could see from it |
| `engine/Learning*`, `agents/LearningAgent`, `launcher/LearningLauncher` | module scaffolding |

Learning parameters live in `learning.parameters.LearningPars`. They used to sit in
`core.parameters`, shared with core's cognitive map, but the one constant core read from them
seeded an agent's spatial ability - a trait every agent has, learning module or not - and nothing
here read it. It is now `CognitiveMap.MEAN_SPATIAL_ABILITY`, and core no longer depends on a
module.

## How it works

1. **Population** — `LearningPopulate` creates `LearningAgent`s (activity agents: persona, agenda,
   commute), assigns home/work via `ActivityPopulate.defineHomeWorkLocations`, and alternates
   `LEARNER` / `NOT_LEARNER` per agent ID. Each agent's `spatialAbility` is drawn from the Santa
   Barbara Sense-of-Direction scale (core `CognitiveMap`).
2. **Map formation** — on first activity, `formCognitiveMap()` builds the core activity bone and
   known network around the agent's cognitive anchors, then seeds initial route memory with one
   route from home to each other anchor (`IncrementalLearning.buildBasicMemory`).
3. **Reinforcement** — after each completed trip, `LEARNER` agents call `updateAgentMemory(route)`:
   the route's visibility space is stamped into the `VividnessGrid` and the known space is
   recomputed from the cells holding at least `memoryThreshold` walks.
4. **Decay** — every step, each `LEARNER` fades its memory (`IncrementalLearning.applyDecay`,
   half-life `LearningPars.halfLife` for the best agent, up to twice as fast for the worst, by
   `spatialAbility`). The known space is rebuilt when a known cell fades below `memoryThreshold`.
   Wired in `LearningAgent.step()`.
5. **LEARNER vs NOT_LEARNER** is the experimental contrast: learners' maps evolve and decay;
   non-learners keep their initial map.
6. **Route choice** is sampled per trip from the learner's own survey-derived probabilities: a
   pure minimisation, or sub-goals from regions, local landmarks, barriers and distant landmarks,
   each only where the city carries the data. A learner's landmarks are those beside its known
   nodes above its vividness-driven recognition threshold.

## Running

Build with the `learning` profile (it is **not** in the default build):

```bash
# headless run (LearningLauncher)
mvn -Plearning compile exec:java@learning

# REST API + browser dashboard (port 8081)
mvn -Plearning compile exec:java@learning-website
```

Start a run via REST once the server is up:

```bash
curl -X POST http://localhost:8081/api/start \
  -H "Content-Type: application/json" \
  -d '{"module":"learning","cityName":"Muenster","days":7,"jobs":1}'
```

`GET /api/modules` lists `learning` with its parameter schema.

## Parameters

Common (any module): `cityName`, `days`, `actualPopulation`, `percentage`, `jobs`.

Module-specific (`LearningSimulationModule`):

| Key | Type | `LearningPars` field |
|---|---|---|
| `halfLife` | double | route-memory half-life (days) |
| `memoryThreshold` | double | `memoryThreshold` (walks a cell needs to be known) |
| `consolidatedShare` | double | share of a walk that does not fade |
| `meaningfulnessGain` | double | what a meaningful route adds to its one walk; 0 is off |

## Notes

- Works for any city that has the required network/landmark/region layers (no city is hard-coded;
  `Pars.cityName` defaults to `Torino`). The cognitive-map machinery needs buildings/landmarks and
  district/region data to be present for the chosen city.
- Memory decay runs per step per learner and costs a multiplication: the grid fades as one scale
  factor.

## Open items

`TODO.md` beside this file. The activity tier's, in `../activity/TODO.md`, apply as well:
`LearningAgent` extends `ActivityAgent`, so personas and daily agendas apply here.
