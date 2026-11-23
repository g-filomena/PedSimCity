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
| `memorability/RouteProperties` | the visibility space of a walked route: what the walker could see from it |
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
   Barbara Sense-of-Direction scale (core `CognitiveMap`), and sorts it into one of three memory
   profiles (`MemoryProfile`): the lowest 30% imprecise navigators, the highest 30% integrators,
   the rest non-integrators (Weisberg and Newcombe's groups).
2. **Map formation** — on first activity, `formCognitiveMap()` builds the core activity bone and
   known network around the agent's cognitive anchors, then seeds initial route memory with one
   route from home to each other anchor (`IncrementalLearning.buildBasicMemory`).
3. **Reinforcement** — after each completed trip, `LEARNER` agents call `updateAgentMemory(route)`:
   the route's visibility space is stamped into the `VividnessGrid` and the known space is
   recomputed from the cells holding at least `memoryThreshold` walks.
4. **Decay** — every step, each `LEARNER` fades its memory (`IncrementalLearning.applyDecay`). The
   walked traces fade at `LearningPars.halfLife` for everybody. The consolidated part (a share of
   each walk) fades at `consolidatedHalfLife` for integrators and `unintegratedFade` times faster
   for the others, who also keep `unintegratedKeep` of the share; imprecise navigators add
   `impreciseRouteGain` of what a walk adds. The known space is rebuilt when a known cell fades
   below `memoryThreshold`. Wired in `LearningAgent.step()`.
5. **LEARNER vs NOT_LEARNER** is the experimental contrast: learners' maps evolve and decay;
   non-learners keep their initial map.
6. **Route choice** is sampled per trip from the learner's own survey-derived probabilities: a
   pure minimisation, or sub-goals from regions, local landmarks, barriers and distant landmarks,
   each only where the city carries the data. A learner's landmarks are those beside its known
   nodes above its vividness-driven recognition threshold.
7. **Familiarity planning** (`familiarityPlanning`): instead of confining the search to the known
   network, a learner plans over the whole network. Each street has a familiarity from 0 to 1 (1
   for what it knew from the start, otherwise `w / (w + memoryThreshold)` over the grid cells it
   crosses); its perceived cost is its length times `1 + uncertaintyPremium * (1 - familiarity)`,
   off by an error whose spread grows to `unfamiliarErrorMultiple` times the familiar one. The
   error is fixed for the leg.
8. **Aware destinations** (`awareDestinations`): a learner chooses among the places it knows of,
   at a known node or in a cell holding `awarenessThreshold` walks, and on a trip with probability
   `discoveryProbability` one place it has not seen.

## Shadow pairs

With `shadowPairs`, every learner gets a shadow: an agent with its home, workplace and persona
that starts from a copy of the learner's knowledge, never learns and never walks, and plans every
leg the learner plans, with the model the learner sampled. The `learning pairs` day log compares
the two routes on the same origin-destination pairs, and both against the shortest path.
`knowledgeLevels` sets the knowledge learners start with, assigned in turn by agent ID: a share of
the community's network nearest the anchors (`0` is a newcomer, `1.0` all of it) or, above 1, a
radius in metres around the anchors. `fixedPersona` gives every learner one persona.

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
| `integratorShare`, `impreciseShare` | double | shares of the population in the two outer memory profiles |
| `impreciseRouteGain` | double | what an imprecise navigator adds of a walk |
| `unintegratedKeep`, `unintegratedFade` | double | share kept and speed of loss of the consolidated part, for non-integrators and imprecise navigators |
| `meaningfulnessGain` | double | what a meaningful route adds to its one walk; 0 is off |
| `consolidatedHalfLife` | double | half-life, in days, of the consolidated part for integrators |
| `learningEnabled` | boolean | false leaves every learner with the map it started with |
| `familiarityPlanning` | boolean | plan over the whole network, pricing streets by familiarity |
| `uncertaintyPremium` | double | extra perceived cost of a wholly unfamiliar street, as a share of its length |
| `unfamiliarErrorMultiple` | double | perception-error spread on a wholly unfamiliar street, in multiples of the familiar one |
| `awareDestinations` | boolean | choose destinations among the places known of |
| `awarenessThreshold` | double | walks a cell needs for a place in it to be known of |
| `discoveryProbability` | double | chance per trip of hearing of one unseen place |
| `habitMode`, `habitGain`, `rewardGain`, `rewardRate` | enum, double | habitual reinforcement of walked streets; `NONE` is off |
| `shadowPairs` | boolean | give every learner a non-learning shadow |
| `knowledgeLevels` | string | starting knowledge per learner, see *Shadow pairs* |
| `fixedPersona` | string | one persona for every learner; empty draws them |
| `rebuildEveryDays` | double | how often fading alone rebuilds the known network |
| `snapshotDays` | string | days on which each learner's activity space is written to `snapshots/` |
| `cellSize` | double | side of a vividness-grid cell, in metres |

Every field of `LearningPars` is settable as `--<field>=<value>`.

## Notes

- Works for any city that has the required network/landmark/region layers (no city is hard-coded;
  `Pars.cityName` defaults to `Torino`). The cognitive-map machinery needs buildings/landmarks and
  district/region data to be present for the chosen city.
- Memory decay runs per step per learner and costs a multiplication: the grid fades as one scale
  factor.

## Open items

`TODO.md` beside this file. The activity tier's, in `../activity/TODO.md`, apply as well:
`LearningAgent` extends `ActivityAgent`, so personas and daily agendas apply here.
