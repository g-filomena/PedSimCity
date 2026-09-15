# cityImage module — what is left to do

This module compares route-choice models over a shared origin-destination matrix: one agent per
scenario, the same ODs, so the models differ in nothing but how they choose a route. What the module
*is* and how it relates to core is in [README.md](README.md); what changed and when is in the root
`CHANGELOG.md`. This file is only the open work.

> **Nothing this module produced before 15 September 2026 is usable.** The assigned model was
> overwritten at plan time, region navigation and barrier sub-goals returned the plain minimisation
> route, no version of the code had ever inserted an on-route mark, and two of the nine scenarios
> were one configuration under two names. The CHANGELOG entry for that date has the account.

---

## 1. London ships two networks and neither loads

`src/main/resources/London/landmarks/` and `.../subdivisions/` each hold a complete network under one
`London_` prefix, and layers are addressed flat as `<cityName>/<cityName>_<layer>`. They need to
become `London_landmarks/` and `London_subdivisions/`, files renamed to match the folder. No importer
change; verified by staging exactly that and running both studies.

Two things to carry into the split:

- `London_distances.csv` is 255 GPS-track lengths with no network reference, read by
  `CityImagePopulate` for *Testing Landmarks*. It belongs with the landmarks network and nothing else
  needs it.
- `London_sight_lines2D.gpkg` is **361,398 rows keyed `buildingID` ↔ `nodeID`** against the landmarks
  network's 8,178 node IDs. It cannot be pointed at the subdivisions network without re-deriving
  every sight line and landmark score — which would change the network the published landmark
  results were computed on.

## 2. Cumulative landmarkness is not implemented

It is this module's natural validation metric — the one the CEUS 2021 paper evaluates against GPS
trajectories — and the route *length* ratios currently standing in for it are a proxy. A single
assertion that the landmark scenarios score higher on it than the distance scenario would have caught
every defect found on 15 September, on day one.

## 3. Two cities cannot run what they are asked to

- **Melbourne.** `src/main/resources/Melbourne/` has `_nodesDual.gpkg` but no `_edgesDual.gpkg`, and
  `Import.readGraphs` loads the dual graph only when both are present — so `ANGULAR_CHANGE`, one of
  the two default scenarios, is silently unavailable.
- **Everywhere but London.** *Testing Landmarks* reads `<City>_distances.csv` and London's is the
  only one in the repo, so the mode fails immediately anywhere else. That failure is at least
  legible.

## 4. Re-run the angular A/B properly

The 13 September fix took angular fallbacks to 0, but attempts fell 420 → 99 in the same run and the
configuration behind the recorded 92/420 was never written down, so the *rate* comparison is not
like-for-like. A clean before/after on one seed would settle it, and would measure how much this
module's simplest-path results move.

## 5. Smaller

- **No per-city configuration, deliberately.** `loadCityConfig` stays core's no-op: those files
  configure activity behaviour and this module models none. If cityImage ever needs city-level
  parameters it needs its own format, not the activity one.
- **A mode's figures are defaults, so state them in `TestPars.defineMode()` and nowhere else.**
  `applyDefaults` runs before the command line, which then overrides whatever it named. Nothing has
  to be re-applied afterwards, and a key added to a design needs no second entry anywhere.

---

## How to test this module at all

**Pin the perception error first.** `Dijkstra.costPerceptionError` multiplies every edge cost by a
draw around 1.0, and over a route it does not average out: at the default sigma it produces ~0.20
volume divergence and ~0.43 edge overlap between two models whether or not those models differ.
`--perceptionErrorSD=0` makes the multiplier exactly 1.0, so `ROAD_DISTANCE` is the true shortest
path and minimal on every OD by construction. **If another model beats it, the comparison is
measuring something other than the model.**

**`-Dpedsim.trace=<file>` writes one line per planned leg** — scenario, agent, trip, origin,
destination, node and edge counts, length — from `Agent.initialiseRoute()`. No timestamps, so two
files are directly comparable with `cmp`, which is how a change is shown to alter nothing.

**Run one configuration twice before reading anything into it.** A mechanism that has just started
working is where non-reproducibility surfaces: region navigation began disagreeing with itself across
runs of one seed the moment it started producing routes.

```bash
java -Xmx24g -Dpedsim.trace=runs/ci.csv -cp "target/classes:$(cat cp.txt)" \
  pedsim.cityimage.launcher.CityImageLauncher \
  --headless --cityName=London_subdivisions \
  --stringMode="Testing Urban Subdivisions" --numberTripsPerAgent=150 --jobs=1 \
  --perceptionErrorSD=0
```
