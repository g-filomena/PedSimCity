# cityImage module — what is left to do

Split out of `/TODO.md` on 13 September 2026. This module compares route-choice models over a shared
synthetic origin-destination matrix: one agent per model, the same ODs, so the models differ in
nothing but how they choose a route.

> **Output from before 13 September 2026 is wrong.** Per-edge volumes were cumulative rather than per
> trip: `edgesWalkedSoFar` accumulated across an agent's trips, so each trip re-counted every earlier
> one. The module also exported nothing at all, and any region- or landmark-based *distance*
> comparison since March 2026 compared a whole route against a last-leg stub. Details in
> `bug_changelog.md`.
>
> **And landmark output from before 14 September 2026 is suspect, not merely superseded** — with
> region-based navigation on, the module could find no landmark route at all. See item 5, which is
> the first thing to run.

---

## 1. "Testing Landmarks" cannot run on any bundled city

`CityImageImport.importDistances` needs `<City>/<City>_distances.csv`, and
`ls src/main/resources/*/*_distances.csv` returns nothing. Since `TestPars.stringMode` defaults to
`"Testing Landmarks"`, **the module's default mode does not run out of the box.**

The file is not city data — it is experiment data: `uniqueID,trackID,length`, 255 GPS-track lengths
from the study, used at `CityImagePopulate:109` so the landmark test places its destinations at the
distances real trips actually had. Recovering it means finding the GPS distance files from the
original study.

The failure is at least immediate and legible (`Resource not found: …_distances.csv`), not silent.

## 2. London ships two networks and neither loads

`src/main/resources/London/` is organised per experiment, where every other city is flat:

| | landmarks | subdivisions |
|---|---|---|
| nodes | 8,178 | 9,997 |
| edges | 12,932 | 16,100 |
| node columns | `nodeID, x, y, height, Bc_Rd` | `nodeID, x, y, height, district, Bc_multi, gateway` |
| edge columns | … `Eb` (no `edgeID`) | `edgeID`, … |

The importer resolves `<City>/<City>_x`, flat, so **London cannot be loaded in either mode**. Each
folder carries exactly what its own study needs — road-distance betweenness for the landmark work,
district/gateway/`Bc_multi` for the region-and-barrier work.

Two ways out, and they are not the same size:

- **Cheap:** flatten to two cities, `London_landmarks/` and `London_subdivisions/`, each
  self-contained in the layout every other city uses. No importer change, no data surgery, both
  studies intact.
- **Real consolidation:** one London network from the pipeline carrying both attribute sets. The
  blocker is `London_sight_lines2D.gpkg` — **361,398 rows keyed `buildingID` ↔ `nodeID`**, bound to
  the landmarks network's 8,178 node IDs. Merging onto the other network dangles every one of them,
  so sight lines and landmark scores have to be re-derived, and that changes the network the
  published landmark results were computed on.

`London_distances.csv` is not part of the problem: it is 255 track lengths with no network reference.

## 3. `Pars.jobs` is reset between `applyMode()` and `runJobs()`

The module logs `… 1 job(s)` with `--jobs=1`, then more than one job executes. Something restores the
test design's own job count after the override. Harmless to correctness, wrong to the user.

## 4. Smaller

- **The deprecated `initFromArgs(String[])` still has a caller here.** It reaches core's three
  parameter classes only; the module's own list is in `parameterClasses()`.
- **No per-city configuration.** `loadCityConfig` stays core's no-op, deliberately: those files
  configure activity behaviour and this module models none. If cityImage ever needs city-level
  parameters, it needs its own file format, not the activity one.
- **Melbourne cannot run this module as shipped.** `src/main/resources/Melbourne/` has
  `_nodesDual.gpkg` but no `_edgesDual.gpkg`, and `Import.readGraphs` loads the dual graph only when
  both are present — so `ANGULAR_CHANGE`, one of the two default models, is silently unavailable.
- **Re-run the angular A/B properly.** The 13 Sep fix took angular fallbacks to 0, but attempts fell
  420 → 99 in the same run and the configuration behind the recorded 92/420 was never written down,
  so the *rate* comparison is not like-for-like. A clean before/after on one seed would settle it and
  would measure how much this module's simplest-path results move.

## 5. Run it — the 14 September subgraph and landmark fixes are unexercised

Two bugs were fixed in the region-subgraph seam on 14 Sep 2026, and both land squarely here. This
module is outside the default build profile, so neither fix has been exercised by a run: they
compile under `-Pcityimage-empirical`, and that is all that has been checked.

1. **`DijkstraGlobalLandmarks.findBestLandmarkness` tested raw subgraph nodes and edges against sets
   of *parent* objects.** `SubGraph` copies the parent's `nodeID`, coordinate and attributes onto the
   child, so the two are indistinguishable in a debugger while `contains()` is false for both. With
   region-based navigation on, an individualised agent could therefore find **no landmark route at
   all**. It now goes through `isNodeKnown` / `isEdgeKnown`, the mapped form `Dijkstra` already had.
2. **`Dijkstra.subGraphInitialisation` discarded the primal edge-avoid set** instead of mapping it
   onto the subgraph — the condition tested the wrong field, in the wrong sense — so avoidance inside
   a region subgraph silently did not happen whenever there was anything to avoid.

**What is needed:** a landmark-routing run on a city that carries landmark scores, with the
route-choice mix exercised so the region and landmark paths are actually taken, and a before/after on
one seed.

**Why this outranks the rest of the list.** (1) could have emptied the candidate set rather than
shifting it, so an old landmark comparison may have been measuring the fallback, not the model. That
is a different claim from "superseded", and it cannot be settled by reasoning — only by running it.
