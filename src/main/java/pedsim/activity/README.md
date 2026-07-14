# Activity module (`pedsim.activity`)

The **activity-based foundation** for modules that run a full 24-hour daily routine. Activity sits
between `core` (pure infrastructure) and the routine modules (`night`, `learning`), adding the things
those modules share: a census-driven **population** (homes, headcount), an OSM-tag-driven
**activity POI** pattern (work and discretionary destinations), and a 24-hour **day/night clock**.

Activity is not census/lighting-agnostic like core, but it is *neutral* about perception and safety:
it has no vulnerability and no street-lighting behaviour. A plain activity run models people who
live in residence-weighted census zones and follow **personas** (worker / student / retiree / flex)
through home-based **tours**: a mandatory work/study leg on weekdays (inside a persona start
window), chained with discretionary stops typed by **activity purpose** (shopping, errands, dining,
nightlife, leisure, stroll) whose destinations are drawn from **OSM-tag-derived POI attraction**
and revisited habitually, with purpose-specific stay durations and opening hours, under a
day-of-week release curve and **seasonal daylight**.

## Relationship to core

Activity specialises each of core's four seams and adds an engine of its own:

| Concern | Core class | Activity class |
|---|---|---|
| Simulation state + activity data | `PedSimCity` | `PedSimCityActivity` (census, POI layers, purpose weights, `isDark`) |
| Engine wiring + 24h clock | `Engine` | `ActivityEngine` (activity import/environment, `onStepUpdate` sets `isDark` — seasonal via `Daylight`) |
| Data import | `Import` | `ActivityImport` (census GPKG + optional tagged-POI GPKG) |
| Environment preparation | `Environment` | `ActivityEnvironment` (census-zone spatial join) + `PoiClassifier` (OSM tags → per-node purpose weights) |
| Population | `Populate` | `ActivityPopulate` (residence-weighted home, work from WORK-tag attraction, persona assignment) |
| Agent | `Agent` | `ActivityAgent` (persona, daily agenda + trip chaining, purpose/habit destination choice, per-purpose stays) |

Supporting types (all in `pedsim.activity`): `Persona`, `ActivityPurpose`, `DailyAgenda`,
`Daylight`, `PoiClassifier`, `parameters.ActivityPars`.

## The 24-hour activity pattern

1. **Clock** — `ActivityEngine.onStepUpdate` sets `PedSimCityActivity.isDark`. With
   `ActivityPars.useSeasonalDaylight` (default) darkness follows the seasonal sunrise/sunset model
   in `Daylight` (latitude + day-of-year, civil-twilight buffer); otherwise the fixed
   `TimePars` 20:00–06:00 window. The exporter's day/night aggregation always uses the fixed
   window so outputs stay comparable. The release curve in `TimePars` is day-of-week aware:
   weekday commute peaks, a Friday night shift, and a weekend curve with no morning commute.
2. **Population** — `ActivityPopulate` draws each agent's **home** from residence-weighted census
   zones (the census is population structure only) and its **work** from the WORK-purpose
   attraction weights derived from OSM use tags (offices, commercial, industrial…), with gravity
   decay from home; it degrades gracefully to core's DMA → uniform-random fallback when either
   dataset is absent. Each agent gets a **persona**, sampled per home zone when the census carries
   age-structure shares (`retiree_pct`/`student_pct`; global `ActivityPars` shares otherwise):
   retirees/flex adults have no work node, students are re-targeted to education-tagged nodes when
   available, and walking speed varies by persona (±10% individual noise).
3. **Tours** — on release, the agent builds a `DailyAgenda`: the mandatory work/study leg is decided
   by the persona's weekday + start-window rule (`shouldGoToWork`), and discretionary stops are
   sampled from the persona's purpose mix restricted to each purpose's opening hours. After each
   stay (persona-specific at work, purpose-specific lognormal elsewhere) the agent **chains**
   directly to the next open activity from where it is, and heads home when the agenda is done.
3b. **Release realism** — who is out when is shaped three ways on top of the day-of-week curve:
   a **walk-share filter** keeps most short sampled trip distances and few long ones (logit,
   `ActivityPars.walkShare*`), **persona × hour affinities** favour commuters at the peaks and
   retirees midday (`Persona.releaseAffinity`), and seeded per-day **weather** scales the release
   budget on rainy days while thinning optional chained stops harder than commutes (`Weather`,
   `ActivityPars.rain*`). All three run through core seams (`acceptTripDistance`,
   `releaseCandidateWeight`, `releaseBudgetMultiplier`) so core stays module-agnostic.
4. **Destinations** — `ActivityAgent.getPOIWeight` weights candidates by the current purpose's
   per-node attraction, built by `PoiClassifier` from OSM-like use tags (`amenity`, `shop`,
   `leisure`, `office`, `use`, …) on the buildings and POI layers, and each leg samples from a
   **purpose-scaled distance band** (errands ×0.6 … leisure ×1.3 of the released distance). Per
   purpose, agents keep a small set of **favourite places** revisited with
   `ActivityPars.habitualDestinationProbability`. Without tags, choice is uniform — the census
   plays no role in destination selection.

## Data layers

A **single** optional GeoPackage, `<City>_censusData.gpkg`, carries the **population structure** as
columns (plus `vulnerability_pct` for the night module). It loads only if present; otherwise the
model falls back gracefully to DMA / uniform-random selection. The census never drives destination
choice — that is the POI/tag layers' job below.

| Column | Type | Purpose |
|---|---|---|
| `residence_pct` | share | residence-weighted home selection (0 for non-residential zones) |
| `residents` | count | absolute headcount; when present the population size is census-driven |
| `vulnerability_pct` | rate [0,1] | per-zone vulnerability (read by the night module) |
| `retiree_pct` | rate [0,1] | optional: share of adult residents 65+ — conditions the persona mix per home zone |
| `student_pct` | rate [0,1] | optional: share of adult residents 15–24 — conditions the persona mix per home zone |

The layer is produced by `pipeline/01_census_istat.py` (run via `build_census.bat`) — an
**ISTAT adapter for Italian cities**; other countries need a sibling adapter emitting the
same columns from their own raw census (the Java side is country-agnostic).
All zones are kept: non-residential zones (streets, parks, commercial) simply get `residence_pct = 0`.
Each zone claims the network nodes within 50 m (growing to 100/200/400 m if none); the
`vulnerability_pct` **rate** is broadcast unchanged to a zone's nodes.

A second optional GeoPackage, `<City>_POIs.gpkg`, carries point/polygon POIs with **OSM-like use
tags**; the buildings layer may carry the same tag columns. `PoiClassifier` reads, in order, the
value-bearing keys `amenity`, `use`, `fclass`, `type`, `building`, `landuse`, `land_use` and the
presence-only keys `shop`, `leisure`, `tourism`, `sport`, `office`, classifying each feature into
`ActivityPurpose`s and snapping it to its nearest network node (≤ `ActivityPars.poiClaimRadius`).
The recognised vocabulary covers both raw OSM values (`pub`, `supermarket`, …) and the cityImage
macro-groups the pipeline writes (`sustenance`, `healthcare`, `entertainment_arts_culture`, …).
Buildings also carry `land_uses` — the **full list** of macro-groups as a stringified Python list —
and `ActivityPurpose.classifyAll` classifies every label in it, so a mixed-use building attracts
each purpose it hosts, not only the first-listed one. The two layers weigh differently: a POI is a
**venue** (1.0 each), a building is **fabric** and weighs its footprint area in units of
`ActivityPars.buildingAreaPerAttractionUnit` (200 m² ≈ one venue, min 1.0) — an office block pulls
proportionally more WORK trips than a corner office. No tags → empty purpose maps → uniform choice.
The POI layer is produced by the `pois` stage of `pipeline/00_city_preparation.py` (run via
`build_city.bat`).

## For module authors

A new 24h-routine module should extend the activity tier, not core:

- state → `extends PedSimCityActivity`
- engine → `extends ActivityEngine`
- populate → `extends ActivityPopulate`
- agent → `extends ActivityAgent`

and add only its own layer on top. See `night` (vulnerability + lighting) and `learning`
(incremental cognitive map) for worked examples. Remember to clear your module's static data in the
engine's `clearStaticData()` (activity's is cleared for you by `ActivityEngine`).

## Running

Activity is both the shared foundation for the routine modules *and* a runnable plain-activity model
in its own right. It compiles in every module profile (it is never excluded) and is launched via
`PedSimCityActivityApplet`, mirroring night:

```bash
mvn compile exec:java@activity-website   # REST API + browser dashboard
mvn compile exec:java@activity           # GUI (PedSimCityActivityApplet)
mvn compile exec:java@activity -Dexec.args="--headless"   # headless
```

Start a run via REST once the server is up:

```bash
curl -X POST http://localhost:8081/api/start \
  -H "Content-Type: application/json" \
  -d '{"module":"activity","cityName":"Melbourne","days":7,"jobs":1}'
```

The GUI city options are **Melbourne** and **Torino**. With no census/POI datasets present for the
chosen city, the model degrades gracefully to uniform-random home/work and destination selection.
