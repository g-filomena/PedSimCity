# Activity module (`pedsim.activity`)

The **activity-based foundation** for modules that run a full 24-hour daily routine. Activity sits
between `core` (pure infrastructure) and the routine modules (`night`, `learning`), adding the things
those modules share: census-driven home/work population, a workplace/evening **activity POI** pattern,
and a 24-hour **day/night clock**.

Activity is not census/lighting-agnostic like core, but it is *neutral* about perception and safety:
it has no vulnerability and no street-lighting behaviour. A plain activity run is the simplest
activity model — people live in residence-weighted census zones, commute to workplace-weighted
destinations by day, and head to night-POI destinations after dark.

## Relationship to core

Activity specialises each of core's four seams and adds an engine of its own:

| Concern | Core class | Activity class |
|---|---|---|
| Simulation state + activity data | `PedSimCity` | `PedSimCityActivity` (census, workplace/night POI, `isDark`) |
| Engine wiring + 24h clock | `Engine` | `ActivityEngine` (activity import/environment, `onStepUpdate` sets `isDark`) |
| Data import | `Import` | `ActivityImport` (census + workplace + night POI GPKG) |
| Environment preparation | `Environment` | `ActivityEnvironment` (census-zone spatial join + POI-density joins) |
| Population | `Populate` | `ActivityPopulate` (residence-weighted home, workplace-weighted work) |
| Agent | `Agent` | `ActivityAgent` (time-of-day POI weighting via `getPOIWeight` + `isDark()`) |

## The 24-hour activity pattern

1. **Clock** — `ActivityEngine.onStepUpdate` sets `PedSimCityActivity.isDark` to `true` between
   ~20:00 and ~06:00 (from `TimePars`). Every activity-based state inherits this field.
2. **Population** — `ActivityPopulate` draws each agent's **home** from residence-weighted census
   zones and its **work** from nearby zones weighted by **workplace POI** density; it degrades
   gracefully to core's DMA → uniform-random fallback when census data is absent.
3. **Destinations** — `ActivityAgent.getPOIWeight` weights candidate destinations by **workplace POI**
   during the day and by **night POI** when `isDark`. `Agent.isDark()` (overridden here) also stops
   agents from targeting their `workNode` after dark.

## Data layers

A **single** optional GeoPackage, `<City>_censusData.gpkg`, carries the whole 24h pattern as columns
(plus `vulnerability_pct` for the night module). It loads only if present; otherwise the model falls
back gracefully to DMA / uniform-random selection.

| Column | Type | Purpose |
|---|---|---|
| `residence_pct` | share | residence-weighted home selection (0 for non-residential zones) |
| `workplace_poi` | count | daytime work destinations |
| `night_poi` | count | evening/leisure destinations |
| `vulnerability_pct` | rate [0,1] | per-zone vulnerability (read by the night module) |

The layer is produced by `pipeline/01_census_and_poi.py` (run via `pipeline/build_city.py`).
All zones are kept: non-residential zones (streets, parks, commercial) simply get `residence_pct = 0`
and remain valid work/night destinations — no type-1/36 fusion. Each zone claims the network nodes
within 50 m (growing to 100/200/400 m if none); POI **counts** are split across a zone's nodes, while
the `vulnerability_pct` **rate** is broadcast unchanged.

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
