# Changelog

Notable changes to PedSimCity over the recent development cycle (March–July 2026),
most recent first. Emphasis is on bug fixes and the changes that affect how
simulations are built, run, and visualised.

_Scope note: the learning module and the activity **persona/realism** work
(weather, personas, lunch peaks, purpose-based distances, persona release) are
intentionally omitted here and will be logged separately. Commit hashes are given
for traceability._

---

## July 2026 — latest

### Fixed
- **Spatial indexing & trajectory recording** — corrected indexing/recording
  issues that produced wrong trajectories and lookups (`daf61ab`).
- **Optional dual graph & landmark data** — import now degrades gracefully when a
  city ships no dual graph or no landmark scores instead of failing (`da6a161`).
- **A/B light-testing visuals** — restored day/night sky colour, fixed lux-metric
  IDs, trail fade-out, camera-follow panning math and auto-cancel on manual
  zoom/pan, tether rendering, and neutral road colouring when lights are off
  (`bc97fc7`, `c95e331`, `7bcadcd`, `edfb26f`).
- **HTML export** — fixed a Java string-literal size-limit compilation error in
  `HtmlExporter` (`033e275`) and a template syntax error that broke the map
  canvas (`3fc6e6e`).
- **Dashboard** — removed the stale weather control and fixed blank results pages
  caused by a missing `<style>` tag (`a0ea093`).

### Changed / Performance
- **Dijkstra pathfinding** — performance improvements and RNG cleanup (`ba8ac99`).
- **Time model** — day-of-week traffic patterns added; simulation step reduced to
  5 minutes (`d519901`).
- **Per-run GIS loading** — GIS preload deferred to each run; day/night hours made
  configurable (`4938c4c`).
- **Census-driven population** — population sourcing wired through the modules
  (`895200c`).
- **A/B testing support** and a refactor of agent movement creation (`82f7364`).

### Data & build
- **`inputData/` reorganisation** — raw source data consolidated under
  `inputData/` (`2859e8e`).
- **Pipeline refactor** — added step 0, centralised paths, consolidated the
  city/census adapters; new city-preparation launcher and refactored Windows
  build scripts (`e055461`, `0442c48`, `503895c`, `db74723`).
- **Census ISTAT alignment** — the raw census layer now keeps the original ISTAT
  field names (`P*`, `SEZ21_ID`, `COD_TIPO_S`, …); the ISTAT→friendly translation
  lives in the `<City>_census_metaData.xlsx` workbook (an `EnglishFieldName` column
  on both sheets), and `01_census_istat.py` reads the raw by its ISTAT names and
  applies that mapping — so the enriched census the sim loads carries friendly names
  only, with no ISTAT codes leaking through.

### Visualisation & publishing
- **Results site on Cloudflare Pages** — `publish_site.py` stages the self-contained
  result pages into a per-city static site (a PedSimCity overview plus one `/<City>`
  sub-page each) and deploys it to Cloudflare Pages, live at
  [pedsimcity.inclusivestreets.org](https://pedsimcity.inclusivestreets.org). It
  rebuilds the staging folder each run (so removed runs drop off the site), works
  before the first run (empty-state landing), and keeps model content off the
  `inclusivestreets.org` apex.

---

## June 2026 — module restructure & night simulation

The largest month of the cycle: the codebase was split into a reusable **core**
plus runnable **modules**, the night-lighting simulation matured, and the
visualisation stack was rebuilt.

### Added / Changed (architecture)
- **Module architecture** — core decoupled from activity/night datasets; added a
  `SimulationModule` abstraction, a REST API, and a `SimulationLauncher`; core
  clearly separated from runnable simulations, with extensibility hooks and
  `NightEngine` overrides (`c0b2dea`, `20b1205`, `cbd473d`, `6bee7ac`, `da6a161`).
- **Night module** — hourly scenarios and night detection, per-edge measured lux
  and counts, directional entrance-lighting, and an illuminated-edges → graph
  join; lighting logic and lux tracking refined throughout.
- **Data pipeline** — restructured with dedicated lighting steps, Windows build
  scripts, and per-module READMEs.
- **Dashboard** — full redesign to a light "SaaS" theme with summary tabs; the
  laggy Streamlit dashboard was replaced by a fast Leaflet HTML dashboard served
  over HTTP.

### Fixed (critical)
- **OOM / memory leak** — route caches bounded with an LRU policy to stop
  out-of-memory crashes (`4ff3699`).
- **Thread-safety** — fixed static-state races from concurrent runs, ensured
  `AgentReleaseManager` is closed in a `finally`, and synchronised environment
  initialisation (`20b1205`, `e8419d4`, `9ad733e`, `e5e0afb`).
- **NullPointerExceptions** — `buildResidenceProbabilities` (`7fcc4fb`),
  `AgentMovement.edgesToAvoid` during night rerouting (`813af96`), `Agent`
  constructor light-sensitivity init (`da4504b`), and `Environment` (`28eaae3`).
- **Dijkstra / routing** — NPEs, out-of-bounds in `cleanDualPath` for short
  sequences (`676f2c5`), compilation mismatch on returned barriers type
  (`685f4b1`), night trip-skipping, and original-route-aware bypass rerouting.
- **Agents** — corrected origin placement and destination selection (`7a45519`),
  release logic (`c97790b`), and day/night travel durations plus stuck
  visualisation markers (`4a39d60`); a batch of "4 critical simulation bugs"
  (`61b88dd`).
- **Dashboard accuracy** — zero-volume bug fixed by snapshotting `volumesMap`
  before clearance (`e53e27e`), agent colouring and live-lux display corrected
  (`793a6ac`), road-layer overlap ordering (`edf1e90`), and constant road
  thickness to avoid distortion (`3653de6`).
- **Live telemetry** — served via HTTP with CORS and load-time polling
  (`75a8f5a`).
- **GUI parameters** — days/population/% now actually applied on Run (`e843c12`,
  `ae40df6`).
- **Torino paths** — city name corrected to `Torino` and illuminated-edges path
  moved to the standard `cityName` prefix (`c185d9e`); Torino GIS restored to Git
  LFS with a pre-commit guard (`170c21c`).

---

## March–May 2026 — earlier stabilisation

Groundwork that made the Torino night runs usable and the engine
concurrency-safe.

### Fixed
- **Landmark navigation** — initialise heuristics for safe access and skip local
  landmark logic when a city (e.g. Torino) has no landmark setup, which had been
  crashing navigation (`474f9dc`, `334f9b9`).
- **Census & centroids** — hardened census loading and made centroid handling
  null-safe (`5684b42`, `edc5367`).
- **Concurrency** — fixed static state being overwritten by concurrent runs and
  added Engine/Environment synchronisation; default city set to `TorinoCentre`
  (`9041d85`, `300196c`, `7fd2097`).
- **Building data guard** — prevented simulation freezes by skipping DMA
  building-location searches when no building layer is loaded (`e9e63e7`).
- **Input data** — fixed data problems, zone/node overlaps, and multi-layer input
  loading errors (`3db1bb2`); resolved layer issues (`7f9465a`).
- **Crashes** — fixed a critical crash on empty routes (`4a16827`) and an NPE in
  Dijkstra (`ae00a97`).
- **Applets** — fixed parameter mapping and compilation errors across the applets;
  preloaded map data at startup for immediate rendering.

### Added
- Night applet runnable for Torino (`629fd3b`, `#5`); REST API + dashboard
  groundwork and map preloading.
