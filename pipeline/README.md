# Data pipeline

Turns raw city inputs into the GeoPackages / CSVs the Java simulation reads.

**Folders** (shared conventions in `pipeline/paths.py`): raw, user-supplied material
lives in **`inputData/<City>/`**; everything the simulation loads is written to
**`src/main/resources/<City>/`**; intermediates (step outputs consumed only by later
steps, staging checkpoints, `prep_config.json`) stay in `inputData/<City>/`, so the
resources folder holds nothing but simulation inputs. Scripts resolve inputs by
searching `inputData/<City>/` first, then the resources folder.

**Naming: every file is `<City>_…`**, where `<City>` = the folder name under both
`inputData/` and `src/main/resources/` = `Pars.cityName`. Every script and launcher
only ever asks for the city (`--city`).

## Running

Double-clickable launchers at the repo root (Windows), each **prompts for the city**:

| Launcher | Runs | Produces |
|---|---|---|
| `build_city.bat` | `pipeline/00_city_preparation.py` | the base simulation layers from OSM: `<City>_nodes/_edges/_nodesDual/_edgesDual/_barriers/_buildings/_POIs/_sight_lines2D.gpkg` |
| `build_city_remote.bat` | the same, **on the server** (via SSH) | same outputs, written into the server checkout's `src/main/resources/<City>/` |
| `build_census.bat` | `pipeline/01_census_istat.py` | **ISTAT (Italian cities) only**: reads raw `<City>_censusData_raw.gpkg` and writes enriched `<City>_censusData.gpkg` |
| `build_lighting.bat` | `pipeline/build_lighting.py` | lighting from the lamp inventory (rich attributes or bare points) |

Or from a terminal:

```bash
python pipeline/00_city_preparation.py --city Torino --place "Torino, Italy" --epsg 3003
python pipeline/01_census_istat.py     --city Torino
python pipeline/build_lighting.py      --city Torino
```

## City preparation (step 0)

`00_city_preparation.py` is one parameterised, **resumable** pipeline built on the
cityImage API (installed from PyPI by `build_city.bat`). Stages — `network`, `districts`,
`elevation`, `barriers`, `pois`, `buildings`, `sightlines`, `landmarks` — are checkpointed under
`inputData/<City>/prep_staging/`; re-running resumes after the last completed stage
(`--force` recomputes; `--stages` selects a subset, e.g. `--stages sightlines,landmarks`).

`build_city.bat` offers a **stage-group menu** — everything / base layers only
(network, districts, barriers, pois: fast, inspect in QGIS) / landmarks only (buildings,
sightlines, landmarks: heavy, run overnight) / custom list — and asks for the OSM place
and EPSG **only on the first run** for a city: they are saved to
`inputData/<City>/prep_config.json` and reused, so stage re-runs cannot mismatch them.

Optional raw inputs in `inputData/<City>/` (also found in the resources folder),
projected in the city CRS:

- `<City>_officialBuildings.gpkg` (legacy name `<City>_detailedBuildings.gpkg`, still
  read) — an **official** building dataset. When present its geometries become the
  **obstructions** (authoritative footprints, replacing the OSM ones), and its own
  `height`/`base` columns are used as-is. Land use / DMA are still borrowed from OSM by
  largest overlap (cityImage's land-use classifier is OSM-vocabulary-specific).
- `<City>_studyArea.gpkg` — an optional polygon delimiting the **analysed** buildings
  (those scored and shipped as `<City>_buildings.gpkg`). Their scores are still computed
  against the full obstructions, so boundary buildings keep a complete neighbourhood.
  Without it, `--analysis-radius <m>` keeps the buildings within that distance of the
  obstructions' union centroid; with neither, every obstruction is analysed.
- `<City>_buildingHeights.gpkg` — a dedicated **height** layer (polygons with a `height`
  field, `base` optional). Heights are joined onto the footprints by largest overlap
  (`assign_building_heights_from_other_gdf`). Use this to attach heights to OSM footprints
  without a full official dataset.
- `<City>_DTM.tif` (bare-earth **terrain**) — gives node `z` (elevation stage) and
  building `base` (ground under the footprint).
- `<City>_DEM.tif` / `<City>_DSM.tif` (first-return **surface**, rooftops included —
  "DEM" is accepted because surface models are often shipped under that generic name) —
  together with the DTM, derives above-ground building heights as surface − terrain
  (`ci.assign_elevations_from_rasters`) when no earlier source provided them. A DTM
  alone cannot give heights — only `z` and `base`.

The height cascade is official layer's own height/base → `buildingHeights.gpkg` overlap →
DEM/DSM−DTM. **OSM `height`/`building:levels` tags are never used.** Without any height
source the sight-lines stage is skipped with a warning and the sim runs without
3D-visibility landmark navigation. Without a DTM, node `z` and building `base` stay at
ground 0 (flat city assumption — fine for Turin's centre, wrong for hilly cities).

Output filenames and columns follow the Java readers exactly: `_sight_lines2D`,
`_nodesDual`/`_edgesDual`, barrier kind in a `type` column, `district`/`gateway` ints on
nodes, `deg` on dual edges.

The buildings layer is `<City>_buildings.gpkg`: the **analysed** footprints, carrying
scalar `land_use` + `DMA` and the landmark scores `gScore_sc`/`lScore_sc` as optional
columns. The wider context set is written alongside as `<City>_obstructions.gpkg`. That
obstructions layer is also cached in `inputData/<City>/` the first time it is built and
**reused as-is** on later runs (skipping the OSM/official regeneration); delete it to
force a rebuild.
There is no separate landmarks file — landmarks are the runtime subset of buildings whose
scores pass the `RouteChoicePars` thresholds.

The `pois` stage writes `<City>_POIs.gpkg`: one point per OSM feature tagged with the
use vocabulary the activity module classifies (`amenity`, `shop`, `leisure`, `tourism`,
`sport`, `office`); `PoiClassifier` turns these (plus tagged buildings) into per-node
attraction weights for purpose-aware destination choice.

It needs the heavier `pedsimcity-prep` Conda environment (`environment-prep.yml`):
igraph (centrality), python-louvain (districts), dask (parallel 2D obstruction check).
The sight-lines stage is the most expensive one, but two changes keep it tractable:

- **Distance cap.** It only considers observer→target pairs within
  `--max-sightline-distance` metres (default **2000**); longer sight lines are dominated
  by rare, mostly-obstructed lines that accounted for the bulk of the old runtime, so the
  cap gives a large speed-up with negligible effect on the visibility scores. Pass
  `--max-sightline-distance 0` to disable the cap and consider all pairs (much slower).
- **Closed-form 3D visibility.** Because buildings are vertical extrusions (flat roofs),
  occlusion is computed analytically — a sight line is blocked when its plan projection
  crosses a footprint and its height dips to/below that roof — instead of ray-tracing
  triangulated meshes. This is exact for such buildings and much faster, and it means the
  stage no longer needs `pyvista`/VTK.

The stage prints per-step wall times and a progress bar as it runs. On a large dense city
(e.g. Barcelona) expect it to run for a few hours; it is checkpointed, so it resumes.
When it finishes, the temporary `sight_lines_tmp/` chunk folder (written by cityImage's
`compute_3d_sight_lines`) is deleted automatically.

### Running the preparation on the server

`build_city_remote.bat` runs the *same* step-0 pipeline on the remote server instead of your
laptop — worth it for the sight-lines stage, which wants the server's cores and RAM. It reads
the SSH host / key / remote base directory from **`server.properties`** (the same file the Java
remote-run uses; copy `server.properties.example` and fill it in), then over SSH it clones the
checkout on first use (public repo, HTTPS — override the URL with a `server.repoUrl` key),
`git pull`s the latest, and runs the pipeline in the `pedsimcity-prep` conda environment
(created from `environment-prep.yml` on first use; cityImage installed from PyPI; on first use it
also accepts the Anaconda default-channel Terms of Service so `conda env create` can proceed —
packages still resolve from conda-forge). It prompts
for city / place / EPSG / stages / consolidation just like `build_city.bat`, and streams the
log back live. The two helper scripts are `pipeline/remote_prep.ps1` (client side) and
`pipeline/run_prep_remote.sh` (server side).

Because the pipeline writes into the server checkout's `src/main/resources/<City>/`, the
outputs are immediately usable by the Java simulation running on that same server — no transfer
back. If instead you want them locally (to run the sim on your laptop or inspect the layers in
QGIS), the launcher offers a final step that `scp`s the produced `src/main/resources/<City>/`
back into your local checkout; say no to it when the sim runs on the server. Two prerequisites:

- **The server pulls from git**, so any code change (this launcher included) must be committed
  and pushed before the remote run can pick it up.
- **Inputs are uploaded automatically, curated and incrementally.** Before running, the launcher
  `scp`s only the files the pipeline reads — the `*_DTM/_DEM/_DSM` rasters, the
  `officialBuildings` / `detailedBuildings` / `buildingHeights` / `studyArea` `.gpkg` layers, the
  reused `_obstructions.gpkg` cache, and `prep_config.json` — skipping QGIS projects, PDFs, raw
  source layers and `prep_staging/` checkpoints. Files already on the server with a matching size
  are skipped, so re-runs send only what changed. It lists what it will upload and asks first; an
  OSM-only city (no such files) skips the step. Because inputs travel over scp, not git, the
  server needs no `git-lfs`. (Uploading `_obstructions.gpkg` makes the server *reuse* obstructions
  instead of rebuilding them — delete that file locally or on the server to force a fresh build.)

The `build_lighting*` scripts are thin **orchestrators**: they run the step scripts in
order, stop on the first failure, and skip a step whose output already exists unless
`--force` is given.

## Census (ISTAT — Italian cities only)

| Script | Effect |
|---|---|
| `01_census_istat.py` | **ISTAT adapter**: reads raw `<City>_censusData_raw.gpkg` (Italian census sections with the `P*` variables), writes enriched `<City>_censusData.gpkg` with `residence_pct`, `residents`, `vulnerability_pct` (+ `retiree_pct`/`student_pct` when the P14–P29 age bands are present, conditioning the persona mix per zone) — population structure only; destination attraction comes from `<City>_POIs.gpkg` / the buildings tags |

The census is population structure only (`P1` population; `P3`/`P30–32`/`P43–45`
vulnerability). The **output schema is the country-neutral contract** the Java side reads —
supporting another country (e.g. the UK from ONS output areas) means writing a sibling
adapter (`01_census_uk.py`) that emits the same columns from that country's raw census;
the Java side needs no change.

The script runs standalone (no orchestrator): `build_census.bat` calls it directly. The
raw census `<City>_censusData_raw.gpkg` is left untouched, so re-running is safe.

## Street lighting

One pipeline (`build_lighting.py`); step 2 has two **adapters** and the orchestrator
picks the right one from the lamp inventory the city provides. Both adapters write the
same intermediate (`<City>_streetlights_with_radius.gpkg`), so steps 3–4 are shared and
adapter-agnostic.

| Step | Script | Effect |
|---|---|---|
| 2 (Turin schema) | `02_street_lights_torino.py` | reads `<City>_puntiLuce.gpkg` (Italian open-data `puntiLuce` schema: `potenza_w_max`, `altezza_palo_m`, `tecnologia`, `uso_ottica`, …) and derives real per-lamp physics, filling gaps by technology/optics-group medians |
| 2 (generic) | `02_street_lights_generic.py` | reads a **point-only** lamp file `<City>_streetlights.gpkg` (no technical attributes) and applies uniform assumed physics (power 100 W, height 9 m, efficacy 70 lm/W, util 0.4) |
| 3 | `03_street_lights.py` | → `<City>_edges_illuminated_continuous.gpkg` (`mean_lux`, to resources) + `<City>_nodes_2m_densified_illuminated.gpkg` (intermediate, to `inputData/`) |
| 4 | `04_directional_lighting.py` | → `<City>_directional_lighting_lookup.csv` (to resources) |

The raw lamp file is read from `inputData/<City>/`; the step-2 intermediate is written
back there. Only the step-3/4 outputs the sim reads land in resources. The generic
defaults live as constants at the top of `02_street_lights_generic.py`.

## Utilities

| Script | Effect |
|---|---|
| `active_frontages.py` | writes `<City>_edges_with_frontages.gpkg` to `inputData/<City>/` — **optional**, not yet read by the sim; run directly |

## Filename contract

Outputs the sim loads (`<City>_edges_illuminated_continuous.gpkg`, `<City>_censusData.gpkg`,
`<City>_directional_lighting_lookup.csv`) must match the Java readers, which use
`Pars.cityName`. Get the city name wrong and the sim silently falls back (e.g. `mean_lux`
defaults to 0).
