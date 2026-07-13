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
cityImage API (installed by `build_city.bat` from the sibling `../cityImage`
checkout when present, falling back to PyPI). Stages — `network`, `districts`,
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

- `<City>_detailedBuildings.gpkg` — footprints with a `height` (and optionally `base`)
  column; first choice for building heights (`assign_building_heights_from_other_gdf`).
- `<City>_DTM.tif` (bare-earth **terrain**) — gives node `z` (elevation stage) and
  building `base` (ground under the footprint).
- `<City>_DEM.tif` / `<City>_DSM.tif` (first-return **surface**, rooftops included —
  "DEM" is accepted because surface models are often shipped under that generic name) —
  together with the DTM, derives above-ground building heights as surface − terrain
  (`ci.assign_elevations_from_rasters`) when no detailed layer provides them. A DTM
  alone cannot give heights — only `z` and `base`.

The height cascade is detailed layer → DEM/DSM−DTM → OSM tags; without any height source
the sight-lines stage is skipped with a warning and the sim runs without 3D-visibility
landmark navigation. Without a DTM, node `z` and building `base` stay at ground 0 (flat
city assumption — fine for Turin's centre, wrong for hilly cities).

Output filenames and columns follow the Java readers exactly: `_sight_lines2D`,
`_nodesDual`/`_edgesDual`, barrier kind in a `type` column, `district`/`gateway` ints on
nodes, `deg` on dual edges.

The buildings layer is `<City>_buildings.gpkg`: **all** footprints, carrying scalar
`land_use` + `DMA` and the landmark scores `gScore_sc`/`lScore_sc` as optional columns.
There is no separate landmarks file — landmarks are the runtime subset of buildings whose
scores pass the `RouteChoicePars` thresholds.

The `pois` stage writes `<City>_POIs.gpkg`: one point per OSM feature tagged with the
use vocabulary the activity module classifies (`amenity`, `shop`, `leisure`, `tourism`,
`sport`, `office`); `PoiClassifier` turns these (plus tagged buildings) into per-node
attraction weights for purpose-aware destination choice.

It needs the heavier `pedsimcity-prep` Conda environment (`environment-prep.yml`):
igraph (centrality), python-louvain (districts), pyvista/dask (3D sight lines).
The sight-lines stage is the expensive one (can take hours on a large city).

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
