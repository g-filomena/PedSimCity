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
| `scripts/build_city.bat` | `pipeline/00_city_preparation.py` | the base simulation layers from OSM: `<City>_nodes/_edges/_nodesDual/_edgesDual/_barriers/_buildings/_POIs/_sight_lines2D.gpkg` |
| `scripts/build_city_remote.bat` | the same, **on the server** (via SSH) | same outputs, written into the server checkout's `src/main/resources/<City>/` |
| `scripts/build_census.bat` | `pipeline/01_census_istat.py` | **ISTAT (Italian cities) only**: reads raw `<City>_censusData_raw.gpkg` and writes enriched `<City>_censusData.gpkg` |
| `scripts/build_lighting.bat` | `pipeline/build_lighting.py` | lighting from the lamp inventory (rich attributes or bare points) |
| `scripts/build_transit_layer.bat` | `scripts/build_transit_layer.py` | snaps a GTFS feed's stops to the city's street nodes → `transit_stops.gpkg` / `.csv` |
| `scripts/run_night_comparison.bat` | `scripts/run_day_night_comparison.py` | runs the night module twice on one city and diffs the trip diagnostics |

Or from a terminal:

```bash
python pipeline/00_city_preparation.py --city Torino --place "Torino, Italy" --epsg 3003
python pipeline/01_census_istat.py     --city Torino
python pipeline/build_lighting.py      --city Torino
```

**Comparing two photometric assumptions** takes `--falloff-law` and `--variant` on steps 2 and 3.
A `--variant` suffixes every file the pair writes and diverts the edges layer to `inputData/`, so a
comparison can never overwrite the layer the simulation reads. Both steps need the same law, since
step 2 applies the normalisation and step 3 the propagation:

```bash
python pipeline/02_street_lights_torino.py --city Torino --falloff-law lambertian --variant lambertian
python pipeline/03_street_lights.py        --city Torino --falloff-law lambertian --variant lambertian
```

`--no-pole-height` on step 2 does the same job for the one mounting-height constant that has no
source. Each step prints the `pct_unlit` and `mean_lux` distribution beside the physics that
produced it, which is what the comparison is read off.

## City preparation (step 0)

`00_city_preparation.py` is one parameterised, **resumable** pipeline built on the
cityImage API (installed from PyPI by `scripts/build_city.bat`). Stages — `network`, `districts`,
`elevation`, `barriers`, `pois`, `buildings`, `sightlines`, `landmarks` — are checkpointed under
`inputData/<City>/prep_staging/`; re-running resumes after the last completed stage
(`--force` recomputes; `--stages` selects a subset, e.g. `--stages sightlines,landmarks`).

`scripts/build_city.bat` offers a **stage-group menu** — everything / base layers only
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

`scripts/build_city_remote.bat` runs the *same* step-0 pipeline on the remote server instead of your
laptop — worth it for the sight-lines stage, which wants the server's cores and RAM. It reads
the SSH host / key / remote base directory from **`server.properties`** (the same file the Java
remote-run uses; copy `server.properties.example` and fill it in), then over SSH it clones the
checkout on first use (public repo, HTTPS — override the URL with a `server.repoUrl` key),
`git pull`s the latest, and runs the pipeline in the `pedsimcity-prep` conda environment
(created from `environment-prep.yml` on first use; cityImage installed from PyPI; on first use it
also accepts the Anaconda default-channel Terms of Service so `conda env create` can proceed —
packages still resolve from conda-forge). It prompts
for city / place / EPSG / stages / consolidation just like `scripts/build_city.bat`. The two helper
scripts are `pipeline/remote_prep.ps1` (client side) and `pipeline/run_prep_remote.sh` (server side).

**The run is detached and survives disconnection.** The pipeline is started under `setsid` with its
output redirected to a per-city log, so it is decoupled from the SSH connection: **hibernating the
laptop or dropping the link does not kill it** — the run keeps going on the server. The launcher
follows the log live; if you get disconnected, just **re-run `scripts/build_city_remote.bat` for the same
city and it reconnects** to the still-running job (detected by its recorded PID) and resumes
following. When the run finishes, the launcher reads its recorded exit code and — on success —
offers the download step. This matters most for the sight-lines stage, which runs for hours and,
if killed mid-way, restarts from scratch (earlier stages are checkpointed and skipped, but the
sight-lines stage itself has no sub-checkpoint).

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

**Running the lighting steps on the server** has no launcher of its own; the steps are invoked
directly in the same `pedsimcity-prep` environment, and three things bite:

```bash
export PATH=/mnt/home/gabriele/miniconda3/bin:$PATH        # .bashrc returns early for ssh <command>
export PROJ_DATA=/mnt/home/gabriele/miniconda3/envs/pedsimcity-prep/share/proj
PY=/mnt/home/gabriele/miniconda3/envs/pedsimcity-prep/bin/python
cd /mnt/home/gabriele/PedSimCity/pipeline
nohup $PY 02_street_lights_torino.py --city Torino > ~/runs/lighting/step2.log 2>&1
```

- **`PROJ_DATA` is not set by the env**, and without it GDAL prints
  `PROJ: proj_create_from_database: Open of .../share/proj failed` before every read.
- **The steps write into the server checkout**, so a run there and a run on the laptop produce two
  layers of the same name; `scp` the winner back rather than leaving both.
- **Never `pgrep -f` on a pattern that also matches your own ssh command line.** `pgrep -f
  04_directional` matches the `bash -c` wrapper running it, so a poll loop written that way reports
  the step as still running forever. Match the full invocation, or check the log.

## Census (ISTAT — Italian cities only)

| Script | Effect |
|---|---|
| `01_census_istat.py` | **ISTAT adapter**: reads raw `<City>_censusData_raw.gpkg` (Italian census sections with the `P*` variables), writes enriched `<City>_censusData.gpkg` with `residence_pct`, `residents`, `vulnerability_pct` (+ `retiree_pct`/`student_pct` when the P14–P29 age bands are present, conditioning the persona mix per zone) — population structure only; destination attraction comes from `<City>_POIs.gpkg` / the buildings tags |

The census is population structure only (`P1` population; `P3`/`P30–32`/`P43–45`
vulnerability). The **output schema is the country-neutral contract** the Java side reads —
supporting another country (e.g. the UK from ONS output areas) means writing a sibling
adapter (`01_census_uk.py`) that emits the same columns from that country's raw census;
the Java side needs no change.

The script runs standalone (no orchestrator): `scripts/build_census.bat` calls it directly. The
raw census `<City>_censusData_raw.gpkg` is left untouched, so re-running is safe.

## Street lighting

One pipeline (`build_lighting.py`); step 2 has two **adapters** and the orchestrator
picks the right one from the lamp inventory the city provides. Both adapters write the
same intermediate (`<City>_streetlights_with_radius.gpkg`), so steps 3–4 are shared and
adapter-agnostic.

| Step | Script | Effect |
|---|---|---|
| 2 (Turin schema) | `02_street_lights_torino.py` | reads `<City>_puntiLuce.gpkg` (Italian open-data `puntiLuce` schema: `potenza_w_max`, `altezza_palo_m`, `tecnologia`, `uso_ottica`, …) and derives real per-lamp physics, filling missing power by technology median and missing mounting height by support type (see below) |
| 2 (generic) | `02_street_lights_generic.py` | reads a **point-only** lamp file `<City>_streetlights.gpkg` (no technical attributes) and applies uniform assumed physics (power 100 W, height 9 m, efficacy 70 lm/W, DLOR 0.80) |
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

## Lighting: what is settled and what is not

**Done, and pending a re-run to take effect.** Four of the five findings of the September 2026
lighting audit are now in the code, and none of them changes a single lux value until
`03_street_lights.py` is re-run.

- **A building no longer occludes the lamp mounted on it.** A wall bracket or an arcade fixture
  whose point falls up to `ARCADE_DEPTH_M` (5 m) inside a footprint lights the street in front of
  it, while a lamp deeper inside the block stays occluded. On Torino that is **5,092 of the 5,873
  lamps inside a footprint**, which until now contributed nothing at all. The occlusion test is
  `crosses()` rather than `intersects()`, so a sight line grazing a corner no longer blocks.
- **Mounting height is imputed from the support, not from the optics.** `altezza_palo_m` is missing
  for 100% of `solo apparecchio in distinta` (31,435 lamps) and 100% of `non determinato` (6,661) -
  no pole, no pole height - against 11.7% of the lamps that do stand on one, so **45% of the
  inventory runs on an imputed height** and the old `uso_ottica` median mixed the two populations.
  `impute_mounting_height` now groups by `tipo_supporto`: a support type with some measured heights
  supplies its own median, and a support type with **none** has no pole to measure, so it takes
  `NO_POLE_HEIGHT_M`. Stated as a rule rather than as a list of Italian labels, so it carries to
  another city's inventory.
- **`NO_POLE_HEIGHT_M = 4.0` is an assumption and is labelled as one.** There is no source for it;
  what the audit establishes is only that it is **not** the ~9 m pole median, since an arcade or
  wall fixture hangs at roughly 4 m and height enters the illuminance law twice. Every lamp now
  carries an `altezza_source` column (`measured` / `support_median` / `no_pole_assumed` /
  `global_median` / `default`), so how much of a city's lighting rests on an assumption is counted
  in the output and printed by both step 2 and step 3 rather than being invisible.
- **A missing height now fails in step 3.** `03_street_lights.py` raises instead of filling 9.0,
  matching the guard it already applied to a missing `downward_intensity_cd`. Safe because step 2
  assigns every lamp a height above — its ladder still ends with `DEFAULT_HEIGHT_M` (9.0) rather
  than failing, so a thin inventory yields a lighting layer instead of nothing, and that last resort
  is counted in `altezza_source`, warned about by name, and reported again by step 3 before it
  computes anything. What made the old behaviour a defect was never the number: it was that the
  number was invisible and reached 45% of Turin's lamps through the wrong grouping.
- **`04_directional_lighting.py` reads 15 m ahead instead of 12 m**, per Fotios, Yang & Uttley
  (2015), whose 10.3 m is the measured fixation distance and 15 m the recommended observation
  distance.

**All of it took effect on 16 September 2026**, when the step was re-run over Torino. Against the
14 September layer, under the same `"mixed"` physics: mean `mean_lux` rose 22.88 -> 25.57, the
*median* per-edge ratio was 0.9993 - so the typical street did not move at all - and 618 edges
(1.4%) crossed the 5 lux service line. What moved are the arcade and wall-bracket lamps that had
been occluding themselves. The canonical layer was then rebuilt again under the two decisions
below.

### The falloff law: chosen by measurement, 16 September 2026 - `"isotropic"`

`lighting.py` used to derive intensity as `F/pi`, the on-axis value of a **Lambertian** emitter,
and then propagate it with the **isotropic** law `E = I h / D^3` - a Lambertian's peak intensity
applied in every direction. The pairing is now explicit and selectable, so a lux value always says
which physics produced it, and both step 2 and step 3 print `describe_law()` beside their numbers:

| `FALLOFF_LAW` | normalisation | propagation | summation radius, Torino inventory |
|---|---|---|---|
| `"mixed"` | `F / pi` | `E = I h / D^3` | 112.5 m |
| `"lambertian"` | `F / pi` | `E = I0 h^2 / D^4` | 67.7 m |
| `"isotropic"` **(in force)** | `F / (2 pi)` | `E = I h / D^3` | 88.7 m |

All three were run over the same 44,278 Torino edges, same lamps, same buildings:

| law | `pct_unlit` mean / median | fully unlit | fully lit | `mean_lux` | edges under 5 lux |
|---|---|---|---|---|---|
| mixed | 11.1 / 0.0 | 7.1% | 81.6% | 25.57 | 9.4% |
| isotropic | 24.3 / 0.0 | 11.4% | 56.3% | 12.72 | 18.5% |
| lambertian | 29.3 / 12.1 | 13.1% | 44.5% | 14.80 | 20.5% |

Two things the table says that an argument would not have.

- **`isotropic` is `mixed` with the honest divisor and nothing else.** Same propagation, so the same
  spatial pattern; `mean_lux` exactly halved. The difference between them is a *level*, not a shape
  - and the level is not the falloff law's to set.
- **`lambertian` is a different shape, and the wrong one here.** It has the higher `mean_lux` of the
  two consistent laws and still the worse `pct_unlit`, because `cos(theta)` piles light under the
  pole and takes it from the mid-span between two poles. That mid-span is what `min_lux` and
  `pct_unlit` exist to measure, and a cobra-head's whole design is lateral throw, so Lambertian is
  backwards for this inventory.

`"mixed"` was also doing something the night module cannot want: under it **81.6% of Turin's streets
contain no unlit sample point at all**, which very nearly deletes the module's subject.

**None of the three is correct.** A per-luminaire IES/LDT photometric file is what would settle it,
and the same file would settle DLOR below; both wait on the same thing.

### DLOR replaced the utilisation factor, 16 September 2026

The middle factor in `I_down = lamp_lumens * X / normalisation` is the share of **lamp** lumens the
**luminaire** emits downward - the downward light output ratio, a quantity EN 13032-1 defines by
measuring the luminaire under standardised conditions. What used to be there was a **utilisation
factor**, 0.3-0.6 by free-text `uso_ottica` label: utilance is the share of luminaire flux landing
on the *carriageway*, and it depends on mounting height, road width and overhang. The propagation
law already computes how much light reaches a point, so a utilance multiplied in as well charges
the same geometry twice.

Two facts bound DLOR for Turin where nothing bounded the utilisation factor:

- **Upward flux is regulated to nothing.** Regione Piemonte L.R. 31/2000 Allegato A punto 1(a), as
  amended by L.R. 3/2018, caps intensity at 0-0.49 cd per 1000 lm for gamma >= 90 degrees. ULOR is
  therefore ~0 for any compliant installation, and DLOR ~ LOR.
- **A LED luminaire's photometry is the luminaire's.** Its rated efficacy is already a luminaire
  efficacy (the map gives LED 120 lm/W), so LOR is 1.0 by convention. A discharge lamp's 90 lm/W is
  the bare lamp's, and the optic around it is what costs.

| technology | optic | DLOR | Torino |
|---|---|---|---|
| LED | road | 1.00 | 59,742 (59.9%) |
| LED | decorative lantern | 0.85 | 117 (0.1%) |
| discharge / undetermined | road | 0.80 | 31,058 (31.1%) |
| discharge / undetermined | decorative lantern | 0.55 | 8,825 (8.8%) |

Mean 0.898, against the old mean utilisation factor of 0.470. **The four values are still
engineering judgement** and step 2 says so; what changed is that they are judgements about a defined
quantity with a regulatory bound on one side, rather than numbers attached to a label.

**The two changes very nearly cancel, and that is the finding.** Isotropic halves the level, DLOR
multiplies it by 1.91, and the canonical layer lands on top of the one they replaced:

| Torino layer | `mean_lux` | `pct_unlit` mean | fully lit | edges under 5 lux |
|---|---|---|---|---|
| 14 Sep, mixed + utilisation factor, pre-fix | 22.88 | 12.05 | 80.8% | 10.2% |
| 16 Sep, mixed + utilisation factor | 25.57 | 11.09 | 81.6% | 9.4% |
| **16 Sep, isotropic + DLOR (canonical)** | **24.84** | **11.35** | **80.4%** | **9.6%** |

So the old level was roughly right for two wrong reasons pointing in opposite directions, and it is
now the same level for reasons that can be checked one at a time. Nothing in `NightPars` has to move
with it: `darkSpotLuxThreshold` (5 lux) and the drawn `lightSensitivityThreshold` still sit in the
same place relative to the city.

### How much rests on `NO_POLE_HEIGHT_M`: measured, 16 September 2026

It has no source (see **Still open** below), so what there is instead is a bound. Same everything
else, the whole pipeline re-run at three values of the height given to the 38.2% of Turin's lamps
whose support type has no measured height anywhere:

| no-pole height | `pct_unlit` mean | fully lit | `mean_lux` | edges under 5 lux | `min_lux` mean |
|---|---|---|---|---|---|
| 3.0 m | 11.93 | 79.2% | 25.40 | 9.8% | 14.97 |
| **4.0 m (in force)** | **11.35** | **80.4%** | **24.84** | **9.6%** | **15.66** |
| 6.0 m | 10.79 | 81.9% | 23.85 | 9.3% | 16.35 |

**Doubling the height of 38% of the inventory moves the share of edges below the service level by
half a percentage point.** The reason is in the law rather than in the data: a higher lamp has a
lower peak directly beneath it (`E = I/h^2` at `d = 0`) and a wider spread, so `mean_lux` falls while
`min_lux` rises and the two effects on "is this point lit" very nearly cancel on a dense urban
network where several lamps are in range of every point. The constant is worth a source; it is not
worth blocking a run for.

### Still open

1. **The arcade depth is a judgement, not a measurement.** 5 m is where Turin's portico depth and
   the observed distribution meet (median 1.6 m inside, 87% within 5 m, 316 lamps deeper than 10 m).
   A different city needs a different number, and no data here says what it should be.
2. **`NO_POLE_HEIGHT_M` still wants a source, but it is now bounded.** 4 m is an order of magnitude
   for a wall or arcade fixture, and it sets the height of 38.2% of Turin's inventory. The sweep
   above says the aggregate barely notices between 3 m and 6 m. **What was checked on 16 September
   2026, so nobody checks it twice:** EN 13201 and UNI 11248 classify roads and set maintained
   illuminance and prescribe no mounting height; CEI 64-8/7 section 714 (replacing CEI 64-7) gives
   a floor rather than a value — a luminaire whose lamp is reachable without a tool must be above
   2.8 m; Regione Piemonte L.R. 31/2000 Allegato A punto 1(d) constrains the spacing-to-height
   *ratio* (> 3.7) and so fixes neither term alone. **The one source that plausibly settles it and
   has not been read is Turin's own PRIC**, whose "fascicolo completo apparecchi" is a 79.9 MB PDF
   on `comune.torino.it`. The inventory cannot answer it: all three support types with no measured
   height have *no* measured height at all, so there is nothing to impute from.
   `DEFAULT_HEIGHT_M` (9.0) wants a source no less, but it is the last rung and should reach
   nothing on a real inventory — if `altezza_source` ever reports `default` on a city you care
   about, that city's step-2 output is the thing to fix, not this constant.
3. ~~**`utilization_factor` has no source.**~~ **Replaced by DLOR, 16 September 2026** — see above.
   The remaining judgement is the four per-class values, which is a smaller and better-posed
   question than the one it replaced: a per-luminaire IES/LDT file answers it, and answers the
   falloff law at the same time.
4. ~~**The occlusion loop is why nobody re-runs this.**~~ **Done 16 September 2026.** Step 3 is
   bulk array work now: one vectorised `line_interpolate_point` for the 1.08M sample points, one
   KD-tree query per block of points, and one bulk `STRtree` `crosses` query per block of sight
   lines, in place of a `LineString` constructed and index-queried per point-lamp pair. **The answer
   is unchanged** - `verify_step3.py` reruns the old per-pair loop beside it over a random slice of
   the real city and compares element by element; the worst absolute difference over 59,643 points
   was 4.5e-12 lux, which is summation order. A full Torino run is **about 10 minutes**, so three
   falloff laws and two mounting heights were run in one afternoon instead of none being run at all.

The same list, with the derivations, is under **Cross-cutting** in the root [`TODO.md`](../TODO.md).
