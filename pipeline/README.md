# Data pipeline

Turns raw city inputs into the GeoPackages / CSVs the Java simulation reads.

**Naming: every file is `<City>_…`**, where `<City>` = the resources folder name =
`Pars.cityName`. Raw inputs, intermediates, and sim-read outputs all share that prefix,
so the launchers only ever ask for the city (the folder under `src/main/resources/`).

## Running

Double-clickable launchers at the repo root (Windows), each **prompts for the city**:

| Launcher | Runs | Produces |
|---|---|---|
| `build_census.bat` | `pipeline/01_census_and_poi.py` | reads raw `<City>_censusData_raw.gpkg` and writes enriched `<City>_censusData.gpkg` |
| `build_lighting_torino.bat` | `pipeline/build_lighting_torino.py` | lighting from a Turin `puntiLuce` inventory |
| `build_lighting.bat` | `pipeline/build_lighting.py` | lighting from bare lamp-point locations |

Or from a terminal:

```bash
python pipeline/01_census_and_poi.py     --input_dir src/main/resources/Torino
python pipeline/build_lighting_torino.py --input_dir src/main/resources/Torino
python pipeline/build_lighting.py        --input_dir src/main/resources/Manchester
```

The `build_lighting*` scripts are thin **orchestrators**: they run the step scripts in
order, stop on the first failure, and skip a step whose output already exists unless
`--force` is given.

## Census / POI

| Script | Effect |
|---|---|
| `01_census_and_poi.py` | reads raw `<City>_censusData_raw.gpkg`, writes enriched `<City>_censusData.gpkg` with `residence_pct`, `workplace_poi`, `night_poi`, `vulnerability_pct` |
| `active_frontages.py` | writes `<City>_edges_with_frontages.gpkg` — **optional**, not yet read by the sim; run directly |

These run standalone (no orchestrator): `build_census.bat` calls `01_census_and_poi.py`
directly. The raw census `<City>_censusData_raw.gpkg` is left untouched, so re-running is safe.

## Street lighting — two variants

Only **step 2** differs between cities; steps 3–4 are shared and city-agnostic.

| Step | Turin (`build_lighting_torino.py`) | Generic (`build_lighting.py`) |
|---|---|---|
| 2 | `02_street_lights_torino.py` — reads `<City>_puntiLuce.gpkg` (Italian `puntiLuce` schema: `potenza_w_max`, `altezza_palo_m`, `tecnologia`, `uso_ottica`, …) and derives real per-lamp physics → `<City>_puntiLuce_with_radius.gpkg` | `02_street_lights_generic.py` — reads a **point-only** lamp file `<City>_streetlights.gpkg` (no technical attributes) and applies uniform assumed physics (power 100 W, height 9 m, efficacy 70 lm/W, util 0.4) → `<City>_streetlights_with_radius.gpkg` |
| 3 | `03_street_lights.py` → `<City>_edges_illuminated_continuous.gpkg` (`mean_lux`) + `<City>_nodes_2m_densified_illuminated.gpkg` | same |
| 4 | `04_directional_lighting.py` → `<City>_directional_lighting_lookup.csv` | same |

Pick the pipeline by what data you have: real per-lamp technical attributes → Turin;
only pole/lamp locations → generic. The generic defaults live as constants at the top of
`02_street_lights_generic.py`.

## Filename contract

Outputs the sim loads (`<City>_edges_illuminated_continuous.gpkg`, `<City>_censusData.gpkg`,
`<City>_directional_lighting_lookup.csv`) must match the Java readers, which use
`Pars.cityName`. Get the folder name wrong and the sim silently falls back (e.g. `mean_lux`
defaults to 0).
