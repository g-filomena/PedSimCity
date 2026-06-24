# Data pipeline

Turns raw city inputs into the GeoPackages / CSVs the Java simulation reads.

- **Raw inputs** live in `inputData/` (rawer, pre-resources) and `src/main/resources/<City>/`
  (the raw `<prefix>…` files: census, edges, buildings, lamp inventory).
- **Processed outputs** are written into `src/main/resources/<City>/` as `<City>_…` files and read by
  the sim, where `<City>` = the resources folder name = `Pars.cityName`.

## Running

Two double-clickable launchers at the repo root (Windows). Each **prompts** for the city (the folder
name under `src/main/resources/`) and the raw-file prefix (e.g. `Torino_`):

| Launcher | Runs | Produces |
|---|---|---|
| `build_census.bat` | `pipeline/01_census_and_poi.py` | `<City>_censusData.gpkg` only |
| `build_city.bat` | `pipeline/build_city.py` (steps 1–4) | census + all lighting layers |

Or from a terminal:

```bash
python pipeline/build_city.py        --input_dir src/main/resources/TorinoCentre --prefix Torino_
python pipeline/01_census_and_poi.py --input_dir src/main/resources/Torino       --prefix Torino_
```

`build_city.py` is a thin **orchestrator**: it runs the step scripts in order and skips any whose
output already exists. It does no data work itself.

## Steps

| Step | Script | Output (read by Java as `<City>_…` unless noted) |
|---|---|---|
| 1 | `01_census_and_poi.py` | `<City>_censusData.gpkg` — columns `residence_pct`, `workplace_poi`, `night_poi`, `vulnerability_pct` |
| 2 | `02_street_lights_simple.py` | `<prefix>puntiLuce_with_radius.gpkg` (lamp physics; intermediate) |
| 3 | `03_street_lights.py` | `<City>_edges_illuminated_continuous.gpkg` (`mean_lux`) + `nodes_2m_densified_illuminated.gpkg` (intermediate) |
| 4 | `04_directional_lighting.py` | `<City>_directional_lighting_lookup.csv` |
| — | `active_frontages.py` | `<City>_edges_with_frontages.gpkg` — **optional**, NOT in the ordered build and not yet read by the sim |

**Filename contract:** outputs are `<City>_…` to match the Java readers; raw inputs are `<prefix>…`.
Get this wrong and the sim silently falls back (e.g. `mean_lux` defaults to 0). The lighting steps
(2–4) need the raw lamp inventory (`<prefix>puntiLuce.gpkg`); continuous lux currently exists only
for Turin.
