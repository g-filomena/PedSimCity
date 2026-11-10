# Turin: model against mobile-network presence

Compares a PedSimCity run with Vodafone Analytics Smart Locator presence data for the comune di
Torino, on a shared 216 m quadkey grid in EPSG:3003 over the API's nine time bands.

Method, results and caveats are in `METHODS_VALIDATION.md`.

## Where the observed data comes from

Not from here. Download and aggregation live in `Scripts/vodafoneAPI/`, outside this repo, along
with the credentials, the job definitions and the 125 days of daily CSVs. The interface between
the two is one file, `data/torino_presence.gpkg`, copied in — so this folder needs neither API
access nor a sibling path on disk.

That costs a duplication: `vfgrid.py` (grid, quadkey maths, time bands, municipal outline) and
`vfstyle.py` (palette, typeface, timing) exist on both sides. They are short and change rarely,
but a change to either must be copied across by hand.

## Layout

```
extract_simulation.py   sim/edges.gpkg + sim/edge_volume.csv -> data/torino_simulation.gpkg
unique_agents.py        sim/trips.csv                        -> agents_<band> in sim_cells
sensitivity.py          both GeoPackages + the city network  -> sensitivity.csv, cell_exposure.csv
animate_sim.py          both GeoPackages                     -> figures/torino_sim.mp4
vfgrid.py, vfstyle.py   shared with Scripts/vodafoneAPI/ (copies, see above)

sim/     the validated re-run of 3 Sep 2026: edges, per-edge-per-hour volumes, trips
data/    torino_presence.gpkg (observed, copied in) and torino_simulation.gpkg (derived)
figures/ the three-panel comparison
```

`sim/` holds the run `METHODS_VALIDATION.md` reports on: 63,180 active agents, 164,425 trips,
3,342,615 crossings. It is kept here rather than read from `outputs/`, which is gitignored,
transient, and currently holds a different and much smaller run.

`sim/results_firstrun.html` is the superseded first run (1,495 agents), read only as a fallback
when the GeoPackage export is absent. It is 29 MB and can go once that fallback is dropped from
`extract_simulation.py`.

Everything except the scripts and the Markdown files is gitignored: the data is ~85 MB and
regenerable.

## Running

Use the `geovis` conda environment, activated. Calling `envs/geovis/python.exe` directly
leaves `Libraryin` off the PATH; the render then dies part-way through on a delay-load
DLL fault (`0xc06d007f`) inside numpy with no Python traceback, which looks like a script
bug and is not.

```bash
conda run -n geovis --no-capture-output python extract_simulation.py
conda run -n geovis --no-capture-output python unique_agents.py
conda run -n geovis --no-capture-output python sensitivity.py
conda run -n geovis --no-capture-output python animate_sim.py --poster 18_20 --no-video
conda run -n geovis --no-capture-output python animate_sim.py --out figures/torino_sim.mp4
```

`conda run` rejects a multi-line `python -c`, so pass scripts as files. `unique_agents.py` must
run before `sensitivity.py`, which reads the `agents_<band>` columns it writes.

Requires geopandas 2.x for `union_all`; the `gis` and `cityImage` environments are too old.

## Known limits of the comparison

`METHODS_VALIDATION.md` has the full account. The two that matter most:

- **The observed side is presence, not walking.** A 0–30 min dwell removes residents and workers,
  but the API's dwell dimension has three buckets (0–30, 30–60, >60) and every motorised traveller
  crosses a 216 m cell inside the first: a car in ~26 s against a pedestrian's ~2.8 min. Dwell
  time cannot separate mode, so cars, buses, trams and the metro remain in the data.
  `sensitivity.py` measures the floor of that contamination instead of estimating it — 4.1% of
  observed passing presence sits in cells with no walkable street at all, and railway cells in the
  sparsest quartile of walkable street carry 3.54× the presence per metre.
- **The units differ.** The model counts edge crossings, the API unique visitors per cell per
  band. `unique_agents.py` recovers the per-person count from `trips.csv`. It is worth about +0.02
  on ρ, but removes 44% of the model's apparent evening over-representation, which was an artefact
  of counting events rather than people.
