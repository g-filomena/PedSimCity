# Validation: methods

How the simulated pedestrian model is compared against Vodafone Analytics
mobile-network data for the comune di Torino. Everything below is at 216 m
quadkeys in EPSG:3003, over the nine Smart Locator time bands.

## Simulation run

| Parameter | Value | Source |
|---|---|---|
| City | Torino_simplified | 14,531 edges, 7,367 nodes, EPSG:3003 |
| Population base | 846,567 | Census resident total (P1), applied automatically |
| Sampling fraction | 0.1616 | Calibrated to the 155,000-trip target |
| Agents | 136,805 | Derived: population × fraction |
| Duration | 1 day | Full 24 h, 1 job |
| Simulation step | 300 s | 288 steps per day |
| Release cadence | 20 min | 72 release events per day |
| Trip distance band | 900–2700 m | avgTripDistance 1800 m, unchanged |
| Vulnerability | census-sampled | Per-agent draw on home-zone `vulnerability_pct` (superseded — see below) |

Delivered as `edges.gpkg`, `edge_volume.csv` (edgeID × hour × vulnerable/normal
counts) and `trips.csv`. Observed in the export: **164,425 trips from 63,180
distinct agent ids** — under half the 136,805 configured, so `trips.csv` holds
only agents that actually moved. 3,342,615 edge crossings, 13,948 of 14,531
edges used, 75.1% of crossings by vulnerable agents.

## Observed data

Daily `footprint` calls, 125 days across the four contracted windows, averaged
within each window. Two quantities matter here:

- `band_<t>` — all unique visitors in the cell in that band
- `passing_<t>` — those with a **0–30 min dwell**

**The validation uses `passing_*` only.** Around 70% of daytime and 80% of
overnight presence is people with >60 min dwell — at home or at work — which the
model does not simulate. Restricting to short dwell is what makes the two
quantities describe the same thing. Gender likewise comes from
`passF_*` / `passM_*`, downloaded with `dwellTimeRange` filtered to 0–30.

## The three-panel figure

`figures/torino_sim.mp4` steps through the nine bands:

1. **who walks** — street network, colour is the share of vulnerable agents
   against the simulation's own 75% average, width follows each edge's volume
   rank within the band; edges under 25 crossings are not drawn
2. **how many walk** — simulated crossings per 216 m cell **per hour**, log
   scale, carried by luminance alone so that hue stays reserved for the two
   diverging panels. Per hour rather than per band: seven bands are 2 h but
   20-24 is 4 h and 00-06 is 6 h, so band totals made the wide bands look
   busiest — by total, 20-24 leads with 769,884 crossings, but per hour 18-20
   is the true peak at 341,967 against 192,471
3. **simulated vs observed** — the residual described below

Panel 3 cannot subtract the two datasets directly, so each side becomes that
cell's **share of the band's city total** and the panel shows their
**difference in percentage points**, on a purple–orange scale (distinct from the
blue/red carrying vulnerability beside it) running to ±0.15 pp, which holds 97%
of cells. It stays a share throughout — the two datasets are never differenced
as counts, since the model's totals are a 63,180-agent sample of crossings and
the API's are unique visitors. The figure captions it as the *difference in % share of total pedestrian
counts*, anchored with one equivalence: ±0.15 pp is about 750 people in a
2-hour band. The denominator is named because panel 2 is normalised per hour
while this anchor is per band — crossings are events and divide by duration,
unique visitors are people and do not.

Worked example, cell `12022123233323333`, 18:00–20:00: the model puts 1,218 of
its 683,934 crossings there (0.178%); the data records 142 of its 502,198
passers-by (0.028%). The difference of +0.150 pp is what the map shows — the
model's pattern would place roughly 894 people there against the 142 observed.

A difference rather than a ratio, because it is self-weighting: a busy cell that
is slightly wrong outweighs a quiet cell that is wildly wrong, and the values sum
to the total misallocated share. The ratio did the opposite — its worst values
sat in the quietest cells (ρ −0.22 against observed volume), overstating error
where little was at stake. Weighting also removes the need for a threshold, so
every street cell is drawn.

**The residual is computed on total volume, so it cannot say which agent type
is misplaced.** A cell reading as overestimation may hold too many vulnerable
agents, too many non-vulnerable ones, or both. That decomposition is not
available: the API carries no vulnerability attribute, and the only candidate
proxy — gender — does not track the model's vulnerability geography (ρ +0.05,
below). Volume is validated; composition is not.

Summed across cells, **28% of the city's walking sits in the wrong cell** in a
typical band. The pattern is coherent: the model underestimates the core and
overestimates the surrounding ring and radial corridors — it pushes people
outward along arteries where the data keeps them central.

## Putting them on one grid

Edge counts are split across the cells each edge crosses, weighted by the length
inside each. `street_m` records the metres of network per cell, so "no street
here" stays distinct from "street nobody walked". Comparison is restricted to
the **2,366 cells containing street**.

Units differ — crossings versus unique visitors, and one simulated day versus a
30-day mean — so only **rank** and **share** comparisons are made, never levels.

## Results

| test | result |
|---|---|
| Spatial, per band, crossings | ρ 0.70–0.73 (autumn 0.713, winter 0.717, spring 0.710, summer 0.697) |
| Spatial, per band, unique agents | ρ 0.73 mean — +0.021 in all 36 band × season cells, never worse |
| Contamination floor | 4.1% of observed passing sits in cells with no walkable street |
| Railway corridors, sparsest quartile | 3.54× the passing presence per walkable metre |
| Temporal, band totals | Pearson 0.674, Spearman 0.833 |
| Coverage | 97.0% of street-cells walked; 99.6% of observed passing presence inside them |
| Concentration, top 10% of cells | sim 38.7–40.7% vs observed 42.8–44.9% |
| Composition, vulnerable vs women | ρ +0.02 to +0.11, no threshold changes it |
| Residual geography | stable across all nine bands — a fixed pattern of error, not noise |

**Spatial fit is good and stable** across seasons and bands, with no weak
overnight band. The first run's ρ of 0.55 was sampling noise; 21× the agents
moved it to 0.71.

**The daily profile is not calibrated, but it is less wrong than it looked.**
Counting crossings, the model puts 43.5% of walking after 18:00 against an
observed 27.2%. Counting *people* — see "Crossings against people" below — it
puts 36.3%, so the evening gap falls from +16.4 to +9.2 percentage points and
**44% of the apparent temporal error was the unit mismatch, not the model.**
What survives is real and unchanged in sign: the midday deficit does not
improve, it worsens slightly (10–12: 11.8% observed against 5.2% of agents).
The evening figure to quote is 36%, not 44%.

**These temporal figures were measured against the superseded diurnal curve.**
They describe `TimePars.computeTimeStepShare`, a curve of tuned Gaussian peaks
whose shape was chosen because it looked right — so comparing it with observed
presence compared an input with an observation, and calling the result a
design choice rather than a validated one was literally correct. Departures are
now timed by `DepartureProfile` (mandatory start windows, OSM opening hours,
persona preferences, ISTAT walking share of commuting), which predicts a shape
instead of being handed one. The predicted weekday profile is 10.0% at 06–08,
21.8% at 08–10, then close to flat at 11–12% through the afternoon and 11.8% at
20–24 — visibly less evening-heavy than the 36% above.

**The comparison has not been re-run.** Nothing in this file has been remeasured
against the new profile, so the temporal rows in the table remain the old
curve's. Re-running it is the single pre-registered step: none of the profile's
parameters were informed by the Vodafone series, so one comparison decides it.
Departures are also not presence — a trip chain leaving at 08:00 puts people on the
street later too — so the two series need reconciling before they are read
against each other. The spatial, composition and contamination results are
unaffected: they do not depend on release timing.

**The errors have a fixed geography.** The over/under pattern in panel 3 keeps
its shape across every band: the model over-concentrates through the western and
central-western districts and under-represents a corridor from the centre-south
to the north-east. Sampling error would flicker between frames; this does not.
Note this sits alongside the ρ 0.71 spatial correlation without contradicting
it — the *ranking* of cells is broadly right while individual shares are off by
a factor of two or more in particular districts.

**Composition does not match.** Where the model concentrates vulnerable agents
is not where women are. This is not a sampling artefact: decomposing the
variance, sampling accounts for about a third of the spread in the simulated
vulnerable share (total sd 0.056, sampling sd 0.034, true spatial sd 0.045 at
n≥66), and the correlation stays near +0.05 at every threshold from n≥66 to
n≥600. There is real spatial structure in the model's vulnerability; it simply
does not align with the observed geography of women out on foot.

The 75% vulnerable base rate compresses the dynamic range — a share that high
cannot vary much per cell — but it does not explain the null. Two things worth
checking: whether `vulnerability_pct` varies enough between home zones to
produce spatial contrast once agents disperse, and whether the construct is
meant to correspond to observed gender at all.

**Answered, 19 Sep 2026: it did not.** The vulnerable set was all women plus
male children plus elderly men, over every resident, and a node took the highest
rate among the zones around it — 74.1% of agents, against a census 68.9%. It is
women now, over the zone's adults, drawn per agent from the home zone: 52.5%.
This comparison should be re-run against that construct before its null is read
as a finding about the model's geography.

## Crossings against people

The two sides count different things, and until now only one of them was
reported. The model counts **edge crossings**: an agent who walks a street three
times in a band is three. The API counts **unique visitors**: that agent is one.
Wherever the model sends people back and forth, or wherever a cell holds many
edges, the simulated side inflates by construction — and rank correlation does
not remove it, because the inflation is not uniform across cells or bands.

`trips.csv` carries an agent id, so the per-person count is recoverable.
`unique_agents.py` writes it into `sim_cells` as `agents_<band>` beside the
existing crossings, with `repeat_<band>` for the ratio.

| | crossings | unique agents |
|---|---:|---:|
| whole day | 3,342,615 | 2,043,506 agent-cell-band presences |
| ρ against observed passing | 0.684 – 0.731 | 0.705 – 0.755 |
| crossings per presence, 06–08 | | 1.00 |
| crossings per presence, 20–24 | | 2.26 |

**ρ improves by +0.021 in all 36 band × season combinations, and is never
worse in any of them.** The gain is small because the two quantities are
strongly collinear; the point is not the gain but that a known bias is gone,
and that the bias was not uniform — the repeat rate runs from 1.00 in the
06–08 band to 2.26 in 20–24, which is why removing it moves the daily profile
so much more than it moves ρ.

## What the observed side really contains

`sensitivity.py` tests the two assumptions the comparison rests on, using data
already on disk.

### The dwell cut barely matters

`passing` is a 0–30 min dwell, and the worry cuts both ways: it keeps every
motorised traveller (a car crosses a 216 m cell in ~26 s, a pedestrian takes
~2.8 min — all of them inside the same bucket, so **dwell time cannot separate
mode at all**), and it drops pedestrians who linger, since a market, a piazza
or a park bench land in 30–60 min. The API's dwell dimension has only three
buckets, so 0–30 is the finest cut available.

`build_gpkg.py` now carries the middle bucket as `mid_<band>` instead of
discarding it, which lets the cut be tested:

| cut | mean ρ |
|---|---:|
| 0–30 min (`passing`, in use) | 0.730 |
| 0–60 min (`passing` + `mid`) | 0.731 |
| everyone present | 0.718 |

The 30–60 bucket is 3.5–8.3% of all presence, and moving the boundary changes ρ
by 0.001. **The lingering-pedestrian worry is empirically negligible for ranks**
— which also means the filter is not doing the work; what separates `passing`
from `everyone` is the >60 min class, as before.

### Presence where nobody can walk

The cleanest available measure of non-pedestrian contamination needs no model
of mode at all. Turin's full pedestrian network is 44,278 edges covering the
whole comune; **648 of the 2,790 cells contain none of it**, and 647 of those
still record passing presence. That presence is not somebody walking the
network.

**It is 4.11% of all observed passing presence** — median 66 per cell against
870 in a walkable cell. That is the floor: the residual sits on top of it, so
contamination is at least this much and certainly more, since a cell with some
walkable street also carries vehicles.

### The railway signal survives its own confound

Cells touching the model's railway barriers carry 1.49× the passing presence
per walkable metre, and cells touching its road barriers 1.83×. Neither ratio
means much on its own: Turin's corsi are simultaneously the car arteries and
the busiest pedestrian streets, and the railway cells include Porta Nuova and
Porta Susa, two of the largest pedestrian generators in the city. Road class,
for the same reason, is useless as a covariate here — and it is collinear with
the model's own routing besides.

Stratifying by how much walkable street a cell holds separates the two stories,
because stations sit in the densest quartile and open rail corridors in the
sparsest:

| walkable street | median m | railway cells | elsewhere | ratio | n |
|---|---:|---:|---:|---:|---:|
| Q1 sparsest | 240 | 1.95 | 0.55 | **3.54×** | 34 |
| Q2 | 673 | 1.46 | 0.94 | 1.55× | 31 |
| Q3 | 1058 | 1.61 | 1.31 | 1.23× | 23 |
| Q4 densest | 1609 | 1.15 | 1.13 | 1.02× | 18 |

A pedestrian-generator effect would be **largest** in Q4, where the stations
are. It is largest in Q1 and vanishes in Q4 — the signature of rail traffic
being recorded as presence, not of stations attracting walkers.

### Still open

- **The metro is not tested.** Line 1 runs under Corso Francia and Via Nizza,
  and underground passengers geolocate to the surface cells above. It is not in
  any layer on disk and would have to come from OSM. It is the single sharpest
  expected contaminant, and the prediction is specific: it inflates the observed
  side, so it should appear as the model *under*-estimating a corridor — which
  is the sign of the residual already reported through the centre-south.
- **A network inconsistency.** 359 cells that the simulation network reaches are
  called unwalkable by the full pedestrian network. `Torino_simplified` is not a
  subset of `Torino_edges`, so "street cells" means two different things in the
  two files. Worth resolving before the 2,366-cell comparison set is quoted
  again.
- Localisation error, under-16 coverage, tourist and foreign SIMs, market share
  and its spatial variation, and the API's minimum detection threshold — all
  unquantified, all needing a question to Vodafone rather than more analysis.

## Reproducing

```bash
python extract_simulation.py                      # edges.gpkg + edge_volume.csv -> sim gpkg
python unique_agents.py                           # adds agents_<band> to sim_cells
python sensitivity.py                             # the three tests above
python animate_sim.py --poster 18_20              # three-panel comparison
```

`build_gpkg.py`, which turns the daily CSVs into the observed GeoPackage, lives
on the preparation side in `Scripts/vodafoneAPI/`. Run everything in the
activated `geovis` environment — see `README.md`.
