# Brief: re-run the Torino night pedestrian simulation

**Superseded — the re-run was delivered on 3 Sep 2026** as `edges.gpkg`,
`edge_volume.csv` and `trips.csv` (63,180 active agents, 164,425 trips,
3,342,615 crossings). Kept as the record of what was asked for, and as a
template if the model is run again. Results are in `METHODS_VALIDATION.md`.

Paste this to Claude Code on the machine that holds the model.

## Goal

Re-run the **Torino_simplified** night pedestrian model. The previous run
(`Day 1, Job 0`) is statistically too thin to map agent composition: 1,495
agents / 5,779 trips spread over 24 h and 14,531 edges leaves a median of
**0 trips per edge per 2-hour band**. Scale it up and change the export format.

## What to change

| | previous | new |
|---|---|---|
| Agents | 1,495 (0.19% of Turin's 800,000) | **~40,000 (5%)** |
| Trips | 5,779 | **~155,000** (keep ~3.9 trips/agent) |

Size to **~155,000 trips**; if trips-per-agent changes, adjust the agent count
to hit that. Everything else stays identical: same network, same 20-minute
steps (72 per day), same lighting model, same vulnerable/normal behaviour rules,
same seed policy. Replications pooled to ~155,000 trips are equally good.

## What to check before launching

1. **73% of trips are flagged vulnerable** in the old run. Confirm that is
   intended and not a config slip — it sets the baseline everything is compared
   against.
2. Confirm the run covers the **full 24 h**, not only night hours. The old
   `HOURLY_VOL` spans all 24.

## Output format — important

Do **not** export as a self-contained `results.html`. The old one embeds the
trip paths and is already 29 MB; at 27× the trips it would be ~800 MB and
unusable. Write plain data files instead, all in **EPSG:3003**:

1. `edges.gpkg` (or GeoJSON) — one row per edge: `edgeID`, geometry,
   `mean_lux`.
2. `edge_volume.csv` — **the one that matters**, with the agent split already
   done: `edgeID, hour, vuln_count, norm_count` (hour 0-23; rows only for
   non-zero). This removes any need to reconstruct volumes from paths.
3. `trips.csv` — one row per trip: `trip_id, agent_id, start_step, end_step,
   vulnerable, path_length_m, mean_lux`. Paths optional; include as an edgeID
   sequence rather than coordinates if they are wanted at all.

Keep the HTML viewer if it is useful locally, but the files above are what gets
transferred.

## Why this size

Precision on a per-cell vulnerable share needs ~66 trips per cell per band for
±10 pp at 95%. At 5% of population, 95% of daytime walking and 87% of overnight
walking falls in cells that clear it; below 2% the night band is unusable.
Cell coverage itself saturates near a third of street-cells at any feasible
size, because most peripheral cells see almost no pedestrians — that is real,
not a sampling shortfall.

Cheaper alternatives if the compute is not available: three parts of day
instead of nine bands (~3× cheaper), or 432 m blocks instead of 216 m
(~4× cheaper).
