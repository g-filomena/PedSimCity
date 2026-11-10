# Night on full Torino — server run, 13 September 2026

First run of the post-refactor code on `gdsl1`. One simulated day, **4,232 agents (0.5% of the
census population)**, seed 20260912, night module, `Torino`.

Reproduce with:

```bash
java -Xmx64g -cp "classes:lib/*" pedsim.night.launcher.NightLauncher \
  --headless --cityName=Torino --percentage=0.005 \
  --days=1 --jobs=1 --stepDelayMs=0 --exportHtmlDashboard=false
```

Log kept at `/mnt/home/gabriele/PedSimCity-wip/runs/night_torino_20260913_213202.log`.

## What the run produced

| | model | reference |
|---|---|---|
| walked commute share, workers | **16.4%** | ISTAT 16.3% |
| walked commute share, students | **36.9%** | ISTAT 38.0% |
| walked commute length ≤1278 / –2556 / –5112 / >5112 m | **78.2 / 18.5 / 3.1 / 0.2** | ISTAT 76.2 / 19.0 / 3.4 / 1.3 |
| metres walked per agent per day | **662 m** | 600–1000 m band, ISFORT-derived (`RELEASE_BUDGET.md`) |
| mean walked leg | **1,364 m** | 1,384 m measured earlier at the same 3,000 m choice-set radius |
| legs realised / budgeted | 2,054 / 2,158 (95%) | budget = 4,232 × `walkedTripsPerPersonPerDay` 0.51 |
| trips exported | 2,054 | |
| edge-volume rows | 58,802 | |
| network circuity | 1.292, measured | matches the recorded Torino figure |

Routing was clean: 0 unusable route lengths, 0 destination fallbacks, 0 band widenings, 0 angular
fallbacks, 0 full-network escalations, 0 known networks left in pieces.

The leg-budget arithmetic closes: 1,213 mandatory legs + 944 discretionary (0.093 chains per person
× 4,232 × 2.40 legs) = 2,157 against a 2,158-leg budget, 95% realised, the remainder still in flight
at midnight.

## Reading it

**The commute shares are a check, not an input.** Nothing in the model reads
`walkShareCommuteWorker`/`Student` to decide anything — the per-agent decision is
`ActivityAgent.decideCommuteMode`, a draw against the fitted logit at that agent's own home–work
distance, and `ActivityTravelDemand.prepare` logs the realised share against the ISTAT figures once
per simulated day. 16.4% against 16.3% is the model reproducing a number it was not handed.

The caveat from `TODO.md` item 3 still stands: the curves **were fitted** to these same ISTAT
targets, so this is the fit working, not an out-of-sample test.

**662 m per agent per day** sits inside the band `metersPerDayPerPerson` was derived from, at the low
end. Note that nothing in the run computes this — that field was deleted on 13 Sep, having had no
readers — so it was worked out by hand from `walked / agents`. Making it a real end-of-run assertion
in the activity module would be worth doing.

**The gap is the long tail.** The model produces **0.2%** of walked commutes over 5,112 m where ISTAT
has **1.3%** — under by roughly six-fold in that one band, while matching the other three closely.
That is the one place the fitted curve does not reproduce the observed shape.

## Caveats

- It is the **night** module, so these are activity-tier behaviours with night routing. Structural
  commutes are still generated in daylight.
- The server's `Torino_censusData.gpkg` is the older file (4,022,272 bytes against 4,190,208 local),
  so the persona mix came from a stale census. Ship the current one before repeating.

## What it took to get the run to start — three silent vintage mismatches on `gdsl1`

Worth recording, because each presented as something other than what it was.

1. **`lib/GeoMason-light-2.2.0.jar` was a different build under the same version number** — 115,882
   bytes on the server against 117,232 locally. The server's predated the `Islands` work and the
   `Route.length` fix. It presented as
   `NullPointerException: getIntegerAttribute("nodeID") is null` inside
   `Environment.prepareGraph`, which points nowhere near the cause. What isolated it was running
   night on Torino **locally** with identical code and data, where it completed cleanly.
   **2.2.0 is unpublished, and one version number now covers two different builds.** Publishing it,
   or stamping the rebuild, would stop this recurring.
2. `Torino_edges_illuminated_continuous.gpkg` was the June build against September's rebuild.
3. `Torino_censusData.gpkg` still differs, as above — read after the crash point, so it did not bite.

Two further traps, both already in `CLAUDE.md` and both still live:

- `server.properties` has `ssh.key` on a `C:` path that no longer exists (OneDrive moved to `E:`), so
  the built-in remote launcher still fails. The key beside the repo was used directly.
- With the working tree uncommitted, the built-in remote path (`git pull` then compile) would have
  run September code. Classes were hand-shipped instead; the previous ones are preserved on the
  server as `classes/pedsim.old_20260913_212833`.
