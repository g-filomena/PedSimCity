# Activity module — what is left to do

The activity tier is the behavioural model: who the population is, what they do in a day, where they
go, how and when. Most of the model's open calibration questions land here.

Absorbed `FUTURE_WORK.md` (a July 2026 realism assessment) on 13 September 2026 — the implemented
items there are now described in `README.md` and `CLAUDE.md`, and what was still open is below.

---

## 1. Layer 2 is a count with an invented interior

`ActivityPars.walkedTripsPerPersonPerDay = 0.51` anchors *how many* trips happen; it is ISFORT's own
quantity. Nothing anchors *what is in them*: `DailyAgenda` composes agendas from
`secondActivityProbability`, `postWorkActivityProbability` and `secondPostWorkActivityProbability`,
none of which has a source.

The right input is a whole observed day from a diary — not activity-by-activity draws, which destroy
the correlations that make a day a day: whoever takes a child to school shops on the way back. For
Italy that is the ISTAT **Uso del tempo** microdata, a request to UniData rather than a download.
**Start the request early.**

There is also a literature on inferring activity sequences from mobile network data instead of
surveys: Widhalm et al. fit a relational Markov network on Boston and Vienna and recover sequences
and scheduling compatible with the city surveys; a Paris/Chicago study puts 90% of survey travel
patterns as compatible with phone data; Zilske et al. feed CDRs to MATSim in place of travel diaries.
The catch is resolution — those use individual trajectories, while the Turin Vodafone data is
aggregated presence on a quadkey grid.

Before anyone invests in that route, the sobering numbers: the GPS schedule-generation work
(PMC9259096) reports 63–71% accuracy on individual days, with aggregates matching within 3%.
Individual days come out largely wrong even in the good work; what validates is the aggregate.

## 2. `distanceWeight` is uncalibrated, and it *is* the trip-length distribution

Everything that used to be decided by a sampled length is now decided by one coefficient
(`ActivityPars.distanceWeight = 0.0012`). That is the point of the destination-choice redesign and
also its outstanding debt. Calibrating it needs an observed distribution: **Audimob microdata**,
again a request to ISFORT. Until then any trip-length result from this model is a plausible shape,
not a validated one.

Note the coupling: it was fitted while `networkCircuityFactor` sat at 1.41 and the measurement never
ran. The two want re-deriving under one convention, or the circuity correction is applied twice.

## 3. Settle `choiceSetRadiusMetres`

Measured on a fixed seed, varying only the radius:

| radius | trips | mean leg |
|---|---|---|
| 1,500 m | 715 | 1,103 m |
| **3,000 m** (current) | 644 | **1,384 m** |
| 6,000 m | 664 | 1,438 m |
| 12,000 m | 639 | 1,371 m |

Converged by 6,000 m; the current value costs about 4% of the mean walked leg, so it bounds the
behaviour and not merely the work. Either raise it to 6,000, or keep 3,000 and report the truncation
with every trip-length result.

## 4. There is no independent check on commuting left

Workers were fitted to ISTAT, then students were fitted too, and `workplaceDistanceDecay` /
`workplaceMinDistanceMetres` were fitted alongside them. A Torino day reporting `workers 16.4%
(ISTAT 16.3%), students 36.9% (38.0%)` says the fit worked and nothing else.

Worth keeping in view, because it is what the fit replaced: on two networks differing in one relevant
way, with an unfitted decay of β = 2 from a 540 m floor, Torino_simplified (no WORK tags, uniform
random workplace) gave **13.3%** of workers walking and full Torino gave **49.2%**, against ISTAT's
12.0% of the time. **Where the model puts workplaces mattered more than the curve.** Neither
arrangement was right for the right reason — uniform has no decay at all — but any future candidate
should be checked against that sensitivity.

Candidates for a real check, least new work first:

- **The ISTAT matrix's non-walk commute durations** — already extracted in `COMMUTE_DISTANCE.md`, and
  nothing has used the car / bus / tram / metro rows. The duration distribution by mode is itself a
  check on where the model puts workplaces.
- **Observed commuting OD.** Census workplace-flow tables, where a city publishes them, could replace
  or validate the OSM WORK-tag attraction with gravity decay.
- **The Turin Vodafone diurnal series.** `DepartureProfile` is a prediction that has never been
  compared against it. A single pre-registered step, and the cheapest real check available.
- **The Melbourne pedestrian counts**, for spatial pattern (city-image module; no clock).

## 5. The long walked-commute tail is under-produced

The Torino run gives **0.2%** of walked commutes over 5,112 m against ISTAT's **1.3%** — short by
roughly six-fold in that band while matching the other three (78.2 / 18.5 / 3.1 against
76.2 / 19.0 / 3.4). The fitted logit matches the mass and misses the tail. See
`analysis/validation/Torino/night_torino_2026-09-13.md`.

## 6. The commute is not multi-modal, so the walking it generates is missing

The commuters who do not walk currently make **no trip at all**. What the model should produce for
them is the walked part of a multi-modal journey — home to the boarding stop, alighting stop to work
— which is also the pedestrian volume that concentrates around stations and that nothing else here
can generate. The transit module (`TransitStop`, `TransitVehicle`, the mode split in
`ActivityAgent.planTrip`) is the half that already exists.

Related, and smaller: **mode choice is evaluated only on a trip chain's first leg.** Chained legs go
through `startChainedTrip`, which calls `planRoute()` directly and bypasses the transit split, so
every intermediate leg is walked regardless of distance. Routing chained planning through the same
`planTrip` seam would fix it.

## 7. The `!isDark()` guard on commuting has no source

`CommuterAgent.shouldGoToWork` refuses to set off after dark, so nobody in the model commutes in the
dark — and people commute in the dark all winter. `planMandatoryDeparture` was made to agree with it
so the leg budget is not mischarged; agreeing with a rule is not the rule being right. **This matters
most to the night module**; see `../night/TODO.md`.

## 8. Opening windows and stay durations are 32 invented numbers

`ActivityPurpose` holds open hour, close hour, mean stay and log sigma for each of eight purposes, and
`DepartureProfile` reads the windows straight off the enum. **Nothing reads an OSM `opening_hours`
tag.** They are city parameters with no city behind them — the enum says 11:00–23:00 for dining
everywhere — and belong in `<City>.properties` under a key convention the flat file does not have yet
(`purpose.DINING.open`, `purpose.DINING.stayMinutes`).

## 9. Worker/flex from P101 has one residue

The 2021 permanent census publishes no enrolment variable at section level, so students are the 15-24
age band and the employed among them are counted twice — borrowed from flex rather than from student.
Bounded by the youth employment rate times the 15-24 share, a couple of points of adults. Closing it
needs a per-zone employed-15-24 figure the census does not give; the municipal aggregate would do, or
state it as an assumption. Wiring: `01_census_istat.py` → `CensusZone` → `Persona.sample`.

## 10. Smaller, and deliberately not done

- **A city with no WORK tags gets no decay at all.** Core's `selectRandomNode` ends the workplace
  ladder with a uniform draw, so on such a city commutes come out far longer than they should.
- **Lunch trips from work** — splitting the mandatory stay around a midday dining leg adds agenda
  complexity for little effect at pedestrian scale. The work stay stays whole.
- **Only `ActivityPars` and `Pars` keys are exercised by the city file.** `NightPars`,
  `LearningPars`, `SocialPars`, `TestPars` and `EmpiricalPars` have no city files at all.

---

## Interactions worth knowing

- **Night** agents get personas, purposes and chaining — their populate goes through
  `ActivityPopulate.defineHomeWorkLocations` — and use the same destination choice as everyone else,
  with a park/water refusal applied to the result.
- **Learning** agents extend core `Agent`, not `ActivityAgent`, so personas and agendas do not apply
  there. Memory decay reads `STEP_DURATION`, so a finer step rescales it automatically.
- **Behavioural `isDark` is seasonal; the exporters' day/night aggregation window is fixed.** The
  divergence is intentional, so that aggregates stay comparable across dates.
- **Home and work assignment goes through `NodesLookup`**, so it needs GeoMason-light 2.2.0 to be
  seeded. Seeded runs do not replay traces from before that change.
