# Activity module — what is left to do

The activity tier is the behavioural model: who the population is, what they do in a day, where they
go, how and when. Most of the model's open calibration questions land here.

Absorbed `FUTURE_WORK.md` (a July 2026 realism assessment) on 13 September 2026 — the implemented
items there are now described in `README.md` and `CLAUDE.md`, and what was still open is below.

---

## Read this before trusting an earlier activity figure

**Every commute number in this file was measured before 14 September 2026 and is superseded.** The
per-purpose attraction maps iterated in identity-hash order, so `WorkplaceChoice.draw` picked a
different workplace on each JVM build; making them insertion-ordered moved where the model puts
workplaces, and with it the commute distance and `walksToWork`. The current Torino day reports
`workers 16.8% (ISTAT 16.3%), students 38.3% (ISTAT 38.0%)`. The walked-commute length bands in
item 5 have not been re-measured since.

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

Note the coupling: it was set while `networkCircuityFactor` sat at a hardcoded 1.41 and the
measurement never ran. **Decided 16 Sep 2026: leave it at 0.0012.** Only the product of the two sets
any choice probability and measuring the factor (1.292 on Torino) has drifted that product by 8.4%,
but rescaling to hold it invariant would re-import through the coefficient the circularity that
measuring circuity removed — 1.41 was itself measured on the old mechanism's trips. Report the drift
rather than absorbing it; fitting the coefficient properly still needs Audimob. `Torino.properties`
sets the key, so the Java default alone does nothing for Turin.

## 3. Settle `choiceSetRadiusMetres`

Measured on a fixed seed, varying only the radius:

| radius | trips | mean leg |
|---|---|---|
| 1,500 m | 715 | 1,103 m |
| **3,000 m** (current) | 644 | **1,384 m** |
| 6,000 m | 664 | 1,438 m |
| 12,000 m | 639 | 1,371 m |

Converged by 6,000 m; the current value costs about 4% of the mean walked leg, so it bounds the
behaviour and not merely the work. **Decided 14 Sep 2026: keep 3,000 and report the truncation with
every trip-length result.** The javadoc's claim that the radius bounds the work rather than the
behaviour is therefore wrong by about 4%, and that is the number to quote.

## 4. There is no independent check on commuting left

Workers were fitted to ISTAT, then students were fitted too, and `workplaceDistanceDecay` /
`workplaceMinDistanceMetres` were fitted alongside them. A Torino day reporting `workers 16.8%
(ISTAT 16.3%), students 38.3% (38.0%)` says the fit worked and nothing else — and note that those
are the figures *after* the 14 Sep workplace-draw fix, which moved them without anything being
refitted, which is its own small warning about how much of the agreement the fit is carrying.

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

**Those four numbers are from 13 September and the workplace draw has changed since**, so re-measure
before quoting them: the fix moved where workplaces are, which is exactly what sets this
distribution. The bands are now identical on both machines, which they were not before.

## 6. The commute is not multi-modal, so the walking it generates is missing

The commuters who do not walk currently make **no trip at all**. What the model should produce for
them is the walked part of a multi-modal journey — home to the boarding stop, alighting stop to work
— which is also the pedestrian volume that concentrates around stations and that nothing else here
can generate. The transit module (`TransitStop`, `TransitVehicle`, the mode split in
`ActivityAgent.planTrip`) is the half that already exists.

~~Related, and smaller: **mode choice is evaluated only on a trip chain's first leg.**~~ **Fixed
16 Sep 2026.** The transit split was the tail of `planTrip()`, which `startChainedTrip` never reaches
because it plans its own route; it is now `ActivityAgent.applyModeChoice()` and both paths call it,
so a five-kilometre second activity is no longer walked by construction.

**A second leak went with it, and it was the larger of the two.** `egressStop` had no writer that
cleared it: the vehicle removes the agent from `agentTransitDestinations` on alighting and
`boardingStop` is nulled at the platform, but the egress stop stayed set for the rest of the agent's
life — and the mode split is gated on `egressStop == null`. **So one transit journey made every later
leg of that agent's day a walk, whatever its length.** `clearTransitLegState()` now runs at the start
of each leg, in `startChainedTrip` and in `startWalkingAlone`, which makes that guard mean "this leg
has not already been split" rather than "this agent has never taken transit".

Unmeasured, and worth knowing before reading a mode split: `countTrip("WALK")` now fires per walked
*leg* rather than per first leg, so the walk count rises and the reported mode shares move without
any behaviour changing. Neither figure was ever compared against anything.

## 7. ~~The `!isDark()` guard on commuting~~ — removed 14 September 2026

`ActivityAgent.shouldGoToWork` no longer consults `isDark()`, and
`planMandatoryDeparture` no longer refuses to draw a departure into darkness. Darkness was never a
reason not to go to work: the persona's start window says when somebody sets off, and the season says
whether it is light when they do. The guard had been deleting the winter commute — Turin's sunset is
before 17:00 through December.

**It changes results, in two places.** Mandatory legs now occur in winter darkness, so more of the
day's leg budget goes to commuting and fewer discretionary chains are bought; and night aggregates
over `[20:00, 06:00)` now contain commutes in winter, which they did not before. A summer day is
unaffected, which is why nothing showed in the June runs.

## 8. Opening windows and stay durations: the mechanism is there, the data is not

`ActivityPurpose` holds open hour, close hour, mean stay and log sigma for each of eight purposes, and
`DepartureProfile` reads the windows straight off the enum. They are city parameters with no city
behind them — the enum says 11:00–23:00 for dining everywhere, which is not an Italian day.

**Half of this is fixed (14 Sep 2026).** A city file may now set any of the four through
`purpose.<NAME>.open` / `.close` / `.stayMinutes` / `.staySigma`; `CityConfig` applies and reports
them like every other key, and `ActivityPurpose.resetToDefaults()` runs first so a second city in one
JVM cannot inherit the first's hours. The enum values are now defaults rather than facts.

**What is left is the data, and it is deliberately not invented.** `Torino.properties` carries the
key block commented out, because filling it with plausible-sounding Italian hours would only move the
invention somewhere that looks sourced. It wants either an aggregation of OSM `opening_hours` for
Turin — **nothing in the pipeline reads that tag**, so this is pipeline work — or a local
retail/hospitality schedule.

## 9. Worker/student overlap — closed as an assumption, 14 September 2026

The 2021 permanent census publishes no enrolment variable at section level, so students are the 15-24
age band while ISTAT P101 counts everyone employed at 15-64: the employed young were both, and the
residual borrowed them from flex. `Persona.sample` now thins the student share by
`ActivityPars.youthEmploymentRate`, leaving the employed young among the workers — the side they
belong on, since a job means a commute and the commute is what this model simulates.

**The residue is now one number instead of a silent double count.** `youthEmploymentRate = 0.18` is a
national order of magnitude, not a Turin figure: ISTAT's 15-24 employment rate for Italy has run in
the high teens recently, Piedmont sits above the national rate, and a city rate would be higher
still. **Check it, and set the local value in `Torino.properties`** — and if a per-zone employed-15-24
count ever becomes available, it supersedes the parameter entirely. Wiring:
`01_census_istat.py` → `CensusZone` → `Persona.sample`.

## 10. Smaller, and deliberately not done

- **A city with no WORK tags gets no decay at all.** Core's `selectRandomNode` ends the workplace
  ladder with a uniform draw, so on such a city commutes come out far longer than they should.
- **Lunch trips from work** — splitting the mandatory stay around a midday dining leg adds agenda
  complexity for little effect at pedestrian scale. The work stay stays whole.
- **The workplace draw now follows the POI and building layers' row order.** The attraction maps are
  `LinkedHashMap`s, which is what makes a run reproducible across machines, and their insertion order
  is the order `PoiClassifier` reads those layers in. So re-exporting `<City>_POIs.gpkg` with the rows
  in a different order is a change to the model's output, with nothing in the run to say so. It is the
  cheaper half of the trade — the alternative was an order that differed per JVM build — but a POI
  layer is now a versioned input, not just a set of points.
- **An agent's known network can still differ across machines**, through
  `Islands.mergeConnectedIslands`: the library re-wraps the edge set it is handed into a `HashSet`,
  so which bridge joins two islands of an agent's known space is identity-hash ordered. It reaches
  this tier and learning, not night. Detail and the proposed library fix are in `../core/TODO.md`
  item 5.
- **Only `ActivityPars` and `Pars` keys are exercised by the city file.** `NightPars`,
  `LearningPars`, `SocialPars`, `TestPars` and `EmpiricalPars` have no city files at all.

---

## Interactions worth knowing

- **Night** agents get personas, purposes and chaining — their populate goes through
  `ActivityPopulate.defineHomeWorkLocations` — and use the same destination choice as everyone else,
  with a park/water refusal applied to the result.
- **Learning** agents are `ActivityAgent`s, so everything in this file applies to them: personas,
  agendas, purposes, destination choice, the commute and its mode choice. Their own memory decay
  reads `STEP_DURATION`, so a finer step rescales it automatically.
- **Behavioural `isDark` is seasonal; the exporters' day/night aggregation window is fixed.** The
  divergence is intentional, so that aggregates stay comparable across dates.
- **Home and work assignment goes through `NodesLookup`**, so it needs GeoMason-light 2.2.0 to be
  seeded. Seeded runs do not replay traces from before that change.
