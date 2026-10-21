# Activity module (`pedsim.activity`)

The **activity-based foundation** for modules that run a full 24-hour daily routine. Activity sits
between `core` (pure infrastructure) and the routine modules (`night`, `learning`, `social`), adding
what those modules share: a census-driven **population**, an **activity programme** for each person,
**destination choice** over purpose-typed attraction, **mode choice** for every leg, and a 24-hour
**day/night clock**.

Activity is *neutral* about perception and safety: it has no vulnerability and no street-lighting
behaviour. A plain activity run models residents who follow **personas** (worker / student /
retiree / flex) through home-based **trip chains**, plus the people who come into the city every
day to work or study, and the night's **tourists**.

## Relationship to core

| Concern | Core class | Activity class |
|---|---|---|
| Simulation state + activity data | `PedSimCity` | `PedSimCityActivity` (census, purpose weights, `isDark`, day counters) |
| Engine wiring + 24h clock | `Engine` | `ActivityEngine` (sets `isDark` from `Daylight` each step) |
| Data import | `Import` | `ActivityImport` (census, POIs, employment) |
| Environment preparation | `Environment` | `ActivityEnvironment` (census zones, `PoiClassifier`, employment, stops, car parks, tourists) |
| Population | `Populate` | `ActivityPopulate` (homes, workplaces, personas, inbound commuters, tourists, car-park spaces) |
| Travel demand | `BaselineTravelDemand` | `ActivityTravelDemand` (mandatory departures, the day's leg budget, `DepartureProfile`) |
| Agent | `Agent` | `ActivityAgent` (persona, agenda, chaining, destination and mode choice, rides) |

Packages: `agents` (`Persona`, `ActivityPurpose`, `DailyAgenda`, `TouristDay`,
`DestinationChoice`, `WorkplaceChoice`, `DepartureProfile`), `engine` (the classes above plus
`Daylight`, `Weather`, `InboundCommuters`, `Tourists`, `DaySummary`), `mobility` (`ModeChoice`,
`CommuteMode`, `Ride`, `TransitStops`, `CarParks`), `parameters` (`ActivityPars`, `CityConfig`).

## A day

1. **Clock.** `PedSimCityActivity.isDark` comes from `Daylight`: sunrise and sunset for the date at
   the city's position, measured from the street network, converted to clock time through
   `ActivityPars.timeZoneId`. A network with no usable CRS falls back to the fixed
   `dayStartHour`/`nightStartHour` window, and the run says which regime is in force. The exporter
   splits its volumes by the same model.
2. **Population.** Each resident's home is drawn from residence-weighted census zones, and its
   persona from the home zone's shares (`retiree_pct`, `student_pct`, `worker_pct`; flex is the
   residual). Workers get a workplace drawn by employees (or WORK attraction) with distance decay,
   students an education node.
3. **Mandatory travel.** Everyone with a workplace whose persona attends that day gets a departure
   minute inside the persona's start window, and goes (`planMandatoryDeparture`). How the commute
   is made is settled once per person (`ModeChoice.commute`): walked with the probability the
   ISTAT-fitted commute curve gives at its distance, otherwise by transit, car or a mode that puts
   no one on the street.
4. **Lunch.** On a working day that spans the break, a worker goes out to a dining destination near
   the workplace (`lunchOutShareWorker`), or home by the commute's mode when the break leaves time
   to eat there (`lunchHomeShareWorker`), or stays.
5. **Discretionary travel.** The day's legs are `tripsPerPersonPerDay` (2.04, ISFORT, every mode)
   per resident; the commutes take theirs and the rest is bought as trip chains, released along
   `DepartureProfile` with persona-by-hour affinities and, on rainy days (`Weather`), fewer of them.
   A chain is a `DailyAgenda` of purposes open at that hour; after each stay the agent chains to the
   next stop from where it is, and goes home when the agenda is done.
6. **Destinations.** `DestinationChoice` is a logit over 60 nodes drawn within
   `choiceSetRadiusMetres` among those that hold something for the purpose:
   `sizeWeight * ln(attraction) - distanceWeight * metres + habitWeight * known`. Agents keep a
   familiar set of at most 25 places and return to them by exploration / preferential return
   (Song et al. 2010; Alessandretti et al. 2018).
7. **Mode.** Every outbound and chained leg crosses `walksLeg`: walked with the probability the
   general walk-share curve gives at its length. A leg not walked goes by public transport, by car,
   or off the street; see below. The way home is walked if the agent went out on foot, and made as
   the outing was otherwise (`walksBack`).

## Legs that are not walked

`ModeChoice` decides what follows a no, by the shares in `ActivityPars` (zero unless the city file
states them). Each mode is a `Ride`: the agent is `AWAY` for the ride and reappears on the street
where it ends.

- **Public transport.** The agent walks to a stop within `transitAccessRadiusMetres` of its origin,
  drawn in proportion to the stop's weekday departures (`<City>_transit_stops.csv`), waits half the
  headway, rides at `transitSpeedKmh`, and walks from a stop near its destination. After an outing
  by transit it goes home by transit when stops serve both ends.
- **Car.** A car driven to a destination inside the restricted zone
  (`<City>_restricted_zone_nodes.csv`) is left in a parking structure within `carParkRadiusMetres`
  and the rest is walked; the way home starts with the walk back to it. Elsewhere the car is left at
  the door and the journey puts no one on the street. Commuters take a space for the day within the
  structures' capacity (`<City>_parking.csv`); a shorter visit needs a space they left free.
- **Anything else** (bicycle, motorbike, a car outside the zone) puts no one on the street.

`trip_diagnostic.csv` names each walked leg's role: `walk`, `to_stop`, `from_stop`, `to_car` or
`from_car`.

## People who do not live in the city

Both groups are added after the residents, so they leave the residents' draws unchanged, and both
are outside the residents' leg budget, statistics and discretionary release
(`ActivityAgent.isResident`).

- **Inbound commuters** (`InboundCommuters`): everyone who lives elsewhere and works or studies in
  the city, from `<City>_inbound_commuters.csv`, built when their journey reaches the street. By
  train they arrive at the station nearest the job among those their origin's trains call at
  (`<City>_rail_reach.csv`); by tram, metro or bus at a stop near the job; by car at a car park when
  the job is in the restricted zone. Departure and journey times come from the file's minute
  windows.
- **Tourists** (`Tourists`, `TouristDay`): the night's tourists are the month's presences
  (`touristNightsByMonth`) over its days, each sleeping at a node drawn by beds
  (`<City>_accommodation.csv`). A day is two outings: sights until late afternoon with lunch on
  the way, back to the accommodation, then dinner and, for `touristNightlifeShare` of them, a bar
  or a show. Sights are the `SIGHTSEEING` purpose, weighted by yearly visitors
  (`<City>_sights.csv`), looked for within `touristChoiceSetRadiusMetres`, never revisited. A
  tourist has no car: a leg it does not walk goes by public transport where stops serve both ends,
  by taxi otherwise.

## Data layers

All optional, all under `src/main/resources/<City>/`. A city without one runs without the mechanism
it feeds, and the run log says so.

| File | Written by | Feeds |
|---|---|---|
| `<City>_censusData.gpkg` | `pipeline/01_census_istat.py` | homes, headcount, sex and persona shares per census zone |
| `<City>_POIs.gpkg`, tags on `<City>_buildings.gpkg` | `pipeline/00_city_preparation.py` (`pois`) | attraction per purpose (`PoiClassifier`) |
| `<City>_employment.gpkg` | `pipeline/employment_istat.py` | WORK attraction: employees per census section |
| `<City>_transit_stops.csv` | `scripts/build_transit_layer.py` | stops and their weekday departures |
| `<City>_parking.csv`, `<City>_restricted_zone_nodes.csv` | `pipeline/parking.py` | car parks with capacity; where a car is parked in one |
| `<City>_inbound_commuters.csv` | `pipeline/inbound_commuters_istat.py` | people who come in to work or study |
| `<City>_rail_stations.csv`, `<City>_rail_reach.csv` | `pipeline/rail_reach.py` | the stations a commuter's trains reach |
| `<City>_accommodation.csv` | `pipeline/accommodation.py` | beds per node, where tourists sleep |
| `<City>_sights.csv` | `pipeline/sights.py` | SIGHTSEEING attraction, visitors per sight |
| `<City>.properties` | by hand | the city's behavioural facts, with their sources (`CityConfig`) |

The census columns:

| Column | Type | Purpose |
|---|---|---|
| `residence_pct` | share | residence-weighted home selection (0 for non-residential zones) |
| `residents` | count | absolute headcount; when present the population size is census-driven |
| `female_pct` | rate [0,1] | share of adult residents who are women; each agent's sex is drawn from it |
| `retiree_pct` | rate [0,1] | share of adult residents 65+ |
| `student_pct` | rate [0,1] | share of adult residents 15-24 |
| `worker_pct` | rate [0,1] | share of adult residents employed (ISTAT P101) |

Each zone claims the network nodes within 50 m (growing to 100/200/400 m if none). Per-zone rates
are read from the home zone an agent was sampled from, not from the node it was placed on.

`PoiClassifier` reads the value-bearing keys `amenity`, `use`, `fclass`, `type`, `building`,
`landuse`, `land_use` and the presence-only keys `shop`, `leisure`, `tourism`, `sport`, `office`,
classifies each feature into `ActivityPurpose`s and snaps it to its nearest node within
`ActivityPars.poiClaimRadius`. A POI weighs 1.0; a building weighs its footprint in units of
`buildingAreaPerAttractionUnit` (200 m2, at least 1.0), for each purpose in its `land_uses` list.

## For module authors

A new 24h-routine module should extend the activity tier, not core:

- state → `extends PedSimCityActivity`
- engine → `extends ActivityEngine`
- populate → `extends ActivityPopulate` (non-residents go through `createNonResidentAgent` /
  `finishNonResidentAgent`)
- agent → `extends ActivityAgent`

and add only its own layer on top. See `night` (vulnerability and lighting) and `learning`
(incremental cognitive map). Clear your module's static data in the engine's `clearStaticData()`
(activity's is cleared by `ActivityEngine`).

## Running

```bash
mvn compile exec:java@activity-website   # REST API + browser dashboard
mvn compile exec:java@activity           # headless (ActivityLauncher)
```

Start a run via REST once the server is up:

```bash
curl -X POST http://localhost:8081/api/start \
  -H "Content-Type: application/json" \
  -d '{"module":"activity","cityName":"Torino","days":1,"jobs":1}'
```

Torino carries every layer above and a city file. Other bundled cities run without the layers
they lack.
