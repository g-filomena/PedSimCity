# Sources behind the calibration parameters

The reports themselves are not redistributed. Each is named below by the file it is cited under,
and *Citations* gives the full reference and where it is published.

| file | settles | used by |
|---|---|---|
| `ISTAT_2017_spostamenti_quotidiani.pdf` | Walking share of commuting: 12.0% of `occupati`, 27.9% of `scolari e studenti`, 17.4% of all commuters, 14.8% of employed women (Figura 3, "per 100 persone con le stesse caratteristiche") | `ActivityPars.walkShareCommuteWorker`, `walkShareCommuteStudent` |
| `ISFORT_2024_21_rapporto_mobilita.pdf` | Distance classes of Italian daily travel (proximity ≤2 km, urban 2–10 km, medium 10–50 km, long >50 km) and overall modal shares, walking at 19–21% of all trips | Context for the walk-share curve and `tripsPerPersonPerDay` |
| `ISFORT_2025_22_rapporto_mobilita.pdf` | Systematic share of trips (repeated at least 3-4 days a week): 57.5% in 2023, 59.5% in 2024, 58.4% in H1 2025 (Graf. 20). Weekend trips, where work/study is only 13.7%, are systematic in 35.2% of cases (Tab. 13). Tab. 18 of the 21st report crosses motive with systematicity over three worker clusters, which separates the work/study rate from the discretionary one | Floor for the destination-reuse probability |
| not stored (not redistributable) - Song et al. (2010) | Exploration vs preferential return: `P_new = rho * S^-gamma`, gamma = 0.21 +/- 0.02, rho normal with mean 0.6; returns are drawn in proportion to past visits | `ActivityPars.explorationRho`, `explorationGamma` |
| not stored (not redistributable) - Alessandretti et al. (2018) | The number of familiar locations a person holds at any moment is conserved at about 25, across ~40,000 individuals and four datasets, with turnover in which places they are | `ActivityPars.familiarLocationCapacity` |
| not stored (not redistributable) - Watson et al. (2021) | Walking trip length by purpose, 2017 NHTS, 54,034 trips: distances not significantly different by purpose; durations 11.9 min overall, 14.6 work, 13.5 shopping, 12.9 other/errands, 11.7 social-recreation - a 1.15x spread. Settles that no purpose scaling of trip distance is warranted | why `ActivityPurpose` carries no `tripDistanceFactor` |
| `Federdistribuzione_2020_mercato_buoni_pasto.pdf` | Where employed people in Italy eat lunch, ISTAT data, thousands (p. 8): at home 11,643, canteen 2,219, restaurant or trattoria 1,512, bar 994, at the workplace 5,098 | `ActivityPars.lunchOutShareWorker`, `lunchHomeShareWorker` (Torino) |
| `ISTAT_AVQ_2018-19_pasti_per_regione_Annuario_VdA_2020_tav21-39.pdf` | ISTAT *Aspetti della vita quotidiana*, 2018-2019, people aged 3+, by region: lunch at home, canteen, restaurant, bar, workplace. Piemonte against Italia: restaurant 3.2 / 2.85, bar 2.75 / 1.95, home 66.9 / 72.15 | scales the national lunch shares to Piedmont |
| `Regione_Piemonte_2023_turismo_in_Piemonte_nel_2022.pdf` | Osservatorio Turistico: presences in the city of Turin 2013-2022, 3,862,976 in 2022 (p. 20); Torino e prima cintura 5,004,318 | the city's share of the area's presences, 77.2% |
| `Regione_Piemonte_2024_turismo_in_Piemonte_nel_2023.pdf` | Monthly presences of Torino e prima cintura, 2022 and 2023 (p. 28) | `ActivityPars.touristNightsByMonth` (Torino) |

## Limits

ISFORT publishes distance classes and modal shares separately, never crossed. There is no
published table giving the walking share of, say, a 2 km trip, which is what a distance-dependent
mode filter needs. That requires the Audimob microdata, obtained by request from ISFORT. Checked
against the 22nd report: length classes are Graf. 14 (p. 19) and modal shares Graf. 21-22 (p. 23),
two distributions over the same trips with no cell in common.

**Audimob's trip universe excludes walking trips under five minutes**, which is about 420 m at
pedestrian speed. The definition is on p. 13: the mobile population is everyone who travelled
yesterday *"ad eccezione dei soli spostamenti a piedi di durata inferiore a 5 minuti"*, and people
whose whole day is such walks are counted as non-mobile and reported separately as the 6.7%
*popolazione di prossimità*. This bears on both figures the model takes from ISFORT, not just the
curve: **2.04 trips per resident is a lower bound**, since those walks are missing from it, and the
20.6% walking share is computed on the same reduced universe. A model statistic compared against
either has to drop its own sub-420 m legs first, or it is answering a different question.

Two published quantities are usable as targets as they stand. Walking is **20.6% of trips**
(H1 2025; 19.3% in 2024), which is one calibration target for the general walk-share curve. And
Graf. 14 is an all-mode **trip-length distribution** - about a third of trips under 2 km, 81.3%
under 10 km, 16.4% medium, 2.2% over 50 km - which is the only Italian distribution available to
check layer 3's output against.

ISFORT publishes trip length and trip motive separately too, so there is no Italian figure for the
length of a walk *by purpose*. Watson et al. is the only source found that measures walking trips
resolved by purpose on a sample large enough to separate them, and what it reports is that the
distances do not separate: the spread is 1.15x and shows only in duration. That is why the model
carries no purpose scaling of trip distance at all. The version that would be worth having is a
trip-length distribution per purpose rather than a multiplier, and for Italy that needs the Audimob
microdata.

"Systematic" in Audimob means the trip is repeated at least 3-4 days a week. That is stricter than
"goes somewhere already known": a fortnightly visit to the same bar is habitual reuse and
non-systematic. The ISFORT share is therefore a **floor** for a destination-reuse probability, not
a point estimate.

Both sources are national. Turin has a metro, four tram lines and above-average car ownership, so
its walking share of commuting is plausibly below the national 12%.

## Citations

- ISTAT (2018). *Spostamenti quotidiani e nuove forme di mobilità — Anno 2017*. Statistiche
  Report, 20 pp. https://www.istat.it/it/files/2018/11/Report-mobilit%C3%A0-sostenibile.pdf
- ISFORT (2024). *21° Rapporto sulla mobilità degli italiani — C'è bisogno di una scossa*.
  Osservatorio Audimob, 186 pp.
  https://www.isfort.it/wp-content/uploads/2024/11/RapportoMobilita2024.pdf
- ISFORT (2025). *22° Rapporto sulla mobilità degli italiani - Eppur si muove*. Osservatorio
  Audimob. https://www.isfort.it/wp-content/uploads/2026/01/RapportoMobilita2025_DEF.pdf
- Watson, K.B., Whitfield, G.P., Bricka, S., & Carlson, S.A. (2021). *Purpose-Based Walking Trips
  by Duration, Distance, and Select Characteristics, 2017 National Household Travel Survey.*
  Journal of Physical Activity and Health, 18(S1), S86-S93. doi:10.1123/jpah.2021-0096
- Song, C., Koren, T., Wang, P., & Barabasi, A.-L. (2010). *Modelling the scaling properties of
  human mobility.* Nature Physics, 6(10), 818-823. arXiv:1010.0436
- Alessandretti, L., Sapiezynski, P., Sekara, V., Lehmann, S., & Baronchelli, A. (2018). *Evidence
  for a conserved quantity in human mobility.* Nature Human Behaviour, 2(7), 485-491.
  doi:10.1038/s41562-018-0364-x
- ISTAT. *Gli spostamenti per motivi di studio o lavoro nel 2019 secondo il Censimento permanente
  della popolazione*. Corroborates the 2017 figures; not stored.
  https://www.istat.it/comunicato-stampa/gli-spostamenti-per-motivi-di-studio-o-lavoro-nel-2019-secondo-il-censimento-permanente-della-popolazione/

## Commuters, transport and visitors

Data the pipeline downloads; each script records the source in the city's provenance file.

| source | gives | read by |
|---|---|---|
| ISTAT, *Matrici del pendolarismo* 2011 | every commute by origin, destination, motive, mode, departure band and duration band | `pipeline/inbound_commuters_istat.py`; the Turin transit and car shares in `Torino.properties` (commutes inside the city) |
| ISTAT, *Censimento dell'industria e dei servizi* 2011, `dati-sce_2011.zip` | employees per census section | `pipeline/employment_istat.py` |
| GTT GTFS (urban network) | stops and weekday departures | `scripts/build_transit_layer.py` |
| Regione Piemonte rail GTFS (CC-BY 2.5); SFM GTFS 2014-2016, Mobility Database `tfs-664` | which city stations the trains from each origin call at | `pipeline/rail_reach.py` |
| 5T, `get_pk` (CC-BY); Comune di Torino, `ztl_geo.zip` | parking structures with capacity; the limited-traffic zone | `pipeline/parking_5t_torino.py` |
| Regione Piemonte, register of accommodation, 2020 (Comune di Torino open data, CC-BY) | beds per establishment, two thirds with coordinates | `pipeline/accommodation_torino.py` |
| Osservatorio Culturale del Piemonte, *Relazione annuale 2023-2024* | visitors to the most visited museums in the city: Egizio 1,061,157, Cinema 754,999, Musei Reali 626,321 and six more | `inputData/Torino/sights/visitors.csv` |
| Nomisma for Cirfood, *Osservatorio Buona Pausa Pranzo* (1,200 interviews) | a lunch break out rarely lasts more than 40 minutes | `lunchStayMinMinutes`, `lunchStayMaxMinutes` |

**Limits.** The commuting matrix is from 2011 and the most recent open SFM timetable from 2016.
The museum ranking is from the 2023-2024 report and whether its figures are for 2023 or 2024 is
not confirmed. The accommodation register leaves out short-term lets. ISTAT's lunch-at-home share
counts part-time and home workers, whose day does not span lunch, so for a worker who works through
it the share is an upper bound. Nothing here gives a tourist's day: how many places tourists go to,
how long they stay at each, or when they go back to the accommodation. Those
parameters (`tourist*` in `ActivityPars`) are assumptions.

## Night route choice

Sources for the night module's route cost: how darkness, parks and water, other people and
familiarity weigh against distance. **Read** says how far each was checked: *full* is the whole
paper, *abstract* is the abstract or a published summary only, so the figure should be confirmed in
the paper before it is cited as settling anything.

| file or link | settles | would set | read |
|---|---|---|---|
| `Portnov_Fotios_2024_optimal_illuminance_segmented_regression.pdf` | Reassurance rises with horizontal illuminance and plateaus: the final breakpoint (optimal illuminance) lies between 8.9 and 26 lx depending on location; earlier work found above 10 lx "less than one scale unit" of further gain and above 30 lx less than half | The shape of the darkness term - concave, reaching zero near 10 lx - and the lux at which it does | full |
| [Fotios, Unwin & Farrall 2015](https://journals.sagepub.com/doi/10.1177/1477153514524587) | Review: an optimum near 10 lx for pedestrian reassurance | Corroborates the plateau above | abstract |
| [Fotios, Uttley & Fox 2019](https://journals.sagepub.com/doi/10.1177/1477153517738306); Uttley & Fotios 2017 (daylight-saving natural experiment) | Fewer people walk after dark than in daylight at the same clock time, and more use lit than unlit paths after dark | That darkness acts on whether people walk as well as where - the release side, not route choice | abstract |
| [Basu, Sevtsuk et al. 2023, *Cities*](https://www.sciencedirect.com/science/article/pii/S0264275123003633) | Day and night route-choice models differ; women avoid routes that are not well lit at night | That the lighting weight differs by group and by darkness | abstract |
| [Sevtsuk & Basu 2021, San Francisco GPS](https://www.sciencedirect.com/science/article/abs/pii/S2214367X21000569) | Pedestrians deviate from the shortest path for safer or more pleasant routes, within a threshold | That route choice is a bounded, priced trade-off rather than a forbidden set | abstract |
| [Broach & Dill 2015, Portland GPS](https://www.researchgate.net/publication/336221745_Pedestrian_Route_Choice_Model_Estimated_from_Revealed_Preference_GPS_Data) | Path-size logit on revealed routes: a turn is worth ~50 m, pedestrians walk ~70 m to avoid an unsignalised arterial | The scale of route-attribute costs expressed in metres | abstract |
| ONS 2022, *Perceptions of personal safety and experiences of harassment* | 82% of women feel unsafe walking alone in parks or open spaces after dark, against 42% of men | The relative weight of parks and water for the two groups | summary only - confirm in the ONS bulletin |
| [Gallup 2023 crime poll](https://news.gallup.com/) | Over half of US women are afraid to walk alone at night near home, against 26% of men | Corroborates the group difference | summary only |
| [Malmö focus groups, *Urban Design International* 2020](https://link.springer.com/article/10.1057/s41289-020-00134-6) | After dark, greenery plus darkness produces entrapment and avoidance; the presence of others reassures | The park/water cost and the busy-street discount | abstract |
| [Tong & Bode 2022, *J. R. Soc. Interface*](https://pmc.ncbi.nlm.nih.gov/articles/PMC8984324/) | Review: pedestrians update route decisions en route as they perceive new information | That re-planning is triggered by new information, not re-drawn per street | full (open access) |
| `Filomena_2025_AGILE_navigating_the_night.pdf` | This model's earlier specification: situated reroute is A* from the current position to the destination; a street is busy at or above the 20th percentile of agent counts over non-empty streets. Cites Ferraro (1995) for the reassurance of others' presence, Phillips et al. (2013) for anxiety in unfamiliar places, Fotios et al. (2019) and Basu et al. (2022) for walking faster in the dark | The busy-street definition; the re-plan target | full - the model's own paper, not independent evidence |

**What no source here gives:** the absolute size of the darkness weight - how many metres of
detour a given darkness is worth - and whether vulnerable agents should assume unknown streets are
darker than non-vulnerable ones. The direction of both is supported (women report far more fear
after dark; familiarity raises perceived safety), the magnitude is not. Both need a stated-choice
study or calibration, and until then they are assumptions to state and sweep, not settled values.
