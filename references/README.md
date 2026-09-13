# Sources behind the calibration parameters

The reports are stored here rather than linked. A parameter whose justification is a URL has no
justification once the URL moves or the report is superseded.

| file | settles | used by |
|---|---|---|
| `ISTAT_2017_spostamenti_quotidiani.pdf` | Walking share of commuting: 12.0% of `occupati`, 27.9% of `scolari e studenti`, 17.4% of all commuters, 14.8% of employed women (Figura 3, "per 100 persone con le stesse caratteristiche") | `ActivityPars.walkShareCommuteWorker`, `walkShareCommuteStudent` |
| `ISFORT_2024_21_rapporto_mobilita.pdf` | Distance classes of Italian daily travel (proximity ≤2 km, urban 2–10 km, medium 10–50 km, long >50 km) and overall modal shares, walking at 19–21% of all trips | Context for `TripDistanceBands` |
| `ISFORT_2025_22_rapporto_mobilita.pdf` | Systematic share of trips (repeated at least 3-4 days a week): 57.5% in 2023, 59.5% in 2024, 58.4% in H1 2025 (Graf. 20). Weekend trips, where work/study is only 13.7%, are systematic in 35.2% of cases (Tab. 13). Tab. 18 of the 21st report crosses motive with systematicity over three worker clusters, which separates the work/study rate from the discretionary one | Floor for the destination-reuse probability |
| not stored (not redistributable) - Song et al. (2010) | Exploration vs preferential return: `P_new = rho * S^-gamma`, gamma = 0.21 +/- 0.02, rho normal with mean 0.6; returns are drawn in proportion to past visits | `ActivityPars.explorationRho`, `explorationGamma` |
| not stored (not redistributable) - Alessandretti et al. (2018) | The number of familiar locations a person holds at any moment is conserved at about 25, across ~40,000 individuals and four datasets, with turnover in which places they are | `ActivityPars.familiarLocationCapacity` |
| not stored (not redistributable) - Watson et al. (2021) | Walking trip length by purpose, 2017 NHTS, 54,034 trips: distances not significantly different by purpose; durations 11.9 min overall, 14.6 work, 13.5 shopping, 12.9 other/errands, 11.7 social-recreation - a 1.15x spread. Settles that no purpose scaling of trip distance is warranted | why `ActivityPurpose` carries no `tripDistanceFactor` |

## Limits

ISFORT publishes distance classes and modal shares separately, never crossed. There is no
published table giving the walking share of, say, a 2 km trip, which is what a distance-dependent
mode filter needs. That requires the Audimob microdata, obtained by request from ISFORT.

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

See `RELEASE_BUDGET.md` for how each figure enters the model.
