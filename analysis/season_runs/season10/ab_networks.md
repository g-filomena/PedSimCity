# Season10: Torino against Torino_sidewalks

Same seeds, dates and size (20% of residents, 3 days, 5 seeds x 4 seasons) on both networks. Figures are means over the 20 runs of each network unless stated.

## 1. Walked metres per 216 m cell by distance from Piazza Castello, relative to the 4-5 km ring

| ring (m) | Torino | Torino_sidewalks | Vodafone passing |
|---|---|---|---|
| 0-500 | 4.02 | 3.91 | 6.90 |
| 500-1000 | 3.53 | 3.42 | 5.92 |
| 1000-1500 | 2.87 | 2.81 | 5.23 |
| 1500-2000 | 2.13 | 2.09 | 3.06 |
| 2000-3000 | 1.87 | 1.85 | 2.05 |
| 3000-4000 | 1.42 | 1.41 | 1.53 |
| 4000-5000 | 1.00 | 1.00 | 1.00 |
| 5000-6000 | 0.67 | 0.67 | 0.79 |
| 6000-8000 | 0.47 | 0.47 | 0.61 |
| 8000-20000 | 0.16 | 0.17 | 0.32 |

Spearman, walked metres per cell against Vodafone passing presence (whole day), mean ± sd over the 5 seeds:

| season | Torino | Torino_sidewalks |
|---|---|---|
| winter | 0.791 ± 0.002 | 0.789 ± 0.003 |
| spring | 0.782 ± 0.001 | 0.781 ± 0.003 |
| summer | 0.770 ± 0.003 | 0.769 ± 0.003 |
| autumn | 0.785 ± 0.002 | 0.784 ± 0.002 |

## 2. Walked km per run by the role of the edge (roles from Torino_streets.csv)

| role | Torino | share | Torino_sidewalks | share |
|---|---|---|---|---|
| carriageway | 62,504 | 67.9% | 56,393 | 64.6% |
| side_road | 10,625 | 11.5% | 9,949 | 11.4% |
| sidewalk | 10,684 | 11.6% | 12,495 | 14.3% |
| pedestrian_street | 1,225 | 1.3% | 1,196 | 1.4% |
| crossing | 2,425 | 2.6% | 2,716 | 3.1% |
| path | 4,609 | 5.0% | 4,604 | 5.3% |
| total | 92,071 | | 87,352 | |

## 3. Walked km per run by street (carriageways, side roads and sidewalks together)

768 streets with at least 20 km a run on Torino. Spearman between the two networks: 0.992. Median ratio Torino_sidewalks / Torino: 0.87 on the 69 streets that lost at least 300 m of carriageway, 0.96 on the 508 that lost none.

The 15 busiest streets on Torino:

| street | Torino | Torino_sidewalks | ratio | carriageway removed (km) |
|---|---|---|---|---|
| Corso Unione Sovietica | 1,350 | 1,275 | 0.94 | 0.04 |
| Via Nizza | 1,191 | 1,089 | 0.91 | 1.24 |
| Corso Francia | 1,116 | 1,072 | 0.96 | 0.00 |
| Corso Regina Margherita | 980 | 931 | 0.95 | 0.01 |
| Corso Vittorio Emanuele II | 926 | 893 | 0.96 | 0.36 |
| Corso Peschiera | 923 | 860 | 0.93 | 0.74 |
| Corso Orbassano | 910 | 849 | 0.93 | 0.01 |
| Corso Giulio Cesare | 845 | 782 | 0.93 | 0.12 |
| Via Monginevro | 728 | 644 | 0.88 | 0.79 |
| Corso Racconigi | 646 | 564 | 0.87 | 0.95 |
| Corso Giovanni Agnelli | 614 | 568 | 0.92 | 0.02 |
| Corso Grosseto | 612 | 560 | 0.91 | 0.23 |
| Corso Bramante | 581 | 535 | 0.92 | 0.03 |
| Corso Trapani | 579 | 555 | 0.96 | 0.00 |
| Via Filadelfia | 551 | 538 | 0.98 | 0.01 |

Largest losses and gains among streets that lost at least 300 m of carriageway:

| street | Torino | Torino_sidewalks | ratio | carriageway removed (km) |
|---|---|---|---|---|
| Corso Giuseppe Siccardi | 29 | 18 | 0.60 | 0.52 |
| Via Gioachino Rossini | 129 | 82 | 0.64 | 0.65 |
| Via Giuseppe Mazzini | 69 | 49 | 0.72 | 0.77 |
| Via Antonio Giuseppe Bertola | 169 | 122 | 0.72 | 0.89 |
| Corso Bolzano | 72 | 54 | 0.74 | 0.62 |
| Via della Consolata | 83 | 61 | 0.74 | 0.49 |
| Via Tommaso Agudio | 42 | 40 | 0.95 | 0.30 |
| Viale Primo Maggio | 67 | 63 | 0.95 | 0.47 |
| Corso Francesco Ferrucci | 343 | 328 | 0.96 | 0.42 |
| Corso Vittorio Emanuele II | 926 | 893 | 0.96 | 0.36 |
| Via Giuseppe Baretti | 34 | 34 | 1.03 | 0.31 |
| Via San Domenico | 57 | 59 | 1.03 | 0.42 |
