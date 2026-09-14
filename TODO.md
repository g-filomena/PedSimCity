# What to work on next

Open work lives per module. This file is the index plus the few things that belong to no single one.
What has already been done is in `CHANGELOG.md`; how the code works is in `CLAUDE.md`.

Each module file leads with what invalidates earlier output from that module — read that before
trusting an old result.

| module | what is in it |
|---|---|
| [core](src/main/java/pedsim/core/TODO.md) | the remote-run route has no caller; escalation length error; `departuresPerPersonPerDay`; the rest of the cross-machine divergence; `RoutePlanner` on unconfigured properties; the invariants not to break |
| [activity](src/main/java/pedsim/activity/TODO.md) | **its commute figures are superseded** — layer 2's invented interior and two microdata requests to start now; `distanceWeight`; `choiceSetRadiusMetres`; the lost commuting check; the long commute tail |
| [night](src/main/java/pedsim/night/TODO.md) | **read before running experiments** — what to commit or ship first, A/B design, threats to validity, void comparisons |
| [cityImage](src/main/java/pedsim/cityimage/TODO.md) | "Testing Landmarks" runs on no bundled city; London's two networks; `Pars.jobs` reset |
| [empirical](src/main/java/pedsim/empirical/TODO.md) | only Muenster has cluster data; what not to re-add after `PopulationPars` |
| [learning](src/main/java/pedsim/learning/TODO.md) | **it ran for the first time on 14 Sep** — three fixes, the decay-threshold decision still open, and what makes it slow |

Social has no list of its own; it inherits the activity tier's.

## Cross-cutting

- **A seed does not fully reproduce a run across machines.** `gdsl1` and the Windows laptop each
  replay themselves exactly and disagree with each other by about 1% of trips, on the same seed, code,
  data and jar. The cause is identity-hash iteration order, not floating point, and the population
  layer was closed on 14 Sep; the remaining 1% is downstream, in destination choice or routing, and
  has not been found. Until it is, **a comparison has to be run entirely on one machine**, and the
  machine belongs in the write-up beside the seed. The numbers are in the night file; what has been
  swept, what is still suspect and the one-line fix that would close the class of bug are in the core
  file, item 5.

- **Nothing from 12–14 September 2026 is committed.** ~100 files here, plus ten modified and the whole
  new `src/test/` in `GeoMason-light`. Both compile; `mvn -Pall-modules compile` is clean and
  GeoMason-light's 163 tests pass. The CHANGELOG follows the natural commit seams.
- **Publish GeoMason-light 2.2.0, or stamp the rebuild.** One version number currently covers two
  different builds, and nothing detects the difference. Affects every module and every machine;
  detail in the core file.
- **Consolidate London's two networks**, or flatten them into two cities. Detail in the cityImage
  file; it is a data decision, not a code one.
