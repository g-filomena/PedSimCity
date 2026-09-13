# What to work on next

Open work lives per module. This file is the index plus the few things that belong to no single one.
What has already been done is in `CHANGELOG.md`; how the code works is in `CLAUDE.md`.

Each module file leads with what invalidates earlier output from that module — read that before
trusting an old result.

| module | what is in it |
|---|---|
| [core](src/main/java/pedsim/core/TODO.md) | the remote-run route has no caller; escalation length error; `departuresPerPersonPerDay`; the invariants not to break |
| [activity](src/main/java/pedsim/activity/TODO.md) | layer 2's invented interior and two microdata requests to start now; `distanceWeight`; `choiceSetRadiusMetres`; the lost commuting check; the long commute tail |
| [night](src/main/java/pedsim/night/TODO.md) | **read before running experiments** — what to commit or ship first, A/B design, threats to validity, void comparisons |
| [cityImage](src/main/java/pedsim/cityimage/TODO.md) | "Testing Landmarks" runs on no bundled city; London's two networks; `Pars.jobs` reset |
| [empirical](src/main/java/pedsim/empirical/TODO.md) | only Muenster has cluster data; what not to re-add after `PopulationPars` |

Learning and social have no list of their own; they inherit the activity tier's.

## Cross-cutting

- **Nothing from 12–13 September 2026 is committed.** ~100 files here, plus ten modified and the whole
  new `src/test/` in `GeoMason-light`. Both compile; `mvn -Pall-modules compile` is clean and
  GeoMason-light's 163 tests pass. The CHANGELOG follows the natural commit seams.
- **Publish GeoMason-light 2.2.0, or stamp the rebuild.** One version number currently covers two
  different builds, and nothing detects the difference. Affects every module and every machine;
  detail in the core file.
- **Consolidate London's two networks**, or flatten them into two cities. Detail in the cityImage
  file; it is a data decision, not a code one.
