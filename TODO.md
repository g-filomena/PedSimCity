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
| [cityImage](src/main/java/pedsim/cityimage/TODO.md) | **landmark routing may have found nothing at all** until 14 Sep, and is still unexercised; "Testing Landmarks" runs on no bundled city; London's two networks; `Pars.jobs` reset |
| [empirical](src/main/java/pedsim/empirical/TODO.md) | **the region-subgraph avoid-set fix is unexercised**; only Muenster has cluster data; what not to re-add after `PopulationPars` |
| [learning](src/main/java/pedsim/learning/TODO.md) | **it ran for the first time on 14 Sep** — three fixes, the decay-threshold decision still open, and what makes it slow |

Social has no list of its own; it inherits the activity tier's.

## Cross-cutting

- **Cross-machine reproducibility is CLOSED (14 Sep 2026).** A seed now replays on any machine:
  core, night and activity give byte-identical per-leg traces between `gdsl1` and the Windows laptop
  on seed 20260912. It needed two things and neither is sufficient alone - core's release draw and
  agent scheduling ordered deterministically, *and* GeoMason-light 2.2.1's `hashCode` on `NodeGraph`
  and `EdgeGraph`. Mechanism and evidence in `CLAUDE.md`. **Earlier comparisons are still void**, and
  every run made before this date is superseded.

- **Two region-subgraph fixes on 14 Sep are still unexercised.** `Dijkstra.subGraphInitialisation`
  and `DijkstraGlobalLandmarks.findBestLandmarkness` both land on cityImage and empirical, which sit
  outside the default build profile and were never run. They compile under `-Pcityimage-empirical`
  and that is all that has been checked. What each module needs, and what of its earlier output is
  suspect rather than merely superseded, is now in its own file:
  [cityImage](src/main/java/pedsim/cityimage/TODO.md) and
  [empirical](src/main/java/pedsim/empirical/TODO.md).

- **Nothing from 12–14 September 2026 is committed.** Both trees compile; `mvn -Pall-modules compile`
  and `-Pcityimage-empirical` are clean here, and GeoMason-light's 171 tests pass. The CHANGELOG
  follows the natural commit seams.
- **Commit the spotless sweep as its own commit, then add `.git-blame-ignore-revs`.** The tree is
  formatted and `HEAD` is not, so the formatting sits uncommitted across 58 Java files, mixed into
  the working tree. Leaving it there does not avoid the noise — it makes it permanent, because every
  `git status` and `git diff` carries it and the pre-commit hook re-applies it after each checkout.
  Committing it once ends it: `spotless:check` already reports 158 files clean, so no later commit
  has any formatting left to add. Of the 58 files exactly **one** — `TransitVehicle.java`, the
  `countTrip` refactor — also carries a real change, so the split is nearly free. A worktree at
  `E:/tmp/fmt` is already formatted and ready to commit for this; the sequence is in the session
  notes, and **the push is the user's to make, not an agent's**.
- **Set `<ratchetFrom>origin/main</ratchetFrom>` in the spotless config.** `spotless:apply` formats
  the whole tree while the pre-commit hook re-stages only what you staged, so a single new
  non-conforming file drags every other file it touches into your working tree as whitespace churn.
  That is what produced the 58-file spread above. The ratchet confines it to files that actually
  changed. Do it before the next new file lands, or this recurs.
- **Prune the stale worktree.** `git worktree list` shows
  `C:/Users/gfilo/OneDrive - .../pedsimcity-clean` marked `prunable`, left from the
  `rest-module-restructure-integrated` branch. `git worktree prune` once it is confirmed dead.
- **Publish GeoMason-light 2.2.1.** 2.2.0 went to Maven Central on 14 Sep and Central is immutable,
  so everything since is 2.2.1: the `hashCode` above (measured to be load-bearing, not hygiene), the
  `intersectingFeatures` ordering fix, the `Route.computeRouteSequences` guard and its list-clearing
  fix, and the new tests. 171 tests pass. The 2.2.0 one-version-two-builds hazard is closed - the
  published jar is byte-identical to the local `.m2` copy.
- **Consolidate London's two networks**, or flatten them into two cities. Detail in the cityImage
  file; it is a data decision, not a code one.
