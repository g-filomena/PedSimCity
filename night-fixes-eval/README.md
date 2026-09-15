# night-fixes-eval

Proposed fixes and evaluation code for the night-time pedestrian behavioural
model, prepared by Marcin Wozniak for review by Gabriele Filomena.

## How this folder works

**Nothing outside this folder is touched.** Every file in here is a *copy* of
a file that lives elsewhere in the repo at the same relative path, e.g.:

```
night-fixes-eval/src/main/java/pedsim/night/agents/NightBehaviour.java
  -> proposes changes to
src/main/java/pedsim/night/agents/NightBehaviour.java
```

The rest of the tree on this branch — and everything on `main` — stays
byte-identical to the published repo. Nothing here is meant to be merged
as-is; it's a staging area for evaluating candidate fixes before anyone
decides whether/how to fold them into the real source locations.

To compare a proposed file against the original it mirrors:

```bash
git diff main -- night-fixes-eval/<relative-path> <relative-path>
```

(or just open both side by side — the relative path after
`night-fixes-eval/` is the pointer to the original.)

## Python pipeline fixes (A1, A2, A3, A4, A7, A8, B1 horizon)

`pipeline/` mirrors `lighting.py`, `02_street_lights_torino.py`, `03_street_lights.py`, and
`04_directional_lighting.py` the same way `src/` mirrors the Java files — same convention, same
rule (nothing outside this folder touched). Unlike earlier sessions this environment now has a
real Python install, so these are tested against real Torino data, not just proposed. See
[`PIPELINE_FIXES.md`](PIPELINE_FIXES.md) for what changed and how it was verified.

## Source of the findings

The issues these files address come from an independent audit of the
lighting-computation pipeline and night behavioural layer, written up in
`night_model_issues.html` (shared separately, not part of this repo). Each
proposed fix below is expected to reference the specific finding ID (e.g.
"A3", "C5") it addresses.

## Status

| Finding | Files | What changed |
|---|---|---|
| **C1** | `NightBehaviour.java` | Lighting gate now also requires `min_lux >= threshold`, not just the average — an edge dark in the middle no longer passes on `mean_lux` alone. |
| **C5** | `NightBehaviour.java`<br>`NightPars.java` | Reroute-vs-speed-up split was a flat 50/50, blind to darkness. Now scales from 0.5 up to a new tunable ceiling (`NightPars.maxRerouteProbabilityInDarkness`, default 0.9) as the edge gets darker. |
| **C6** | `NightAgent.java` | Docs only, no logic change. Notes explicitly that night agents never board transit (deliberate — existing transit code is worse than walk-only) and states the direction of the resulting bias. |
| **B1** | `NightPars.java` | Directional-lux statistic default switched `MIN` → `MEAN`. `MIN` over the 12m visibility window collapses to "distance from the nearest lamp" rather than measuring the street; both columns are already written by the pipeline, so this is a config default, not a re-run. Horizon-distance half of B1 (12m → 15m) is untouched — that needs a pipeline re-run this environment can't do. |
| **C2** | `DijkstraRoadDistanceNight.java`, `NightPars.java` | Lux now enters route-planning cost, not just reactive behaviour. A **known** edge's Dijkstra cost scales up toward a new tunable ceiling (`NightPars.maxKnownDarkEdgeCostMultiplier`, default `1.5`) as its `mean_lux` falls below the agent's threshold. Unknown edges are untouched — their darkness stays a situated-reaction matter only, so this can't double-count the same darkness with the reactive layer. |
| **C3** | `NightAgentMovement.java` | Non-vulnerable agents' reroute avoid-set was built from the raw OSM `lit` tag, inconsistent with the continuous-lux gate that actually triggers the reroute. Replaced with a lux-based set (same fallback shape as the C1-fixed gate) built and cached **in the night module**, not by editing `SharedCognitiveMap` (core) — avoids giving core code a dependency on `NightPars`. Vulnerable-agent avoid-set (knowledge-based, not lighting) is untouched, per the register's own scoping. Rebased 15 Sep onto Gabriele's own `a10a108` (a *different* fix to the same method — see Branch section below). |
| **F** — `distanceWeight` | `ActivityPars.java` | Not a register finding ID, but a Section F item ("`distanceWeight` is stale for Torino's real circuity"). `DestinationChoice.choose()` scales distance by `Pars.networkCircuityFactor`, and only the *product* `distanceWeight × networkCircuityFactor` sets choice probabilities. `distanceWeight = 0.0012` was set while circuity sat at a hardcoded 1.41; circuity is now self-measured (1.292 for Torino), so the product silently drifted 8.4% with nobody deciding that. Re-derived by holding the product invariant: `0.0012 × (1.41/1.292) ≈ 0.0013096`. **Consistency fix, not a calibration** — `distanceWeight` still has never been fitted to real data (Audimob is a separate, still-open request). Scoped to Torino: this is one global constant, not a per-city dynamic correction — see the file's own Javadoc for why a live-reference fix (matching the pattern `decideCommuteMode()`/`CommuteCalibration` already use) would be more general but is a bigger change than made here. |
| **F** — barrier-blind at night | *(none — investigated, not fixed)* | See [`BARRIER_BLIND_AT_NIGHT.md`](BARRIER_BLIND_AT_NIGHT.md). Traced the register's one line to three separate sub-issues: one is already a no-op for night agents, one (the hard avoid-set) can't be "fixed" via the obvious route without silently turning region-based navigation on for a random subset of night agents, and one (barrier preference) would be a complete no-op today regardless, since night agents' cognitive maps never populate known barriers. No code change; write-up explains why and what a real fix would actually require. |

Paths are relative to `src/main/java/pedsim/night/`. Each row corresponds to
one finding ID from `night_model_issues.html`; open the file's Javadoc for
the full rationale. Keep this table in sync as files are added.

**These six were run, not just compiled** — see
[`VALIDATION.md`](VALIDATION.md) for a real 7-day Torino baseline-vs-fixed
comparison (isolated worktrees, same seed, same population). Headline: the
fixes visibly engage (detour overhead +2.5pp) and C3 measurably improves
non-vulnerable agents' night-lux exposure (+8.7%) — but vulnerable agents
measurably got *worse* (−6.7%), which VALIDATION.md traces to a real,
pre-existing gap: the vulnerable-agent avoid-set isn't lighting-based at
all, a design question C3 explicitly left open for the code owner. Worth
reading before review.

**Confirmed already fixed upstream, not proposed here:** the register's
Section F item *"Nobody commutes in the dark"* — `CommuterAgent`'s unsourced
`!isDark()` guard — was independently removed in `f646722` ("Fix commute
darkness and city purpose config"), before this branch got to it. Don't
re-propose it; the register itself is updated to mark it resolved.

## Branch

`night-fixes-eval`, branched from `main` at `b2692d2`.

**Checked against upstream movement, 15 Sep:** `main` has since advanced to
`08063cf` (5 new commits from Gabriele, all same-day). Diffed
`b2692d2..main` and checked every file this branch touches — only
`NightAgentMovement.java` (C3) overlapped. Gabriele's `a10a108` fixes a
*different* bug there (vulnerable agents weren't unconditionally avoiding
the edge they were fleeing) and leaves `SharedCognitiveMap.getEdgesNonLitNonCommunityKnown()`
— C3's actual target — untouched, so no duplicate work; C3 was rebased onto
his restructuring rather than left stale. C1, C5, C6, B1, C2's files are
untouched upstream and remain valid as proposed.

Re-check `main` for further movement before Gabriele reviews — this was a
point-in-time check, not a standing guarantee.
