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

Paths are relative to `src/main/java/pedsim/night/`. Each row corresponds to
one finding ID from `night_model_issues.html`; open the file's Javadoc for
the full rationale. Keep this table in sync as files are added.

## Branch

`night-fixes-eval`, branched from `main` at `b2692d2`. Rebase/merge status
against `main` should be checked before Gabriele reviews, since `main` may
have moved on since this branch was cut.
