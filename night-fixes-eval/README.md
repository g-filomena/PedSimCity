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
| **C5** | `src/main/java/pedsim/night/agents/NightBehaviour.java`<br>`src/main/java/pedsim/night/parameters/NightPars.java` | The reroute-vs-speed-up split (`rerouteOrIncreaseSpeed()`) was a hardcoded `random.nextDouble() < 0.5`, independent of how dark the edge actually is. Replaced with a probability that equals 0.5 at/above the agent's sensitivity threshold (i.e. **unchanged** wherever the original constant applied — the still-lit branch in `whenLitVulnerable`, and any edge with no continuous lux reading) and rises linearly toward a new tunable ceiling, `NightPars.maxRerouteProbabilityInDarkness` (default `0.9`, uncalibrated — needs tuning against the Torino/Lyon validation data), as the edge's measured illuminance falls toward 0 lux. Darkness is read from the same directional-entrance/mean-lux values `checkLightLevel()` already uses, so no new data dependency is introduced. Shadow-compiled clean against the real project classpath (`target/classes` + resolved `.m2` deps) before being added here. |
| **C6** | `src/main/java/pedsim/night/agents/NightAgent.java` | **Documentation only, no functional change** — this is not a code fix, matching the register's own suggested treatment. Night agents never board transit: `NightAgent.step()` fully overrides `ActivityAgent.step()` without calling `super`, so every trip is walked end to end even though the phone-data validation (Torino Vodafone, Lyon comptage-mobilites) counts metro and bus riders too. The existing transit-boarding code was deliberately **not** enabled — it assigns riders from three hardcoded archetype lines and walks an all-stops loop at one tick per hop, which would distort results more than walk-only does. Added a class-level Javadoc note stating this explicitly, plus the direction of the resulting bias: reported walking volumes/distances and lighting-exposure metrics are a conservative *upper bound* on real pedestrian street exposure, biased heavier on trips that parallel transit corridors. Shadow-compiled clean (both the proposed file and, as a control, the untouched original through the same pipeline). |
| **C1** | `src/main/java/pedsim/night/agents/NightBehaviour.java` | `min_lux` and `pct_unlit` are computed per edge by the pipeline (`03_street_lights.py`) and shipped in the GeoPackage, but the Java-side gate only ever read `mean_lux` — so an edge bright at both ends and dark in the middle passed on its average. `meanLightPasses()` now also requires `min_lux >= threshold` when that column is present, falling back to the pre-fix mean-only check when it's absent (backward compatible with lighting datasets predating the column). `pct_unlit` deliberately left unused — it's computed against the pipeline's fixed 5 lux service threshold, not each agent's personal one, so it isn't directly comparable in this gate; noted in the docstring as a candidate for separate, explicitly-scoped use. Marked "free" by the register (no pipeline re-run, no new parameters). Shadow-compiled clean alongside C5 and C6. |

Each row above corresponds to one finding ID from `night_model_issues.html`.
This table is the single source of truth for what's actually in this folder —
keep it in sync as files are added.

## Branch

`night-fixes-eval`, branched from `main` at `b2692d2`. Rebase/merge status
against `main` should be checked before Gabriele reviews, since `main` may
have moved on since this branch was cut.
