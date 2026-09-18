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
| **C1** | `NightEnvironment.java` | Re-scoped 18 Sep — see note directly below the table. `NightEnvironment.joinIlluminatedEdges()` never attached `min_lux` to graph edges, only `mean_lux`, so `NightLighting.isLit()`'s min_lux check silently passed every edge. Fix: join `min_lux` alongside `mean_lux`. |
| **C5** | *(none — confirmed resolved upstream, 18 Sep)* | Reroute-vs-speed-up split was a flat 50/50, blind to darkness. This branch proposed scaling it from 0.5 up to a new tunable ceiling as the edge gets darker; `main`'s own `NightBehaviour.rerouteOrIncreaseSpeed()`/`darknessDepth()` now does exactly that, same formula, same parameter name (`NightPars.maxRerouteProbabilityInDarkness`, default 0.9). Nothing left to propose. |
| **C6** | *(none — confirmed resolved upstream, 18 Sep)* | Docs only, no logic change proposed. Notes explicitly that night agents never board transit (deliberate — existing transit code is worse than walk-only) and states the direction of the resulting bias. `main`'s `NightAgent.java` class Javadoc (17 Sep) now states this exact finding, in the code owner's own words: "the bias runs both ways and does not cancel." Nothing left to propose. |
| **B1** | `NightPars.java` | Directional-lux statistic default switched `MIN` → `MEAN`. `MIN` over the 12m visibility window collapses to "distance from the nearest lamp" rather than measuring the street; both columns are already written by the pipeline, so this is a config default, not a re-run. Horizon-distance half of B1 (12m → 15m) is untouched — that needs a pipeline re-run this environment can't do. |
| **C2** | *(none — confirmed resolved upstream, 18 Sep)* | Lux now enters route-planning cost, not just reactive behaviour — this branch proposed a known edge's Dijkstra cost scaling up toward a tunable ceiling as `mean_lux` falls below the agent's threshold. `main`'s `DijkstraRoadDistanceNight.lightingCostMultiplier()` (moved to `pedsim.night.routing.search` in `dd8b3cc`) does this already, same parameter name (`NightPars.maxKnownDarkEdgeCostMultiplier`, default `1.5`), same known-edges-only scoping — and additionally gates on `PedSimCityNight.isDark`, so the penalty doesn't apply to a daytime trip plan (a refinement this branch's proposal didn't have). Nothing left to propose. |
| **C3** | *(none — confirmed resolved upstream, 18 Sep)* | Non-vulnerable agents' reroute avoid-set was built from the raw OSM `lit` tag, inconsistent with the continuous-lux gate that actually triggers the reroute. `main`'s `NightAgentMovement.defineEdgesToAvoid()` now builds *both* the vulnerable and non-vulnerable avoid-sets from `NightLighting.unlitEdgesOutsideCommunityKnown()` — a lux-based set, cached per threshold. This goes beyond what was proposed here: this branch's C3 left the vulnerable-agent avoid-set as knowledge-only, which `VALIDATION.md` (below) found was exactly why vulnerable agents got *worse* under this branch's fix; `main` closes that gap too, with its own Javadoc explicitly calling out the lighting term on the vulnerable branch as load-bearing. Nothing left to propose. |
| **F** — `distanceWeight` | `ActivityPars.java` | Not a register finding ID, but a Section F item ("`distanceWeight` is stale for Torino's real circuity"). `DestinationChoice.choose()` scales distance by `Pars.networkCircuityFactor`, and only the *product* `distanceWeight × networkCircuityFactor` sets choice probabilities. `distanceWeight = 0.0012` was set while circuity sat at a hardcoded 1.41; circuity is now self-measured (1.292 for Torino), so the product silently drifted 8.4% with nobody deciding that. Re-derived by holding the product invariant: `0.0012 × (1.41/1.292) ≈ 0.0013096`. **Consistency fix, not a calibration** — `distanceWeight` still has never been fitted to real data (Audimob is a separate, still-open request). Scoped to Torino: this is one global constant, not a per-city dynamic correction — see the file's own Javadoc for why a live-reference fix (matching the pattern `decideCommuteMode()`/`CommuteCalibration` already use) would be more general but is a bigger change than made here. |
| **F** — barrier-blind at night | *(none — investigated, not fixed)* | See [`BARRIER_BLIND_AT_NIGHT.md`](BARRIER_BLIND_AT_NIGHT.md). Traced the register's one line to three separate sub-issues: one is already a no-op for night agents, one (the hard avoid-set) can't be "fixed" via the obvious route without silently turning region-based navigation on for a random subset of night agents, and one (barrier preference) would be a complete no-op today regardless, since night agents' cognitive maps never populate known barriers. No code change; write-up explains why and what a real fix would actually require. |

**C1 note (18 Sep):** re-checked against current `main` (`5b9dea1`). The behavioural gate itself
was independently reimplemented by Gabriele as `NightLighting.isLit()`/`darknessDepth()` in the
night module's `engine` package — it already checks `min_lux` against
`NightPars.darkSpotLuxThreshold`, doing what this branch's original C1 edit to
`NightBehaviour.java` (below, in git history) asked for. But a direct code review found
`NightEnvironment.joinIlluminatedEdges()` never actually attached `min_lux` to graph edges, only
`mean_lux` — so `isLit`'s `minLuxAttr == null` fallback passed every edge unconditionally,
regardless of how dark its darkest sampled point actually was. C1 was never really fixed upstream
despite looking fixed from the diff (`isLit` reads exactly the right check). Fixed here by joining
`min_lux` alongside `mean_lux`. This branch's original `NightBehaviour.java` C1 edit predates
Gabriele's reimplementation, is superseded by it, and should be read as historical — not
re-proposed as-is against current `main`.

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
reading before review — but read it alongside the C2/C3/C5 note below: that
gap has since been independently closed on `main`, so this regression no
longer reflects current upstream behaviour. **Update, 18 Sep:** a narrower
re-validation isolating just the C1 join fix against current `main` (`5b9dea1`)
is done — see [the new section in VALIDATION.md](VALIDATION.md#empirical-validation-18-sep--isolating-the-c1-join-fix).
Headline: detour overhead is far lower on both sides now (main's reroute-loop
fix working network-wide), and the join fix lifts night-lux exposure for
**both** vulnerable and non-vulnerable agents (+8.1% / +8.9%) — unlike the
15-Sep run, because `main` already made the vulnerable avoid-set
lighting-based too.

**Confirmed already fixed upstream, not proposed here:** the register's
Section F item *"Nobody commutes in the dark"* — `CommuterAgent`'s unsourced
`!isDark()` guard — was independently removed in `f646722` ("Fix commute
darkness and city purpose config"), before this branch got to it. Don't
re-propose it; the register itself is updated to mark it resolved.

**C2, C3, C5, C6 note (18 Sep):** re-checked against current `main` (`5b9dea1`) — see the table
above for what each one now reads. All four are independently resolved upstream and need nothing
further proposed. (Also checked this pass: A1–A8, D1, D2 — all unchanged since the register's own
17-Sep pass except C6 and D2, both updated in the register directly; the pipeline files A1–A8 and
B1 depend on haven't moved upstream since `71e06b2` on 16 Sep.) This also retires the "Branch" section's 15-Sep claim below about C3 and
`a10a108`: that was accurate against `main` as it stood then, but `main`'s `defineEdgesToAvoid()`
has since been rewritten entirely (still lux-based on both branches, now additionally cached per
threshold and read through a bypass predicate rather than a whole-network set). Practical effect
for review: this branch's own C1/C2/C3/C5 file edits (in git history, under the real source paths
they mirror) are now historical record of what was proposed, not current proposals — `main` got
there independently on all four. The one live gap found was the C1 join, fixed above.

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

**Superseded, 18 Sep:** that last claim ("remain valid as proposed") no longer holds for C1's gate
logic, C2, C3, or C5. `main` has since advanced well past `08063cf` (now `5b9dea1`, 6 more commits)
and independently reimplemented all four — see the C1 note and the C2/C3/C5 note above for what
changed and what's actually still open (just the C1 join, in `NightEnvironment.java`). Left the
15-Sep paragraph above as-is rather than rewritten: it was an accurate point-in-time check, and the
reasoning in it about why no duplicate work existed *then* still holds as history.
