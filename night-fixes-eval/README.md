# night-fixes-eval

Proposed fixes and evaluation code for the night-time pedestrian behavioural
model, prepared by Marcin Wozniak for review by Gabriele Filomena.

## How this folder works

**Nothing outside this folder is touched.** Every file in here is a *copy* of
a file that lives elsewhere in the repo at the same relative path, e.g.:

```
night-fixes-eval/src/main/java/pedsim/night/engine/NightBehaviour.java
  -> proposes changes to
src/main/java/pedsim/night/engine/NightBehaviour.java
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

Scaffold only — no proposed fixes have been added yet. This README will be
updated with a table of contents as files are added.

## Branch

`night-fixes-eval`, branched from `main` at `b2692d2`. Rebase/merge status
against `main` should be checked before Gabriele reviews, since `main` may
have moved on since this branch was cut.
