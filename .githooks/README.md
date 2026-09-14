# Git hooks

## pre-push — formatting gate, then Git LFS

Runs `mvn spotless:check`. If it passes, the push proceeds. If it fails, the
hook runs `spotless:apply`, prints the files it reformatted, and **stops the
push** — the commits being pushed still contain the unformatted code, so the fix
has to be committed before it means anything. Commit it and push again.

Two constraints this file must keep:

- **It ends by calling `git lfs pre-push "$@"`.** That is what this hook was
  before, and LFS needs it to upload objects. Replacing the file rather than
  extending it silently breaks LFS uploads.
- **Nothing before that handoff may read stdin.** Git feeds the hook the refs
  being pushed on stdin and LFS reads them, which is why the Maven calls have
  `< /dev/null`.

It decides with `spotless:check` rather than by comparing which files are dirty
before and after `apply`: a file that was already modified stays modified, so
reformatting it changes nothing about that list and the gate passes on
unformatted code. That was the first version of this hook, and it did not work.

Needs network the first time, since the plugin resolves from Maven Central —
so no `-o`. Bypass with `git push --no-verify` or `SKIP_SPOTLESS=1 git push`.

### One-time setup per clone
```sh
cp .githooks/pre-push .git/hooks/pre-push
chmod +x .git/hooks/pre-push
```

## pre-commit — Git LFS guard

Refuses to commit a Git-LFS-tracked file (e.g. `*.gpkg`) as a **raw binary**
instead of a pointer. This prevents the mistake that bloated history when large
GIS files were committed on a machine where Git LFS was not active.

### One-time setup per clone
```sh
git lfs install
cp .githooks/pre-commit .git/hooks/pre-commit
chmod +x .git/hooks/pre-commit
```

Do **not** point `core.hooksPath` at this directory — Git LFS installs its own
hooks (pre-push, post-checkout, …) under `.git/hooks`, and `core.hooksPath`
would bypass them. Copy the hook in instead (as above).
