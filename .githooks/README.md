# Git hooks

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
