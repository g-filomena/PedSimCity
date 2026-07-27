#!/usr/bin/env bash
#
# Server-side runner for the city-preparation pipeline (pipeline/00_city_preparation.py).
#
# Invoked over SSH by build_city_remote.bat / pipeline/remote_prep.ps1. The client does the
# `git pull` before calling this, so the freshest copy of the script runs. This script:
#   1. locates conda,
#   2. ensures the pedsimcity-prep environment exists (from environment-prep.yml),
#   3. makes sure cityImage is installed (from PyPI),
#   4. runs the pipeline, forwarding every argument it was given to the Python script.
#
# All arguments are passed straight through to 00_city_preparation.py, e.g.
#   bash pipeline/run_prep_remote.sh --city Torino --place 'Torino, Italy' --epsg 3003
set -euo pipefail

ENV_NAME="pedsimcity-prep"

# When launched detached (build_city_remote.bat / remote_prep.ps1), the client is not attached to
# our stdout, so it learns the outcome by reading this file. Record the exit code on any exit —
# success or a set -e failure — so a client that reconnects after a disconnect can report it.
if [ -n "${PREP_EXIT_FILE:-}" ]; then
    trap 'ec=$?; echo "$ec" > "$PREP_EXIT_FILE" 2>/dev/null || true' EXIT
fi

# Run from the repo root (this script lives in pipeline/).
REPO_ROOT="$(cd "$(dirname "$0")/.." && pwd)"
cd "$REPO_ROOT"

ENV_FILE="$REPO_ROOT/environment-prep.yml"
if [[ ! -f "$ENV_FILE" ]]; then
    echo "Missing environment-prep.yml at repo root: $ENV_FILE" >&2
    exit 1
fi

# --- locate conda -----------------------------------------------------------
# A non-login SSH shell usually has not sourced conda's init, so `conda` may be
# absent from PATH even when it is installed. Try PATH first, then common homes.
if ! command -v conda >/dev/null 2>&1; then
    for base in \
        "$HOME/miniconda3" "$HOME/anaconda3" "$HOME/miniconda" "$HOME/anaconda" \
        "/opt/miniconda3" "/opt/anaconda3" "/usr/local/miniconda3" "/usr/local/anaconda3"; do
        if [[ -f "$base/etc/profile.d/conda.sh" ]]; then
            # shellcheck disable=SC1091
            source "$base/etc/profile.d/conda.sh"
            break
        fi
    done
fi

if ! command -v conda >/dev/null 2>&1; then
    echo "conda not found on the server. Install Miniconda, or adjust the search paths in" >&2
    echo "pipeline/run_prep_remote.sh." >&2
    exit 1
fi
echo ">> Using conda: $(command -v conda)"

# --- accept the Anaconda-channel Terms of Service (once) --------------------
# Conda 24+ gates the default channels (pkgs/main, pkgs/r) behind a ToS acceptance, and
# `conda env create` cannot override channels — so the gate fires on a fresh machine even though
# our env is solved from conda-forge (environment-prep.yml pins conda-forge + nodefaults). Accept
# it for those channels; idempotent and harmless. Guarded for older conda without the subcommand.
if conda tos --help >/dev/null 2>&1; then
    conda tos accept --override-channels --channel https://repo.anaconda.com/pkgs/main >/dev/null 2>&1 || true
    conda tos accept --override-channels --channel https://repo.anaconda.com/pkgs/r    >/dev/null 2>&1 || true
fi

# --- ensure environment -----------------------------------------------------
if conda env list | awk '{print $1}' | grep -qx "$ENV_NAME"; then
    echo ">> Conda environment already exists: $ENV_NAME (reusing as-is)"
else
    echo ">> Creating conda environment from environment-prep.yml: $ENV_NAME"
    conda env create -f "$ENV_FILE"
fi

# --- ensure cityImage (installed from PyPI, matching build_city.bat) --------
if conda run --no-capture-output -n "$ENV_NAME" python -c "import cityImage" >/dev/null 2>&1; then
    echo ">> cityImage already importable in $ENV_NAME"
else
    echo ">> Installing cityImage from PyPI into $ENV_NAME"
    conda run --no-capture-output -n "$ENV_NAME" python -m pip install cityImage
fi

# --- run --------------------------------------------------------------------
echo ">> Running city preparation: 00_city_preparation.py $*"
conda run --no-capture-output -n "$ENV_NAME" \
    python pipeline/00_city_preparation.py "$@"
