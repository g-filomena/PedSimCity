#!/bin/bash
# The validation runs of the night model on Torino_sidewalks, on gdsl1, from a clean checkout.
#
# One Wednesday and one Sunday for each of four months: February, April and July for Cuebiq (2025)
# and for Vodafone's winter, spring and summer windows; October for Vodafone's autumn window. The
# same seed in every run, the perception error left at its default, nothing else passed but the
# date, the size and the output tag. It refuses to run from a checkout with uncommitted changes and
# records the commit, the Java it ran on and the exact command in PROVENANCE, so a run can be
# repeated byte for byte on any machine that has the commit.
#
#   ssh gdsl1; cd /mnt/home/gabriele/PedSimCity; git pull; mvn -Pall-modules compile
#   mvn -q -Pall-modules dependency:build-classpath -Dmdep.outputFile=cp.txt
#   analysis/validation/Torino/run_validation.sh 0.5        # share of the census residents
#
# Outputs land in outputs/PedSimCityNight/<tag>/data/{edge_volume.csv,trips.csv}.
set -euo pipefail
PERCENT=${1:?share of the census residents, e.g. 0.5}
HEAP=${HEAP:-200g}
PARALLEL=${PARALLEL:-8}
SEED=20261110
JAVA=/usr/local/software/java/jdk-21.0.6/bin/java
RUNS=/mnt/home/gabriele/runs/validation_torino
cd "$(git rev-parse --show-toplevel)"

if [ -n "$(git status --porcelain --untracked-files=no)" ]; then
  echo "uncommitted changes: commit them first, a run must be repeatable from its commit" >&2
  git status --short --untracked-files=no >&2
  exit 1
fi
[ -f target/classes/pedsim/night/launcher/NightLauncher.class ] || { echo "build first" >&2; exit 1; }
[ -f cp.txt ] || { echo "cp.txt missing: dependency:build-classpath" >&2; exit 1; }

mkdir -p "$RUNS/logs"
{
  echo "commit $(git rev-parse HEAD)"
  echo "date   $(date -u +%FT%TZ)"
  echo "host   $(hostname)"
  echo "java   $($JAVA -version 2>&1 | head -1)"
  echo "seed   $SEED"
  echo "share  $PERCENT of the census residents"
  echo "city   Torino_sidewalks, night module, one day per run, 1 job"
  echo "resources (sha256):"
  (cd src/main/resources/Torino_sidewalks && sha256sum * | sed 's/^/  /')
} > "$RUNS/PROVENANCE"

run() {  # run <tag> <date>
  local tag=$1 date=$2 t0=$(date +%s)
  $JAVA -Xmx$HEAP -cp "target/classes:$(cat cp.txt)" pedsim.night.launcher.NightLauncher --headless \
    --cityName=Torino_sidewalks --percentage=$PERCENT --days=1 --jobs=1 --stepDelayMs=0 \
    --exportHtmlDashboard=false --SIMULATION_START_DATE=$date --seed=$SEED --outputTag=$tag \
    > "$RUNS/logs/$tag.log" 2>&1
  echo "$(date +%H:%M:%S) $tag rc=$? $(( $(date +%s) - t0 ))s" >> "$RUNS/SUMMARY"
}
export -f run; export RUNS JAVA HEAP PERCENT SEED
echo "$(date +%H:%M:%S) START" >> "$RUNS/SUMMARY"
printf '%s\n' \
  "val_feb_wed 2025-02-12" "val_feb_sun 2025-02-16" \
  "val_apr_wed 2025-04-09" "val_apr_sun 2025-04-13" \
  "val_jul_wed 2025-07-09" "val_jul_sun 2025-07-13" \
  "val_oct_wed 2025-10-15" "val_oct_sun 2025-10-19" \
  | xargs -P "$PARALLEL" -L 1 bash -c 'run "$0" "$1"'
echo "$(date +%H:%M:%S) DONE" >> "$RUNS/SUMMARY"
