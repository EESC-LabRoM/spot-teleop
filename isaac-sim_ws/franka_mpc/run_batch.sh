#!/usr/bin/env bash
# A batch of grasp trials, back to back, LIGHT=1 by default (no video, physics-only steps between renders; see run_grasp.sh):
# one trial per grasp json in $GRASPS (grasp_variants.py), runs/<prefix>_gNN, then batch_summary.py -> runs/<prefix>.csv/.png.
#   PREFIX=b1 GRASPS=runs/b1_grasps [FULL=0|1] [any run_grasp.sh env] ./run_batch.sh
#   e.g. python3 grasp_variants.py runs/grasp_full1 --object drill --k 6 --out runs/b1_grasps && PREFIX=b1 GRASPS=runs/b1_grasps ./run_batch.sh
set -uo pipefail
cd "$(dirname "$0")"
PREFIX=${PREFIX:?PREFIX=}; GRASPS=${GRASPS:?GRASPS=dir of grasp jsons}; export LIGHT=${LIGHT:-1}
for g in "$GRASPS"/g*.json; do
  n=$(basename "$g" .json); run=${PREFIX}_$n; t0=$(date +%s)
  GRASP_JSON=$g ./run_grasp.sh "$run" > "runs/$run.console.log" 2>&1
  echo "$run: exit $? in $(( $(date +%s) - t0 )) s | $(grep -o '"centre_moved_mm[^}]*}' runs/$run/RESULT_isaac.json 2>/dev/null | head -c 160)"
done
/usr/bin/python3 batch_summary.py runs/${PREFIX}_g* --csv runs/$PREFIX.csv --plot runs/$PREFIX.png
