#!/usr/bin/env bash
# Attraction-field ablation in Isaac (LIGHT=1, FULL=0: no map), drill grasp #14 (runs/best_grasp/g00.json): a scripted operator aims with a
# fixed error (grasp_goal.py op_err = [dx,dy,dz (m), rotation vector (rad)]); the same 5 errors with the field OFF and ON (pf_mode:=grasp,
# 6D grasp-pose attraction). Runs runs/pf_off_tN, runs/pf_on_tN, then pf_eval.py -> runs/pf_eval.json/.png.
#   [CONDS="off on"] [TRIALS="1 2 3 4 5"] ./pf_batch.sh
set -uo pipefail
cd "$(dirname "$0")"
ERRS=([1]='[0.04,0.03,0.0,0.0,0.0,0.15]' [2]='[-0.03,0.02,0.01,0.0,0.0,-0.10]' [3]='[0.0,-0.05,0.0,0.05,0.0,0.0]'
      [4]='[0.02,0.0,0.02,0.0,0.08,0.08]' [5]='[-0.05,-0.03,0.0,0.0,0.0,0.2]')
for c in ${CONDS:-off on}; do for i in ${TRIALS:-1 2 3 4 5}; do
  run=pf_${c}_t$i; t0=$(date +%s)
  if [ "$c" = on ]; then NA="-p pf_mode:=grasp"; GA="-p grasp_target:=true"; else NA="-p pf_mode:=none"; GA=""; fi
  LIGHT=1 FULL=0 GRASP_JSON=runs/best_grasp/g00.json NODE_ARGS="-p extra_collision_sphere_buffer:=0.03 $NA" \
    GOAL_ARGS="-p op_err:=${ERRS[$i]} $GA" ./run_grasp.sh "$run" > "runs/$run.console.log" 2>&1
  echo "$run: exit $? in $(( $(date +%s) - t0 )) s"
done; done
python3 pf_eval.py runs/pf_off_t* runs/pf_on_t* --json runs/pf_eval.json --plot runs/pf_eval.png
