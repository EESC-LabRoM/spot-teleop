#!/usr/bin/env bash
# Staged REAL grasp test of a grasp.json (default: live dashboard grasp).
# Run ON THE VM (nexus).
#   ./grasp_test.sh plan [ZOFF] [GRASP_URL] [PLANARGS]   plan + checks only (no motion)
#   ./grasp_test.sh run [ZOFF] [GRASP_URL] [PLANARGS] [FROM]   interactive: ENTER before EVERY stage, 'q' aborts.
#   GRASP_URL 'bound' = the grasp the regrasp loop binds to the drill now (regrasp/state/grasp_binding.json); a run from S1
#     refuses if the live drill cloud moved > 3 cm (xy) from the plan's cloud.
#   ./grasp_test.sh unlock      clear /tmp/regrasp.lock (HELD_OBJECT) after the object was released by hand
#   ./grasp_test.sh home [ZOFF] [GRASP_URL] [PLANARGS]   from any pose of this plan: retreat to PRE, then home (same plan args!)
#     ZOFF (m, default 0.0) raises the grasp; GRASP_URL (default live
#     dashboard grasp.json) may be a file:// URL exported from a checked run.
#     PLANARGS (default empty) extra grasp_plan.py flags, e.g. "--cart-step 0.15 --wp-min-time 6".
#     FROM (default S1) resume at that stage (S1 home->pre, S2 pre->grasp, S3 close+lift, S4 put back,
#     S5 open+retreat to pre, S6 home). The executor refuses if the arm is > 0.10 rad from the stage
#     start, unless it is already within 0.20 rad of the stage target (short correction move).
#     S1 opens the gripper only if it is narrower than 75 mm.
# Path: direct JTC (fr3_arm_controller) with joint waypoints from IK; no MPC, no bridge.
# The teleop must be DISENGAGED (stage executor refuses if anyone else publishes on the JTC topic).
# Ctrl-C or 'q' = stop: the JTC holds the last target. The e-stop is the real safety.
set -u
MODE=${1:-plan}; ZOFF=${2:-0.0}; GRASPURL=${3:-http://127.0.0.1:8791/grasp.json}; PLANARGS=${4:-}; FROM=${5:-S1}
D=$(cd "$(dirname "$0")" && pwd); TS=$(date -u +%Y%m%d_%H%M%S); OUT=$D/runs_real/grasp_test_$TS; mkdir -p "$OUT"
BINDING=$D/../regrasp/state/grasp_binding.json
if [ "$GRASPURL" = bound ]; then   # 2026-11-06: use the grasp the regrasp loop currently binds to the drill (never a stale fixed file)
  BRUN=$(python3 -c "import json;print(json.load(open('$BINDING'))['run_id'])" 2>/dev/null) || { echo "no grasp binding ($BINDING)"; exit 1; }
  [ -f "$D/runs/$BRUN/grasp_plan_input.json" ] || { echo "bound run $BRUN has no grasp_plan_input.json"; exit 1; }
  GRASPURL=file://$D/runs/$BRUN/grasp_plan_input.json; echo "bound grasp: run $BRUN"
fi
KEY=~/.ssh/id_ed25519_lab_claude; LAB=murilo@100.88.134.127
SRC="source /opt/ros/jazzy/setup.bash; source /ros2_ws/install/setup.bash 2>/dev/null; export ROS_DOMAIN_ID=77 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST"
lab(){ ssh -i $KEY -o ConnectTimeout=8 $LAB "docker exec fr3-ros2 bash -c '$SRC; $*'"; }
log(){ echo "[$(date +%T)] $*" | tee -a "$OUT/test.log"; }
# regrasp lock (fr3_real_repro/regrasp/DESIGN.md): RUN_ACTIVE while this script runs (PID + 10 s heartbeat = file mtime),
# HELD_OBJECT from a successful close until a successful open. HELD_OBJECT survives an abort/crash (the object is still
# in the hand); clear it only by opening or with `grasp_test.sh unlock`. The loop never regenerates while either is set.
LOCK=/home/nexus/spot-teleop/fr3_real_repro/regrasp/state/regrasp.lock  # repo path: visible inside the VM containers (vm-viewer mounts /home/nexus/spot-teleop)
lockset(){ printf '{"state": "%s", "pid": %d, "ts": %s, "run_dir": "%s"}\n' "$1" $$ "$(date +%s.%N)" "$OUT" > $LOCK.tmp && mv -f $LOCK.tmp $LOCK; }
lockstate(){ python3 -c "import json;print(json.load(open('$LOCK'))['state'])" 2>/dev/null || echo NONE; }

if [ "$MODE" = unlock ]; then log "unlock: lock was $(lockstate)"; rm -f $LOCK; exit 0; fi
log "planning from $GRASPURL (z_offset $ZOFF)"
docker cp "$D/grasp_plan.py" vm-viewer:/tmp/grasp_plan.py >/dev/null
docker cp "$D/grasp_plan_lib.py" vm-viewer:/tmp/grasp_plan_lib.py >/dev/null
ORIGURL=$GRASPURL
if [[ "$GRASPURL" == file://* ]]; then
  docker cp "${GRASPURL#file://}" vm-viewer:/tmp/grasp_input_$TS.json >/dev/null
  GRASPURL="file:///tmp/grasp_input_$TS.json"
fi
docker exec vm-viewer timeout 300 python3 /tmp/grasp_plan.py /tmp/grasp_plan_$TS.json --grasp-url "$GRASPURL" --z-offset "$ZOFF" $PLANARGS | tee -a "$OUT/test.log" || { log "PLAN FAILED"; exit 1; }
docker cp vm-viewer:/tmp/grasp_plan_$TS.json "$OUT/plan.json" >/dev/null
grep -q '"problems": \[\]' "$OUT/plan.json" || { log "plan has PROBLEMS, refusing"; exit 1; }
if [ "$MODE" = run ] && [ "$FROM" = S1 ]; then   # 2026-11-06: a grasp made before the drill moved lands beside it (Murilo's try 2: 20 cm off)
  DRIFT=$(docker exec vm-viewer python3 -c "
import json,urllib.request,numpy as np
g=json.load(open('/tmp/grasp_input_$TS.json')) if '$ORIGURL'.startswith('file://') else json.load(urllib.request.urlopen('$ORIGURL',timeout=5))
c=np.asarray(json.loads(g['cloud']) if isinstance(g['cloud'],str) else g['cloud'],float)
d=json.load(urllib.request.urlopen('http://127.0.0.1:8791/drill.json',timeout=5))
L=np.asarray(json.load(urllib.request.urlopen('http://127.0.0.1:8791/grasp.json',timeout=5))['cloud'],float)
print('%.3f' % np.linalg.norm(np.median(c[:,:2],0)-np.median(L[:,:2],0)))" 2>/dev/null)
  log "drill drift since this grasp: ${DRIFT:-?} m (xy median, plan cloud vs live cloud)"
  python3 -c "import sys;sys.exit(0 if '${DRIFT:-9}' != '' and float('${DRIFT:-9}') <= 0.03 else 1)" || { log "the drill moved > 3 cm since this grasp was made (or no live cloud): refusing. Use GRASP_URL=bound after the loop rebinds"; exit 1; }
fi
[ "$MODE" = run ] || [ "$MODE" = home ] || { log "plan only. To execute: $0 run $ZOFF $ORIGURL \"$PLANARGS\" [FROM]"; exit 0; }
case "$FROM" in S[1-6]) ;; *) log "FROM must be S1..S6, got $FROM"; exit 1;; esac

log "pre-checks"
MODE_R=$(lab "timeout 8 ros2 topic echo --once --qos-reliability best_effort --field robot_mode /franka_robot_state_broadcaster/robot_state 2>/dev/null | head -1")
CTL=$(lab "timeout 25 ros2 control list_controllers 2>/dev/null" | sed 's/\x1b\[[0-9;]*m//g' | awk '$1=="fr3_arm_controller"{print $NF}')
log "robot_mode=$MODE_R fr3_arm_controller=$CTL"
[ "$MODE_R" = 2 ] && [ "$CTL" = active ] || { log "need robot_mode 2 and fr3_arm_controller active; refusing"; exit 1; }
[ "$(lockstate)" = HELD_OBJECT ] && log "lock: HELD_OBJECT from an earlier run (object in the hand), keeping it"
[ "$(lockstate)" = HELD_OBJECT ] || lockset RUN_ACTIVE
( while kill -0 $$ 2>/dev/null; do [ -f $LOCK ] && touch $LOCK 2>/dev/null; sleep 10; done ) & HB=$!  # never recreate a removed lock (11-06: an empty lock read as corrupt -> loop LOCKED)
trap 'kill $HB 2>/dev/null; [ "$(lockstate)" = RUN_ACTIVE ] && rm -f $LOCK' EXIT
scp -q -i $KEY "$OUT/plan.json" $LAB:/tmp/grasp_plan_$TS.json && scp -q -i $KEY "$D/grasp_stage_exec.py" "$D/grasp_gripper.sh" $LAB:/tmp/
ssh -i $KEY $LAB "docker cp /tmp/grasp_plan_$TS.json fr3-ros2:/tmp/grasp_plan.json && docker cp /tmp/grasp_stage_exec.py fr3-ros2:/tmp/grasp_stage_exec.py && docker cp /tmp/grasp_gripper.sh fr3-ros2:/tmp/grasp_gripper.sh" || exit 1

ask(){ read -r -p ">>> $1  [ENTER = go, q = abort] " A; [ "$A" = q ] && { log "ABORTED by operator before: $1"; exit 0; }; log "GO: $1"; }
stage(){ lab "timeout ${2:-60} python3 /tmp/grasp_stage_exec.py /tmp/grasp_plan.json $1 ${3:-0.05}" 2>&1 | tee -a "$OUT/test.log"; [ "${PIPESTATUS[0]}" = 0 ] || { log "STAGE $1 FAILED -> stopping here (JTC holds the last target)"; exit 1; }; }
# 2026-11-06: a failed gripper command STOPS the run (a refused re-grasp released the drill and S3 lifted empty)
grip(){ O=$(lab "bash /tmp/grasp_gripper.sh $1" 2>&1); R=$(echo "$O" | grep -a -E "Goal finished with status|libfranka gripper|rejected" | head -2); echo "$R" | tee -a "$OUT/test.log"
        echo "$R" | grep -q "status: SUCCEEDED" || { log "GRIPPER $1 FAILED -> stopping here (JTC holds the last target); check the gripper/object before resuming"; exit 1; }
        if [ "$1" = close ]; then lockset HELD_OBJECT; else lockset RUN_ACTIVE; fi; log "lock: $(lockstate)"; }
asks(){ read -r -p ">>> $1  [ENTER = go, s = skip, q = abort] " A; [ "$A" = q ] && { log "ABORTED by operator before: $1"; exit 0; }; [ "$A" = s ] && { log "SKIPPED: $1"; return 1; }; log "GO: $1"; }

[ "$MODE" = home ] && { ask "HOME: retreat to 12 cm above the grasp (if near it), then go home"; stage HOME 100; log "HOME done"; exit 0; }
SKIP=1; [ "$FROM" = S1 ] && SKIP=0; [ "$FROM" = S1 ] || log "RESUMING at $FROM (earlier stages skipped)"
at(){ [ "$1" = "$FROM" ] && SKIP=0; [ "$SKIP" = 0 ]; }
width(){ lab "timeout 8 ros2 topic echo --once --field position /joint_states 2>/dev/null | grep array" | python3 -c "import re,sys; v=re.findall(r\"-?\\d+\\.\\d+(?:e-?\\d+)?\", sys.stdin.read()); print(round(1000*(float(v[7])+float(v[8]))) if len(v)>=9 else -1)"; }
at S1 && { W=$(width); log "gripper width ${W} mm"
           if [ "$W" -lt 75 ]; then ask "OPEN the gripper (80 mm)"; grip open; fi
           ask "S1: home -> PRE-GRASP, 12 cm back along the approach (look: hand over the drill, fingers aligned?)"; stage S1_home_to_pre; }
at S2 && { ask "S2: PRE-GRASP -> GRASP";                         stage S2_pre_to_grasp; }
at S3 && { if [ "$FROM" = S3 ]; then asks "CLOSE the gripper (60 N)? s = skip if it is ALREADY holding the object" && grip close
           else ask "CLOSE the gripper (grasp, 60 N)"; grip close; fi
           ask "S3: LIFT (target 10 cm; the arm may sag ~2-3 cm under the object)"; stage S3_lift 60 0.12; }
at S4 && { ask "S4: put it back down";                            stage S4_put_back 60 0.12; }
at S5 && { ask "OPEN the gripper";                                grip open
           ask "S5: retreat to PRE-GRASP (12 cm back)";           stage S5_grasp_to_pre; }
at S6 && { ask "S6: back to home";                                stage S6_pre_to_home; }
log "DONE. log: $OUT/test.log"
