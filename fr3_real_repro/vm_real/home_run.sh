#!/usr/bin/env bash
# REAL: bring the PARKED arm back to its start pose by replaying (reversed, slowed) the joint path the arm itself drove in avoid_20261101_111504. THE ARM MOVES.
#   ESTOP=1 EP="<name of the person at the e-stop>" ./home_run.sh        CHECK=1 ./home_run.sh   (checks only, nothing starts)
# No planner, no map: home_replay.py -> UDP -> lab curobo_to_jtc (max-step, max-accel) -> fr3_arm_controller, with the contact guard + video running.
# Same gates as avoid_run.sh (robot_mode, controller loaded inactive, joints must be where the path starts), plus: planner stays in standby (NOPLAN=1).
# Guard: --ftrip ${FTRIP:-15} N (the old default was 8; run 2 of 11-01 tripped at 8.1 N with dTau 1.3 Nm = a slow pose-dependent drift of the force estimate), --ttrip 5 Nm unchanged.
set -u
H=$(cd "$(dirname "$0")" && pwd); LAB="ssh -i $HOME/.ssh/id_ed25519_lab_claude -o BatchMode=yes -o ConnectTimeout=8 murilo@100.88.134.127"
SRC="source /opt/ros/jazzy/setup.bash; source /ros2_ws/install/setup.bash; export ROS_DOMAIN_ID=77"
SRCRUN=${SRCRUN:-$H/saved_runs/avoid_20261101_111504}; CSV=$SRCRUN/guard_$(basename $SRCRUN | sed 's/^avoid_//').csv
TFROM=${TFROM:-38.0}; TTO=${TTO:-92.6}; VMAX=${VMAX:-0.2}
ID=$(date +%Y%m%d_%H%M%S); OUT=$H/saved_runs/home_$ID
die() { echo "REFUSED: $*"; exit 1; }
[ -f "$CSV" ] || die "missing $CSV"
[ "${CHECK:-0}" = 1 ] || { [ "${ESTOP:-0}" = 1 ] && [ -n "${EP:-}" ] || die "set ESTOP=1 and EP=<name of the person standing at the e-stop>"; }
QN=$($LAB "docker exec fr3-ros2 bash -c '$SRC; timeout 8 ros2 topic echo --once --field position /joint_states'" 2>/dev/null | python3 -c "
import sys,re; v=re.findall(r'-?\d+\.\d+(?:e-?\d+)?', sys.stdin.read())[:7]; print(','.join(v) if len(v)==7 else '')")
[ -n "$QN" ] || die "no joint states"
RM=$($LAB "docker exec fr3-ros2 bash -c '$SRC; timeout 8 ros2 topic echo --once --field robot_mode /franka_robot_state_broadcaster/robot_state'" 2>/dev/null | grep -o -E '^[0-9]+' | head -1)
echo "robot_mode: ${RM:-?}  (0=Other: brakes locked / FCI off, 1=idle, 2=move, 4=reflex, 5=user stopped)"
case "${RM:-}" in 1|2) ;; *) die "robot_mode ${RM:-unknown}: the arm cannot be commanded (brakes / FCI in Desk)";; esac
[ "$(docker ps --format '{{.Names}}' | grep -c -E '^(vm-mpc|vm-relay)$')" = 2 ] || die "vm-mpc/vm-relay not up"
$LAB "docker exec fr3-ros2 bash -c '$SRC; timeout 10 ros2 control list_controllers 2>&1'" | sed 's/\x1b\[[0-9;]*m//g' | grep -q "fr3_arm_controller.*inactive" || die "fr3_arm_controller is not loaded inactive in the driver"
MODE=${MODE:-direct}   # direct = straight min-jerk joint move to the start pose (checked against the real object clouds first); replay = retrace the recorded path in reverse (slow, but known clear)
if [ "$MODE" = direct ]; then
  QT=$(head -1 "$H/rehearsal_q0.txt" 2>/dev/null); [ -n "$QT" ] || die "no rehearsal_q0.txt: nothing says where home is"
  SCRIPT=/h/home_direct.py; ARGS="--q-target=$QT --vmax ${VMAXD:-0.3}"
else
  SCRIPT=/h/home_replay.py; ARGS="--csv /h/saved_runs/$(basename $SRCRUN)/$(basename $CSV) --t-from $TFROM --t-to $TTO --vmax $VMAX"
fi
RP="docker exec -i vm-relay /isaac-sim/python.sh $SCRIPT $ARGS"
PLAN=$($RP --q-now=$QN --dry --dump /tmp/home_path.csv 2>&1); echo "$PLAN" | sed 's/^/  /'; echo "$PLAN" | grep -q "^DRY: nothing sent" || die "path check failed (MODE=$MODE)"
DUR=$(echo "$PLAN" | sed -n -E 's/.*-> (replay )?([0-9]+) s.*/\2/p' | head -1); [ -n "$DUR" ] || die "cannot read the move duration"
TMPP=$(mktemp -d); docker cp vm-relay:/tmp/home_path.csv $TMPP/home_path.csv >/dev/null
CLR=$(cd /home/nexus/spot-teleop/isaac-sim_ws/franka_mpc/deploy_sim && source ds_env.sh && "${DSRUN[@]}" --gpus device=1 -v /home/nexus/spot-teleop/fr3_real_repro/calib_1031/avoidance_sim:/o:ro -v $TMPP:/s:ro $IMG /workspace/franka_mpc/deploy_sim/clearance_q.py /s/home_path.csv /o/objects_raw.npz 0.042 home 2>&1 | grep -E "clearance")
echo "$CLR" | sed 's/^/  /'
MING=$(echo "$CLR" | grep -E "bottle|drill|box" | sed -n -E 's/.*min clearance +(-?[0-9.]+) cm.*/\1/p' | sort -g | head -1); [ -n "$MING" ] || die "cannot read the clearance"
python3 -c "import sys; sys.exit(0 if float('$MING') >= ${MINGAP:-5} else 1)" || die "this $MODE path comes within $MING cm of an object (bar ${MINGAP:-5} cm): not moving. Try MODE=replay"
rm -rf $TMPP
echo "checks OK: $MODE move $DUR s, min gap to bottle/drill/box $MING cm, planner stays in standby, JTC loaded inactive"; [ "${CHECK:-0}" = 1 ] && { echo "CHECK only: nothing started"; exit 0; }
mkdir -p $OUT; echo "e-stop person: $EP"; echo "$EP" > $OUT/estop_person.txt; echo "parked joints: $QN" > $OUT/start_joints.txt
VS=$((DUR + 45)); GS=$((DUR + 50))
[ "${VIDEO:-0}" = 1 ] && $LAB "docker exec -d zed-build bash -c 'source /opt/ros/humble/setup.bash; export ROS_DOMAIN_ID=8 RMW_IMPLEMENTATION=rmw_fastrtps_cpp; python3 /tmp/vidrec.py /tmp/home_$ID.mp4 $VS 15 > /tmp/vidrec_$ID.log 2>&1'"   # video only with VIDEO=1
$LAB "docker exec -d fr3-ros2 bash -c '$SRC; python3 /tmp/contact_guard.py --out /tmp/guard_home_$ID.csv --secs $GS --ftrip ${FTRIP:-15} > /tmp/guard_home_$ID.log 2>&1'"
echo "recorders started (guard ftrip ${FTRIP:-15} N), baseline 7 s ..."; sleep 7
cleanup_rec() { $LAB "docker exec zed-build pkill -TERM -f 'vidrec.py /tmp/home_$ID' ; docker exec fr3-ros2 pkill -TERM -f 'contact_guard.py --out /tmp/guard_home_$ID'" >/dev/null 2>&1; }
ESTOP=1 BRIDGE_ABSENT_OK=1 NOPLAN=1 MAXACCEL=10 "$H/vm_real.sh" go; rc=$?
if [ $rc = 1 ]; then echo "go REFUSED: nothing was started on the arm"; cleanup_rec; exit 1; fi
trap 'echo "interrupted: releasing the arm"; RESTORE_CART=1 "$H/vm_real.sh" back; cleanup_rec; exit 130' INT TERM
if [ $rc = 0 ]; then QN2=$($LAB "docker exec fr3-ros2 bash -c '$SRC; timeout 8 ros2 topic echo --once --field position /joint_states'" 2>/dev/null | python3 -c "
import sys,re; v=re.findall(r'-?\d+\.\d+(?:e-?\d+)?', sys.stdin.read())[:7]; print(','.join(v))")
  echo "moving home, $MODE ($DUR s) ..."; $RP --q-now="$QN2" 2>&1 | tail -40; fi
RESTORE_CART=1 "$H/vm_real.sh" back; trap - INT TERM
echo "released; waiting for the recorders ..."; sleep 14; cleanup_rec; sleep 4
T=$($LAB "docker exec fr3-ros2 bash -c 'tail -3 /tmp/guard_home_$ID.log'" 2>/dev/null | sed 's/\x1b\[[0-9;]*m//g' | cut -c1-200); echo "$T"
$LAB "rm -rf /home/murilo/franka_ws/lab/calib_1031/home_$ID; mkdir -p /home/murilo/franka_ws/lab/calib_1031/home_$ID; docker cp fr3-ros2:/tmp/guard_home_$ID.csv /home/murilo/franka_ws/lab/calib_1031/home_$ID/; docker cp fr3-ros2:/tmp/guard_home_$ID.log /home/murilo/franka_ws/lab/calib_1031/home_$ID/; docker cp zed-build:/tmp/home_$ID.mp4 /home/murilo/franka_ws/lab/calib_1031/home_$ID/" >/dev/null 2>&1
scp -q -i $HOME/.ssh/id_ed25519_lab_claude -o BatchMode=yes "murilo@100.88.134.127:/home/murilo/franka_ws/lab/calib_1031/home_$ID/*" $OUT/ && echo "collected -> $OUT"
echo "final controllers:"; $LAB "docker exec fr3-ros2 bash -c '$SRC; timeout 10 ros2 control list_controllers 2>&1'" | sed 's/\x1b\[[0-9;]*m//g' | awk '{print "  "$1, $NF}'
echo "arm joints now vs the start pose of the avoidance runs:"; CHECK=1 "$H/avoid_run.sh" 2>&1 | sed -n 1,2p
