#!/usr/bin/env bash
# REAL avoidance run with the drill / bottle / box on the table: VM planner + live 5 cm nvblox map + extra collision buffer. THE ARM MOVES.
#   ESTOP=1 EP="<name of the person at the e-stop>" ./avoid_run.sh        CHECK=1 ./avoid_run.sh   (checks only, nothing starts)
# Needs: vm_real.sh up (EXTRA_BUF set), fr3_arm_controller loaded INACTIVE, contact_guard.py + vidrec.py copied (lab_trial/, scratch).
# Order: checks -> recorders (video + contact guard) -> baseline 7 s -> go -> traj -> back (RESTORE_CART=1) -> stop recorders -> collect. A refused `go` (exit 1) touches nothing.
set -u
H=$(cd "$(dirname "$0")" && pwd); LAB="ssh -i $HOME/.ssh/id_ed25519_lab_claude -o BatchMode=yes -o ConnectTimeout=8 murilo@100.88.134.127"
SRC="source /opt/ros/jazzy/setup.bash; source /ros2_ws/install/setup.bash; export ROS_DOMAIN_ID=77"
Q0DEF=-0.3826228678,-0.7655044198,0.2073732913,-2.3191034794,0.2171889096,1.4969648122,0.5221893787
[ -s "$H/rehearsal_q0.txt" ] && Q0DEF=$(head -1 "$H/rehearsal_q0.txt")   # written by the rehearsal (rehearse.sh): the joints the sim started from
Q0=${Q0:-$Q0DEF}   # joints the rehearsal (sim) started from
export TRAJ_WPS=${TRAJ_WPS:-"0.166,0.311,-0.268,12,2;0.166,-0.289,-0.368,16,3;0.106,0.151,-0.368,14,3;0,0,0,14,3"}
ID=$(date +%Y%m%d_%H%M%S); OUT=$H/saved_runs/avoid_$ID
die() { echo "REFUSED: $*"; exit 1; }
[ "${CHECK:-0}" = 1 ] || { [ "${ESTOP:-0}" = 1 ] && [ -n "${EP:-}" ] || die "set ESTOP=1 and EP=<name of the person standing at the e-stop>"; }
QN=$($LAB "docker exec fr3-ros2 bash -c '$SRC; timeout 8 ros2 topic echo --once --field position /joint_states'" 2>/dev/null | python3 -c "
import sys,re; t=sys.stdin.read(); v=[float(x) for x in re.findall(r'-?\d+\.\d+(?:e-?\d+)?', t)][:7]; q0=[float(x) for x in '$Q0'.split(',')]
print(' '.join(f'{x:.4f}' for x in v), '|', 'maxdev %.3f rad' % max(abs(a-b) for a,b in zip(v,q0)) if len(v)==7 else 'NO JOINTS')")
echo "arm joints now | deviation from the rehearsed start: $QN"; echo "$QN" | grep -q "NO JOINTS" && die "no joint states"
RM=$($LAB "docker exec fr3-ros2 bash -c '$SRC; timeout 8 ros2 topic echo --once --field robot_mode /franka_robot_state_broadcaster/robot_state'" 2>/dev/null | grep -o -E '^[0-9]+' | head -1)
echo "robot_mode: ${RM:-?}  (0=Other: brakes locked / FCI off, 1=idle, 2=move, 4=reflex, 5=user stopped)"
case "${RM:-}" in 1|2) ;; *) die "robot_mode ${RM:-unknown}: the arm cannot be commanded. Open the brakes and activate FCI in Desk first (libfranka rejects the controller switch otherwise)";; esac
DEV=$(echo "$QN" | sed -n 's/.*maxdev \([0-9.]*\) rad.*/\1/p'); python3 -c "import sys; sys.exit(0 if float('$DEV') < 0.03 else 1)" || die "the arm is not in the configuration that was simulated (max joint deviation $DEV rad > 0.03): re-simulate from the new joints"
[ "$(docker ps --format '{{.Names}}' | grep -c -E '^(vm-mpc|vm-relay)$')" = 2 ] || die "vm-mpc/vm-relay not up (EXTRA_BUF=${XBUF:-0.08} MAXACCEL=10 ./vm_real.sh up)"
XBUF=${XBUF:-$([ -s "$H/rehearsal_buf.txt" ] && head -1 "$H/rehearsal_buf.txt" || echo 0.08)}; XBS=$(printf "%.4f" $XBUF)   # buffer the rehearsal (sim) passed with; 0.05 failed the bottle bar (1.7 cm), 0.08 gave bottle 3.8 / drill 9.8 / box 12.3 cm
grep -a -q "config + extra $XBS" $H/log/mpc/mpc_stdout.log || die "the planner was not started with EXTRA_BUF=$XBUF"
$LAB "docker exec fr3-ros2 bash -c '$SRC; timeout 10 ros2 control list_controllers 2>&1'" | sed 's/\x1b\[[0-9;]*m//g' | grep -q "fr3_arm_controller.*inactive" || die "fr3_arm_controller is not loaded inactive in the driver"
if [ -s "$H/rehearsal_cfg.txt" ]; then RCFG=$(head -1 "$H/rehearsal_cfg.txt"); RCC=${RCFG%%|*}; RCM=${RCFG#*|}; MCMD=$(docker inspect vm-mpc --format '{{json .Config.Cmd}}')
  echo "$MCMD" | grep -q -- "$RCC" || die "the planner is not running the config the rehearsal passed with ($RCC): restart it (CFG=$RCC NODE_EXTRA=\"$RCM\" EXTRA_BUF=$XBUF MAXACCEL=10 ./vm_real.sh up)"
  [ -z "$RCM" ] || echo "$MCMD" | grep -q -- "$RCM" || die "the planner lacks '$RCM' that the rehearsal passed with"; fi
echo "checks OK (planner extra buffer $XBS m, joints within $DEV rad of the rehearsal, JTC loaded inactive, guard ftrip ${FTRIP:-15} N, posture stop ${QTRIP:-1.0} rad)"; [ "${CHECK:-0}" = 1 ] && { echo "CHECK only: nothing started"; exit 0; }
mkdir -p $OUT; echo "e-stop person: $EP"; echo "$EP" > $OUT/estop_person.txt; echo "start joints: $QN" > $OUT/start_joints.txt
VS=130; GS=140
$LAB "docker exec -d zed-build bash -c 'source /opt/ros/humble/setup.bash; export ROS_DOMAIN_ID=8 RMW_IMPLEMENTATION=rmw_fastrtps_cpp; python3 /tmp/vidrec.py /tmp/avoid_$ID.mp4 $VS 15 > /tmp/vidrec_$ID.log 2>&1'"
$LAB "docker exec -d fr3-ros2 bash -c '$SRC; python3 /tmp/contact_guard.py --out /tmp/guard_$ID.csv --secs $GS --ftrip ${FTRIP:-15} --qtrip ${QTRIP:-1.0} > /tmp/guard_$ID.log 2>&1'"
echo "recorders started, baseline 7 s ..."; sleep 7
cleanup_rec() { $LAB "docker exec zed-build pkill -TERM -f 'vidrec.py /tmp/avoid_$ID' ; docker exec fr3-ros2 pkill -TERM -f 'contact_guard.py --out /tmp/guard_$ID'" >/dev/null 2>&1; }
ESTOP=1 BRIDGE_ABSENT_OK=1 MAXACCEL=10 "$H/vm_real.sh" go; rc=$?
if [ $rc = 1 ]; then echo "go REFUSED: nothing was started on the arm"; cleanup_rec; exit 1; fi
trap 'echo "interrupted: releasing the arm"; RESTORE_CART=1 "$H/vm_real.sh" back; cleanup_rec; exit 130' INT TERM
if [ $rc = 0 ]; then
  ST=""; for i in $(seq 1 90); do ST=$(grep -a -o 'MPC state: [a-z]*' "$H/log/mpc/mpc_node.csv" 2>/dev/null | tail -1 | awk '{print $3}'); [ "$ST" = active ] && break; sleep 2; done
  echo "MPC state after go: ${ST:-unknown}"
  if [ "$ST" != active ]; then echo "go accepted but the MPC never reached active (still '${ST:-unknown}'): releasing the arm, nothing was commanded"; RESTORE_CART=1 "$H/vm_real.sh" back; cleanup_rec; exit 1; fi
  "$H/vm_real.sh" traj
fi
RESTORE_CART=1 "$H/vm_real.sh" back; trap - INT TERM
echo "released; waiting for the recorders ..."; sleep 14; cleanup_rec; sleep 4
T=$($LAB "docker exec fr3-ros2 bash -c 'tail -3 /tmp/guard_$ID.log'" 2>/dev/null | sed 's/\x1b\[[0-9;]*m//g' | cut -c1-200); echo "$T"
$LAB "rm -rf /home/murilo/franka_ws/lab/calib_1031/avoid_$ID; mkdir -p /home/murilo/franka_ws/lab/calib_1031/avoid_$ID; docker cp fr3-ros2:/tmp/guard_$ID.csv /home/murilo/franka_ws/lab/calib_1031/avoid_$ID/; docker cp fr3-ros2:/tmp/guard_$ID.log /home/murilo/franka_ws/lab/calib_1031/avoid_$ID/; docker cp zed-build:/tmp/avoid_$ID.mp4 /home/murilo/franka_ws/lab/calib_1031/avoid_$ID/"
scp -q -i $HOME/.ssh/id_ed25519_lab_claude -o BatchMode=yes "murilo@100.88.134.127:/home/murilo/franka_ws/lab/calib_1031/avoid_$ID/*" $OUT/ && echo "collected -> $OUT"
ls -d $H/log/mpc/traj_* | tail -1 > $OUT/traj_dir.txt; cp -r "$(tail -1 $OUT/traj_dir.txt)" $OUT/traj 2>/dev/null
ffmpeg -loglevel error -y -i $OUT/avoid_$ID.mp4 -c:v libx264 -pix_fmt yuv420p -crf 23 $OUT/avoid_${ID}_h264.mp4 && echo "video: $OUT/avoid_${ID}_h264.mp4"
echo "final controllers:"; $LAB "docker exec fr3-ros2 bash -c '$SRC; timeout 10 ros2 control list_controllers 2>&1'" | sed 's/\x1b\[[0-9;]*m//g' | awk '{print "  "$1, $NF}'
