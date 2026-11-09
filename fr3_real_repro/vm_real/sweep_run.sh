#!/usr/bin/env bash
# REAL: EasyHeC calibration sweep. THE ARM MOVES through ~24 poses planned by cuRobo MotionGen (calib_sweep/calib_poses.py + calib_legs.py), dwelling 3 s at each so the
# recorder (live_recorder2.py: stores both ZED images + depth + link poses when the arm is still in a new pose) gets frames. Planned AROUND the objects on the table (live map + raw depth points + table)
# (map + raw depth points + table): refuses if something new appears.
#   ESTOP=1 EP="<name of the person at the e-stop>" ./sweep_run.sh        CHECK=1 ./sweep_run.sh   (checks only, nothing starts)
# Same gates as home_run.sh/avoid_run.sh. Contact guard: --ftrip ${FTRIP:-15} N, joint-excursion trip OFF (a sweep moves every joint on purpose).
# Stop: the e-stop; or `docker exec vm-relay touch /tmp/sweep_stop` (the stream holds the last sample); Ctrl-C releases the arm.
set -u
H=$(cd "$(dirname "$0")" && pwd); LAB="ssh -i $HOME/.ssh/id_ed25519_lab_claude -o BatchMode=yes -o ConnectTimeout=8 murilo@100.88.134.127"
SRC="source /opt/ros/jazzy/setup.bash; source /ros2_ws/install/setup.bash; export ROS_DOMAIN_ID=77"
PLAN=${PLAN:-$H/calib_sweep/sweep_map_v5}; ID=$(date +%Y%m%d_%H%M%S); OUT=$H/saved_runs/sweep_$ID
die() { echo "REFUSED: $*"; exit 1; }
[ -f "$PLAN.csv" ] && [ -f "${PLAN}_summary.txt" ] || die "no plan at $PLAN.csv (run calib_poses.py + calib_legs.py)"
echo "plan: $(basename $PLAN)"; sed 's/^/  /' ${PLAN}_summary.txt
G=$(sed -n -E 's/.*INFLATED boxes ([-0-9.]+) cm.*/\1/p' ${PLAN}_summary.txt); python3 -c "import sys; sys.exit(0 if float('${G:-nan}') >= 0 else 1)" 2>/dev/null || die "the plan's clearance to the inflated world is below 0 (or unreadable)"
[ "${CHECK:-0}" = 1 ] || { [ "${ESTOP:-0}" = 1 ] && [ -n "${EP:-}" ] || die "set ESTOP=1 and EP=<name of the person standing at the e-stop>"; }
Q0=$(head -2 "$PLAN.csv" | tail -1 | cut -d, -f2-8)
QN=$($LAB "docker exec fr3-ros2 bash -c '$SRC; timeout 8 ros2 topic echo --once --field position /joint_states'" 2>/dev/null | python3 -c "
import sys,re; v=re.findall(r'-?\d+\.\d+(?:e-?\d+)?', sys.stdin.read())[:7]; print(','.join(v) if len(v)==7 else '')")
[ -n "$QN" ] || die "no joint states"
DEV=$(python3 -c "print('%.3f' % max(abs(float(a)-float(b)) for a,b in zip('$QN'.split(','),'$Q0'.split(','))))"); echo "arm joints now: $QN | deviation from the plan start: $DEV rad"
python3 -c "import sys; sys.exit(0 if float('$DEV') < 0.03 else 1)" || die "the arm is not at the plan's start pose (deviation $DEV rad > 0.03)"
RM=$($LAB "docker exec fr3-ros2 bash -c '$SRC; timeout 8 ros2 topic echo --once --field robot_mode /franka_robot_state_broadcaster/robot_state'" 2>/dev/null | grep -o -E '^[0-9]+' | head -1)
echo "robot_mode: ${RM:-?}  (1=idle, 2=move)"; case "${RM:-}" in 1|2) ;; *) die "robot_mode ${RM:-unknown}: the arm cannot be commanded";; esac
[ "$(docker ps --format '{{.Names}}' | grep -c -E '^(vm-mpc|vm-relay)$')" = 2 ] || die "vm-mpc/vm-relay not up"
$LAB "docker exec fr3-ros2 bash -c '$SRC; timeout 10 ros2 control list_controllers 2>&1'" | sed 's/\x1b\[[0-9;]*m//g' | grep -q "fr3_arm_controller.*inactive" || die "fr3_arm_controller is not loaded inactive in the driver"
$LAB "curl -s -m 5 http://127.0.0.1:8088/stream | head -c 6" 2>/dev/null | grep -q "data:" || die "dashboard robot stream :8088 not delivering (the recorder needs it)"
# the scene must still match the planning map snapshot (live robot-free depth of both cameras)
$LAB "docker cp /dev/null isaac-build:/dev/null" >/dev/null 2>&1; cat /tmp/claude-1000/-home-nexus-spot-teleop/34966040-515a-4de9-8ee8-a8c549b1568c/scratchpad/grabseg.py >/dev/null 2>&1
cat "$H/../calib_1031/tools/grabseg.py" | $LAB "cat > /tmp/claude_grabseg.py && docker cp /tmp/claude_grabseg.py isaac-build:/tmp/claude_grabseg.py" || die "cannot stage grabseg"
$LAB "docker exec isaac-build bash -c 'source /opt/ros/humble/setup.bash; source /workspaces/isaac_ros-dev/install/setup.bash; export ROS_DOMAIN_ID=8 RMW_IMPLEMENTATION=rmw_fastrtps_cpp FASTDDS_BUILTIN_TRANSPORTS=UDPv4; rm -f /tmp/claude_segcloud.npz; timeout 80 python3 /tmp/claude_grabseg.py >/dev/null 2>&1; test -s /tmp/claude_segcloud.npz'" || die "could not grab the live clouds to verify the table is clear"
TMPP=$(mktemp -d); $LAB "docker cp isaac-build:/tmp/claude_segcloud.npz /tmp/claude_segcloud_sweep.npz" && scp -q -i $HOME/.ssh/id_ed25519_lab_claude -o BatchMode=yes murilo@100.88.134.127:/tmp/claude_segcloud_sweep.npz $TMPP/segcloud.npz || die "cannot copy the clouds"
WORLD=${WORLD:-$H/calib_sweep/world}
TC=$(cd /home/nexus/spot-teleop/isaac-sim_ws/franka_mpc/deploy_sim && source ds_env.sh && "${DSRUN[@]}" --gpus device=1 -v $TMPP:/s:ro -v $H/calib_sweep:/cs:ro -v $WORLD:/w:ro $IMG /cs/scene_match.py /s/segcloud.npz /w/segcloud.npz 2>&1 | grep -E "^  zed|SCENE"); echo "$TC"
echo "$TC" | grep -q "^SCENE_MATCHES_PLAN" || [ "${SCENE_OK:-0}" = 1 ] || die "the scene has changed since the plan was made (something new on/over the table, or a camera moved): re-capture the map and re-plan"
rm -rf $TMPP
RP="docker exec -i vm-relay /isaac-sim/python.sh /h/calib_sweep/calib_sweep.py --csv /h/calib_sweep/$(basename $PLAN).csv"
DRYO=$($RP --q-now=$QN --dry 2>&1); echo "$DRYO" | sed 's/^/  /'; echo "$DRYO" | grep -q "^DRY: nothing sent" || die "the stream check failed"
DUR=$(echo "$DRYO" | sed -n -E 's/^sweep: [0-9]+ samples, ([0-9]+) s.*/\1/p'); [ -n "$DUR" ] || die "cannot read the sweep duration"
echo "checks OK: plan $(basename $PLAN) ($DUR s), arm at start, scene matches the plan, JTC loaded inactive, recorder stream ok, planner stays in standby (NOPLAN=1)"
[ "${CHECK:-0}" = 1 ] && { echo "CHECK only: nothing started"; exit 0; }
mkdir -p $OUT; echo "e-stop person: $EP" | tee $OUT/estop_person.txt; echo "start joints: $QN" > $OUT/start_joints.txt; cp $PLAN.csv ${PLAN}_summary.txt $OUT/
GS=$((DUR + 60)); REC=/tmp/live_calib_1101_$ID
cat "$H/calib_sweep/live_recorder2.py" | $LAB "cat > /tmp/live_recorder2.py && docker cp /tmp/live_recorder2.py zed-build:/tmp/live_recorder2.py"
$LAB "docker exec -d zed-build bash -c 'source /opt/ros/humble/setup.bash; export ROS_DOMAIN_ID=8 RMW_IMPLEMENTATION=rmw_fastrtps_cpp; exec python3 /tmp/live_recorder2.py 40 $REC > /tmp/live_recorder2_$ID.log 2>&1'"
for _ in $(seq 1 30); do $LAB "docker exec zed-build grep -q 'recorder up' /tmp/live_recorder2_$ID.log" 2>/dev/null && break; sleep 1; done
$LAB "docker exec zed-build grep -q 'recorder up' /tmp/live_recorder2_$ID.log" 2>/dev/null || { echo "recorder did not come up"; exit 1; }
$LAB "docker exec -d fr3-ros2 bash -c '$SRC; python3 /tmp/contact_guard.py --out /tmp/guard_sweep_$ID.csv --secs $GS --ftrip ${FTRIP:-15} --qtrip 0 > /tmp/guard_sweep_$ID.log 2>&1'"
echo "recorder + guard started (ftrip ${FTRIP:-15} N), baseline 7 s ..."; sleep 7
cleanup_rec() { $LAB "docker exec zed-build pkill -TERM -f 'live_recorder2.py 40 $REC'; docker exec fr3-ros2 pkill -TERM -f 'contact_guard.py --out /tmp/guard_sweep_$ID'" >/dev/null 2>&1; }
ESTOP=1 BRIDGE_ABSENT_OK=1 NOPLAN=1 MAXACCEL=10 "$H/vm_real.sh" go; rc=$?
if [ $rc = 1 ]; then echo "go REFUSED: nothing was started on the arm"; cleanup_rec; exit 1; fi
trap 'echo "interrupted: stopping the stream and releasing the arm"; docker exec vm-relay touch /tmp/sweep_stop; sleep 3; RESTORE_CART=1 "$H/vm_real.sh" back; cleanup_rec; exit 130' INT TERM
if [ $rc = 0 ]; then QN2=$($LAB "docker exec fr3-ros2 bash -c '$SRC; timeout 8 ros2 topic echo --once --field position /joint_states'" 2>/dev/null | python3 -c "
import sys,re; v=re.findall(r'-?\d+\.\d+(?:e-?\d+)?', sys.stdin.read())[:7]; print(','.join(v))")
  echo "sweeping ($DUR s) ..."; $RP --q-now="$QN2" 2>&1 | tail -60; fi
RESTORE_CART=1 "$H/vm_real.sh" back; trap - INT TERM
echo "released; waiting for the recorders ..."; sleep 10; cleanup_rec; sleep 4
$LAB "docker exec zed-build bash -c 'tail -n 8 /tmp/live_recorder2_$ID.log | grep -E \"saved|done\"; ls $REC | wc -l'" | tail -3
$LAB "rm -rf /home/murilo/franka_ws/lab/calib_1031/sweep_$ID; mkdir -p /home/murilo/franka_ws/lab/calib_1031/sweep_$ID; docker cp fr3-ros2:/tmp/guard_sweep_$ID.csv /home/murilo/franka_ws/lab/calib_1031/sweep_$ID/; docker cp fr3-ros2:/tmp/guard_sweep_$ID.log /home/murilo/franka_ws/lab/calib_1031/sweep_$ID/; docker cp zed-build:/tmp/live_recorder2_$ID.log /home/murilo/franka_ws/lab/calib_1031/sweep_$ID/; mkdir -p /home/murilo/easyhec/data/live_all && docker cp zed-build:$REC /home/murilo/easyhec/data/live_all/live_calib_1101_$ID && echo frames copied to /home/murilo/easyhec/data/live_all/live_calib_1101_$ID"
scp -q -i $HOME/.ssh/id_ed25519_lab_claude -o BatchMode=yes "murilo@100.88.134.127:/home/murilo/franka_ws/lab/calib_1031/sweep_$ID/*" $OUT/ && echo "collected -> $OUT"
T=$($LAB "docker exec fr3-ros2 bash -c 'tail -2 /tmp/guard_sweep_$ID.log'" 2>/dev/null | sed 's/\x1b\[[0-9;]*m//g' | cut -c1-200); echo "$T"
echo "final controllers:"; $LAB "docker exec fr3-ros2 bash -c '$SRC; timeout 10 ros2 control list_controllers 2>&1'" | sed 's/\x1b\[[0-9;]*m//g' | awk '{print "  "$1, $NF}'
echo "arm joints now vs the plan start:"; PLAN=$PLAN CHECK=1 "$H/sweep_run.sh" 2>&1 | sed -n 5,7p | cut -c1-200
