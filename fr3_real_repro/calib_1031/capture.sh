#!/bin/bash
# Calibration frame capture 2026-10-31 = passive recorder (zed-build) + Hipolito's fr3_workspace_sweep.py (fr3-ros2).
# RUN ON THE LAB HOST AS murilo (not as hipolito). Installed copy: /home/murilo/franka_ws/lab/calib_1031/capture.sh
#
#   DRY=1 ./capture.sh      everything EXCEPT motion: preflight, recorder (1 capture at the still pose), sweep --dry-run
#   ESTOP=1 ./capture.sh    the 23-waypoint --box sweep at SPEED (default 0.04 m/s, ~5 min), recorder cap 60
# (a --probe mode was tried and removed: Hipolito's script asserts every point incl. the current pose is inside its safe box, and the home pose is not)
# The real run asks for the NAME of the person at the e-stop and the word MOVE. ESTOP=1 alone proves nothing.
# Ctrl-C = the sweep is told to stop (it holds the measured pose), the recorder stops, captures are copied. It never kills anything else.
set -u
DRY=${DRY:-0}; MODE=box; SPEED=${SPEED:-0.04}
SWEEP=${SWEEP:-/lab/fr3_workspace_sweep.py}            # path INSIDE fr3-ros2 (the lab dir is mounted as /lab)
CAP=${CAP:-60}; SW_FLAGS="--box"
[ "$DRY" = 1 ] && { CAP=1; SW_FLAGS="$SW_FLAGS --dry-run"; }
TS=$(date +%Y%m%d_%H%M%S); OUT=/tmp/live_calib_1031_$TS; HOSTOUT=/home/murilo/easyhec/data/live_all/live_calib_1031_$TS
LOGDIR=/home/murilo/franka_ws/lab/calib_1031; mkdir -p "$LOGDIR"; LOG=$LOGDIR/run_$TS.log
exec > >(trap "" INT; exec tee -a "$LOG") 2>&1
echo "== capture.sh $TS  DRY=$DRY MODE=$MODE SPEED=$SPEED CAP=$CAP flags='$SW_FLAGS' user=$(id -un)"
ROS='source /opt/ros/jazzy/setup.bash; source /ros2_ws/install/setup.bash; export ROS_DOMAIN_ID=77'
clean(){ sed 's/\x1b\[[0-9;]*m//g'; }
die(){ echo "REFUSED: $*"; sleep 0.5; exit 1; }
RECSTARTED=0; SWEEPPID=""; SWEEPSTARTED=0; ABORTED=0

# ---------------- preflight (read-only) ----------------
[ "$(id -un)" = murilo ] || die "run as murilo (the bridge/driver helpers are murilo-owned)"
for c in fr3-ros2 zed-build; do [ "$(docker inspect -f '{{.State.Running}}' $c 2>/dev/null)" = true ] || die "container $c not running"; done
CL=$(docker exec fr3-ros2 bash -c "$ROS; timeout 12 ros2 control list_controllers" 2>&1 | clean)
[ "$(echo "$CL" | awk '$1=="cartesian_impedance_target_controller"{print $NF}')" = active ] || die "cartesian_impedance_target_controller is not active"
[ "$(echo "$CL" | awk '$1=="fr3_arm_controller"{print $NF}')" = active ] && die "fr3_arm_controller (JTC) is ACTIVE; two controllers on the arm"
PUBS=$(docker exec fr3-ros2 bash -c "$ROS; timeout 12 ros2 topic info /cartesian_impedance_target_controller/equilibrium_pose" 2>&1 | awk '/Publisher count/{print $3}')
if [ "$PUBS" != 0 ]; then
  WHO_PUB=$(docker exec fr3-ros2 bash -c "$ROS; timeout 12 ros2 topic info -v /cartesian_impedance_target_controller/equilibrium_pose" 2>&1 | awk '/^Node name:/{n=$3} /Endpoint type: PUBLISHER/{print n}' | tr '\n' ' ')
  [ "$DRY" = 1 ] && echo "WARNING (dry run, continuing): equilibrium_pose publisher(s) present: $WHO_PUB" || die "somebody already publishes the equilibrium pose (count '$PUBS', node(s): $WHO_PUB): teleop, replay or another sweep. The arm is in use."
fi
ST=$(curl -s -m 3 http://127.0.0.1:8784/status || true)
if [ -z "$ST" ]; then [ "${BRIDGE_ABSENT_OK:-0}" = 1 ] || die "bridge status page unreachable (set BRIDGE_ABSENT_OK=1 only if the owner is away and confirmed)"
else echo "$ST" | grep -q '"armed": true' && die "Hipolito's joystick bridge is ARMED"; fi
docker exec fr3-ros2 pgrep -f fr3_workspace_sweep >/dev/null 2>&1 && die "a sweep is already running in fr3-ros2"
docker exec zed-build pgrep -f live_recorder.py >/dev/null 2>&1 && die "a live_recorder is already running in zed-build"
curl -s -m 5 http://127.0.0.1:8088/stream | head -c 6 | grep -q "data:" || die "dashboard robot stream :8088 not delivering (the recorder needs it)"
docker exec fr3-ros2 test -f "$SWEEP" || die "$SWEEP missing inside fr3-ros2"
MODE0=$(docker exec fr3-ros2 bash -c "$ROS; timeout 8 ros2 topic echo --once --qos-reliability best_effort --field robot_mode /franka_robot_state_broadcaster/robot_state" 2>&1 | head -1)
[ "$MODE0" = 2 ] || die "robot_mode is '$MODE0' (need 2)"
echo "preflight OK: cartesian active, JTC not active, equilibrium_pose publishers 0, bridge not armed, robot_mode 2, stream ok"

# ---------------- confirmation (real run only) ----------------
if [ "$DRY" != 1 ]; then
  [ "${ESTOP:-0}" = 1 ] || die "set ESTOP=1: a person must be at the e-stop"
  [ -t 0 ] || die "needs a terminal (ssh -t) for the confirmation"
  read -r -p "Name of the person standing at the e-stop: " WHO; [ -n "$WHO" ] || die "no name given"
  echo "e-stop person: $WHO"
  read -r -p "THE ARM WILL MOVE ($MODE, $SW_FLAGS, $SPEED m/s). Type MOVE: " A; [ "$A" = MOVE ] || die "not confirmed"
fi

# ---------------- cleanup (always) ----------------
finish(){
  trap '' INT TERM HUP
  if [ "$SWEEPSTARTED" = 1 ]; then      # do NOT trust the docker-exec client: ask the process inside the container
    if docker exec fr3-ros2 pgrep -f fr3_workspace_sweep >/dev/null 2>&1 || docker exec fr3-ros2 pgrep -f "$SWEEP" >/dev/null 2>&1; then
      echo "stopping the sweep (SIGINT: it holds the measured pose)"
      docker exec fr3-ros2 pkill -INT -f "$SWEEP"
      for _ in $(seq 1 20); do docker exec fr3-ros2 pgrep -f "$SWEEP" >/dev/null 2>&1 || break; sleep 0.5; done
      docker exec fr3-ros2 pgrep -f "$SWEEP" >/dev/null 2>&1 && { echo "still alive: SIGTERM"; docker exec fr3-ros2 pkill -TERM -f "$SWEEP"; sleep 3; }
      docker exec fr3-ros2 pgrep -f "$SWEEP" >/dev/null 2>&1 && echo "!!!!!! THE SWEEP IS STILL RUNNING. USE THE E-STOP. !!!!!!"
    fi
  fi
  if [ "$RECSTARTED" = 1 ]; then
    docker exec zed-build pkill -TERM -f "live_recorder.py $CAP $OUT" 2>/dev/null; sleep 2
    N=$(docker exec zed-build bash -c "ls $OUT 2>/dev/null | wc -l"); echo "recorder saved $N capture(s) in zed-build:$OUT"
    docker exec zed-build bash -c "tail -n 12 /tmp/live_recorder_1031_$TS.log | grep -E 'saved|done'" | cut -c1-120
    mkdir -p "$(dirname "$HOSTOUT")" && docker cp zed-build:"$OUT" "$HOSTOUT" >/dev/null 2>&1 && echo "copied to $HOSTOUT" || echo "WARNING: copy to host failed; frames stay in zed-build:$OUT"
  fi
  echo "-- post state:"
  docker exec fr3-ros2 pgrep -f "$SWEEP" >/dev/null 2>&1 && echo "!!! WARNING: a sweep process is still alive in fr3-ros2 !!!" || echo "no sweep process left"
  docker exec zed-build pgrep -f live_recorder.py >/dev/null 2>&1 && echo "WARNING: a recorder is still alive in zed-build" || echo "no recorder left"
  echo "robot_mode: $(docker exec fr3-ros2 bash -c "$ROS; timeout 8 ros2 topic echo --once --qos-reliability best_effort --field robot_mode /franka_robot_state_broadcaster/robot_state" 2>&1 | head -1)"
  echo "hand pose: $(docker exec fr3-ros2 bash -c "$ROS; timeout 8 ros2 topic echo --once --qos-reliability best_effort --field pose.position /franka_robot_state_broadcaster/current_pose" 2>&1 | grep -E '^[xyz]:' | tr '\n' ' ')"
  echo "log: $LOG"; sleep 0.5
}
trap 'exit 130' INT TERM          # before the sweep exists: just leave (finish runs on EXIT)
trap finish EXIT

# ---------------- recorder (passive) ----------------
docker exec -d zed-build bash -c "source /opt/ros/humble/setup.bash; export ROS_DOMAIN_ID=8 RMW_IMPLEMENTATION=rmw_fastrtps_cpp; exec python3 /tmp/live_recorder.py $CAP $OUT > /tmp/live_recorder_1031_$TS.log 2>&1"
RECSTARTED=1
for _ in $(seq 1 30); do docker exec zed-build grep -q "recorder up" /tmp/live_recorder_1031_$TS.log 2>/dev/null && break; sleep 1; done
docker exec zed-build grep -q "recorder up" /tmp/live_recorder_1031_$TS.log 2>/dev/null || { echo "recorder did not come up"; exit 1; }
echo "recorder up -> $OUT (cap $CAP)"

# ---------------- sweep ----------------
echo "starting sweep: python3 $SWEEP $SW_FLAGS --speed $SPEED"
setsid docker exec fr3-ros2 bash -c "$ROS; echo \$\$ > /tmp/claude_sweep.pid; exec python3 $SWEEP $SW_FLAGS --speed $SPEED" &
SWEEPPID=$!; SWEEPSTARTED=1
# From here a signal only sets a flag (no exit inside the trap): repeated Ctrl-C is harmless, finish() does the stopping.
trap 'ABORTED=1; echo "SIGNAL received: stopping after this line"' INT TERM
RC=0
while kill -0 "$SWEEPPID" 2>/dev/null; do wait "$SWEEPPID"; RC=$?; [ "$ABORTED" = 1 ] && { RC=130; break; }; done
echo "sweep exit code $RC (aborted=$ABORTED)"
[ "$DRY" = 1 ] && sleep 6          # let the recorder grab its single still-pose capture
exit $RC
