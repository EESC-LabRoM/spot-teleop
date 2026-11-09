#!/bin/bash
# Send the real FR3 to its home hand pose with the lab's fr3_home.py (impedance equilibrium target, min-jerk, no controller swap).
# RUN ON THE LAB HOST AS murilo. Installed copy: /home/murilo/franka_ws/lab/calib_1031/home.sh
#   DRY=1 ./home.sh      plan only, nothing published
#   ESTOP=1 ./home.sh    asks for the e-stop person's NAME and the word MOVE (needs a terminal)
# Ctrl-C = fr3_home.py re-targets the measured pose (it holds). fr3_home.py refuses when it cannot read the controller list (flaky under load): just run again.
set -u
DRY=${DRY:-0}; SPEED=${SPEED:-0.06}; TS=$(date +%Y%m%d_%H%M%S)
LOGDIR=/home/murilo/franka_ws/lab/calib_1031; mkdir -p "$LOGDIR"; LOG=$LOGDIR/home_$TS.log
exec > >(trap "" INT; exec tee -a "$LOG") 2>&1
echo "== home.sh $TS DRY=$DRY SPEED=$SPEED user=$(id -un)"
ROS='source /opt/ros/jazzy/setup.bash; source /ros2_ws/install/setup.bash; export ROS_DOMAIN_ID=77'
HOMEPY=/lab/fr3_home.py
die(){ echo "REFUSED: $*"; sleep 0.5; exit 1; }
rx(){ docker exec fr3-ros2 bash -c "$ROS; $1" 2>&1; }
[ "$(id -un)" = murilo ] || die "run as murilo"
[ "$(docker inspect -f '{{.State.Running}}' fr3-ros2 2>/dev/null)" = true ] || die "fr3-ros2 not running"
# anyone else commanding the arm?
PUBS=$(rx "timeout 12 ros2 topic info /cartesian_impedance_target_controller/equilibrium_pose" | awk '/Publisher count/{print $3}')
if [ "$PUBS" != 0 ]; then
  NAMES=$(rx "timeout 12 ros2 topic info -v /cartesian_impedance_target_controller/equilibrium_pose" | awk '/^Node name:/{n=$3} /Endpoint type: PUBLISHER/{print n}' | tr '\n' ' ')
  die "somebody publishes the equilibrium pose (count '$PUBS', node(s): $NAMES): replay, teleop or a sweep. The arm is in use."
fi
docker exec fr3-ros2 pgrep -f "python3 /tmp/play.py|/lab/fr3_workspace_sweep.py|$HOMEPY" >/dev/null 2>&1 && die "a replay/sweep/home process is running in fr3-ros2"
ST=$(curl -s -m 3 http://127.0.0.1:8784/status || true)
if [ -z "$ST" ]; then [ "${BRIDGE_ABSENT_OK:-0}" = 1 ] || die "bridge status page unreachable (BRIDGE_ABSENT_OK=1 only if the owner confirmed)"
else echo "$ST" | grep -q '"armed": true' && die "Hipolito's joystick bridge is ARMED"; fi
[ "$(rx "timeout 8 ros2 topic echo --once --qos-reliability best_effort --field robot_mode /franka_robot_state_broadcaster/robot_state" | head -1)" = 2 ] || die "robot_mode is not 2"
# plan check (publishes nothing); fr3_home.py's controller listing is flaky, so retry
OKPLAN=0; for i in 1 2 3 4 5; do P=$(rx "timeout 40 python3 $HOMEPY --dry-run --speed $SPEED" | sed 's/\x1b\[[0-9;]*m//g' | grep -E "HOME (OK|FAILED)|->" | tail -2); echo "$P" | grep -q "HOME OK" && { OKPLAN=1; break; }; sleep 2; done
echo "$P"; [ "$OKPLAN" = 1 ] || die "the home plan did not validate: $(echo "$P" | tail -1)"
echo "preflight OK: no other publisher, no replay/sweep, bridge not armed, robot_mode 2, plan valid"
if [ "$DRY" = 1 ]; then echo "DRY: nothing moved"; sleep 0.5; exit 0; fi
[ "${ESTOP:-0}" = 1 ] || die "set ESTOP=1: a person must be at the e-stop"
[ -t 0 ] || die "needs a terminal for the confirmation"
read -r -p "Name of the person standing at the e-stop: " WHO; [ -n "$WHO" ] || die "no name given"; echo "e-stop person: $WHO"
read -r -p "THE ARM WILL MOVE to home ($SPEED m/s). Type MOVE: " A; [ "$A" = MOVE ] || die "not confirmed"
ABORTED=0
trap 'ABORTED=1; echo "SIGNAL received: telling fr3_home.py to hold"' INT TERM
setsid docker exec fr3-ros2 bash -c "$ROS; exec python3 $HOMEPY --speed $SPEED" & HP=$!
while kill -0 "$HP" 2>/dev/null; do
  wait "$HP"; RC=$?
  if [ "$ABORTED" = 1 ]; then docker exec fr3-ros2 pkill -INT -f "$HOMEPY"; for _ in $(seq 1 20); do docker exec fr3-ros2 pgrep -f "$HOMEPY" >/dev/null 2>&1 || break; sleep 0.5; done
    docker exec fr3-ros2 pgrep -f "$HOMEPY" >/dev/null 2>&1 && echo "!!!!!! fr3_home.py STILL RUNNING. USE THE E-STOP. !!!!!!"; RC=130; break; fi
done
trap '' INT TERM
echo "home exit code ${RC:-0}"
echo "-- post state: robot_mode $(rx "timeout 8 ros2 topic echo --once --qos-reliability best_effort --field robot_mode /franka_robot_state_broadcaster/robot_state" | head -1)"
echo "hand pose: $(rx "timeout 8 ros2 topic echo --once --qos-reliability best_effort --field pose.position /franka_robot_state_broadcaster/current_pose" | grep -E '^[xyz]:' | tr '\n' ' ')"
docker exec fr3-ros2 pgrep -f "$HOMEPY" >/dev/null 2>&1 && echo "!!! fr3_home.py still alive !!!" || echo "no fr3_home.py left"
echo "log: $LOG"; sleep 0.5; exit "${RC:-0}"
