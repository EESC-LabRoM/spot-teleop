#!/usr/bin/env bash
# Big-path trial of the cuRobo node on the lab FR3. RUN ON THE LAB PC (as murilo), with a person at the e-stop.
#   run_trial.sh --dry   everything except activating fr3_arm_controller: the arm CANNOT move (commands are dropped by the inactive controller)
#   run_trial.sh         shows the plan, asks you to type MOVE, then moves the arm along the 65 s path. Ctrl-C = abort.
# What it does: freezes Hipolito's joystick_bridge (SIGSTOP; resumed at the end), starts the two curobo_to_jtc halves + recorder/watchdog,
# activates fr3_arm_controller, publishes the goal path to the lab node (fr3_joystick_mpc), then ALWAYS cleans up: commands stop,
# controller deactivated, processes stopped, bridge resumed. Watchdog (rec_watch.py) kills the forwarder on tracking error > 0.05 rad,
# joint speed > 1.5 rad/s, robot mode outside {1,2}, any error flag, hand > 32 cm from the start, stale robot state.
set -u
T=/home/murilo/franka_ws/lab/claude_trial; DRY=0; [ "${1:-}" = "--dry" ] && DRY=1
ID=$(date +%Y%m%d_%H%M%S)$([ $DRY = 1 ] && echo _dry); RUN=$T/runs/$ID; mkdir -p "$RUN"
WPS="0,0.10,0,3,2;0,0.20,0,3,2;0,0.20,0.12,3,2;0,-0.20,0.12,8,2;0,-0.20,0,3,2;0,0,0,5,2;0.15,0,0,3.5,3;0,0,0,3.5,5"; HOLD=10
SRC='source /opt/ros/jazzy/setup.bash; source /ros2_ws/install/setup.bash; export ROS_DOMAIN_ID=77 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST'
S142='source /opt/ros/jazzy/setup.bash; export ROS_DOMAIN_ID=142 ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET RMW_IMPLEMENTATION=rmw_fastrtps_cpp'
drv() { docker exec fr3-ros2 bash -c "$SRC; $*" 2>/dev/null; }
log() { echo "[$(date +%T)] $*" | tee -a "$RUN/trial.log"; }
BP=""; ACTIVE=0; CLEANED=0
cleanup() {
  [ $CLEANED = 1 ] && return; CLEANED=1; trap - EXIT INT TERM
  log "CLEANUP: stop commands (JTC keeps its last target)"; docker exec fr3-ros2 bash -c 'kill $(cat /tmp/claude_fwd3.pid) 2>/dev/null'; sleep 1
  if [ $ACTIVE = 1 ]; then log "deactivate fr3_arm_controller"; drv "timeout 15 ros2 control switch_controllers --deactivate fr3_arm_controller --strict" | tail -1 | tee -a "$RUN/trial.log"; fi
  docker exec fr3-ros2 bash -c 'for p in $(ps -eo pid,args | grep "[r]ec_watch.py --fwd-pid" | awk "{print \$1}"); do kill -INT $p; done'; sleep 2
  docker cp fr3-ros2:/tmp/claude_rec3.csv "$RUN/rec.csv" >/dev/null 2>&1; docker cp fr3-ros2:/tmp/claude_fwd_lab3.log "$RUN/fwd_lab.log" >/dev/null 2>&1
  docker logs claude-fwd-a > "$RUN/fwd_a.log" 2>&1; docker rm -f claude-fwd-a claude-goal >/dev/null 2>&1
  [ -n "$BP" ] && { kill -CONT "$BP" 2>/dev/null; log "joystick_bridge $BP resumed"; }
  log "final: controllers: $(drv 'timeout 10 ros2 control list_controllers' | sed 's/\x1b\[[0-9;]*m//g' | grep fr3_arm | awk '{print $1"="$NF}')  robot_mode: $(drv 'timeout 8 ros2 topic echo --once --qos-reliability best_effort --field robot_mode /franka_robot_state_broadcaster/robot_state' | head -1)"
  if [ -s "$RUN/rec.csv" ]; then log "---- recorder summary"; docker exec fr3-ros2 python3 /lab/claude_trial/rec_stats.py /lab/claude_trial/runs/$ID/rec.csv 2>&1 | tee -a "$RUN/trial.log"; fi
  [ -s "$RUN/goal.csv" ] && docker exec fr3-ros2 python3 /lab/claude_trial/trial_summary.py /lab/claude_trial/runs/$ID 2>&1 | tee -a "$RUN/trial.log"
  log "run dir: $RUN"
}
trap cleanup EXIT INT TERM
die() { log "ABORT: $*"; exit 1; }

# ---------- pre-checks (read-only) ----------
log "run $ID  dry=$DRY"
docker ps --format '{{.Names}}' | grep -qx fr3-ros2 || die "fr3-ros2 is not running"
docker ps --format '{{.Names}}' | grep -qx fr3-zed-live-mpc || die "fr3-zed-live-mpc (the lab node) is not running"
docker ps --format '{{.Names}}' | grep -qE '^claude-(fwd-a|goal)$' && die "leftover claude-* containers"
docker exec fr3-ros2 bash -c 'ps -eo args | grep -E "[c]laude_trial/(curobo_to_jtc|rec_watch|goal_)"' | grep -q . && die "leftover claude_trial processes in fr3-ros2"
CTL=$(drv 'timeout 10 ros2 control list_controllers' | sed 's/\x1b\[[0-9;]*m//g' | awk '$1=="fr3_arm_controller"{print $NF}'); [ "$CTL" = inactive ] || die "fr3_arm_controller is '$CTL' (someone else is using the arm?)"
MODE=$(drv 'timeout 8 ros2 topic echo --once --qos-reliability best_effort --field robot_mode /franka_robot_state_broadcaster/robot_state' | head -1); [ "$MODE" = 1 ] || die "robot_mode is '$MODE', expected 1 (idle)"
BP=$(ps -eo pid,args | grep '[j]oystick_bridge.py hardware' | awk '{print $1}'); [ "$(echo "$BP" | wc -w)" = 1 ] || die "expected exactly one 'joystick_bridge.py hardware', got '$BP'"
curl -s -m 3 http://127.0.0.1:8784/status | grep -q '"armed": false' || die "Hipolito's bridge is ARMED (or its status page is down): somebody is teleoperating"
NS=$(docker exec fr3-zed-relay bash -c "$S142; timeout 5 ros2 topic echo --once --qos-durability transient_local --qos-reliability reliable /fr3_joystick_mpc/state std_msgs/msg/String" 2>/dev/null | head -1); [ "$NS" = "data: active" ] || die "lab node state is '$NS', expected 'data: active'"
HZ=$(docker exec fr3-zed-relay bash -c "$S142; timeout 7 ros2 topic hz /fr3_joystick_mpc/joint_preview" 2>/dev/null | awk '/average/{print int($3)}' | tail -1); [ "${HZ:-0}" -ge 20 ] || die "preview rate '${HZ:-0}' Hz < 20"
log "pre-checks OK: controller inactive, robot idle, bridge $BP disarmed, node active, preview ${HZ} Hz"
BP_FOUND=$BP; BP=""   # only set BP once we actually freeze it

if [ $DRY = 0 ]; then
  cat <<TXT

  ===== THE ARM WILL MOVE =====
  Path (relative to where the hand is now): y +10, +20 cm; up 12 cm; 40 cm sweep to y -20; down; back to centre; x +15 cm; back. 65 s, peak 9.4 cm/s.
  Needs: a person at the e-stop, nobody and nothing within ~25 cm around the hand's path (y +-20, x +15, z +12 cm).
  Hipolito's joystick_bridge will be frozen for ~2.5 min and resumed. Ctrl-C aborts safely.
TXT
  read -r -p "  Type MOVE to start: " A; [ "$A" = MOVE ] || die "not confirmed"
fi

# ---------- run ----------
kill -STOP "$BP_FOUND" || die "could not freeze joystick_bridge $BP_FOUND"; BP=$BP_FOUND; log "joystick_bridge $BP frozen"
docker exec -d fr3-ros2 bash -c "$SRC; echo \$\$ > /tmp/claude_fwd3.pid; exec python3 /lab/claude_trial/curobo_to_jtc.py --in udp --out ros --port 47802 --max-step 0.03 --max-accel 0 > /tmp/claude_fwd_lab3.log 2>&1"; sleep 3
docker run --rm -d --name claude-fwd-a --network host --entrypoint bash -e ROS_DOMAIN_ID=142 -e ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET -e RMW_IMPLEMENTATION=rmw_fastrtps_cpp -e FASTDDS_BUILTIN_TRANSPORTS=UDPv4 -e CMD_HOST=127.0.0.1 -v $T:/t:ro murilo/franka-ros2:jazzy-3.6.0 -c "source /opt/ros/jazzy/setup.bash; exec python3 /t/curobo_to_jtc.py --in ros --msg joint_trajectory --topic-in /fr3_joystick_mpc/joint_preview --out udp --port 47802 --max-step 0.03 --max-accel 0" >/dev/null
docker exec -d fr3-ros2 bash -c "$SRC; exec python3 /lab/claude_trial/rec_watch.py --fwd-pid \$(cat /tmp/claude_fwd3.pid) --out /tmp/claude_rec3.csv --seconds 400 --max-drift 0.32 > /tmp/claude_rec3.log 2>&1"
sleep 12
docker exec fr3-ros2 tail -1 /tmp/claude_fwd_lab3.log | grep -q forwarded || die "commands do not reach the driver side"
log "forwarders up: $(docker exec fr3-ros2 tail -1 /tmp/claude_fwd_lab3.log | sed 's/\x1b\[[0-9;]*m//g' | sed 's/.*curobo_to_jtc\]: //')"
if [ $DRY = 0 ]; then
  log "ACTIVATING fr3_arm_controller"; ACTIVE=1; drv "timeout 15 ros2 control switch_controllers --activate fr3_arm_controller --strict" | tail -1 | tee -a "$RUN/trial.log"
  sleep 15
  HOLDCHK=$(docker exec -i fr3-ros2 python3 - <<'PY'
import csv, math
R = [r for r in csv.DictReader(open("/tmp/claude_rec3.csv")) if r["hx"] != "nan"][-1000:-100]
h = [(float(r["hx"]), float(r["hy"]), float(r["hz"])) for r in R]
print("%.2f" % (max(math.dist(p, h[0]) for p in h) * 1e3))
PY
)
  log "hold check: hand moved ${HOLDCHK} mm in the last ~8 s"; awk "BEGIN{exit !(${HOLDCHK:-99} < 3.0)}" || die "hold not steady (>3 mm): not starting the path"
fi
ORIGIN=$(docker exec -i fr3-ros2 python3 - <<'PY'
import csv
r = next(x for x in csv.DictReader(open("/tmp/claude_rec3.csv")) if x["hx"] != "nan")
print(*[r[k] for k in ("hx", "hy", "hz", "hqx", "hqy", "hqz", "hqw")])
PY
); log "path origin (hand pose before activation = node hold goal): $ORIGIN"; set -- $ORIGIN
log "PATH START (dry=$DRY)"
docker run --rm --name claude-goal --network host --entrypoint bash -e ROS_DOMAIN_ID=142 -e ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET -e RMW_IMPLEMENTATION=rmw_fastrtps_cpp -e FASTDDS_BUILTIN_TRANSPORTS=UDPv4 -v $T:/t murilo/franka-ros2:jazzy-3.6.0 -c "source /opt/ros/jazzy/setup.bash; mkdir -p /t/runs/$ID; exec python3 /t/goal_path.py --p0 $1 $2 $3 --q0 $4 $5 $6 $7 --out /t/runs/$ID --hold $HOLD --wps '$WPS'" 2>&1 | tee -a "$RUN/trial.log" | grep -v "^$"
log "PATH END"; sleep 3
docker exec fr3-ros2 ls /tmp/claude_rec3.csv.TRIP >/dev/null 2>&1 && log "WATCHDOG TRIPPED: $(docker exec fr3-ros2 cat /tmp/claude_rec3.csv.TRIP)"
exit 0
