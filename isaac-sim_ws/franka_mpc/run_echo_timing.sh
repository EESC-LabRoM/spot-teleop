#!/usr/bin/env bash
# Node timing without Isaac: echo_sim.py (kinematic stand-in, sim time = wall time) + curobo_mpc_node.py + wrist_path.py.
#   ./run_echo_timing.sh [run_name] [echo_sim.py args, e.g. --clock-hz 50 --js-every 1]       outputs in runs/<run_name>: RESULT_echo.json, echo_trace.npz, mpc_node.csv, logs
set -uo pipefail
cd "$(dirname "$0")"
RUN=${1:-echo_timing}; shift || true; OUT=$PWD/runs/$RUN; mkdir -p "$OUT"; rm -f "$OUT"/mpc_node.csv
stop() { docker stop franka-goal franka-mpc franka-echo >/dev/null 2>&1; }
trap stop EXIT
wait_for() {
  local t=0; until grep -q "$2" "$1" 2>/dev/null; do sleep 1; t=$((t + 1)); [ $t -ge "$3" ] && { echo "timeout waiting for '$2' in $1"; return 1; }; done; return 0; }
J=/isaac-sim/exts/isaacsim.ros2.core/jazzy
docker run --rm --name franka-echo --network host --ipc host -e ROS_DISTRO=jazzy -e RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
  -e ROS_DOMAIN_ID=8 -e LD_LIBRARY_PATH=$J/lib -e PYTHONPATH="/isaac-sim/exts/omni.pip.compute/pip_prebundle:$J/rclpy" \
  -e PYTHONUNBUFFERED=1 -e ACCEPT_EULA=Y -v "$PWD":/fm:ro -v "$OUT":/out:rw --entrypoint bash spot-isaac-sim:6.0.0 \
  -c "/isaac-sim/python.sh /fm/echo_sim.py --out /out --seconds 70 $*; chown -R 1000:1000 /out" > "$OUT/echo.log" 2>&1 &
ECHO=$!
OUT=$OUT ./run_mpc_node.sh > "$OUT/node.log" 2>&1 &
wait_for "$OUT/node.log" "Initialized goal to current physical pose" 300 || { tail -20 "$OUT/node.log"; exit 1; }
echo "node warmed up"
./run_goal.sh > "$OUT/goal.log" 2>&1 &
wait $ECHO; echo "echo done"; grep RESULT "$OUT/echo.log"
