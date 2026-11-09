#!/usr/bin/env bash
# Checkpoint 1 of the MPC-node port: curobo_mpc_node.py (robot:=franka, no map) drives the sim Franka in free space.
# Order: Isaac (franka_ros_arm.py: /clock, /joint_states_isaac, applies /joint_command_curobo) -> MPC node (warms up on
# the measured joints, holds the current pose) -> goal path (wrist_path.py) -> Isaac stops after --seconds of sim time.
#   ./run_free_space.sh [run_name] [--no-video] [--free-run]      outputs in runs/<run_name>: trace.npz, mpc_node.csv, logs, video.mp4
set -uo pipefail
cd "$(dirname "$0")"
RUN=${1:-ros_free}; shift || true
OUT=$PWD/runs/$RUN; mkdir -p "$OUT"; rm -f "$OUT"/mpc_node.csv
stop() { docker stop franka-goal franka-mpc franka-isaac >/dev/null 2>&1; }
trap stop EXIT
wait_for() {  # file pattern timeout_s
  local t=0; until grep -q "$2" "$1" 2>/dev/null; do sleep 1; t=$((t + 1)); [ $t -ge "$3" ] && { echo "timeout waiting for '$2' in $1"; return 1; }; done; return 0; }
NAME=franka-isaac ROS=jazzy ./run_isaac.sh -s franka_ros_arm.py "$RUN" --seconds 18 "$@" > "$OUT/isaac.log" 2>&1 &
ISAAC=$!
wait_for "$OUT/isaac.log" "waiting for /wrist_pose" 600 || { tail -20 "$OUT/isaac.log"; exit 1; }
echo "isaac up"
OUT=$OUT ./run_mpc_node.sh > "$OUT/node.log" 2>&1 &
wait_for "$OUT/node.log" "Initialized goal to current physical pose" 300 || { tail -20 "$OUT/node.log"; exit 1; }
echo "node warmed up on the measured joints"
./run_goal.sh > "$OUT/goal.log" 2>&1 &
wait $ISAAC; echo "isaac done (exit $?)"
grep "RESULT" "$OUT/isaac.log" | tail -1; grep "^video" "$OUT/isaac.log"
