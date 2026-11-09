#!/usr/bin/env bash
# Map dumps DURING an episode (an arm in motion is where a self-filter lag would leak): N x { q before, ESDF dump of the grasp region, q after } into <run>/<ep>/mid_<i>/
#   RUN=valve_isaac1 EP=ep02 N=8 ./valve_stack/mid_dump.sh        (needs the nvblox-lite container of run_valve_session.sh)
set -uo pipefail
Z=/home/nexus/spot-teleop/zed_lite; RUN=${RUN:?}; EP=${EP:?}; N=${N:-8}; OUT=/home/nexus/spot-teleop/isaac-sim_ws/franka_mpc/runs/$RUN/$EP
for ((i = 0; i < N; i++)); do
  t0=$(date +%s.%N)
  docker exec nvblox-lite bash -c "source /opt/ros/humble/setup.bash && mkdir -p /ws/nvblox/out/$RUN/mid_$EP\_$i && cd /ws/nvblox/out/$RUN/mid_$EP\_$i && \
    (timeout 5 ros2 topic echo --once --field position /joint_states_isaac > q_before.txt 2>/dev/null); \
    python3 /ws/nvblox/map_check.py --workcell /workcell --out . --service /nvblox_node/get_esdf_and_gradient --aabb-min 0.20 -0.45 0.10 --aabb-size 0.60 0.60 0.60 > /dev/null 2>&1; \
    (timeout 5 ros2 topic echo --once --field position /joint_states_isaac > q_after.txt 2>/dev/null); chown -R 1000:1000 ." > /dev/null 2>&1
  t1=$(date +%s.%N); mkdir -p "$OUT"; rm -rf "$OUT/mid_$i"; cp -r "$Z/nvblox/out/$RUN/mid_$EP\_$i" "$OUT/mid_$i" 2>/dev/null; echo "$i $t0 $t1" >> "$OUT/mid_times.txt"
  echo "mid $i dumped in $(echo "$t1 - $t0" | bc) s"
done
