#!/usr/bin/env bash
# Start the sim mapping chain in order: Isaac workcell + virtual ZED stream + /joint_states -> ZED wrapper (zed-lite) ->
# nvblox + static TF (nvblox-lite) -> robot self-filter (franka_segmenter.py, Isaac image). Logs in zed_lite/logs/.
# Stop with stop_mapping.sh (reverse order, so no stale stream lock is left).
#   [SECS=3600] [ARM=hold|sweep] [SELF_FILTER=true|false] zed_lite/nvblox/start_mapping.sh
set -euo pipefail
Z=/home/nexus/spot-teleop/zed_lite; L=$Z/logs; mkdir -p "$L"
SECS=${SECS:-3600}; ARM=${ARM:-hold}; SELF_FILTER=${SELF_FILTER:-true}
cd /home/nexus/spot-teleop/isaac-sim_ws/franka_mpc
ROS=1 IMAGE=spot-isaac-sim:6.0.0-zed nohup ./run_isaac.sh -s zed_stream.py zed_stream --seconds "$SECS" --resolution HD1080 --fps 30 \
  --arm "$ARM" \
  > "$L/isaac.log" 2>&1 &
timeout 300 bash -c "until grep -q 'Initializing streamer' '$L/isaac.log'; do sleep 2; done" || { echo "Isaac stream did not start: $L/isaac.log"; exit 1; }
echo "1/4 Isaac streaming the virtual ZED X + /joint_states (arm $ARM)"
nohup "$Z/run_zed.sh" > "$L/zed.log" 2>&1 &
timeout 600 bash -c "until grep -q 'zed started' '$L/zed.log'; do sleep 2; done" || { echo "ZED wrapper did not start: $L/zed.log"; exit 1; }
echo "2/4 ZED wrapper publishing /zed/zed_node/..."
nohup "$Z/nvblox/run_nvblox.sh" self_filter:="$SELF_FILTER" > "$L/nvblox.log" 2>&1 &
timeout 120 bash -c "until grep -q 'static TF published' '$L/nvblox.log'; do sleep 2; done" || { echo "static TF not published: $L/nvblox.log"; exit 1; }
echo "3/4 nvblox mapping in fr3_link0 (self_filter $SELF_FILTER); ESDF service /nvblox_node/get_esdf_and_gradient"
nohup "$Z/nvblox/run_segmenter.sh" > "$L/segmenter.log" 2>&1 &
timeout 180 bash -c "until grep -q 'depth_out' '$L/segmenter.log'; do sleep 2; done" || { echo "segmenter produced no output: $L/segmenter.log"; exit 1; }
echo "4/4 robot self-filter publishing /franka_seg/depth"
