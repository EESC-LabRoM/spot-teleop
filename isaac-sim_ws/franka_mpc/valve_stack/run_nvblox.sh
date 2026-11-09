#!/usr/bin/env bash
# nvblox for the Franka workcell, in the lightweight nvblox-lite container: publishes the static TF (camera -> robot base,
# zed_extrinsics_tf.py, one per camera) and runs nvblox_node on the ZED wrappers' topics (franka_zed_nvblox.launch.py).
# Start the Isaac stream and the ZED wrapper(s) first.
#   [CAMS="zed zed2"] [CAMDIR=<franka_ros_arm.py run dir with camera_<name>.json>] zed_lite/nvblox/run_nvblox.sh [launch args]
#   (default: CAMS=zed with the workcell's camera.json)      stop: docker stop nvblox-lite
set -euo pipefail
D=/home/nexus/spot-teleop/zed_lite/nvblox
WORKCELL=/home/nexus/spot-teleop/isaac-sim_ws/franka_mpc/runs/workcell     # camera.json + scene.json from franka_scene.py
CAMS=${CAMS:-zed}; CAMDIR=${CAMDIR:-$WORKCELL}
mkdir -p "$D/out"
TF=""
for c in $CAMS; do
  J=/camdir/camera_$c.json; [ "$c" = zed ] && [ ! -f "$CAMDIR/camera_zed.json" ] && J=/camdir/camera.json
  TF+="(python3 /ws/nvblox/zed_extrinsics_tf.py --workcell /workcell --camera-json $J --cam-link ${c}_camera_link \
        --optical ${c}_left_camera_frame_optical --out /ws/nvblox/out/extrinsics_$c.json &) && "
done
docker run --rm --name nvblox-lite --gpus all --network host --ipc host -e ROS_DOMAIN_ID=${DOM:-41} \
  -v "$D":/ws/nvblox -v "$WORKCELL":/workcell:ro -v "$CAMDIR":/camdir:ro \
  nvblox-lite:3.2.5-humble bash -c "source /opt/ros/humble/setup.bash && $TF \
    trap 'chown -R 1000:1000 /ws/nvblox/out' EXIT && \
    ros2 launch /ws/nvblox/franka_zed_nvblox.launch.py cameras:=$(echo $CAMS | tr ' ' ,) $*"
