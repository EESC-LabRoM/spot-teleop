#!/usr/bin/env bash
# Launch the ZED ROS 2 wrapper (v5.4.1, ZED SDK 5.4.1) in sim mode, reading the Isaac Sim virtual ZED stream
# (franka_mpc/zed_stream.py). Topics come out as on the real camera: /zed/zed_node/... (ROS_DOMAIN_ID 8).
#   [CAM=zed PORT=30000 NAME=zed-lite] zed_lite/run_zed.sh [extra launch args, e.g. depth_mode:=NEURAL]   stop: docker stop zed-lite
#   second camera: CAM=zed2 PORT=30010 NAME=zed-lite2 (topics /zed2/zed_node/..., frames zed2_*)
# sim_static.yaml: fixed camera, no positional tracking (the sim stream has no usable IMU). The SDK's TensorRT depth engines are optimised on the first run (minutes) and kept in zed_lite/zed_resources.
set -euo pipefail
D=/home/nexus/spot-teleop/zed_lite
IMG=zed-lite:5.4.1-humble
CAM=${CAM:-zed}; PORT=${PORT:-30000}; NAME=${NAME:-zed-lite}; ZED_YAML=${ZED_YAML:-sim_static.yaml}   # or sim_quality.yaml
mkdir -p "$D/zed_resources" "$D/zed_settings"
if [ -z "$(ls -A "$D/zed_resources")" ]; then          # seed with the depth models shipped in the image
  docker run --rm -v "$D/zed_resources":/out "$IMG" bash -c 'cp -a /usr/local/zed/resources/. /out/ && chown -R 1000:1000 /out'
fi
docker run --rm --name "$NAME" --gpus "${GPUS:-all}" --network host --ipc host -e ROS_DOMAIN_ID=8 \
  -v "$D/install":/ws/install:ro -v "$D/$ZED_YAML":/ws/sim_static.yaml:ro -v "$D/zed_resources":/usr/local/zed/resources -v "$D/zed_settings":/usr/local/zed/settings \
  "$IMG" bash -c "source /opt/ros/humble/setup.bash && source /ws/install/setup.bash && \
    ros2 launch zed_wrapper zed_camera.launch.py camera_model:=zedx camera_name:=$CAM sim_mode:=true sim_address:=127.0.0.1 sim_port:=$PORT \
      use_sim_time:=false ros_params_override_path:=/ws/sim_static.yaml publish_tf:=false publish_map_tf:=false \
      publish_imu_tf:=false $*"
