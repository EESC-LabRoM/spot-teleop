#!/usr/bin/env bash
set -euo pipefail
HERE=$(cd "$(dirname "$0")" && pwd)
REPO=$(cd "$HERE/../../../../.." && pwd)
GPU_ID=${FR3_SIM_GPU:-1}
DOMAIN_ID=${ROS_DOMAIN_ID:-188}
[[ "$GPU_ID" =~ ^[0-9]+$ ]] || { echo 'FR3_SIM_GPU must be a physical GPU index'; exit 2; }
if [[ "$DOMAIN_ID" == 188 ]] && docker inspect -f '{{.State.Running}}' fr3-remote-mujoco 2>/dev/null | grep -qx true; then
 echo 'Stop fr3-remote-mujoco before starting Isaac on DDS 188'; exit 1
fi
mkdir -p "$HERE/run" "$HERE/cache" "$HERE/kit_cache"
J=/isaac-sim/exts/isaacsim.ros2.core/humble
exec docker run --rm --name fr3-remote-isaac --gpus "\"device=$GPU_ID\"" --network host --ipc private --shm-size=2g \
 -e ACCEPT_EULA=Y -e PRIVACY_CONSENT=Y -e ROS_DISTRO=humble -e ROS_DOMAIN_ID="$DOMAIN_ID" -e RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
 -e FR3_PHYSICAL_GPU="$GPU_ID" -e NVIDIA_DRIVER_CAPABILITIES=all \
 -e FASTRTPS_DEFAULT_PROFILES_FILE=/remote/fastdds_udp_only.xml -e PYTHONPATH="$J/rclpy" -e LD_LIBRARY_PATH="$J/lib" \
 -v "$REPO:/home/spot-teleop:rw" -v "$HERE:/remote:rw" -v "$HERE/cache:/root/.cache:rw" -v "$HERE/kit_cache:/isaac-sim/kit/cache:rw" \
 --entrypoint /isaac-sim/python.sh spot-isaac-sim:6.0.0 \
 /remote/isaac_remote_arm.py --physics-device cuda:0 --physics-hz 60 --arm-drive-damping-scale "${FR3_SIM_ARM_DAMPING_SCALE:-0.25}" --js-every 1 --out "${FR3_SIM_OUT:-/remote/run}" --fps "${FR3_SIM_FPS:-15}" --render-every 3 --wait 86400 --video-res 640 360 --view 1.6 -1.5 1.55 .25 0 1.18 20 --free-run "$@"
