#!/usr/bin/env bash
set -euo pipefail
HERE=$(cd "$(dirname "$0")" && pwd)
mkdir -p "$HERE/run" "$HERE/cache" "$HERE/kit_cache"
J=/isaac-sim/exts/isaacsim.ros2.core/humble
exec docker run --rm --name fr3-remote-isaac --gpus '"device=0"' --network host --ipc private --shm-size=2g \
 -e ACCEPT_EULA=Y -e PRIVACY_CONSENT=Y -e ROS_DISTRO=humble -e ROS_DOMAIN_ID=188 -e RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
 -e FASTRTPS_DEFAULT_PROFILES_FILE=/remote/fastdds_udp_only.xml -e PYTHONPATH="$J/rclpy" -e LD_LIBRARY_PATH="$J/lib" \
 -v "$HERE:/remote:rw" -v "$HERE/cache:/root/.cache:rw" -v "$HERE/kit_cache:/isaac-sim/kit/cache:rw" \
 --entrypoint /isaac-sim/python.sh spot-isaac-sim:6.0.0 \
 /remote/isaac_remote_arm.py --out /remote/run --fps 4 --render-every 3 --wait 86400 --video-res 640 360 --view 1.6 -1.5 1.55 .35 0 1.0 14 --free-run
