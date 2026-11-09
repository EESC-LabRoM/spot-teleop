#!/usr/bin/env bash
# Run franka_mpc_isaac.py headless in the Isaac Sim 6.0 image, then encode the frames to video.mp4 and delete them.
#   [IMAGE=spot-isaac-sim:6.0.0-zed] [ROS=1|jazzy] [NAME=container] ./run_isaac.sh [-s script.py] [run_name] [extra args]
#   (default script: franka_mpc_isaac.py; ROS=1 = bundled Humble rclpy, ROS=jazzy = bundled Jazzy rclpy)
set -euo pipefail
WS=${ISAAC_WS:-/home/nexus/spot-teleop/isaac-sim_ws}          # mounted as /workspace
CACHE=${ISAAC_CACHE:-/home/nexus/spot-isaac-sim/docker/cache}
script=franka_mpc_isaac.py
if [ "${1:-}" = "-s" ]; then script="$2"; shift 2; fi
name="${1:-simple_mpc}"; shift || true
out="$WS/franka_mpc/runs/$name"
PY=/isaac-sim/extsDeprecated/omni.isaac.ml_archive/pip_prebundle   # Isaac 6.0's torch 2.11.0+cu128
PY=$PY:/isaac-sim/exts/omni.pip.compute/pip_prebundle:/workspace/curobo-isaac6/src
PY=$PY:/workspace/curobo-isaac6/.venv/lib/python3.12/site-packages:/workspace/franka_mpc/pydeps
ROSENV=()                                         # ROS=1: Isaac 6.0's bundled ROS 2 Humble (rclpy for Python 3.12)
if [ "${ROS:-0}" = "1" ]; then
  PY=$PY:/isaac-sim/exts/isaacsim.ros2.core/humble/rclpy
  ROSENV=(-e ROS_DISTRO=humble -e RMW_IMPLEMENTATION=rmw_fastrtps_cpp -e ROS_DOMAIN_ID=${DOM:-8}
          -e LD_LIBRARY_PATH=/isaac-sim/exts/isaacsim.ros2.core/humble/lib)
elif [ "${ROS:-0}" = "jazzy" ]; then
  PY=$PY:/isaac-sim/exts/isaacsim.ros2.core/jazzy/rclpy
  ROSENV=(-e ROS_DISTRO=jazzy -e RMW_IMPLEMENTATION=rmw_fastrtps_cpp -e ROS_DOMAIN_ID=${DOM:-8}
          -e LD_LIBRARY_PATH=/isaac-sim/exts/isaacsim.ros2.core/jazzy/lib)
fi
[ -n "${NAME:-}" ] && ROSENV+=(--name "$NAME")
docker run --rm --gpus "${ISAAC_GPUS:-all}" --network host --ipc host \
  -e ACCEPT_EULA=Y -e PRIVACY_CONSENT=Y -e PYTHONDONTWRITEBYTECODE=1 -e PYTHONPATH="$PY" -e SETUPTOOLS_SCM_PRETEND_VERSION=0.7.7.post1.dev5 "${ROSENV[@]}" \
  -v "$WS":/workspace:rw \
  -v "$CACHE/kit":/isaac-sim/kit/cache:rw -v "$CACHE/ov":/root/.cache/ov:rw \
  -v "$CACHE/glcache":/root/.cache/nvidia/GLCache:rw -v "$CACHE/computecache":/root/.nv/ComputeCache:rw \
  --entrypoint /isaac-sim/python.sh "${IMAGE:-spot-isaac-sim:6.0.0}" \
  "/workspace/franka_mpc/$script" --out "/workspace/franka_mpc/runs/$name" "$@"
if [ -d "$out/frames" ] && ls "$out/frames"/*.jpg >/dev/null 2>&1; then
  ffmpeg -loglevel error -y -framerate 25 -i "$out/frames/%05d.jpg" -c:v libx264 -pix_fmt yuv420p -crf 23 "$out/video.mp4"
  rm -rf "$out/frames"
  echo "video: $out/video.mp4 ($(du -h "$out/video.mp4" | cut -f1))"
fi
