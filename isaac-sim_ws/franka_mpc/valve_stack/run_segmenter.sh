#!/usr/bin/env bash
# Robot self-filter node (franka_segmenter.py) in the Isaac 6.0 image's Python 3.12: our curobo-isaac6 build + Isaac's
# bundled ROS 2 Humble rclpy, no Kit app. Reads the ZED depth + /joint_states, publishes /franka_seg/depth for nvblox.
#   [NAME=franka-seg] zed_lite/nvblox/run_segmenter.sh [--threshold 0.05 --buffer 0.01 --joint-states /joint_states_isaac]
#   second camera: NAME=franka-seg2 ... --depth /zed2/zed_node/depth/depth_registered --info /zed2/zed_node/depth/camera_info
#                  --out-depth /franka_seg_zed2/depth --out-mask /franka_seg_zed2/robot_mask        stop: docker stop <NAME>
set -euo pipefail
WS=/home/nexus/spot-teleop/isaac-sim_ws; ZL=/home/nexus/spot-teleop/zed_lite
H=/isaac-sim/exts/isaacsim.ros2.core/humble
PY=/isaac-sim/extscache/omni.warp.core-1.13.0+lx64:/isaac-sim/extsDeprecated/omni.isaac.ml_archive/pip_prebundle
PY=$PY:/isaac-sim/exts/omni.pip.compute/pip_prebundle:/workspace/curobo-isaac6/src
PY=$PY:/workspace/curobo-isaac6/.venv/lib/python3.12/site-packages:/workspace/franka_mpc/pydeps:$H/rclpy
docker run --rm --name "${NAME:-franka-seg}" --gpus all --network host --ipc host \
  -e ROS_DISTRO=humble -e RMW_IMPLEMENTATION=rmw_fastrtps_cpp -e ROS_DOMAIN_ID=${DOM:-41} -e LD_LIBRARY_PATH=$H/lib \
  -e PYTHONPATH="$PY" -e PYTHONUNBUFFERED=1 -e SETUPTOOLS_SCM_PRETEND_VERSION=0.7.7.post1.dev5 -e ACCEPT_EULA=Y \
  -v "$WS":/workspace:ro -v "$ZL":/zl:ro -v "$WS/cache/computecache":/root/.nv/ComputeCache \
  --entrypoint /isaac-sim/python.sh spot-isaac-sim:6.0.0 /zl/nvblox/franka_segmenter.py "$@"
