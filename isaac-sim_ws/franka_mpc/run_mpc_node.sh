#!/usr/bin/env bash
# Our MPC node (spot_operation_ros2/curobo_mpc_node.py, robot:=franka) in the Isaac 6.0 image's Python 3.12: the
# curobo-isaac6 build + Isaac's bundled ROS 2 Jazzy rclpy + nvblox_msgs 3.2.5 built on Jazzy (zed_lite/nvblox_msgs_jazzy),
# no Kit app. On sim time (/clock from franka_ros_arm.py). Extra args are ROS args, e.g. -p use_esdf:=true.
#   OUT=<dir for mpc_node.csv> ./run_mpc_node.sh [-p name:=value ...]        stop: docker stop franka-mpc
set -euo pipefail
WS=/home/nexus/spot-teleop/isaac-sim_ws
NODE=${NODE:-/home/nexus/spot-teleop/spot-ros2_ws/src/spot_operation_ros2/spot_operation_ros2}
NVM=/home/nexus/spot-teleop/zed_lite/nvblox_msgs_jazzy/install
OUT=${OUT:-$WS/franka_mpc/runs/mpc_node}; mkdir -p "$OUT"
J=/isaac-sim/exts/isaacsim.ros2.core/jazzy
PY=/isaac-sim/extscache/omni.warp.core-1.13.0+lx64:/isaac-sim/extsDeprecated/omni.isaac.ml_archive/pip_prebundle
PY=$PY:/isaac-sim/exts/omni.pip.compute/pip_prebundle:/workspace/curobo-isaac6/src
PY=$PY:/workspace/curobo-isaac6/.venv/lib/python3.12/site-packages:/workspace/franka_mpc/pydeps
PY=$PY:$J/rclpy:/nvm/lib/python3.12/site-packages
ROSARGS=(-p robot:=franka -p base_frame:=fr3_link0 -p robot_config:=/workspace/franka_mpc/node_config/fr3.yml
         -p use_sim:=true -p use_sim_time:=true -p use_esdf:=false -p esdf_frame_id:=fr3_link0
         -p esdf_global_frame:=fr3_link0 -p log_csv_path:=/out/mpc_node.csv)
docker run --rm --name "${NAME:-franka-mpc}" --gpus "${GPUS:-all}" --network host --ipc host ${DOCKER_EXTRA:-} \
  -e ROS_DISTRO=jazzy -e RMW_IMPLEMENTATION=rmw_fastrtps_cpp -e ROS_DOMAIN_ID=${DOM:-8} -e LD_LIBRARY_PATH=$J/lib:/nvm/lib \
  -e PYTHONPATH="$PY" -e PYTHONUNBUFFERED=1 -e PYTHONDONTWRITEBYTECODE=1 -e SETUPTOOLS_SCM_PRETEND_VERSION=0.7.7.post1.dev5 \
  -e ACCEPT_EULA=Y -v "$WS":/workspace:ro -v "$NODE":/node:ro -v "$NVM":/nvm:ro -v "$OUT":/out:rw \
  -v "$WS/cache/computecache":/root/.nv/ComputeCache \
  --entrypoint bash spot-isaac-sim:6.0.0 -c \
  "${MPC_NICE:+nice -n $MPC_NICE }/isaac-sim/python.sh /node/curobo_mpc_node.py --ros-args ${ROSARGS[*]} $*; rc=\$?; chown -R 1000:1000 /out; exit \$rc"
