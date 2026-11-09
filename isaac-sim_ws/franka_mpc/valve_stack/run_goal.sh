#!/usr/bin/env bash
# Goal source (wrist_path.py: /wrist_pose on sim time) in the Isaac image's Python + bundled Jazzy rclpy.
#   ./run_goal.sh [-p path:=sweep]        stop: docker stop franka-goal
set -euo pipefail
WS=/home/nexus/spot-teleop/isaac-sim_ws; J=/isaac-sim/exts/isaacsim.ros2.core/jazzy
docker run --rm --name franka-goal --network host --ipc host \
  -e ROS_DISTRO=jazzy -e RMW_IMPLEMENTATION=rmw_fastrtps_cpp -e ROS_DOMAIN_ID=${DOM:-41} -e LD_LIBRARY_PATH=$J/lib \
  -e PYTHONPATH="/isaac-sim/exts/omni.pip.compute/pip_prebundle:$J/rclpy" -e PYTHONUNBUFFERED=1 -e ACCEPT_EULA=Y \
  -v "$WS/franka_mpc":/fm:ro --entrypoint /isaac-sim/python.sh spot-isaac-sim:6.0.0 \
  /fm/${SCRIPT:-wrist_path.py} --ros-args -p use_sim_time:=true "$@"
