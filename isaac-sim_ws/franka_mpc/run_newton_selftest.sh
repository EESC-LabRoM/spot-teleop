#!/usr/bin/env bash
# newton_ros_arm.py --selftest in the Isaac image with the host's Newton (no ROS needed): sine tracking, gain tuning.  [extra newton_ros_arm.py args]
WS=/home/nexus/spot-teleop/isaac-sim_ws; FM=$WS/franka_mpc; SP=$WS/crawl-lab/IsaacLab/.venv/lib/python3.12/site-packages; J=/isaac-sim/exts/isaacsim.ros2.core/jazzy
docker run --rm --gpus all --network host --ipc host -e ACCEPT_EULA=Y -e PYTHONUNBUFFERED=1 -e PYTHONDONTWRITEBYTECODE=1 -e LD_LIBRARY_PATH=$J/lib -e PYTHONPATH="$SP:$J/rclpy" \
  -v "$SP":"$SP":ro -v "$WS":/workspace:rw -v "$FM/cache_newton_home/newton":/root/.cache/newton -v "$FM/cache_newton_home/warp":/root/.cache/warp \
  --entrypoint /isaac-sim/python.sh spot-isaac-sim:6.0.0 /workspace/franka_mpc/newton_ros_arm.py --out /workspace/franka_mpc/runs/newton_selftest --selftest "$@"
