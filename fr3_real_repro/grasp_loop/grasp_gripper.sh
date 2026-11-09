#!/bin/bash
# Gripper commands for grasp_test.sh (runs inside fr3-ros2, sourced env from caller). Usage: grasp_gripper.sh open|close
case "$1" in
  open)  timeout 25 ros2 action send_goal /franka_gripper/move franka_msgs/action/Move "{width: 0.08, speed: 0.05}" ;;
  close) timeout 25 ros2 action send_goal /franka_gripper/grasp franka_msgs/action/Grasp "{width: 0.03, speed: 0.03, force: 60.0, epsilon: {inner: 0.03, outer: 0.05}}" ;;
  *) echo "usage: $0 open|close"; exit 2 ;;
esac
