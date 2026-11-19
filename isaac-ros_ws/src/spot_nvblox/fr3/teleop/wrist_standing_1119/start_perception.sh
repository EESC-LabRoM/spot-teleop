#!/usr/bin/env bash
# Perception only. No robot trajectories, engagement or controller switching.
set -euo pipefail
ROOT=/home/hipolito/franka_teleop_checks/standing_ready_1119
SOURCE=/home/hipolito/spot-teleop/isaac-ros_ws/src/spot_nvblox/fr3/teleop/wrist_body_calibration_1118/runtime
docker run -d --name wrist-standing-perception-1119 --network host --ipc host --cpus 3 --memory 4g \
 -v "$SOURCE":/standing:ro \
 -v /home/hipolito/franka_teleop_checks/live_wrist_20261116:/deploy:ro \
 -v /home/hipolito/spot-teleop:/home/spot-teleop:ro \
 -v "$ROOT":/out \
 -e PYTHONDONTWRITEBYTECODE=1 -e NVIDIA_VISIBLE_DEVICES=void -e ROS_DOMAIN_ID=8 \
 -e OMP_NUM_THREADS=1 -e OPENBLAS_NUM_THREADS=1 -e MKL_NUM_THREADS=1 \
 -e PYTHONPATH=/home/spot-teleop/tmp/cv_site_hipolito:/home/spot-teleop/tmp/mp_site_hipolito:/home/spot-teleop/spot-ros2_ws/src/arm_pose_estimator \
 spot-teleop-isaac-ros:latest bash -lc 'source /opt/ros/humble/setup.bash; exec nice -n 10 python3 -u /standing/operator_hand_tracker_debug.py --domain 8 --pose-source rgbd_body --calibrate-arm --orientation-output full_6d --output-prefix /wrist_standing --no-tf --max-input-age .25 --max-output-age .35 --smooth-preview --preview-port 47814 --calibration-port 47814 --ros-args -r /fr3_sim/operator/camera_info:=/zed3/zed_node/rgb/camera_info -r /fr3_sim/operator/rgb:=/zed3/zed_node/rgb/image_rect_color -r /fr3_sim/operator/depth:=/zed3/zed_node/depth/depth_registered'
