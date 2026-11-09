#!/usr/bin/env bash
# VM103 production SAM2 tracker (cut over from lab 2026-11-02 on Murilo's go).
# Node sam2_tracker_node, domain 8, GPU1. Same params/topics as the lab node.
# Image vm-tracker:sam2 (Dockerfile + source snapshot: see README).
# Subscribes: /zed1/zed_node/left/image_rect_color, /franka_seg_zed1/depth,
#   /zed1/zed_node/depth/camera_info, /perception/seed_command, /tracking/seed_pixel, ...
# Publishes: /zed1/segmentation_mask, /tracking/seed_3d, /tracking_state, ...
# WARNING: do NOT reseed rapidly (2+ seeds inside ~2s can crash the SAM video
# predictor with KeyError output_dict). Seed once, then leave it alone.
set -euo pipefail
D=$(cd "$(dirname "$0")" && pwd)
docker run -d --name vm-tracker-prod --net host --ipc host --gpus device=1 \
  -e ROS_DOMAIN_ID=8 -e RMW_IMPLEMENTATION=rmw_fastrtps_cpp -e PYTHONUNBUFFERED=1 \
  --entrypoint bash vm-tracker:sam2 -c 'source /opt/ros/humble/setup.bash && cd /tmp/trackpkg && python3 -u -m spotop.sam2_tracker_node --ros-args -p rgb_topic:=/vm/cable/rgb -p depth_topic:=/vm/cable/depth -p depth_info_topic:=/zed1/zed_node/depth/camera_info -p segmentation_mask_topic:=/zed1/segmentation_mask -p visualize:=false'
