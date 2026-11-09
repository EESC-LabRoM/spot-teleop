#!/usr/bin/env bash
# VM103 production nvblox (cut over from lab 2026-11-02 on Murilo's go).
# Node nvblox_curobo, domain 8, GPU1. 2cm, human_with_static_tsdf: the drill
# target mask (/vm/target_mask, live from vm-masksync) carves the drill out of
# the static layer, so dashboard mesh + planner ESDF both exclude it.
# Subscribes (all lab-hosted, over the 10.0.0.x cable via cable.xml):
#   depth /franka_seg_zed1/depth + /zed1/zed_node/depth/camera_info
#   color /zed1/zed_node/left/image_rect_color + /zed1/zed_node/left/camera_info
# Serves: /nvblox_curobo/get_esdf_and_gradient, /nvblox_curobo/mesh, ...
set -euo pipefail
D=$(cd "$(dirname "$0")" && pwd)
# SEG=1: grasp variant (node nvblox_seg) consuming /vm/target_mask; prod stays untouched.
SEG=${SEG:-0}
if [ "$SEG" = 1 ]; then
  NAME=vm-nvblox-seg; NODE=nvblox_seg
  EXTRA="-p mapping_type:=human_with_static_tsdf -p use_segmentation:=true -r camera_0/mask/image:=/vm/target_mask -r camera_0/mask/camera_info:=/zed1/zed_node/depth/camera_info"
else
  NAME=vm-nvblox-prod; NODE=nvblox_curobo
  EXTRA="-p mapping_type:=human_with_static_tsdf -p use_segmentation:=true -r camera_0/mask/image:=/vm/target_mask -r camera_0/mask/camera_info:=/zed1/zed_node/depth/camera_info"
fi
# --pid host: unique host PIDs (hygiene for DDS GUID derivation).
docker run -d --name $NAME --net host --ipc host --pid host --gpus device=1 \
  -e ROS_DOMAIN_ID=8 -e RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
  -v "$D/nvblox_base.yaml":/yamls/nvblox_base.yaml:ro \
  -v "$D/specializations/nvblox_zed.yaml":/yamls/specializations/nvblox_zed.yaml:ro \
  --entrypoint bash nvblox-lite:3.2.5-humble -c 'source /opt/ros/humble/setup.bash && ros2 run nvblox_ros nvblox_node --ros-args --params-file /yamls/nvblox_base.yaml --params-file /yamls/specializations/nvblox_zed.yaml -p num_cameras:=1 -p global_frame:=fr3_link0 -p pose_frame:=fr3_link0 -p map_clearing_frame_id:=fr3_link0 -p map_clearing_radius_m:=-1.0 -p use_lidar:=false -p use_depth:=true -p use_color:=true -p input_qos:=SENSOR_DATA -p use_tf_transforms:=true -p voxel_size:=0.02 '"$EXTRA"' -p integrate_depth_rate_hz:=5.0 -p integrate_color_rate_hz:=5.0 -p update_mesh_rate_hz:=2.0 -p update_esdf_rate_hz:=2.0 -p esdf_mode:=3d -p static_mapper.esdf_slice_min_height:=0.02 -p static_mapper.esdf_slice_max_height:=0.8 -p static_mapper.esdf_slice_height:=0.15 -p static_mapper.projective_integrator_max_integration_distance_m:=2.5 -p static_mapper.projective_integrator_truncation_distance_vox:=4.0 -p max_back_projection_distance:=7.0 -r __node:='"$NODE"' -r camera_0/depth/image:=/vm/cable/depth -r camera_0/depth/camera_info:=/zed1/zed_node/depth/camera_info -r camera_0/color/image:=/vm/cable/rgb -r camera_0/color/camera_info:=/zed1/zed_node/left/camera_info'
