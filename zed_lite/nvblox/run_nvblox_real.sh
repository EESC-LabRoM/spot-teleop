#!/usr/bin/env bash
# REAL lab: nvblox on the two ZED wrappers (zed1, zed2), mapping in the FR3 base frame. The camera->base static TFs
# (fr3_link0 -> zedN_camera_link) come from real_extrinsics_tf.py (calibrated, see easyhec results). PASSIVE: no robot commands.
source /opt/ros/humble/setup.bash
source /workspaces/isaac_ros-dev/install/setup.bash
export CUDA_VISIBLE_DEVICES=1 RMW_IMPLEMENTATION=rmw_fastrtps_cpp ROS_DOMAIN_ID=8
B=/workspaces/isaac_ros-dev/src/isaac_ros_nvblox/nvblox_examples/nvblox_examples_bringup/config/nvblox
F=fr3_link0
exec ros2 run nvblox_ros nvblox_node --ros-args \
  --params-file $B/nvblox_base.yaml --params-file $B/specializations/nvblox_zed.yaml \
  -p num_cameras:=2 -p global_frame:=$F -p pose_frame:=$F -p map_clearing_frame_id:=$F -p map_clearing_radius_m:=-1.0 \
  -p esdf_slice_bounds_visualization_attachment_frame_id:=$F -p workspace_height_bounds_visualization_attachment_frame_id:=$F \
  -p use_lidar:=false -p use_depth:=true -p use_color:=true -p input_qos:=SENSOR_DATA -p use_tf_transforms:=true \
  -p voxel_size:=0.02 -p mapping_type:=static_tsdf -p use_segmentation:=false \
  -p integrate_depth_rate_hz:=5.0 -p integrate_color_rate_hz:=5.0 -p update_mesh_rate_hz:=2.0 -p update_esdf_rate_hz:=2.0 \
  -p esdf_mode:=3d -p static_mapper.esdf_slice_min_height:=0.02 -p static_mapper.esdf_slice_max_height:=0.8 -p static_mapper.esdf_slice_height:=0.15 \
  -p static_mapper.projective_integrator_max_integration_distance_m:=2.5 -p static_mapper.projective_integrator_truncation_distance_vox:=6.0 \
  -p max_back_projection_distance:=7.0 -p layer_streamer_bandwidth_limit_mbps:=-1.0 -p decay_tsdf_rate_hz:=0.0 -p print_rates_to_console:=true \
  -r camera_0/depth/image:=/zed1/zed_node/depth/depth_registered -r camera_0/depth/camera_info:=/zed1/zed_node/depth/camera_info \
  -r camera_0/color/image:=/zed1/zed_node/rgb/image_rect_color -r camera_0/color/camera_info:=/zed1/zed_node/rgb/camera_info \
  -r camera_1/depth/image:=/zed2/zed_node/depth/depth_registered -r camera_1/depth/camera_info:=/zed2/zed_node/depth/camera_info \
  -r camera_1/color/image:=/zed2/zed_node/rgb/image_rect_color -r camera_1/color/camera_info:=/zed2/zed_node/rgb/camera_info
