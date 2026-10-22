#!/usr/bin/env bash
# The ONE nvblox for cuRobo on the REAL FR3 (isaac-build, domain 8, GPU 1): node /nvblox_node, service /nvblox_node/get_esdf_and_gradient,
# frame fr3_link0, 2 cm voxels, ESDF 2 Hz, parameters of run_nvblox_real.sh (same base yamls). PASSIVE: no robot commands.
#   MODE=masked (default)  integrates /zedN/arm_filter/depth (arm + tracked object = NaN, fr3_arm_mask_node.py in zed-build must run)
#   MODE=raw               integrates the raw ZED depth (= the old run_nvblox_real.sh behaviour)
#   CAMERAS=zed1,zed2 (default); zed3 is a fixed camera on the OPERATOR, not on the FR3 workspace and uncalibrated: only add it with a real
#   zed3 extrinsic published (fr3/zed3/README.md) AND fr3_arm_mask_node.py started with the same --cameras.
# Needs launch file /tmp/fr3_nvblox.launch.py in the container (docker cp .../spot_nvblox/launch/fr3_nvblox.launch.py isaac-build:/tmp/).
source /opt/ros/humble/setup.bash
source /workspaces/isaac_ros-dev/install/setup.bash
export CUDA_VISIBLE_DEVICES=1 RMW_IMPLEMENTATION=rmw_fastrtps_cpp ROS_DOMAIN_ID=8
case "${MODE:-masked}" in masked) AF=true ;; raw) AF=false ;; *) echo "MODE=masked|raw"; exit 1 ;; esac
exec nice -n 19 taskset -c 12-15 ros2 launch /tmp/fr3_nvblox.launch.py node_name:=${NODE_NAME:-nvblox_node} arm_filter:=$AF cameras:=${CAMERAS:-zed1,zed2}
