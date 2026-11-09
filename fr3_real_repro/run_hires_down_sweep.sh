#!/usr/bin/env bash
# Grab-vs-pub split vs run C: SAME HD1080 grab, but downscaled publishing (640x360,
# sim_real.yaml) instead of NATIVE 1920x1080. Everything else = run 5/C stack.
# Clears -> grab detail is the key. Hits -> published density into nvblox is the key.
cd /home/nexus/spot-teleop/isaac-sim_ws/franka_mpc || exit 1
export REAL=1 VOXEL=0.05 ZED_WAIT=1500 ZED_YAML=sim_real.yaml NVLAUNCH=franka_zed_nvblox.launch.py
export NODE_ARGS="-p extra_collision_sphere_buffer:=0.03 -p robot_config:=/workspace/franka_mpc/node_config/fr3_vcap1.2_acc10.yml -p target_clear_frame:=fr3_hand -p target_clear_radius_m:=0.15 -p grid_size_m:=[1.4,1.6,1.2] -p grid_center_m:=[0.35,0.0,0.55]"
echo "RUN START $(date +%T)"
./run_obstacle.sh real_like_5cm_light_hires --wait 1800 --zed-res HD1080 --zed-fps 15
echo "RUN FINISHED rc=$? $(date +%T)"
