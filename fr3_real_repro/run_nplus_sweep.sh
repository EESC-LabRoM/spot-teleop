#!/usr/bin/env bash
# Single factor vs run 5 (real_like_5cm_light_m3_simnvblox): ONLY the ZED depth network
# NEURAL_LIGHT -> NEURAL_PLUS (same SVGA/15fps/downscale-1.5 streaming, 5cm, sim nvblox, 3.4cm margin, vcap1.2, real poses).
cd /home/nexus/spot-teleop/isaac-sim_ws/franka_mpc || exit 1
export REAL=1 VOXEL=0.05 ZED_WAIT=300 ZED_YAML=sim_neuralplus_svga.yaml NVLAUNCH=franka_zed_nvblox.launch.py
export NODE_ARGS="-p extra_collision_sphere_buffer:=0.03 -p robot_config:=/workspace/franka_mpc/node_config/fr3_vcap1.2_acc10.yml -p target_clear_frame:=fr3_hand -p target_clear_radius_m:=0.15 -p grid_size_m:=[1.4,1.6,1.2] -p grid_center_m:=[0.35,0.0,0.55]"
echo "RUN START $(date +%T)"
./run_obstacle.sh real_like_5cm_nplus --wait 1800 --zed-res SVGA --zed-fps 15
echo "RUN FINISHED rc=$? $(date +%T)"
