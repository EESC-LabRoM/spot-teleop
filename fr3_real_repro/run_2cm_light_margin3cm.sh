#!/usr/bin/env bash
# Single-factor vs run 3 (real-like 5cm light margin3cm): ONLY voxel 5cm -> 2cm.
# Same margin 3.4cm total (extra 0.03 + base 0.004), vcap1.2, real poses, real nvblox params, SVGA light.
cd /home/nexus/spot-teleop/isaac-sim_ws/franka_mpc || exit 1
export REAL=1 VOXEL=0.02 ZED_WAIT=1500 ZED_YAML=sim_real_svga.yaml
export NODE_ARGS="-p extra_collision_sphere_buffer:=0.03 -p robot_config:=/workspace/franka_mpc/node_config/fr3_vcap1.2_acc10.yml -p target_clear_frame:=fr3_hand -p target_clear_radius_m:=0.15 -p grid_size_m:=[1.4,1.6,1.2] -p grid_center_m:=[0.35,0.0,0.55]"
echo "RUN START $(date +%T)"
./run_obstacle.sh real_like_2cm_light_margin3cm --wait 1800 --zed-res SVGA --zed-fps 15
echo "RUN FINISHED rc=$? $(date +%T)"
