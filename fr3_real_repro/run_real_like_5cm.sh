#!/usr/bin/env bash
# Real-like avoidance run: lab-calibrated ZEDs + real nvblox params + Hipolito-like MPC settings (5 cm voxels, 4 mm margin, 0.15 m hand clear sphere, his grid)
cd /home/nexus/spot-teleop/isaac-sim_ws/franka_mpc || exit 1
export REAL=1 VOXEL=0.05 ZED_WAIT=1500
export NODE_ARGS="-p extra_collision_sphere_buffer:=0.0 -p robot_config:=/workspace/franka_mpc/node_config/fr3_vcap1.2_acc10.yml -p target_clear_frame:=fr3_hand -p target_clear_radius_m:=0.15 -p grid_size_m:=[1.4,1.6,1.2] -p grid_center_m:=[0.35,0.0,0.55]"
echo "RUN START $(date +%T)"
./run_obstacle.sh real_like_5cm --wait 1800
echo "RUN FINISHED rc=$? $(date +%T)"
