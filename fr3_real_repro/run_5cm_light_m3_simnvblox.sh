#!/usr/bin/env bash
# Single-factor vs run 3 (real-like 5cm light margin3cm): ONLY nvblox params -> sim/Spot defaults.
# No min-weight filter, TSDF max weight 100, truncation 6 vox (franka_zed_nvblox.launch.py).
# Same 5cm voxels, margin 3.4 total, vcap1.2, real 10-20 poses, SVGA light.
cd /home/nexus/spot-teleop/isaac-sim_ws/franka_mpc || exit 1
export REAL=1 VOXEL=0.05 ZED_WAIT=1500 ZED_YAML=sim_real_svga.yaml NVLAUNCH=franka_zed_nvblox.launch.py
export NODE_ARGS="-p extra_collision_sphere_buffer:=0.03 -p robot_config:=/workspace/franka_mpc/node_config/fr3_vcap1.2_acc10.yml -p target_clear_frame:=fr3_hand -p target_clear_radius_m:=0.15 -p grid_size_m:=[1.4,1.6,1.2] -p grid_center_m:=[0.35,0.0,0.55]"
echo "RUN START $(date +%T)"
./run_obstacle.sh real_like_5cm_light_m3_simnvblox --wait 1800 --zed-res SVGA --zed-fps 15
echo "RUN FINISHED rc=$? $(date +%T)"
