#!/usr/bin/env bash
# buf3 replica: old L40S working setup (10-14 obstacle_2zed_buf3) on the A2000.
# 2cm voxels, sim stands (no REAL), NEURAL_PLUS full-res (sim_quality.yaml),
# sim nvblox defaults (franka_zed_nvblox.launch.py), franka.yml, 3.4cm margin, no clear sphere.
cd /home/nexus/spot-teleop/isaac-sim_ws/franka_mpc || exit 1
export VOXEL=0.02 ZED_WAIT=900 ZED_YAML=sim_quality.yaml NVLAUNCH=franka_zed_nvblox.launch.py
export NODE_ARGS="-p extra_collision_sphere_buffer:=0.03 -p robot_config:=/workspace/franka_mpc/node_config/fr3_fullspeed.yml"
echo "RUN START $(date +%T)"
./run_obstacle.sh buf3_replica --wait 1800 --zed-res HD1080 --zed-fps 15
echo "RUN FINISHED rc=$? $(date +%T)"
