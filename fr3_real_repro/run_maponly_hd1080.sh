#!/usr/bin/env bash
# MAP_ONLY static neural map at HD1080 (the clearing grab res): quantifies ESDF shrink
# for the config that dodges, vs +5.2 cm (SVGA) and +4.4 cm (exact).
cd /home/nexus/spot-teleop/isaac-sim_ws/franka_mpc || exit 1
export MAP_ONLY=1 MAP_SECS=90 REAL=1 VOXEL=0.05 ZED_WAIT=1500 ZED_YAML=sim_real.yaml NVLAUNCH=franka_zed_nvblox.launch.py
echo "RUN START $(date +%T)"
./run_obstacle.sh maponly_hd1080 --wait 1800 --zed-res HD1080 --zed-fps 15
echo "RUN FINISHED rc=$? $(date +%T)"
