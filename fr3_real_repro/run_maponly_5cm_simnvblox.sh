#!/usr/bin/env bash
# MAP_ONLY vs run 5: same stack (5cm, sim/Spot nvblox params, real 10-20 poses, SVGA light),
# NO node, NO sweep. Static pre-contact map + raw depth frames. Answers: is the veja in the map at start pose?
cd /home/nexus/spot-teleop/isaac-sim_ws/franka_mpc || exit 1
export REAL=1 VOXEL=0.05 ZED_WAIT=1500 ZED_YAML=sim_real_svga.yaml NVLAUNCH=franka_zed_nvblox.launch.py
export MAP_ONLY=1 MAP_SECS=150
echo "RUN START $(date +%T)"
./run_obstacle.sh maponly_5cm_simnvblox --wait 1800 --zed-res SVGA --zed-fps 15
echo "RUN FINISHED rc=$? $(date +%T)"
