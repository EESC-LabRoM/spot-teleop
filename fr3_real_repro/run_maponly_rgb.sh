#!/usr/bin/env bash
# MAP_ONLY + RGB: same env as run 2 (real-like 5cm, Spot nvblox params come from REAL=1... note: this reuses run-5-era env).
cd /home/nexus/spot-teleop/isaac-sim_ws/franka_mpc || exit 1
export REAL=1 VOXEL=0.05 ZED_WAIT=300 ZED_YAML=sim_real_svga.yaml
export MAP_ONLY=1 MAP_SECS=150
echo "RUN START 21:34:57"
./run_obstacle.sh maponly_5cm_rgb --wait 1800 --zed-res SVGA --zed-fps 15
echo "RUN FINISHED rc=0 21:34:57"
