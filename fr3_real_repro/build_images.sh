#!/usr/bin/env bash
# Rebuild the FR3-sim images on the VM from public sources (two parallel streams). Logs next to this script.
set -uo pipefail
D=/home/nexus/spot-teleop/fr3_real_repro; H=/home/nexus
(
  echo "[isaac] $(date +%T) pull nvcr.io/nvidia/isaac-sim:6.0.0"
  docker pull nvcr.io/nvidia/isaac-sim:6.0.0 || { echo "[isaac] PULL FAILED"; exit 1; }
  echo "[isaac] $(date +%T) build spot-isaac-sim:6.0.0"
  docker build --progress=plain -t spot-isaac-sim:6.0.0 $H/spot-isaac-sim/docker || { echo "[isaac] BUILD 6.0.0 FAILED"; exit 1; }
  echo "[isaac] $(date +%T) build spot-isaac-sim:6.0.0-zed"
  docker build --progress=plain -t spot-isaac-sim:6.0.0-zed -f $H/spot-teleop/isaac-sim_ws/franka_mpc/docker/Dockerfile.isaac-zed $H/spot-teleop/isaac-sim_ws/franka_mpc/docker || { echo "[isaac] BUILD -zed FAILED"; exit 1; }
  echo "[isaac] $(date +%T) DONE"
) > $D/build_isaac.log 2>&1 &
(
  echo "[zed] $(date +%T) build zed-lite:5.4.1-humble"
  docker build --progress=plain -t zed-lite:5.4.1-humble -f $H/spot-teleop/zed_lite/Dockerfile $H/spot-teleop/zed_lite/ctx || { echo "[zed] BUILD zed-lite FAILED"; exit 1; }
  echo "[zed] $(date +%T) build nvblox-lite:3.2.5-humble"
  docker build --progress=plain -t nvblox-lite:3.2.5-humble -f $H/spot-teleop/zed_lite/nvblox/Dockerfile $H/spot-teleop/zed_lite/nvblox/ctx || { echo "[zed] BUILD nvblox-lite FAILED"; exit 1; }
  echo "[zed] $(date +%T) DONE"
) > $D/build_zed.log 2>&1 &
wait
echo "ALL FINISHED $(date +%T)" > $D/build_images.done
