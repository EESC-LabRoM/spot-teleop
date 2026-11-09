#!/usr/bin/env bash
# target_mask.py in the nvblox-lite image (ROS 2 Humble): the grasp target's mask per ZED depth frame for nvblox's human layer.
#   RUN=<run dir name under franka_mpc/runs> [CAMS="zed zed2"] [DILATE=3] [EXTRA=12,20] [SNAP=4] ./run_target_mask.sh      stop: docker stop target-mask
set -euo pipefail
FM=/home/nexus/spot-teleop/isaac-sim_ws/franka_mpc; CAMS=${CAMS:-zed zed2}
docker run --rm --name target-mask --network host --ipc host -e ROS_DOMAIN_ID=8 -e PYTHONUNBUFFERED=1 \
  -v "$FM":/fm -v "$FM/runs/$RUN":/run nvblox-lite:3.2.5-humble bash -c \
  "source /opt/ros/humble/setup.bash && python3 /fm/target_mask.py --run /run --workcell-scene /fm/runs/workcell/scene.json --cams $(echo $CAMS | tr ' ' ,) --mode ${MODE:-human} ${CLEARBOX_FROM_MASK:+--clear-box} --dilate ${DILATE:-3} ${OCCL:+--occlusion-m $OCCL} ${EXTRA:+--extra-dilate $EXTRA} --snap-every ${SNAP:-0}; chown -R 1000:1000 /run"
