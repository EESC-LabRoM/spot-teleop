#!/usr/bin/env bash
set -euo pipefail
HERE=$(cd "$(dirname "$0")" && pwd)
REPO=$(cd "$HERE/../../../../.." && pwd)
IMAGE=${CONTROL_IMAGE:-fr3-curobo-remote-sim:checkpoint}
# Local snapshot contains the already compiled /opt/curobo and /opt/nvblox_msgs.
# Build once from the working native-Humble cuRobo container:
# docker commit --pause=false fr3-curobo fr3-curobo-remote-sim:checkpoint
if docker inspect fr3-remote-control >/dev/null 2>&1; then
 echo 'fr3-remote-control exists. Stop/remove only this container before restarting.';exit 1
fi
docker run -d --name fr3-remote-control --runtime=runc --device=nvidia.com/gpu=1 -e NVIDIA_VISIBLE_DEVICES=void --network host --ipc host \
 -v "$REPO:/home/spot-teleop" --entrypoint sleep "$IMAGE" infinity
# One visible physical GPU becomes CUDA device 0 inside this container.
docker exec -d fr3-remote-control bash -c '
 source /opt/ros/humble/setup.bash
 export ROS_DOMAIN_ID=188 CUDA_VISIBLE_DEVICES=0
 export PYTHONPATH=/home/spot-teleop/tmp/cv_site_hipolito:/home/spot-teleop/tmp/mp_site_hipolito:/home/spot-teleop/spot-ros2_ws/src/arm_pose_estimator:$PYTHONPATH:/opt/nvblox_msgs/local/lib/python3.10/dist-packages:/opt/nvblox_msgs/lib/python3.10/site-packages
 export LD_LIBRARY_PATH=/opt/nvblox_msgs/lib:${LD_LIBRARY_PATH:-}
 R=/home/spot-teleop/isaac-ros_ws/src/spot_nvblox/fr3
 python3 -u "$R/remote_sim/zed_rgbd_receiver.py" > /tmp/fr3_remote_rgbd.log 2>&1 &
 python3 -u "$R/remote_sim/operator_hand_tracker.py" > /tmp/fr3_remote_perception.log 2>&1 &
 exec python3 -u "$R/remote_sim/sim_session.py" > /tmp/fr3_remote_session.log 2>&1
'
echo 'Simulation viewer: http://100.88.134.127:8791/'
