#!/usr/bin/env bash
# Dedicated simulation controller + bag player; never runs the live UDP relay.
set -euo pipefail
HERE=$(cd "$(dirname "$0")" && pwd)
REPO=$(cd "$HERE/../../../../.." && pwd)
if [[ $# -lt 1 ]]; then
  echo "usage: bash $0 /absolute/path/to/session [recorded|retrack] [rate]"; exit 2
fi
BAG=$(realpath "$1"); MODE=${2:-recorded}; RATE=${3:-1}
[[ -d "$BAG/d8" ]] && BAG=$BAG/d8
[[ "$BAG" == "$REPO/bags/"* && -f "$BAG/metadata.yaml" ]] || { echo 'Expected a bag under spot-teleop/bags'; exit 2; }
[[ "$MODE" == recorded || "$MODE" == retrack ]] || { echo 'Expected recorded or retrack'; exit 2; }
python3 -c 'import sys,math; n=float(sys.argv[1]); assert math.isfinite(n) and .1<=n<=2' "$RATE"
if docker inspect fr3-bag-control >/dev/null 2>&1; then
  echo 'fr3-bag-control exists. Stop/remove that container before repeating.'; exit 1
fi
if docker inspect -f '{{.State.Running}}' fr3-remote-control 2>/dev/null | grep -qx true; then
  echo 'fr3-remote-control is running on DDS 188/HTTP 8791. Stop that simulation session first.'; exit 1
fi
docker run -d --name fr3-bag-control --gpus '"device=1"' --network host --ipc private --shm-size=1g \
  -e BAG="/home/spot-teleop/${BAG#"$REPO/"}" -e BAG_MODE="$MODE" -e BAG_RATE="$RATE" \
  -e FR3_SIM_POSITION_GAINS="${FR3_SIM_POSITION_GAINS:-1,1,1}" \
  -e FR3_SIM_ROTATION_GAIN="${FR3_SIM_ROTATION_GAIN:-1}" \
  -e ROS_DOMAIN_ID=188 -e RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
  -e FASTRTPS_DEFAULT_PROFILES_FILE=/home/spot-teleop/isaac-ros_ws/src/spot_nvblox/fr3/remote_sim/fastdds_udp_only.xml \
  -v "$REPO:/home/spot-teleop" --entrypoint bash "${CONTROL_IMAGE:-fr3-curobo-remote-sim:checkpoint}" -c '
  set -eo pipefail
  source /opt/ros/humble/setup.bash
  set -u
  export CUDA_VISIBLE_DEVICES=0
  export PYTHONPATH=/home/spot-teleop/tmp/cv_site_hipolito:/home/spot-teleop/tmp/mp_site_hipolito:/home/spot-teleop/spot-ros2_ws/src/arm_pose_estimator:${PYTHONPATH:-}:/opt/nvblox_msgs/local/lib/python3.10/dist-packages:/opt/nvblox_msgs/lib/python3.10/site-packages
  export LD_LIBRARY_PATH=/opt/nvblox_msgs/lib:${LD_LIBRARY_PATH:-}
  R=/home/spot-teleop/isaac-ros_ws/src/spot_nvblox/fr3/remote_sim
  trap "kill \$(jobs -pr) 2>/dev/null || true" EXIT
  if [[ "$BAG_MODE" == retrack ]]; then
    python3 -u "$R/operator_hand_tracker.py" --domain 188 --calibrate-arm --smooth-preview > /tmp/bag_tracker.log 2>&1 &
  fi
  python3 -u "$R/bag_replay.py" "$BAG" --mode "$BAG_MODE" --rate "$BAG_RATE" > /tmp/bag_replay.log 2>&1 &
  python3 -u "$R/sim_session.py" > /tmp/bag_session.log 2>&1 &
  wait -n
  '
echo 'Bag paused: http://100.88.134.127:8792/ | Robot simulation: http://100.88.134.127:8791/'
echo 'Start Isaac separately with: bash run_isaac.sh'
