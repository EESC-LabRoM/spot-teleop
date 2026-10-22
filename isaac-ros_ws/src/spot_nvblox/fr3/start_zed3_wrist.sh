#!/usr/bin/env bash
# Host side: start the OPERATOR-WRIST perception on zed3 for the FR3 (OFF by default; nothing runs until you call this).
# zed3 = ZED 2i serial 34046618, fixed on a tripod at the operator station (not on the robot). PERCEPTION ONLY: no robot command is sent.
#   bash start_zed3_wrist.sh                      wrist_detector on /zed3 -> /fr3_operator/wrist_pose (+ TF fr3_link0 -> wrist_target)
#   HAND_ORIENTATION=1 bash start_zed3_wrist.sh   also hand_orientation_estimator (hand roll quaternion)
#   TF=1 bash start_zed3_wrist.sh                 also publish the static TF fr3_link0 -> zed3_camera_link from zed3/zed3_extrinsic_*.json
#                                                  (refused while the file says "calibrated": false). Only needed for nvblox, not for the wrist.
#   DRY=1 bash start_zed3_wrist.sh                print what would run, check prerequisites, start nothing
# Stop with stop_zed3_wrist.sh. Runs in fr3-curobo (domain 8, mediapipe in a private target dir), TF helper in zed-build. Never touches
# /zed1, /zed2, /nvblox_node or the zed3 wrapper itself.
set -u
HERE=$(cd "$(dirname "$0")" && pwd); Z=$HERE/zed3
C=${CONTAINER:-fr3-curobo}; ZC=zed-build; DOM=8
CALIB=${CALIB:-$Z/zed3_extrinsic_PLACEHOLDER.json}
# host path -> container path (the repo is mounted at /home/spot-teleop in fr3-curobo)
CP() { echo "${1/#\/home\/hipolito\/spot-teleop//home/spot-teleop}"; }
SRC=/home/spot-teleop/spot-ros2_ws/src/arm_pose_estimator; MPSITE=/home/spot-teleop/tmp/mp_site_hipolito; CVSITE=/home/spot-teleop/tmp/cv_site_hipolito   # private pip dirs: mediapipe 0.10.14, opencv-contrib 4.8 (the container cv2 4.5.4 has no cv2.aruco.DetectorParameters)
LAUNCH=$(CP "$Z/fr3_zed3_wrist.launch.py"); PARAMS=$(CP "$Z/fr3_operator_wrist.yaml")
HO=${HAND_ORIENTATION:-0}; TFON=${TF:-0}; TRACKER=${TRACKER:-direct}
[[ "$TRACKER" == direct || "$TRACKER" == legacy ]] || { echo "TRACKER must be direct or legacy"; exit 1; }
OM=$(python3 -c "import yaml; print(yaml.safe_load(open('$Z/fr3_operator_wrist.yaml'))['wrist_detector']['ros__parameters'].get('orientation_mode','roll'))")
# Full 6D uses synchronized RGB-D + Hands inside wrist_detector; no separate roll node.
[ "$OM" = full_6d ] && HO=0
ENVC="source /opt/ros/humble/setup.bash; export ROS_DOMAIN_ID=$DOM RMW_IMPLEMENTATION=rmw_fastrtps_cpp CUDA_VISIBLE_DEVICES=1 PYTHONPATH=$CVSITE:\$PYTHONPATH:$SRC:$MPSITE"
HOST_OK() { [ -f "$1" ] && echo ok || echo MISSING; }

echo "plan: container=$C domain=$DOM launch=$LAUNCH params=$PARAMS hand_orientation=$HO static_tf=$TFON calib=$CALIB"
echo "      cmd: ros2 launch $LAUNCH params_file:=$PARAMS hand_orientation:=$([ "$HO" = 1 ] && echo true || echo false)"
echo "      files: launch $(HOST_OK "$Z/fr3_zed3_wrist.launch.py"), params $(HOST_OK "$Z/fr3_operator_wrist.yaml"), calib $(HOST_OK "$CALIB")"
CAL=$(python3 -c "import json,sys;print(str(json.load(open(sys.argv[1])).get('calibrated',True)).lower())" "$CALIB" 2>/dev/null || echo unreadable)
echo "      calibration flag in $(basename "$CALIB"): calibrated=$CAL"
if [ "${DRY:-0}" = 1 ]; then
  echo "DRY: prerequisite checks (read-only)"
  docker exec $C bash -c "$ENVC; python3 -c 'import mediapipe,cv2,cv_bridge,message_filters;cv2.aruco.DetectorParameters();print(\"  imports ok: mediapipe\",mediapipe.__version__,\"cv2\",cv2.__version__)' 2>&1 | tail -1; python3 -c 'import arm_pose_estimator.wrist_detector' 2>&1 | tail -1; ros2 topic list --no-daemon --spin-time 3 2>/dev/null | grep -c '^/zed3/zed_node/depth/depth_registered\$' | sed 's/^/  zed3 depth topic present (1=yes): /'" 2>&1
  [ "$TFON" = 1 ] && [ "$CAL" != true ] && echo "  NOTE: TF=1 would be refused (not calibrated)"
  echo "DRY: nothing started"; exit 0
fi
[ -f "$Z/fr3_zed3_wrist.launch.py" ] && [ -f "$Z/fr3_operator_wrist.yaml" ] || { echo "zed3 files missing in $Z"; exit 1; }
docker exec $C bash -c "$ENVC; ros2 topic list --no-daemon --spin-time 3 2>/dev/null | grep -q '^/zed3/zed_node/depth/depth_registered\$'" || { echo "zed3 wrapper topics not visible in domain $DOM (is /zed3/zed_node up in zed-build? /tmp/start_zed3.sh)"; exit 1; }
docker exec $C bash -c "$ENVC; python3 -c 'import mediapipe,cv2;cv2.aruco.DetectorParameters()'" 2>/dev/null || { echo "mediapipe / cv2>=4.7 missing: docker exec $C pip install --no-deps --target $MPSITE mediapipe==0.10.14; ... --target $CVSITE opencv-contrib-python-headless==4.8.1.78"; exit 1; }
if [ "$(docker exec $C bash -c 'cat /tmp/zed3_wrist.pid 2>/dev/null')" != "" ]; then
  docker exec $C bash -c 'kill -0 $(cat /tmp/zed3_wrist.pid) 2>/dev/null' && { echo "already running (pid $(docker exec $C cat /tmp/zed3_wrist.pid)); stop_zed3_wrist.sh first"; exit 1; }
fi
if [ "$TFON" = 1 ]; then
  [ "$CAL" = true ] || { echo "TF=1 refused: $CALIB is not a calibration (calibrated=$CAL). See fr3/zed3/README.md"; exit 1; }
  docker exec $ZC bash -c 'test -f /tmp/real_extrinsics_tf.py' || docker cp /home/murilo/zed_lite/nvblox/real_extrinsics_tf.py $ZC:/tmp/real_extrinsics_tf.py
  docker cp "$CALIB" $ZC:/tmp/zed3_extrinsic.json
  docker exec -d $ZC bash -c "source /opt/ros/humble/setup.bash; source /home/spot_ws/zed_ws/install/setup.bash; export ROS_DOMAIN_ID=$DOM; cd /home/spot_ws/zed_ws; python3 /tmp/real_extrinsics_tf.py --calib /tmp/zed3_extrinsic.json --cam zed3 > /tmp/zed3_tf.log 2>&1 & echo \$! > /tmp/zed3_tf.pid; wait"
  echo "static TF started in $ZC (pid file $ZC:/tmp/zed3_tf.pid, log /tmp/zed3_tf.log)"
fi
if [ "$TRACKER" = direct ]; then
  DIRECT=/home/spot-teleop/isaac-ros_ws/src/spot_nvblox/fr3/remote_sim/operator_hand_tracker.py
  docker exec -d "$C" bash -c "$ENVC; echo \$\$ > /tmp/zed3_wrist.pid; exec python3 -u $DIRECT --domain 8 --calibrate-arm --smooth-preview --preview-port 8786 --calibration-port 47815 --output-prefix /fr3_operator --max-input-age .25 --max-output-age .35 --ros-args -r /fr3_sim/operator/rgb:=/zed3/zed_node/rgb/image_rect_color -r /fr3_sim/operator/depth:=/zed3/zed_node/depth/depth_registered -r /fr3_sim/operator/camera_info:=/zed3/zed_node/depth/camera_info -r /fr3_operator/sample:=/fr3_operator/teleop_sample -r /fr3_operator/overlay:=/fr3_operator/wrist_overlay > /tmp/zed3_wrist.log 2>&1"
else
docker exec -d $C bash -c "$ENVC; echo \$\$ > /tmp/zed3_wrist.pid; exec ros2 launch $LAUNCH params_file:=$PARAMS hand_orientation:=$([ "$HO" = 1 ] && echo true || echo false) > /tmp/zed3_wrist.log 2>&1"
fi
# Humble -> Jazzy uses loopback UDP, avoiding cross-distribution DDS transport issues.
RELAY=/home/spot-teleop/isaac-ros_ws/src/spot_nvblox/fr3/teleop/zed_input_relay.py
if ! docker exec "$C" pgrep -f "^python3 $RELAY" >/dev/null; then
  docker exec -d "$C" bash -c "source /opt/ros/humble/setup.bash; export ROS_DOMAIN_ID=$DOM; echo \$\$ > /tmp/zed3_input_relay.pid; exec python3 $RELAY --orientation-mode $OM > /tmp/zed_input_relay.log 2>&1"
fi
sleep 3
echo "started: pid $(docker exec $C cat /tmp/zed3_wrist.pid) in $C, log $C:/tmp/zed3_wrist.log; output /fr3_operator/wrist_pose"
