#!/usr/bin/env bash
# Record a ZED-teleop session for offline/sim review: two time-synchronised MCAP bags (zstd), one per ROS domain.
#   bash record_zed_teleop.sh start [name]    ->  ~/spot-teleop/bags/<stamp>_<name>/{d8,d77}
#   bash record_zed_teleop.sh zed [name] [raw]  ->  ZED-only session: just ~/spot-teleop/bags/<stamp>_<name>/d8 with the operator ZED3
#                                                  (colour, depth, camera_info, plus the hand tracker outputs and /tf_static); no robot
#                                                  bag. "raw" records the depth uncompressed (lossless 32FC1, ~25 MB/s more).
#   REC_DIR=session/stage bash record_zed_teleop.sh start  ->  ~/spot-teleop/bags/session/stage/{d8,d77}  (used by zed_protocol)
#   bash record_zed_teleop.sh stop            ->  stops both, copies the robot-domain bag next to the perception one, prints sizes
# d8  (fr3-curobo, perception): ZED3 colour (compressed) + depth (compressedDepth) + camera_info, hand tracker outputs
#     (/fr3_operator/*: teleop_sample = the 6D hand sample sent to the teleop, tracking, hand_camera_pose), cuRobo goal/preview,
#     relayed /joint_states, /tf, /tf_static. Enough to re-run the hand tracker offline on the same images.
# d77 (fr3-ros2, robot): teleop inputs/outputs (/teleop/*, equilibrium targets), the cuRobo bridge (/fr3_curobo/*, JTC commands),
#     what the robot did (joint states, desired joints, O_T_EE pose, external wrench, robot_state, gripper), /tf.
set -uo pipefail
ROOT=$HOME/spot-teleop/bags; STATE=$ROOT/.recording
D8_TOPICS="/zed3/zed_node/rgb/image_rect_color/compressed /zed3/zed_node/rgb/camera_info \
 /zed3/zed_node/depth/depth_registered/compressedDepth /zed3/zed_node/depth/camera_info \
 /fr3_operator/teleop_sample /fr3_operator/tracking /fr3_operator/hand_camera_pose \
 /fr3_joystick_mpc/wrist_goal /fr3_joystick_mpc/joint_preview /fr3_joystick_mpc/state /fr3_zed_mpc/wrist_goal \
 /joint_states /tf /tf_static"
D77_TOPICS="/teleop/command_tool /teleop/engaged /teleop/input_source /teleop/zed_status /teleop/speed_scale \
 /cartesian_impedance_target_controller/equilibrium_pose /fr3_curobo/equilibrium_pose /fr3_curobo/status \
 /fr3_arm_controller/joint_trajectory /joint_states /franka_gripper/joint_states \
 /franka_robot_state_broadcaster/current_pose /franka_robot_state_broadcaster/measured_joint_states \
 /franka_robot_state_broadcaster/desired_joint_states /franka_robot_state_broadcaster/external_wrench_in_base_frame \
 /franka_robot_state_broadcaster/robot_state /joy_remote /tf /tf_static"
ZED_TOPICS="/zed3/zed_node/rgb/image_rect_color/compressed /zed3/zed_node/rgb/camera_info /zed3/zed_node/depth/camera_info \
 /fr3_operator/teleop_sample /fr3_operator/tracking /fr3_operator/hand_camera_pose /tf_static"
case "${1:-}" in
  zed)
    [ -f "$STATE" ] && { echo "already recording: $(cat $STATE) (run: $0 stop)"; exit 1; }
    STAMP=$(date +%Y%m%d_%H%M%S)${2:+_$2}; mkdir -p "$ROOT/$STAMP"; [ -f "$ROOT/.gitignore" ] || echo "*" > "$ROOT/.gitignore"
    DEPTH=/zed3/zed_node/depth/depth_registered/compressedDepth; [ "${3:-}" = raw ] && DEPTH=/zed3/zed_node/depth/depth_registered
    docker exec -d fr3-curobo bash -c "source /opt/ros/humble/setup.bash; export ROS_DOMAIN_ID=8; mkdir -p /home/spot-teleop/bags/$STAMP; \
      echo \$\$ > /tmp/rec_d8.pid; exec ros2 bag record -s mcap --storage-preset-profile zstd_fast -o /home/spot-teleop/bags/$STAMP/d8 $ZED_TOPICS $DEPTH > /tmp/rec_d8.log 2>&1"
    echo "$STAMP zed" > "$STATE"; sleep 4; docker exec fr3-curobo tail -n 1 /tmp/rec_d8.log
    echo "recording ZED only: $STAMP (depth: $DEPTH)  (stop: bash $0 stop)" ;;
  start)
    [ -f "$STATE" ] && { echo "already recording: $(cat $STATE) (run: $0 stop)"; exit 1; }
    STAMP=${REC_DIR:-$(date +%Y%m%d_%H%M%S)${2:+_$2}}; mkdir -p "$ROOT/$STAMP";   # host-owned dir: the robot bag is copied in at stop
    R77=${STAMP//\//_}                                   # REC_DIR may be session/stage (zed_protocol): flat name inside fr3-ros2
    [ -f "$ROOT/.gitignore" ] || echo "*" > "$ROOT/.gitignore"
    docker exec -d fr3-curobo bash -c "source /opt/ros/humble/setup.bash; export ROS_DOMAIN_ID=8; mkdir -p /home/spot-teleop/bags/$STAMP; \
      echo \$\$ > /tmp/rec_d8.pid; exec ros2 bag record -s mcap --storage-preset-profile zstd_fast -o /home/spot-teleop/bags/$STAMP/d8 $D8_TOPICS > /tmp/rec_d8.log 2>&1"
    docker exec -d fr3-ros2 bash -c "source /opt/ros/jazzy/setup.bash; source /ros2_ws/install/setup.bash; export ROS_DOMAIN_ID=77 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST; \
      mkdir -p /runs/bags; echo \$\$ > /tmp/rec_d77.pid; exec ros2 bag record -s mcap --storage-preset-profile zstd_fast -o /runs/bags/${R77}_d77 $D77_TOPICS > /tmp/rec_d77.log 2>&1"
    echo "$STAMP" > "$STATE"; sleep 4
    docker exec fr3-curobo tail -n 2 /tmp/rec_d8.log; docker exec fr3-ros2 tail -n 2 /tmp/rec_d77.log
    echo "recording $STAMP  (stop: bash $0 stop)" ;;
  stop)
    [ -f "$STATE" ] || { echo "not recording"; exit 1; }; read STAMP MODE < "$STATE"
    docker exec fr3-curobo bash -c 'kill -INT $(cat /tmp/rec_d8.pid) 2>/dev/null'
    if [ "${MODE:-}" != zed ]; then
      docker exec fr3-ros2 bash -c 'kill -INT $(cat /tmp/rec_d77.pid) 2>/dev/null'
      for i in $(seq 1 20); do docker exec fr3-ros2 bash -c 'kill -0 $(cat /tmp/rec_d77.pid) 2>/dev/null' || break; sleep 0.5; done; sleep 1
      docker cp "fr3-ros2:/runs/bags/${STAMP//\//_}_d77" "$ROOT/$STAMP/d77" && echo "robot bag copied"
    fi
    for i in $(seq 1 20); do docker exec fr3-curobo bash -c 'kill -0 $(cat /tmp/rec_d8.pid) 2>/dev/null' || break; sleep 0.5; done
    rm -f "$STATE"; du -sh "$ROOT/$STAMP"/* 2>/dev/null; echo "saved: $ROOT/$STAMP" ;;
  *) echo "usage: bash $0 start [name] | zed [name] [raw] | stop"; exit 2 ;;
esac
