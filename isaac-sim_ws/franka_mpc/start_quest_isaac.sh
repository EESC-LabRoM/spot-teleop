#!/usr/bin/env bash
# Meta Quest teleop of the simulated FR3 in the Isaac HiveBoard valve scene. 10-20: the headset shows a 3D TWIN (real FR3 meshes + valve + panel, FK in the page, joint states at 20 Hz)
#   in your room; the Isaac camera JPEG screen is opt-in (VIDEO=1 here and ?video=1 in the headset URL).
#   Isaac (franka_ros_arm_teleop.py: valve scene, view camera -> JPEG on /isaac/view/compressed)  <-lockstep->  cuRobo MPC node (node_prof, /wrist_pose -> joint commands)
#   Quest side (container quest-isaac-quest = a FORK of the Quest lane's package, quest_isaac/pkg): isaac_quest_bridge (joint states/TF/gripper),
#   quest_wrist_teleop (hand tracking server + the Isaac screen for the headset), quest_fr3_teleop (hand -> /wrist_pose, engage, workspace box).
# Own ROS domain (DOM, default 41; never 8 = the Quest lane's live node, 77 = the real FR3, 78). Own ports (8446 HTTPS, 8091 HTTP): the lane's node on 8444/8090 is left alone.
#   [DOM=41] [HTTPS_PORT=8446] [HTTP_PORT=8091] [OPERATOR_YAW_DEG=0|180] [VIEW="ex ey ez tx ty tz focal"] [RES="640 360"] [VIDEO=0|1] [FPS=4] [JPEGQ=70] [RENDER_EVERY=5] [FREE_RUN=1|0] [READY=1|0] ./start_quest_isaac.sh     stop: ./stop_quest_isaac.sh
set -uo pipefail
cd "$(dirname "$0")"; FM=$PWD
DOM=${DOM:-41}; case "$DOM" in 8|77|78) echo "refusing ROS_DOMAIN_ID $DOM (the Quest lane's live node / the real FR3 / Newton)"; exit 1;; esac
OPERATOR_YAW_DEG=${OPERATOR_YAW_DEG:-0.0}    # rotates operator +X (head-forward) into the FR3 base frame; use 180 when facing the robot
HTTPS=${HTTPS_PORT:-8446}; HTTP=${HTTP_PORT:-8091}; RES=${RES:-"640 360"}; FPS=${FPS:-4}; JPEGQ=${JPEGQ:-70}; RE=${RENDER_EVERY:-5}
VIEW=${VIEW:-"-2.1 -0.35 1.95 -0.65 -0.52 0.9 14"}                      # over the operator-side shoulder of the robot (behind, high), looking down at the arm and the board
RUN=quest_isaac_run; OUT=$FM/runs/$RUN; mkdir -p "$OUT/mpc" "$FM/quest_isaac/cert"
IMG=spot-isaac-sim:6.0.0
[ "${QUEST_ONLY:-0}" = 1 ] && CHK="quest-isaac-quest" || CHK="quest-isaac-sim franka-mpc quest-isaac-quest"     # QUEST_ONLY=1: restart only the Quest side (Isaac + MPC keep running)
for c in $CHK; do docker ps --format '{{.Names}}' | grep -qx "$c" && { echo "$c is already running: ./stop_quest_isaac.sh first"; exit 1; }; done
for p in $HTTPS $HTTP; do ss -ltn | grep -q ":$p " && { echo "port $p is in use"; exit 1; }; done
for i in $IMG zed-lite:5.4.1-humble; do docker image inspect "$i" >/dev/null 2>&1 || { echo "missing image $i"; exit 1; }; done
wait_for() { local t=0; until grep -qa "$2" "$1" 2>/dev/null; do sleep 1; t=$((t + 1)); [ $t -ge "$3" ] && { echo "timeout waiting for '$2' in $1"; tail -15 "$1"; return 1; }; done;  return 0; }
if [ "${QUEST_ONLY:-0}" != 1 ]; then
echo "[1/3] Isaac valve scene (teleop mode, domain $DOM)"
( DOM=$DOM NAME=quest-isaac-sim ROS=jazzy ./run_isaac.sh -s franka_ros_arm_teleop.py $RUN --stage /workspace/franka_mpc/franka_workcell_hive.usda --episodes 1 --seconds 100000 \
    --render-every $RE --video-res $RES --fps $FPS --jpeg-q $JPEGQ --view $VIEW $([ "${VIDEO:-0}" = 1 ] || echo --no-video) $([ "${FREE_RUN:-1}" = 1 ] && echo --free-run) > "$OUT/isaac.log" 2>&1 & )
wait_for "$OUT/isaac.log" "TELEOP loop" 420 || exit 1
echo "[2/3] cuRobo MPC node (free space, no map, /wrist_pose goal)"
( OUT=$OUT/mpc DOM=$DOM ./valve_stack/run_mpc_node.sh -p step_on_clock:=true -p pf_mode:=none > "$OUT/mpc.log" 2>&1 & )
wait_for "$OUT/mpc.log" "Initialized goal to current physical pose" 240 || exit 1
fi
echo "[3/3] Quest side (bridge + hand-tracking server with the Isaac screen + FR3 teleop)"
# Sim-only tuning of the FR3 follower (its defaults are the real-robot pad's: 0.25 m/s, 4 cm leash; a hand waved at 0.5 m/s got 1/3 of the motion at 0.5-2 s lag, 10-20) and of the hand frame
# (the head-anchored comfort box pinned 66 of 74 hand samples of the 10-20 headset session, so the arm stopped following). Hand orientation directly sets the base-frame TCP orientation.
# 10-20 night: the user asked for the safety limits OFF in the simulation (cuRobo's own limits do the limiting): `unlimited` = no follower lag/step/lead/slew, no leash to the measured pose, no workspace box,
# no rotation cap (node defaults stay the real pad's; the node refuses `unlimited` unless the setup is recognised as a simulation). Orientation always follows the hand in the robot base frame: hand +X points along TCP +Z (approach).
# No rotation selector or initial-gripper rotation offset.
FR3_ARGS=${FR3_ARGS:-"-p unlimited:=true"}; WRIST_ARGS=${WRIST_ARGS:-'-p box_mode:=\"off\"'}     # quoted: ROS parses a bare off as a boolean
mkdir -p "$OUT/traces"
docker rm -f quest-isaac-quest >/dev/null 2>&1; docker run -d --rm --name quest-isaac-quest --network host --ipc host -e ROS_DOMAIN_ID=$DOM -e RMW_IMPLEMENTATION=rmw_fastrtps_cpp -e FORK_PKG=/pkg/arm_pose_estimator/arm_pose_estimator \
  -v "$FM/quest_isaac/pkg":/pkg:ro -v "$FM/quest_isaac":/qi:ro -v "$FM/quest_isaac/cert":/certs -v "$OUT/traces":/traces zed-lite:5.4.1-humble bash -c \
  "source /opt/ros/humble/setup.bash; cd /pkg/arm_pose_estimator/arm_pose_estimator; \
   python3 /qi/isaac_quest_bridge.py & python3 /qi/trace_recorder.py --dir /traces & \
   python3 quest_wrist_teleop.py --https-port $HTTPS --http-port $HTTP --cert /certs/cert.pem --key /certs/key.pem --output-frame fr3_link0 --no-wrist-pose \
     --ros-args -p video_topic:=/isaac/view/compressed -p lever_topic:=/isaac/lever_q -p robot_view_origin:='[0.25,0.0,-0.60]' $WRIST_ARGS & \
   python3 quest_fr3_teleop.py --ros-args -p wrist_topic:=/wrist_pose -p quest_engage_enabled:=true -p gripper_enabled:=false -p operator_yaw_deg:=$OPERATOR_YAW_DEG -p base_frame:=fr3_link0 $FR3_ARGS & wait" > /dev/null
sleep 12
if [ "${READY:-1}" = 1 ]; then
  echo "[4/4] moving the arm to the READY pose in front of the valve (the g00 pregrasp; ~16 s)"
  docker exec quest-isaac-quest bash -c "source /opt/ros/humble/setup.bash; python3 /qi/ready_pose.py /qi/g00_pregrasp.json 16" 2>&1 | tail -2
fi
docker logs --tail 6 quest-isaac-quest 2>&1 | cut -c1-200
echo
echo "== up. Headset (Quest browser): https://nexus.tail0994ab.ts.net:$HTTPS  (or https://100.111.174.61:$HTTPS), accept the certificate once, ENTER AR, touch RECENTER facing the way you will work."
echo "   Monitor from any tailnet browser: https://nexus.tail0994ab.ts.net:$HTTPS/viewer"
echo "   Engage (without the headset sphere): docker exec quest-isaac-quest bash -c 'source /opt/ros/humble/setup.bash; ros2 service call /teleop/engage std_srvs/srv/Trigger'"
echo "   Put the lever back to 0 deg:         docker exec quest-isaac-quest bash -c 'source /opt/ros/humble/setup.bash; ros2 topic pub --once /isaac/reset_lever std_msgs/msg/Empty {}'"
echo "   Logs: $OUT/{isaac,mpc}.log, docker logs quest-isaac-quest        Stop: ./stop_quest_isaac.sh"
