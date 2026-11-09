#!/usr/bin/env bash
# Newton FR3 + the MPC node with NO goal stream (nothing publishes /wrist_pose): does the node's commanded pose drift?  NODEDIR=<dir with curobo_mpc_node.py> SECS=60 ./run_newton_nogoal.sh <run>
set -uo pipefail
cd "$(dirname "$0")"
RUN=${1:?run name}; SECS=${SECS:-60}; WS=/home/nexus/spot-teleop/isaac-sim_ws; FM=$WS/franka_mpc; OUT=$FM/runs/$RUN; IMG=spot-isaac-sim:6.0.0
SP=$WS/crawl-lab/IsaacLab/.venv/lib/python3.12/site-packages; J=/isaac-sim/exts/isaacsim.ros2.core/jazzy
mkdir -p "$OUT"; rm -f "$OUT/mpc_node.csv" "$OUT"/node.log "$OUT"/newton.log
stop() { docker stop -t 3 franka-mpc esdf-replay >/dev/null 2>&1; docker stop -t 10 newton-sim >/dev/null 2>&1; }; trap stop EXIT
wait_for() { local t=0; until grep -a -q "$2" "$1" 2>/dev/null; do sleep 1; t=$((t + 1)); [ $t -ge "$3" ] && { echo "timeout waiting for '$2'"; tail -8 "$1"; return 1; }; done; return 0; }
ROSENV=(-e ROS_DISTRO=jazzy -e RMW_IMPLEMENTATION=rmw_fastrtps_cpp -e ROS_DOMAIN_ID=8 -e PYTHONUNBUFFERED=1 -e PYTHONDONTWRITEBYTECODE=1 -e ACCEPT_EULA=Y)
docker run --rm -d --name newton-sim --gpus all --network host --ipc host "${ROSENV[@]}" -e LD_LIBRARY_PATH=$J/lib -e PYTHONPATH="$SP:$J/rclpy" \
  -v "$SP":"$SP":ro -v /home/nexus/newton_sp:/home/nexus/newton_sp:ro -v "$WS":/workspace:rw -v "$FM/cache_newton_home/newton":/root/.cache/newton -v "$FM/cache_newton_home/warp":/root/.cache/warp \
  --entrypoint /isaac-sim/python.sh $IMG /workspace/franka_mpc/${SIMSCRIPT:-newton_ros_arm.py} --out /workspace/franka_mpc/runs/$RUN --seconds 3000 --wait 900 --episodes 1 >/dev/null
( docker logs -f newton-sim > "$OUT/newton.log" 2>&1 & )
wait_for "$OUT/newton.log" "\[newton\] ready" 300 || exit 1
ESDFARGS=""
if [ "${USE_ESDF:-false}" = true ]; then
  mkdir -p "$OUT/ctl"; rm -f "$OUT/ctl/esdf_go"; NVM=/home/nexus/spot-teleop/zed_lite/nvblox_msgs_jazzy/install; MAP=$FM/runs/maponly_hd1080/map_check
  docker run --rm -d --name esdf-replay --network host --ipc host "${ROSENV[@]}" -e LD_LIBRARY_PATH=$J/lib:/nvm/lib -e PYTHONPATH="/isaac-sim/exts/omni.pip.compute/pip_prebundle:$J/rclpy:/nvm/lib/python3.12/site-packages" \
    -v "$FM":/fm:ro -v "$NVM":/nvm:ro -v "$MAP":/map:ro -v "$OUT/ctl":/ctl:rw --entrypoint /isaac-sim/python.sh $IMG /fm/esdf_replay.py --npz /map/esdf.npz --gate-file /ctl/esdf_go --service /esdf_replay/get_esdf_and_gradient >/dev/null
  ESDFARGS="-p use_esdf:=true -p voxel_size:=0.05 -p grid_size_m:=[1.4,1.6,1.2] -p grid_center_m:=[0.35,0.0,0.55] -p esdf_service_name:=/esdf_replay/get_esdf_and_gradient"
fi
OUT=$OUT NODE=${NODEDIR:-} ./run_mpc_node.sh -p use_esdf:=${USE_ESDF:-false} ${ESDFARGS} -p robot_config:=/workspace/franka_mpc/node_config/fr3_vcap1.2_acc10.yml -p step_on_clock:=true ${NODE_EXTRA:-} > "$OUT/node.log" 2>&1 &
wait_for "$OUT/node.log" "Initialized goal to current physical pose" 240 || exit 1
[ "${USE_ESDF:-false}" = true ] && touch "$OUT/ctl/esdf_go"
echo "node initialised its own goal; no goal stream; running ${SECS}s"; sleep "$SECS"
echo "done; /wrist_pose messages ever received by the sim: $(grep -a -c 'first /wrist_pose' "$OUT/newton.log")"
