#!/usr/bin/env bash
# The Newton version of run_session.sh: NO Isaac, ZED, nvblox or self-filter. A Newton FR3 + drill sim (newton_ros_arm.py) speaks franka_ros_arm.py's ROS interface, the nvblox
# service is replayed from a SAVED map (esdf_replay.py, esdf_full.npz from esdf_capture.py), and the MPC node (curobo_mpc_node.py) and grasp_goal.py run UNCHANGED.
# One Newton process for the whole batch; per grasp the MPC node and the goal source are restarted (as in run_session.sh).
#   GRASPS=runs/b1_grasps ESDF=runs/<map run>/esdf_full.npz [DOM=8] [TAG=] [SECS=14] [JITTERS='0,0,0;0.01,0,2'] [VOXEL=0.02] ./run_newton_session.sh <run_name>
# SIM=isaac: the SAME session with the Isaac sim instead of Newton (franka_ros_arm_session.py, no ZED, no nvblox, no video, physics-only steps between renders: RENDER_EVERY=5),
#   same node / map replay / goal source / grasps, for a fair overhead comparison (DOM is honoured by run_isaac.sh).
# SIMSCRIPT=newton_ros_valve.py: the HiveBoard valve scene instead of the drill (OBJ=lever, ESDF=runs/valve_esdf.npz from valve_esdf.py, GOAL_ARGS='-p clear_box:=true -p turn_deg:=-80.0 -p hinge:=[0.6,0.06,0.554,-1.0,0.0,0.0]').
# DOM / TAG: ROS domain and container-name suffix, so that several sessions can run side by side (e.g. DOM=21 TAG=1).
# Outputs as run_session.sh: runs/<run>/ep<NN>/{RESULT_isaac.json,trace.npz,node.log,mpc_node.csv,goal.log}, runs/<run>.csv/.png
set -uo pipefail
cd "$(dirname "$0")"
RUN=${1:?run name}; GRASPS=${GRASPS:?dir of grasp jsons}; ESDF=${ESDF:?esdf_full.npz}; OBJ=${OBJ:-drill}; SECS=${SECS:-14}; VOXEL=${VOXEL:-0.02}
DOM=${DOM:-8}; TAG=${TAG:-}; NODE_ARGS=${NODE_ARGS--p extra_collision_sphere_buffer:=0.03}; JITTERS=${JITTERS:-}
WS=/home/nexus/spot-teleop/isaac-sim_ws; FM=$WS/franka_mpc; OUT=$FM/runs/$RUN; IMG=spot-isaac-sim:6.0.0
SP=/home/nexus/spot-teleop/isaac-sim_ws/crawl-lab/IsaacLab/.venv/lib/python3.12/site-packages            # host Newton 1.5 / warp 1.16 / mujoco_warp
J=/isaac-sim/exts/isaacsim.ros2.core/jazzy; NVM=/home/nexus/spot-teleop/zed_lite/nvblox_msgs_jazzy/install
NODE=${NODE:-/home/nexus/spot-teleop/spot-ros2_ws/src/spot_operation_ros2/spot_operation_ros2}
GLIST=("$GRASPS"/g*.json); NEP=${#GLIST[@]}; [ "$NEP" -ge 2 ] || { echo "needs >= 2 grasps in $GRASPS"; exit 1; }
ESDF_ABS=$(readlink -f "$ESDF"); [ -f "$ESDF_ABS" ] || { echo "no $ESDF"; exit 1; }
mkdir -p "$OUT"; rm -rf "$OUT/ctl"; mkdir -p "$OUT/ctl"; for ((k = 0; k < NEP; k++)); do mkdir -p "$OUT/ep$(printf %02d $k)/grasps"; done
cp "$ESDF_ABS" "$OUT/esdf_full.npz" 2>/dev/null || true
N_SIM=newton-sim$TAG; N_ESDF=esdf-replay$TAG; N_MPC=franka-mpc$TAG; N_GOAL=franka-goal$TAG
stop() { docker stop -t 3 $N_GOAL $N_MPC $N_ESDF >/dev/null 2>&1; docker stop -t 10 $N_SIM >/dev/null 2>&1; }
trap stop EXIT
wait_for() { local t=0 n=${4:-1}; until [ "$(grep -c "$2" "$1" 2>/dev/null || echo 0)" -ge "$n" ]; do sleep 1; t=$((t + 1)); [ $t -ge "$3" ] && { echo "timeout waiting for '$2' in $1"; tail -15 "$1"; return 1; }; done; return 0; }
wait_file() { local t=0; until [ -e "$1" ]; do sleep 0.5; t=$((t + 1)); docker ps -q -f name=^$N_SIM$ | grep -q . || { [ -e "$1" ] && break; echo "sim died waiting for $1"; tail -20 "$OUT/newton.log"; return 1; }
    [ $t -ge $(( $2 * 2 )) ] && { echo "timeout waiting for $1"; return 1; }; done; return 0; }
ROSENV=(-e ROS_DISTRO=jazzy -e RMW_IMPLEMENTATION=rmw_fastrtps_cpp -e ROS_DOMAIN_ID=$DOM -e PYTHONUNBUFFERED=1 -e PYTHONDONTWRITEBYTECODE=1 -e ACCEPT_EULA=Y)
# 1. the sim: Newton (host Newton site-packages in front of the image's) or, with SIM=isaac, Isaac in its session mode (no cameras)
T_UP0=$(date +%s)
if [ "${SIM:-newton}" = isaac ]; then
  ( DOM=$DOM IMAGE=$IMG NAME=$N_SIM ROS=jazzy ./run_isaac.sh -s franka_ros_arm_session.py "$RUN" --seconds $SECS --episodes $NEP --ctl-dir /workspace/franka_mpc/runs/$RUN/ctl \
      --no-video --track-object $OBJ --render-every ${RENDER_EVERY:-5} --no-pace --clock-every 4 ${ISAAC_ARGS:-} > "$OUT/newton.log" 2>&1 & )
  wait_for "$OUT/newton.log" "waiting for /wrist_pose" 900 || exit 1; echo "1 isaac sim up ($(( $(date +%s) - T_UP0 )) s)"
else
docker run --rm -d --name $N_SIM --gpus all --network host --ipc host ${NEWTON_PROF:+-e NEWTON_PROF=$NEWTON_PROF} "${ROSENV[@]}" -e LD_LIBRARY_PATH=$J/lib -e PYTHONPATH="$SP:$J/rclpy" \
  -v "$SP":"$SP":ro -v "$WS":/workspace:rw -v "$FM/cache_newton_home/newton":/root/.cache/newton -v "$FM/cache_newton_home/warp":/root/.cache/warp \
  --entrypoint /isaac-sim/python.sh $IMG /workspace/franka_mpc/${SIMSCRIPT:-newton_ros_arm.py} --out /workspace/franka_mpc/runs/$RUN --seconds $SECS --episodes $NEP \
  --ctl-dir /workspace/franka_mpc/runs/$RUN/ctl ${JITTERS:+--jitters "$JITTERS"} ${NEWTON_ARGS:-} > /dev/null
( docker logs -f $N_SIM > "$OUT/newton.log" 2>&1 & )
wait_for "$OUT/newton.log" "\[newton\] ready" 600 || exit 1; echo "1 newton sim up ($(( $(date +%s) - T_UP0 )) s)"
fi
# 2. the saved map as nvblox's service
docker run --rm -d --name $N_ESDF --network host --ipc host "${ROSENV[@]}" -e LD_LIBRARY_PATH=$J/lib:/nvm/lib -e PYTHONPATH="/isaac-sim/exts/omni.pip.compute/pip_prebundle:$J/rclpy:/nvm/lib/python3.12/site-packages" \
  -v "$FM":/fm:ro -v "$NVM":/nvm:ro -v "$(dirname "$ESDF_ABS")":/map:ro -v "$OUT/ctl":/ctl:rw --entrypoint /isaac-sim/python.sh $IMG /fm/esdf_replay.py --npz /map/$(basename "$ESDF_ABS") \
  --gate-file /ctl/esdf_go > /dev/null
( docker logs -f $N_ESDF > "$OUT/esdf_replay.log" 2>&1 & )
wait_for "$OUT/esdf_replay.log" "serving" 120 || exit 1; echo "2 esdf replay up"
ROSARGS=(-p robot:=franka -p base_frame:=fr3_link0 -p robot_config:=/workspace/franka_mpc/node_config/fr3.yml -p use_sim:=true -p use_sim_time:=true -p use_esdf:=${USE_ESDF:-true} -p esdf_frame_id:=fr3_link0
         -p esdf_global_frame:=fr3_link0 -p voxel_size:=$VOXEL "-p" "grid_size_m:=[1.6,1.6,1.2]" -p "grid_center_m:=[0.45,0.0,0.45]" ${NODE_ARGS:-} -p step_on_clock:=true)
PYM=/isaac-sim/extscache/omni.warp.core-1.13.0+lx64:/isaac-sim/extsDeprecated/omni.isaac.ml_archive/pip_prebundle:/isaac-sim/exts/omni.pip.compute/pip_prebundle:/workspace/curobo-isaac6/src
PYM=$PYM:/workspace/curobo-isaac6/.venv/lib/python3.12/site-packages:/workspace/franka_mpc/pydeps:$J/rclpy:/nvm/lib/python3.12/site-packages
node_start() {  # ep dir
  docker run --rm -d --name $N_MPC --gpus all --network host --ipc host ${PY_SWITCH:+-e PY_SWITCH=$PY_SWITCH} ${PROF:+--cap-add SYS_PTRACE -v "$FM/cache_newton_home/py-spy":/py-spy:ro} "${ROSENV[@]}" -e LD_LIBRARY_PATH=$J/lib:/nvm/lib -e PYTHONPATH="$PYM" -e SETUPTOOLS_SCM_PRETEND_VERSION=0.7.7.post1.dev5 \
    -v "$WS":/workspace:ro -v "$NODE":/node:ro -v "$NVM":/nvm:ro -v "$1":/out:rw -v "$WS/cache/computecache":/root/.nv/ComputeCache --entrypoint bash $IMG -c \
    "/isaac-sim/python.sh /node/curobo_mpc_node.py --ros-args ${ROSARGS[*]} -p log_csv_path:=/out/mpc_node.csv; rc=\$?; chown -R 1000:1000 /out; exit \$rc" > /dev/null
  ( docker logs -f $N_MPC > "$1/node.log" 2>&1 & ); }
goal_start() {  # ep dir name
  docker run --rm -d --name $N_GOAL --network host --ipc host "${ROSENV[@]}" -e LD_LIBRARY_PATH=$J/lib -e PYTHONPATH="/isaac-sim/exts/omni.pip.compute/pip_prebundle:$J/rclpy" \
    -v "$FM":/fm:ro --entrypoint /isaac-sim/python.sh $IMG /fm/grasp_goal.py --ros-args -p use_sim_time:=true -p grasp_json:=/fm/runs/$RUN/$2/grasps/${OBJ}_selected.json ${GOAL_ARGS:-} > /dev/null
  ( docker logs -f $N_GOAL > "$1/goal.log" 2>&1 & ); }
T_SESSION=$(date +%s); k=0
for g in "${GLIST[@]}"; do
  kk=$(printf %02d $k); EP=$OUT/ep$kk; cp "$g" "$EP/grasps/${OBJ}_selected.json"; t0=$(date +%s)
  rm -f "$OUT/ctl/esdf_go"; node_start "$EP"                                   # the map is withheld until the warm-up is done (as in the Isaac runs)
  wait_for "$EP/node.log" "Warm-up done" 300 || exit 1; touch "$OUT/ctl/esdf_go"
  [ "${USE_ESDF:-true}" = true ] && { wait_for "$EP/node.log" "First ESDF map received" 60 || exit 1; }
  wait_for "$EP/node.log" "Initialized goal to current physical pose" 60 || exit 1
  goal_start "$EP" ep$kk
  echo "ep$kk: node up in $(( $(date +%s) - t0 )) s, goal started: $(basename $g)"
  # PROF=1: py-spy on the MPC node (all threads) for 20 s once the episode is under way -> ep00/prof.raw (+ prof.log); analyse with prof_summary.py
  [ -n "${PROF:-}" ] && [ $k = 0 ] && ( sleep ${PROF_DELAY:-45}; docker exec $N_MPC bash -c "/py-spy record -p \$(pgrep -f 'kit/python/bin/python3 /node/curobo_mpc_node.py' | head -1) -d 20 -r 100 -t -f raw -o /out/prof.raw" > "$EP/prof.log" 2>&1 & )
  if [ $k -lt $((NEP - 1)) ]; then
    wait_file "$OUT/ctl/done_$k" 600 || exit 1
    docker stop -t 3 $N_GOAL $N_MPC >/dev/null 2>&1; touch "$OUT/ctl/reset_$k"
    wait_file "$OUT/ctl/ready_$((k + 1))" 120 || exit 1
  else
    if [ "${SIM:-newton}" = isaac ]; then t=0; while docker ps -q -f name=^$N_SIM$ | grep -q .; do sleep 1; t=$((t + 1)); [ $t -ge 600 ] && break; done
    else wait_for "$OUT/newton.log" "\[newton\] RESULT" 600 $NEP || exit 1; fi
    sleep 2; docker stop -t 3 $N_GOAL $N_MPC >/dev/null 2>&1
  fi
  echo "${RUN}_ep$kk: $(( $(date +%s) - t0 )) s | drill moved $(python3 -c "import json;print(json.load(open('$EP/RESULT_isaac.json'))['object_centre_moved_mm']['$OBJ'])" 2>/dev/null) mm"
  k=$((k + 1))
done
echo "session: $NEP episodes in $(( $(date +%s) - T_SESSION )) s after the sim was up"
/usr/bin/python3 batch_summary.py "$OUT"/ep* --csv "runs/$RUN.csv" --plot "runs/$RUN.png" || true
