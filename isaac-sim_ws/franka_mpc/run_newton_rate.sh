#!/usr/bin/env bash
# cuRobo MPC node rate WITHOUT Isaac/ZED/nvblox on the GPU (question: can the real FR3 run the node at 50 Hz? Hipolito's PC = same A2000, measured 25-30 Hz).
# Newton FR3 sim (newton_ros_arm.py, lockstep: one node step per 20 ms sim step) + a saved map served as nvblox's service (esdf_replay.py) + curobo_mpc_node.py
# (real-robot MPC config by default) + wrist_path.py sweep. The node's loop_ms / mpc_step_ms in mpc_node.csv = its cost per control step, alone on the GPU.
#   [ESDF=runs/real_like_5cm/map_check/esdf.npz] [VOXEL=0.05] [CFG=fr3_vcap1.2_acc10.yml] [SECS=18.5] [NODE_ARGS=...] ./run_newton_rate.sh <run_name>
# LOAD (10-29), the lab's camera stack WITHOUT Isaac: LOADCAMS="zed zed2 zed3" = ZED wrappers playing the SVOs recorded from the sim ZEDs (zed_lite/zed_settings/svo,
#   runs/svo_record; zed3 replays zed's file) looped at 30 fps, NEURAL_LIGHT, 640x360 (zed_lite/svo_load.yaml); NVLOAD=1 = nvblox (real 5 cm launch) on zed+zed2
#   [+ SEGLOAD=1 self-filters]. (svo_stream.py + sim-mode wrapper: wrapper SVO playback cannot loop without pos. tracking; SVO = 15 fps HD1080, stabilization 0.) Their map is NOT used: the node keeps the replayed map (served as /esdf_replay/...), so control is identical to the no-load runs.
#   nvidia-smi samples -> gpu.csv. MPC_EXECUTOR via DOCKER_EXTRA="-e MPC_EXECUTOR=events".
set -uo pipefail
cd "$(dirname "$0")"
RUN=${1:?run name}; ESDF=${ESDF:-runs/real_like_5cm/map_check/esdf.npz}; VOXEL=${VOXEL:-0.05}; CFG=${CFG:-fr3_vcap1.2_acc10.yml}; SECS=${SECS:-18.5}
NODE_ARGS=${NODE_ARGS--p extra_collision_sphere_buffer:=0.0 -p grid_size_m:=[1.4,1.6,1.2] -p grid_center_m:=[0.35,0.0,0.55]}
WS=/home/nexus/spot-teleop/isaac-sim_ws; FM=$WS/franka_mpc; OUT=$FM/runs/$RUN; IMG=spot-isaac-sim:6.0.0; DOM=${DOM:-8}
SP=$WS/crawl-lab/IsaacLab/.venv/lib/python3.12/site-packages   # -> ~/newton_sp on the VM (Newton 1.5 / warp 1.16 / mujoco_warp)
J=/isaac-sim/exts/isaacsim.ros2.core/jazzy; NVM=/home/nexus/spot-teleop/zed_lite/nvblox_msgs_jazzy/install
NODE=/home/nexus/spot-teleop/spot-ros2_ws/src/spot_operation_ros2/spot_operation_ros2
ESDF_ABS=$(readlink -f "$ESDF"); [ -f "$ESDF_ABS" ] || { echo "no $ESDF"; exit 1; }
mkdir -p "$OUT"; rm -rf "$OUT/ctl" "$OUT/mpc_node.csv"; mkdir -p "$OUT/ctl"; cp "$ESDF_ABS" "$OUT/esdf.npz"
# GPU SPLIT (10-29, 2x A2000 on VM 103; lab layout: ZED wrappers GPU 0, nvblox GPU 1): G_MPC (cuRobo node), G_WRAP (ZED wrappers), G_NV (nvblox + self-filters),
#   G_SIM (Newton + SVO senders + esdf replay: not on the real robot, so they go to the GPU cuRobo is NOT on). Unset = all GPUs, as before.
gp() { [ -n "${1:-}" ] && echo "device=$1" || echo all; }
GM=$(gp "${G_MPC:-}"); GW=$(gp "${G_WRAP:-}"); GN=$(gp "${G_NV:-}"); GS=$(gp "${G_SIM:-}")
LOADCAMS=${LOADCAMS:-}; NVLOAD=${NVLOAD:-0}; SEGLOAD=${SEGLOAD:-0}; Z=/home/nexus/spot-teleop/zed_lite; SVOREC=$FM/runs/svo_record; SMI=
stop() { [ -n "$SMI" ] && kill $SMI 2>/dev/null; docker stop -t 3 franka-goal franka-mpc esdf-replay >/dev/null 2>&1; docker stop -t 10 newton-sim >/dev/null 2>&1
         for c in $LOADCAMS; do docker stop -t 5 svo-$c svo-send-$c >/dev/null 2>&1; done; docker stop -t 5 nvblox-lite franka-seg franka-seg2 >/dev/null 2>&1; }
trap stop EXIT
wait_for() { local t=0 n=${4:-1}; until [ "$(grep -c "$2" "$1" 2>/dev/null || true)" -ge "$n" ] 2>/dev/null; do sleep 1; t=$((t + 1)); [ $t -ge "$3" ] && { echo "timeout waiting for '$2' in $1"; tail -15 "$1"; return 1; }; done; return 0; }
ROSENV=(-e ROS_DISTRO=jazzy -e RMW_IMPLEMENTATION=rmw_fastrtps_cpp -e ROS_DOMAIN_ID=$DOM -e PYTHONUNBUFFERED=1 -e PYTHONDONTWRITEBYTECODE=1 -e ACCEPT_EULA=Y)
echo "RUN START $(date +%T) $RUN map=$ESDF voxel=$VOXEL cfg=$CFG"
docker run --rm -d --name newton-sim --gpus "$GS" --network host --ipc host "${ROSENV[@]}" -e LD_LIBRARY_PATH=$J/lib -e PYTHONPATH="$SP:$J/rclpy" \
  -v "$SP":"$SP":ro -v /home/nexus/newton_sp:/home/nexus/newton_sp:ro -v "$WS":/workspace:rw -v "$FM/cache_newton_home/newton":/root/.cache/newton -v "$FM/cache_newton_home/warp":/root/.cache/warp \
  --entrypoint /isaac-sim/python.sh $IMG /workspace/franka_mpc/newton_ros_arm.py --out /workspace/franka_mpc/runs/$RUN --seconds $SECS --episodes 1 ${NEWTON_ARGS:-} > /dev/null
( docker logs -f newton-sim > "$OUT/newton.log" 2>&1 & )
wait_for "$OUT/newton.log" "\[newton\] ready" 600 || exit 1; echo "1 newton up"
docker run --rm -d --name esdf-replay --network host --ipc host "${ROSENV[@]}" -e LD_LIBRARY_PATH=$J/lib:/nvm/lib -e PYTHONPATH="/isaac-sim/exts/omni.pip.compute/pip_prebundle:$J/rclpy:/nvm/lib/python3.12/site-packages" \
  -v "$FM":/fm:ro -v "$NVM":/nvm:ro -v "$OUT":/o:rw --entrypoint /isaac-sim/python.sh $IMG /fm/esdf_replay.py --npz /o/esdf.npz --gate-file /o/ctl/esdf_go \
  --service /esdf_replay/get_esdf_and_gradient > /dev/null
( docker logs -f esdf-replay > "$OUT/esdf_replay.log" 2>&1 & )
wait_for "$OUT/esdf_replay.log" "serving" 120 || exit 1; echo "2 esdf replay up"
declare -A LPORT=([zed]=30000 [zed2]=30010 [zed3]=30020)
for c in $LOADCAMS; do                                  # per camera: svo_stream.py (loops the SVO on the sim port) -> a normal sim-mode wrapper (real ZED SDK depth)
  f=rec_$c.svo2; [ "$c" = zed3 ] && f=rec_zed.svo2
  docker run --rm -d --name svo-send-$c --gpus "$GS" --network host --ipc host -v "$Z":/z:ro -v "$Z/zed_resources":/usr/local/zed/resources -v "$Z/zed_settings":/usr/local/zed/settings \
    zed-lite:5.4.1-humble python3 /z/svo_stream.py --svo /usr/local/zed/settings/svo/$f --port ${LPORT[$c]} > /dev/null
  ( docker logs -f svo-send-$c > "$OUT/svo_send_$c.log" 2>&1 & ); sleep 4
  ( GPUS="$GW" ZED_YAML=svo_load.yaml CAM=$c PORT=${LPORT[$c]} NAME=svo-$c ROS_DOMAIN_ID=$DOM "$Z/run_zed.sh" > "$OUT/svo_$c.log" 2>&1 & ); sleep 3
done
for c in $LOADCAMS; do wait_for "$OUT/svo_$c.log" "=== $c started ===" 600 || exit 1; done; [ -n "$LOADCAMS" ] && echo "2b SVO wrappers up: $LOADCAMS"
if [ "$NVLOAD" = 1 ]; then
  GPUS="$GN" CAMS="zed zed2" CAMDIR=$SVOREC NVLAUNCH=franka_zed_nvblox_real.launch.py "$Z/nvblox/run_nvblox.sh" voxel_size:=0.05 self_filter:=$([ "$SEGLOAD" = 1 ] && echo true || echo false) > "$OUT/nvblox.log" 2>&1 &
  wait_for "$OUT/nvblox.log" "static TF published" 180 2 || exit 1; echo "2c nvblox up (load only)"
fi
if [ "$SEGLOAD" = 1 ]; then
  GPUS="$GN" NAME=franka-seg "$Z/nvblox/run_segmenter.sh" --joint-states /joint_states_isaac --threshold 0.15 > "$OUT/seg_zed.log" 2>&1 &
  GPUS="$GN" NAME=franka-seg2 "$Z/nvblox/run_segmenter.sh" --joint-states /joint_states_isaac --threshold 0.15 --depth /zed2/zed_node/depth/depth_registered \
    --info /zed2/zed_node/depth/camera_info --out-depth /franka_seg_zed2/depth --out-mask /franka_seg_zed2/robot_mask > "$OUT/seg_zed2.log" 2>&1 &
  for c in zed zed2; do wait_for "$OUT/seg_$c.log" "depth_out" 300 || exit 1; done; echo "2d self-filters up (load only)"
fi
nvidia-smi --query-gpu=timestamp,index,utilization.gpu,memory.used --format=csv,noheader -lms 500 > "$OUT/gpu.csv" & SMI=$!
GPUS="$GM" OUT=$OUT ./run_mpc_node.sh -p use_esdf:=true -p voxel_size:=$VOXEL -p robot_config:=/workspace/franka_mpc/node_config/$CFG -p step_on_clock:=true -p esdf_service_name:=/esdf_replay/get_esdf_and_gradient $NODE_ARGS > "$OUT/node.log" 2>&1 &
wait_for "$OUT/node.log" "Warm-up done" 300 || exit 1; touch "$OUT/ctl/esdf_go"
wait_for "$OUT/node.log" "First ESDF map received" 60 || exit 1
wait_for "$OUT/node.log" "Initialized goal to current physical pose" 60 || exit 1; echo "3 node warmed up, map in"
./run_goal.sh -p path:=sweep > "$OUT/goal.log" 2>&1 &
wait_for "$OUT/newton.log" "\[newton\] RESULT" 900 || exit 1; echo "4 sweep done"
sleep 2; grep "\[newton\] RESULT" "$OUT/newton.log" | cut -c1-400

echo "RUN FINISHED $(date +%T)"
