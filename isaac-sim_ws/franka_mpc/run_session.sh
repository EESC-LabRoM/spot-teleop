#!/usr/bin/env bash
# SESSION version of run_grasp.sh (10-17): ONE Isaac + ZED + nvblox + self-filter stack for a whole batch of grasps. Per grasp json in $GRASPS (grasp_variants.py):
# the MPC node and the goal source are started, the episode is recorded by franka_ros_arm_session.py (--episodes N), then both are stopped, Isaac puts the arm
# and every object back to its start pose, and the next grasp runs. The nvblox map PERSISTS across episodes (the human node keeps the target out of the static
# map, so the map stays valid when the target is put back; the old per-trial runs rebuilt it every time).  Outputs: runs/<run>/ep<NN>/ (RESULT_isaac.json,
# trace.npz, objects.json, node.log, goal.log, map_pre/[map_post]), runs/<run>/SESSION.json (+ reset errors), runs/<run>.csv/.png (batch_summary.py).
#   GRASPS=runs/b1_grasps [REAL=1] [FULL=1] [LIGHT=1] [MAPSECS=30] [SETTLE_EP=5] [MAPPOST=0|1] ./run_session.sh <run_name>
# Handshake with Isaac (runs/<run>/ctl): done_<k> (episode k recorded) -> this script stops goal+node -> reset_<k> -> Isaac resets -> ready_<k+1> -> next episode.
# Everything else (env, REAL=1 preset, stack bring-up) is as in run_grasp.sh, whose header follows.
# Step B of the grasp integration: capture -> GraspGen-X -> filters -> the MPC node executes pregrasp, grasp, close, lift on the sim FR3.
# Order: Isaac (franka_ros_arm.py: scene, both ZED stands' segmentation + depth -> clouds/, then waits for /wrist_pose) -> [FULL=1: ZED wrappers
# -> nvblox -> robot self-filter] -> grasp_generate.py + grasp_filter.py on the host GPU -> MPC node -> grasp_goal.py (pose, gripper, clear box).
#   [FULL=0|1] [OBJ=drill] [REUSE=runs/<earlier run>] [VIA='[x,y,z,...]' flat hand-flange waypoints before the pregrasp] [VIEW="ex ey ez tx ty tz focal"] [SECS=17] ./run_grasp.sh [run_name] [franka_ros_arm.py args, e.g. --no-video]
#   MASKMODE=human (default since 10-15, = the paper's release open-vocab-shared-autonomy): mask -> nvblox human_with_static_tsdf (target out of the
#   STATIC map) + the clear box measured from the mask and depth like sam2_tracker_node (MASKBOX=1, /target_object/clear_box, the node pads 3 cm):
#   nvblox keeps the masked target as a dynamic obstacle in the ESDF it serves, the clear box carves it. MASKBOX=0: mask only (the MPC stalls).
#   MASKMODE=filter: the mask is applied to the robot-free depth BEFORE nvblox (target_mask.py --mode filter): the target is in no layer.
#   MASKMODE=human: the mask goes into nvblox's human_with_static_tsdf (static map without the target, but nvblox keeps the masked target as a DYNAMIC
#   obstacle in the ESDF it serves, measured 10-15: the MPC then stalls above the target) -- see HANDOFF.
#   MASK=1 (default with FULL=1): the target's mask keeps it out of the map: target_mask.py (exact mesh at the live pose, rasterized per ZED
#   depth frame) -> nvblox_node in human_with_static_tsdf mode (masked pixels -> dynamic layer), as in the paper; NO clear box in the MPC. MASK=0:
#   the object stays in the map (nothing clears it). CLEARBOX=1: the old coarse AABB via /target_object/clear_box. MAPDUMP=1: ESDF of the
#   grasp region before the goal starts and after the run -> map_pre/ map_post/ (esdf.npz, base frame).
#   FULL=0: no map (free-space MPC): checks the sequence, the gripper and the grasp physics.
#   FULL=1: the whole stack of run_obstacle.sh (2 ZEDs, NEURAL_PLUS depth, nvblox 1 cm ESDF in the node, 3 cm margin) with the target's clear box.
#   outputs in runs/<run_name>: clouds/, grasps/, trace.npz (q_fingers, obj = the grasped object's pose), mpc_node.csv, logs, video.mp4
set -uo pipefail
cd "$(dirname "$0")"
Z=/home/nexus/spot-teleop/zed_lite
RUN=${1:-grasp}; shift || true; OBJ=${OBJ:-drill}; FULL=${FULL:-0}; MASK=${MASK:-1}; MASKMODE=${MASKMODE:-human}; MASKBOX=${MASKBOX:-1}; CLEARBOX=${CLEARBOX:-0}; MAPDUMP=${MAPDUMP:-0}; SECS=${SECS:-14}; CAMS=${CAMS:-zed zed2}; SEG_ARGS=${SEG_ARGS:---threshold 0.15}
# REAL=1 (10-16): the lab's setup instead of the sim's: the two ZED X left lenses at the EasyHeC-calibrated real camera poses (real_cams.json: sim zed = lab zed1,
# sim zed2 = lab zed2; franka_ros_arm.py --real-cams, no sim stands), the real depth settings (zed_lite/sim_real.yaml: HD720 grab -> 640x360, NEURAL_LIGHT,
# stabilization 30) and the real 2 cm voxels. (The Isaac ZED X has no HD720: it streams HD1080 and the wrapper downscales by 3 = the same 640x360.) Override any of them with ZED_YAML= / VOXEL=. REAL=0 (default) = the sim's own setup, unchanged.
REALARG=(); if [ "${REAL:-0}" = 1 ]; then ZED_YAML=${ZED_YAML:-sim_real.yaml}; VOXEL=${VOXEL:-0.02}; REALARG=(--real-cams /workspace/franka_mpc/real_cams.json); fi
export ZED_YAML=${ZED_YAML:-sim_quality.yaml}
NODE_ARGS=${NODE_ARGS--p extra_collision_sphere_buffer:=0.03}; VOXEL=${VOXEL:-0.01}
NV=nvblox-lite:3.2.5-humble
GRASPS=${GRASPS:?GRASPS=dir of grasp jsons}; GLIST=("$GRASPS"/g*.json); NEP=${#GLIST[@]}; [ "$NEP" -ge 2 ] || { echo "session mode needs >= 2 grasps in $GRASPS"; exit 1; }
MAPPOST=${MAPPOST:-0}; SETTLE_EP=${SETTLE_EP:-5}
OUT=$PWD/runs/$RUN; mkdir -p "$OUT"; rm -f "$OUT"/mpc_node.csv; rm -rf "$OUT/ctl"; mkdir -p "$OUT/ctl"
for ((k = 0; k < NEP; k++)); do mkdir -p "$OUT/ep$(printf %02d $k)/grasps"; done   # made here, by us: Isaac (root) would create them root-owned and this shell could not write its logs there
printf 'REAL=%s ZED_YAML=%s VOXEL=%s FULL=%s LIGHT=%s MASKMODE=%s CAMS="%s" SEG_ARGS=%s NODE_ARGS=%s GRASP_JSON=%s\n' "${REAL:-0}" "$ZED_YAML" "$VOXEL" "$FULL" "${LIGHT:-0}" "$MASKMODE" "$CAMS" "$SEG_ARGS" "$NODE_ARGS" "${GRASP_JSON:-}" > "$OUT/config.txt"
IMG=spot-isaac-sim:6.0.0; VIEWARG=(); [ -n "${VIEW:-}" ] && VIEWARG=(--view $VIEW)
# LIGHT=1 (batches): no video (VIDEO=1 keeps it), render only every N control steps (physics-only steps between, franka_ros_arm.py
# --render-every; the MPC node runs its loop from /clock, step_on_clock): FULL=0 renders every 5th step and does not hold the sim to wall time; FULL=1 renders every 3rd (ZEDs at 15 fps, paced)
LIGHTARG=(); LIGHTNODE=; if [ "${LIGHT:-0}" = 1 ]; then [ "${VIDEO:-0}" = 1 ] || LIGHTARG+=(--no-video); LIGHTNODE="-p step_on_clock:=true"
  if [ "${FULL:-0}" = 1 ]; then LIGHTARG+=(--render-every "${RENDER_EVERY:-3}" --zed-fps "${ZED_FPS:-15}"); else LIGHTARG+=(--render-every "${RENDER_EVERY:-5}" --no-pace --clock-every "${CLOCK_EVERY:-4}"); fi; fi
declare -A PORT=([zed]=30000 [zed2]=30010)
ZEDARG=; ZEDFLAG=()
if [ "$FULL" = 1 ]; then IMG=spot-isaac-sim:6.0.0-zed
  ZEDARG=$(for c in $CAMS; do printf "%s:%s," "$c" "${PORT[$c]}"; done); ZEDARG=${ZEDARG%,}; ZEDFLAG=(--zeds "$ZEDARG"); fi
stop() { docker stop -t 30 mesh-main mesh-b mesh-c >/dev/null 2>&1
         docker run --rm -v "$OUT":/out "$NV" chown -R 1000:1000 /out >/dev/null 2>&1
         docker stop target-mask franka-goal franka-mpc franka-seg franka-seg2 nvblox-lite zed-lite zed-lite2 >/dev/null 2>&1
         docker stop franka-isaac >/dev/null 2>&1; }
trap stop EXIT
mapdump() {  # service tag: the grasp region of the nvblox map (base frame) -> $OUT/map_<tag>
  docker exec nvblox-lite bash -c "source /opt/ros/humble/setup.bash && mkdir -p /ws/nvblox/out/$RUN/$2 && python3 /ws/nvblox/map_check.py \
    --workcell /workcell --out /ws/nvblox/out/$RUN/$2 --service $1 --aabb-min 0.20 -0.50 -0.04 --aabb-size 0.50 0.60 0.45 \
    && chown -R 1000:1000 /ws/nvblox/out/$RUN" >> "$OUT/map_check.log" 2>&1 && cp -r "$Z/nvblox/out/$RUN/$2" "$OUT/map_$2"; }
aux() {  # name cmd...: a helper container from the nvblox image (ROS env, nvblox_msgs), outputs in $OUT
  local n=$1; shift
  docker run --rm -d --name "$n" --network host --ipc host -e ROS_DOMAIN_ID=8 -v "$Z/nvblox":/ws/nvblox:ro -v "$OUT":/out "$NV" \
    bash -c "source /opt/ros/humble/setup.bash && exec $*" >/dev/null; }
wait_for() {  # file pattern timeout_s [count]
  local t=0 n=${4:-1}; until [ "$(grep -c "$2" "$1" 2>/dev/null)" -ge "$n" ]; do sleep 1; t=$((t + 1))
    [ $t -ge "$3" ] && { echo "timeout waiting for '$2' in $1"; tail -15 "$1"; return 1; }; done; return 0; }
IMAGE=$IMG NAME=franka-isaac ROS=jazzy ./run_isaac.sh -s franka_ros_arm_session.py "$RUN" --seconds "$SECS" --episodes "$NEP" --ctl-dir /workspace/franka_mpc/runs/$RUN/ctl --no-video --capture-cams zed,zed2 \
  --track-object "$OBJ" "${ZEDFLAG[@]}" "${VIEWARG[@]}" "${REALARG[@]}" "${LIGHTARG[@]}" "$@" > "$OUT/isaac.log" 2>&1 &
ISAAC=$!
wait_for "$OUT/isaac.log" "waiting for /wrist_pose" 900 || exit 1; echo "1 isaac up, clouds captured"
if [ "$FULL" = 1 ]; then
  for c in $CAMS; do n=zed-lite; [ "$c" = zed ] || n=zed-lite${c#zed}
    CAM=$c PORT=${PORT[$c]} NAME=$n "$Z/run_zed.sh" > "$OUT/zed_$c.log" 2>&1 &
  done
  for c in $CAMS; do n=zed-lite; [ "$c" = zed ] || n=zed-lite${c#zed}
    wait_for "$OUT/zed_$c.log" "=== $c started ===" "${ZED_WAIT:-240}" || { echo "restarting $n"; docker stop "$n" >/dev/null
      CAM=$c PORT=${PORT[$c]} NAME=$n "$Z/run_zed.sh" >> "$OUT/zed_$c.log" 2>&1 &
      wait_for "$OUT/zed_$c.log" "=== $c started ===" 600 || exit 1; }
  done; echo "2 ZED wrappers up"
  if [ "$MASK" = 1 ]; then RUN=$RUN CAMS="$CAMS" MODE="$MASKMODE" CLEARBOX_FROM_MASK=$([ "$MASKMODE" = human ] && [ "$MASKBOX" = 1 ] && echo 1) EXTRA="${VARIANTS:+${EXTRA_B-${DILATE_B:-12}}}" SNAP="${SNAP:-0}" DILATE="${DILATE:-3}" OCCL="${OCCL:-}" ./run_target_mask.sh > "$OUT/target_mask.log" 2>&1 &
    wait_for "$OUT/target_mask.log" "target mask for" 120 || exit 1; echo "2b target mask node up"; fi
  CAMS="$CAMS" CAMDIR=$OUT "$Z/nvblox/run_nvblox.sh" voxel_size:=$VOXEL mask:=$([ "$MASK" = 1 ] && [ "$MASKMODE" = human ] && echo true || echo false) \
    filter_target:=$([ "$MASK" = 1 ] && [ "$MASKMODE" = filter ] && echo true || echo false) ${NVARGS:-} > "$OUT/nvblox.log" 2>&1 &
  wait_for "$OUT/nvblox.log" "static TF published" 180 "$(echo $CAMS | wc -w)" || exit 1; echo "3 nvblox up (voxel $VOXEL)"
  if [ "${VARIANTS:-0}" = 1 ]; then   # map comparison: nvblox_node = mask dilated DILATE px, _b = DILATE_B px, _c = no mask at all (the target stays in the map)
    docker exec -d nvblox-lite bash -c "source /opt/ros/humble/setup.bash && ros2 launch /ws/nvblox/franka_zed_nvblox.launch.py cameras:=$(echo $CAMS | tr ' ' ,) \
      voxel_size:=$VOXEL node_name:=nvblox_node_b mask:=true mask_suffix:=${MASK_SUFFIX_B:-_d${DILATE_B:-12}} > /ws/nvblox/out/nvblox_b_$RUN.log 2>&1"
    docker exec -d nvblox-lite bash -c "source /opt/ros/humble/setup.bash && ros2 launch /ws/nvblox/franka_zed_nvblox.launch.py cameras:=$(echo $CAMS | tr ' ' ,) \
      voxel_size:=$VOXEL node_name:=nvblox_node_c mask:=false > /ws/nvblox/out/nvblox_c_$RUN.log 2>&1"
    echo "3b variants: nvblox_node_b (mask +${DILATE_B:-12} px) and nvblox_node_c (no mask)"
  fi
  if [ "${MESH:-1}" = 1 ]; then aux mesh-main python3 /ws/nvblox/mesh_recorder.py --topic /nvblox_node/mesh --out /out/mesh_main.npz
    if [ "${VARIANTS:-0}" = 1 ]; then aux mesh-b python3 /ws/nvblox/mesh_recorder.py --topic /nvblox_node_b/mesh --out /out/mesh_b.npz
      aux mesh-c python3 /ws/nvblox/mesh_recorder.py --topic /nvblox_node_c/mesh --out /out/mesh_c.npz; fi; echo "3c recording the mesh topic(s)"; fi
  for c in $CAMS; do
    if [ "$c" = zed ]; then NAME=franka-seg "$Z/nvblox/run_segmenter.sh" --joint-states /joint_states_isaac $SEG_ARGS > "$OUT/seg_$c.log" 2>&1 &
    else NAME=franka-seg${c#zed} "$Z/nvblox/run_segmenter.sh" --joint-states /joint_states_isaac $SEG_ARGS --depth /$c/zed_node/depth/depth_registered \
           --info /$c/zed_node/depth/camera_info --out-depth /franka_seg_$c/depth --out-mask /franka_seg_$c/robot_mask > "$OUT/seg_$c.log" 2>&1 &
    fi
  done
  for c in $CAMS; do wait_for "$OUT/seg_$c.log" "depth_out" 300 || exit 1; done; echo "4 self-filters up"
fi
wait_file() {  # file timeout_s
  local t=0; until [ -e "$1" ]; do sleep 1; t=$((t + 1)); kill -0 $ISAAC 2>/dev/null || { [ -e "$1" ] && break; echo "isaac died while waiting for $1"; return 1; }
    [ $t -ge "$2" ] && { echo "timeout waiting for $1"; return 1; }; done; return 0; }
MAPRESET=${MAPRESET:-1}   # 1: every episode starts from the SAME static map: saved after the first settle, re-loaded (nvblox save_map / load_map) after each reset.
                          # 0: the map persists and keeps integrating across episodes (10-17 measured: the unobserved voxels fill in, +350 occupied voxels, worse approaches)
nvsvc() {  # save_map|load_map : the pristine map file lives in the nvblox container's out dir
  docker exec nvblox-lite bash -c "source /opt/ros/humble/setup.bash && mkdir -p /ws/nvblox/out/$RUN && ros2 service call /nvblox_node/$1 nvblox_msgs/srv/FilePath '{file_path: /ws/nvblox/out/$RUN/pristine}'" 2>&1 | grep -oE "success=(True|False)" | head -1; }
NODEARGS=(${NODE_ARGS:-} $LIGHTNODE); [ "$FULL" = 1 ] && NODEARGS=(-p use_esdf:=true -p voxel_size:=$VOXEL -p "grid_size_m:=[1.6,1.6,1.2]" -p "grid_center_m:=[0.45,0.0,0.45]" "${NODEARGS[@]}")
T_SESSION=$(date +%s); k=0
for g in "${GLIST[@]}"; do
  kk=$(printf %02d $k); EP=$OUT/ep$kk; mkdir -p "$EP/grasps"; cp "$g" "$EP/grasps/${OBJ}_selected.json"; t0=$(date +%s)
  if [ "$MAPRESET" = 1 ] && [ $k -ge 1 ]; then echo "ep$kk: load_map -> $(nvsvc load_map)"; sleep 2; fi
  OUT=$EP ./run_mpc_node.sh "${NODEARGS[@]}" > "$EP/node.log" 2>&1 &
  wait_for "$EP/node.log" "Initialized goal to current physical pose" 300 || exit 1
  if [ "$FULL" = 1 ]; then wait_for "$EP/node.log" "ESDF voxel grid updated" 900 5 || exit 1; fi
  if [ $k = 0 ]; then sleep "${MAPSECS:-0}"; else sleep "$SETTLE_EP"; fi
  if [ "$FULL" = 1 ]; then mapdump /nvblox_node/get_esdf_and_gradient pre_$kk && mv "$OUT/map_pre_$kk" "$EP/map_pre"; fi
  if [ "$MAPRESET" = 1 ] && [ $k = 0 ]; then echo "ep00: save_map -> $(nvsvc save_map)"; fi
  echo "ep$kk: node up, map dumped ($(( $(date +%s) - t0 )) s), starting the goal: $g"
  SCRIPT=grasp_goal.py ./run_goal.sh -p grasp_json:=/fm/runs/$RUN/ep$kk/grasps/${OBJ}_selected.json ${VIA:+-p via:=$VIA} $([ "$CLEARBOX" = 1 ] && echo "-p clear_box:=true") ${GOAL_ARGS:-} > "$EP/goal.log" 2>&1 &
  if [ $k -lt $((NEP - 1)) ]; then
    wait_file "$OUT/ctl/done_$k" 900 || exit 1
    [ "$MAPPOST" = 1 ] && [ "$FULL" = 1 ] && { mapdump /nvblox_node/get_esdf_and_gradient post_$kk && mv "$OUT/map_post_$kk" "$EP/map_post"; }
    docker stop -t 5 franka-goal franka-mpc >/dev/null 2>&1; touch "$OUT/ctl/reset_$k"
    wait_file "$OUT/ctl/ready_$((k + 1))" 120 || exit 1
  else
    wait $ISAAC; echo "isaac done (exit $?)"
    [ "$MAPPOST" = 1 ] && [ "$FULL" = 1 ] && { mapdump /nvblox_node/get_esdf_and_gradient post_$kk && mv "$OUT/map_post_$kk" "$EP/map_post"; }
    docker stop -t 5 franka-goal franka-mpc >/dev/null 2>&1
  fi
  echo "${RUN}_ep$kk: $(( $(date +%s) - t0 )) s | drill moved $(python3 -c "import json;print(json.load(open('$EP/RESULT_isaac.json'))['object_centre_moved_mm']['$OBJ'])" 2>/dev/null) mm"
  k=$((k + 1))
done
echo "session: $NEP episodes in $(( $(date +%s) - T_SESSION )) s after the stack was up"
/usr/bin/python3 batch_summary.py "$OUT"/ep* --csv "runs/$RUN.csv" --plot "runs/$RUN.png" || true
