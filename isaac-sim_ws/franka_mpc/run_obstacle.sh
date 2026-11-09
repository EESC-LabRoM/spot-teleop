#!/usr/bin/env bash
# Checkpoint 2 of the MPC-node port: obstacle avoidance on the nvblox map. curobo_mpc_node.py (robot:=franka, use_esdf)
# drives the sim Franka while the fingertip reference sweeps through the blue bottle (wrist_path.py path:=sweep).
# Order: Isaac (franka_ros_arm.py + ZED streams) -> ZED wrappers -> nvblox + static TF per camera -> robot self-filter per
# camera -> MPC node (ESDF from nvblox at 1 Hz) -> wait for the map -> sweep -> Isaac stops after --seconds of sim time.
#   [CAMS="zed zed2"] [SEG_ARGS="--threshold 0.15"] [NODE_ARGS="-p extra_collision_sphere_buffer:=0.03"] [GOAL_ARGS="-p sweep_tip_z:=0.28"] ./run_obstacle.sh [run_name] [franka_ros_arm.py args, e.g. --no-video]
#   SEG_ARGS: robot self-filter margin; 0.15 m = Isaac Manipulator's cuMotion bringup default (0.05 m left stereo edge
#   pixels around the arm in the map, which blocked the arm: runs/obstacle_2zed_thr005)
#   Defaults since 10-14 (the bottle's cap was missing from the map before): ZED_YAML=sim_quality.yaml (NEURAL_PLUS depth,
#   full resolution; ZED_WAIT=1200 if the SDK must optimise the model again), VOXEL=0.01 (nvblox + node), NODE_ARGS = 3 cm
#   margin. Older runs used sim_static.yaml + VOXEL=0.02 + no margin. EXTRA_VOXEL=0.01 (a second nvblox instance, nvblox_node_extra, same inputs, map
#   dumped + meshed too), MESH=1 (record the nvblox mesh topic(s): mesh_main.npz / mesh_extra.npz), MAP_ONLY=1 MAP_SECS=150
#   (no node, no sweep: map for MAP_SECS wall seconds, dump one depth frame per camera and the maps, stop)
#   outputs in runs/<run_name>: trace.npz, mpc_node.csv, objects*.{npz,json}, camera_*.json, logs, video.mp4
set -uo pipefail
cd "$(dirname "$0")"
Z=/home/nexus/spot-teleop/zed_lite
RUN=${1:-obstacle}; shift || true; CAMS=${CAMS:-zed zed2}; SEG_ARGS=${SEG_ARGS:---threshold 0.15}
# REAL=1 (added 10-27 for the avoidance repro): the lab's setup instead of the sim's: the ZED X lenses at the 10-20 EasyHeC-calibrated REAL poses (real_cams.json, no sim stands),
# the real depth settings (zed_lite/sim_real.yaml: 640x360 NEURAL_LIGHT) and the real nvblox mapping parameters (franka_zed_nvblox_real.launch.py = the lab's nvblox_5cm launch).
# Pass VOXEL=0.05 for the lab's map. REAL=0 (default) = the sim's own setup, unchanged.
REALARG=(); if [ "${REAL:-0}" = 1 ]; then ZED_YAML=${ZED_YAML:-sim_real.yaml}; REALARG=(--real-cams /workspace/franka_mpc/real_cams.json); export NVLAUNCH=${NVLAUNCH:-franka_zed_nvblox_real.launch.py}; fi
export ZED_YAML=${ZED_YAML:-sim_quality.yaml}                # receivers: NEURAL_PLUS depth at full resolution
NODE_ARGS=${NODE_ARGS--p extra_collision_sphere_buffer:=0.03}   # 3 cm margin (set NODE_ARGS= for none)
VOXEL=${VOXEL:-0.01}; EXTRA_VOXEL=${EXTRA_VOXEL:-}; MESH=${MESH:-1}; MAP_ONLY=${MAP_ONLY:-0}; MAP_SECS=${MAP_SECS:-150}
NV=nvblox-lite:3.2.5-humble
aux() {  # name cmd...: a helper container from the nvblox image (its ROS env, nvblox_msgs), outputs in $OUT; runs as
        # root like the other ROS containers (as uid 1000 fast-DDS shared memory with them fails and nothing arrives)
  local n=$1; shift
  docker run --rm -d --name "$n" --network host --ipc host -e ROS_DOMAIN_ID=8 \
    -v "$Z/nvblox":/ws/nvblox:ro -v "$OUT":/out "$NV" bash -c "source /opt/ros/humble/setup.bash && exec $*" >/dev/null; }
mapdump() {  # service tag: the node's grid region of that nvblox map -> $OUT/map_check[_tag]
  docker exec nvblox-lite bash -c "source /opt/ros/humble/setup.bash && mkdir -p /ws/nvblox/out/$RUN/$2 && python3 /ws/nvblox/map_check.py \
    --workcell /workcell --out /ws/nvblox/out/$RUN/$2 --service $1 --aabb-min -0.36 -0.80 -0.16 --aabb-size 1.6 1.6 1.2 \
    && chown -R 1000:1000 /ws/nvblox/out/$RUN" >> "$OUT/map_check.log" 2>&1 && { d=map_check; [ "$2" = main ] || d=map_check_$2; cp -r "$Z/nvblox/out/$RUN/$2" "$OUT/$d"; }; }
OUT=$PWD/runs/$RUN; mkdir -p "$OUT"; rm -f "$OUT"/mpc_node.csv
declare -A PORT=([zed]=30000 [zed2]=30010)
ZEDARG=$(for c in $CAMS; do printf "%s:%s," "$c" "${PORT[$c]}"; done); ZEDARG=${ZEDARG%,}
stop() { docker stop -t 30 mesh-main mesh-extra >/dev/null 2>&1   # recorders first: they save on SIGTERM
         docker run --rm -v "$OUT":/out "$NV" chown -R 1000:1000 /out >/dev/null 2>&1
         docker stop franka-goal franka-mpc franka-seg franka-seg2 nvblox-lite zed-lite zed-lite2 >/dev/null 2>&1
         docker stop franka-isaac >/dev/null 2>&1; }      # receivers before Isaac (no stale stream lock)
trap stop EXIT
wait_for() {  # file pattern timeout_s [count]
  local t=0 n=${4:-1}; until [ "$(grep -c "$2" "$1" 2>/dev/null)" -ge "$n" ]; do sleep 1; t=$((t + 1))
    [ $t -ge "$3" ] && { echo "timeout waiting for '$2' in $1"; tail -15 "$1"; return 1; }; done; return 0; }
NAME=franka-isaac ROS=jazzy IMAGE=spot-isaac-sim:6.0.0-zed ./run_isaac.sh -s franka_ros_arm.py "$RUN" --seconds 18.5 \
  --zeds "$ZEDARG" "${REALARG[@]}" "$@" > "$OUT/isaac.log" 2>&1 &
ISAAC=$!
wait_for "$OUT/isaac.log" "waiting for /wrist_pose" 900 || exit 1; echo "1 isaac up, streaming $ZEDARG"
for c in $CAMS; do
  n=zed-lite; [ "$c" = zed ] || n=zed-lite${c#zed}
  CAM=$c PORT=${PORT[$c]} NAME=$n "$Z/run_zed.sh" > "$OUT/zed_$c.log" 2>&1 &
done
for c in $CAMS; do                                  # a wrapper sometimes hangs in the stream handshake: restart it once
  n=zed-lite; [ "$c" = zed ] || n=zed-lite${c#zed}
  wait_for "$OUT/zed_$c.log" "=== $c started ===" "${ZED_WAIT:-240}" || { echo "restarting $n"; docker stop "$n" >/dev/null
    CAM=$c PORT=${PORT[$c]} NAME=$n "$Z/run_zed.sh" >> "$OUT/zed_$c.log" 2>&1 &
    wait_for "$OUT/zed_$c.log" "=== $c started ===" 600 || exit 1; }
done; echo "2 ZED wrappers up"
CAMS="$CAMS" CAMDIR=$OUT "$Z/nvblox/run_nvblox.sh" voxel_size:=$VOXEL > "$OUT/nvblox.log" 2>&1 &
wait_for "$OUT/nvblox.log" "static TF published" 180 "$(echo $CAMS | wc -w)" || exit 1; echo "3 nvblox up (voxel $VOXEL)"
if [ -n "$EXTRA_VOXEL" ]; then
  docker exec -d nvblox-lite bash -c "source /opt/ros/humble/setup.bash && ros2 launch /ws/nvblox/franka_zed_nvblox.launch.py \
    cameras:=$(echo $CAMS | tr ' ' ,) voxel_size:=$EXTRA_VOXEL node_name:=nvblox_node_extra > /ws/nvblox/out/nvblox_extra_$RUN.log 2>&1"
  echo "3b second nvblox (voxel $EXTRA_VOXEL) as nvblox_node_extra"
fi
if [ "$MESH" = 1 ]; then
  aux mesh-main python3 /ws/nvblox/mesh_recorder.py --topic /nvblox_node/mesh --out /out/mesh_main.npz
  [ -n "$EXTRA_VOXEL" ] && aux mesh-extra python3 /ws/nvblox/mesh_recorder.py --topic /nvblox_node_extra/mesh --out /out/mesh_extra.npz
  echo "3c recording the mesh topic(s)"
fi
for c in $CAMS; do
  if [ "$c" = zed ]; then NAME=franka-seg "$Z/nvblox/run_segmenter.sh" --joint-states /joint_states_isaac $SEG_ARGS > "$OUT/seg_$c.log" 2>&1 &
  else NAME=franka-seg${c#zed} "$Z/nvblox/run_segmenter.sh" --joint-states /joint_states_isaac $SEG_ARGS --depth /$c/zed_node/depth/depth_registered \
         --info /$c/zed_node/depth/camera_info --out-depth /franka_seg_$c/depth --out-mask /franka_seg_$c/robot_mask > "$OUT/seg_$c.log" 2>&1 &
  fi
done
for c in $CAMS; do wait_for "$OUT/seg_$c.log" "depth_out" 300 || exit 1; done; echo "4 self-filters up"
if [ "$MAP_ONLY" = 1 ]; then
  sleep "$MAP_SECS"
  DP=""; for c in $CAMS; do s=; [ "$c" = zed ] && s=/franka_seg/depth || s=/franka_seg_$c/depth
    DP+=" ${c}_raw:/$c/zed_node/depth/depth_registered:/$c/zed_node/depth/camera_info ${c}_filtered:$s:/$c/zed_node/depth/camera_info"; done
  docker run --rm --network host --ipc host -e ROS_DOMAIN_ID=8 -v "$Z/nvblox":/ws/nvblox:ro -v "$OUT":/out \
    "$NV" bash -c "source /opt/ros/humble/setup.bash && python3 /ws/nvblox/depth_dump.py --out /out/depth.npz --pairs $DP" > "$OUT/depth_dump.log" 2>&1
  mapdump /nvblox_node/get_esdf_and_gradient main && echo "map saved (main)"
  [ -n "$EXTRA_VOXEL" ] && mapdump /nvblox_node_extra/get_esdf_and_gradient extra && echo "map saved (extra)"
  exit 0
fi
OUT=$OUT ./run_mpc_node.sh -p use_esdf:=true -p voxel_size:=$VOXEL -p "grid_size_m:=[1.6,1.6,1.2]" \
  -p "grid_center_m:=[0.45,0.0,0.45]" ${NODE_ARGS:-} > "$OUT/node.log" 2>&1 &
wait_for "$OUT/node.log" "Initialized goal to current physical pose" 300 || exit 1; echo "5 node warmed up"
wait_for "$OUT/node.log" "ESDF voxel grid updated" 900 5 || exit 1
grep "occupied voxels" "$OUT/node.log" | tail -1; echo "6 map in the node (5 ESDF updates)"
./run_goal.sh -p path:=sweep ${GOAL_ARGS:-} > "$OUT/goal.log" 2>&1 &
wait $ISAAC; echo "isaac done (exit $?)"
mapdump /nvblox_node/get_esdf_and_gradient main && echo "map saved (the node's grid region)"
[ -n "$EXTRA_VOXEL" ] && mapdump /nvblox_node_extra/get_esdf_and_gradient extra && echo "map saved (extra)"
grep "RESULT\|moved" "$OUT/isaac.log" | tail -2 | cut -c1-400; grep "^video" "$OUT/isaac.log"
