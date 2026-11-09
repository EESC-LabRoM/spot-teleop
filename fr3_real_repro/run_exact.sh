#!/usr/bin/env bash
# Exact-depth (no ZED SDK) obstacle run: Isaac publishes exact sim depth+rgb on the
# ZED wrapper topic names, so nvblox + self-filter run UNCHANGED (single factor vs run 5 = depth source only).
# Same viewpoints (REAL=1), 5 cm, Spot nvblox, 3.4 cm margin, vcap1.2.
#   [MAP_ONLY=1] [MAP_SECS=90] [VOXEL=0.05] ./run_exact.sh [run_name]
#   MAP_ONLY=1 (default): static map only, no node/sweep. MAP_ONLY=0: full MPC sweep.
cd /home/nexus/spot-teleop/isaac-sim_ws/franka_mpc || exit 1
Z=/home/nexus/spot-teleop/zed_lite
RUN=${1:-exact_maponly_5cm}
CAMS=${CAMS:-zed zed2}; SEG_ARGS=${SEG_ARGS:---threshold 0.15}
export NVLAUNCH=${NVLAUNCH:-franka_zed_nvblox.launch.py}   # Spot defaults, like run 5
export TFBIN=exact_extrinsics_tf.py
export NODE_ARGS="-p extra_collision_sphere_buffer:=0.03 -p robot_config:=/workspace/franka_mpc/node_config/fr3_vcap1.2_acc10.yml -p target_clear_frame:=fr3_hand -p target_clear_radius_m:=0.15 -p grid_size_m:=[1.4,1.6,1.2] -p grid_center_m:=[0.35,0.0,0.55]"
VOXEL=${VOXEL:-0.05}; MESH=${MESH:-1}; MAP_ONLY=${MAP_ONLY:-1}; MAP_SECS=${MAP_SECS:-90}
NV=nvblox-lite:3.2.5-humble
aux() {
  local n=$1; shift
  docker run --rm -d --name "$n" --network host --ipc host -e ROS_DOMAIN_ID=8 \
    -v "$Z/nvblox":/ws/nvblox:ro -v "$OUT":/out "$NV" bash -c "source /opt/ros/humble/setup.bash && exec $*" >/dev/null; }
mapdump() {
  docker exec nvblox-lite bash -c "source /opt/ros/humble/setup.bash && mkdir -p /ws/nvblox/out/$RUN/$2 && python3 /ws/nvblox/map_check.py \
    --workcell /workcell --out /ws/nvblox/out/$RUN/$2 --service $1 --aabb-min -0.36 -0.80 -0.16 --aabb-size 1.6 1.6 1.2 \
    && chown -R 1000:1000 /ws/nvblox/out/$RUN" >> "$OUT/map_check.log" 2>&1 && { d=map_check; [ "$2" = main ] || d=map_check_$2; cp -r "$Z/nvblox/out/$RUN/$2" "$OUT/$d"; }; }
OUT=$PWD/runs/$RUN; mkdir -p "$OUT"; rm -f "$OUT"/mpc_node.csv
stop() { docker stop -t 30 mesh-main mesh-extra >/dev/null 2>&1
         docker run --rm -v "$OUT":/out "$NV" chown -R 1000:1000 /out >/dev/null 2>&1
         docker stop franka-goal franka-mpc franka-seg franka-seg2 nvblox-lite >/dev/null 2>&1
         docker stop franka-isaac >/dev/null 2>&1; }
trap stop EXIT
wait_for() {
  local t=0 n=${4:-1}; until [ "$(grep -c "$2" "$1" 2>/dev/null)" -ge "$n" ]; do sleep 1; t=$((t + 1))
    [ $t -ge "$3" ] && { echo "timeout waiting for '$2' in $1"; tail -15 "$1"; return 1; }; done; return 0; }
if [ "$MAP_ONLY" = 1 ]; then NOvid=--no-video; else NOvid=; fi
echo "RUN START $(date +%T) exact run=$RUN voxel=$VOXEL map_only=$MAP_ONLY"
NAME=franka-isaac ROS=jazzy IMAGE=spot-isaac-sim:6.0.0-zed ./run_isaac.sh -s franka_ros_arm.py "$RUN" --seconds 18.5 \
  --exact-depth --exact-cams $(echo $CAMS | tr ' ' ,) --real-cams /workspace/franka_mpc/real_cams.json --zed-preview $NOvid --wait 1800 > "$OUT/isaac.log" 2>&1 &
ISAAC=$!
wait_for "$OUT/isaac.log" "waiting for /wrist_pose" 900 || { echo "isaac failed to boot"; exit 1; }
wait_for "$OUT/isaac.log" "exact depth /World/zed" 600 || { echo "exact publishers never came up"; exit 1; }
echo "1 isaac up, exact depth streaming (no ZED wrappers)"
CAMS="$CAMS" CAMDIR=$OUT "$Z/nvblox/run_nvblox.sh" voxel_size:=$VOXEL > "$OUT/nvblox.log" 2>&1 &
wait_for "$OUT/nvblox.log" "static TF published" 180 "$(echo $CAMS | wc -w)" || exit 1; echo "2 nvblox up (voxel $VOXEL)"
if [ "$MESH" = 1 ]; then
  aux mesh-main python3 /ws/nvblox/mesh_recorder.py --topic /nvblox_node/mesh --out /out/mesh_main.npz
  echo "2b recording the mesh topic"
fi
for c in $CAMS; do
  if [ "$c" = zed ]; then NAME=franka-seg "$Z/nvblox/run_segmenter.sh" --joint-states /joint_states_isaac $SEG_ARGS > "$OUT/seg_$c.log" 2>&1 &
  else NAME=franka-seg${c#zed} "$Z/nvblox/run_segmenter.sh" --joint-states /joint_states_isaac $SEG_ARGS --depth /$c/zed_node/depth/depth_registered \
         --info /$c/zed_node/depth/camera_info --out-depth /franka_seg_$c/depth --out-mask /franka_seg_$c/robot_mask > "$OUT/seg_$c.log" 2>&1 &
  fi
done
for c in $CAMS; do wait_for "$OUT/seg_$c.log" "depth_out" 300 || exit 1; done; echo "3 self-filters up"
if [ "$MAP_ONLY" = 1 ]; then
  sleep "$MAP_SECS"
  DP=""; for c in $CAMS; do s=; [ "$c" = zed ] && s=/franka_seg/depth || s=/franka_seg_$c/depth
    DP+=" ${c}_raw:/$c/zed_node/depth/depth_registered:/$c/zed_node/depth/camera_info ${c}_filtered:$s:/$c/zed_node/depth/camera_info"; done
  docker run --rm --network host --ipc host -e ROS_DOMAIN_ID=8 -v "$Z/nvblox":/ws/nvblox:ro -v "$OUT":/out \
    "$NV" bash -c "source /opt/ros/humble/setup.bash && python3 /ws/nvblox/depth_dump.py --out /out/depth.npz --pairs $DP" > "$OUT/depth_dump.log" 2>&1
  mapdump /nvblox_node/get_esdf_and_gradient main && echo "map saved (main)"
  echo "RUN FINISHED rc=$? $(date +%T)"
  exit 0
fi
OUT=$OUT ./run_mpc_node.sh -p use_esdf:=true -p voxel_size:=$VOXEL -p "grid_size_m:=[1.6,1.6,1.2]" \
  -p "grid_center_m:=[0.45,0.0,0.45]" ${NODE_ARGS:-} > "$OUT/node.log" 2>&1 &
wait_for "$OUT/node.log" "Initialized goal to current physical pose" 300 || exit 1; echo "4 node warmed up"
wait_for "$OUT/node.log" "ESDF voxel grid updated" 900 5 || exit 1
grep "occupied voxels" "$OUT/node.log" | tail -1; echo "5 map in the node (5 ESDF updates)"
./run_goal.sh -p path:=sweep ${GOAL_ARGS:-} > "$OUT/goal.log" 2>&1 &
wait $ISAAC; echo "isaac done (exit $?)"
mapdump /nvblox_node/get_esdf_and_gradient main && echo "map saved (the node's grid region)"
grep "RESULT\|moved" "$OUT/isaac.log" | tail -2 | cut -c1-400; grep "^video" "$OUT/isaac.log"
echo "RUN FINISHED rc=$? $(date +%T)"
