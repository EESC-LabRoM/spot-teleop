#!/usr/bin/env bash
# Deployment-path sim: Newton FR3 from the lab READY pose + the VM's MPC node (vm_real/node) + deploy_bridge (the real slew() from
# curobo_to_jtc.py, optional faults) + a moving goal. Everything is localhost-only in DDS: nothing here may be seen by the lab's domain 8.
#   RUN=<name> [FEEDBACK=hybrid|measured] [USE_ESDF=true] [BRIDGE_ARGS="--spike 12"] [CFG=fr3_vcap1.2_acc10.yml] [INFEAS=reset|keep] [CMDTYPE=joint_state|joint_trajectory (add --traj to BRIDGE_ARGS)] ./run_deploy_sim.sh
set -uo pipefail
cd "$(dirname "$0")"; source ds_env.sh
RUN=${RUN:?set RUN}; OUT=$FM/runs/$RUN; FB=${FEEDBACK:-hybrid}; CFG=${CFG:-fr3_vcap1.2_acc10.yml}; SECS=${SECS:-41}
mkdir -p "$OUT"; rm -f "$OUT"/*.csv "$OUT"/*.log "$OUT"/trace.npz "$OUT"/summary.json; mkdir -p "$OUT/ctl"; rm -f "$OUT/ctl/esdf_go"
LOC=(-e ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST)
stop() { docker stop -t 3 franka-mpc esdf-replay ds-bridge ds-goal ds-fwdA ds-fwdB >/dev/null 2>&1; docker stop -t 10 newton-sim >/dev/null 2>&1; }; trap stop EXIT; stop
wait_for() { local t=0; until grep -a -q "$2" "$1" 2>/dev/null; do sleep 1; t=$((t + 1)); [ $t -ge "$3" ] && { echo "timeout waiting for '$2'"; tail -8 "$1"; return 1; }; done; return 0; }
ROSENV=(-e ROS_DISTRO=jazzy -e RMW_IMPLEMENTATION=rmw_fastrtps_cpp -e ROS_DOMAIN_ID=8 -e PYTHONUNBUFFERED=1 -e PYTHONDONTWRITEBYTECODE=1 -e ACCEPT_EULA=Y "${LOC[@]}")
SP=$WS/crawl-lab/IsaacLab/.venv/lib/python3.12/site-packages
docker run --rm -d --name newton-sim --gpus device=0 --network host --ipc host -e Q0 "${ROSENV[@]}" -e LD_LIBRARY_PATH=$J/lib -e PYTHONPATH="$SP:$J/rclpy" \
  -v "$SP":"$SP":ro -v /home/nexus/newton_sp:/home/nexus/newton_sp:ro -v "$WS":/workspace:rw -v "$FM/cache_newton_home/newton":/root/.cache/newton -v "$FM/cache_newton_home/warp":/root/.cache/warp \
  --entrypoint /isaac-sim/python.sh $IMG /workspace/franka_mpc/newton_ros_arm_home.py --out /workspace/franka_mpc/runs/$RUN --seconds $SECS --wait 900 --episodes 1 >/dev/null
( docker logs -f newton-sim > "$OUT/newton.log" 2>&1 & )
wait_for "$OUT/newton.log" "\[newton\] ready" 300 || exit 1
ESDFARGS=""
if [ "${USE_ESDF:-true}" = true ]; then
  MAP=${MAP:-$FM/runs/maponly_hd1080/map_check}
  docker run --rm -d --name esdf-replay --network host --ipc host "${ROSENV[@]}" -e LD_LIBRARY_PATH=$J/lib:/nvm/lib -e PYTHONPATH="/isaac-sim/exts/omni.pip.compute/pip_prebundle:$J/rclpy:/nvm/lib/python3.12/site-packages" \
    -v "$FM":/fm:ro -v "$NVM":/nvm:ro -v "$MAP":/map:ro -v "$OUT/ctl":/ctl:rw --entrypoint /isaac-sim/python.sh $IMG /fm/esdf_replay.py --npz /map/${NPZ:-esdf.npz} --gate-file /ctl/esdf_go --service /esdf_replay/get_esdf_and_gradient >/dev/null
  ESDFARGS="-p use_esdf:=true -p voxel_size:=${VOXEL:-0.05} -p grid_size_m:=[1.4,1.6,1.2] -p grid_center_m:=${GRIDC:-[0.35,0.0,0.55]} -p esdf_service_name:=/esdf_replay/get_esdf_and_gradient"
fi
# E2E=1 (wiring test): the REAL curobo_to_jtc.py twice, as in vm_real.sh: VM half (ros -> udp, flags copied from vm_real.sh) and lab half (udp -> ros).
# UDP stays on 127.0.0.1 at a private port, nothing can reach the lab. Run with NODE_OUT=/curobo/joint_trajectory CMDTYPE=joint_trajectory
# BRIDGE_ARGS="--traj --noslew --cmd-topic /fr3_arm_controller/joint_trajectory" (the bridge then only stands in for the controller + sim).
if [ "${E2E:-0}" = 1 ]; then
  P=47892; FA=$(sed -n 's/.*lab\/curobo_to_jtc.py \(.*\) > \/out\/to_jtc_in.log.*/\1/p' "$VR/vm_real.sh"); FA=${FA//47802/$P}; FA=$(MAXSTEP=0.03 MAXACCEL=0 eval echo "$FA")
  echo "VM half flags (from vm_real.sh): $FA"; echo "$FA" > "$OUT/fwdA.args"
  DOMAIN=8 "${DSRUN[@]}" -d --name ds-fwdB "${LOC[@]}" -e CMD_HOST=127.0.0.1 --gpus device=1 $IMG /vr/lab/curobo_to_jtc.py --in udp --out ros --port $P >/dev/null
  DOMAIN=8 "${DSRUN[@]}" -d --name ds-fwdA "${LOC[@]}" -e CMD_HOST=127.0.0.1 --gpus device=1 $IMG /vr/lab/curobo_to_jtc.py $FA >/dev/null
fi
# bridge first: the sim idles until the first command/goal, so nothing is lost
DOMAIN=8 "${DSRUN[@]}" -d --name ds-bridge "${LOC[@]}" -v "$OUT":/out:rw --gpus device=1 $IMG /workspace/franka_mpc/deploy_sim/deploy_bridge.py --max-step ${MAXSTEP:-0.03} ${BRIDGE_ARGS:-} --out /out/bridge.csv >/dev/null
OUT=$OUT NODE=$VR/node GPUS=device=1 DOCKER_EXTRA="-e MPC_EXECUTOR=events ${LOC[*]}" ./../run_mpc_node.sh -p use_esdf:=${USE_ESDF:-true} ${ESDFARGS} \
  -p robot_config:=/workspace/franka_mpc/node_config/$CFG -p step_on_clock:=true -p feedback_mode:=$FB -p mpc_on_infeasible:=${INFEAS:-reset} -p command_type:=${CMDTYPE:-joint_state} -r /joint_command_curobo:=${NODE_OUT:-/cmd_raw} ${NODE_EXTRA:-} > "$OUT/node.log" 2>&1 &
wait_for "$OUT/node.log" "Initialized goal to current physical pose" 300 || exit 1
[ "${USE_ESDF:-true}" = true ] && touch "$OUT/ctl/esdf_go"
echo "node up ($FB, $CFG, esdf=${USE_ESDF:-true}); starting the goal"; sleep 3
DOMAIN=8 "${DSRUN[@]}" -d --name ds-goal -e P0 -e QUAT -e WPS -e HOLD -e TAIL "${LOC[@]}" --gpus device=1 $IMG /workspace/franka_mpc/deploy_sim/${GOAL:-goal_step.py} --ros-args -p use_sim_time:=true >/dev/null
t=0; until [ -f "$OUT/trace.npz" ]; do sleep 2; t=$((t + 2)); [ $t -ge 400 ] && { echo "timeout: no trace.npz"; tail -5 "$OUT/newton.log"; exit 1; }; done; sleep 2
for f in ds-fwdA ds-fwdB; do docker logs $f > "$OUT/$f.log" 2>&1; done; docker stop -t 3 ds-bridge >/dev/null 2>&1
"${DSRUN[@]}" --gpus device=1 -v "$OUT":/out:ro $IMG /workspace/franka_mpc/deploy_sim/analyze_deploy.py /out "${RUN}" 2>&1 | tee "$OUT/summary.txt"
