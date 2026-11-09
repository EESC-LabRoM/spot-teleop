#!/usr/bin/env bash
# cuRobo MPC on VM 103 against the real FR3 driver on the lab PC, over the direct cable (VM 10.0.0.3 <-> lab 10.0.0.2).
#   up      INERT. Relay (domain 142) + MPC in STANDBY on GPU 0 of the VM. Joint states arrive by udp_fwd on the lab (127.0.0.1:47801 -> 10.0.0.3).
#   status  what runs, joint-state rate, controllers, whether Hipolito's joystick_bridge is alive
#   traj    65 s goal path to the VM planner (needs MPC active; ARM MOVES if the arm path is live)
#   run     single-command deploy: go + traj + back, with MOVE confirmation. ARM MOVES, needs ESTOP=1.
#   go      THE ARM MOVES. Needs ESTOP=1 (a person at the e-stop). Refuses if Hipolito's bridge is ARMED, if anything publishes to the JTC, or on unknown controller state.
#           A stranded both-inactive driver (fr3_arm_controller + Cartesian both inactive, e.g. after a failed swap) is recovered automatically:
#           the Cartesian hold is re-activated first (no motion), then the normal swap runs. Unknown states still refuse.
#           starts curobo_to_jtc inside fr3-ros2 (--msg joint_trajectory --max-step 0.03 --max-accel off), activates fr3_arm_controller (deactivating Cartesian first if present), MPC active. Hold test: no goal stream.
#   back    MPC standby, deactivate fr3_arm_controller, stop curobo_to_jtc in the driver.   down: remove the VM containers.
# env: MAXACCEL=0 (acceleration limiter in the VM forwarder, off; MAXACCEL=10 to enable)  CFG=fr3_vcap1.2_acc10.yml  FEEDBACK=hybrid  USE_ESDF=false  GPU=0
#   OBSTACLE_BOX="x,y,z,sx,sy,sz" (planner-local virtual cuboid in fr3_link0, metres; "" = none). Safe avoidance probe: nothing physically there.
set -uo pipefail
H=$(cd "$(dirname "$0")" && pwd); LAB="ssh -i $HOME/.ssh/id_ed25519_lab_claude -o ConnectTimeout=8 murilo@100.88.134.127"
CURO=$H/../../isaac-sim_ws/curobo-isaac6; PYD=$H/../../isaac-sim_ws/franka_mpc/pydeps; NVM=$H/../../zed_lite/nvblox_msgs_jazzy/install
IMG=spot-isaac-sim:6.0.0; J=/isaac-sim/exts/isaacsim.ros2.core/jazzy; L=$H/log; mkdir -p $L
SRC="source /opt/ros/jazzy/setup.bash; source /ros2_ws/install/setup.bash"
EA="-p voxel_size:=0.05 -p grid_size_m:=[1.4,1.6,1.2] -p grid_center_m:=[0.35,0.0,0.55] -p esdf_service_name:=/nvblox_curobo/get_esdf_and_gradient"; [ "${USE_ESDF:-true}" = true ] || EA=""
VB=""; [ -n "${OBSTACLE_BOX:-}" ] && VB="-p virtual_box:=$OBSTACLE_BOX"
XB=""; [ -n "${EXTRA_BUF:-}" ] && XB="-p extra_collision_sphere_buffer:=$EXTRA_BUF"
D=(-e ROS_DOMAIN_ID=143 -e ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET -e FASTDDS_BUILTIN_TRANSPORTS=UDPv4 -e RMW_IMPLEMENTATION=rmw_fastrtps_cpp -e ROS_DISTRO=jazzy)
PY=/isaac-sim/extscache/omni.warp.core-1.13.0+lx64:/isaac-sim/extsDeprecated/omni.isaac.ml_archive/pip_prebundle:/isaac-sim/exts/omni.pip.compute/pip_prebundle
PY=$PY:/workspace/curobo-isaac6/src:/workspace/curobo-isaac6/.venv/lib/python3.12/site-packages:/workspace/franka_mpc/pydeps:$J/rclpy:/nvm/lib/python3.12/site-packages
drv() { $LAB "docker exec fr3-ros2 bash -c '$SRC; $*'"; }
ctrl() { drv "timeout 10 ros2 control list_controllers 2>/dev/null" | sed 's/\x1b\[[0-9;]*m//g' | grep -E "^(fr3_arm_controller|cartesian_impedance_target_controller) " | awk '{print $1"="$NF}' | tr '\n' ' '; }
bridge() { $LAB 'docker exec fr3-ros2 bash -c "for p in /proc/[0-9]*; do c=\$(tr \"\\0\" \" \" < \$p/cmdline 2>/dev/null); case \"\$c\" in *joystick_bridge*) echo \${p#/proc/};; esac; done"' | grep -v '^$' | head -1; }
mset() { docker exec -e ROS_DOMAIN_ID=143 -e ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET -e FASTDDS_BUILTIN_TRANSPORTS=UDPv4 -e RMW_IMPLEMENTATION=rmw_fastrtps_cpp -e LD_LIBRARY_PATH=$J/lib -e PYTHONPATH=$J/rclpy:/isaac-sim/exts/omni.pip.compute/pip_prebundle vm-mpc /isaac-sim/python.sh -c "
import rclpy,time
from std_srvs.srv import SetBool
rclpy.init(); n=rclpy.create_node('vmset'); c=n.create_client(SetBool,'/curobo_mpc_node/set_active'); time.sleep(6); r=None
for k in range(3):
    f=c.call_async(SetBool.Request(data=$1)); rclpy.spin_until_future_complete(n,f,timeout_sec=6)
    if f.done(): r=f.result(); break
print('set_active',$1,'->',None if r is None else (r.success, r.message))" 2>&1 | tail -1; }
state() { grep -a -o 'MPC state: [a-z]*' $L/mpc/mpc_node.csv 2>/dev/null | tail -1 | awk '{print $3}'; }
case "${1:-}" in
  up)
    docker rm -f vm-mpc vm-relay >/dev/null 2>&1; rm -rf $L/mpc $L/relay; mkdir -p $L/mpc $L/relay
    docker run -d --rm --name vm-relay --network host "${D[@]}" -e RELAY_BIND=10.0.0.3 -e CMD_HOST=10.0.0.2 -e PYTHONUNBUFFERED=1 \
      -e LD_LIBRARY_PATH=$J/lib -e PYTHONPATH=$J/rclpy:/isaac-sim/exts/omni.pip.compute/pip_prebundle -v $H:/h:ro -v $L/relay:/out:rw --entrypoint bash $IMG -c \
      "/isaac-sim/python.sh /h/shadow/relay_out.py 47801 > /out/relay_out.log 2>&1 & /isaac-sim/python.sh /h/lab/curobo_to_jtc.py --in ros --out udp --msg joint_trajectory --topic-in /curobo/joint_trajectory --port 47802 --max-step ${MAXSTEP:-0.03} --max-accel ${MAXACCEL:-0} > /out/to_jtc_in.log 2>&1 & sleep infinity" >/dev/null || { echo FAIL relay; exit 1; }
    docker run -d --rm --name vm-mpc --gpus device=${GPU:-0} --network host --ipc host --cap-add SYS_NICE --ulimit rtprio=99:99 "${D[@]}" \
      -e LD_LIBRARY_PATH=$J/lib:/nvm/lib -e PYTHONPATH="$PY" -e PYTHONUNBUFFERED=1 -e PYTHONDONTWRITEBYTECODE=1 -e SETUPTOOLS_SCM_PRETEND_VERSION=0.7.7.post1.dev5 -e ACCEPT_EULA=Y -e MPC_EXECUTOR=events \
      -v $CURO:/workspace/curobo-isaac6:ro -v $H/node_config:/workspace/franka_mpc/node_config:ro -v $PYD:/workspace/franka_mpc/pydeps:ro -v $NVM:/nvm:ro -v $H/node:/node:ro -v $H/vm_goal_path.py:/goal/vm_goal_path.py:ro -v $L/mpc:/out:rw \
      -v $HOME/spot-teleop/isaac-sim_ws/cache/computecache:/root/.nv/ComputeCache --entrypoint bash $IMG -c "nice -n -10 /isaac-sim/python.sh /node/curobo_mpc_node.py --ros-args \
        -p robot:=franka -p base_frame:=fr3_link0 -p robot_config:=/workspace/franka_mpc/node_config/${CFG:-fr3_vcap1.2_acc10.yml} -p use_sim:=false -p use_ros2_control:=false \
        -p use_esdf:=${USE_ESDF:-true} $EA $VB $XB -p \"pf_mode:='off'\" -p esdf_frame_id:=fr3_link0 -p esdf_global_frame:=fr3_link0 -p feedback_mode:=${FEEDBACK:-hybrid} -p start_active:=false \
        -p log_csv_path:=/out/mpc_node.csv -p command_type:=joint_trajectory ${NODE_EXTRA:-} -r /arm/joint_command:=/curobo/joint_trajectory > /out/mpc_stdout.log 2>&1; chown -R $(id -u):$(id -g) /out" >/dev/null || { echo FAIL mpc; exit 1; }
    for i in $(seq 1 180); do [ "$(state)" = standby ] && break; docker ps --format '{{.Names}}' | grep -qx vm-mpc || { tail -15 $L/mpc/mpc_stdout.log; echo "MPC exited"; exit 1; }; sleep 1; done
    sleep 5; echo "up: MPC $(state) after ${i}s; $(grep -a 'relay last' $L/relay/relay_out.log | tail -1)" ;;
  status)
    docker ps --format '{{.Names}} {{.Status}}' | grep -E "^vm-(mpc|relay)"; echo "MPC state: $(state)"; grep -a 'relay last' $L/relay/relay_out.log 2>/dev/null | tail -1
    echo "controllers: $(ctrl)"; echo "joystick_bridge pid in driver: $(bridge)"; $LAB 'tail -1 /home/murilo/claude_plan_run/fwd_js.log; tail -1 /home/murilo/claude_plan_run/fwd_cmd.log' ;;
  go)
    [ "${ESTOP:-0}" = 1 ] || { echo "refused: set ESTOP=1 (person at the e-stop)"; exit 1; }
    [ "$(state)" = standby ] || { echo "refused: MPC not in standby (up first)"; exit 1; }
    rj=0; if [ -n "$(find $L/relay/relay_out.log -mmin -1 2>/dev/null)" ]; then rj=$(grep -a 'relay last' $L/relay/relay_out.log | tail -1 | grep -o 'joint_states [0-9]*' | awk '{print $2}'); fi
    [ "${rj:-0}" -ge 100 ] || { echo "refused: lab joint states not flowing (relay shows ${rj:-none}/10 s; lab relay_in down?)"; exit 1; }
    c=$(ctrl); echo "controllers: $c"
    armed=$($LAB 'curl -s -m 3 http://127.0.0.1:8784/status' | grep -o '"armed": *[a-z]*' | head -1); echo "bridge: ${armed:-unreachable}"
    if echo "${armed:-}" | grep -q '"armed": *false'; then echo "bridge disarmed, OK"
    elif [ "${BRIDGE_ABSENT_OK:-0}" = 1 ]; then echo "WARNING: bridge page down, proceeding on explicit BRIDGE_ABSENT_OK=1 (owner away + permission; his driving stack verified absent, all other guards still apply)"
    else echo "refused: Hipolito's bridge is ARMED (or its status page is down)"; exit 1; fi
    if [ -n "$(bridge)" ]; then   # Hipolito's bridge may stay alive, but nothing may be publishing to the JTC right now
      drv "timeout 10 ros2 topic hz /fr3_arm_controller/joint_trajectory 2>&1" | grep -q "average rate" && { echo "refused: something is publishing to fr3_arm_controller/joint_trajectory (his session is live)"; exit 1; }; fi
    if echo "$c" | grep -q "fr3_arm_controller=active"; then SW=0; elif echo "$c" | grep -q "fr3_arm_controller=inactive" && echo "$c" | grep -q "cartesian_impedance_target_controller=active"; then SW=1
    elif echo "$c" | grep -q "fr3_arm_controller=inactive" && ! echo "$c" | grep -q "cartesian_impedance_target_controller"; then SW=2
    elif echo "$c" | grep -q "fr3_arm_controller=inactive" && echo "$c" | grep -q "cartesian_impedance_target_controller=inactive"; then SW=3
    else echo "refused: controllers '$c'"; exit 1; fi
    if [ $SW = 3 ]; then echo "both motion controllers inactive: restoring the Cartesian hold first (no motion)"
      drv "timeout 4 ros2 service call /teleop/disengage std_srvs/srv/Trigger" >/dev/null 2>&1; sleep 1.5
      drv "timeout 10 ros2 control switch_controllers --activate cartesian_impedance_target_controller --strict" 2>&1 | tail -1
      echo "controllers: $(ctrl)"
      echo "$(ctrl)" | grep -q "cartesian_impedance_target_controller=active" || { echo "refused: the Cartesian hold did not re-activate (nothing started, nothing switched): on the lab run ./fr3_ros2.sh real-recover (Desk: clear error, unlock joints, Activate FCI), then retry"; exit 1; }
      echo "hold restored, continuing with the normal swap"
    fi
    $LAB "docker cp /home/murilo/franka_ws/hub-curobo/lab/curobo_to_jtc.py fr3-ros2:/tmp/curobo_to_jtc.py" >/dev/null
    $LAB "docker exec -d fr3-ros2 bash -c '$SRC; python3 /tmp/curobo_to_jtc.py --in udp --out ros --msg joint_trajectory --port 47802 --max-step ${MAXSTEP:-0.03} --max-accel ${MAXACCEL:-0} > /tmp/curobo_to_jtc_out.log 2>&1'"
    if [ $SW = 1 ] || [ $SW = 3 ]; then drv "timeout 4 ros2 service call /teleop/disengage std_srvs/srv/Trigger" >/dev/null 2>&1; sleep 1.5
      drv "timeout 10 ros2 control switch_controllers --activate fr3_arm_controller --deactivate cartesian_impedance_target_controller --strict" 2>&1 | tail -1
    elif [ $SW = 2 ]; then drv "timeout 10 ros2 control switch_controllers --activate fr3_arm_controller --strict" 2>&1 | tail -1; fi
    echo "controllers: $(ctrl)"
    if ! echo "$(ctrl)" | grep -q "fr3_arm_controller=active"; then   # the switch can fail (e.g. libfranka: brakes closed): never arm the planner against an inactive controller
      $LAB 'docker exec fr3-ros2 bash -c "for p in /proc/[0-9]*; do c=\$(tr \"\\0\" \" \" < \$p/cmdline 2>/dev/null); case \"\$c\" in *curobo_to_jtc*udp*) kill \${p#/proc/};; esac; done"'
      echo "go FAILED: fr3_arm_controller did not activate (see /runs/fr3-ros2.real.log in fr3-ros2); lab forwarder stopped, MPC left in standby, nothing reached the arm"; exit 1; fi
    if [ "${NOPLAN:-0}" = 1 ]; then echo "NOPLAN=1: planner stays in standby (the arm path is live for a scripted replay, home_run.sh)"; else mset True; fi ;;
  dry)    # whole path with the ARM DISCONNECTED: MPC active on the real joint states + live map, goal path, then standby. Refuses if anything could reach the arm.
    [ "$(state)" = standby ] || { echo "refused: MPC not in standby (up first)"; exit 1; }
    c=$(ctrl); echo "controllers: $c"
    echo "$c" | grep -q "fr3_arm_controller=active" && { echo "refused: fr3_arm_controller is ACTIVE (arm path live)"; exit 1; }
    [ -n "$($LAB 'docker exec fr3-ros2 bash -c "ps -eo pid,args | grep curobo_to_jtc | grep -v grep | grep -v bash"' | head -1)" ] && { echo "refused: a curobo_to_jtc forwarder is running in the driver (arm path may be live)"; exit 1; }
    mset True; "$0" traj; mset False; echo "dry run done; MPC $(state)" ;;
  traj)   # goal path for the VM planner. ARM MOVES if MPC is active and the arm path is live.
    [ "$(state)" = active ] || { echo "refused: MPC not active (go first)"; exit 1; }
    ID=$(date +%Y%m%d_%H%M%S); OUT=$L/mpc/traj_$ID; mkdir -p $OUT
    WPS="${TRAJ_WPS:-0,0.10,0,3,2;0,0.20,0,3,2;0,0.20,0.12,3,2;0,-0.20,0.12,8,2;0,-0.20,0,3,2;0,0,0,5,2;0.15,0,0,3.5,3;0,0,0,3.5,5}"
    echo "traj $ID -> $OUT"
    docker exec -e ROS_DOMAIN_ID=143 -e ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET -e FASTDDS_BUILTIN_TRANSPORTS=UDPv4 -e RMW_IMPLEMENTATION=rmw_fastrtps_cpp -e LD_LIBRARY_PATH=$J/lib:/nvm/lib -e PYTHONPATH="$PY" -e PYTHONUNBUFFERED=1 vm-mpc /isaac-sim/python.sh /goal/vm_goal_path.py --out /out/traj_$ID --hold 10 --wps "$WPS" 2>&1 | tail -4 ;;
  run)    # single-command deploy. ARM MOVES. Needs ESTOP=1 (a person at the e-stop).
    [ "${ESTOP:-0}" = 1 ] || { echo "refused: set ESTOP=1 (person at the e-stop)"; exit 1; }
    read -r -p "Type MOVE to run the trajectory on the real arm: " A; [ "$A" = MOVE ] || { echo "not confirmed"; exit 1; }
    # go exits 1 when it REFUSES (nothing started, nothing touched): then do NOT run back. Any other exit means something may be running: always release.
    trap 'echo "interrupted: releasing the arm"; trap - INT TERM; "$0" back; exit 130' INT TERM
    "$0" go; gorc=$?
    if [ $gorc = 1 ]; then echo "go refused: nothing was started, nothing to release"; exit 1; fi
    [ $gorc = 0 ] && "$0" traj
    "$0" back ;;
  back)   # controllers first (release the arm), then MPC standby (mset waits ~6 s for discovery)
    if echo "$(ctrl)" | grep -q "fr3_arm_controller=active" && [ "${RESTORE_CART:-0}" = 1 ] && echo "$(ctrl)" | grep -q "cartesian_impedance_target_controller=inactive"; then
      # atomic swap back to the Cartesian impedance controller (its on_activate holds the CURRENT pose until a target arrives): the arm is never without a controller
      drv "timeout 10 ros2 control switch_controllers --deactivate fr3_arm_controller --activate cartesian_impedance_target_controller --strict" 2>&1 | tail -1
    elif echo "$(ctrl)" | grep -q "fr3_arm_controller"; then drv "timeout 10 ros2 control switch_controllers --deactivate fr3_arm_controller --strict" 2>&1 | tail -1; else echo "back: no fr3_arm_controller loaded, nothing to release"; fi
    mset False
    $LAB 'docker exec fr3-ros2 bash -c "for p in /proc/[0-9]*; do c=\$(tr \"\\0\" \" \" < \$p/cmdline 2>/dev/null); case \"\$c\" in *curobo_to_jtc*udp*) kill \${p#/proc/};; esac; done"'
    echo "controllers: $(ctrl)" ;;
  down) docker rm -f vm-mpc vm-relay ;;
  *) sed -n 2,11p "$0" ;;
esac
