#!/usr/bin/env bash
# FR3 joystick -> cuRobo, faster: MPC config vcap1.2 (1.2 rad/s joints, same 10 rad/s^2), guard limits matched (1.2 rad/s, step 1.2*dt),
# teleop target allowed to lead the arm by 0.20 m (was 0.06) with a 0.25 m leash (was 0.08). Run with someone at the e-stop.
# Every intermediate state is safer than the running one: the bridge restarts DISARMED, the teleop restarts DISENGAGED,
# no MPC = no command. It re-arms at the end only if the bridge reports command_ready. Then press Start on the pad.
set -uo pipefail
HERE=$(cd "$(dirname "$0")" && pwd)
X(){ docker exec fr3-ros2 bash -c "source /opt/ros/jazzy/setup.bash; source /ros2_ws/install/setup.bash; export ROS_DOMAIN_ID=77 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST PYTHONPATH=/lab:\$PYTHONPATH; cd /ros2_ws; $1"; }
ENABLE='import sys,rclpy
from std_srvs.srv import SetBool
rclpy.init(domain_id=77); n=rclpy.create_node("go_fast_enable"); c=n.create_client(SetBool,"/fr3_curobo/enable"); c.wait_for_service(timeout_sec=8)
f=c.call_async(SetBool.Request(data=sys.argv[1]=="on")); rclpy.spin_until_future_complete(n,f,timeout_sec=8); print(f.result().message)'
docker exec -i fr3-ros2 bash -c "cat > /tmp/go_fast_enable.py" <<< "$ENABLE"

echo "[1/5] disarm the bridge"; X "python3 /tmp/go_fast_enable.py off"
echo "[2/5] bridge with guard 1.2 rad/s"
docker cp "$HERE/command_guard.py" fr3-ros2:/lab/fr3_zed_runtime/curobo/command_guard.py
X 'P=$(pgrep -f "^python3 -u /lab/fr3_zed_runtime/curobo/joystick_bridge.py hardware"); [ -n "$P" ] && kill -INT $P; sleep 2; [ -n "$P" ] && kill -9 $P 2>/dev/null; (setsid nohup python3 -u /lab/fr3_zed_runtime/curobo/joystick_bridge.py hardware > /runs/fr3-curobo-joystick-bridge.log 2>&1 < /dev/null &); sleep 3'
echo "[3/5] teleop: lead 0.20 m, leash 0.25 m (cuRobo-routed, starts disengaged)"
X 'P=$(pgrep -f "^python3 /lab/fr3_zed_runtime/fr3_input_teleop_nobox.py"); A=$(tr "\0" " " < /proc/$P/cmdline | sed -e "s#^python3 /lab/fr3_zed_runtime/fr3_input_teleop_nobox.py ##" -e "s#-p max_lead:=[^ ]*##g" -e "s#-p max_error:=[^ ]*##g"); kill -INT $P; sleep 2; kill -9 $P 2>/dev/null; (setsid nohup python3 /lab/fr3_zed_runtime/fr3_input_teleop_nobox.py $A -p max_lead:=0.20 -p max_error:=0.25 >> /runs/fr3-ros2.teleop.log 2>&1 < /dev/null &); sleep 4'
echo "[4/5] cuRobo MPC with vcap1.2 (needs franka-mpc-real as the container template for ~1 min)"
docker rm -f fr3-joystick-mpc >/dev/null 2>&1
( cd /home/murilo/nvblox_real/isaac-sim_ws/franka_mpc/real && HUB=/home/murilo/franka_ws/hub-curobo bash ./run_mpc_real.sh >/dev/null 2>&1 )
python3 "$HERE/run_joystick_mpc.py"; docker stop franka-mpc-real >/dev/null 2>&1
L="$HERE/joystick_logs/stdout.log"
for i in $(seq 1 60); do grep -aq "Step " "$L" 2>/dev/null && break; sleep 3; done
echo "[5/5] arm when the planner is ready"
for i in $(seq 1 30); do S=$(curl -s --max-time 2 http://127.0.0.1:8784/status); echo "$S" | grep -q '"command_ready": true' && break; sleep 2; done
echo "bridge: $S"
echo "$S" | grep -q '"command_ready": true' && X "python3 /tmp/go_fast_enable.py on" || echo "NOT armed: planner not ready (see $L)"
echo "done: press Start on the pad (Back disengages)."
