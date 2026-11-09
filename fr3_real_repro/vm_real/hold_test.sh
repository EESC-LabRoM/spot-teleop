#!/usr/bin/env bash
# Guarded hold test: ./hold_test.sh [seconds=25]. Needs ESTOP=1 and `up` done. Aborts (kills the MPC, then back) if any joint drifts > LIM (0.015 rad) from the start pose.
H=$(cd "$(dirname "$0")" && pwd); S=${1:-25}; rm -f $H/log/relay/TRIP
J=/isaac-sim/exts/isaacsim.ros2.core/jazzy
docker exec -d -e LIM=${LIM:-0.015} -e ROS_DOMAIN_ID=143 -e ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET -e FASTDDS_BUILTIN_TRANSPORTS=UDPv4 -e RMW_IMPLEMENTATION=rmw_fastrtps_cpp -e LD_LIBRARY_PATH=$J/lib -e PYTHONPATH=$J/rclpy:/isaac-sim/exts/omni.pip.compute/pip_prebundle vm-relay /isaac-sim/python.sh /h/guard.py
sleep 4
"$H/vm_real.sh" go > $H/log/go.out 2>&1 &
GP=$!; T0=$(date +%s); tripped=0
while [ $(( $(date +%s) - T0 )) -lt $((S+12)) ]; do
  if [ -e $H/log/relay/TRIP ]; then docker kill vm-mpc >/dev/null 2>&1; docker exec vm-relay pkill -f curobo_to_jtc >/dev/null 2>&1; tripped=1; echo "TRIP: $(cat $H/log/relay/TRIP) rad -> MPC killed"; break; fi
  grep -q "set_active True" $H/log/go.out 2>/dev/null && [ -z "${ACT:-}" ] && ACT=$(date +%s) && echo "MPC active"
  [ -n "${ACT:-}" ] && [ $(( $(date +%s) - ACT )) -ge $S ] && break; sleep 0.05; done
wait $GP; grc=$?; cat $H/log/go.out
[ $grc -ne 0 ] && { echo "go refused/failed: NOT running back (controllers untouched)"; docker exec vm-relay pkill -f guard.py; exit 1; }
"$H/vm_real.sh" back; docker exec vm-relay pkill -f guard.py
echo "guard max drift over run: $(tail -1 $H/log/relay/guard.csv)  tripped=$tripped"; sort -t, -k2 -n $H/log/relay/guard.csv | tail -1
