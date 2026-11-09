#!/bin/bash
# FR3 collision-reflex thresholds at EVERY driver start (2026-11-05, Murilo: "make it the default").
# Run INSIDE fr3-ros2 by collision_watch.sh (murilo crontab, every minute, once per container start), so every new
# container gets them, however it was started. The driver itself sets none (franka_hardware only
# via the service; the firmware default is 20 N / 20 Nm and comes back on every driver restart).
# Values = fr3_ros2.sh real-collision = Oopsie fr3_bringup/config/motion.yaml. Override: COLL_TQ / COLL_F.
# Result (applied / FAILED) is appended to /runs/collision_apply.log (= host /home/murilo/franka_ws/runs).
TQ=${COLL_TQ:-[40.0,40.0,36.0,36.0,16.0,14.0,12.0]}; FF=${COLL_F:-[50.0,50.0,50.0,30.0,30.0,30.0]}
L=/runs/collision_apply.log
log(){ echo "$(date -u +%FT%TZ) [${COLL_TAG:-manual}] $*" >> "$L"; }
source /opt/ros/jazzy/setup.bash; [ -f /ros2_ws/install/setup.bash ] && source /ros2_ws/install/setup.bash
SVC=/service_server/set_full_collision_behavior
REQ="{lower_torque_thresholds_acceleration: $TQ, upper_torque_thresholds_acceleration: $TQ, lower_torque_thresholds_nominal: $TQ, upper_torque_thresholds_nominal: $TQ, lower_force_thresholds_acceleration: $FF, upper_force_thresholds_acceleration: $FF, lower_force_thresholds_nominal: $FF, upper_force_thresholds_nominal: $FF}"
log "waiting for $SVC"
for i in $(seq 1 90); do ros2 service list 2>/dev/null | grep -qx "$SVC" && break; sleep 2; done
out="service never appeared (180 s)"
for try in 1 2 3 4 5; do
  ros2 service list 2>/dev/null | grep -qx "$SVC" || { sleep 3; continue; }
  out=$(timeout 20 ros2 service call "$SVC" franka_msgs/srv/SetFullCollisionBehavior "$REQ" 2>&1 | tr -d '\n')
  echo "$out" | grep -q "success=True" && { log "applied (torque $TQ, force $FF), try $try"; exit 0; }
  sleep 3
done
log "FAILED: the arm keeps the 20 N / 20 Nm default: ${out: -200}"; exit 1
