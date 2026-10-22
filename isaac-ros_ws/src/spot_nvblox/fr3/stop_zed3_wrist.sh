#!/usr/bin/env bash
# Host side: stop what start_zed3_wrist.sh started - by PID only (pid files + the exact module names), nothing else.
# Never touches /zed1, /zed2, /zed3 wrapper, /nvblox_node, sam2/tf_projection.   DRY=1 only lists the PIDs.
set -u
C=${CONTAINER:-fr3-curobo}; ZC=zed-build
own() { docker exec $C bash -c "ps -eo pid,args | grep -E 'operator_hand_tracker\.py --domain 8|arm_pose_estimator\.(wrist_detector|hand_orientation_estimator)|launch .*fr3_zed3_wrist\.launch\.py' | grep -v grep | awk '{print \$1}'"; }
P=$(own); echo "wrist processes in $C: ${P:-none}  (pidfile: $(docker exec $C cat /tmp/zed3_wrist.pid 2>/dev/null || echo none))"
RP=$(docker exec "$C" pgrep -f '^python3 /home/spot-teleop/isaac-ros_ws/src/spot_nvblox/fr3/teleop/zed_input_relay.py' || true)
echo "input relay processes in $C: ${RP:-none}"
P=$(echo "$P $RP" | tr '\n' ' ')
TP=$(docker exec $ZC bash -c 'cat /tmp/zed3_tf.pid 2>/dev/null'); echo "zed3 static-TF pid in $ZC: ${TP:-none}"
[ "${DRY:-0}" = 1 ] && { echo "DRY: nothing stopped"; exit 0; }
if [ -n "$P" ]; then docker exec $C bash -c "for p in $(echo $P); do kill -INT \$p 2>/dev/null; done; sleep 3; for p in $(echo $P); do kill -0 \$p 2>/dev/null && kill -TERM \$p; done; sleep 1; for p in $(echo $P); do kill -0 \$p 2>/dev/null && kill -KILL \$p; done; rm -f /tmp/zed3_wrist.pid"; fi
[ -n "$TP" ] && docker exec $ZC bash -c "kill -INT $TP 2>/dev/null; sleep 1; kill -0 $TP 2>/dev/null && kill -TERM $TP; rm -f /tmp/zed3_tf.pid"
echo "left in $C: $(own | tr '\n' ' ')"
