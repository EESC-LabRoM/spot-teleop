#!/usr/bin/env bash
# soft_switch.sh: automatic soft-switch recovery for the FR3 driver (lab PC host).
# Restores the cartesian_impedance_target_controller hold when NO motion controller
# is active (the stranded both-inactive state, e.g. after a failed swap). It moves
# nothing: it only re-activates the holding controller, then the normal flows work
# again (VM home_run.sh, or ./fr3_ros2.sh real-home). Refuses if fr3_arm_controller
# is active (a move may be live), if robot_mode is not 1/2 (Desk: clear the error,
# unlock the joints, Activate FCI), or if the hold does not re-activate
# (run ./fr3_ros2.sh real-recover).
#
# Install: copy next to fr3_ros2.sh on the lab PC (~/franka_ws/lab/soft_switch.sh).
#   ESTOP=1 EP="<name of the person at the e-stop>" ./soft_switch.sh
#   DRY=1 ./soft_switch.sh   (checks only, changes nothing, no ESTOP needed)
set -uo pipefail
NAME=${NAME:-fr3-ros2}
SRC="source /opt/ros/jazzy/setup.bash; source /ros2_ws/install/setup.bash"
die() { echo "REFUSED: $*"; exit 1; }
[ "${DRY:-0}" = 1 ] || { [ "${ESTOP:-0}" = 1 ] && [ -n "${EP:-}" ] || die "set ESTOP=1 and EP=<name of the person at the e-stop>"; }
docker ps --format '{{.Names}}' | grep -qx "$NAME" || die "driver container $NAME is not running"
m=$(docker exec "$NAME" bash -c "$SRC; timeout 8 ros2 topic echo --once --field robot_mode /franka_robot_state_broadcaster/robot_state" 2>/dev/null | grep -o -E '^[0-9]+' | head -1)
echo "robot_mode: ${m:-?} (need 1 idle or 2 move)"
case "${m:-}" in 1|2) ;; *) die "robot_mode ${m:-unknown}: in Desk clear the error, unlock the joints, Activate FCI, then retry";; esac
c=$(docker exec "$NAME" bash -c "$SRC; timeout 10 ros2 control list_controllers 2>/dev/null" | sed 's/\x1b\[[0-9;]*m//g' | awk '{print $1"="$NF}' | tr '\n' ' ')
echo "controllers: $c"
echo "$c" | grep -q "fr3_arm_controller=active" && die "fr3_arm_controller is ACTIVE (a move may be live): not touching anything"
echo "$c" | grep -q "cartesian_impedance_target_controller=active" && { echo "hold already active: nothing changed"; exit 0; }
echo "$c" | grep -q "cartesian_impedance_target_controller=inactive" || die "cartesian controller is not loaded inactive: run ./fr3_ros2.sh real-recover"
[ "${DRY:-0}" = 1 ] && { echo "DRY: would disengage teleop and activate the Cartesian hold; nothing switched"; exit 0; }
docker exec "$NAME" bash -c "$SRC; timeout 20 ros2 service call /teleop/disengage std_srvs/srv/Trigger >/dev/null 2>&1"
docker exec "$NAME" bash -c "$SRC; timeout 10 ros2 control switch_controllers --activate cartesian_impedance_target_controller --strict" 2>&1 | tail -1
c2=$(docker exec "$NAME" bash -c "$SRC; timeout 10 ros2 control list_controllers 2>/dev/null" | sed 's/\x1b\[[0-9;]*m//g' | awk '{print $1"="$NF}' | tr '\n' ' ')
echo "controllers: $c2"
echo "$c2" | grep -q "cartesian_impedance_target_controller=active" || die "the hold did not re-activate: run ./fr3_ros2.sh real-recover (Desk: clear error, unlock joints, Activate FCI)"
echo "hold restored: cartesian_impedance_target_controller active, arm held in place"
echo "next: re-run the VM ./home_run.sh, or on lab ./fr3_ros2.sh real-home"
