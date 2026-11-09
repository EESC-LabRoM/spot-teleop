#!/usr/bin/env bash
# Focused regression test for vm_real.sh `go` controller soft-switching.
# Stubs out ssh/docker/the lab driver with fakes, so nothing moves and the
# lab PC is never touched: the script under test runs from a hermetic copy
# (byte-compared with the repo file before each case).
#
#   bash tests/test_go_controllers.sh [path/to/vm_real.sh]
#
# Cases: both-inactive auto-recovers (SW=3) and reaches JTC active; the
# SW=0/SW=1 paths still work; unknown states still refuse with exit 1 and
# never start the lab forwarder; a failed hold-restore refuses before the
# forwarder starts.
set -u
HERE=$(cd "$(dirname "$0")" && pwd)
UNDER_TEST=${1:-$HERE/../vm_real.sh}

pass=0; fail=0
ok() { pass=$((pass + 1)); echo "  PASS: $1"; }
no() { fail=$((fail + 1)); echo "  FAIL: $1"; }

# --- hermetic stage -------------------------------------------------------
T=$(mktemp -d)
export T
trap 'rm -rf "$T"' EXIT
mkdir -p "$T/bin" "$T/log/mpc" "$T/log/relay"
cp "$UNDER_TEST" "$T/vm_real.sh"
cmp -s "$UNDER_TEST" "$T/vm_real.sh" || { echo "FAIL: test copy differs from $UNDER_TEST"; exit 1; }
echo "MPC state: standby" > "$T/log/mpc/mpc_node.csv"
echo "relay last 10 s: joint_states 150 msgs" > "$T/log/relay/relay_out.log"

cat > "$T/bin/docker" <<'EOF'
#!/usr/bin/env bash
if [ "${1:-}" = ps ]; then printf 'vm-mpc\nvm-relay\n'; exit 0; fi
echo "fake-docker: unexpected call: $*" >&2; exit 1
EOF
cat > "$T/bin/ssh" <<'EOF'
#!/usr/bin/env bash
# Fake lab driver. State in $T/labstate as "JTC=<s> CART=<s>" (s: active|inactive|missing).
echo "SSH: $*" >> "$T/calls.log"
state=$(cat "$T/labstate"); jtc=$(echo "$state" | grep -o 'JTC=[a-z]*' | cut -d= -f2); cart=$(echo "$state" | grep -o 'CART=[a-z]*' | cut -d= -f2)
cmd="$*"
case "$cmd" in
  *list_controllers*) # byte-faithful to the real driver: cyan names, cyan inactive / green active
    if [ "$jtc" != missing ]; then [ "$jtc" = active ] && sc=92 || sc=96
      printf '\033[96mfr3_arm_controller                   \033[0m joint_trajectory_controller/JointTrajectoryController       \033[%sm%s\033[0m\n' "$sc" "$jtc"; fi
    if [ "$cart" != missing ]; then [ "$cart" = active ] && sc=92 || sc=96
      printf '\033[96mcartesian_impedance_target_controller\033[0m fr3_lab_controllers/CartesianImpedanceTargetController      \033[%sm%s\033[0m\n' "$sc" "$cart"; fi
    printf '\033[92mfranka_robot_state_broadcaster       \033[0m franka_robot_state_broadcaster/FrankaRobotStateBroadcaster  \033[92mactive  \033[0m\n'
    printf '\033[92mjoint_state_broadcaster              \033[0m joint_state_broadcaster/JointStateBroadcaster               \033[92mactive  \033[0m\n' ;;
  *switch_controllers*)
    if echo "$cmd" | grep -q -- '--activate fr3_arm_controller'; then
      echo "JTC=active CART=inactive" > "$T/labstate"; echo "activated fr3_arm_controller"
    elif echo "$cmd" | grep -q -- '--activate cartesian_impedance_target_controller'; then
      if [ "${FAIL_RESTORE:-0}" = 1 ]; then echo "FAILED to activate (hardware unconfigured)"
      else echo "JTC=$jtc CART=active" > "$T/labstate"; echo "activated cartesian hold"; fi
    else echo "switch ok"; fi ;;
  *robot_mode*) echo "2" ;;
  *curl*|*"/proc/"*) : ;;
  *teleop/disengage*) echo "success=True" ;;
  *curobo_to_jtc.py*--in\ udp*) echo "forwarder started" ;;
  *) : ;;
esac
exit 0
EOF
chmod +x "$T/bin/docker" "$T/bin/ssh"

run_go() { # $1=JTC $2=CART -> sets state, runs go, prints exit code; output in $T/out
  echo "JTC=$1 CART=$2" > "$T/labstate"; : > "$T/calls.log"
  ( FAIL_RESTORE=${FAIL_RESTORE:-0} ESTOP=1 BRIDGE_ABSENT_OK=1 NOPLAN=1 MAXACCEL=10 PATH="$T/bin:$PATH" bash "$T/vm_real.sh" go > "$T/out" 2>&1 )
  echo $?
}
final_state() { tr '\n' ' ' < "$T/labstate"; }
fwd_started() { grep -q -- '--in udp' "$T/calls.log"; }
restore_before_fwd() { # restore-activate line must precede forwarder start line
  a=$(grep -n -- '--activate cartesian' "$T/calls.log" | head -1 | cut -d: -f1)
  b=$(grep -n -- '--in udp' "$T/calls.log" | head -1 | cut -d: -f1)
  [ -n "$a" ] && [ -n "$b" ] && [ "$a" -lt "$b" ]
}

echo "under test: $UNDER_TEST"

echo "case 1: both inactive -> auto soft-switch, JTC ends active"
rc=$(run_go inactive inactive)
[ "$rc" = 0 ] && ok "exit 0" || no "exit $rc, want 0"
grep -q "hold restored" "$T/out" && ok "reports hold restored" || no "missing 'hold restored'"
[ "$(final_state)" = "JTC=active CART=inactive " ] && ok "ends JTC=active CART=inactive" || no "ends $(final_state)"
restore_before_fwd && ok "hold restore precedes forwarder start" || no "wrong order (see calls.log)"

echo "case 2: JTC inactive + Cartesian active -> normal swap (SW=1)"
rc=$(run_go inactive active)
[ "$rc" = 0 ] && ok "exit 0" || no "exit $rc, want 0"
grep -q "hold restored" "$T/out" && no "unexpected SW=3 path" || ok "no SW=3 detour"
[ "$(final_state)" = "JTC=active CART=inactive " ] && ok "ends JTC=active CART=inactive" || no "ends $(final_state)"

echo "case 3: JTC already active -> no switch (SW=0)"
rc=$(run_go active inactive)
[ "$rc" = 0 ] && ok "exit 0" || no "exit $rc, want 0"
grep -q "switch_controllers" "$T/calls.log" && no "touched the switches" || ok "no switch call"

echo "case 4: JTC missing -> still refuses, forwarder never starts"
rc=$(run_go missing inactive)
[ "$rc" = 1 ] && ok "exit 1" || no "exit $rc, want 1"
grep -q "refused: controllers" "$T/out" && ok "refuses on controllers" || no "missing refusal"
fwd_started && no "forwarder was started" || ok "forwarder never started"

echo "case 5: hold restore fails -> refuses before forwarder starts"
rc=$(FAIL_RESTORE=1 run_go inactive inactive)
[ "$rc" = 1 ] && ok "exit 1" || no "exit $rc, want 1"
grep -q "did not re-activate" "$T/out" && ok "points at real-recover" || no "missing re-activate refusal"
fwd_started && no "forwarder was started" || ok "forwarder never started"

echo "----"
echo "pass=$pass fail=$fail"
[ "$fail" = 0 ]
