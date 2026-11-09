# Dashboard recovery button — DEPLOYED 2026-11-04

- Backend: `fr3_web_dashboard.py` (fr3-ros2 container) gained a `recover`
  zed-control action: error recovery -> hardware-interface activation ->
  joint-controller strict switch (already-active tolerated via bridge
  status check) -> bridge enable. Refuses while engaged; moves nothing.
  Backup: `fr3_web_dashboard.py.bak_1104_recover`.
- Frontend: `lab_dashboard.html` (isaac-build `/tmp/nvblox_real/`,
  backup `.bak_1104_recoverbtn`) gained a "Recover arm + bridge" button
  posting the recover action; result lands in the existing note line.
  No mesh_server change (zed-control already piped to :8089).
- Proven end-to-end from the lab host (same path the button uses):
  `{"ok": true, "message": "recovered: ..."}` with the bridge landing
  armed + controller-ready.

# Auto-rearm patch — DEPLOYED 2026-11-04 to the live bridge

Target: lab `fr3-ros2` container,
`/lab/fr3_zed_runtime/curobo_live_noguards/joystick_bridge.py`
(hardware mode, guards-down build owned by lane e64db8e7).
Deployed with Murilo's go. Backup on the box:
`joystick_bridge.py.bak_1104_autorearm`.

## What is live now (verified read-only after deploy)

- `:8784/status` carries `fault_disarm` and the new reason strings.
- `bridge_rearm.py` (pure gate + poll table) lives next to the bridge and
  is imported by it; covered by 26 unittest checks in this directory.
- `maybe_auto_rearm`: re-arms ONLY on a fresh Start press with controller
  ownership + valid planner packet; fault (ownership-loss) disarms still
  require manual `/fr3_curobo/enable`. Arming never moves the robot — the
  unchanged gate chain below still has to pass on the same tick.
- `poll_decision` + 2 s retry in `poll_controllers`: a stuck in-flight
  `list_controllers` call can no longer wedge discovery forever
  (found live: fresh bridge showed `controllers: {}` for minutes while a
  twin probe matched in 1 s; after the fix the bridge holds controllers
  at ~150 ms age on its own).
- Generation guard: a late/abandoned poll result that arrives after a
  newer poll started is ignored instead of overwriting fresh state.

## Recovery performed the same day (movement-free steps)

1. `/action_server/error_recovery` -> SUCCEEDED (robot error state).
2. `set_hardware_component_state` FrankaHardwareInterface -> active
   (it was lifecycle-inactive, which is why the JTC switch was rejected).
3. `switch_controller` activate `fr3_arm_controller` (strict) -> ok.
4. `/fr3_curobo/enable` true -> "MPC ready; press joystick Start".
5. Verified: armed, controller+mode+command ready, JTC active,
   teleop joystick + disengaged. Final proof = operator presses Start.

## Operator-visible change

After any future dashboard ZED round-trip that disarms the bridge, there
is no more lab-shell step for the benign case: press Start on the pad and
the joystick drives (gates permitting). If the disarm came from a real
controller-ownership fault, the bridge stays down and says
"manual enable required" — that path is deliberate.
