# Clean teleop source switch: joystick <-> ZED wrist target (cuRobo)

Offline candidate (this workspace only). Nothing here touches the live
robot, bridge, MPC, map, or dashboard. Deploy needs Murilo's go +
live-stack handoff (lane e64db8e7, holds franka-robot + bridge :8784 + MPC).

## What already exists live (verified read-only)

- `joystick_bridge.py` (hardware, fr3-ros2, domain 77) gates every goal on
  latched `/teleop/input_source` (`joystick`|`zed`) + `/teleop/engaged` +
  ZED-active phase. No bridge restart needed for a switch.
- `/fr3_joy_teleop` accepts runtime `input_source` param changes
  (auto-disengage + zed_anchor reset + latched republish).
- ZED activation is NOT a param write: ROS service `/teleop/zed_start`
  (Trigger) runs `start_zed()` = sets `input_source=zed` AND sends UDP
  `{"action":"calibrate_arm"}` to 127.0.0.1:47815. Param-only switch to zed
  leaves the tracker uncalibrated -> bridge correctly holds (`ZED paused`).

## Files here

- `source_switch.py` — pure gate/verify logic + stdlib `SwitchClient`
  for the bridge `:8784` API (`GET /status`, `POST /source` candidate).
- `bridge_source_endpoint.py` — pure `handle_source_post()` to wire into
  lab `joystick_bridge.py`: zed via `/teleop/zed_start` Trigger, joystick
  via param set, engaged -> 409, failures fail closed (502, old source kept).
- `dashboard_source_toggle.js` — isolated `<script type="module">` block
  for the live dashboard: Joystick / ZED-wrist buttons + status line,
  polls `:8784/status`, POSTs `:8784/source`. Own element id
  (`source-toggle`); touches nothing else on the page.
- `tests/test_source_switch.py` — 15 unittest checks, no ROS.
  Run: `python3 -m unittest discover -s tests -v` (+ `node --check` the JS).

## Deploy sketch (needs Murilo's go + e64 handoff + dashboard owner)

1. Lab `joystick_bridge.py`: add POST `/source` route using
   `handle_source_post()` + Trigger/param clients; add `source` to
   `/status` JSON; keep CORS `*`. No controller/guard changes.
2. Verify from lab: `curl :8784/status`; POST zed while disengaged ->
   `source=zed`, reason `ZED paused` until calibration active; POST
   joystick -> back. Re-engage with Start only after reason leaves hold.
3. Dashboard: paste the JS block, `node --check`, mirror sync + `cmp`,
   `curl` page 200. Confirm button placement with the dashboard owner.
4. Live test: disengaged switch -> calibrate (ZED wrist) -> Start ->
   cuRobo follows wrist; switch back disengages first by construction.

## Open items (do NOT guess live)

- "ZED3": no third physical ZED known to the live lanes; confirm with
  Murilo what wrist-tracking hardware/feeds the zed socket before use.
- `zed_input_relay.py:47817` as possible tracker front (unverified lead).
- Tracking jumps / orientation / stop-reacquisition remain unvalidated.

## A/B result 2026-11-04 (direct path wins, bridge retired)
Replayed the 20:05 bridged minute (1.26 m, 263 goals) through the direct
path (MPC 77 -> engaged-gated forwarder -> JTC), real-time timing.
- pos RMSE: direct 0.64 deg vs bridged 0.91 deg (per-joint all lower-or-equal except j2)
- max abs err: direct 3.05 deg (single instant mid-play) vs bridged 2.16 deg
- dropouts >100 ms: 0 both; lag: 0 both; faults/disarms during direct: 0
- velocity RMSE not comparable (30 Hz trace vs 6 Hz MPC log); the visible
  staircase is MPC preview stepping, identical node/params both eras,
  bridge slew (4 rad/s) never bound at these speeds.
Retired: bridge code archived off the import path (lab retired_1104_direct/,
VM retired_1104_direct/). Restore = move back + restart process.
