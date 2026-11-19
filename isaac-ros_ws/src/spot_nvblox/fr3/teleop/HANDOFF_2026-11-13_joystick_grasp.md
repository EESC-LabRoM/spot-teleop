# Continue: Franka joystick grasp — 2026-11-13

**User authorized implementation/setup, reusing Murilo’s grasp code. Installed
and running DISENGAGED. No motion, Home, gripper action, controller activation,
recovery, gains or payload changes were issued. Physical validation pending.**

| Control | Installed behavior |
|---|---|
| Start | cuRobo/nvblox joystick; reseed from measured joints and await preview |
| Y | Existing DIRECT-JTC pick/lift; second Y places/releases/retreats/Homes |
| Back | Cancel, explicitly preempt full trajectory; no automatic resume/open |
| A/B; hold X | Manual gripper; existing Home, blocked during automatic grasp |

Y: DInput4/XInput3. **Automatic MPC-executed grasping is not implemented.**

## Session close — 2026-11-14
Completed: reused Murilo's canonical VM planner/stage executor; installed joystick
grasp adapter and dedicated disengaged launcher; 66 isolated tests passed and
Nov13 startup audit observed zero motion commands. No physical grasp, hardware
recovery, controller activation or camera changes by this lane. Nov14 only
coordination and handoff updates; runtime state has NOT been re-audited today.

Next: check current robot owner/resource and runtime state before starting
anything; resolve robot connection/gripper feedback with the operator; connect
joystick; verify live perception and a fresh executable plan. ZED2 was displaced
59 degrees per calibration lane: its TF is invalid; exclude its data and verify
fresh map geometry or recalibrate before using it. Then perform the pending
physical joystick/grasp validation with the operator. Automatic grasp uses direct
JTC; autonomous MPC execution remains unimplemented. Do not recreate the existing
VM grasp pipeline or execute historical plans. Detailed files, commands, tests,
backups and owner/session identifiers follow below. Installed source remains
outside Git, uncommitted; integrate after review without clobbering other work.

## Current state / blockers
- Five-second startup audit:96 status messages,10 engagement=false messages,
  ZERO joystick pose commands, ZERO JTC trajectories. Resource claim released.
- Robot connection LOST: peer verified libfranka UDP timeout + gripper exit-6
  around20:01:46Z. JTC and both state broadcasters inactive; no joint feedback.
  `runs/autorecover.disable` is intentional. Operator/robot-owner recovery is
  separate; do not silently recover/activate hardware. No local gamepad found.
- Real VM planning-only adapter call REFUSED stale object cloud. Need a fresh
  validated candidate; real regrasp loop/target publisher were not running at
  audit. Today’s ZED2 depth fix may cure old fusion flap—retest before assuming
  primary-only workaround is still needed. Historical saved plans are stale.

## Update 2026-11-14 — ZED2 mount invalid (calibration lane measurement)
Calibration lane `88267649b9134913ad60bbb5e1c8538d` reports its static IMU
gravity vector is **59 degrees off the calibrated mount orientation**, confirming
physical camera displacement in that lane's investigation. ZED2 TF is invalid
until re-seated/recalibrated; treat grasp perception as ZED1-only meanwhile.
Measurement supplied by that lane, not independently measured here. This
supersedes the earlier provisional camera-status note below. Do not assume an
existing fused map has excluded misregistered ZED2 data; verify map inputs and
fresh geometry before planning. This lane has made no camera/runtime/robot changes.

## Update 2026-11-14 15:53Z — perception needs recheck
RAL calibration lane `88267649b9134913ad60bbb5e1c8538d` reports ZED2 raw
depth ~0% valid, SAM2 mask 0%, RGB AprilTag close-up/no drill; ZED1 OK.
Peer-reported, not independently verified by this lane. Supersedes Nov13's
“ZED2 depth restored” assumption: do not assume two-camera grasp coverage.
Recheck camera placement, depth, segmentation and TF before another grasp test;
if physically moved, calibration must be checked. This lane did not touch ZED2
or its TF. No new robot commands, runtime changes or physical tests.

## Files / startup / tests
Runtime host `/home/murilo/franka_ws/lab/fr3_zed_runtime/`, mounted as
`fr3-ros2:/lab/fr3_zed_runtime/`:
`fr3_input_teleop.py`, `joystick_grasp.py`, `grasp_broker.py`,
`joystick_grasp.yaml`, `start_joystick_grasp.sh`, `vendor/grasp_stage_exec.py`.
Guide `JOYSTICK_GRASP.md`; backup+SHA manifest
`joystick_grasp_backups/20261113_202218/`.

Restart: `bash /home/murilo/franka_ws/lab/fr3_zed_runtime/start_joystick_grasp.sh`
(refuses duplicate input; never prepares/activates controllers).
Config:joystick,fr3_curobo,direct_mpc=true,ZED UDP47983 separate from relay47813.
Host-only SSH planning broker uses Unix socket `grasp.sock`; no key copied into
container. Normal runtime `backend` remains cartesian; use dedicated launcher.
Logs host `../runs/joystick_grasp_{teleop,broker}.log`; plans/stage logs in
`../runs/joystick_grasp/`. Status `/teleop/grasp_status`; `/teleop/grasp` and
`/teleop/grasp_cancel` are motion-capable services, not read-only probes.

**66 isolated tests passed:**43 existing +19 grasp/input (full mocked pick/place)
+4 lock-order tests; DDS191/193, all actions/subprocesses mocked. Log in
`fr3-ros2:/tmp/joystick_grasp_tests.log`; tests `/tmp/joystick_grasp_test/`.
Source/tests/installer staged `/home/hipolito/franka_teleop_checks/joystick_grasp/`.
**Not committed. Installed code is outside Git.** Integrate after review; do not
clobber the already-dirty tower/VM repos. Full handoff in staged `HANDOFF.md`.

Implementation:broker imports canonical VM planner and validates selected pose
against fresh cloud (<=2s,>=300pts,<=3cm XY/top drift) and current mesh. Never
execute dashboard rendering-only shifted/yawed pick. Existing stage executor
copied with subscriber wait>=2 discovery fix. Back kills child AND publishes
measured hold. Ordered owner-scoped RUN_ACTIVE heartbeat; HELD_OBJECT survives
cancellation. Teleop fixes completed JTC polling future, initial button edges,
disconnect cancellation and shutdown before ROS context destruction.

## Existing work / identifiers (already investigated—do not redo)
VM ubuntu-gpu:nexus@100.84.207.115; SSH host key `~/.ssh/id_ed25519`,
HostKeyAlias=omnigent-vm, host known_hosts. The separate
id_ed25519_omnigent_vm identity is restricted to log streaming.
Canonical `/home/nexus/spot-teleop/fr3_real_repro/grasp_loop/`:
`grasp_test.sh`, `grasp_plan.py/lib`, `grasp_stage_exec.py`, `grasp_gripper.sh`.
Nov12 two successful physical direct-JTC S1–S6 cycles; original commands/results
verified in VM chat.db conversation **c7730019277146f7acac96446ab49054**,
positions1181–1215: .0303/.0287/.0401/.0413(retry)/.0360/.0359rad;
close SUCCEEDED,49mm holding; loaded hold TCP p2p<=.86mm.
VM ROOT docs `HANDOFF_2026-11-12_real-grasp.md`, `NOTE_FOR_FRANKA_TELEOP.md`;
commits4d8452f/79f96d8. Historical run20261112_210720_drill_offline_regen,
plan `grasp_loop/runs_real/grasp_test_20261112_210732/plan.json`.
Older missing `/tmp/mpc_grasp_drive.py` is NOT needed for this proven path.
Discovery audit `/home/hipolito/franka_teleop_checks/GRASP_CURRENT_STATE_2026-11-13.md`.

Peer **7e1c8f9e30374af8ba48a82396d54b68** knows installation/state. Their
recording hold was released before deployment; tracker/relay stopped. Leave
ZED3 wrapper to camera/GPU owner RAL a36b1ab1. Original owner from handoff:
01a1d173-60ac-7000-b3d8-c27b701ce9ea. `sys_duet_say` uses **to + text**;
`to_session_id` ignored, `sys_session_send` child-only. Resource tools available
through ALL_TOOLS. Do not spawn agents unless authorized.
