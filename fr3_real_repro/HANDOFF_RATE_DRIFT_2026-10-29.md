# FR3 cuRobo MPC: 50 Hz and joystick-mode drift (handoff, 2026-10-29)

## Goal
Get the cuRobo MPC node (`curobo_mpc_node.py`) to hold 50 Hz on the real FR3 (Hipolito's lab PC, 2x A2000) and run joystick mode with
avoidance; Murilo hands it to Hipolito afterwards. Real runs had 25-30 Hz and the arm "stopped early, then hit things".

## Decided (by the user)
- EventsExecutor goes into the lab node (done, `MPC_EXECUTOR=events`; default unchanged).
- Speed limits for joystick mode: "same as the impedance controller" -> NOT changed, deferred until the drift is fixed; ask which config.
- Lab PC is read-only unless the user says go; the arm moves only with a person at the e-stop (`run`/`joy` of real_plan_test.sh).

## Measured (delivered Hz = steps / wall time; lab load avg ~70)
- Default executor + live map, arm still: **27 Hz** (loop 28/p95 64 ms). Events + live map: **49.3 Hz** (16/24 ms). Events, arm moving +5 cm: 49.2 Hz.
- Sim: cuRobo step 6 ms alone, 14 ms with Isaac+3 ZED+nvblox on one GPU; two GPUs fix that. CPU saturation alone does NOT reproduce the lab slowdown.
- Hipolito 10-23 real run (nvblox on, default executor): 22.8 Hz. 10-27 no-map: 33 Hz. Driver /joint_states is ~100 Hz (not 1 kHz).
- nvblox_curobo answers usually 12-50 ms, sometimes 1-5 s (async, does not block). Node "occupied voxels" counts UNKNOWN (-1000) space: real obstacle voxels ~2.3k of 23.9k.
- Moving run (`run`, vcap0.8, +5 cm x, goal stream): hand +50.8 mm, err 1.7 mm. Avoidance was never tested (free-space move).

## Joystick-mode drift: cause found in sim, NOT yet fixed on the robot
`joy` with the MPC active and no goal stream: commanded joints drift (real: 16 cm, then 33 mm; command 1->132 mrad in 14 s, pose_err ~0).
Cause: `cspace.retract_config [0,-1.3,0,-2.5,0,1.0,0]` (null_space_weight 1) pulls the arm toward a pose far from the lab ready pose, and the
no-stream hold goal is `Pose(fk.ee_pos_seq,...)` without `.clone()` (follows the hand), so nothing resists. Newton no-goal tests (70 s):
from the retract pose 0 drift; from the ready pose 643 mrad (map irrelevant, executor only changes the speed, the unmodified original
spot-teleop node drifts too: 430 mrad @12 s); null_space_weight 0 -> 0.0; **retract_config = ready pose (one config line) -> 0.0**;
clone alone -> oscillates +-260 mrad; clone + ready-pose retract -> 0.0.

## Open
1. Apply on the lab: `retract_config: [0.0,-0.7853981634,0.0,-2.3561944902,0.0,1.5707963268,0.7853981634,0.04,0.04]` in the lab `node_config/fr3_vcap*.yml`
   (+ clone the FK tensors at node lines ~316/332/1363), then a guarded hold test: run `joy` in the BACKGROUND with `claude_watchdog.py`, abort at 8 mm (my first guard ran `joy` in the foreground and fired 8 s late).
2. Same executor block into Hipolito's `joystick_mpc.py` (line ~1844, still `MultiThreadedExecutor`).
3. Velocity-cap decision; avoidance test with an obstacle; clean-up of the staging (below); GPU 0 neighbours (franka-seg-zed1/2, 3rd ZED) if more margin is needed.
4. Direct cable VM 103 <-> lab (10.0.0.3 <-> 10.0.0.2) is up; MPC on the VM against the real driver is not done (driver domain 77 is localhost-only; needs the UDP relay across the cable).

## Lab PC (murilo@100.88.134.127, key `~/.ssh/id_ed25519_lab_claude`) state left
- Arm on the Cartesian controller at (0.277,-0.016,0.572), ~33 mm from the ready pose (`/teleop/home` sends it back, slow, with guards).
- Containers: `hub-curobo-mpc` (events, live map via proxy, vcap0.8, STANDBY), `hub-curobo-relay`, `claude-esdf-proxy`; helpers `shadow_relay_in.py goals` + `curobo_to_jtc.py` run inside `fr3-ros2`; `fr3_arm_controller` loaded INACTIVE. `nvblox_curobo` (isaac-build, domain 8, GPU 1) was restarted (backup of argv/env: `/tmp/nvblox_curobo.restart_backup.json` inside isaac-build). Put back as found: `cd /home/murilo/claude_plan_run && ./real_plan_test_events.sh stop`.
- Files: `/home/murilo/franka_ws/hub-curobo/node/curobo_mpc_node.py` (patched, backup `.bak_before_events_<ts>`), `log/real_plan_mpc_before_events_<ts>` (old logs), `/home/murilo/claude_plan_run/` = `real_plan_test_events.sh` (prep|run|joy|back|abort|stop; a copy of shadow/real_plan_test.sh with the events env), `mpc_up.sh` (EXEC=events|default USE_ESDF=true|false CFG=), `shadow_run.sh <secs> <outdir>` (node active, arm inert), `esdf_proxy.py` (domain 142 -> live nvblox domain 8), `guarded_joy.sh`, `claude_watchdog.py`, `probe/leak_probe.py`, S1/S2/S3 result dirs.
- Domains: driver 77, MPC/relay 142, nvblox + ZEDs 8. Services: `/curobo_mpc_node/set_active`, `/teleop/home`, `/teleop/disengage`.
- Gotchas: `prep` wipes `log/real_plan` and needs domain 142 empty (remove relay+MPC first) and must not run twice; the driver restart (~1.5 h before) killed the helpers in `fr3-ros2`, the node then waits silently in `warming` for /joint_states; `mpc_up.sh` can print "standby after 1s" from a stale log (check the executor/config lines); `ps` does not exist in `fr3-ros2` (scan /proc); `coll_cost` in the CSV is the TOTAL cost; JTC ignores commands while inactive.

## VM 103 (this machine) files
`isaac-sim_ws/franka_mpc/`: `run_newton_rate.sh` (rate + camera load, G_MPC/G_WRAP/G_NV/G_SIM GPU pinning), `run_newton_nogoal.sh` (drift test: USE_ESDF, SIMSCRIPT=newton_ros_arm_home.py, NODEDIR, NODE_EXTRA, DOCKER_EXTRA), `newton_ros_arm_home.py` (ready-pose start), `node_config/fr3_test_retract_home.yml`, `fr3_test_ns0.yml`, `run_render_trace.sh` + `newton_render_trace.py` (mp4 from trace.npz, `--rtx`), `run_rtx_probe.sh`. Node: `spot-ros2_ws/.../curobo_mpc_node.py` has the `MPC_EXECUTOR` switch. `zed_lite/svo_load.yaml`, `svo_stream.py`. Newton deps: `~/newton_sp` (symlinked as `crawl-lab/.../site-packages`), `~/newton_viewer_sp`, `~/newton_rtx_sp`, `~/newton_usd_sp`. Results in `runs/rate_*`, `split_*`, `cpu_*`, `nogoal_*`. Details of the rate tests: `fr3_real_repro/README.md`.

## Update 10-30 (night): MPC on VM 103 over the cable, staged INERT (not run on the arm)
- `fr3_real_repro/vm_real/vm_real.sh` (up|status|go|back|down). `up` = relay + MPC (lab node copy, events, hybrid, GPU 0, fr3_vcap0.8_acc10 with the ready-pose retract_config) in STANDBY on the VM. Joint states reach the VM at ~100 Hz via `udp_fwd.py` on the lab (127.0.0.1:47801 -> 10.0.0.3). Commands go VM -> 10.0.0.2:47802 (udp_fwd, `--allow 10.0.0.3`) -> `curobo_to_jtc --in udp` started inside fr3-ros2 only by `go`.
- Lab side left running (mine, in /home/murilo/claude_plan_run): `udp_fwd.py` x2 (`fwd_js.log`, `fwd_cmd.log`). Removed my `hub-curobo-relay` container. ssh to the lab stays on Tailscale (the key is `from=` pinned).
- `go` needs ESTOP=1 and REFUSES if `joystick_bridge` runs in the driver or the controllers are not (JTC inactive, Cartesian active). At 10-30 02:30 Hipolito's session was live (fr3-zed-live-mpc container + joystick_bridge, fr3_arm_controller ACTIVE), so go refuses. `go` was never run.
- Untested: `go`/`back` end to end, ESDF over the cable (nvblox is domain 8 on the lab), joy mode with the pad goal stream.

## Update 10-30 (02:40-02:50 UTC / 23:40 -03): first real hold runs from VM 103 (hold_test.sh, ESTOP=1, Hipolito at the e-stop, no goal stream)
- Stack: VM domain 143 (SUBNET over the cable) + lab `vm-esdf-proxy` (domain 143 -> live nvblox domain 8, mine, `claude_plan_run/esdf_proxy143.py`). Domain 143 keeps the VM's /joint_states away from Hipolito's domain 142. ESDF answered in 14 ms over the cable.
- 4 runs of 9-22 s, guard (0.015 rad) never tripped (arm max drift 0.6 mrad). MPC on the VM A2000: loop 7.3 ms (134 Hz compute), mpc.step 5.9 ms, publishes 50 Hz, js_age ~5 ms, pose_err < 1 mm.
- Command still wanders from the measured joints in hold: ~2.5 mrad/s (22-27 mrad after 10 s) with the ready-pose retract_config, vs ~9 mrad/s before. The effort JTC did NOT follow (JTC received ~32 msg/s, cmd 3 mrad off, arm still): so the arm is held by the JTC's own hold, not by the MPC tracking. Open: why no tracking of small offsets; try the clone fix (hybrid plans from its own last command).
- `go` bug found: hold_test ran `back` after a refused `go` and switched Hipolito's active JTC to Cartesian. Fixed. Controllers now: Cartesian active, JTC inactive (his joystick_bridge session needs fr3_arm_controller re-activated).

## Update 2026-10-30 (why Hipolito's live-ZED run felt frozen/slow) — read before testing again
- His stack: `fr3-zed-live-mpc` (his guarded `live_mpc.py`, feedback measured) + `joystick_bridge.py` in fr3-ros2 (disarmed by default). His notes: `/home/hipolito/spot-teleop/isaac-ros_ws/src/spot_nvblox/fr3/teleop/curobo/live_zed/SESSION.md` (read access granted by the user, read-only). His run logs: `.../live_zed/logs/`.
- Not the rate: his loop was 12-17 ms, no step gaps > 44 ms in his motion windows (00:21 and 00:28-00:29 -03).
- The bridge (`command_guard.validate`) is fail-closed: packet age 0.15 s, goal 0.35 s, joints 0.2 s, ESDF 1.5 s, depth 0.7 s, `feasible` must be True, tracking error <= 0.05 rad, joint step/accel limits. It resets `last_sent` after 0.15 s, so one dropped packet does not lock it out. It logs no reasons, only the live `/status` (port 8784).
- ESDF replies stalled ~2 s about 260 times in 4 h (50-74/h): each stall can exceed the 1.5 s ESDF gate.
- My VM node (`vm-mpc`, measured, live map, inert arm, stale hold goal) logged `status=BLOCKED` (= MPC `feasible` False) for 33 % of steps during his 00:28-00:29 motion window, 0 % at a clean hold (pose_err < 0.01), and 100 % from 09:32 -03 on. coll_cost 4000+ when blocked. The node's `min_dist` reads +0.000 on OK steps too (and `sphere#59` with it), so do not use it as evidence.
- CORRECTIONS to my own earlier claims: `publish_count` increments BEFORE the `if not is_feasible: return` (live_mpc.py:1634 vs 1727), so "steps == published" proves nothing; the "~80 % of motion never reached the arm" estimate is unsupported.
- Open hypothesis (not tested): the map is `static_tsdf` with decay 0, so people/objects seen once stay as obstacles; a ZED hand goal lands where the operator's hand is mapped -> infeasible -> bridge blocks. Needs an ESDF snapshot near the arm at a BLOCKED moment. Do not clear the shared map without the user's go.
- Safety: `vm-mpc` was ACTIVE for 13 h with no goal stream; its commands went to a UDP port nobody listened on. On 2026-10-30 12:30 -03 I killed `curobo_to_jtc.py` inside `vm-relay` (by PID) so nothing leaves the VM. `vm_real.sh up` recreates it; `go` needs ESTOP=1.

### Correction (2026-10-30 12:50 -03) — the "infeasible -> bridge blocks" story is NOT supported for Hipolito's run
- cuRobo defines `feasible = (constraint == 0.0)` (curobo/rollout/arm_base.py:414).
- His node logs its step lines AFTER `if not is_feasible: return` (live_mpc.py:1727), so an infeasible step shows up as an absent every-10th-step slot. Absent slots in his whole stdout: 27 of ~163k (0.02 %); his CSV `constraint` is 0 throughout. So his MPC was feasible almost always, including in the 00:21 and 00:28 motion windows.
- My VM node's BLOCKED share (33 % in his window, 100 % since 09:32 -03, constant constraint 106) comes from MY node's stale hold goal (pose_err 0.7 m, arm inert) and is NOT comparable. Do not cite it as evidence about his run.
- The single `packet_reason: MPC solution infeasible` I read from his bridge /status at ~09:56 -03 was one sample; it is not backed by his logs.
- Still-possible causes of "travado/lento" in his run, none confirmed: bridge freshness gates (ESDF 1.5 s with ~260 two-second ESDF stalls; goal 0.35 s; 'ZED paused' status 0.4 s, with ZED2/ZED3 down), the by-design speed caps in his teleop command (linear 0.10, vertical 0.08 m/s, 30 deg/s; SESSION.md says 5/4 cm/s and 15 deg/s), and the bridge's `first joint step` rule (|q - measured| <= 15 mrad after any reset; his gaps reached 18 mrad). The bridge logs no reasons, so which gate fired is unknown. A passive poller of `http://127.0.0.1:8784/status` on the lab would show it next time.

### 2026-10-30 later: step-time spike explained, defaults changed, deployment-path sim
- The `vm-mpc` step time doubling (mpc_step 6.0 -> 12.8 ms at 12:32:2x UTC / 09:32 -03) is NOT an overload (VM load 1.1, no new process, only vm-mpc on the GPU). It tracks `constraint` rising 0 -> 106 right after the arm moved (pose_err 0.635 -> 0.708). Cause: the node calls `self.mpc.step(..., max_attempts=2)` (curobo_mpc_node.py:1604); cuRobo's `MpcSolver.step` (mpc.py:529-538) re-solves AFTER `self.reset()` whenever `feasible` is False. So every infeasible step costs 2 solves and wipes the MPPI warm start (possible jerk source). Still < 20 ms (70 Hz).
- `vm_real.sh`: default config now `fr3_vcap1.2_acc10.yml` (user's call); `--max-step ${MAXSTEP:-0.03}` on the VM-side forwarder. `vm_real/lab/curobo_to_jtc.py` now CLIPS (slew) instead of dropping, no feasibility/freshness/arming gate. The lab copy that `go` pushes into the driver is unchanged (old drop rule, sees only already-limited steps).
- `vm-relay`: I killed its `curobo_to_jtc.py` (nothing leaves the VM). `vm_real.sh up` recreates it.
- Deployment-path sim: `isaac-sim_ws/franka_mpc/deploy_sim/` (`run_deploy_sim.sh`, `deploy_bridge.py` = the REAL `slew()` from curobo_to_jtc.py + spike/stall/delay injection, `goal_step.py` from the ready pose, `analyze_deploy.py`). All containers use `ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST` because the cable puts the lab's domain 8 (nvblox, ZEDs) on the same subnet. Results in `runs/deploy_*/summary.txt`. Sim arm is a stiff Newton PD, not the real impedance: it says nothing about contact or real tracking lag.

### Deployment-path sim results (2026-10-30, 41 sim s each, ready-pose start, vcap1.2_acc10, ESDF replay, real slew() at max-step 0.03)
Goal: +5 cm x, back, fast +12 cm y, back. Raw summaries: `isaac-sim_ws/franka_mpc/runs/deploy_*/summary.txt`; rerun with `deploy_sim/run_matrix.sh`.
| run | hold error / drift | moves mean/max err | stop-and-go | joint accel max | notes |
|---|---|---|---|---|---|
| A clean, hybrid | ~1.2 mm / <=0.8 mm | 8.3 / 51 mm | 0 | 0.9 rad/s2 | clipper never fired (cmd step max 6.9 mrad) |
| B one bad packet (+0.3 rad) at t=12 | same | 8.3 / 51 mm | 0 | 33 rad/s2 | clipped to a 30 mrad bump; libfranka accel limit is ~10-15, so a glitch still shows as an acceleration spike |
| C relay stall 0.5 s during the first move | same | 8.0 / 50 mm | 0 | 5.9 rad/s2 | last target held, smooth catch-up |
| D +3 control steps (60 ms) latency | same | 10.0 / 59 mm | 1 | 1.1 rad/s2 | slightly worse tracking, still smooth |
| E clean, measured | ~1.0 mm / <=0.4 mm | 9.5 / 58 mm | 0 | 1.0 rad/s2 | no wandering with measured either |
Not covered: contact/impedance gains (the sim arm is a stiff Newton PD), real latency jitter, the live (non-replayed) map and phantom obstacles, joint-state dropouts and ESDF stalls (lockstep sim cannot model them faithfully), a real e-stop test. Possible follow-up: limit the change between consecutive commands (acceleration) in slew(), not only the step, so a glitch does not produce the 33 rad/s2 spike (B); MPC `reset()` on an infeasible step is a plausible glitch source.

## Acceleration limiter in the relay + optional "keep" on infeasible MPC steps (2026-10-30, sim only)

Changes (VM repo only; nothing deployed to the lab or the robot):
- `vm_real/lab/curobo_to_jtc.py`: new `limit()` before the `slew()` backstop. `--max-accel` rad/s^2, default 10 (cuRobo/libfranka limit), 0 = off.
  Per joint: follows the target with the MPC's own velocity as feed-forward; closing speed relative to the target is bounded by braking at
  amax/2 (at full amax the discrete follower overshoots 16-24 mm in a unit test, at amax/2 1-3 mm); commanded velocity changes by <= amax*dt per
  command. After a gap > 3 periods it restarts from rest (the controller held still). Never gates or stops the stream.
- `vm_real/node/curobo_mpc_node.py`: parameter `mpc_on_infeasible` = `reset` (default, old behaviour) | `keep`. `MpcSolver.step` calls `reset()` after
  ANY infeasible attempt, even with max_attempts=1 (mpc.py:529-538), so `keep` calls `_step_once` directly (NaN output still resets).
- `deploy_sim/`: `deploy_bridge.py --max-accel`, `run_matrix_lim*.sh`, `INFEAS=` in `run_deploy_sim.sh`.

Results (41 sim s each, same setup as the table above; sim is not bit-reproducible run to run):
| Run | Hold error | Peak joint accel | Notes |
| F clean + limiter | ~1 mm | 0.9 | limiter acted on ~1% of commands (25/2376), outcome identical to A |
| G2 bad packet +0.3 rad + limiter | ~1 mm, drift 0.4 mm | 11.5 (was 33) | |
| I +60 ms + limiter | ~1 mm | 1.0 | |
| J limiter + INFEAS=keep | ~1 mm | 1.0 | limiter acted 9x (F: 25x) |
| H/H2/H3 stall mid-move + limiter | ~1 mm | 12.8-16.4 | NOT comparable to C/C2 (see below) |

Stall caveat: C/C2 (no limiter) hit the stall when the MPC output was nearly still (joint 3 creeping at ~5 mrad/s), H* (limiter) when it was moving
~70 mrad/s, so the sim runs are different tests (MPC redundancy motion varies run to run). Fair comparison = replay H3's recorded raw command stream
with the same 0.5 s stall through both paths (command level, no sim): clip-only -> peak command accel 75 rad/s^2 (30 mrad step, i.e. the clip);
limiter -> 11.9 rad/s^2 (10.5 mrad step). C2's stream: identical (limiter does not act). Not done: the same stream through the sim arm without the limiter.
Also seen: the raw MPC stream itself had a 16 rad/s^2 transient at t=8.42 with no stall in the replay.
Open: `keep` is only shown harmless in the clean sim; whether it removes real-arm glitches needs the robot.

## Correction: what Spot's ROS-side driver does with the commands (read 2026-10-30)
Earlier chat/notes said Spot ran without a clipper because "the Spot driver had its own protection". That was NOT checked. Read now:
submodule `spot-ros2_ws/src/spot_ros2` -> github.com/murilo-vinicius04/spot_ros2 @ 09c30a2 (the pinned commit; not checked out in the VM working copy).
- `spot_controllers/src/spot_joint_controller.cpp` (177 lines): copies position / velocity / effort / k_q_p / k_qd_p of the JointCommand straight into the
  command interfaces. No clamp, rate limit, smoothing, staleness check or NaN check.
- `spot_hardware_interface/src/spot_hardware_interface.cpp`: streams them to the Spot SDK joint stream with constant gains and a 5 ms
  `extrapolation_duration` (~line 796). Nothing else shapes them in this code.
So the ROS side is a pure passthrough. Whatever protected Spot (if anything) is inside the Spot SDK / robot firmware, not visible here and NOT verified.
The FR3 side is different: `fr3_arm_jtc.yaml` = stock JointTrajectoryController on the effort interface, PD gains, holds the measured position if a
joint is >0.08 rad from its reference or no point arrives for 0.2 s; no acceleration limit.

## Node publishes JointTrajectory directly (2026-10-30, sim only; nothing deployed to the lab or robot)
- `vm_real/node/curobo_mpc_node.py`: params `command_type` (`joint_state` default | `joint_trajectory`) and `command_period` (0.02): one-point
  trajectory_msgs/JointTrajectory with the same position + velocity. Spot/sim behaviour unchanged (default).
- `vm_real/lab/curobo_to_jtc.py`: `--msg joint_trajectory` reads that message on its ROS input (still picks fr3_joint1..7, drops fingers); it remains the
  UDP forwarder + limiter + hard clip. Limiter stays in the VM-side forwarder (it sees gaps when the node stops publishing; needs no lab change).
  Forwarder tested alone in a localhost-only container: 30 in, 30 out, finger dropped, time_from_start 20 ms.
- `vm_real.sh`: node `-p command_type:=joint_trajectory`, topic `/arm/joint_command` -> `/curobo/joint_trajectory`, forwarder `--msg joint_trajectory`.
  NOT run on the VM containers yet (`vm_real.sh up` untested with the new flags). The lab copy of curobo_to_jtc.py is still the old one and compatible.
- Sim (node -> JointTrajectory -> deploy_bridge limiter): K clean ~1 mm hold, accel 1.0, limiter on 20/2376; L bad packet accel 11.5 (was 33) but hand
  deviation 3.5 mm max / 4.2 mm drift in the hold at the spike (same as the first limiter run G, 4.0 mm; G2 gave 0.4 mm -> 2 of 3 runs, not noise);
  M mid-move stall: same as H* (15.3 rad/s2, 17 stop-and-go steps, the harder phase explained above).

## Limiter default: OFF (user decision, 2026-10-30)
`curobo_to_jtc.py --max-accel` now defaults to 0 (only the `--max-step` 0.03 clip, like Spot's passthrough); `vm_real.sh` passes `--max-accel ${MAXACCEL:-0}`.
Enable with `MAXACCEL=10 ./vm_real.sh up` if the real arm shows jerky commands / locks up. Plan for the first real hold test: limiter off, log the raw commands.

## Wiring test of the new flags, no robot / no lab (2026-10-30)
`isaac-sim_ws/franka_mpc/deploy_sim/run_e2e_wiring.sh` (E2E=1 in run_deploy_sim.sh): node `command_type:=joint_trajectory` -> /curobo/joint_trajectory ->
REAL `curobo_to_jtc.py` VM half (flags parsed out of vm_real.sh line 35, only the port changed to 47892 on 127.0.0.1, MAXSTEP 0.03, MAXACCEL 0)
-> UDP -> REAL `curobo_to_jtc.py --in udp --out ros` (as the lab half is started) -> /fr3_arm_controller/joint_trajectory -> bridge standing in for the
controller + Newton sim. localhost-only DDS; nothing sent to the lab. Result: 41 sim s, hold ~1 mm (drift <= 0.9 mm), moves mean 8.1 / max 48 mm,
0 stop-and-go, accel 1.1 rad/s2, forwarders: refused 0, clipped 0 (their logs print every 10 s only, last 1645 at ~t=20 s of wall time).
NOT covered: the sim runs the node with use_sim=True (topic /joint_command_curobo), so the `-r /arm/joint_command:=/curobo/joint_trajectory` remap
in vm_real.sh (use_sim:=false) was checked by reading the node code (use_sim False + use_ros2_control False -> '/arm/joint_command', JointTrajectory
publisher when command_type=joint_trajectory), not executed. Also not covered: the real cable/UDP to 10.0.0.2, the lab's own copy of the script.
OPEN BEFORE `vm_real.sh up`: line 56 copies the LAB's own `/home/murilo/franka_ws/hub-curobo/lab/curobo_to_jtc.py` into fr3-ros2, not the repo
version. If that file is the old one it still reads the same {q,v} UDP packets, but its clip/drop behaviour may differ from the repo's. Not checked
(would need a read-only look at the lab); sync or diff it first.

## Definitive version: repo `fr3_real_repro/vm_real/` == lab PC == VM 103 (2026-10-30, user's instruction)
Compared first (read-only), then synced with the user's go. Before: cuRobo library src IDENTICAL on both (178 files, manifest md5 d8e99789...), all 8 configs and
fr3_arm_jtc.yaml identical; DIFFERENT were the node (lab = the 10-29 events-patched file, i.e. our `.orig_lab`) and `lab/curobo_to_jtc.py` (lab = the old 10-22 3.6 KB version,
which `vm_real.sh up` docker-cp's into the driver container).
Changed on the lab (only these two, in /home/murilo/franka_ws/hub-curobo/, murilo's tree; nothing restarted):
- `node/curobo_mpc_node.py`   <- repo version (md5 e470eb65...). Backup: `node/curobo_mpc_node.py.bak_before_1030sync` (= the previous lab file).
- `lab/curobo_to_jtc.py`      <- repo version (md5 0b44f534...; --max-accel default 0). Backup: `lab/curobo_to_jtc.py.bak_before_1030sync`.
Revert: `mv <file>.bak_before_1030sync <file>`.
Checked first that nothing running reads them: Hipolito's `fr3-zed-live-mpc` mounts hub-curobo/node read-only but its launcher runs his own `live_zed/live_mpc.py`
(launch_mpc.py replaces `/node/curobo_mpc_node.py`), and `lab/` is mounted nowhere. Defaults keep the old behaviour (command_type=joint_state, mpc_on_infeasible=reset).
Drift check: `vm_real/check_sync.sh` (read-only, exit 1 if anything differs). The repo is the source of truth: edit here, then copy to the lab.
NOT synced: the RUNNING `vm-mpc` container on the VM still has the node loaded before the new params existed (it needs a restart to pick them up; it is idle,
its command forwarder is stopped). `vm_real.sh up` itself still untested on the real containers.

## CORRECTION (2026-10-30 22:30 -03): the lab's running cuRobo IS our node
Earlier in this doc I wrote that Hipolito's `fr3-zed-live-mpc` runs his own `live_mpc.py` and does not import our node. True for the 10-29 run, WRONG for the one
started 10-30 ~22:15: it runs `live_zed/murilo_current.py`, which reads `/node/curobo_mpc_node.py` (= hub-curobo/node, the file synced at 18:53 -03; md5 e470eb65...)
verbatim, adds telemetry hooks, and records its sha256 (not pinned). Params seen on the live process: node `fr3_joystick_mpc`, `fr3_vcap1.2_acc10.yml`, `use_sim:=false`,
`use_esdf:=true` (live nvblox service, 4 Hz), `feedback_mode:=measured`, `start_active:=false`, `command_type:=joint_trajectory`, `command_period:=0.02`,
`mpc_on_infeasible:=keep`, output remapped `/arm/joint_command -> /fr3_joystick_mpc/joint_preview` (private preview topic; his guarded `joystick_bridge.py hardware`
sits behind it, disarmed). So my sync changed what his stack executes; the defaults are unchanged, but he already uses both new flags (`keep` is not yet validated on
the real arm). The process was restarted twice between 22:15 and 22:20.

## FIRST REAL-ARM TRIAL of the cuRobo node on the lab FR3 (2026-10-30 22:44-22:50 -03), user's go, Hipolito at the e-stop
Node under test: the lab's running `fr3_joystick_mpc` (Hipolito's container, our node via `murilo_current.py`, fr3_vcap1.2_acc10, measured feedback, live ESDF, mpc_on_infeasible=keep,
command_type=joint_trajectory, already ACTIVE, output on `/fr3_joystick_mpc/joint_preview`). Path used: preview -> `curobo_to_jtc.py` (ros->udp, container `claude-fwd-a`, domain 142, FASTDDS UDPv4)
-> UDP 127.0.0.1:47802 -> `curobo_to_jtc.py` (udp->ros, in fr3-ros2) -> `/fr3_arm_controller/joint_trajectory`. Limiter OFF, clip 0.03. Joystick bridge NOT used.
Needed his `joystick_bridge.py hardware` (pid 776232) FROZEN with SIGSTOP 22:44:00-22:50:26 (its hold-goal stream would fight our goal; resumed with SIGCONT, status page back, wrist_goal 17 Hz again).
Sequence: scripts in /home/murilo/franka_ws/lab/claude_trial/ (vm_real/lab_trial/ here: rec_watch.py recorder+watchdog, goal_run.py goal publisher, rec_stats.py, trial_analysis.py);
dry run with controller inactive; activate fr3_arm_controller (nothing else was active) 22:47:47; 45 s hold; goal run 22:49:12 (hold 8 s, +5 cm x in 3 s min-jerk, dwell 6 s, back 3 s, tail 10 s); deactivate 22:50:16.
Watchdog (never tripped): tracking > 0.05 rad, |qd| > 1.5 rad/s, robot mode outside {1,2}, any error flag, hand > 7 cm from start, stale robot state -> SIGKILL of the lab-half forwarder.
Data: vm_real/lab_trial/data_2026-10-30/ (rec.csv 100 Hz, goal.csv, preview.csv, forwarder logs, analysis.txt).
Results: no trips, mode 1 -> 2 -> 1, no error flags, peak joint speed 0.124 rad/s, measured joint accel max 1.0 rad/s2, commanded |v| <= 0.21 rad/s, steps <= 3.8 mrad, 0 clipped / refused, hand peak 4.4 cm/s.
 * Activation: hand sagged 1.5 mm in z within 1 s, then held: 0.008 mm wander over 20 s (steady error to goal 1.65 mm, like the sim's 1.2 mm).
 * Tracking: hand error to the goal mean ~14.5 mm / max ~27 mm during the 3 s legs; hand reached 50% of the step 810 ms after the goal did.
 * Steady-state offset after each leg: dwell +5 cm: 6.5 mm short in x, 7.2 mm low in z; tail at P0: 7 mm beyond in x, 4 mm low in z, creeping ~0.2 mm/s. The sim settled to ~1 mm. NOT explained. Node log: in dwell pose_err ~0.09
   (hold 0.013) and coll_cost ~15 (hold 0.2), all steps OK/feasible. Candidates: obstacle/ESDF trade-off on the live map, 'measured' feedback acting as a pure P loop on the JTC (deadband/friction/gravity,
   no integral; 'hybrid' keeps the command lead), JTC gains. Discriminating tests: same move with USE_ESDF=false and with feedback hybrid (VM instance), and compare older real runs (dual_jtc_hybrid vs dual_jtc_measured).
 * Rate on the lab: 32.4 Hz (35 Hz in standby), inter-arrival p50 21 ms, p99 117 ms, max 140 ms, 33 gaps > 100 ms in 27.5 s; the MPC plan runs at 65% of real time (20 ms per step). Node ~80-90% of one core at nice -10, lab load 57-64,
   GPU0 32%. ESDF refresh every 0.25 s (service 40 ms mean, 85 max). The 13 h VM run had 49 Hz, p99 8 ms. Cause of the gaps not established (CPU contention vs ESDF update).
 * Commanded stream stayed smooth through the gaps (max change of commanded v per command 0.023 rad/s).
Process notes: first claude-fwd-a used FASTDDS_BUILTIN_TRANSPORTS=DEFAULT and received nothing (the relay uses UDPv4); fixed. I started the arm activation about 25 min after the user said Hipolito was at the e-stop without re-asking.
Left on the lab: only /home/murilo/franka_ws/lab/claude_trial/ (scripts + out/) and /tmp/claude_* inside fr3-ros2. Controller inactive again, mode 1.
