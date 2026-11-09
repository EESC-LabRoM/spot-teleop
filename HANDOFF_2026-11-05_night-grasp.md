# HANDOFF 2026-11-05 night: grasp lane (real FR3) + joystick incidents

Lane "Grasp robô real", omnigent session 10098820a59f44898914e02d12d03c51. Times are local (UTC-3) unless marked Z.

## CURRENT RESUME (2026-11-06; latest update from RAL lane)
- **Current deployed camera calibration remains robotfit7**: `fr3_real_repro/zed2_align/zed1_extrinsic_20261105_robotfit7.json` and `zed2_extrinsic_20261105_robotfit7.json`; depth scales ZED1 0.9767, ZED2 0.9959, applied in VM fan-ins. No replacement was fitted or deployed. Sim lane was told to keep mirroring these files.
- **Table-step result: unresolved; do not use either earlier estimate.** The unsupervised ZED1 split of 29 mm was withdrawn. Seeded static patches near the Franka base (A) and drill (B) yield h=+3.9 mm, within the ~9 mm method floor, but their normals differ by 3.61°; the back band matches neither plane. Thus current ZED1 depth does not support two parallel table planes, and the joint table+ICP fit was stopped. Likely spatially varying stereo/depth bias near ZED1 image edge; this remains a hypothesis, not proven. ZED2 sees only the drill/AprilTag table; ZED1 sees both. Murilo says the drill table is visibly higher and the base sits on the Franka table, so camera data contradict physical layout.
- Seeded-fit artifacts and analysis: `fr3_real_repro/zed2_align/calib_robot/table_step_seeded.py` and related table-step scripts/plots in that directory. Lane's metrics: A Franka disk r=0.10–0.30 m, 10,785 pts, plane RMS 1.9 mm; B drill disk r<0.22 m, 2,491 pts, RMS 1.4 mm; h=+3.9 mm scale-corrected; normal gap 3.61°; back band 3,328 static pts, 2,323 match neither plane. The 29 mm split and the previous “|h|≤15 mm” inference are both retracted.
- **Next calibration work:** inspect/fix ZED1 edge distortion/rectification; use robot-fit captures spread across workspace; ground-truth table step if a reliable physical reference becomes available. Don't fit/deploy a joint two-plane calibration from these data. Any replacement calibration requires Murilo's go to deploy. Offline analysis only so far; robot untouched, `franka-robot` free.
- **Force limits:** set to Oopsie/ROS1 values (force [50,50,50,30,30,30], torque [40,40,36,36,16,14,12]) and made automatic on every `fr3-ros2` start. `fr3_ros2.sh` has the start hook; cron watcher is backup. Applications logged at `/home/murilo/franka_ws/runs/collision_apply.log`. Backup: `/home/murilo/franka_ws/lab/fr3_ros2.sh.bak_collision_20261105`. Recorder is passive and verified alive at `/runs/fext_rec_20261105_2252/` inside `fr3-ros2`, expected to end ~04:52Z; check `events.log` after reflex. Arm last reported mode 2, teleop disengaged, ~3 N at rest; no new reflex in recorder at the lane's check.
- **Other pending work, requires Murilo's go:** add 10 rad/s² acceleration clamp to `/tmp/direct_topic_fwd.py` (backup first); grasp rerun with `--depth 0.025` with a named e-stop person. Claim `franka-robot` before any arm-adjacent work. No arm motion or calibration deployment was authorized/performed in the handoff lane.
- **Coordination:** calibration/grasp work handed to RAL lane `Teste grasp robô real` session `d8b7488839304031adbd7b70025e578d`; it confirmed the seeded-fit result and stopped before joint fit. Sim lane `62f98b58e719469dbd79db4b78620603` was told robotfit7 remains the real calibration truth.
- Older sections below preserve the history; where they conflict with this current resume block, this block is authoritative.

## RESUME HERE (22:15Z; details in "Update 21:55Z" and "Note 22:10Z" at the bottom)
- Goal: real FR3 grasp (grasp_test.sh) + pad teleop on the direct path (cuRobo MPC codex-fr3-live-mpc -> fr3-ros2:/tmp/direct_topic_fwd.py -> effort JTC).
- Arm: LOCKED in cartesian_reflex since 21:36:05Z, mid-air, robot_mode 4, HW inactive, pad disengaged. I hold NO claim.
- Problem: recurring cartesian_reflex (11 today + 11-04 14:18Z through the old bridge). NOT speed (0.49 rad/s max vs 1.2 cap, err <0.05 rad; script tools_1105/mpc_log_speed.py), NOT the bridge removal (bridge had no force check and also locked up).
- Suspects: collision thresholds at factory 20 N (reset on every driver restart, no read-back; `./fr3_ros2.sh real-collision` sets 50 N/30 Nm) or a payload/model bias in the F_ext estimate.
  - Checked 22:50Z: the driver started ONCE today (container fr3-ros2 main process, 14:14:57Z). `real-up` calls `real-collision` after every start (fr3_ros2.sh:132), but its output is not logged anywhere (last logged apply: autorecover.out 10-31). The driver itself sets NO thresholds at init (franka_hardware robot.cpp only via the service). So 50 N today is plausible but unproven; the F_ext recorder at the next reflex shows which limit fired (~20 N vs ~50 N).
  - 22:48:27Z (Murilo go): `./fr3_ros2.sh real-collision` -> success=True, torque [40,40,36,36,16,14,12] Nm, force [50,50,50,30,30,30]. Holds until the next driver restart. Arm still in reflex, not recovered, /teleop/engaged false. franka-robot claimed for the call and released.
  - 22:50Z state (Murilo go for recovery + recorder): the arm was ALREADY recovered. Driver log: error_recovery 21:50:50Z, hardware activated 21:50:59Z, fr3_arm_controller active 21:51:03Z (not by me after compaction; who did it is not recorded). Now: robot_mode 2, no error flags, joints unchanged [1.873,-0.694,-1.267,-1.780,-0.621,1.480,1.227], MPC active, preview <= 0.030 rad from the joints, /teleop/engaged false, F_ext 2.2-2.4 N at rest. The thresholds survive error_recovery (only a driver restart resets them).
  - Configured load: inertia_ee m 0.73 kg (stock Franka Hand), inertia_load 0. Wrong if anything else is mounted on the hand (ask Murilo).
  - F_ext recorder RUNNING (passive, never commands): lab container fr3-ros2 `/tmp/fext_recorder.py` (source: fr3_real_repro/grasp_loop/tools_1105/fext_recorder.py), output `/runs/fext_rec_20261105_2252/` (rec100.csv, heartbeat.log, events.log + event_<n>_mode4.npz on every reflex), runs until ~04:52Z. It handles ~700 of 1000 msg/s. After a reflex: `docker exec fr3-ros2 cat /runs/fext_rec_20261105_2252/events.log`: peak force/torque as a fraction of the 50 N / Oopsie limits (~0.4 = the arm was really at 20 N), collision flags, peak dq, peak commanded-torque step.
  - Next: Murilo drives. Step 3 (forwarder acceleration clamp) not done.
  - 23:01Z thresholds are now the DEFAULT on every fr3-ros2 start (Murilo go): lab murilo crontab runs `/home/murilo/franka_ws/lab/collision_watch.sh` every minute; on a new container StartedAt (real hardware) it runs `/lab/collision_on_start.sh` inside the container (waits for the driver service, applies the real-collision values, retries next minute on failure). Every apply/failure is logged in `/home/murilo/franka_ws/runs/collision_apply.log` (= /runs in the container); `fr3_ros2.sh real-collision` does NOT log there (that file is Hipolito's, murilo has no write access, left untouched). Disable: `touch /home/murilo/franka_ws/runs/collision_watch.disable`. Tested on the running container 23:00:52Z (applied try 1, second run no-op). Sources: fr3_real_repro/grasp_loop/tools_1105/collision_{watch,on_start}.sh.
  - 23:03Z (Murilo go, via the hipolito login: ACL u:murilo:rwx on fr3_ros2.sh): fr3_ros2.sh now ALSO starts /lab/collision_on_start.sh inside the container in real-up's docker run (COLL_TAG=real-up, skipped for FAKE=1 or COLLISION=0), and real-collision logs to runs/collision_apply.log. Backup fr3_ros2.sh.bak_collision_20261105. Tested: composed command parses; real-collision logged 23:03:30Z. The real-up path itself is untested until the next driver start (check collision_apply.log for a [real-up] line); the cron watcher stays as backup.
  - 23:10Z calib tilt measured OFFLINE, no touch (fr3_real_repro/zed2_align/calib_robot/table_plane_check.py run_1105 ..): Franka-table plane from the saved background depth with the deployed robotfit7 calib + depth scale. ZED1: tilt 2.66 deg, z at base +6 mm, corners -34..+46 mm. ZED2: tilt 1.94 deg, z at base +45 mm, corners +11..+47 mm. Plane rms ~4 mm, so the table is flat and the cameras are tilted. Next: joint fit = robot-mesh ICP + table plane at z=0 (needs Murilo: base directly on the tabletop or on a plate?), offline, then tagcheck cross-camera, then deploy with go.
  - 23:20Z HANDED OVER to lane 'Teste grasp robô real' (d8b7488839304031adbd7b70025e578d), Murilo's instruction (quota). Calib work in progress:
    - Murilo: Franka base sits DIRECTLY on its tabletop (table = z 0 in fr3_link0). The drill table is a second table, slightly higher; Murilo wants that step MEASURED geometrically, not assumed.
    - Height maps (calib_robot/table_heightmap.png, table_heightmap.py) and plane residuals + zed1-zed2 difference (table_resid.png, table_resid.py); both run in vm-viewer (has matplotlib): docker cp script + run_1105 + robotfit7 JSONs to /tmp/thm, `python3 /tmp/thm/table_resid.py /tmp/thm/run_1105 /tmp/thm /tmp/thm/resid.png`.
    - Numbers: zed1 plane z = +12.9 mm/m x -23.0 mm/m y +11 mm (only 58% inliers at 12 mm, so it is not one plane); zed2 z = -27.8 x -9.0 y +42 mm (88%). zed1-zed2 on 584 shared 3 cm cells: median -8 mm, linear part +23 mm/m x -21 mm/m y -21 mm, rms 19 mm (17.6 after removing the plane).
    - Idea: the physical step is common to both cameras (it cancels in zed1-zed2); calibration tilt is per camera. Next: (1) find the table edge in the residual maps, (2) fit two parallel horizontal planes (z=0 Franka table, z=h drill table) jointly with the robot-mesh ICP (fit_robot7.py) per camera, h shared across cameras, (3) validate with tagcheck.py cross-camera, (4) deploy only with Murilo's go.
  - ZED1 23:00Z: set 30 Hz, really ~23 Hz (camera_info 23.3, compressed 22.5); lab load avg ~58. Dashboard feed choppy (Murilo).
- Plan, WAITING for Murilo's go (none of it moves the arm), claim franka-robot first: (1) in-place recovery + `real-collision`; (2) passive 100 Hz robot_state recorder (o_f_ext_hat_k, tau_ext, q, dq) based on fr3_real_repro/vm_real/lab_trial/contact_guard.py --dry; (3) 10 rad/s^2 accel clamp in /tmp/direct_topic_fwd.py (backup first); (4) Murilo drives; if clean, raise teleop speeds (now ~0.125 m/s, 15 deg/s).
- Still open after that: calib tilt (touch test + robot-fit with spread poses), grasp rerun with `--depth 0.025` (needs a named person at the e-stop).

## State at hand-off (~18:22 local / 21:22Z; SUPERSEDED by "Update 21:25Z" at the bottom)
- Arm: parked over the drill. Fingertips z≈0.357 in fr3_link0, straight down, gripper OPEN 80 mm.
  Joints [0.439,-0.195,-1.065,-2.091,-0.199,1.976,0.342].
- Stopped by a libfranka **cartesian_reflex** at 21:16:36Z (force/contact safety) while Murilo jogged with the pad.
  Cause NOT known: ask Murilo whether the hand touched anything.
- FrankaHardwareInterface INACTIVE, both controllers inactive, robot_mode 1, /teleop/engaged False (disengaged by me),
  MPC /fr3_joystick_mpc in standby (set_active false).
- franka-robot claim: the dashboard lane (076ac24b270644f8b9eb1a46f6301ab0) holds it to restart :8089 then :8091.
  That restart clears the dashboard "recovery hold active" refusal (sha mismatch after the forwarder fix).
  I am queued behind it.

## Next steps (in order)
1. Once the dashboards are restarted and I hold the claim, do the in-place recovery (does not move the arm),
   in fr3-ros2, domain 77, ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST:
   - check /teleop/engaged == False
   - `ros2 action send_goal /action_server/error_recovery franka_msgs/action/ErrorRecovery '{}'`
   - `ros2 control set_hardware_component_state FrankaHardwareInterface active`
   - `ros2 control switch_controllers --activate fr3_arm_controller`
   - check the joints did not change
   - `ros2 service call /fr3_joystick_mpc/set_active std_srvs/srv/SetBool '{data: true}'` (re-seeds from the measured joints)
   - check the first /fr3_joystick_mpc/joint_preview is < 0.05 rad from /joint_states
   - only then: pad Start
2. Do NOT use the dashboard "Start teleop ZED": it checks the retired bridge and always refuses.
   Avoid backend cuRobo<->cartesian switches.
3. Calibration question (open, see below), then the grasp.

## Root causes found today (verified in logs)
- **Jerk/velocity reflex on pad Start** (20:20:24Z, 20:21:54Z, 21:10:37Z): the lab forwarder `fr3-ros2:/tmp/direct_topic_fwd.py`
  (joint_preview -> /fr3_arm_controller/joint_trajectory, gated on /teleop/engaged) kept its slew-clip `last` pose across a disengage.
  The next Start then slewed at 0.03 rad/20 ms = 1.5 rad/s from the OLD pose.
  **Fixed** 21:13:44Z: re-seed from the measured joints on every engage edge; refuse if the MPC command is > 0.05 rad off.
  Backup: `.bak_staleslew_20261105`; log `/tmp/direct_topic_fwd.log`.
  After the fix Start worked (6.9k packets, 0 refused) until the cartesian_reflex.
- MPC "latched fault" (tracking error > 0.1 rad): a consequence of the reflexes. Clear it with set_active true while disengaged.
- **Dashboard Home button disabled** for two reasons:
  - The lab mesh_server (:8788, VM :8443 tunnel) reads `--robot-dash 127.0.0.1:8088`, but the robot dashboard runs on :8091.
    Fix: forwarder `/home/murilo/franka_ws/lab/tcp_fwd_8088.py`, not persistent.
  - The page gated Home on `zed.controller_reason`. Patched `isaac-build:/tmp/nvblox_real/lab_dashboard.html` (backup `.bak_homegate_20261105`).
- Home = `cd /home/murilo/franka_ws/lab && ./fr3_ros2.sh real-home` (= dashboard Home = pad X) -> pose [0,-pi/4,0,-3pi/4,0,pi/2,pi/4].

## Grasp runner changes (fr3_real_repro/grasp_loop/, backups *.bak_20261105 / *.bak_stages_20261105)
- grasp_test.sh does NOT use cuRobo: numpy IK -> direct JTC (effort P/D, no I term).
  Use the sparse plan `"--cart-step 0.15 --wp-min-time 6"` (multi-waypoint plans don't track).
- Stages now: S1 home->PRE(12 cm) · S2 PRE->GRASP · close · S3 lift · S4 put back · open · S5 ->PRE · S6 home.
  - Gripper opens only if < 75 mm.
  - 5th arg FROM=S1..S6 resumes a run.
  - `home` mode exists.
- Q_HOME = real-home pose (the old one was 0.107 rad off, so S1 refused after real-home).
- grasp_plan.py options:
  - `--depth` (push deeper along the approach)
  - `--map-z-bias` (default 0; DO NOT use -0.028, see below)
  - `--seed-q`
  - IK solved densely (1 cm) to avoid elbow flips
  - grasp_hand check: palm/finger points in the cloud, fingertip below the drill top
- grasp_stage_exec.py: a short correction move is allowed if within 0.20 rad of the stage target.
- Plan input used: `grasp_loop/runs/20261105_175633_drill_dual2/grasp_plan_input.json` (pick in fr3_link0).
  - Executed grasp reached the planned TCP within 2 mm.
  - BUT the fingertip centre was only 1.8 cm below the cloud drill top, and the upper tip sat at the top edge (approach tilted 41°).
  - `--depth 0.025` gives 4.1 cm depth with the palm still clear.

## Calibration finding (open question, expensive to redo)
- The dashboard grasp view (seg_view.py `_live_shift_grasp`) re-anchors the grasp onto the LIVE cloud, aligning top surfaces.
  So the dashboard always "looks right" and cannot show a vertical error the robot sees.
- Murilo: the drill table is physically HIGHER than the Franka table (slight step, visible in ZED1, not in ZED2).
  So the "+2.8 cm table" is partly real, and `--map-z-bias -0.028` is WRONG.
- Raw depth with the deployed robotfit7 calibs (scripts copied to fr3_real_repro/grasp_loop/tools_1105/: grab_table.py, run in vm-viewer, domain 8, `/vm/cable{,2}/depth`):

  | Region | ZED1 | ZED2 |
  |---|---|---|
  | Near the drill | +2.2..+2.9 cm | +2.5..+3.5 cm |
  | Near the base (x 0.1-0.3, y +0.3..+0.45) | -1.5..+0.4 cm | +3.8..+5.5 cm |

  Both cameras also slope 2-3 cm along a strip that should be flat. So the calibs carry a TILT of several cm away from the arm-fit region.
  - Deployed calibs are robot-fit ICP (NOT tag z; the tag was only the ZED2 init).
  - Proposed fix: redo the robot-fit with poses spread over the workspace + fingertip touch anchors on both tables.
- Why the real hand looked "too high" at S3 is still unexplained.
  Candidates: the drill-cloud top edge (stereo edge points) or the shallow GraspGen grasp.
  Next measurement: a fingertip touch on the drill table beside the drill and on the drill top, then compare FK z with the camera heights.

## Other notes
- ESDF reports ~70% occupied voxels and frequent request timeouts (steady for hours; not the fault cause).
- Lane 8a no longer exists; nobody owns the MPC chain. Hipolito's teleop lane 846022d5 is offline.
- Dashboard lane 076ac24b added a switch guard + Recover read-back in fr3_web_dashboard.py (:8089) and the :8091 file
  (backups `.bak_switchguard_20261105`); both active after their restart.

## Update 21:25Z (18:25 local)
- Dashboard lane finished the :8089/:8091 restarts (:8091 sidecar now runs INSIDE fr3-ros2; host and container copies of /tmp/dash_sidecar diverge).
- In-place recovery done, no motion: error_recovery OK, hardware active, fr3_arm_controller active, robot_mode 2, joints changed 3e-5 rad.
- MPC set_active true while disengaged: state active, first preview 0.028 rad from /joint_states (< 0.05 forwarder gate). Ready for pad Start.
- franka-robot claim RELEASED 21:30Z (queue empty, inbox empty). MPC left active but teleop disengaged, so the forwarder sends nothing. Re-claim before any live step, including pad Start.
- Still open: cause of the 21:16Z cartesian_reflex (asked Murilo: did the hand touch something?); calibration tilt (touch test, then robot-fit with spread poses); then grasp rerun with `--depth 0.025` (NOT `--map-z-bias`).

## Update 21:40Z: second cartesian_reflex during pad jogging
- Murilo engaged the pad at 21:34:33Z (claim not held by this lane). cartesian_reflex at **21:36:05Z** (driver log `fr3-ros2:/runs/fr3-ros2.real.log` line ~12348). robot_mode 4.
- No JTC tolerance errors before it (unlike the 21:10Z velocity reflex). Joint limits are not involved.
- Arm stopped at joints [1.873,-0.694,-1.267,-1.780,-0.621,1.480,1.227]: j1 is rotated 107° to +y; o_t_ee (0.515, 0.098, 0.452), so the hand is well above the table. F_ext is now ~0.8 N, i.e. not still pressed against anything.
- Thresholds: `fr3_ros2.sh real-collision` sets 40/40/36/36/16/14/12 Nm and 50 N / 30 Nm, "at every start". Whether it ran after the 14:15 local driver start is NOT verified, and libfranka has no read-back. If it did not run, the firmware default is 20 N / 20 Nm.
- /teleop/engaged was still True after the reflex, and the forwarder kept sending into the inactive JTC. Disengage BEFORE recovery.
- Open question for Murilo: did the elbow or the upper arm touch something on the +y side? Both reflexes so far happened 1-2 min into pad jogging, at different poses.

## Update 21:55Z: lock-up analysis + proposal (NOT applied; waiting for Murilo's go)
- Murilo disengaged the pad. The arm is still in reflex (robot_mode 4) at [1.873,-0.694,-1.267,-1.780,-0.621,1.480,1.227], mid-air, nothing touching. No claim held.
- **The old bridge did NOT limit force.** Its `command_guard.py` (lab `/home/murilo/franka_ws/lab/fr3_zed_runtime/curobo_live/`, backups `*.bak_guardsdown_1104`; retired copy `retired_1104_direct/`; not in git) refused commands on:
  - joint limits, |v| > 1.2 rad/s, step > 0.005 + 1.2·dt, dv > 0.05 + 10·dt (≈ 10 rad/s²);
  - tracking > 0.05 rad, first step > 0.015 rad;
  - stale packets, infeasible MPC, robot outside map coverage.
  - The "8 N" guard was `fr3_real_repro/vm_real/lab_trial/contact_guard.py` (11-01 avoidance runs), not the bridge.
- Cartesian impedance does not trip because it caps its force (translational_clip 0.02 × stiffness 1000 = 20 N; rotational 0.05 × 75) and filters the target (target_filter 0.005 per 1 kHz cycle, ≈ 0.2 s). The JTC path has only the forwarder's 0.03 rad/packet slew and the 0.05 rad gate. JTC gains: p 150, d 15, i 0, ff_velocity 0.
- **cuRobo already caps speed:** live MPC robot_config `/workspace/franka_mpc/node_config/fr3_vcap1.2_acc10.yml`: 1.2 rad/s every joint, 10 rad/s², jerk 500.
- **Measured** from the MPC log `codex-fr3-live-mpc:/tmp/codex-live-mpc.csv` (5 Hz, cmd_pos/curr_pos columns; analysis script `fr3_real_repro/grasp_loop/tools_1105/mpc_log_speed.py`, run on a grep of the CSV):
  - Before both reflexes: max joint speed 0.49 / 0.48 rad/s, smooth over the last 3 s, max |cmd - meas| 0.036 / 0.047 rad.
  - So speed is NOT the cause, and a tighter speed cap would not help.
  - The MPC log shows the arm stopping ~0.4-0.9 s before the driver's reflex timestamp (clock skew, not investigated).
- Remaining candidates:
  1. Collision thresholds at the firmware default 20 N (not verified that `fr3_ros2.sh real-collision` ran after the 14:15 local driver start; no read-back possible).
  2. 20 ms command spikes the 5 Hz log cannot show.
  3. A light real touch (Murilo saw nothing).
- Teleop speeds now (live `fr3_input_teleop_review.py` args): linear 0.20 × speed_scale 0.625 ≈ 0.125 m/s, vertical 0.10 m/s, angular 30°/s × rot_scale 0.5 = 15°/s, max_lead 0.03. The yaml has 0.60/0.40/90° × 0.5. Murilo wants faster teleop.
- **Proposed plan** (all needs Murilo's go + a franka-robot claim; steps 1-3 do not move the arm):
  1. Disengage check, in-place recovery (steps at the top), then `./fr3_ros2.sh real-collision` (50 N / 30 Nm, 40/40/36/36/16/14/12 Nm).
  2. Passive 100 Hz recorder of robot_state (o_f_ext_hat_k, tau_ext_hat_filtered, q, dq) + JTC commands to a CSV (base on contact_guard.py with --dry).
  3. Forwarder acceleration clamp at 10 rad/s² (backup first, sha check).
  4. Murilo drives a few minutes; if clean, restart the teleop with higher speeds (on go).

### Note 22:10Z: the bridge did NOT prevent cartesian_reflex
- 11-04 14:18Z: cartesian_reflex during cuRobo pad teleop THROUGH the old bridge (teleop -> /fr3_curobo/equilibrium_pose, stick capped at 5 cm/s), ~23 s after engage (lab /runs/codex_teleop_reflex_20261104.log, codex_reflex_20261104.log).
- Driver log /runs/fr3-ros2.real.log reflexes 11-05 (UTC): cartesian 14:48, 15:20, 15:30, 15:39, 16:34 (x3), 16:40, 17:41, 21:16, 21:36; joint_velocity_violation 20:21, 21:10 (stale-slew bug, fixed). What drove the arm at 14:48-17:41 is not recorded.
- So removing the bridge is not the cause, and neither is speed. Main suspects: collision thresholds at the default (they reset on every driver restart, no read-back) or a model/payload bias in the external-force estimate. Plan unchanged: re-apply 50 N + 100 Hz F_ext recorder.
