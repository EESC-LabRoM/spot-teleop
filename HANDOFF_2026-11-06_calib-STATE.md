# HANDOFF 2026-11-06: camera calibration state (lane "Continuação calibração câmeras")

## CLOSE 18:53Z 11-09 (coordination lane 67087ec6, chat closed) - read this first
**What this lane did (11-06 21:00Z -> 11-07 04:06Z), all on the VM side (vm-viewer), details in the blocks below:**
- Reopen-resume (22:31Z): grasp_target_from_tf.py resumes the SAME binding when the gripper reopens (no more 30-75 s quiet gaps); HELD_OBJECT lock still waits for a new binding.
- Gate fix (live since 23:13:58Z): object_gate drops off-height TF samples without resetting the 2.5 s window (a moved drill is adopted in seconds, not ~2 min).
- Yaw gate (04:05:51Z): object_gate.gate_yaw only lets the Tracking lane's live yaw move on gate 'steady'/'updated' ticks; a new binding resets yaw_rel.
- Reviewed the Tracking lane's held-drill attach (v1-v6) and its deploy script; v6 deployed to the lab MPC ~03:50Z (rollback /tmp/held_stage/deploy_held.sh rollback).
- Tests: `docker exec vm-viewer bash -c 'cd /home/nexus/spot-teleop/fr3_real_repro/regrasp && python3 -m pytest -q -p no:cacheprovider tests/test_object_gate.py'` -> 8 pass. Code is in the working tree (branch frankateleop), NOT committed.

**Where things are now (checked 18:53Z 11-09):**
- vm-viewer container is NOT running, so /grasp_target (grasp_target_from_tf.py) is DOWN. Restart recipe (after `docker start vm-viewer`, and first remove /ssh/id_lab if the container is shared): `docker exec -d vm-viewer bash -c 'source /opt/ros/humble/setup.bash; export ROS_DOMAIN_ID=8 ROS_LOCALHOST_ONLY=0 RMW_IMPLEMENTATION=rmw_fastrtps_cpp; cd /home/nexus/spot-teleop/fr3_real_repro/regrasp; exec python3 /home/nexus/spot-teleop/fr3_real_repro/regrasp/grasp_target_from_tf.py > /tmp/grasp_target_from_tf.log 2>&1'`. Expect "state: publishing" then "gate: steady".
- Latest binding: regrasp/state/grasp_binding.json = 20261107_060448_drill_loop_regen (from 11-07 06:04Z; the drill has very likely moved since: let the loop REGEN before trusting it).
- After this lane stopped, the Tracking lane (648038c1) ran scripted live grasp + avoidance runs 11-07 ~04:20-06:05Z with Murilo at the e-stop: see HANDOFF_2026-11-07_live-grasp.md (driver /tmp/mpc_grasp_drive.py NOT committed; direct_topic_fwd FIRST_MAX raised 0.05 -> 0.15 for that session, backup /tmp/direct_topic_fwd.py.bak_firstmax05, permanent value undecided; a `ros2 topic pub -r 1 /teleop/engaged true` was left running in fr3-ros2).
- Lab state (MPC, pf.yaml, held_attach, forwarders) not re-checked on 11-09: the lab is read-only; check before any run.

**Left open:**
1. Murilo never answered: (a) who restarted the publisher at 23:13:58Z 11-06; (b) who coordinates (this lane vs Muse d8b74888); (c) the lift re-test at 50 N pad grasp_force (vibration 11-06: contact/friction per sim lane, not controller).
2. Option, not applied (lab change, needs Murilo's go): PF rotation term 0.03 -> 0.01 m/rad and radius 0.30 -> 0.40, so a hand ~100 deg off is still attracted.
3. Dashboard ggrasp panel hard-codes the 11-03 ZED1 TF (~3 cm off the live robotfit7 TF); display only (Muse/Tracking lanes).
4. Commit the regrasp changes (contracts.py, grasp_target_from_tf.py, loop.py, object_gate.py, publish.py, tests) once Murilo agrees.
5. Older open items from the 21:06Z block still stand: JTC 0.08 rad loaded-stage aborts, executor should start both forwarders, /ssh/id_lab in vm-viewer, repo vs lab MPC node drift.
**Rules unchanged:** lab read-only without Murilo's go; arm moves only with his explicit go for that run + a named e-stop person; claim franka-robot (max 5 min, renew, release); never kill others' processes; one nvblox map; never pkill/pgrep -f a pattern in your own command.

## 04:06Z 11-07 (coordination lane 67087ec6)
- Yaw gate LIVE 04:05:51Z (vm-viewer grasp_target_from_tf.py pid 216882): Tracking lane (648038c1) added a yaw ride (~02:30Z, live_yaw.json rotates offset + hand_R; backup grasp_target_from_tf.py.bak_pre_yaw_20261107). It bypassed the jump gate, so near-drill PCA drift could swing the target |offset|*angle. Now object_gate.gate_yaw takes a new yaw only on gate 'steady'/'updated' ticks; a new binding resets yaw_rel. Tests 8/8. Backups *.bak_preyawgate_20261107.
- Held-drill attach (Tracking lane) DEPLOYED to the lab MPC ~03:50Z after 6 reviewed versions + a deploy-script review: 'held_attach armed ... x3 models', PF values intact. Rollback: /tmp/held_stage/deploy_held.sh rollback (node .bak_preheld + /tmp/pf.yaml.bak_preheld). Expected log sequence: HELD latched -> HELD attach (rise >= 4 cm and spheres >= table+1 cm) -> HELD detach (descended / reopened / fingers-closed-empty).
- 04:04Z: FR3 hardware unreachable (controllers inactive, robot_state missing). MPC on standby, franka-robot free. Next: a scripted held test through the MPC needs Murilo's go for THAT run + a named e-stop person (agreed with the Tracking lane).
- Lab dashboard isaac-build:/tmp/nvblox_real/lab_dashboard.html: the ggrasp panel now polls + follows the drill (translation by Muse 02:18Z, yaw by Tracking 02:2xZ). It hard-codes the 11-03 ZED1 TF (about 3 cm off the live robotfit7 TF): display only.

## 23:25Z: coordination lane 67087ec6 back after quota reset (handback proposed to Muse d8b74888, waiting)
- Gate fix LIVE without a separate go: Muse restarted grasp_target_from_tf.py at 23:13:58Z (pid 107390), which re-imported regrasp/object_gate.py with my tested-but-undeployed change: off-height TF samples (tracker table blob (0.55,0.10,0.02)) are dropped WITHOUT resetting the 2.5 s window (before: a moved drill took about 2 min to adopt, 22:33-22:35Z). tests/test_object_gate.py 7/7; new test fails on the old gate. Revert: cp object_gate.py.bak_resetonjump_20261106 object_gate.py + restart the publisher. Murilo told.
- Pad grasp_force is now 50 N (Muse, 23:12Z block). Lift vibration: sim lane (648038c1) says contact, not controller (20 N needs mu >= 0.32 on 1.3 kg; sim lift clean, 0 faults). Re-test the lift at 50 N; if it still vibrates, look at the controller.
- Sim A/B (648038c1, runs/pf_live_{hand,goal}_t5): with live params both gates are smooth and lift 15 cm, hand err 22 mm; gate choice does not matter with caps < 1.
- Open: binding 223704 grasp is 17 cm from the tracker point at the cloud y-edge (cloud includes table, z min 0.025). Asked Muse.

## RESUME HERE (23:12Z 11-06) - coordination taken over by Muse lane "Grasp e nuvem real"
Coord lane 67087ec6 hit its model quota; this lane now owns this file + the live-chain duties (claims, executor/teleop state, binding validity). Same rules: lab changes only on Murilo's go, robot claims max 5 min, never pkill -f a pattern in your own command.
**Grasp force default 20 -> 50 N (Murilo go, 23:12Z):** pad-close grasp_force reset to 20 N every teleop start (yaml + code default + dashboard slider). Changed, backups `*.bak_grip50_20261106`: lab `fr3_joy_teleop.yaml:57` (50.0), `fr3_joy_teleop.py:139` (code fallback 50.0), `fr3_ros2.sh:24` (comment), `fr3_dashboard.html:67` (slider default 50); repo mirror `zed_compare_vm/.../live_fr3_input_teleop.py:178`. Live (disengaged, motion-free): `ros2 param set /fr3_joy_teleop grasp_force 50.0` on domain 77, verified 50.0, affects the NEXT grasp goal only. `GRASP_FORCE=` env override still wins. Scripted `grasp_test.sh` path untouched (closes at 60 N in grasp_gripper.sh).
**Live snapshot 23:12Z:** loop TRACKING trusted (static ~5 mm, n~6.3k), binding `20261106_223704_drill_loop_regen` (bound 22:30Z-ish, depth 0.02, flip 180), TF publisher at ~20 Hz on that binding, no regrasp lock, teleop disengaged. Binding 215632 from the 22:20Z block has since been superseded by a loop REGEN.

## RESUME HERE (22:20Z 11-06) - supersedes the 21:06Z block below for the live chain
**Goal:** real FR3 drill grasp; Murilo drives the pad, the cuRobo MPC adds a 6D ATTRACTION (not a takeover) toward the loop's grasp.
**Live chain now:** loop binding (regrasp/state/grasp_binding.json) -> vm-viewer `regrasp/grasp_target_from_tf.py` (TF target_object + offset, hand_R, 20 Hz; jump gate `regrasp/object_gate.py`: holds the bind position, adopts a new one only if arm still 2.5 s, TF stable 1 cm, hand > 15 cm from grasp, |z - bind z| <= 2 cm) -> lab relay `codex-fr3-live-mpc:/tmp/grasp_target_relay_d77.py` -> MPC.
**MPC field (deployed 22:16Z, Murilo "set it up, let's test it"):** lab node /home/murilo/franka_ws/hub-curobo/node/curobo_mpc_node.py patched by `fr3_real_repro/regrasp/lab/patch_pf_hand.py` (repo vm_real/node identical in that function; backups *.bak_pfhand_20261106). New params, defaults = old behaviour (quest-fr3-curobo-sim also loads this node): pf_dist_from goal|hand, pf_max_weight_rot, pf_weight_rate. Host /tmp/pf.yaml: grasp, dist_from hand, radius 0.30, full 0.03, max_weight 0.65, rot cap 0.85, rate 1.5/s, rot_m_per_rad 0.03 (backup /tmp/pf.yaml.bak_goaldist_20261106 = old goal-distance setup). goal = op + w(d_hand)(grasp - op); caps < 1 so a still stick settles partway (20 cm goal -> 8.3 cm hand; 10 -> 3.5). Offline model `regrasp/tools/pf_hand_model.py` (statics = equilibria count, must be 1; replay of the 22:00Z run csv regrasp/state/run_2201.csv). Old setup had 3 equilibria (snaps). NOT yet tested by Murilo. Log line: "Grasp attraction w= w_rot= d= from hand".
**Goal relay fixed (21:34Z):** fr3-ros2 /tmp/direct_goal_fwd.py converts /teleop/command_tool (fr3_hand_tcp) -> fr3_hand (repo retired_1104_direct/direct_goal_fwd.py). Re-copy after any fr3-ros2 restart.
**Robot segmenter (21:50Z):** franka-seg-zed1/2 THRESHOLD 0.06 (was 0.15: drill erased within 15 cm of the hand). Relaunch recipe in the 21:57Z section.
**Findings worth keeping:** tracker /tracking_3d_point = ONE ZED1 pixel + its depth; bad points lie on the ZED1 ray behind the drill (0.2-3 deg), i.e. depth fell through to the table. 22:00Z run (FK of csv joints): hand reached 1.5 cm / 10.6 deg of the grasp; pad goal rotated 30 deg away from the grasp; 8 cm sideways jump 2 s later = likely the knock.
**OPEN (next):** (1) drill state unknown: Murilo knocked it ~22:01Z; loop says OCCLUDED (no regen) while the viewer sees the top at the same height; binding 215632 (standing) may be wrong; gate shifted target to (0.380,-0.082,0.312). Ask Murilo; do not close on it if lying. (2) Murilo's pad test of the hand-distance field. (3) reuse bar tip >= 3.0 cm rejected the old grasp at 2.7 cm (option: 2.5). (4) tracker single-pixel depth (shared tracker; fix = mask-median or drill cloud). (5) Muse lane not told anything since 21:20Z (Murilo: Claude lane only); Muse's flatness ranking went live with the 21:54Z loop restart. (6) still open from before: JTC 0.08 rad loaded-stage aborts, executor should start both forwarders, /ssh/id_lab in vm-viewer.
**22:31Z fix (Murilo: "the grasp was already generated once ... the object is the same"):** pad test 22:19-22:23Z: attraction worked (w 0.65/0.85, hand 1-3 cm from grasp) but grasp_target_from_tf went quiet at every gripper close until the loop's NEXT binding (30-75 s) = "sometimes it doesn't attract". Now it resumes the SAME binding as soon as the gripper reopens (lock HELD_OBJECT still waits for a new binding). Restarted 22:30:55Z, pid 91486 in vm-viewer; backup grasp_target_from_tf.py.bak_quietuntilbind_20261106; old log /tmp/grasp_target_from_tf.log.prev_2231. Second cause, not changed: rotation term (0.03 m/rad) pushes d past the 30 cm radius when the hand is ~100 deg off (option: 0.01 m/rad, radius 0.40).
**Rules in force:** robot claims max 5 min, renew while Murilo drives, release when idle; lab changes only on Murilo's go; never pkill -f a pattern in your own command.

## RESUME HERE (21:06Z 11-06) - read this first; details in the dated sections below
**Goal (Murilo):** real FR3 grasp of the drill with the cuRobo MPC 6D attraction (pf_mode grasp) toward a grasp that a loop keeps current.
**Live chain:**
SAM2 tracker (vm-tracker-prod) -> TF target_object (vm-target-bridge, latched through occlusion)
-> vm-viewer `regrasp/grasp_target_from_tf.py` (/grasp_target = TF + binding offset, hand_R, 20 Hz, quiet after a gripper close until a new binding; log /tmp/grasp_target_from_tf.log)
-> lab relay `codex-fr3-live-mpc:/tmp/grasp_target_relay_d77.py` (d8 -> d77)
-> MPC (pf params in lab HOST file /tmp/pf.yaml, bind-mounted: pf_mode grasp, pf_radius_m 0.35, pf_full_radius_m 0.10, pf_rot_m_per_rad 0.03; edit in place on the host; backups /tmp/pf.yaml.bak_*)
-> fr3-ros2 `/tmp/direct_goal_fwd.py` (teleop /teleop/command_tool -> MPC wrist_goal) + `/tmp/direct_topic_fwd.py` (MPC preview -> JTC, engaged-gated).
- **MPC restart recipe** (pad disengaged): `docker restart codex-fr3-live-mpc` kills both docker-exec children. Restart them: `docker exec -d codex-fr3-live-mpc bash -c "/isaac-sim/python.sh /tmp/esdf_proxy_d77.py > /tmp/d77.log 2>&1"` and the same for grasp_target_relay_d77.py. Then `ros2 service call /fr3_joystick_mpc/set_active std_srvs/srv/SetBool "{data: true}"` (domain 77 LOCALHOST, in fr3-ros2).
- **fr3-ros2 restart wipes /tmp**: re-copy both forwarders (repo grasp_loop/tools_1105/direct_topic_fwd.py, retired_1104_direct/direct_goal_fwd.py) and start them in fr3-ros2 (jazzy + /ros2_ws, ROS_DOMAIN_ID=77, LOCALHOST). TODO: make the curobo executor start them.
**Regrasp loop (Muse lane d8b74888..., "Grasp e nuvem real"):** fr3_real_repro/regrasp/ (DESIGN.md), runs in vm-viewer, log /tmp/regrasp_loop_live.log (host).
- BINDER only, never publishes. REGEN (GraspGen server on lab GPU1, persistent authorized infra, ~0.5 s MPC stall per inference) / SE(2)-ICP REUSE -> grasp_loop/runs/<ts>_drill_loop_regen, atomic grasp_binding.json (monotonic, never an older run).
- Gates: no inference while the arm moves; nothing while regrasp/state/regrasp.lock is HELD_OBJECT / RUN_ACTIVE.
- Selection: tip >= 3.0 cm, vertical ladder 15/25 deg/rest, then pts/margin. Contact-flatness filter in progress (replay only).
**Scripted grasp:** `cd fr3_real_repro/grasp_loop; ./grasp_test.sh run 0.0 bound "--cart-step 0.15 --wp-min-time 6 --depth 0.01"`
- `bound` = the loop's current grasp. Refuses if the drill drifted > 3 cm. ENTER per stage. 60 N close; stops on a gripper failure. Lift 10 cm.
- `unlock` clears HELD_OBJECT after a manual release. Planner = grasp_plan_lib.py (shared).
- Arm not near home -> `./fr3_ros2.sh real-home` on the lab (lab dir /home/murilo/franka_ws/lab) or the dashboard Home.
**Known limits / open:**
1. JTC path tolerance 0.08 rad (lab fr3_arm_jtc.yaml): loaded single-trajectory stages abort (S4 stalled 0.24 rad at 20:56Z). Fix: finer waypoints/slower loaded stages, or a looser tolerance. Recover with the pad.
2. Pad test of the 6D field: test 1 had w~0 (orientation term); test 2 felt like a "fight" (partial blend + teleop max_lead 3 cm). Retuned to full pull at 10 cm (21:05Z), NOT yet tested by Murilo.
3. MPC node drift: repo vm_real/node != lab /node (lab has _normalize_esdf_values; repo has virtual_box/mpc_override param); pf code equal. Merge needs Murilo's go.
4. vm-viewer holds the lab ssh key at /ssh/id_lab: remove before sharing the container.
**Results today:** first full real grasp cycle 18:37Z (reuse152, 60 N). Holds at tip 3.0-4.0 cm; slips at 1.8-2.0 cm. Calib = robotfit7 + dz (zed1 +4.4, zed2 +4.7 mm), deployed 16:46Z.
**Lessons:** never pgrep/pkill -f a pattern contained in the calling command (it bit us 3x today); don't edit a running bash script; the dashboard :8089 allow-list includes matheuzu11@gmail.com (Murilo's browser login).

## 21:35Z: pad "attracts and bounces back" = two input bugs, not the pf formula (fixed)
- Bug 1, target jumps: tracker target_object TF jumped drill <-> table (up to 29 cm, also with the arm still: points on a line drill -> (0.58,0.19,0.02) = mask with a second blob), and /grasp_target followed. MPC log d 0.17 -> 0.58 -> 0.16 m in 0.5 s steps. Fix (VM, live): regrasp/object_gate.py in grasp_target_from_tf.py: hold the bound position; adopt a new one only if the arm is still 2.5 s, the TF is stable (1 cm), the hand is > 15 cm from the grasp, and |z - bind z| <= 2 cm. tests/test_object_gate.py 6/6. Backup grasp_target_from_tf.py.bak_nogate_20261106.
- Bug 2, frame: /teleop/command_tool = fr3_hand_tcp pose, MPC ee_link = fr3_hand; the retired bridge converted (goal_math.tcp_to_hand), direct_goal_fwd.py did not, so the pad goal sat 10.34 cm off along the approach (and every Start pulled the hand 10 cm along it). Fix: retired_1104_direct/direct_goal_fwd.py converts with TF fr3_hand_tcp->fr3_hand (== goal_math to 4e-16). DEPLOYED 21:34Z on Murilo's go (fr3-ros2 /tmp, pid 952693, log "hand in TCP frame: t=[0,0,-0.1034]"); backup /tmp/direct_goal_fwd.py.bak_notcp_20261106. Re-copy THIS version after an fr3-ros2 restart.
- Drill moved ~6.5 cm / 13 deg yaw during/after the test (viewer live re-anchor); binding 210308 is stale. Muse loop crashed 21:23:52Z in REUSE (grasp_plan_lib.py:203, 1-D mesh array); not restarted, Muse not told (Murilo: Claude's lane only).

## 21:57Z: robot-mask leak fixed, loop back up (Murilo go)
- franka-seg-zed1/2 (lab, Murilo's containers, run_segmenter_real.sh) relaunched with THRESHOLD=0.06 (was 0.15: every depth pixel within 15 cm of a robot sphere was zeroed, so the drill vanished from the cloud near the hand and the tracker's single-pixel depth fell through to the table along the ZED1 ray; bad points were on that ray to 0.2-3 deg). Masked fraction at home zed1 12.4% -> 8.7%. nvblox occupied 235k -> 233k after a brief 303k restart transient: no robot ghosts. Revert: docker stop franka-seg-zedN; THRESHOLD=0.15 GPU=0 ./run_segmenter_real.sh zedN (in /home/murilo/nvblox_real/isaac-sim_ws/franka_mpc/real).
- regrasp/loop.py: _ensure_mesh() also before REUSE (the 21:23:52Z crash); backup loop.py.bak_reusemesh_20261106; tests/test_loop.py all ok. Restarted 21:54:11Z (setsid docker exec vm-viewer ... loop.py --anchor-run binding >> /tmp/regrasp_loop_live.log). NOTE: this restart also put Muse's undeployed flatness ranking (angle rank) live.
- REUSE of 210308 idx6 was rejected at tip 2.7 cm (< 3.0 hard bar, 3 mm short) -> REGEN -> BOUND 20261106_215632_drill_loop_regen idx112: problems none, 343 pts, tip 3.1 cm, tilt 16.3 deg, margin 0.18 rad, live cloud offset 0.2 mm.

The heavy calibration work is done by the Muse lane "Calibração câmeras robô real" (d8b7488839304031adbd7b70025e578d).
This lane (67087ec66b4f4c0a94aa0ab726827dca) coordinates and keeps this file current.
This file supersedes the calibration part of HANDOFF_2026-11-05_night-grasp.md ("RESUME HERE" 23:10Z/23:20Z bullets).

## Deployed calibration: UNCHANGED (robotfit7, Murilo go 2026-11-05 ~01:25Z)
- `fr3_real_repro/zed2_align/zed{1,2}_extrinsic_20261105_robotfit7.json`, depth_scale zed1 0.9767 / zed2 0.99589. The VM fan-ins pre-apply these (DEPTH_SCALE env), so the viewer uses scale 1.0; any viewer_calib.json override must use 1.0 too, not the JSON value.
- Viewer on live TF (`:8791/grasp.json` live.zed2_T = "tf"); merged drill cloud live (live_mask_depth+zed2_box, ~1.8-2.9k zed2 pts).

## 2026-11-06 Muse lane results (live, no arm motion)
- Live tag re-anchor T2-live (`/tmp/thm/live_tag/T_zed2_live.npy` in vm-viewer) passed the tag gates, was deployed as a viewer_calib.json override, made the drill seam WORSE (8.5 -> 25.2 mm median; ray test 7.9 vs 1.4 mm), and was rolled back within minutes. The offline joint-tag `..._CANDIDATE.json` (seam 19.2 mm, never deployed) is likewise withdrawn: both are marked superseded / DO NOT DEPLOY.
- IMU gravity test (`/tmp/live_imu_test.py` in vm-viewer, domain 8; implied gravity from the extrinsic vs the ZED IMU, with the cam->IMU rotation from `/zedN/zed_node/left_cam_imu_transform`):
  | | angle to IMU gravity |
  |---|---|
  | zed1 T1 deployed (control) | 0.35 deg |
  | zed2 deployed robotfit7 | 1.56 deg |
  | zed2 candidate | 6.17 deg |
  | zed2 T2-live | 5.75 deg |
- Root cause: zed2 sees the tag obliquely, so IPPE picks a plane normal ~6 deg off. The tag-anchored fit then twisted a good T2 by ~8 deg. Tag paper also drifts 17-37 mm between sessions (the "zed1 wobble" was paper motion). The NN seam metric is blind along the drill's long axis.
- Still valid: T1, deployed T2 (~1 deg / ~8 mm at the drill). Table step: Muse tag-center estimate ~+11 mm, but UNRESOLVED (see "Z / TABLE STEP").

## Caveats (coordination lane, 11-06)
- The IMU test checks only 2 rotation DOF (roll/pitch about gravity). It does NOT check yaw about vertical or translation. The yaw/translation of T2 rest on the robot-mesh ICP (arm residual 1.9-3.3 mm) and the ~8 mm drill seam.
- The IMU test assumes fr3_link0 z = gravity. zed1 at 0.35 deg supports that (a level base).
- So the offline table-plane tilt (11-05 23:10Z: zed1 2.66 deg, zed2 1.94 deg; zed1 only 58% inliers) is NOT camera tilt. It comes from ZED depth warp/bias on the table at range. Do not use the table plane to correct rotation.
- Sim lane (HANDOFF_2026-11-06_sim-isaac-replica.md): with perfect truth, grasping still fails on the GraspGen sampler/ranker, so calib is not the grasp blocker.

## Z / TABLE STEP: NOT FIXED (correction 11-06, after Murilo asked)
- The verdict below covers rotation and lateral error ONLY. Height (Z) is unresolved, and no Z correction was ever applied to robotfit7.
- Table step (drill table vs Franka table, base on the Franka table = z 0): the estimates disagree. ZED1 split 29 mm (withdrawn); seeded patches +3.9 mm (inside the ~9 mm method floor, normals 3.6 deg apart); tag-center ~+11 mm (Muse, 11-06). Murilo says the step is visibly real. The night-grasp CURRENT RESUME block says "unresolved, use neither estimate", and that stands.
- Cross-camera Z on the shared table: zed1 minus zed2 median -8 mm, rms 19 mm. Table height at the base reads +6 mm for zed1 and +45 mm for zed2 (depth-based, warp-affected).
- Grasp relevance: S3 "hand looked too high" is unexplained, and the fingertip was only 1.8 cm below the cloud drill top. That is the size of the Z ambiguity, so the "z-offset" grasp failure may be partly calibration.
- Muse's per-camera breakdown of the +11 mm (cap_1104c, 11-04 20:09; tag-center PnP -> link0 through each camera's OWN extrinsic, no cross-anchor):
  - zed1 +15.9 +- 6 mm. This is the serious number: PnP unambiguous, plane 0.27 deg from horizontal, zero-check -2.2 mm, T1 rotation IMU-validated. The weak link is T1 translation.
  - zed2 +6.0 +- 18 mm. The center is robust, but deployed-T2 translation swings ~150 mm leave-one-out over its 4 constraining poses.
  - So "+11" is only a midpoint. The 10 mm zed1/zed2 gap is the translation-uncertainty measure.
- Grasp: the +30 mm grasp offset is proven and enough to explain the miss on its own, but a Z bias of +-10-15 mm adds to it. Ruler + touch separates the two.
- PLAN (both lanes agree):
  1. RULER first: Murilo measures the drill-table step (~2 min, +-1-2 mm, no claim). This gives h directly.
  2. TOUCHES, with Murilo's go + a named e-stop person + a franka-robot claim. The Franka table is finger-unreachable near the base, so touch the drill table at 2 spots (far + near), then the drill top LAST, because it may shift the drill (re-seed the tracker after).
     - Contact detection: the 11-05 probe was parked for false triggers in air at the threshold edge and a wrong force sign. Use force-curve logging, 2-3 mm JTC micro-steps near the surface, eyes-on confirmation, and a 10 N monotonic-rise rule. The script has a Ctrl+C hold.
     - Before any scripted move: VERIFY teleop is disengaged and the input source is joystick (lesson from the 11-06 night ghost drive).
  3. FIT: Z only, one constant tz bump per camera (2+ spots) in viewer_calib.json, a reversible hot-reload. NEVER fit tilt from touches.
  4. RE-VALIDATE: IMU (tz does not change it), tag centers converge across cameras, seam shrinks. Then a real grasp close as proof. Deploy only with Murilo's go.

## VERDICT 11-06 (both lanes agree): rotation/lateral DONE for now (NOT Z, see above); further checks are optional
- Error at the drill is ~5-8 mm (rays/seam), against an 80 mm gripper with a 15-60 mm finger channel. Every grasp failure to date traces to planning/execution (z-offset, S2 timing, forwarder slew), never to calib.
- Scope limits: (1) the seam p90 tail is ~35 mm, so some zed2 points sit well off. The cloud recipe's outlier removal handles them, but the grasp ranker must not assume a clean cloud. (2) Yaw is only bounded together with translation and is valid for the drill AT THIS SPOT. If grasping moves to other table areas (longer lever arm), revisit with the rungs below.

## Open / next (none needs a calib redeploy)
1. Yaw/translation ground truth for zed2, if wanted. No pure yaw number exists. Muse lumped bounds: yaw+translation ~20 mm-equiv at the tag (corners minus IMU-pinned roll/pitch); lateral ~5 mm at the drill (centroid rays 1.0-1.4 mm).
   - Passive parked-arm capture: gate it on arm visibility in zed2 first. Checked 11-06 ~13:25Z (read-only, `/franka_seg_{zed1,zed2}/robot_mask`, domain 8): zed1 31034 px (13.5%), zed2 only 1998 px (0.87%). That is a sliver, too little baseline for yaw, so a passive capture at the current park is NOT worth doing.
   - Next cheapest rung (Muse): multi-spot tag. A human moves the tag paper to 3+ spots spread across the drill table, with no robot motion and no claim (pure subscribers). Compare tag CENTERS only, zed1 vs zed2 (never the zed2 normals). The spread gives a yaw baseline.
   - Last rung: an arm sweep through zed2's view, 4+ poses spread across the drill table (Murilo go + named e-stop person + franka-robot claim), then fit_robot7.py with zed2 poses weighted.
   - Table planes: dropped for good (Muse agrees). Depth planes forced to z=0 bake depth warp into the extrinsics. Only safe table use: tag centers as weak priors.
2. Any new calib deploy needs Murilo's go. Validate with the zed1-anchored tag CENTER (robust; never fit to zed2 tag plane NORMALS — IPPE orientation bias), the IMU tilt (<=2 deg), and the drill seam + centroid rays. A rotation-only "fix" that worsens the drill seam is refuted, not improved.

## Joystick touch session 11-06 (Murilo drives; this lane did the setup on his go)
- ~13:45Z setup: old teleop (pid 2302, floor 0.22, input zed) killed; MPC standby; Cartesian ACTIVE, JTC inactive (arm moved ~1 mrad); teleop relaunched with BACKEND=cartesian, INPUT_SRC=joystick, WS_MIN [0.15,-0.45,-0.02], WS_MAX [0.65,0.35,0.70]. RESTORE after: default floor (`./fr3_ros2.sh real-teleop` without WS_MIN), and cuRobo if Murilo wants it.
- Touch 1 (~13:53Z): TCP [0.3583, 0.0090, 0.0375], 3.2 deg from straight down, fingers 79.8 mm, F_ext z -18.5 N (rest ~0.2 N) = contact at the 20 N impedance ceiling. Surface z ~ +37 mm (minus 1-2 mm tcp-vs-tip). Which table: asked Murilo.
- ~13:57Z dashboard 3D robot missing: mesh_server (isaac-build, :8788) takes the robot pose/meshes from --robot-dash 127.0.0.1:8088. The lab forwarder `tcp_fwd_8088.py 8088 8091` pointed at the :8091 sidecar, which has been dead since the fr3-ros2 restart at 00:02Z. Repointed: `tcp_fwd_8088.py 8088 8089` (main dashboard; /config, /stream and /mesh/... verified 200). Still NOT persistent: it dies with a lab reboot.
- Touch 1 identity (Murilo, ~14:00Z): bare drill table, no shim; the drill table is a fixed physical step above the Franka table. So drill table ~ +36 mm (FK), and zed1 reads +21.9 mm there (saved bg, rms 0.8) = zed1 ~14 mm LOW. zed2 is unreliable at that XY (edge zone). Next: touch 2 beside the tag (zed2 clean zone), then a Z-only tz fit per camera (Muse, offline), deploy only with Murilo's go.
- Touch 2 (~14:05Z, beside the tag): TCP [0.5718, 0.2105, 0.0463], tilt 9.0 deg, F_ext z -19.5 N (contact at ceiling, not box-limited). TCP 8.8 mm above touch 1; the tilt may put the lower fingertip up to ~6 mm below the TCP, so Muse computes the fingertip FK. A level re-press was requested.
- Touch 2b (~14:10Z, same spot, level re-press): TCP [0.5839, 0.2133, 0.0391], tilt 0.7 deg, F_ext z -18.6 N; fingertips z 38.8 / 39.4 mm. Surface beside the tag = +39 mm. Touch 2 (46.3) was a tilt artifact: DISCARD. The drill table is flat within ~2-3 mm between touch 1 and 2b, so the step is ~36-39 mm.
- ~14:15Z touch session closed: hand lifted to z 0.23, disengaged, F_ext ~1 N. Teleop relaunched with the DEFAULT box (floor z 0.22, [0.20,-0.35,0.22]..[0.65,0.35,0.70]), input joystick, Cartesian backend. Executor LEFT on Cartesian (cartesian active, JTC inactive, MPC standby); switch back to cuRobo from the dashboard Executor card if wanted. franka-robot released.

## Z RESULT (14:05-14:15Z) - supersedes the "+36-39 mm / cameras 15 mm low" lines above
- Fingertips sit 8.9 mm BELOW fr3_hand_tcp (URDF: TCP at hand z 103.4 mm; finger base 58.4 + finger.dae tip 53.9 = 112.3 mm; Muse). Both lanes had wrongly assumed tip = TCP +-1 mm.
- True surface heights: touch 1 ~27.5 mm, touch 2b ~29.9 mm. Drill-table step h ~ +29 mm (both tables flat; Murilo).
- Cameras at touch-2b XY: zed1 +27.3 (rms 3), zed2 +25.2 (rms 11). Z-only fit (fit_z_touch.py, drop0 8.9): zed1 dz +4.4 +- 2.0 mm (2 touches), zed2 +4.7 +- 6.1 mm (1 touch).
- VERDICT (both lanes): Z CLOSED. The ~4.5 mm common offset is ~2 sigma and within depth-warp/mesh bars, so NOT deployed (candidate in Muse's /tmp only). The grasp miss is back to planning (the proven +30 mm offset >> 4 mm). Revisit only if real grasps show a systematic z offset.
- Lesson: any touch/probe height must subtract the 8.9 mm tip-below-TCP offset (fingers straight down).

## Fused cloud: ZED2 is on the BOX fallback, not the SAM2 mask (found ~15:00Z after Murilo asked)
- :8791/drill.json live_cloud_zed2_src = 'box' (3/3 reads). The grasp.json label "live_mask_depth+zed2_box" is hard-coded (seg_view.py l.610), so it can't tell.
- /zed2/segmentation_mask is live (~1.5 Hz, ~5.7k px), but it arrives ~0.96 s after its frame stamp while depth arrives ~0.25 s after. _zed2_points pairs the LATEST depth with the nearest mask under a 1.0 s gate, and depths2 maxlen 5 (~1.1 s) can't reach the mask's frame, so it falls back to the box crop.
- Murilo: no box crops for the cloud (precision). Fix sent to Muse: buffer ~3 s of depth, pair the depth to the newest mask (gate ~0.1 s), no box fallback, true src label, revisit the ZED1-derived ±4 cm box and the 3000-pt integer stride.

## ZED2 mask-only cloud (Muse, ~16:20Z, verified live by RAL)
- seg_view.py `_zed2_points`: depth frame paired to the nearest mask (adaptive gate [0.1,0.5] s), NO box fallback, 4 s hold of the last mask set, floor support+0.01, exact random subsample cap 4000, honest label `live_mask_depth+zed2_mask|none`. Tests test_zed2_mask_gate 10/10, test_drilljson_live3d 6/6.
- Live: src=mask 100% over 60 s, n_zed2 4000 (zed1 ~3.8k), face 35%, seam median 7.5 mm / p90 20.7 mm (was 8.4 / 37.9). Render http://100.84.207.115:8904/z2fused_maskonly.png.
- Left: ~10% flying-pixel halo at the drill's concave edges (both cams); zed1 path has no floor (stray tail to z~0). vm-cable-fanin2 restarted by Muse (cable2 depth was 0 Hz).

## DEPLOYED 16:46Z (Murilo go: "publish the corrected calibration to curobo too"): robotfit7 + dz
- New TF = robotfit7 with only T_base_cam[2][3] raised: zed1 +4.385 mm, zed2 +4.700 mm (the touch Z fit). Optical z zed1 0.633->0.637, zed2 1.036->1.041. Depth scales UNCHANGED (fan-ins untouched).
- Files: repo fr3_real_repro/zed2_align/zed{1,2}_extrinsic_20261106_robotfit7_dz.json; lab zed-build /home/spot_ws/zed_ws/fr3_runtime/recovered_1102/ (same md5).
- Lab zed-build publishers: pid 388996 zed1, 389042 zed2 (real_extrinsics_tf.py), logs /tmp/nvblox_real/extrinsics_zed{1,2}_robotfit7dz_1106.log. Old 358199/358210 killed.
- Verified: VM domain 8 tf2_echo shows the new values. Chain to cuRobo: codex-fr3-live-mpc (use_esdf true) -> /tmp/esdf_proxy_d77.py -> /nvblox_curobo (vm-nvblox-prod, use_tf_transforms) -> new TF. static_tsdf keeps old voxels (4.5 mm < 2 cm voxel); no map clear done.
- viewer_calib.json retired -> calib_robot/viewer_calib.json.retired_ondeploy_1106; viewer now on live TF (grasp.json zed2_T 'tf').
- REVERT: in zed-build kill the two publishers, relaunch real_extrinsics_tf.py with recovered_1102/zed{1,2}_extrinsic_20261105_robotfit7.json (env: humble + zed_ws install, ROS_DOMAIN_ID=8, ROS_LOCALHOST_ONLY=0).
- Sim lane 62f98b58 told.

## GraspGen on mask-only dz cloud + plan READY (16:50Z)
- Muse run grasp_loop/runs/20261106_134738_drill_maskonly (dual2 recipe, 7995 pts, top z 0.2534). PICK idx41 conf 0.835 score 0.645, tilt 12.8 deg, 20 mm deep before --depth.
- Plan (no motion): `./grasp_test.sh plan 0.0 file:///home/nexus/spot-teleop/fr3_real_repro/grasp_loop/runs/20261106_134738_drill_maskonly/grasp_plan_input.json "--cart-step 0.15 --wp-min-time 6 --depth 0.025"`: PROBLEMS none, TCP [0.4134,-0.1963,0.2085], palm 0 / finger 0 / between pads 444, tcp 4.0 cm below top, min obstacle 0.436 m. Log fr3_real_repro/grasp_loop/runs_real/grasp_test_20261106_164942.
- To execute: same line with `run` instead of `plan`. NEEDS: Murilo go + named e-stop person; executor back to JTC (fr3_arm_controller active; now Cartesian); teleop disengaged; claim franka-robot (5 min max, renew per stage).
- 16:54Z dashboard anchor fix (Muse, seg_view.py viewer-only): the grasp.json live re-anchor compared the full live-cloud box with the run's top-8cm box, so mask-only ZED2's far face shifted idx41 +26 mm in y on a static drill. Now top-8cm p2/p98 on both sides (test_prologue_anchor.py 4/4). Verified 3 reads: dashboard pick == file pick [0.4066,-0.224,0.3337], no offset. The dashboard shows the GraspGen pick; the executed grasp adds --depth 0.025 along the approach (TCP [0.4134,-0.1963,0.2085]).

## Dashboard ZED2 mask panel (17:05Z, Murilo asked)
- Muse: seg_view.py GET /snap2.jpg = ZED2 RGB (/vm/cable2/rgb) + /zed2/segmentation_mask overlay + mask-age text, re-rendered on new mask (<=2 Hz); test_snap_overlay 6/6.
- RAL: lab isaac-build /tmp/nvblox_real: mesh_server.py route /segsnap2 -> 10.0.0.3:8791/snap2.jpg; lab_dashboard.html panel "Drill mask ZED2 · live" next to ZED1 (one JS chain for both, ZED2 polled every 700 ms). Backups *.bak_zed2seg_20261106. Repo/scratch copies in the RAL scratchpad dash/.
- mesh_server restarted 17:04:52Z (pid 2989582, same args/env, log /tmp/mesh_server_8788.log; restart helper /tmp/start_mesh_ral.sh in isaac-build). ~1 min outage: my first kill used pgrep -f, which matched its own bash -c and killed the restart shell -> use a script file, not pgrep -f inside bash -c. Verified /, /segsnap, /segsnap2, /graspjson, /drilljson 200 on 8788, 8795 and VM tunnel 18089.

## Executor switch failure 17:13Z ("Falhou: Backend request failed") — fixed by stopping my CLI teleop
- Cause: my touch-session teleop (fr3_ros2.sh real-teleop, `fr3_input_teleop.py`, pid 626194, 14:04Z) held UDP 47813. The dashboard switch only kills `fr3_input_teleop_review`, so it skipped mine; its new `_review` teleop died with "Address already in use" (/tmp/teleop_curobo.log, /tmp/teleop_rollback_cartesian.log in fr3-ros2), _await_backend timed out, rollback ran (verified after: Cartesian active, JTC inactive, MPC standby, mode 2, no motion).
- Message bug in fr3_web_dashboard.py _control_backend: `'...; '+detail%target` (precedence) raises TypeError on the rollback message -> generic "Backend request failed". Same pattern in 'controller swap ... moved hardware state'. NOT fixed (needs a launcher restart of :8089; Murilo's call).
- 17:17Z (franka-robot claimed+released): stopped pid 626194/626173 (disengaged checked). No teleop now; port 47813 free; Cartesian still active. Next: Murilo presses cuRobo on the Executor card (it launches its own teleop).
- Lesson: never leave a CLI `real-teleop` running after a session; the dashboard owns the teleop.

## Real grasp 17:45Z (idx41 upright) + lying-drill replan (18:03Z)
- Murilo at e-stop, executor cuRobo/JTC. idx41 --depth 0.025: S1/S2 OK, close 25 N SUCCEEDED, S3 lift OK, drill slipped and fell (aborted before S4; log runs_real/grasp_test_20261106_174512). Pads straddled ~54 mm, ~1 cm off-center, 4 cm under top. grasp_gripper.sh close force 25 -> 60 N (backup .bak_25N_20261106). Arm sent home by Murilo; franka-robot released.
- grasp_plan.py (backup .bak_preflip_20261106): flip chosen so PRE is reachable too; new PROBLEMS: any waypoint < 0.10 rad from an FR3 joint limit; < 50 cloud pts between pads.
- Lying drill: idx38 (run 20261106_145517_drill_lying) unreachable safely (joint 4 at limit). 200-sample rerun runs/20261106_150037_drill_lying2, 19 alts batch-planned. CHOSEN idx152 --depth 0.01: PROBLEMS none, 411 pts between pads, tcp 3.2 cm under top, margins >= 0.68 rad, near-home config. Runner-up idx52.
  Command: ./grasp_test.sh run 0.0 file:///home/nexus/spot-teleop/fr3_real_repro/grasp_loop/runs/20261106_150037_drill_lying2/alt/grasp_plan_input_idx152.json "--cart-step 0.15 --wp-min-time 6 --depth 0.01"
- Auto-regrasp loop: spec agreed with Muse (accumulator in the loop, SE(2) ICP fast path, fixed anchor, occlusion latch held-not-trusted + revalidation, stale-safe HELD_OBJECT lock, persistent GraspGen server with MPC p99 check, numeric gates). NOT built: waiting for Murilo's go.

## FIRST FULL REAL GRASP CYCLE 18:31-18:37Z (Murilo at e-stop) — Murilo confirmed: it lifted the drill
- Grasp: lying2 idx152 REUSED on the dropped drill via SE(2) ICP lying2 -> lying3 (yaw -0.3 deg, t [-11,+8] mm, fitness 0.96, RMSE 4.5 mm), checked against the fresh lying3 cloud: grasp_loop/reuse/grasp_plan_input_reuse152_lying3.json, --depth 0.01, lift 10 cm, close 60 N. Dashboard pointer runs/20261106_152450_drill_lying3_reuse152.
- Log runs_real/grasp_test_20261106_183106: S1 0.010, S2 0.011, close SUCCEEDED, S3 lift 0.052 (sag under load, inside the 0.12 bar), S4 0.023, open SUCCEEDED, S5 0.011, S6 0.012, DONE. Drill back on the table (drill.json ~[0.345,-0.138]). franka-robot released.
- Runner changes today (backups *.bak_*_20261106): 60 N close; stop on a failed gripper command (parses "Goal finished with status: SUCCEEDED" only, since the result also prints error: ''); `s` to skip the re-close when resuming from S3; --lift default 0.10; S3/S4 arrival bar 0.12; planner flip check at PRE, joint-limit margin >= 0.10 rad, >= 50 pts between pads.
- Earlier attempts: 17:45Z idx41 upright at 25 N slipped; 18:06Z idx152 lifted only 1.7 cm (0.05 bar + 5 cm target), then a re-close failed and released the drill. set_load is rejected while the controller runs ("Move" mode).

## ZED2 nvblox carve now SAM2 (18:41Z, Murilo go)
- vm-mask-sync2 = vm_mask_sync.py on /zed2/segmentation_mask + /vm/cable2/depth -> /vm/target_mask2 (--max-age 2.0 --release-after 10 --dilate 12); launcher grasp_loop/run_vm_mask_sync2_seg.sh. Was the drill-box projection (vm_mask_sync2.py --pad 0.015 --dilate 5).
- Verified: 19 Hz depth-stamped masks, ~1.5 Hz SAM2 input (46 masks/30 s), mask age ~0.3 s, coverage 4.1% (dilated), bbox x 205-318 y 9-153 = drill in the ZED2 image. nvblox-prod untouched (only pre-existing truncation warnings).
- Rollback: docker rm -f vm-mask-sync2 && docker rename vm-mask-sync2-box vm-mask-sync2 && docker start vm-mask-sync2 (old box container kept, stopped).
- Caveat: when the drill moves, the ZED2 mask lags (~0.7-2 s) and a few frames may integrate drill voxels into the static map.

## Regrasp loop + 6D attraction: build started 18:45Z (Murilo go)
- Design: fr3_real_repro/regrasp/DESIGN.md (package layout, state machine, contracts, Phase B). Muse builds Phase A modules (regrasp/*).
- RAL done:
  - planner split into grasp_loop/grasp_plan_lib.py (plan_grasp/PlanError/load_grasp/load_mesh; bit-identical to the old CLI on 4 cases; backup grasp_plan.py.bak_prelib_20261106);
  - grasp_test.sh lock /tmp/regrasp.lock (RUN_ACTIVE + 10 s heartbeat, HELD_OBJECT close->open, `unlock` mode; abort test: set during, removed after, no motion);
  - Phase B relay regrasp/lab/grasp_target_relay_d77.py (not deployed; filter test OK).
- Found: the live MPC node already has pf_mode (default 'object' = 3D affordance pull); 'grasp' = 6D pull to /grasp_target (fr3_hand in fr3_link0). Phase B = relay + MPC relaunch with -p pf_mode:=grasp (never 'off'): needs Murilo's go + e-stop.

## PHASE B LIVE 19:08Z (Murilo "go ahead, do it"): cuRobo MPC 6D grasp attraction
- Planner FK fr3_hand == TF fr3_hand (0.00 mm, 0.000 deg), so /grasp_target = plan grasp_pos + hand_R is the MPC ee_link pose.
- MPC: lab host /tmp/pf.yaml (bind-mounted into codex-fr3-live-mpc; edit ON THE HOST in place, container root can't write) pf_mode 'off' -> grasp (backup /tmp/pf.yaml.bak_off_20261106). `docker restart codex-fr3-live-mpc` 19:07:43Z -> log "Attraction field: grasp {pf_radius_m 0.25, full 0.02, max_w 1.0, rot 0.1 m/rad, timeout 0.5 s}", standby -> set_active true while disengaged (preview within 0.03 rad of the joints).
- Restarted inside codex-fr3-live-mpc (the restart kills docker-exec children): /tmp/esdf_proxy_d77.py (log /tmp/d77.log) and NEW /tmp/grasp_target_relay_d77.py (log /tmp/grasp_target_relay.log; source regrasp/lab/). ESDF grid updates again, with the usual intermittent timeouts.
- Interim target: vm-viewer `regrasp/interim_grasp_target.py grasp_loop/reuse/reuse152_now_plan.json --max-min 30` (log vm-viewer:/tmp/interim_grasp_target.log). Publishes only while the gripper is open, EXITS at the first close or HELD_OBJECT. The plan is reuse152 re-anchored by regrasp/tools/reuse_shift.py (yaw 3.0 deg, t [-5.7,-2.9] mm, fitness 0.95, RMSE 5.0 mm), PROBLEMS none, 448 pts between pads.
- Verified: relay in=out, MPC logs "Grasp attraction w=0.00 d=0.458 (dp=0.393 m, ang=37.5 deg)" at home.
- ROLLBACK: on the lab host `cp /tmp/pf.yaml.bak_off_20261106 /tmp/pf.yaml` (cat > to keep the inode), docker restart codex-fr3-live-mpc, re-start the esdf proxy, set_active true while disengaged.

## Regrasp loop Phase A built + dry-run (19:24-19:31Z, Muse)
- regrasp/: 9 modules + server/graspgen_server.py (select.py renamed selection.py), suite 74/74 incl. replays (bootstrap->idx152, static hold, 13 mm hold, +20 mm REUSE, tipped REGEN, lock).
- Dry-run on live data (publisher OFF, scratch runs dir): bootstrap REGEN idx78 (score 0.63) -> TRACKING trusted (static ~1 mm), ~2 s cloud dropout handled as OCCLUDED hold. One-shot GraspGen 13.8 s wall (infer ~1.6 s).
- Incident: an earlier dry-run instance wrote a regen run into the REAL runs dir and flipped the dashboard for ~6 min; Muse removed it, and the dashboard is verified back on lying3_reuse152.
- KEY HYGIENE: the lab ssh key was copied into vm-viewer at /ssh/id_lab (600, not in the image) for the loop's GraspGen client. REMOVE it (`docker exec vm-viewer rm -rf /ssh`) before vm-viewer is ever committed/exported/shared, or when the loop moves to its own container.
- /grasp_target: only the interim publisher (vm-viewer pid 45339, started 19:09Z, --max-min 30 -> ends ~19:39Z). Next: GPU1 server p99 check (Muse, lab-gpu1 claim), then the publisher handoff to the loop.
- 19:40Z GraspGen server on lab GPU1 KEPT (Muse measured): +1.2 GB VRAM. MPC step cadence idle p99 281 ms; 6 back-to-back infers -> p99 551 / max 592 ms, 9 gaps > 400 ms, no skipped steps; quiet p99 277. Each inference ~ one 0.5 s MPC stall. Treated as PERSISTENT AUTHORIZED INFRA (like nvblox/tracker, no claim). Gate requested: no GraspGen inference while the arm moves (max |dq| > 0.02 rad/s over 1 s); REUSE still allowed. Revisit (MPS / GPU0 / disengaged-only) if teleop stalls are felt.
- 19:39Z interim publisher exited at its 30 min cap; /grasp_target is handed to the loop (Muse starts it with --publish-target on the real runs dir after the gate).

## 20:11Z SIMPLIFIED (Murilo: reuse the tracker pipeline): /grasp_target = target_object TF + bound grasp offset
- The loop's own publisher flickered: it treated ZED2-unpaired ticks as occlusion, so /grasp_target was OFF ~80 % of the time, with gaps up to 4 s. It was removed.
- Now: vm-tracker-prod (sam2_tracker, zed1 + zed2 secondary) -> /tracking_3d_point -> vm-target-bridge -> TF fr3_link0->target_object (20 Hz, latched through occlusion) -> vm-viewer regrasp/grasp_target_from_tf.py -> /grasp_target = TF + offset, hand_R (20 Hz) -> lab relay -> MPC pf_mode grasp.
- Binding regrasp/state/grasp_binding.json (regrasp/tools/bind_grasp.py PLAN.json; later the loop rebinds on REGEN/REUSE): offset = plan grasp_pos - target_object at bind. Current: run 20261106_195756_drill_loop_regen idx49 (+0.01), offset [0.016,-0.053,0.073].
- Publisher gating: gripper open only; first close or lock HELD_OBJECT -> quiet until a NEW binding (bound_at change).
- Measured 60 s at rest: 19.1 Hz, max gap 0.067 s, 0 gaps > 0.5 s; relay in=out; MPC "Grasp attraction w=0.00 d=0.496" at home.
- Restart: docker exec -d vm-viewer bash -c 'source /opt/ros/humble/setup.bash; export ROS_DOMAIN_ID=8; exec python3 /home/nexus/spot-teleop/fr3_real_repro/regrasp/grasp_target_from_tf.py > /tmp/grasp_target_from_tf.log 2>&1'
- 20:17Z FOUND + FIXED: the fr3-ros2 restart at 00:02Z wiped /tmp/direct_topic_fwd.py, so the cuRobo direct path (MPC preview -> JTC) had NO forwarder all day (Start on cuRobo would not move the arm; the 11-06 executor switch only checks controllers). Restored from the repo (grasp_loop/tools_1105/direct_topic_fwd.py, the 11-05 stale-slew fixed version) into fr3-ros2:/tmp and started (log /tmp/direct_topic_fwd.log). Silent until engaged. It will vanish again on the next fr3-ros2 restart: make it part of the curobo backend start (dashboard/fr3_ros2.sh), TODO.
- 20:22Z also restored the GOAL RELAY /tmp/direct_goal_fwd.py (/teleop/command_tool -> /fr3_joystick_mpc/wrist_goal, engaged + fresh only), wiped by the same 00:02Z fr3-ros2 restart (lab host copy /tmp/direct_goal_fwd.py == repo retired_1104_direct/direct_goal_fwd.py). Without it the stick did nothing on cuRobo (Murilo's first pad test). Started while disengaged. TODO: the curobo backend start must (re)launch BOTH /tmp/direct_goal_fwd.py and /tmp/direct_topic_fwd.py from the repo, not rely on /tmp.
- 20:27Z pad test 1: the field never pulled. The hand arrived ~94 deg off in orientation (mostly yaw about vertical), so d = dp + 0.1 m/rad * ang carried 0.16 m from rotation alone; the closest was dp 7.8 cm, d 0.242 -> w ~0.001. Retuned (Murilo test, MPC restarted while disengaged): host /tmp/pf.yaml pf_mode grasp, pf_radius_m 0.35, pf_rot_m_per_rad 0.03 (backup /tmp/pf.yaml.bak_grasp025_20261106). ESDF proxy + grasp_target relay restarted; MPC set_active true while disengaged.
- 20:36Z "user not allowed" on the :8089 dashboard: Murilo's browser tailnet login is matheuzu11@gmail.com, but the watchdog-relaunched dashboard (after the 00:02Z fr3-ros2 restart) ran the launcher default remote_users=murilo.mv4321@gmail.com (the 11-05 two-login fix had only been passed by hand). Fixed persistently: lab lab/start_remote_dashboard.sh default USERS=murilo.mv4321@gmail.com,matheuzu11@gmail.com (backup .bak_users_20261106); killed the dashboard (pids 667/720 in fr3-ros2), the watchdog relaunched it in 20 s with both logins; teleop untouched. Also 20:36Z real-home run by RAL (Murilo at robot): HOME OK.

## 20:3x-20:51Z failed tries on loop regen 203025 + fixes
- Try 1 (203025): the grasp itself was marginal: 25 deg tilt, higher tip exactly 2.0 cm under the top. With ~1 cm JTC sag that finger sat on the top edge; it lifted, then fell. Try 2: same FIXED plan file after the drill fell 20 cm away -> grasp beside the drill.
- Loop stuck LOCKED 'lock-corrupt': grasp_test's heartbeat `touch` recreated an EMPTY lock after `unlock` removed it. Also my background installer's pgrep -f matched itself (third pgrep/pkill self-match today: never use pgrep/pkill -f with a pattern contained in the calling command).
- grasp_test.sh fixes (backup .bak_drift_20261106): heartbeat touches only an existing lock; GRASP_URL `bound` = runs/<binding run_id>/grasp_plan_input.json; a run from S1 refuses if the plan cloud vs live cloud xy-median drift > 3 cm.
- After removing the empty lock, the loop REGENed 205015 (tilt 17.3 deg). At depth 0.01 the higher tip was 2.5 cm; at --depth 0.02: higher tip 3.5 cm, 408 pts, palm 0 -> recommended. Asked Muse: vert >= 0.966 hard filter (25 deg fallback), TIP_MIN_CM 3.0, verticality above margin.
- 20:56Z run 205015 (--depth 0.02): close + lift OK (lift err 0.060), S4 put-back "failed" at err 0.243. ROOT CAUSE: lab fr3_arm_jtc.yaml path tolerance `trajectory: 0.08` rad per joint. The ~0.06 rad load sag plus descent lag exceeded it, the JTC aborted the goal and holds the current pose (not pushing). Recovery: Murilo drives down with the pad (cuRobo: small 50 Hz steps stay inside tolerance), opens, then unlock + home. TODO: grasp_test loaded stages (S3/S4) need either finer waypoints/slower timing or a looser trajectory tolerance; never a single long trajectory under load.
- Murilo: the drill surface under one pad is irregular (half the finger without contact) -> selection should check contact-patch flatness between the pads (not only point count/depth).
