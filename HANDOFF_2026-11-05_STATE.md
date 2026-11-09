# START HERE — 2026-11-05 (lane: ZED1/ZED2 calibration + dual-camera grasp, Murilo). Last update ~04:35 UTC, session CLOSED (e64db8e7)

> Read "## SESSION CLOSE" at the bottom first: it lists the open items in priority order and the corrections that supersede earlier notes.

Previous detail (chronological): `HANDOFF_2026-11-04_evening-STATE.md` (bottom sections = this night).

## Robot state at end of day (verified 01:55 UTC)
- Arm stopped at HOME (Murilo's home, hand ≈ (0.315, −0.003, 0.476)), **User Stop pressed at 01:55:20 UTC** → `robot_mode 0`, FrankaHardwareInterface **inactive**, fr3_arm_controller **inactive**. No recovery done by me.
- To resume (person at the robot): release User Stop → `/action_server/error_recovery` → `ros2 control set_hardware_component_state FrankaHardwareInterface active` → `ros2 control switch_controllers --activate fr3_arm_controller` (all in fr3-ros2, domain 77, LOCALHOST).
- Claims: I hold NONE. Lane 8a ("Trava/Espera teleop cuRobo Franka", 8a38210dc6d6415b998fa97f237f712f) owns the joystick chain: MPC codex-fr3-live-mpc (d77) → direct_topic_fwd (engaged-gated) → JTC. The old hardware bridge :8784 is RETIRED (do not restart). Park 8a (forwarder stopped, operator off Start) before ANY scripted motion — a pad/forwarder vs script goal-fight caused the 00:17 reflex.

## What is deployed now (all on Murilo's go)
| Thing | State | Revert |
|---|---|---|
| ZED1 TF (lab zed-build) | `recovered_1102/zed1_extrinsic_20261105_robotfit7.json` (robot-fit 7DOF, optical [0.260,−0.573,0.633]) | relaunch real_extrinsics_tf.py with `zed1_extrinsic.json` (pre-11-05) or `…_robotfit.json` (6-pose) |
| ZED2 TF (lab zed-build) | `recovered_1102/zed2_extrinsic_20261105_robotfit7.json` (optical [0.661,0.007,1.036]) | `zed2_extrinsic_20261104_planetag.json` / `…_tagreg.json` |
| Depth scale (VM fan-ins) | `DEPTH_SCALE` ZED1 0.9767 (`run_vm_fanin.sh`), ZED2 0.9959 (`run_vm_fanin2.sh`, `FANIN2_DEPTH=seg` raw robot-masked) | `DEPTH_SCALE=1.0 bash run_vm_fanin*.sh` (+ TF revert, they belong together) |
| Map | `vm-nvblox-prod` = 2 cameras (ZED1 camera_0 + ZED2 camera_1; ZED2 carve = `vm-mask-sync2`, drill-box projection) | stopped ZED1-only container `vm-nvblox-prod-1cam` |
| Cable | MTU 9000 persisted lab/pve/VM; VM `/etc/sysctl.d/90-cable-reasm.conf` (ipfrag_max_dist 0, 64/48 MB) — this was the real fix for feed flaps | see memory pve-access-and-cable |
| Lab web_video_server :8787 | restart loop in zed-build (it segfaults on stream removal) | — |
| Grasp shown on dashboard | run `grasp_loop/runs/20261105_011430_drill_dual` (merged ZED1+ZED2 cloud) — **STALE: the drill was rotated ~20–30° by the failed attempt** | regenerate |

## Calibration findings (numbers)
- Robot-based fit (10 JTC poses, `zed2_align/calib_robot/`, `fit_robot.py`/`fit_robot7.py`): ZED1 pre-11-05 calib was off 3.5°/47 mm. Arm residuals: ZED1 ~4–5 mm, ZED2 ~2–3 mm (Z poses).
- Cameras differ in depth SCALE by ~2–2.5 % (arm and AprilTag agree on the relative, not on the absolute). Tag cross-camera after 7DOF: 19 mm.
- ZED1 near-field depth warps ~2 cm (region x .6–.8, y −.45..−.30) → map table ~2 cm "thick" near ZED1.
- **Open: both robot-fits put the table at ≈ +2.8 cm in fr3_link0** (old calib 1.3 cm). If the base sits on the same tabletop, there is a ~2–3 cm common vertical bias. Murilo to say how high above the drill top the fingertips closed in the test.

## Grasp test 01:33–01:36 UTC (Murilo operating; `grasp_loop/grasp_test.sh`)
- Runner: `grasp_test.sh plan|run [Z_OFFSET]` (VM) → `grasp_plan.py` (IK, FK verified vs measured hand to 0.001°) + `grasp_stage_exec.py` (lab; foreign-publisher guard; refuses out-of-order) + `grasp_gripper.sh`. ENTER before each stage. Log `grasp_loop/runs_real/grasp_test_20261105_013303/`.
- Result: closed in the air too high (gripper grasp ABORTED). Cause = MY z_offset +0.03 on an already shallow GraspGen grasp (TCP only ~2.5 cm below the drill top) → planned TCP 0.247 ≈ drill top. Stages executed as planned (err ~0.02 rad). The plan is frozen before motion, so the segmentation change did NOT alter the trajectory.
- Murilo homed the arm himself; then User Stop at 01:55.

## Segmentation leak (Murilo saw it) — real, not yet fixed
- When the arm occluded the drill (01:35:15 UTC, S3) the SAM2 mask leaked onto the table; it recovered after the arm left. Nothing rejects a mask whose area/centroid jumps.
- Tracker logs "No valid depth for tracking point" since then: centroid sits on the image bottom edge (no ZED1 depth there) — drill is at the bottom edge of ZED1's view.

## Next (proposed, waiting for Murilo)
1. Tracker guard: reject masks with area > ~1.5× or centroid jump > N px between steps; hold the last good mask until the object reappears.
2. Occlusion freeze: while the FK-projected arm silhouette overlaps the drill box in ZED1, freeze drill cloud/box and the carve masks (mask-sync / mask-sync2).
3. Regenerate the grasp on the current drill pose (drill rotated), with a depth criterion: fingertips ≥ 4 cm below the drill top; run with z_offset 0 (or lower, depending on the vertical-bias answer).
4. Resolve the +2.8 cm table question (physical check of the base mounting; or a fingertip-touch test on the table).
5. Rules learned tonight: after ANY fan-in restart → `docker restart vm-viewer` (it loses depth silently); rclpy Image.data must be `array('B', …)` in Python republishers; never two scripted publishers on the JTC topic.

## ~02:05 UTC: new sim lane 62f98b58 ("Simulação Isaac ZEDs", Isaac replica on VM103)
- Told them: calib truth = robotfit7 TFs + DEPTH_SCALE (applied in VM fan-ins; /vm/cable* already scaled); drill.json center is AABB incl. extension to table, drill rotated by the failed grasp, yaw approximate, leak on occlusion.
- WARNED: VM domain 8 is the LIVE bus shared with the lab (not localhost) — sim must use its own ROS_DOMAIN_ID; Isaac on VM GPU0 (GPU1 = nvblox_curobo + SAM2 at ~95 %).
- 03:40 UTC: sim lane (62f98b58) flagged vm-mask-sync2/seg_view fallback calib = planetag. Live uses TF (robotfit7) — log 'T from tf'; fallback defaults now changed to zed2_extrinsic_20261105_robotfit7.json in both files (no restart; takes effect on next start).
- (SUPERSEDED by 04:25 correction) 03:45 UTC: sim/live coll_cost gap explained — sim MPC had target_clear_frame:=fr3_hand radius 0.15 (leftover). The node already has target_clear_frame/target_clear_radius_m (vm_real/node/curobo_mpc_node.py:134-137, default radius 0 = off) → the sphere-59 self-clear may be a launch arg (small radius ~6-8 cm; clears real obstacles inside it too). Proposed to 8a; Murilo decides; test in sim lane first.
- 04:20 UTC: live MPC CSV (codex-fr3-live-mpc /tmp/codex-live-mpc.csv) coll_cost is set by pose_error: holding <0.01 → med 4.9; >0.5 → med 566. Since ~01:20Z the live MPC chases a stale wrist_target (pose_err ~0.9, arm User-Stopped) → its 'hot' cost is a goal artefact, not ESDF clutter. closest_sphere=59 / min_dist 0 in 100% of rows. Told 8a (suggested resetting the MPC goal to the measured hand) and the sim lane (compare matched pose_err bins).
- 8a agreed (clutter read was wrong; correction forwarded to sim lane). MPC stale-goal reset DEFERRED to tomorrow's resume, with Murilo, during the chain re-verify.
- 04:25 UTC CORRECTION: curobo_mpc_node.py:1033 maps unobserved (−1000) → FREE. My earlier "sphere 59 at min_dist 0 = unobserved voxels treated as occupied" was WRONG. min_dist = max(sphere_dists) (l.1766–1769); cuRobo returns 0 for clear spheres → constant "NEAR-OBSTACLE +0.000 sphere#59" most likely = no sphere near anything (reporting artefact; label l.1819 fires on min_dist > −0.05). Unverified: print raw sphere_dists once. The target_clear idea is therefore NOT needed for sphere 59. Corrected to 8a and the sim lane.
- Live pre-01:20Z moving-arm coll_cost by pose_err: <0.01 med 5.9 | 0.01–0.1 med ~17 | 0.1–0.5 med 71 | >0.5 med 1158. Sim tracking bins run ~4–5× higher (~75–86): open, in the sim lane.
- VM containers admiring_bartik / friendly_hypatia (spot-isaac-sim, hung full Kit via runheadless.sh entrypoint, ~80% CPU each) are NOT mine; told the sim lane.


## SESSION CLOSE (~04:35 UTC 11-05) — read this first
**Robot:** stopped at home, User Stop pressed (01:55:20Z), robot_mode 0, FrankaHardwareInterface + fr3_arm_controller inactive. Nobody recovered it. Resume (person at robot): release User Stop → `/action_server/error_recovery` → `set_hardware_component_state FrankaHardwareInterface active` → `switch_controllers --activate fr3_arm_controller` (fr3-ros2, d77, LOCALHOST). Then 8a re-verifies its chain AND resets the live MPC goal to the measured hand (it has chased a stale wrist_target (0.355,−0.001,0.472) since ~01:20Z).
**Claims:** I hold none. 8a (8a38210dc6d6415b998fa97f237f712f) owns the joystick chain (MPC codex-fr3-live-mpc d77 → direct_topic_fwd → JTC); park it before any scripted motion.
**Running on the VM (mine, all healthy at close):** vm-cable-fanin (ZED1, DEPTH_SCALE 0.9767), vm-cable-fanin2 (ZED2 raw seg, 0.9959), vm-tracker-prod (SAM2), vm-mask-sync, vm-mask-sync2, vm-nvblox-prod (2-cam map), vm-viewer (:8791). Stopped backups: vm-nvblox-prod-1cam, vm-nvblox-seg. NOT mine: vm-mpc/vm-relay (2 days, unhealthy), admiring_bartik + friendly_hypatia (hung spot-isaac-sim Kits, ~80 % CPU each; sim lane 62f98b58 is finding the owner — no one has OK'd killing them).

**Corrections that supersede earlier text in this file and in HANDOFF_2026-11-04_evening-STATE.md:**
- Sphere 59 / "NEAR-OBSTACLE min_dist +0.000": NOT unobserved voxels. curobo_mpc_node.py:1033 maps −1000 → free. min_dist = max(sphere_dists) with cuRobo returning 0 for clear spheres → most likely a clear arm mislabeled (l.1766–1769, l.1819). Verify once by printing raw sphere_dists. target_clear is NOT needed for it (8a retracted it with the sim lane).
- Live MPC coll_cost is driven by pose_error (holding ~5, >0.5 rad-ish error ~460–1160), not by map clutter.

**Open, in priority order (all need Murilo's go where hardware is involved):**
1. Segmentation guard (tracker): reject SAM2 masks whose area jumps > ~1.5× or centroid jumps between steps; hold last good mask. Leak seen 01:35:15Z when the arm occluded the drill.
2. Occlusion freeze: while the FK-projected arm overlaps the drill box in ZED1, freeze drill cloud/box and both carve masks.
3. Regenerate the grasp — the drill was ROTATED ~20–30° by the failed attempt; the dashboard grasp run `grasp_loop/runs/20261105_011430_drill_dual` is STALE. Add a ranking criterion: fingertips ≥ 4 cm below the drill top. Run `grasp_loop/grasp_test.sh run 0` (NOT +0.03; that offset caused the miss).
4. Vertical bias question: both robot-fits put the table at ≈ +2.8 cm in fr3_link0. Ask Murilo how high above the drill top the fingertips closed, and/or check how the base is mounted. If ~3 cm low/high, correct before the next grasp.
5. ZED1 near-field depth warp (~2 cm) makes the map table ~2 cm thick near ZED1 — conservative, but note for clearance tuning.
6. Sim lane parity: their tracking-bin coll_cost is ~4–5× live (~75–86 vs ~17); live settled ~5 vs sim 0.0 — theirs to chase.
7. Housekeeping: tracker clear-box warns `"world" frame missing` (target_parent_frame param). Dashboard `_control_zed` disengage/home sends bridge enable-off (bridge retired now, so likely moot — verify).

**Key paths:** calibration tools/data `fr3_real_repro/zed2_align/calib_robot/` (move_pose.py, capture_pose.py, fit_robot.py, fit_robot7.py, tagcheck.py, run_1105/); deployed calib JSONs `fr3_real_repro/zed2_align/zed{1,2}_extrinsic_20261105_robotfit7.json` (+ lab `recovered_1102/`); fan-ins `fr3_real_repro/vm_offload/run_vm_fanin*.sh`; viewer `fr3_real_repro/vm_offload/seg_view.py`; ZED2 carve `fr3_real_repro/grasp_loop/vm_mask_sync2.py`; grasp runner `fr3_real_repro/grasp_loop/grasp_test.sh` + `grasp_plan.py` + `grasp_stage_exec.py`; real test log `grasp_loop/runs_real/grasp_test_20261105_013303/`.
**Rules learned:** don't argue with Murilo's eyes (NN medians hid a 6 cm doubling); after ANY fan-in restart `docker restart vm-viewer`; Python republishers must set Image.data as `array('B', …)`; never two scripted publishers on the JTC topic (00:17 reflex); VM domain 8 is the LIVE bus — sims use their own domain; spot-isaac-sim needs `--entrypoint bash`.
