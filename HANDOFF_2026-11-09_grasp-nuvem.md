# Handoff — lane "Grasp e nuvem real" (Muse) — 2026-11-09 (chat closed)

Lane owned: live drill grasp generation (GraspGen + rank + plan inputs),
`vm-viewer` fusion (`seg_view.py`), ZED calibration (offline fits + verdicts),
regrasp loop Phase A (binder), GraspGen server on lab GPU1, dashboard grasp
panel edits, held-attach sim verification. It never moves the arm directly;
motion runs are Murilo's go + named e-stop + `franka-robot` claim.

Related notes (read these too): `HANDOFF_2026-11-06_calib-STATE.md`
(coord lane's living doc, latest block 04:06Z 11-07),
`HANDOFF_2026-11-07_live-grasp.md` (tracking lane's close-out §9, includes the
04:20–06:05Z live grasp session), `fr3_real_repro/regrasp/DESIGN.md`.

## What this lane did

**Grasp pipeline + runner (11-05/06)**
- Diagnosed the first live miss: plan added operator `z_offset +0.03` on a shallow
  grasp (TCP ~4 mm below drill top). Defaults were already 0; no guard added
  (Murilo's call, e-stop covers it).
- Fixed S4 density gate in `grasp_loop/grasp_loop.py`: measured at fixed
  `pos−[0,0,0.103]` instead of `tcp_of_pose(R)` (41 mm apart on the pick);
  extracted `density_gate()`, test-first (`test_density_gate_measures_at_tool_tcp`,
  7/7 then).
- New grasps on live clouds: `dual2` idx26 → `maskonly` idx41 (score 0.645,
  TCP 20 mm deep) → lying saga → `lying2` idx152 via new 200-sample
  (far-side approach) → SE(2) `reuse152` → first FULL cycle 18:37Z 11-06.
- Sparse plan args that track: `--cart-step 0.15 --wp-min-time 6`
  (dense 7-waypoint S2 never tracked; single-waypoint does). Guards relaxed
  0.03→0.05 arrival / 0.05→0.10 continuity. `grasp_test.sh` gained grasp-URL
  3rd arg, `bound` mode, 60 N close, 10 cm lift, stop-on-gripper-failure.

**Tracker + fusion (11-05/06)**
- Restarted `vm-tracker-prod` with `secondary_cameras:=zed2` (+ new
  `vm_offload/target_object_bridge.py` publishing the `target_object` TF the
  secondary needs; `--ipc=host` required or frames never arrive).
- Rewrote `_zed2_points` in `vm_offload/seg_view.py` to mask-only SAM2
  (mask-anchored pairing, no box fallback, ±8 cm box, FK table floor 0.039 m,
  exact 4000-pt subsample, honest `+zed2_mask/none` label, 5x5 erosion):
  src=mask 100%, seam ~7.5 mm. Fixed the +26 mm prologue y-shift the new
  side face caused (top-8cm both sides). Added `/snap2.jpg`.
  Tests: `test_zed2_mask_gate` 10/10, `test_drilljson_live3d` 6/6.

**Calibration (11-05/06, all offline except touches)**
- Table step: killed 29 mm phantom + 3° stereo wedge; final h≈+29 mm FK
  (Murilo's joystick touches), per-camera dz +4.4/+4.7 mm deployed to live TF
  + cuRobo ESDF 16:46Z 11-06 (`zed{1,2}_extrinsic_20261106_robotfit7_dz.json`).
  ZED1 solid (IMU 0.35°); ZED2's "8° error" was IPPE orientation bias —
  candidate withdrawn, deployed vindicated. `table_step*.py`,
  `fit_zed2_joint.py`, `validate_zed2_joint.py` in `zed2_align/calib_robot/`.
- Touch probe `grasp_loop/tools_1105/touch_probe.py`: PARKED (force contacts
  untrustworthy — inverted heights, wrong sign). Not for reuse as-is.
- Found the ZED-teleop reflex cause (recovery auto-engage + ZED input drove
  8448 goals); flipped default to joystick in lab `fr3_ros2.sh` (backup
  `.bak_joystick_default_20261106`). Later set pad grasp default 20→50 N
  (yaml+code+slider, backups `.bak_grip50_20261106`, live-verified 50.0).

**Regrasp loop Phase A (11-06, `fr3_real_repro/regrasp/`)**
- 9 modules + lab `server/graspgen_server.py` (file-drop over scp, persistent
  on GPU1): contracts/source/accumulator/motion(SE2 ICP)/occlusion/graspgen/
  selection/publish/loop. Binder-only: writes `state/grasp_binding.json`
  (9-key contract), never publishes `/grasp_target` (TF publisher owns it).
- Gates: stillness (no inference while |dq|>0.02 rad/s), lock
  (`state/regrasp.lock` HELD_OBJECT/RUN_ACTIVE), occlusion (median over 2 s,
  zed2_none skips), monotonic adopt (never rebind older run).
- Selection: tip≥3.0 cm, tilt ladder 15°/25°/rest, then pts/vert/angle/margin/
  S1; flatness normal-angle ranks, coverage logged only (coverage is
  anti-predictive: held 0.02–0.18, slipped 0.20).
- Suite green (83+ incl. median/adopt/binding/tip-bar); live dry-run OK.
- GPU1 verdict: KEEP server (p99 281→551 ms under burst, 0 skipped solves,
  3–4 s vs 13.8 s one-shot). Recorded as persistent authorized infra.

**Dashboard (lab `isaac-build:/tmp/nvblox_real/lab_dashboard.html`)**
- ggrasp panel live: poll graspjson 5 s + drilljson 2 s + applyFollow
  (02:18:57Z, `.bak_ggrasp_live_20261107`), then TF constants updated to dz
  values in both blocks (`.bak_tfdz_20261107`, removes 30.7 mm/1.46° bias).
  Tracking lane added yaw-follow after (their backup `.bak_ggrasp_yaw_*`).

**Held-attach sim checks (11-07)**
- Ran tracking lane's v1→v6 in isolated sim (own node/topics, quest MPC
  untouched): v6 GREEN — empty-close refuses, slip-out detaches, full recipe
  704 OK/0 BLOCKED. Found the v4 empty-close floor bug (verdict at latch edge
  where w≈0.067 always). v6 deployed live by tracking lane.

**Saves**
- `snapshots/20261106_2319Z/` (2.3 MB live-state capture, in commit).
- Branch `frankateleop`, commit `1499f11` on top of master `d6b5453`
  (local only — user paused the ~400 MB push; resume with
  `git push -u origin frankateleop`).

## What's left

1. **Uncommitted work on `frankateleop`** (post-1499f11): regrasp edits
   (contracts/loop/publish/object_gate/grasp_target_from_tf + tests),
   calib-STATE updates, ~12 new `runs/20261107_*` dirs. Decide: commit and/or
   resume the paused push. Nothing was pushed, so remote is untouched.
2. **clear_box d77 relay** (top live blocker, from live-grasp §5): live attach
   never fires — 0 publishers on domain 77. Mirror `regrasp/lab/
   grasp_target_relay_d77.py`, then re-run loaded sweep.
3. **Driver commit**: VM103 `/tmp/mpc_grasp_drive.py` still uncommitted
   (suggested `regrasp/tools/`); decide forwarder FIRST_MAX 0.15 vs 0.05
   (backup `.bak_firstmax05`); box xyz still unknown (user never answered).
4. **Loop/binder ongoing**: flatness angle-ranking is in code but deployment
   to the live loop was held for the pad test — confirm current live revision
   (`/tmp/regrasp_loop_live.log` in vm-viewer) vs repo before restarting.
   Binding staleness rule stands: re-acquire after any low hover, never grasp
   on a >5 min binding.
5. **Bringup items**: 1.3 kg load at next idle bringup (mass/CoM/inertia in
   live-grasp §3); `/ssh/id_lab` hygiene in vm-viewer; JTC 0.08 loaded-stage
   aborts still open.
6. **Coordination**: coord lane (67087ec6) is back and owns the STATE file +
   live chain; this lane keeps loop/grasp side. Their files
   (`grasp_target_from_tf.py`, `object_gate.py`) — don't touch without notice.

## Where things are

- Loop: `fr3_real_repro/regrasp/` (DESIGN.md, loop.py, selection.py,
  state/grasp_binding.json, state/decisions.log, tests/); live log
  `vm-viewer:/tmp/regrasp_loop_live.log`; TF-publisher log
  `/tmp/grasp_target_from_tf.log`.
- Runs: `fr3_real_repro/grasp_loop/runs/` (+ `runs_real/` logs);
  planner `grasp_plan_lib.py`; runner `grasp_test.sh` (60 N).
- Viewer: `fr3_real_repro/vm_offload/seg_view.py`; endpoints `:8791/`
  (grasp.json, drill.json, merge3d, snap.jpg, snap2.jpg).
- Calibs: `fr3_real_repro/zed2_align/*robotfit7*.json` (+ `calib_robot/`
  scripts); lab copies in `recovered_1102/`; TF via zed-build publishers.
- Lab: teleop yaml/py/sh/html (50 N), `/tmp/pf.yaml`, forwarders
  (`direct_topic_fwd/goal_fwd.py`), MPC node `hub-curobo/node/`
  (held v6, deploy script `/tmp/held_stage/deploy_held.sh`),
  dashboard `isaac-build:/tmp/nvblox_real/lab_dashboard.html`.
- Sim-check artifacts (tracking lane's, verified here): `/tmp/held_stage/`
  on hipolito-Precision; isolated test container `held-sim-test`.
- Standing safety rule: arm moves only on Murilo's explicit go for THAT run
  + named e-stop person; claims max 5 min.
