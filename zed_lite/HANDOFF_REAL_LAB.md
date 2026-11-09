# Real lab: ZED cameras on the remote host. Handoff (2026-10-14)

## What and why
The lab machine `murilo@100.88.134.127` (Tailscale; host `hipolito-Precision-5860-Tower`, 2x RTX A2000 12 GB) has
three ZED 2i on USB. Goal: stream them live to the browser, run nvblox on the depth, and calibrate the cameras
to the Franka FR3 base so the map is in robot coordinates. The camera-to-arm transform does not exist anywhere
on the machine (searched 10-14), so it must be measured.

## Decided
- Calibration: one Charuco board on the table, seen by both cameras; the FR3 tip touches its four outer corners.
  Board 600 x 400 mm (6 x 4 squares of 100 mm, DICT_5X5_100). Reason: a 280 mm board gives about 7 mm camera
  error at 1 mm touch noise, the 600 mm board about 3 mm. User's preference.
- Streaming: `web_video_server` (ROS) serves the topics; the dashboard is a static page. Not a direct pyzed script
  (that held the USB cameras).
- Stream settings for the dashboard: `width=480&height=270&quality=60` (full 640x360 q80 stuttered under CPU load).
- nvblox runs on GPU 1 (`CUDA_VISIBLE_DEVICES=1`); the three ZED wrappers stay on GPU 0.

## Done
- zed-build container: 3 wrappers (`zed1` 33874668, `zed2` 39498278, `zed3` 34046618). Left images
  `/zedN/zed_node/left/image_rect_color` at about 30 Hz (`bgra8` 640x360). Depth `/zedN/zed_node/depth/depth_registered`
  (32FC1). Neural-light depth model compiled (about 6 min per camera on first run, now cached).
- web_video_server on `100.88.134.127:8787`; dashboard on `:8788` shows three live panels (verified in headless Chrome).
- nvblox built from source in isaac-build (`nvblox_node`, `fuser_node`) and run once: mesh about 9.8 Hz, ESDF about
  9.6 Hz, both cameras consumed. Stopped on request. Its placeholder `map -> zedN_camera_link` transforms were
  **not** calibrated (zedN at (0,0,1) and (1,0,1)).
- Calibration tool `calib/board_calibration.py` written and tested offline (selftest, solve on synthetic data,
  board detection on a synthetic view, capture-without-board). See `calib/README.md`.
- Single-tag camera-to-camera pose (2026-10-14, `calib/aruco_pair.py`): AprilTag 36h11 id 16 (13.8 cm, measured) on the table,
  seen by zed1 and zed2 together. zed2 origin in zed1: (0.220, -0.557, 0.515) m. Needs subpixel corners. Open:
  zed2 scale ~3 % short by depth (zed1 matches 13.8 cm within 1 %). Details in `calib/README.md`, results in
  `~/spot-build-logs/aruco_pair/`.

## Still open
1. **Real calibration run**: print `calib/charuco_board.png` at 100 %, measure one square, place the board, then
   `capture` -> `touch` -> `solve` -> `check` (inside zed-build). Nothing has been run on the real board.
2. **Touch-off input**: `touch` takes typed xyz. Need the TCP readout source: the FR3 ROS 2 driver in
   `~/franka_ws` (other lane) or the ROS 1 `franka_states` in `oopsie-ros1`. Not verified.
3. **FCI is single-client**: `oopsie-ros1` (Paiva's ROS 1 FR3 stack, not ours) holds the robot. The touch step
   needs it stopped or run inside that stack. Ask the user first.
4. Replace the nvblox placeholder transforms with the solved `zed_extrinsics.yaml` once calibrated.
5. Not verified: a live run of the zed_wrapper `ros2 launch` for a new camera beyond the three already running.

## Do not
- Do not kill the Gazebo process (`gz sim -r -s empty.sdf`, other lane's Franka sim, `franka_gazebo_bringup`).
- Do not browse `/home/hipolito` (permission denied; the oopsie dataset is reached only through container mounts).
- Do not delete anything on the remote machine without asking. Docker cleanup so far: build cache and dangling
  images only. The July container `spot-teleop-spot-ros2-1` had a 28 GB rosbag, which the user had authorised deleting.
- Commit only when asked. No Claude co-author lines.

## Files and commands
Local (repo `~/spot-teleop`):
- `zed_lite/calib/board_calibration.py`, `zed_lite/calib/README.md`, `zed_lite/calib/charuco_board.png`
- `zed_lite/` root: Isaac Sim ZED pipeline (`README.md`), not the real-lab stack.

Remote, copies in `~/spot-build-logs/`, live copies in containers at `/tmp` (lost if a container restarts):
- `start_zed_ros.sh` (zed1, zed2), `start_zed3.sh` (zed3), `start_wvs.sh` (web_video_server)
- `run_nvblox2.sh` (nvblox on GPU 1 + placeholder static TFs), `board_calibration.py`
- logs: `zed1.log`, `zed2.log`, `zed3.log`, `wvs.log`, `nvblox2.log` in containers; `isaac-nvblox2.log`, `colcon-zed.log` on host
- dashboard: `~/spot-build-logs/dashboard/index.html` -> container `/tmp/dashboard/index.html`, served by
  `python3 -m http.server 8788 --bind 100.88.134.127`

Containers: `zed-build` (image `spot-teleop-zed:latest`, ZED SDK 5.0.7, ROS 2 Humble, `zed_ws` built at
`/home/spot_ws/zed_ws/install`), `isaac-build` (image `spot-teleop-isaac-ros:latest`, Isaac ROS 3.2.5 jammy apt repo
`isaac.download.nvidia.com/isaac-ros/release-3`, nvblox built at `/workspaces/isaac_ros-dev/install`).
ROS settings: `ROS_DOMAIN_ID=8`, `RMW_IMPLEMENTATION=rmw_fastrtps_cpp`; containers use host network and ipc.

Useful checks:
- `ros2 topic hz /zed1/zed_node/left/image_rect_color`
- stream: `http://100.88.134.127:8787/stream?topic=/zed1/zed_node/left/image_rect_color` (topic unencoded; `%2F` breaks it)
- dashboard: `http://100.88.134.127:8788/`
- `nvidia-smi` for GPU use (GPU 0 also drives the desktop; ask before moving the wrappers)

## Measured, expensive to find again
- ZED frames: wrapper publishes 640x360 by default. Frame chain per camera: `zedN_camera_link` (root, no parent)
  -> `zedN_camera_center` -> `zedN_left_camera_frame` -> `zedN_left_camera_optical_frame`. Link -> optical offset
  about (-0.010, 0.060, 0.015) m.
- Wrapper CPU: each about 50-60 % of a core. web_video_server about 27 %. A stray Gazebo sim plus Hipolito's Chrome
  dropped delivered frame rate from 30 to 6-8 fps before the stream settings were lowered.
- GPU 0 (wrappers): about 330 MB per wrapper; 1.26 GB total with three. nvblox on GPU 1: about 274 MB.
- nvblox output rates: mesh 9.8 Hz, ESDF 9.6 Hz, back-projected depth 6.5 Hz (camera 1), depth input about 25 Hz.
- Board fit: median camera-position error 3.2 mm at 1 mm touch noise (600 mm board), 6.9 mm (280 mm board).
- A Franka-to-camera transform is not in any file on the machine: checked the oopsie code and container mounts,
  the `fr3-datahive` profile (calibration fields `null`), the franka_ros2 kit (generic D405 wrist mount only),
  RLinf (its `calibration` is the PICO VR headset), the Spot tools (irrelevant to FR3).
- oopsie repo git history IS readable without changing config: `git --git-dir=/workspace/.git ...` inside `oopsie-ros1`
  (explicit GIT_DIR skips the ownership check; the container's git 2.25 ignores `-c safe.directory`).
- Wrist RealSense (D435i) to flange/hand transform: rigorous search 2026-10-14 found none. Searched: Paiva's catkin
  src (static_transform_publisher, TF code, realsense/wrist/hand-eye terms, gripper TF params), the whole `/workspace`
  mount (RLinf, openpi_fr3, diffusion policy, tools, tests, ui, docker, oopsie-config, samples metadata), the
  `oopsie-ros1` filesystem, all 174 commits on every branch plus 2 stashes (pickaxe: static_transform_publisher 0,
  wrist_camera_mount 0, hand_eye 0), the `fr3-datahive` image (extrinsics null), `murilo`'s files, /opt, /etc,
  /usr/local, /srv. Only generic or other-project hits: RLinf roboverse sim wrist mount (panda_hand, sim task),
  franka_ros2 vision kit D405 on Robotiq (0, 0.027, 0.0363), Spot hand-camera tooling, Isaac ROS Carter example.
  Not searched: /home/hipolito directly (no permission, and off limits), binary files (hdf5, npz, pt), Desk config.
- The wrist camera frame is only the string `realsense_wrist` (cameras.launch); `uvc_camera.py` publishes no TF and no
  camera_info. Not in any URDF.
- EE frame: Paiva's `fr3_simulation/config/simulation.yaml` documents `F_T_EE` as the Franka Hand default (rotate -45 deg
  about z, translate 0.1034 m along z). So `O_T_EE` is the hand TCP, not the flange. For hand-eye use the flange:
  `O_T_F = O_T_EE * inv(F_T_EE)`, with `F_T_EE` read from the real `franka_states` (not yet read: stack down).
- 2026-10-14 state of `oopsie-ros1`: `franka_control` and the `uvc_camera.py` nodes are defunct (nothing publishes
  `/franka_state_controller/franka_states` or the camera topics; the topics only appear because web_video_server and
  rosbridge subscribe). A wrist-only `cameras.launch` was started by hand (log `/tmp/he_cam.log` in the container);
  the front/top nodes it also starts die at once (ZED devices are held by `zed-build`). Wrist frame shows tag 16 square-on.
- July Isaac image history holds an HF token in a build argument (`HF_TOKEN`). Rotate it.

## 2026-10-15 - markerless hand-eye (EasyHeC) on recorded real data
- Recorded data exists: `oopsie-ros1:/workspace/samples/<session>/<episode>.h5` + `_front/_top/_wrist.mp4` (6217 episodes, 113+ sessions with front video, last 2026-10-14), joint_position at 10 Hz. Front/top = ZED 2i left half of 2560x720 UVC, centre-cropped to 4:3 and resized to 640x480 (raw, distorted).
- Isolated env: container `easyhec-build` (GPU 1), `/home/murilo/easyhec` (README.md there). torch 2.11 cu128, easyhec 0.1.9, nvdiffrast OK (smoke test). SAM2 not installed yet.
- Background check, recorded `front` vs live zed1 (raw mapped to recorded geometry): reliable static patches (pole, robot base) agree to about dx -9 px, dy +2..4 px (about 1.4 deg if rotation). The same offset appears against sessions 09-04..10-14, so the recorded camera was stable for weeks (exceptions: 08-27, 09-15, 10-01), but live differs by about 9 px. NOT resolved whether zed1 moved after 10-14 or the wrapper raw geometry differs from the UVC stream. Decisive test: grab a UVC frame through the same uvc_camera crop (needs the zed1 wrapper stopped briefly).
- 2026-10-15 ZED health: zed2 had a CAMERA REBOOTING episode at 10:59 UTC (recovered); zed3 was down (USB device missing from lsusb).

### 2026-10-15 EasyHeC fit, recorded `front` camera (zed1 view), 40 frames
- Code in `/home/murilo/easyhec/hec/` (hec_lib, hec_core, hec_search3 = sample-and-optimise + SAM2 projected-prompt rounds, hec_validate), results in `/home/murilo/easyhec/results/` (`front_camera_extrinsic.json`, `final_overlay.png`).
- Result: T_base_cam (camera optical frame in fr3_link0) = [[0.9313,-0.0629,0.3588,0.2261],[-0.3472,-0.4514,0.8220,-0.6124],[0.1102,-0.8901,-0.4422,0.6334],[0,0,0,1]]; camera at (0.226,-0.612,0.633) m, 0.91 m from the base.
- Checks: FK of the URDF reproduces the recorded cartesian_position exactly (0.00 mm, xyzw quaternion); renderer convention self-test 100%; three independent starts converge within 1.4 mm / 0.08 deg; 4 held-out-subset fits within 4.1 mm / 0.25 deg; overlay outlines follow the real arm in all inspected frames. NOT accuracy: no ground truth.
- Open: live zed1 vs recorded background differ by about 9 px (moved after 10-14 or geometry offset?); repeat on `top` and compare with the tag-derived T_zed1_zed2 as the independent check; intrinsics are the wrapper's left_raw camera_info.
- 3D viewer of the fit: `http://100.88.134.127:8790/` (Tailscale only, static Three.js, files in `/home/murilo/easyhec/viewer`, restart with `bash /home/murilo/easyhec/startview.sh`; regenerate data with `hec/export_viewer.py` inside `easyhec-build`). Shows the FR3 at each of the 40 recorded poses, the camera frustum with the real frame, and a "Through camera" mode that overlays the robot on the photo. The table is only an assumed z=0 plane.

### 2026-10-15 top camera + validation status
- Top (assumed zed2) fit: T_base_cam = [[-0.1101,0.9911,-0.0743,0.6461],[0.9917,0.1045,-0.0756,0.0095],[-0.0672,-0.0820,-0.9944,0.9413],[0,0,0,1]] (camera 0.94 m above the table, looking down). Seeded from T_base_front @ T_zed1_zed2, so NOT independent. Results in `/home/murilo/easyhec/results_top_seed/`. Repeatability 5-17 mm / <=0.9 deg (front: 1-4 mm / 0.25 deg).
- Unseeded global search with a weak look-down prior reached only IoU 0.55 vs 0.66 and ended 51 mm / 11.4 deg from the seeded fit: the top camera pose is NOT independently confirmed. Plain global search without the prior found no basin.
- Cross-checks (no ground truth): fitted front->top vs tag-derived T_zed1_zed2 differ by 29 mm / 5.6 deg (the tag relation itself carries ~36 mm / 2.2 deg of scale uncertainty from zed2). The table tag, put into the base frame from each fit, lands 46 mm / 4.4 deg apart; tilt from vertical: front 1.9 deg (z=2 cm), top 3.8 deg (z=4 cm).
- Viewer now shows both cameras and both tag squares: http://100.88.134.127:8790/ (export with `export_all.sh` in the easyhec-build container).
- Still to do for real validation: touch test or tag-on-hand test with the robot (see the answer in the session), plus a refit on frames from other days to check the cameras have not moved.

### 2026-10-15 real robot: calibrated TF + vision chain brought up PASSIVELY (no robot commands)
Context: the lab FR3 was live and being driven by the pad teleop (franka_ros2 `real-up`, container `fr3-ros2`, ROS domain 77, localhost discovery: NOT visible from the ZED/nvblox containers in domain 8). Robot state was read only through the dashboard SSE `http://127.0.0.1:8088/stream` (link poses in fr3_link0, tcp, engaged).
- Static camera TFs (domain 8, zed-build): `/tmp/real_extrinsics_tf.py --calib <json> --cam zed1|zed2` publishes `fr3_link0 -> zedN_camera_link` so the wrapper's own optical frame lands on the EasyHeC pose (verified: TF lookup fr3_link0 -> zedN_left_camera_optical_frame equals the calibration to 1e-7). Calibrations: `/home/murilo/easyhec/results/front_camera_extrinsic.json` (zed1), `results_top_seed/top_camera_extrinsic.json` (zed2). Source copy: `zed_lite/nvblox/real_extrinsics_tf.py` is NOT yet copied here (it lives in the scratchpad copy + zed-build /tmp).
- nvblox (isaac-build): `/tmp/run_nvblox_real.sh` (nice 19, taskset cores 12-15, GPU 1), 2 cameras, global frame fr3_link0, 2 cm voxels, static TSDF, 3D ESDF, NO self-filter (the arm is mapped as an obstacle). Healthy: ~9 Hz depth, ESDF 2 Hz, no TF errors; robot side unaffected (no flight-recorder events, dashboard 20 Hz).
- LIVE CHECKS (arm still, ONE pose only): depth points near the arm vs the model at the live pose: zed1 median 15.5 mm (34% within 1 cm), depth ICP asks for 37 mm / 0.9 deg; zed2 19.4 mm (22%), asks for 42 mm / 5.4 deg. Model-free: table plane zed1 z=22 mm tilt 1.4 deg, zed2 z=57 mm tilt 3.0 deg; the two clouds disagree by about 5 cm / 7 deg. => the recorded-frame calibrations do NOT hold for the live cameras (zed1 may have moved ~1 deg since 10-14: the 9 px offset; zed2 also has the unresolved depth-scale issue). The fused map ghosts the arm: DO NOT use for MPC.
- Next: `live_recorder.py` (zed-build /tmp, log /tmp/live_recorder.log, output /tmp/live_calib/NNN/) saves both cameras' images + depth + link poses whenever the arm is still >=1 s in a new pose; refit both cameras on LIVE data (EasyHeC with the dashboard's link poses and the wrappers' rectified K), then re-publish the TFs. MPC on the real arm NOT started (never run on the real FR3; needs the free-space test first, user + e-stop).
- Started by me and still running: 2x real_extrinsics_tf (zed-build), nvblox_node (isaac-build), live_recorder (zed-build), viewer http.server :8790 (host), idle container easyhec-build. Stop: `docker exec isaac-build pkill -f nvblox_node`, `docker exec zed-build pkill -f live_recorder.py`, etc. (by PID, separately).

## 2026-10-15 (late) — automated calibration sweep on the real FR3 (BUILT, TESTED IN SIM ONLY, NEVER RUN ON THE ARM)
User asked to run EasyHeC on the real Franka with a script that drives the arm through predefined poses. Files in
`zed_lite/calib/` (mirrored `/home/murilo/easyhec/sweep/` and `/home/murilo/franka_ws/lab/calib/` = `/lab/calib/` in `fr3-ros2`):
`fr3_kin.py`, `plan_sweep.py`, `sweep_waypoints.json` (+`sweep_preview.png`), `real_calib_sweep.py`, `SWEEP_README.md` (operator checklist).
- Planner: 234 feasible of 1800 candidates -> 36 diverse poses, 2-opt tour, 4.3 m, ~7 min at 4 cm/s / 15 deg/s / 3 s dwell; legs <= 22 cm, <= 70 deg;
  joint margin >= 0.58 rad, self-clearance >= 16 cm, floor >= 9 cm, wrist+hand inside both recorded-frame camera images. DH FK == URDF (1e-15).
  Nullspace target = mean of the 40 recorded real joint configs (the lab controller's own behaviour).
- Runner publishes only `equilibrium_pose` (same interface as the lab's real_lift.py). Hard limits in code; pre-flight refuses unless: fresh state, mode 2,
  no error flags, `cartesian_impedance_target_controller` ACTIVE, nobody else publishing (pad teleop disengaged), IK of every leg from MEASURED joints OK.
  Aborts + re-targets measured pose on: stale, mode change, errors, lag > 2.5 cm for 2 s, force > 25 N, joint within 0.08 rad of a limit, other publisher, Ctrl+C.
  Simulator tests passed for all (`--sim --sim-fault ...`); the ROS I/O class (RosRobot) is UNTESTED (running anything inside fr3-ros2 was blocked by the
  permission classifier, even read-only; the user must run `--dry-run` there first).
- State found at 15:58 local: `cartesian_impedance_target_controller` INACTIVE (list_controllers), dashboard `engaged: true` but TCP static for >10 min
  at (0.461,-0.288,0.472). Table has tools on it (drill, toolbox case, bit sets) -> must be cleared. Host load avg 60 (a hipolito training, not ours).
- live_recorder (zed-build, cap 60) holds 14 teleop captures (orientation almost constant: weak) and has room for the 36 sweep poses.
- 2026-10-15 17:5x local UPDATE (superseded the "controller INACTIVE" line above): the other session (Codex + Claude Code agents on `mhc-lenovo`, SSH as `hipolito`) ran
  `fr3_home.py` at 17:43:55 -> libfranka reflex `joint_motion_generator_acceleration_discontinuity` (no contact, 2.9 N), hardware deactivated, robot_mode 4.
  Found with `docker events --filter container=fr3-ros2 --filter event=exec_create` (keeps every command). 17:49 `./fr3_ros2.sh real-recover` from nexus (log
  `/home/murilo/recover_1792097342.log`): in-place error_recovery ABORTED -> restart path -> mode 2 at 20:50:02 UTC, arm did not move. 17:56 `real-fast` (user OK):
  400/20 then 1000/40, read back with `ros2 param get` (1000 / 40 / clip 0.02, 0.10 / filter 0.05 / rot damping 0.4).
  Persisted (user OK): `fr3_ros2.sh` real-recover now ends at the WORKING gains; `RECOVER_SOFT=1` = old soft 150/10. Backup `fr3_ros2.sh.bak_before_default_gains`.
  `bash -n` OK, the edited recover path NOT run end to end (it restarts the driver). real-up / real-hold already defaulted to the working set. Gains set with
  `ros2 param set` are runtime only. Sweep dry run after recovery: PRE-FLIGHT OK (first leg 26 cm). The sweep recorder `/tmp/live_calib_sweep` (zed-build) is alive, 4 saved.
- NEXT after the sweep: EasyHeC refit of both cameras on /tmp/live_calib (wrappers' rectified K, link poses from robot.json), re-publish TFs, re-run live depth check.

## RESUME HERE (written 2026-10-15 ~21:30 local): dynamic-sim study of the real sweep failure, EasyHeC in sim
**Goal.** Calibrate the FR3 cameras (front = zed1, top = zed2) with EasyHeC on REAL data from an automated 36-pose arm sweep. The real sweep (`/lab/calib/real_calib_sweep.py`, plan `sweep_waypoints.json`) failed on the real arm: hand 23 mm off target -> abort; a +180 deg tool-yaw "park" turned ~1 deg/s. User (left the lab) wants it SOLVED IN A DYNAMIC SIM with the real controller, not teleported joints, then applied to the robot.
**Root cause (source + sim).** `cartesian_impedance_target_controller` (fr3_lab_controllers/src/cartesian_impedance_target_controller.cpp) stores its posture reference `q_d_nullspace_ = q_` ONCE at activation (line ~153) and pulls toward it with nullspace_stiffness (20). The projector uses a damped pseudo-inverse with hard-coded lambda 0.2 (not exact), so the posture torque leaks into the hand. After the lab's HOME move joint 7 sat 2.4 rad from q_d. Gazebo (real C++ controller, real-run params) reproduces it: hold error at +120 deg yaw: k20 6.8 mm/10 deg, k5 6.4/11.9, k1 4.6/7.0, k0 1.0/1.2 (ready pose, no posture error: 0.0). Setting k=5 (done live on the real arm, runtime only) does NOT cure it. Rot-leash 10 deg stalls the runner (dynamic turn lag ~7-13 deg): use >= 25.
**Sweep in sim (36 poses, same guards as the runner; `gz_sim/results/`):** unpatched controller: k20 rl10 / rl25 abort wp0 (19-20 mm, the real failure); k5 rl10 2/36 (rot-gate stall), k5 rl25 3/36 (20 mm); k0 2/36 (joint-2 limit, elbow wanders); k20 rl25 guard 35 mm: abort wp2 (40 mm = clip); k5 rl25 guard 35 mm ("w1"): reached wp16, slowing (21.8 s vs 7.9 plan, steady 18 mm/25 deg), final result in `/home/murilo/easyhec/gz/logs/sweep_w1.json`. Patched lambda (live param `nullspace_pinv_lambda`): control 0.2 = abort wp0; k20 rl10 lam .05/.01 2/36 (rot gate); rl25: lam .05 3/36, lam .01 4/36 (18 mm at wp4), lam .001 4/36, k5 lam .01 6/36 (16 mm), k2 lam .01 5/36 (joint 7 hit 3.05 rad). NOTHING completes all 36 yet; planned joints match the sim's actual joints for wp0-2 (<0.1 rad), diverge at wp3+.
**Current hypothesis / next step.** Plan and controller disagree on posture: the planner picks diverse joint configs (feature includes q), the controller pulls to the activation posture. Fix = replan with `--q-ref` (the controller's posture at activation) and `--max-dq` (~0.7-0.8 rad) so the posture error stays small, then test in sim with the UNPATCHED controller (needs no rebuild on the real arm). I started patching `calib/plan_sweep.py` (backup `plan_sweep.py.bak_before_qref`): the patch script FAILED on the home-IK line (assertion), NOTHING was written; redo it (args --q-ref 7 floats, --max-dq, --q-weight; IK with q_null=QREF; filter candidates and legs). Gazebo's activation posture = ready [0,-0.785,0,-2.356,0,1.571,0.785]; real: pick/record the posture at re-activation. Then: sim variants (k20 / k5, rl25, default and loosened guards), then real-robot procedure. Also consider: the lag guard on the target is the wrong signal for calibration (measured joints are used, not commanded); guard "arm still + limits" instead.
**Real robot state when I left.** Arm static, driver up (restarted 17:49 local via real-recover), controller active: K 1000/40, clip 0.04/0.2 (other session doubled them), filter 0.05, nullspace_stiffness 5 (set by the user live), teleop process running DISENGAGED; two idle agent sessions from `mhc-lenovo` (hipolito, Codex + Claude Code) can run `docker exec fr3-ros2` anytime; `docker events --filter container=fr3-ros2 --filter event=exec_create` shows who ran what. `fr3_ros2.sh` real-recover now ends at working gains (RECOVER_SOFT=1 = soft). Runner in `/lab/calib` has --park-yaw, --rot-leash/--max-rot-lag/--move-slack, trace log, dry-run log; UNTESTED on the real arm after these fixes. My writes inside `fr3-ros2` were blocked by the classifier (param set, service calls): the user pastes commands.
**EasyHeC in sim (Isaac, kinematic capture, only for the CALIBRATION math, not for motion):** `franka_mpc/calib_sim_capture.py` (cameras at the real fits, 40 real poses + 36 sweep poses, joints set directly), data `franka_mpc/runs/calib_sim1` (47 MB; copy on the lab PC `/home/murilo/easyhec/sim_in/calib_sim1`, datasets `/work/data/sim_<set>_<cam>_1014`), `calib_sim/{make_inputs,prep_sim,hec_sim}.py`, results `/home/murilo/easyhec/results_sim/results.jsonl`, video `runs/calib_sim_video/video.mp4` (kinematic playback). Truth known: exact masks recover 80 mm/5 deg -> 0.3-0.5 mm/0.04 deg on both cameras and both sets; SAM-prompted loop: front 0.7 mm/0.07 deg OK, TOP DIVERGED (80 -> 185 mm; in the 40 recorded poses 2 frames show no arm in the top view, 6 < 5000 px). NOT yet run: SAM loop on the sweep36 poses (top camera), several offsets, distortion.
**Not done / caveats.** Nothing committed. Gazebo has no joint friction/rotor inertia (n=1 per variant). Patched controller overlay only in `/home/murilo/easyhec/gz/ws/install` (lab source untouched, not deployed; deploying = rebuild fr3_lab_controllers in the lab workspace + driver restart, user decision). Sim container `fr3-gz-w1` was still finishing.

## UPDATE 2026-10-15 late night: first complete dynamic sweep (Gazebo, unpatched lab controller)
- Planner now has `--q-ref` (controller posture at ACTIVATION) and `--max-dq`; plans around Q_NULL = [-0.15 0.25 0.05 -2.06 0.23 2.27 0.66] in `calib/gz_sim/plans/sweep_n{9,7,6,5}.json` (max|q-q_ref| 0.9/0.7/0.6/0.5; ready-pose q_ref gives ~0 feasible poses: not in both camera images).
- Gazebo can START at any posture: `gz_sim/gz_variant3.sh` + `mk_xacro.py` mount a private copy of franka_ros2_control_macros.xacro (lab files untouched), so q_d at activation = that posture. `launch_q.sh` wraps it. `exp_sweep.py --recapture` re-activates the controller after every waypoint (q_d := current joints).
- RESULT: plan n7, posture stiffness 5, unpatched controller, `--leash 0.03 --max-lag 0.025 --rot-leash 30 --max-rot-lag 45`, activation at Q_NULL: **36/36 in 341 s, repeated 36/36 in 342 s** (runs d2, e1). Steady hand-orientation error 18 deg mean / 36 max (posture leak persists; irrelevant for calibration: joints are measured). Achieved joints: min joint margin 0.71 rad, arm inside BOTH recorded-frame camera images at all 36 (min margin front 82 px / top 29 px), self-clearance 0.17 m, floor 0.10 m (`check_achieved.py`).
- NOT robust: n9 23/36, n6 2/36, n7 at k10 22/36 (all rotation-gate stalls); k0/k1 hit joint limits in 1-3 poses; `--recapture` did not help (1-7/36). Orientation lag gate is the bottleneck: the rot-leash gate must be >= ~30 and max-rot-lag ~45 (runner caps rot-leash at 30).
- Not done: real-robot run (needs: controller ACTIVATED at Q_NULL posture = move arm there at standstill then real-hold/recover; runner args as above; user pastes commands; table clear, e-stop); runner `--leash/--max-lag` defaults 0.03/0.025 match; n7 plan must first be re-planned if the live camera fits change. Leftover: container fr3-gz-e4 looked hung (own container; remove on user's word).

## UPDATE 2026-10-16: EasyHeC refit on LIVE captures of the real arm, held-out validated (NOT published to TF yet)
- Data: 91 passive captures (live_calib_run 11 + live_calib 34 + live_calib_sweep 46; 90 distinct poses, wrist yaw -177..180 deg, tilt up to 69 deg, hand x 0.18-0.58 m) of BOTH ZEDs + real link poses, copied to `/home/murilo/easyhec/data/live_all/` (lab PC). Rectified 640x360 images + live camera_info K (no distortion model needed).
- Code `calib/live_easyhec_1016/hec_live.py` (lab: `/home/murilo/easyhec/hec/hec_live.py`, runs in easyhec-build on GPU 1 via `/home/murilo/run_live1.sh CAM START SEED`); passive grab `live_now.py`; depth validation `hec/live_validate.py`. Results JSON + overlays + `held_out_summary.png` in `calib/live_easyhec_1016/`.
- Held-out results (fit on 70 (zed1) / 47-50 (zed2, arm clearly visible in 66 of 91), tested on the rest): zed1 mask IoU 0.52 -> 0.78-0.82, depth median 13.4 -> 8.1-8.3 mm, within 1 cm 39 -> 59 %; zed2 IoU 0.53 -> 0.85-0.87, depth 21.2 -> 4.0-4.1 mm, within 1 cm 22 -> 81 %.
- Convergence: zed1 recorded-start vs depth-start fits 2.4 mm / 0.16 deg apart; split0 vs split1 1.0 mm / 0.28 deg. zed2: depth-start split0 vs split1 3.2 mm / 0.35 deg, BUT recorded-start fit is 24.7 mm / 3.7 deg away (worse metrics) -> top fit depends on the start; use the depth-start one.
- Fits (T_base_cam, camera optical frame in fr3_link0): zed1 camera origin (0.286,-0.609,0.633) m (recorded fit was (0.226,-0.612,0.633): +60 mm along base x, 7.5 deg); zed2 (0.648,-0.029,0.914) (recorded (0.646,0.010,0.941)). Full matrices in `zed{1,2}_depth_s{0,1}.json` (`T_base_cam_fit`).
- NOT met: my own bar of <5 mm / 0.5 deg held-out depth-ICP residual. Residual after the fit: zed1 14-16 mm / 0.4-0.7 deg, zed2 17-23 mm / 1.9-2.3 deg (largely along the optical axis for zed2). Silhouettes constrain range weakly and ZED depth scale is unvalidated, so the remaining part is unattributed (mask fit vs depth bias).
- Tag cross-check (table tag, AprilTag 36h11 id 16, 0.14 m; tag measurement itself noisy, rot spread ~2 deg, t std tens of mm): fit-implied zed1->zed2 differs from tag-derived by 125 mm / 4.7 deg for the recorded fits, 25 mm / 4.4 deg for the new fits.
- Not done: republish static TFs / nvblox with the new fits, touch or tag-on-hand test, repeat on another day, joint mask+depth objective, wrist D435i.

## UPDATE 2026-10-16 (later): joint mask+depth fit, NEW POSES PUBLISHED to the live TF tree
- `calib/live_easyhec_1016/hec_live2.py`: same EasyHeC loop plus a truncated point-to-model depth term (weight 1, both terms normalised by their start value), optional free depth scale (`--free-scale`). Results in `calib/live_easyhec_1016/results_v2/`; run helper `run_live2.sh`.
- Held-out (fit on 70 / 47-50 frames, rest held out), two splits: zed1 IoU 0.78-0.81, depth median 5.7-6.3 mm (mask-only fit: 8.1-8.3), within 1 cm 69-74 %, depth-ICP residual 3.6-4.3 mm / 0.3 deg (meets the 5 mm / 0.5 deg bar). zed2 IoU 0.85-0.87, depth median 3.9-4.1 mm, within 1 cm 82 %; its depth-ICP residual stays 17-18 mm / 1.8 deg BUT applying it changes the held-out median only 3.85 -> 3.94 mm: an ill-conditioned (sliding) direction, so that residual is NOT a valid error measure for the top camera (my 5 mm bar was the wrong criterion for it).
- Depth scale (free-scale runs): zed2 1.0002 (ZED depth agrees with mask geometry), zed1 0.975 (2.5 % tension between depth and masks at ~1 m: either ZED depth bias or ~2 cm of range error; the s=1 and free-s fits differ by about 5-12 mm in camera position). Not resolved; physical test needed.
- FINAL fits on all usable frames (zed1 91, zed2 66 with arm clearly in view), within 2.3-3.4 mm / <0.5 deg of the split fits: zed1 camera origin (0.3007,-0.6169,0.6318) m (44 mm / 8.3 deg from the recorded fit), zed2 (0.6555,-0.0287,0.9118) m (59 mm / 5.2 deg). Files: `calib/live_easyhec_1016/calib_live1016/{front,top}_camera_extrinsic.json` (lab: `/home/murilo/easyhec/calib_live1016/`, zed-build `/tmp/calib_live1016/`).
- PUBLISHED 10-16 ~13:15 local: the two `real_extrinsics_tf.py` publishers in zed-build were stopped (old PIDs 3046255/3046256, one at a time) and restarted with the new files (log `/tmp/tf_zed{1,2}_live1016.log`, detached). `tf_verify.py`: TF fr3_link0 -> zedN_left_camera_optical_frame equals the new calibration within 0.0 mm / 0.03 deg (old calibration kept in zed-build `/tmp/calib/`; to roll back, restart the publishers with those files).
- Live check with the new poses on a fresh frame (arm still, TCP 0.522 0.213 0.436): zed1 median 5.5 mm, 77 % within 1 cm (recorded fit: 14.3 mm, 35 %). zed2 fresh frame is NOT a usable test (arm mostly cropped at the top-right corner: median 25 mm, correction meaningless); on the fit set zed2 is 3.7-4.0 mm. nvblox (isaac-build) keeps running; its static map contains ghost geometry built before the swap: restart `/tmp/run_nvblox_real.sh` for a clean map; still do not feed MPC.
- Still open for the lab visit: touch / tag-on-hand test (decides the zed1 range tension), a frame with the arm clearly inside the top view as a zed2 check, optional repeat after any camera bump, then sweep the robot (see the 10-15 late-night sweep notes: controller activated at the plan's reference posture, `--rot-leash 25`, loosened guards).

## UPDATE 2026-10-16 (afternoon): the real runner rehearsed in the Newton sim (no feed-forward, lab controller law)
- User, arriving at the lab: "forget the feedforward, I want it faithful to the controller of the robot we're using ... run the calibration in newton and record a video; if good I run it on the real robot".
- Harness (lab `/home/murilo/easyhec/newton_calib/`, copies in `calib/newton_sweep_1016/`): `run_calib_newton.sh` starts the other lane's Newton sim (`hub-curobo/newton_sim/newton_fr3_sim.py --controller cartesian`, `velocity_feedforward` untouched = 0) at the plan's reference posture Q_NULL with the real working set (Kt/Kr 1000/40, clips 0.02/0.10, target filter 0.05, rot damping 0.4, posture stiffness 5), a shim (`newton_state_shim.py`) turns /joint_states into FrankaRobotState, then the UNCHANGED `/home/murilo/franka_ws/lab/calib/real_calib_sweep.py` runs dry-run + `--go --yes`. Own ROS domain 85, cores 11-15. Video: `render_sweep_video.py` (nexus, curobo-isaac6 venv).
- Result (run e, plan n7, 36 waypoints, 407 s sim, rtf 0.88): 36/36 reached and settled, 0 aborts, 0 warnings; hand lag during moves mean 6.8 / max 13.6 mm, rotation lag max 11.3 deg, dwell position error mean 4.7 / max 12.5 mm (posture-torque leak through the damped pseudo-inverse: net joint torque ~0 at rest, checked with an independent J^T K e), dwell spread <= 0.35 mm, min joint margin 0.77 rad, TCP z >= 0.151 m, clip never active. NOT modelled: force estimate (guard not exercised), firmware reflexes, real friction.
- Runner bug found and fixed (deployed to lab, md5 9d9fc079..., backup `real_calib_sweep.py.bak_before_settle_fix`): the settle test needed >40 samples in 1 s but the loop runs slower, so `settled` was always False and every waypoint waited the full 8 s timeout. Now time-based (>=8 samples spanning >0.9 s). Would have hit the real arm too.
- Plan n7 copied to lab `/home/murilo/franka_ws/lab/calib/sweep_n7.json`. Still unchecked: n7 image margins under the NEW camera fits. Precondition on the real arm unchanged: controller activated with the arm at the plan's reference posture.

## UPDATE 2026-10-16 (evening): REAL sweep done, top camera refit and republished
- Arm: controller was off (robot idle). Sequence used (no hand-guiding): `real-hold` + `real-fast` at the standing pose, `fr3_home.py --pose=<Q_NULL>` (hand to the reference hand pose), quick deactivate/activate to re-latch the posture reference (the controller latches q at on_activate; no parameter), `nullspace_stiffness 5`. Measured posture reference [0.207 0.166 -0.279 -2.214 0.257 2.465 0.741] (0.36 rad from Q_NULL). Replanned around it with the live camera fits: `calib/newton_sweep_1016/plans_qa/sweep_qa_{n7,n9,n6}.json` (n7 used; lab `/home/murilo/franka_ws/lab/calib/sweep_qa_n7.json`). Helper `calib/real_steps.sh` (where/watch/hold/ref-dry/ref-move/dry/first3/full), `qnow.py`.
- REAL SWEEP (plan qa_n7, 36 poses, ~10 min, runner with the settle fix): 36/36 reached and settled, 0 warnings, 0 aborts, hand lag max 20 mm (mean pos error 9.3 mm), rot lag max 17 deg, min joint margin 0.70 rad, force estimate max 6.2 N. First 3 poses run separately before. Newton rehearsal of the same plan (from the measured posture): 36/36, max lag 10 mm.
- 34 new captures (zed-build /tmp/live_calib_run/011..044) -> lab `data/live_all/live_sweep_real/`. Out-of-sample check of the then-published fits on them: zed1 median 5.0 mm, 82 % within 1 cm (depth correction 7 mm / 0.9 deg, fine); zed2 median 7.7 mm, 62 % (depth asks 38 mm / 2.9 deg, so the old top fit lacked these poses).
- zed2 refit with all 125 captures (hec_live3.py = hec_live2 + the new set; 100 usable frames): held-out split (74 fit / 26 held-out) IoU 0.49 -> 0.76, depth median 22.2 -> 5.5 mm (74 % < 1 cm), residual depth correction 10 mm / 1.1 deg; all-frames fit is 6.4 mm / 0.9 deg from the split fit and 15.6 mm / 1.25 deg from the previous published pose. PUBLISHED: `/tmp/calib_live1016/top_camera_extrinsic.json` in zed-build (backup `.bak_before_sweep_refit`, also in `live_easyhec_1016/calib_live1016/`), only the zed2 static-TF publisher swapped (new PID 3581523, log /tmp/tf_zed2_live1016_sweep.log); tf_verify: TF equals the new file within 0.0 mm / 0.04 deg. zed1 unchanged (3520956). New zed2 origin (0.6642, -0.0413, 0.9085) m.
- Not done: zed1 refit with the new frames (not needed by the check), zed1 depth scale 0.975 vs 1.000 (touch test), a frame with the arm clearly inside the top view as a final zed2 check, nvblox restart for a clean map. Lessons: three EasyHeC jobs at once on the 12 GB GPU OOM (run two); the lab PC load average reached 38 with the cuRobo containers (evaluation is CPU-bound, ssh timed out for a while).

## STATE AT CLOSE (2026-10-16 night, chat closed) - READ THIS FIRST
- DONE: Newton rehearsal -> real 36-pose sweep (36/36) -> zed2 refit published (see evening section). Nothing is committed anywhere (commit only when asked; no Claude co-author lines).
- Published calibration now: zed1 = 10-16 joint mask+depth fit (origin 0.3007,-0.6169,0.6318); zed2 = sweep refit (origin 0.6642,-0.0413,0.9085). Files in zed-build `/tmp/calib_live1016/`, repo copy `calib/live_easyhec_1016/calib_live1016/`; rollback = `top_camera_extrinsic.json.bak_before_sweep_refit` + restart the zed2 `real_extrinsics_tf.py` publisher (current PID 3581523; zed1 PID 3520956).
- Real robot: `fr3-ros2` (REAL driver, domain 77, shared with others, restarted by someone else ~1 h before close) has `cartesian_impedance_target_controller` ACTIVE at Kt/Kr 1000/40, posture stiffness 5 (a fresh `real-hold` loads 20), holding its last pose. Nothing of mine commands it. User was told it is free to move. Before any new motion: user present, table clear.
- Still running: `/tmp/live_recorder.py` in zed-build (cap 80, ~45 captures). No EasyHeC fits running; lab GPU 1 is free (cuRobo lane needs lab GPU/CPU: keep load low, run at most two fits at once).
- Open, only if the user asks: (1) touch test for the zed1 depth scale (0.975 vs 1.000); (2) a frame with the arm fully in the top view as a final zed2 check; (3) clean nvblox restart (`/tmp/run_nvblox_real.sh` in isaac-build; still do not feed MPC); (4) optional zed1 refit with the 34 new frames (`hec_live3.py`, set `live_sweep_real`); (5) L40S migration of the fits was requested but not done (nvdiffrast needs nvcc, absent on nexus; 7 GB env in a container) - lab GPU is free now.
- Standing constraints on the lab PC: do not delete anything; do not kill the user's Gazebo; do not browse /home/hipolito; ssh via script files + absolute /home/murilo paths; background jobs fully detached; kill by PID separately.
- Where things are: runner `calib/real_calib_sweep.py` (lab `/home/murilo/franka_ws/lab/calib/`, md5 9d9fc079...), step helper `calib/real_steps.sh`, Newton harness `calib/newton_sweep_1016/` (+ lab `/home/murilo/easyhec/newton_calib/`), fits `calib/live_easyhec_1016/` (`results_sweep_refit/` = latest), new capture set lab `/home/murilo/easyhec/data/live_all/live_sweep_real/`. Memory note: `fr3-camera-calibration-easyhec.md`.

## UPDATE 2026-10-31 — ZED 2 dashboard outage diagnosed and recovered
- User reported ZED 2 missing from dashboard. Lab host `murilo@100.88.134.127` (`hipolito-Precision-5860-Tower`) was reachable. Dashboard page on `:8788` contains the Camera 2 card and the correct topic `/zed2/zed_node/left/image_rect_color`; its HTTPS-aware `snap?topic=` path was intact.
- Root cause: zed2 wrapper launch had remained alive while its ROS component had died. USB still enumerated HID serial `39498278`; `/tmp/zed2.log` in `zed-build` showed repeated `Connection issue detected: CAMERA REBOOTING`. ROS topic had 0 publishers, and `ros2 node info /zed2/zed_node` returned node not found. Reproduced dashboard request: zed1 returned JPEG, zed2 snapshot timed out.
- At user's request, restarted only zed2 in container `zed-build`: backed up old log to `/tmp/zed2.log.bak_before_restart_20261030`; terminated the stale zed2 launch/component/state-publisher processes inside the container; launched detached with `ROS_DOMAIN_ID=8`, `RMW_IMPLEMENTATION=rmw_fastrtps_cpp`, `zed_camera.launch.py camera_model:=zed2i camera_name:=zed2 serial_number:=39498278 publish_tf:=false`, writing `/tmp/zed2.log`. Fresh ROS launch PID observed as 4184019 (container PID; re-check before relying on it).
- Verified recovery: `/zed2/zed_node/left/image_rect_color` now has 1 publisher; `ros2 topic hz` measured ~15.9 Hz; dashboard's local `:8788/snap?topic=/zed2/zed_node/left/image_rect_color&width=480&height=270&quality=60` returned HTTP 200, fresh 480x270 JPEG (~11 KB) in 0.08 s. Dashboard should show Camera 2 again.
- Secondary issue remains: `web_video_server` on host `:8787` is bound to `127.0.0.1` only (process args `address:=127.0.0.1`, log `/tmp/web_video_server_restart_claude.log`, up ~11 h at investigation). This breaks direct remote `:8787/stream` for all cameras; dashboard's local snapshot proxy via `:8788` works. If direct stream access is needed, change/restart its bind address deliberately.
- Follow-up: check `/tmp/zed2.log` for a new `CAMERA REBOOTING` recurrence and confirm topic/snapshot remain live. If it recurs, inspect USB3 port/cable/power; two ZED video interfaces appear on USB bus 002. No camera setting, dashboard source, calibration, robot motion, or other container was changed.

## UPDATE 2026-11-02 — zed2 wrapper restarted again (different killer)
- Symptom as before: launch alive, `/zed2/zed_node` gone, 0 publishers. But this time NOT a reboot loop: log shows the node ran fine on SN 39498278 from Oct 31 22:11, took corrupted-frame warnings + an ISP-lock `CAMERA REBOOTING` episode Nov 1 09:27 UTC (recovered), then on Nov 2 ~17:00 UTC a SECOND `ZedCamera` component was loaded into the live `/zed2/zed_container` with sim params (ZED X, SN 0, sim port 30010) and crashed the whole container (`general.pub_frame_rate` InvalidParameterValueException, exit -6). "Publisher already registered" in the log confirms the duplicate load. Who ran the component load is unknown; old log backed up at `/tmp/zed2.log.bak_before_restart_20261102`.
- Restart (user asked): SIGINT then SIGTERM to stale PIDs 174470/174500 (now harmless zombies), fresh detached launch PID 220797 with the same known-good command (`camera_model:=zed2i camera_name:=zed2 serial_number:=39498278 publish_tf:=false`, domain 8, fastrtps). Verified: node back, 1 publisher, ~20 Hz, dashboard `:8788/snap` 200 with fresh 480x270 JPEG, 0 errors in the new log.
- zed3's node is ALSO down and was left alone (out of scope): its USB serial 34046618 is not enumerated (only 39498278 + 33874668 on HID, two video interfaces on bus 002) — likely unplugged. If zed3 is needed, check the physical USB first, then relaunch with `serial_number:=34046618`.

## CLOSE 2026-11-02 (zed2-restart lane) — done, read this first
- DONE: zed2 wrapper restarted and verified (see UPDATE above). `lab-zed-cameras` claimed for the restart and RELEASED. Nothing of mine is still running on the lab PC; no files changed there except `/tmp/zed2.log` (+ backup `.bak_before_restart_20261102`). Repo change: only this file (not committed).
- Cross-lane state (via duet, unverified by me): Hipolito's `franka_teleop` lane stood down on the arm (controller deactivated back to baseline, grasp run goes first) and REVIVED the zed3 wrapper (~14 Hz, so SN 34046618 reappeared on USB after my check — the zed3 bullet above is stale). They measured the calib tag from zed1+zed2 at 23:34Z and confirm zed2 MOVED (~17 deg tilt, ~0.28 m shift vs 10-31). Their plan: recalibrate only after zed2 is remounted.
- LEFT for next agent: (1) zed2 needs a physical remount + extrinsic refit before its TF is trusted; (2) stay off arm/controllers/cameras/nvblox until the grasp lane posts its resource release; (3) if zed2 dies again the same way (sim component in its container), find who is running the component load — `docker events` / shell history on the lab PC.
- WHERE: lab SSH `murilo@100.88.134.127` with key `~/.ssh/id_ed25519_lab_claude` (plain ssh as murilo is denied); container `zed-build`, domain 8; launch PID 220797 at close (re-check, PIDs churn).
