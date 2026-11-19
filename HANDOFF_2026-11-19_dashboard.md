# FR3 dashboard visibility handoff — 2026-11-19

Goal: restore ZED visibility and robot/camera context in the 3D lab dashboard, then finish drill-mask/map validation. No robot motion was issued.

## Current facts

- Live ZED RGB topics are /zed1/zed_node/rgb/image_rect_color and /zed2/zed_node/rgb/image_rect_color; the deployed dashboard used stale /left/image_rect_color routes and remote clients attempted :8787, which is loopback-only. Same-origin snap?topic= on dashboard :8788 works.
- Dashboard file: isaac-build:/tmp/dash/lab_dashboard.html; mesh_server serves 8788/8795.
- The 3D robot mesh proxy defaulted to 127.0.0.1:8088 while FR3 visuals are served at 127.0.0.1:8089. Murilo approved a loopback 8088-to-8089 forwarder; the lane reports /config 200 and /stream returning FK poses. Browser reload should load the model.
- Deployed ZED1 and ZED2 poses are available. ZED3 and wrist RealSense currently have zero frames / physical USB faults.
- Tracker mask exists for the drill, but only ZED1 was seeded; SAM overlays and per-camera robot segmentation endpoints were reporting zero frames at the time of checks. No live four-object mask publisher exists.
- Calibration fit scale 0.975537 was approved by Murilo and applied in `drill_depth_filter.py`; the single map rebuild is complete. Current map policy and its validation are recorded below.

## Dashboard patch

Tracked patch script: spot-ros2_ws/src/spot_operation_ros2/tools/patch_lab_dashboard.py.
It switches remote camera panels to same-origin snapshots, uses active ZED RGB topics, and draws calibrated ZED1/ZED2 frustums and labels in fr3_link0 with a toggle. It does not touch ROS, TF, map, or robot.

Deploy workflow: docker cp isaac-build:/tmp/dash/lab_dashboard.html /tmp/lab_dashboard.html; run the patch script on that copy; then docker cp the patched copy back to isaac-build:/tmp/dash/lab_dashboard.html. Verify :8788/snap?topic=... for both cameras and reload the dashboard.

## Latest driver observation

At 19:43:59Z the fr3-ros2 log recorded libfranka UDP receive timeout; the hardware interface and joint_state_broadcaster, franka_robot_state_broadcaster, and fr3_arm_controller were deactivated, and the gripper node exited. The last published sample before that showed robot_mode 0 (OTHER) and all current error/reflex booleans false. No recovery, controller activation, or motion was attempted by this lane. This coincided with the operator reporting the robot indicator red, but does not establish the physical LED cause.

## Verification after dashboard update

Dashboard GET returned 200 (about 100 KB); same-origin ZED1 and ZED2 snapshot proxies each returned 200 with 640x360 JPEGs. Through mesh_server, /config returned 11 FR3 visual links and /stream included fr3_hand after Murilo's approved loopback forwarder came up. The edited module passes node --check. Reload the browser to pick up the camera-topic/pose-marker edits; robot pose freshness is uncertain while the FR3 driver is faulted.

## 2026-11-19 map rebuild and current blocker

- Murilo authorized policy A and one rebuild: selected target is the drill; no target selection passes full depth; selected + fresh TRACKING mask carves only the drill; selected + LOST/stale mask pauses map integration. Live tracker is ZED1, `/tracking_state` only emits TRACKING/LOST, so an explicit `/wrist_map/target_selection` bridge disambiguates no-selection from lost-target.
- `drill_depth_filter.py` now applies depth scale `0.9755371809` to positive finite depth (preserves zero/NaN), consumes `/vm/view_mask` (already dilated 2 px; filter dilation 0), reports scale in status, and uses fail-closed target state. Tests: 9 passed; Python compile passed.
- Exactly one `/nvblox_curobo` rebuild was done after scale + bridge; filter status reported `ACTIVE:drill`, mask/depth stamp delta 0 ms, outside mask unchanged, carved fraction ~1.64%. The latest mesh was 14,364 vertices / 19,611 triangles (still integrating). ESDF samples: drill carved/unknown; ball, Sprite bottle, and screwdriver remain occupied.
- Independent read-only validation by Morys: table plane z=29.4 mm at (0.5,-0.1), tilt 0.24 deg, 505 points (touch reference 28.7 mm; fit 30.7 mm). Ball and bottle clusters are present. Drill region has a 10–12 cm unknown-space pocket and an occlusion shadow behind it: at table-level z=0.01 m, 62% of cells are unknown within 8 cm and 38% within 12 cm. Do not assume carved/unknown space is collision-safe.
- Inspected the live MPC source/config: it builds a voxel-only world; `fr3_vcap1.2_acc10.yml` omits `fr3_link0` collision spheres because the base was assumed to stand on the mapped table. Murilo approved adding a static table cuboid alongside the voxel ESDF. His mesh-derived XY proposal is x=[0.25,0.95], y=[-0.55,0.60] in `fr3_link0`; negative-y extent is only the ZED1 view limit and XY has not been touch-validated. Fingertip touch measured z=0.0303 m at A (0.42,0.08) with firm contact, 0.0347 m at B (0.68,-0.12) and 0.0345 m at D (0.58,-0.32), the latter two soft-plateau upper bounds. Preview box top is set conservatively to 0.040 m (5.3 mm above highest touch reading), thickness 0.10 m. Tape-check XY bounds before using the geometry in the live MPC.
- Added an opt-in table cuboid to the passive MPC preview world builder (`teleop/curobo/table_collision.py` and `joystick_mpc.py`). Four geometry tests and Python syntax checks pass. Fingertip touch gives a firm table height of 30.3 mm at A; B and D reached soft-plateau upper bounds 34.7 and 34.5 mm. Preview top is therefore 40 mm (5.3 mm above the highest reading); thickness 100 mm. XY bounds x=[0.25,0.95], y=[-0.55,0.60] remain mesh-derived and need tape verification.
- Passive preview runtime succeeded at 2 cm ESDF resolution: ESDF service responses 15–32 ms, 250k/350,811 occupied voxels, ESDF updates accepted, combined collision checker enabled (`voxel=True`, `primitive=True`), solver stayed `status=OK` with `coll_cost=3.3`. No wrist goal was supplied, so this verifies startup/map ingestion/collision-world composition only, not target reachability or table-contact behavior. Preview ran in ROS domain 8 with `safe_packet_port=0` and joint output remapped to `/fr3_joystick_mpc/joint_preview`; real `fr3-ros2` is domain 77, `fr3_arm_controller` was INACTIVE, and `/fr3_arm_controller/joint_trajectory` had zero publishers. Preview container was stopped after the check.
- Keep live MPC/teleop/grasp inactive until the XY table bounds are physically tape-checked, an explicit goal-based simulation confirms table collision behavior, and the operator confirms the FR3 driver/FCI state. The map contains a 10–12 cm unknown pocket around the carved drill; unknown is treated as free by the current MPC, so the table primitive is required.
- Dashboard patch and FR3 model proxy remain as above. Pre-rebuild mesh backup: `/tmp/wrist_map_rebuild_20261119_1950/premap_mesh.bin`, SHA256 `fd20c7d2f450750d12c3f3b3dd0c4abe3b97d65abf9e160ab27b06306731711c`. Post-rebuild mesh SHA256: `6651df5ca78a861e63724f47655e524e2f6b46fb8a03853b9d414afdeaed8e59` (captured while integration continued).


## 2026-11-19 physical bag/grasp request — preflight blocked

- Operator requested real-robot replay of the hand bag followed by table grasps: drill, tennis ball, Sprite bottle, Phillips screwdriver; operator reports e-stop attended. No physical command was issued.
- Read-only FR3 state: `fr3_arm_controller` INACTIVE, 0 publishers on `/fr3_arm_controller/joint_trajectory`; state broadcasters active. Dashboard SSE is disengaged and had no `zed` exercise profile. `http://127.0.0.1:8784/status` refused connection; `fr3-zed-relay`/physical bag follower is absent. `real_bag_replay.py --take 06_cima_baixo` without `--execute` rejected at preflight (`Wrong backend or exercise; configure paused bag follower`).
- Morys verified no tape-measured table XY limits; current bounds are depth-derived estimates. FCI/robot mode was not re-read and is unconfirmed. `grasp_plan_lib`/broker is drill-only; no validated ball, bottle, or screwdriver grasp path. Morys explicitly requested Murilo's written confirmation of table clearance and readiness before any motion. Duet request sent, routed to `codex:zed-recording-replay-compare` queue; no reply yet.
- An isolated Isaac bag-test startup loaded the FR3 model but reported zero object triangles (`drill`, `bottle`, `mug`, `box` all 0), so that scene cannot validate tabletop grasps. Replay did not start: launcher found an old stopped `fr3-bag-control` container (exit 143, Oct 30); inspect/log snapshot is in `remote_sim/run/bagtest_20261119/`. Isaac was stopped; GPU/sim and robot/tower claims were released.
- Next: wait for Murilo's direct readiness/bounds confirmation; verify/restore the physical bag follower and fresh MPC/map state; only then run the guarded replay. Do not attempt ball/bottle/screwdriver grasps until object-specific perception and grasp generation are implemented and validated.
