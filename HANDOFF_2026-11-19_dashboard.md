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
- Inspected live MPC configs: `fr3_vcap1.2_acc10.yml` has no explicit table primitive; it omits `fr3_link0` collision spheres because the base is assumed to stand on the mapped table. `mpc_override.yml` has no table geometry either. Murilo was asked whether to add an explicit table collision surface at touch plane z=0.0287 m before any grasp test. Keep MPC/teleop/grasp inactive until this is resolved and the FR3 UDP timeout/deactivated-driver fault is cleared.
- The offline wrist replay uses a 1.4 x 1.1 m table cuboid centered at x=0.35 m, but those are simulation dimensions and are NOT confirmed lab table bounds. Do not copy them into the live MPC. A safe finite collision box needs measured X/Y tabletop bounds in `fr3_link0`, plus a verified way to combine that primitive with the live voxel ESDF. Asked Murilo to confirm bounds and approve that addition; until then the carved pocket remains a blocker for teleop/grasp.
- Dashboard patch and FR3 model proxy remain as above. Pre-rebuild mesh backup: `/tmp/wrist_map_rebuild_20261119_1950/premap_mesh.bin`, SHA256 `fd20c7d2f450750d12c3f3b3dd0c4abe3b97d65abf9e160ab27b06306731711c`. Post-rebuild mesh SHA256: `6651df5ca78a861e63724f47655e524e2f6b46fb8a03853b9d414afdeaed8e59` (captured while integration continued).
