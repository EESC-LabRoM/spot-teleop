# Spot → FR3 ZED teleop comparison (2026-11-06)

Scope: source and saved-artifact review only. No physical robot command or motion was issued.

## Finding

The intended FR3 direction convention is correct in the deployed `cartesian_review.py` (SHA-256 `bc8cb884…`): camera-frame forward/depth `dX` lowers TCP Z, image-horizontal `dY` moves robot Y, and hand-up `dZ` extends robot-base X. A pure 10 cm input gives 9.2 cm on the selected axis (8 mm deadband) and zero on the other two. This matches the operator's stated intent.

The main mismatch is exercise isolation. The successful historical Spot ZED review used a separate profile per exercise. Its stage 05 (right/left) produced 40.01 cm horizontal with 0.33 cm reach and 0.06 cm height; stage 04 (forward/back) produced 40.00 cm height with 0.45 cm reach and 0.04 cm horizontal; stage 06 (up/down) produced 38.09 cm reach with 0.10 cm horizontal and 0.89 cm height. This is documented in `clips_out/zed_protocol_analysis/SIMULATION_RESULTS.md`. The current FR3 `full_6d` profile enables all three translation channels, so a nominal right/left gesture also passes any depth and vertical component in the tracked trajectory. It is not the right profile for validating a single-axis exercise. Use `horizontal` for stage 05, `height` for stage 04, and `reach` for stage 06; reserve `full_6d` for a deliberately combined free-motion test.

## What differs from Spot

- The Open-Vocabulary Spot tracker constructs an operator torso/body frame from same-frame depth and Pose landmarks (shoulders, with hip/ankle fallback), anchors wrist position at the shoulder, filters/rejects jumps, and estimates a human-arm-to-robot-reach scale. It publishes a body-frame wrist target. FR3 uses a fixed camera-optical-to-robot-base matrix (`CAMERA_TO_ROBOT`) plus its arm calibration and shoulder-relative adjustment. The fixed matrix assumes the camera is upright and facing the operator; a camera roll/yaw/pitch or an unmeasured mount change can rotate a screen-horizontal move into multiple robot axes. That is a credible second source of cross-axis motion and needs an extrinsic/axis check from recorded poses before changing the mapping.
- Spot's shared-autonomy release does use cuRobo MPPI at 50 Hz and nvblox ESDF, but its teleop reference is first conditioned in the body frame and then passed into that planner. Its older `spot-teleop` README also describes MoveIt Servo. These are different Spot paths; neither means that a raw camera delta should be sent directly to FR3.
- The old validated Spot exercise table is from a GPU-IK wrist-follow simulation, not a cuRobo+ESDF MPC replay. It validates the axis convention and isolation, not Franka controller dynamics.
- The current 8444 Quest/MuJoCo viewer explicitly runs with `use_esdf=false`; the displayed nvblox is a visual overlay, not cuRobo's collision input. The separate FR3 native-MPC pilot runner uses the native Murilo MPC in an isolated, network-disabled child and MuJoCo measured feedback, but the saved pilot used `feedback_mode=measured`, a 5 cm ESDF snapshot from Oct 31, and `full_6d`. It is not parity with the reported live hybrid setup and current 2 cm map.
- That native-MPC pilot did not reach replay: the measured seed produced native world-collision constraint 87 (joint bounds 0, self collision 0); it correctly stopped with zero accepted commands and zero simulated TCP movement. Re-run only after a fresh map snapshot, matching controller mode, and common simulation seed are available. Do not weaken collision gates to make the pilot pass.

## Verified artifacts and tests

- Public sources inspected: [EESC-LabRoM/spot-teleop](https://github.com/EESC-LabRoM/spot-teleop) (master `9e22d7b`) and [EESC-LabRoM/open-vocab-shared-autonomy](https://github.com/EESC-LabRoM/open-vocab-shared-autonomy) (main `6184a94`). The latter documents its body-frame wrist detector, ZED/MediaPipe, cuRobo MPC, and nvblox architecture.
- Current FR3 source: `teleop/fr3_input_teleop.py`; mapper: `teleop/review_defaults/backup_20261030_115620/cartesian_review.py`; tracker: `remote_sim/operator_hand_tracker.py` and `remote_sim/hand_tracking_math.py`.
- Pure-axis arithmetic check against the deployed mapper: right → `[0, +9.2, 0]` cm; forward → `[0, 0, -9.2]` cm; up → `[+9.2, 0, 0]` cm.
- `PYTHONPATH=isaac-ros_ws/src/spot_nvblox/fr3/remote_sim python3 -m unittest test_teleop_mapping -v`: 12 passed. These tests cover the separate legacy cylindrical mapping helper, not `cartesian_review`; the three arithmetic checks above directly exercised the deployed review mapper.

## Next simulation pass

Replay the same bag from the same neutral joint seed with the true native FR3 MPC and a current immutable ESDF snapshot. Run each intended-axis profile separately, then compare position-only and 6D orientation with timestamps, tracker packet axes, measured TCP axes, and target-vs-measured lag. Keep the Quest/MuJoCo viewer labelled visual-only until it consumes the planner's actual ESDF. If the horizontal test still couples into TCP X/Z after profile isolation, inspect the camera-to-base rotation/calibration and source wrist trajectory; do not compensate by arbitrarily changing gains.
