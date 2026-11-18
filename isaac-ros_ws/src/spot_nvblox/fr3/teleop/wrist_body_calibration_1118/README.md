# Operator-arm calibration from recorded ZED3 bags

This is operator calibration using **the person's shoulder, elbow and wrist**, not camera-to-robot extrinsic calibration. It adds an opt-in `rgbd_body` mode to the current RGB-D hand tracker. Live services and the real robot were left untouched, as requested.

## Source comparison

Checked both local spot-teleop's `spot-ros2_ws/src/arm_pose_estimator/arm_pose_estimator/wrist_detector.py` and [open-vocab-shared-autonomy's wrist_detector.py](https://github.com/murilo-vinicius04/open-vocab-shared-autonomy/blob/main/spot-ros2_ws/src/arm_pose_estimator/arm_pose_estimator/wrist_detector.py). `update_arm_length` estimates `norm(elbow-shoulder)+norm(wrist-elbow)`, checks a rolling consensus, and latches `robot_reach / measured_arm_length`. The body frame comes from shoulder/torso landmarks; missing landmarks hold the last accepted frame. The downloaded reference hash and runtime-source hashes are in `source_manifest.json`.

The running lab tracker uses `rgbd_fixed` without arm calibration and explicitly disables the body reference. Its fixed position gain is 1.315. Enabling the legacy arm tracker alone would discard the newer RGB-D wrist/orientation improvements. This candidate combines those improvements with an explicit operator reference.

## Changes

- `rgbd_body` initializes arm calibration automatically, while `rgbd_fixed` remains compatible with the previous behavior.
- Upper-arm and forearm lengths must agree **separately** across a dominant sample cluster. This rejects opposing depth errors that cancel in total length. Nonfinite/repeated timestamps cannot enter calibration. Scale is latched and does not drift with arm flexion.
- Body axes require a stable multi-frame consensus and latch once. Shoulder-line image roll is calibrated while camera-facing forward heading remains fixed; unreliable torso depth never estimates pitch/yaw. This deliberately assumes the upright, fixed operator camera used by these bags. Raising an arm, a missing hip, or a noisy shoulder cannot subsequently rotate the control axes. The wrist is made relative to the reference **before** rotation, avoiding the torso baseline becoming a translation lever arm.
- A filtered, rate-limited reference cancels common torso translation. Current shoulders/hips must agree with their per-landmark calibration templates; an occluded shoulder cannot shift the reference alone. With insufficient agreement the reference holds; `--shoulder-mode off` explicitly disables that cancellation. The last calibrated reference persists through body occlusion, while the hand still requires current RGB-D measurements. Diagnostics expose reference age rather than relabeling held landmarks as fresh.
- Normal Start preserves accepted arm/body calibration; only explicit `reset_person` clears it, including the pending saved profile so a new person cannot silently inherit old lengths. `rgbd_body --no-body-frame` is refused rather than silently ignoring an option.
- Anatomical wrist depth and palm-orientation validity gates remain active. Sample metadata exposes the calibrated position gain/reference mode. Camera-space debug geometry remains camera-space geometry.
- The new profile uses input divider **1.0**, because arm normalization is already applied by perception. The legacy fixed-camera divider 1.315 would partially undo it. Extra amplification is a separate `zed_scale` choice. See `profile.json` and `test_consumer_gain.py`.

## Verification

`run_comparison.sh` re-infers all eight Nov13 ZED3 bags from compressed RGB-D at 1x wall time. It preserves recorded capture-to-delivery delays, duplicate acquisition identity, and gaps between takes. The fixed-camera baseline and first body trial start cold; the latter exposed a 37-second startup delay. The selected candidate explicitly loads arm lengths fitted from take04 only, then establishes fresh body axes/origin from current images. Seven other takes are held out for arm-length validation. No cached hand poses or desired robot trajectory is injected. The same process spans the session so later takes test the latched calibration. The bags/repo/model mounts are read-only; the container uses `--network none`, four CPUs and ROS domain199. Publisher sinks record perception only. There is no controller, motion relay or physical robot connection.

The task uses the installed `spot-teleop-isaac-ros:latest` image, the repository's private MediaPipe/OpenCV packages and an existing hand-landmarker model. No model weights or bag data are committed. `REPO_ROOT` and `MODEL_DIR` can override the launcher defaults. Large per-frame traces/logs remain in `/home/hipolito/franka_teleop_checks/wrist_body_calibration_1118`; compact results and analysis are checked in.

Run pure tests:

```
python3 runtime/test_arm_calibration.py
python3 runtime/test_operator_reference.py
python3 test_consumer_gain.py
```

The consumer test loads the sibling `wrist_reach_fix_1117` mapping kernel without ROS or robot hardware. `python3 analyze.py` summarizes local replay traces. See `RESULTS.md` for measured results and limits.

No change here calibrates the robot-view ZED1/ZED2 extrinsics, validates drill contact height, or authorizes an automatic pickup. Those are separate outstanding tasks. Live ZED3/RealSense USB faults reported by the camera lane do not affect recorded-data validation.
