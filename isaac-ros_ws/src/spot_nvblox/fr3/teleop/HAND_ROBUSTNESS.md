# Hand robustness corrections — 2026-10-20

Saved before changes in local repository `/home/hipolito/fr3-zed-teleop-local`, commit `d2ac21a`, tag `before-hand-robustness`. The shared Spot repository index was preserved. Official `origin/dev` was fetched and compared; its grasp freeze in `hand_orientation_estimator.py` motivated the SO(3) clutch below. Existing Spot defaults and prior uncommitted changes remain.

## Corrected causes

- Orientation jump rejection compared a measurement against the lagging filtered quaternion. A continuous turn could accumulate enough lag to be rejected repeatedly, leading to tracking gaps. Rejection now compares consecutive accepted raw orientations. Smoothing uses a fixed response time across camera rates, with the existing 120 deg/s perception cap.
- A right-hand ROI uses Pose wrist/index/elbow landmarks, crops a square, and resizes to 256 pixels for hand inference. Results return to original image coordinates before association, drawing or depth lookup. Metric world landmarks keep their camera axes. The close-up gives the hand detector more useful pixels than full-frame detection; overlapping candidate hands are still rejected.
- Position now uses the precisely associated Hand wrist, matching the orientation/gesture result instead of mixing Pose and Hand wrist positions.
- FR3 `body_frame_mode: shoulders` uses shoulders and camera up. Occluded hips/ankles no longer veto a good shoulder frame. This assumes the fixed operator camera is upright; it is not full torso-tilt compensation. Hierarchical body fallback remains the default for Spot.
- FR3 robust depth accepts supported finite foreground clusters in 0.35–3.0 m and rejects empty patches, isolated depth pixels and distant background. Shoulder separation is checked. Position and orientation share an orthonormal body frame.
- Spot's two-second grasp orientation freeze is extended to 6D. The hand must still be freshly detected. A brief opening freeze and a new relative orientation reference prevent finger-shape changes from steering the tool when the freeze ends. Subsequent spatial rotations keep base axes. Tracking loss never uses this clutch to fabricate fresh detections.
- Repeated, out-of-order or already old camera frames are rejected before inference. Headless shutdown avoids calling OpenCV window cleanup when visualization is disabled.

Open-hand geometry, gesture debouncing, workspace and robot/controller checks remain in place. Robot speed settings and watchdog thresholds were not raised. No robot movement was used for this work.

## Validation

72 tests passed: 35 isolated Jazzy control/transport tests, 6 fake-backend HTTP tests, 18 geometry/filter tests, 5 actual hand callback/adapter tests, 4 scale tests and 4 synthetic RGB-D tests through the complete perception callback. The ROS tests use DDS domain 191 and mocked pose/gripper outputs. The Jazzy tests ran in a temporary container without external networking or hardware devices.

The synthetic turn regression uses identical inputs before and after: 17 samples of a 180 deg/s turn at 8 Hz. The previous filter rejects 6 samples; the corrected filter rejects zero, and limits each 1/8 s output step to 15 degrees. This verifies filter continuity and its 120 deg/s limit; it is not a measurement of robot tracking accuracy. The previous filter's reacquisition jumps were additionally bounded by the robot backend's watchdog/error limits.

Synthetic RGB-D tests verify shoulder-only operation with background depth at hips, original-pixel coordinates after cropping, consistent wrist position and orientation, fresh paired gesture packets during the intentional grasp lock, no output from missing wrist depth, and no new inference/output from replayed images. Geometry tests include incomplete fingers and fists rejected as open hands.

At application time the Franka container was absent and ZED3 was supplying no new images. Perception was restarted with the corrected code to wait for camera messages. The camera wrapper and robot hardware were not started. Physical trial of the revised capture and rotation remains necessary when the operator restarts the camera and robot stack.
