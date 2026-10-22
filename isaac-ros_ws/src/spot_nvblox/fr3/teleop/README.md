# FR3 joystick, ZED3 and dashboard

The main lab dashboard is port **8788**. Open its ZED3 card:

- **Start teleop ZED** selects ZED, waits for fresh stable tracking, captures the current hand/TCP reference and enables following. No joystick is needed. Initial arm-scale estimation runs automatically in perception; the overlay shows progress.
- Move the right hand for translation, and rotate it for relative roll/pitch/yaw. The initial robot orientation is preserved at Start.
- **Open_Palm** opens the Franka Hand; **Closed_Fist** closes it using the dashboard's grasp-force setting. The model is the same `gesture_recognizer.task` used by Spot's `hand_pose_estimator.py`. The canned classifier requires confidence >= 0.75; four extended fingers in valid metric 3D also qualify as Open_Palm when the palm is sideways. Commands require at least three distinct frames and 0.35 s of a consistent gesture. They run only while ZED is actively following; unknown gestures never imply opening or closing.
- **Home** first disengages and cancels pending calibration, then calls the existing `/lab/fr3_home.py`. This returns the TCP to the configured home pose without swapping controllers. Home does not re-enable teleop afterward.
- **Parar / abortar Home** cancels a pending Start, stops following and interrupts a running Home.

A Start request expires after 60 s if tracking never becomes stable. The neutral reference requires at least four new frames over 0.65 s, within 4 cm and 12 degrees. Source changes cancel it. The arm-length estimate is retained until perception restarts; Start recaptures the relative hand/robot reference.

Freshness remains **0.35 s**. On a brief loss, commands stop immediately. Within a maximum **1.5 s from the last valid sample**, a stable hand may resume the explicitly started session. Recovery re-anchors to the actual TCP and current hand, discarding motion made while tracking was absent. Longer losses disengage and require a new Start. Robot-state/controller faults disengage immediately. Stop and source changes prevent automatic recovery.

## Capture and control

ZED3 serial 34046618 is fixed at the operator station, not mounted on the arm. RGB-D perception runs in `fr3-curobo`, ROS domain 8, using the repository's wrist detector. The two mapping ZEDs and nvblox remain separate. Meta Quest is deferred.

FR3 configuration opts into `orientation_mode: full_6d`, `hand_geometry_source: model`, `hand_gestures: true`. Pose and gestures come from one synchronous GestureRecognizer result; the hand is associated with Pose's right wrist. Metric hand-model landmarks give the palm orientation, while aligned ZED depth gives wrist/body translation. This avoids requiring reliable depth at five small MCP patches when the hand turns sideways. Palm dimensions, planarity, quaternion jumps, body freshness and timestamps remain checked. The default geometry source for other uses stays `depth`; legacy roll remains available.

The palm frame is wrist-to-MCP +x, little-to-index +y, cross product +z. The fixed hand-to-TCP rotation maps TCP +z along fingers and +y across the palm, matching `fr3.urdf`. The body frame removes camera orientation. Full 6D control applies the hand's spatial rotation delta in base axes on the left of the anchored TCP quaternion. Euler angles are display-only and can be ambiguous near gimbal lock.

Legacy detector arm scale uses Spot's median/IQR estimator (90 samples). The active direct tracker uses the per-person consensus described below; scale = FR3 reach 0.855 m / estimated shoulder–elbow + elbow–wrist length. FR3 gates reject total lengths outside 0.45–0.95 m, segments outside 0.15–0.50 m and shoulder depths inconsistent by > 8 cm. These are capture gates, not verified anatomical measurements.

ZED and joystick share the same startup speed caps and dashboard multipliers. Current caps: 0.60 m/s XY, 0.40 m/s Z, 90 deg/s; at 50% the effective ceilings are 0.30 m/s, 0.20 m/s and 45 deg/s. Workspace, measured-pose error limits and follower limits also apply. **This direct Cartesian path does not enable cuRobo/PF obstacle avoidance.**

## Runtime

One unified teleop process in `fr3-ros2`, ROS domain 77, owns the Cartesian pose publisher. The `zed_input_relay.py` forwards the single stamped JSON `/fr3_operator/teleop_sample` from Humble over loopback UDP 47813. Packet version 2 pairs position, full orientation and optional gesture by acquisition timestamp. Invalid, stale or replayed packets cannot renew tracking or trigger the gripper. Legacy roll mode retains its PoseStamped subscription and version-1 protocol.

Services: `/teleop/zed_start`, `/teleop/zed_engage` (ready-only compatibility), `/teleop/stop`, `/teleop/home`. `/teleop/zed_status` includes phase, note, stability, tracking, gesture and last requested gripper state; it does not report measured gripper opening. Buttons use guarded `/zed-control` through the existing local/authenticated HTTPS dashboard path. Remote allow-list and origin checks are retained.

Canonical dashboard/server sources are in this directory; the main page is also stored at `/home/murilo/franka_ws/hub-curobo/esdf/lab_dashboard.html`, runtime `isaac-build:/tmp/nvblox_real`. Existing welded mesh-normal fixes are preserved. Camera 3 shows `/fr3_operator/wrist_overlay`.

`python3 deploy.py --refresh` updates the existing teleop and 8088/8089 dashboards, refuses engaged/stale state and preserves process arguments. Runtime is persisted in `/lab/fr3_zed_runtime`; the original joystick/dashboard Python files are unchanged. Installation starts disengaged in joystick mode; the ZED Start button selects ZED automatically. `--restore` restores saved original commands. Perception is started/stopped with the parent directory's `start_zed3_wrist.sh` / `stop_zed3_wrist.sh`.

Provenance: Spot repository `origin/dev` eb03146; the control backend is ported from Murilo's `/home/murilo/franka_ws/lab/fr3_joy_teleop.py`. Preexisting uncommitted perception and dashboard changes are preserved.

## Validation

`test_fr3_input.py`: 35 isolated-domain tests (191, UDP 47814), fake robot poses and mocked command publication. Includes full-6D axes, rate limits, zero speed, source arbitration, freshness/replays, competing publishers, stable Start/cancellation/expiry, bounded pause/re-anchor, Home cancellation and gesture debouncing/gating. Home subprocesses and gripper calls are mocked.

`test_dashboard_zed.py`: six fake-backend HTTP tests for Start/Stop/Home, invalid actions, origin/header guards and refusals. `test_hand_frame.py`: twelve geometry/filter tests. `test_arm_scale.py`: four scale tests. `test_wrist_geometry.py`: four synthetic tests exercising the actual orientation callback, known rotations, missing geometry/depth and one wrist-depth sample instead of five MCP samples. Dashboard JavaScript syntax is checked with Node.

Tests and installation do not actuate the robot. Earlier operator-controlled movement occurred before these updates. The revised physical orientation and gesture behavior still require an operator trial using the dashboard.

Persistent startup: install_startup_hooks.py installs backed-up hooks in fr3_ros2.sh, start_remote_dashboard.sh and restart_teleop_remote.sh. They select the runtime in the bind-mounted /lab/fr3_zed_runtime when .enabled exists, and recognize both teleop process names. Original Python files remain intact. remote_pad.py links to the original dependency. FR3_ZED_RUNTIME=0 or removal of .enabled selects original programs on future starts. deploy.py --restore also removes this flag. No hook activates teleop.

## Robustness update

See [HAND_ROBUSTNESS.md](HAND_ROBUSTNESS.md) for the raw-orientation gate correction, frame-rate-independent smoothing, hand close-up, precise wrist association, shoulder-only body frame, supported foreground depth, grasp/opening orientation clutch and validation. These opt-in FR3 settings preserve default Spot capture behavior. The revised suite has 72 passing tests; a physical trial is pending camera/robot availability.

## FR3 forward direction

The operator forward/backward translation axis is reversed relative to Spot: default `zed_forward_sign: -1.0` maps positive operator body x displacement to negative FR3 base x displacement, as requested for approaching the table. Pulling the hand back moves the other way. Left/right, up/down, relative orientation, joystick mapping and safety/speed limits are unchanged. This startup-only parameter accepts -1 or +1 and cannot be flipped during a running session. Neutral anchoring still prevents a jump at Start. The direction has synthetic regression coverage; the physical table direction needs confirmation when the robot is available.


## Laboratory hybrid tracking (22 October 2026)

`start_zed3_wrist.sh` now defaults to the direct tracker with mandatory per-person arm calibration (`TRACKER=legacy` retains the prior detector). Each Start sends a loopback calibration request on UDP 47815 and clears the old hand sample. At least 20 distinct valid RGB-D arm measurements over 2.5 seconds are required, with a dominant cluster within 6 cm containing at least 65% of the window and IQR below 3.5 cm. Length is shoulder–elbow plus elbow–wrist; segment limits are 0.15–0.50 m, total 0.45–0.95 m. Scale is FR3 reach 0.855 m divided by the measured median, with no fixed fallback in the real pipeline. Body points continue to check arm length; the applied scale stays latched across normal Start and tracking losses to avoid command jumps. Only **Nova pessoa / recalibrar** explicitly clears it; this stops the controller before requesting reset and a new Start. A shoulder position gate and current/last hand-to-arm association reject switching to another nearby person; after a long loss an ambiguous hand requires showing the arm again.

The full Pose skeleton and 21 hand landmarks are displayed. Live RGB preview runs independently of the inference worker; only the newest synchronized RGB-D pair is processed. Recently measured points may be displayed over newer RGB for up to 0.3 s (hand) / 0.8 s (body); this visual reuse never supplies control or depth measurements. Motion packets still use paired measurements and the 0.35 s freshness gate. An occluded shoulder does not invalidate an already tracked hand.

Lab runtime sets `zed_calibration_port: 47815` and `zed_recovery_timeout: 5.0` in `/lab/fr3_joy_teleop.yaml`, with a pre-hybrid backup. Default recovery remains 1.5 s for legacy users. Five seconds describes only the re-acquisition window: stale samples immediately stop following; longer or displaced recovery requires stability and measured-TCP re-anchoring. A fresh, nearby return after a gap below 0.8 seconds preserves the previous anchor; stale frames never drive motion. Stop remains explicit and prevents automatic recovery.

The physical backend in use was concurrently changed to `fr3_input_teleop_nobox.py`. The hybrid changes were integrated using a three-way merge, preserving that variant's `use_workspace_box:=false` choice and all live process arguments. No driver, controller gains or robot hardware limits were changed by this update. This remains the direct Cartesian joystick controller, without cuRobo/PF obstacle avoidance. Only operator Start enables motion after calibration; no autonomous real-robot motion trial was performed.

Validation: 46 isolated-domain input/dashboard tests and 21 tracking/calibration math tests passed. The main dashboard's camera 3 uses one outstanding JPEG request at a time, with no request queue; reload the page to load the updated JavaScript. Initial measurements before the asynchronous preview: raw RGB approximately 18 Hz / 44 ms median capture age, old detector overlay about 3 Hz; new live snapshot response around 0.11 s after reliable image publication. Final performance depends on current load and must be measured separately from inference rate.


Final preview update: HTTP camera 3 reads a latest-JPEG cache on port 8786 (loopback and the lab Tailscale address). This avoids the web_video_server/ROS image wait; snapshots were served in about 1–2 ms while capture age was around 100–200 ms. The latter is the relevant image delay, not the HTTP response time. HTTPS retains the existing proxy fallback. RGB preview is independent of the latest-only inference worker; ROS overlay is retained at a reduced rate for compatibility. Reload the 8788 page after updating.

Visual style matches the original detector: MediaPipe default Pose landmarks/connections, default hand/finger landmarks, green wrist disk, and measured body/palm axes. Occluded or old landmarks are omitted. The measured optical-frame hand pose is published as `/fr3_operator/hand_camera_pose` and TF `fr3_operator_right_wrist`, only with a valid calibrated sample; camera-to-base extrinsics are not invented. It is a measurement TF, not a robot command. Control remains paired relative 6D with the existing FR3 forward sign.

The lab configuration requests ZED by default, disengaged; the live input can also be joystick if changed by another session. Start can also hand over atomically from an engaged joystick: it disengages, selects ZED, requests a fresh hand reference while retaining accepted person scale, and waits for fresh stable samples. It does not preserve joystick motion during calibration. The deployment guard accepts fresh explicit engage/disengage log events as well as periodic state lines, preventing a false stale-state refusal immediately after Stop.

## Comparison and latest quick fixes

See [FR3_HYBRID_COMPARISON.md](FR3_HYBRID_COMPARISON.md) for the pinned Spot master/dev and G1 main comparison, choices already installed, and adaptations still pending evaluation. Start preserves person-calibration progress and short freshness gaps no longer reset stability immediately. Finger extension geometry supplements the gesture classifier. The new-person button explicitly resets the scale. Full G1 HandFit and physical cuRobo collision avoidance are not enabled by these changes.

## Dashboard and Home follow-up (22 October)

The main column now contains the 3D view and cameras together, with ZED controls first in the sidebar. Camera placement no longer waits for the full sidebar height. Backend ZED status expires after two seconds without updates; Start/Home are disabled when the backend or controller is unavailable, with the reason shown separately from hand detection.

The dashboard uses its own `fr3_home_safe.py`, preserving the shared `/lab/fr3_home.py`. Default peak translation/rotation are reduced to 0.03 m/s and 6 deg/s. Position lag above 3 cm OR angular lag above 10 degrees for 0.6 s aborts and holds the measured pose; success requires final position/orientation within 1.5 cm/5 degrees. Exit errors persist in the dashboard and details go to `/runs/fr3-zed-home.log`. This variant has not been physically validated; it does not guarantee avoidance of the hardware's joint velocity fault.

The real driver recorded `joint_velocity_violation` during Home (170-degree return), during a later automatic Home (138 degrees), and again during controller bring-up. At installation the Cartesian controller was inactive. The automatic recovery disable file already existed; it was preserved. Only the missing teleop backend was restored disengaged; no hardware recovery, gain change, Home or Start was issued by this update.

Hand re-detection can try one local ROI around a recently measured wrist when full-frame detection fails. A separate IMAGE-mode model avoids contaminating the full-frame VIDEO tracker state. Crop landmarks are remapped to the same RGB/depth coordinates; current depth and operator association remain mandatory. Once associated to the calibrated right wrist, a transient Left/Right label flip alone no longer discards the hand. A valid position keeps operator association even when orientation is temporarily rejected. These changes do not estimate an occluded hand behind a table.

52 isolated backend/dashboard tests passed, including persistent Home error reporting; 26 previous math/calibration tests remain applicable. Live perception started and produced valid 6D samples; no real motion test was performed.
