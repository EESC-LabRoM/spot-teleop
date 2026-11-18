# Operator-reference handoff, Nov18

User asked to use shoulder/elbow/wrist references from spot-teleop and open-vocab-shared-autonomy, validate from bags, fix problems and commit. User explicitly forbade running on the real robot. No robot claim or motion/controller/TF/live camera/tracker changes were made.

Selected source: runtime/operator_hand_tracker_debug.py --pose-source rgbd_body --operator-profile operator_profile.json. Saved arm lengths come from take04 only; fresh current shoulders establish axes and origin. The profile must be selected explicitly for the same operator. Consumer input divider is1.0; comfort and vertical gains remain separate. See profile.json. This is operator calibration, not robot-camera extrinsic calibration.

Final source matches safe_run_manifest.json exactly. Twenty-five pure checks pass (10 arm,14 reference,1 exact-consumer gain), plus the post-loss identity guard check. All8 raw RGB-D bags were re-inferred at1x with recorded delivery delays and source gaps, CPU-only network-none Docker. No prerecorded robot trajectory was supplied.

Results: estimated arm length.50005565m, scale1.70980968; fresh reference ready1.033s; scale variation0 and axes variation at numerical precision. Final6031 position-valid frames of6388 processed; paired held-out retention97.73% vs baseline. Median/p95 processing14.24/35.17ms. More than350ms stale output remains rejected by the existing protocol; capture-to-diagnostic p95 is250.5ms, so do not call this a physical latency measurement.

Rejected trials documented: cold startup37.57s; saved profile with unconstrained torso-depth pitch introduced extra coupling (up/down forward excursion28.8cm vs15.5cm baseline). Final camera-facing heading/shoulder-roll axes plus robust shared torso translation bring it back to15.7cm. No current-pose estimate can bypass the post-loss operator identity gate;80frames in take05 correctly wait for fresh arm confirmation after a long gap.

README.md explains the repositories/method, RESULTS.md has metrics and limits, validation.json has explicit checks. run_comparison.sh is a reproducible perception-only launcher using the final runtime and saved profile, never a robot launcher. It requires the existing local image, bags, MediaPipe/OpenCV and hand-landmarker model. Large recordings/traces/logs/frozen rejected trials stay in /home/hipolito/franka_teleop_checks/wrist_body_calibration_1118. No push/source upload performed.

All own replay containers exited. Camera lane c410a3cdbb184c0abf0426acef3e722a owns live camera recovery; ZED3 and wrist RealSense physical USB faults do not block bags. Do not restart their services from this handoff. Robot-view extrinsics, table contact height, loaded lifting and automatic pickup remain separate unvalidated tasks; this work does not clear them.
