# zed3 handoff (2026-10-16, ~18:50 local). Read zed3/README.md too.

## Task / why
Stage (OFF by default) an FR3 version of the Spot branch's "wrist" perception on the third ZED (zed3). Real robot: perception only, never
send motion/controller/gripper commands; GPU 1 only; don't touch /nvblox_node (pid 1442667, started 18:12), zed1/zed2, sam2, tf_projection;
nothing under /home/murilo modified (back up as .bak-hipolito-1016 + setfacl u:murilo if ever needed).

## Decided
- zed3 = ZED 2i serial 34046618, wrapper /zed3/zed_node (zed-build:/tmp/start_zed3.sh, publish_tf:=false, 640x360, ~21 Hz). FIXED on a tripod at
  the OPERATOR station (sees person with VR headset), NOT on the FR3 hand. "wrist" = operator's right wrist (MediaPipe Pose 16), so the
  arm_pose_estimator chain (wrist_detector + hand_orientation_estimator) is what zed3 feeds. SAM2 hand role stays zed1 (primary) + zed2
  (secondary); fr3_target_tracking.yaml deliberately unchanged.
- wrist_detector uses no TF; output remapped to /fr3_operator/wrist_pose (NOT /wrist_pose = Spot curobo_mpc_node input), TF fr3_link0->wrist_target.
- zed3 NOT added to the running nvblox (it sees room+person, would pollute ESDF; also no extrinsic).

## Done (all in fr3/, repo spot-teleop)
Added: zed3/{fr3_zed3_wrist.launch.py, fr3_operator_wrist.yaml (scale 855/650=1.315, shoulder_offset [0,0,0.333] TUNE, output_frame fr3_link0),
zed3_extrinsic_PLACEHOLDER.json (identity, calibrated:false), README.md, zed3_grab_1016.png, zed3_wrist_overlay_1016.png}, start_zed3_wrist.sh, stop_zed3_wrist.sh.
Modified (+ .bak-hipolito-1016 beside them): run_nvblox_real_masked.sh (cameras:=${CAMERAS:-zed1,zed2}), start_nvblox_masked.sh (passes CAMERAS).
fr3_nvblox.launch.py / fr3_arm_mask_node.py already accept zed3 (no change).
Runs in container fr3-curobo, domain 8. Private pip dirs (outside repo): spot-teleop/tmp/mp_site_hipolito (mediapipe 0.10.14),
spot-teleop/tmp/cv_site_hipolito (opencv-contrib 4.8.1; container cv2 4.5.4 lacks cv2.aruco.DetectorParameters -> wrist_detector crashed).

## Measured
rgb 20.6 Hz, depth 21.7 Hz; /fr3_operator/wrist_pose 28.5 Hz; "Body tracking ACTIVE (Level: TORSO)"; overlay right wrist px (66,331) vis 0.88.
zed3 log had a recovered CAMERA REBOOTING at 20:21 UTC (USB flakiness; also dropped 10-15). After test: my nodes stopped by PID, stack intact.

## Open
- shoulder_offset and operator-forward -> +x mapping need bench tuning (no rotation param in wrist_detector).
- zed3 extrinsic to fr3_link0 does not exist; zed3 can't see the robot so EasyHeC can't do it. Plan: AprilTag (36h11) shared with zed1/zed2's
  fr3_link0 frame (../sim/zed{1,2}_extrinsic_1016.json) -> T_base_zed3; write {"T_base_cam","calibrated":true}; `CALIB=<f> TF=1 bash start_zed3_wrist.sh`.
- zed3/__pycache__ is root-owned (leftover from container); `sudo rm -rf` it.
- Nothing committed to git.

## Commands (cd fr3/)
DRY=1 bash start_zed3_wrist.sh | bash start_zed3_wrist.sh | HAND_ORIENTATION=1 ... | bash stop_zed3_wrist.sh (by PID)
