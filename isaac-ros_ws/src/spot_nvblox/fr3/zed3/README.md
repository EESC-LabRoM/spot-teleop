# zed3 on the FR3 bench (staged, OFF by default)

**What zed3 is (checked 2026-10-16):** ZED 2i serial 34046618 (wrapper `/zed3/zed_node`, started by `zed-build:/tmp/start_zed3.sh`,
`publish_tf:=false`, 640x360 rectified, ~21 Hz rgb+depth). It is **fixed on a tripod at the operator station and looks at the operator**
(VR headset), not at the robot and not on the FR3 hand. See `zed3_grab_1016.png`. So "pegar o wrist" = track the *operator's* right wrist
(MediaPipe Pose) for teleop - the Spot branch's `arm_pose_estimator` chain - and NOT the SAM2 hand-camera role (SAM2 tracks the object on
the table: it stays on zed1 primary + zed2 secondary, `../fr3_target_tracking.yaml` is unchanged).

Files: `fr3_zed3_wrist.launch.py`, `fr3_operator_wrist.yaml` (FR3 params), `zed3_extrinsic_PLACEHOLDER.json` (NOT a calibration),
`../start_zed3_wrist.sh`, `../stop_zed3_wrist.sh`.

## Start / stop (perception only; nothing commands the robot)
    DRY=1 bash fr3/start_zed3_wrist.sh              # checks, starts nothing
    bash fr3/start_zed3_wrist.sh                    # wrist_detector -> /fr3_operator/wrist_pose (PoseStamped, fr3_link0) + TF fr3_link0->wrist_target
    HAND_ORIENTATION=1 bash fr3/start_zed3_wrist.sh # + hand roll (/hand_roll_quat)
    bash fr3/stop_zed3_wrist.sh                     # by PID
Runs in `fr3-curobo`, domain 8. mediapipe 0.10.14 and opencv-contrib 4.8 (the container cv2 4.5.4 lacks `cv2.aruco.DetectorParameters`)
live in private dirs `spot-teleop/tmp/{mp,cv}_site_hipolito` (put on PYTHONPATH by the script only; system site-packages untouched).
Output topic is remapped away from `/wrist_pose` (the Spot `curobo_mpc_node` input) so nothing built for the Spot can consume it.

## What is Spot-specific in the chain (and what changed for the FR3)
| item | Spot | FR3 (fr3_operator_wrist.yaml) |
|---|---|---|
| topics | `/zed/zed_node/...` | `/zed3/zed_node/{rgb/image_rect_color,depth/depth_registered,depth/camera_info}` |
| output frame | `body` | `fr3_link0` |
| scale human->robot | 984/650 | 855/650 = 1.315 (FR3 reach 855 mm) |
| shoulder offset | (0.292,0,0.188) = body->arm_sh0 | (0,0,0.333) = joint-2 height, **TUNE** |
| TF needed | none (wrist_detector uses only camera data) | none |
Operator-forward is mapped to +x of `fr3_link0` (no rotation parameter exists); if the operator faces a different way, rotate downstream.
Spot-only, not ported: sam2 `roll_reference_frame:=body` / `hand_deroll`, tf_projection `hand_camera_frame:=hand_color_image_sensor`
(a camera on the Spot arm; here zed1 plays that role, already configured), `curobo_mpc_node` (Spot arm, consumes `/wrist_pose`).

## Using zed3 in nvblox (NOT enabled; the running /nvblox_node uses zed1,zed2)
`fr3_nvblox.launch.py` already accepts `cameras:=zed1,zed2,zed3` and `fr3_arm_mask_node.py --cameras zed1,zed2,zed3`;
`start_nvblox_masked.sh` takes `CAMERAS=zed1,zed2,zed3`. Don't, unless wanted: zed3 sees the room and the person, not the bench, so it would
put the operator into the ESDF the MPC plans against. It also needs a real TF `fr3_link0 -> zed3_camera_link` (not available, see below).

## Missing before zed3 can have a TF to the robot (only nvblox needs it; the wrist pipeline does not)
No extrinsic exists. zed3 does not see the robot, so EasyHeC (mask+depth fit of the arm) cannot calibrate it directly. Options:
1. Put an AprilTag (36h11) on the table that zed1/zed2 already see in `fr3_link0` (their EasyHeC fits are in `../sim/zed{1,2}_extrinsic_1016.json`)
   and make zed3 see the same tag (move the tripod/tag or use a second tag in rigid relation) -> T_base_zed3 = T_base_tag * inv(T_zed3_tag).
2. Or a tag-on-hand test: hold the board/tag visible to zed3 at known robot-link poses (`/lab/calib` runner, user + e-stop) and solve.
Write the result as `{"T_base_cam": 4x4 (optical frame in fr3_link0), "calibrated": true}` in a new file, run
`CALIB=<file> TF=1 bash fr3/start_zed3_wrist.sh` (reuses Murilo's `real_extrinsics_tf.py --cam zed3`; refused while `calibrated` is false).
If zed3 is ever moved onto the FR3 hand instead: eye-in-hand, parent `fr3_link8` (flange) / `fr3_hand`, solve hand-eye with the easyhec-build
env on recorded (flange pose, image) pairs, and publish a static `fr3_hand -> zed3_camera_link`; the sam2 hand role would then use zed3.

## Operational notes
- 2026-10-15: zed3 USB dropped once; today the log shows a recovered `CAMERA REBOOTING` episode - check `docker exec zed-build tail /tmp/zed3.log`.
- Smoke test 2026-10-16: wrist_pose 28.5 Hz, `Body tracking ACTIVE (Level: TORSO)`, overlay in `zed3_wrist_overlay_1016.png`.
