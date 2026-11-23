# FR3 three-object grasp-target selector (live, 2026-11-21)

Dashboard selection of the drill, tennis ball or Sprite bottle. The selected object gets a live
SAM2 mask and is carved from the nvblox collision map; the other objects stay obstacles.
Y grasps the selected object only when its grasp path is validated.

## Deployed
| Piece | Where it runs | Source here |
|---|---|---|
| Selector service (`GET/POST /object-target`, 10.0.0.3:8898) | VM container `vm-object-selector` (image `vm-grasp-loop:latest`, domain 8, `--network host --ipc host` (FastDDS SHM: without host IPC it sees topics but no data), env `SELECTOR_BIND=10.0.0.3 SELECTOR_START_REGRASP=0`), code `fr3_real_repro/object_selector/` | `selector_service.py`, `esdf_points.py`, `fr3_spheres.py`, `robot/` |
| Object grasp planners (broker path, unused since the 11-23 quick Y) | VM `vm-viewer` via the broker VM script | `object_grasps.py` |
| Dashboard card + proxy (2026-11-23: tower :8788/:8795; the VM :8899 server is stopped) | `isaac-build:/tmp/nvblox_real/{mesh_server.py,lab_dashboard.html}` (backups `*.bak_selector_20261123`) | `deploy/mesh_server.py`, `deploy/lab_dashboard_8788.html`, `selector_card.html`, `object_selector.js` |
| Grasp broker gate | tower `fr3_zed_runtime/grasp_broker.py` | `deploy/grasp_broker.py` |

## Switch sequence (only while disengaged, idle, holding nothing)
1. Lock Y: park `grasp_binding.json`, stop `vm-regrasp-live`.
2. Publish the new id on `/wrist_map/target_selection`. The depth filter still expects the old id,
   so map integration pauses instead of carving an unverified mask.
3. Restart `vm-tfproj`. It keeps the first seed as an immutable reference and would re-seed the
   old object.
4. SAM2 seed from the click. Require TRACKING, a fresh mask of plausible size that contains the
   click, and a plausible object cloud.
5. Restart the depth filter (`mask_object_id=<target>`), `/nvblox_curobo` (clears stale voxels) and
   `mesh_server` (it caches mesh blocks across nvblox restarts).
6. Verify the carve in the authoritative ESDF (`get_esdf_and_gradient`) at the ZED1 target points.
7. Grasp: the drill restores its binding and regrasp loop. Other targets get Y only if listed in
   `Y_VALIDATED`; the broker then plans from the live cloud via `object_grasps.PLANNERS`.

## Y status
- Drill: binding path, unchanged.
- Sprite: horizontal or pitched side grasp on the opaque label band. The bottle axis comes from a
  fixed-radius circle fit on the front surface only (see-through points behind the clear PET are
  dropped). Approach azimuth and pitch are searched, and the flattest collision-free candidate is
  taken. Live supervised cycles succeeded on 11-21 (pick, lift, place, Home). It is a supervised
  trial: one 11-21 attempt after the bottle was moved missed. The fixed 60 N grasp force squeezes
  the PET to ~24 mm.
- Tennis ball: Y locked. See `../object_selector_1121/ball_calib/RESULTS.md` and
  `../object_selector_1121/ball_truth/RESULTS.md`.

## Operational notes
- After any `grasp_broker.py` restart, run `setfacl -m u:1004:rw grasp.sock`. The teleop container
  user is uid 1004.
- Never poll `ros2 topic echo` inside `fr3-ros2` during Y runs. It starved the teleop joint
  callback (0.3 s watchdog) and aborted picks with "robot joint feedback stale". The selector reads
  teleop state only on demand.
- A stale `held=true` after a failed close blocks Home and switching. Recover by retreating
  vertically, then press B.

Tests: `python3 -m unittest test_selector_service test_object_grasps`.
