# Swept full-body replay of real drill plans — 2026-11-21 ~06:40Z

Offline only. Nothing was published to ROS, written to the map, or sent to the robot or broker.
Inputs: one read-only `GET http://127.0.0.1:8788/mesh.bin` (/nvblox_curobo, 06:35:12Z, 12,338 vertices,
`mesh_snapshot_V.npy`), VM plan/cloud copies under `ref/`, and live held models from
`/home/murilo/franka_ws/lab/fr3_zed_runtime/attached_spheres/`.

## Tools
- `fr3_spheres.py`: numpy FK from the same URDF and `fr3_mesh.yml` the live MPC resolves
  (hub-curobo `node_config/fr3.yml`, sphere buffer 0.004, fingers open 0.04). It has 59 spheres for
  link1 to the fingers, plus held spheres in fr3_hand. The fr3_hand pose matches the planner's DH `fk`
  to 1e-15 m (26 configurations tested).
- `replay_plan.py`: samples each stage joint-linearly at ≤0.01 rad, the way the direct-JTC executor
  moves. It uses the full arm, hand and fingers, adds held spheres on S3/S4, and reports clearance
  to obstacles, table and target plus the TCP's deviation from the planned straight line.
  The output always carries `known_space_verified=False`, because a mesh has no unknown-space information.
- Tests: `python3 -m unittest test_swept_replay` (6 OK).

## Findings
1. **The held drill model sits below the table at the grasp.** This is independent of the mesh:
   it uses only the plan, the held model and table_z.
   - 050322 (the run carried at ~05:17Z): lowest sphere point is 14.8 mm below the table.
     The held model is therefore within the 0.05 m activation band until lift ≥ 6.5 cm.
   - 053830 (latest loop regen): 34.7 mm below the table (46 mm below the observed drill bottom).
     It needs lift ≥ 8.5 cm, against a planned lift of 10 cm.
   - Horizontally, the model extends up to 45 mm (050322) or 60 mm (053830) beyond the observed drill
     outline. In 050322 it also leaves up to 32 mm of the observed cloud outside the model in one
     direction (a coverage gap, not only over-size).

   This is a measured contributor to "avoidance too early while carrying", not a proven sole cause.
   The fix belongs in held-model generation (a world-z floor at table_z + margin, tighter lateral
   radii with coverage kept), not in lowering the activation distance. Validate that with replay
   before deploying anything.
2. **The planner checks miss the hand, fingers and payload.** The planner reports S2–S5 obstacle
   clearance of 0.113 m. Swept full-body spheres reach 0.039 m: fr3_hand against vertices beside
   the drill.
3. **S1/S6 clearance is limited by fr3_link1, which the planner skips.** It is 78 mm from a mesh
   vertex at (0.142, 0.01, 0.21) even at Home. That is probably robot or base self-geometry in the
   map, and the MPC also sees it.
4. **Single-waypoint stages (CART_STEP 0.15) are joint-linear in JTC.** The TCP deviates up to
   4.6 mm from the intended straight approach line in S2/S5. That is negligible for the drill, but
   it uses almost all of the ball's 5.1 mm per-side capacity. A ball path must use dense
   Cartesian waypoints.
5. **The grasp pose itself is fine.** At the grasp, the hand body spheres overlap the drill cloud by
   2.4 mm, which is inside the 4 mm buffer.

## Caveats
- The mesh is from 06:35Z and the plan from 05:38Z. Findings 2 and 3 depend on the scene and
  will change if objects moved. Finding 1 does not depend on the scene.
- Mesh-vertex distance overestimates surface distance by up to about half the voxel size. It is
  not ESDF, and unknown space is not evaluated, so `swept_esdf.validate` must keep failing.
- No live configuration, margin, held model or map was changed.
