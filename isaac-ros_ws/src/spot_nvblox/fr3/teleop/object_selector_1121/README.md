# Three-object selector integration status

Scope: the dashboard target buttons must choose exactly one of the drill, tennis
ball, or Sprite bottle. A fresh mask for that target is the only depth that may
be carved. All non-target objects must remain collision obstacles. Target
selection is not the same as grasp readiness.

## Reported live status (read-only snapshot, 2026-11-21 07:27Z)

A GET to `http://100.84.207.115:8899/object-target` reported drill selected
(generation 6), ready/Y enabled, fresh TRACKING (0.4 s), fresh mask/map status,
0.43% mask coverage and 1,779 cloud points. The service reports the selected
target fully carved and non-target objects retained. The response explicitly
keeps tennis ball and Sprite Y disabled: ball camera agreement is outside the
gripper budget; transparent bottle depth is not reliable. The served page
contains all three target labels. This confirms the API/UI status snapshot,
not an independent measurement of the live ESDF or robot state. The live
lane also reports a drill→ball→Sprite→drill selection cycle, but this repo
bundle itself remains an offline candidate. The same server's `/health` route
list omitted `/object-target` while its direct GET succeeded; that discrepancy
was reported for review.

## Available offline artifacts

- `map_filter.py` and `test_map_filter.py`: transport-free routing rules.
  Explicit fresh no-selection passes full depth. A selected target is carved
  only with a fresh, source-stamp-matched mask and tracker state carrying the
  same target/generation/selection ID. Lost, stale, absent, wrong-generation,
  oversized, empty, or unmatched evidence pauses integration. Nine tests pass.
- The complete selector state machine, HTTP contract, dashboard preview, swept
  replay tools, and ball/bottle geometry analyses are included under this
  directory. They are offline candidates and are not wired to live ROS or the
  dashboard server.
- Current evidence says the drill is the only planner-validated target. Ball
  grasp fails the measured cross-camera and table-height budgets. Sprite's
  clear-plastic depth is unusable; a silhouette-derived keep-out cuboid is an
  offline obstacle candidate, while no grasp is supported.

## Live interfaces inspected

- `sam2_tracker_node.py` subscribes to JSON on `/perception/seed_command`.
  The seed requires `bbox_1000` and optional `grasps_1000`; it consumes a
  current/snapshotted frame and publishes `/tracking/seed_3d`. It publishes
  only `TRACKING` or `LOST` on `/tracking_state`; that topic does not say
  whether there is no selected target. The node has a separate secondary-camera
  `seed_pixel` path. A target switch must not reuse a warmed SAM2 memory or a
  fixed historical drill box.
- The deployed filter in
  `wrist_drill_1117/map_filter/drill_depth_filter.py` is drill-only. It pairs
  `/franka_seg_zed1/depth` with PNG `/wrist_map/drill_mask/compressed`, carves
  the mask, and publishes `/wrist_map/drill_free_depth`. It has no selected
  object identity or explicit no-target/full-depth route.
- The current nvblox node consumes that filtered depth in the single
  `/nvblox_curobo` map. A complete switch needs a generation/selection
  heartbeat, cold target reseed with fresh prompts per camera, identity-tagged
  mask bridge, pause while pending/lost, and a map clear/rebuild using the new
  target mask before reporting the map ready.
- This bundle's dashboard preview calls `/object-target` through its staged
  proxy at `127.0.0.1:8898`. A separate live coordinator is now reported at
  VM `100.84.207.115:8899/object-target` (read-only snapshot documented above).
  Its internal source was not reconciled to this offline bundle. The historical
  VM `/regrasp` endpoint used a fixed drill box and must not be reused.

## Readiness and rollout gates

1. Reconcile and review the separate live coordinator against this bundle's
   identity contract, including target-specific prompt generation and identity
   binding for tracker, mask, map, grasp, and held model.
2. Keep Y disabled unless fresh mask + rebuilt map + non-target obstacle
   checks + object-specific grasp and held model + current broker preflight all
   pass. Current live status reports only drill Y validated; do not reuse a
   drill binding for another target.
3. For a selected target, preserve the target cloud/model for grasp collision
   planning while carving it from the nvblox background map. Do not equate
   “carved from ESDF” with “handled by cuRobo.”
4. Require swept, full-body and payload clearance samples through every stage.
   The current Y planner checks are waypoint-based, omit hand/payload world
   collision in intermediate stages, and treat unknown ESDF as free.
5. The live lane reports a full drill→ball→Sprite→drill selection cycle and
   per-switch map rebuild; independently recheck map identity/content and
   dashboard health before operator testing. The staged `.04 m` MPC activation
   candidate and held-sphere refits have not been replayed or deployed.

## Latest offline swept evidence (2026-11-21 ~06:40Z)

The swept replay is in `swept_replay/` and is included with its plan/cloud
fixtures. Its FK model matches planner FK to 1e-15 m across 26 configurations.
For the recorded 053830 drill plan, `test_bottle_clearance.py` reconstructs the
full robot/payload spheres and computes exact sphere-to-AABB separation: minimum
227 mm across the six stages. This static replay cannot bound common-mode TF
error or prove unobserved space is clear.

The attached drill spheres also explain some early avoidance: the 050322 model
extends 14.8 mm below the table at grasp, and 053830 extends 34.7 mm below.
Their side overhang reaches 45/60 mm beyond the observed cloud. A first-pass
MILP fit produced no acceptable replacement: its only nominal cloud-coverage
pass missed one third of the support column and increased side overhang. See
`swept_replay/HELD_MODEL_FIT.md`. This is a measured contributor, not proven
sole cause. Keep the 5 cm cuRobo activation distance unchanged until the held
model passes table/contact and full-body replay gates. The replay uses mesh
vertices and cannot certify unknown space; its clearance result does not make
a live map or pick ready.

The offline bundle is now copied into this project directory with the 9 map
filter tests. No live stack changes or robot commands are made by these files.


## Object readiness evidence (2026-11-21)

- **Drill:** only planner-validated target in the selector contract. The live
  API snapshot at 07:27Z reported drill ready/Y enabled with fresh mask/map
  status; this lane did not independently verify live robot state. Historical
  Y operation succeeded. The two latest recorded drill picks were
  20261121_050322 and 20261121_053830; these supply offline path and
  held-payload replay fixtures only.
- **Tennis ball:** remains `planner_validated=false`. Near-synchronous camera
  fits differ by 16.6 mm in XY; measured clearance budget yields 0/800 grasp
  candidates. Fixed-radius fitting and per-camera range scale did not explain
  the mismatch. Need independent surveyed reference and a ball-specific grasp
  path before enabling it. See `backend/BALL_SPRITE_VALIDATION.md`.
- **Sprite bottle:** live status now enables Y as a supervised-trial target.
  The selector lane reports one successful side-grasp/pick/place cycle on run
  `20261121_075900_sprite_selector`, at the operator-moved bottle pose; the
  earlier cap attempt missed. Treat this as a supervised demonstration, not
  general pose-independent validation: live status still reports ~10mm camera
  error against 7.5mm per-side slack, and the lane reports 60N compressed PET
  to 24mm. Cap estimates on prior clouds differed 17.2mm XY / 12.0mm Z; body
  diameter estimate (71–91mm) overlaps/exceeds the 79.9mm opening. Transparent-
  plastic depth is smeared over 18–21 cm. Offline silhouette keep-out is center
  (0.7285,-0.2345,0.1577) m, dimensions (0.120,0.204,0.261) m in `fr3_link0`;
  it covers both masks but cannot bound common-mode camera error. A recorded
  drill route has at least 227 mm exact sphere-to-box clearance from this
  candidate; static replay is not live integration or proof of safe free space.
  See `backend/BOTTLE_VALIDATION.md`.

The isolated candidate map filter accepts full depth only with a fresh
explicit no-selection heartbeat. During selection it requires a same-generation
mask and tracker status; missing, stale, lost, or mismatched evidence pauses
integration. These modules remain offline in this repo. A separate live
coordinator is reported to handle reseed/filter/map switching, but its code and
identity/fail-closed behavior have not been reconciled here. Do not enable ball
or Sprite Y until per-target map, full-path/payload clearance and fresh broker
checks are independently validated.

Offline verification from this project tree: backend tests 74 passed; map
filter 9; selector proxy/API 5; dashboard UI 3; swept clearance contract 7;
full-body swept replay 6 and Sprite route clearance 2. Total 106 checks passed. Commands:
`cd backend && python3 -m unittest -q test_bottle_keepout test_ball_grasp test_selector_backend test_selector_http`;
`python3 -m unittest -q test_map_filter`;
`cd candidate && python3 -m unittest -q test_object_proxy test_end_to_end && node test_selector_ui.js`;
`cd collision_validation && python3 -m unittest -q`;
`cd swept_replay && python3 -m unittest -q test_swept_replay test_bottle_clearance`.
These exercise offline contracts and fixtures only.
