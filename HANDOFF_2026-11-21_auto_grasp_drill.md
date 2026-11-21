# BALL GROUND TRUTH + COMMIT — 2026-11-21 ~08:40Z (Claude lane cc29b70d)

- Ball centred by gripper at A (0.420,-0.270); carried+placed at B (0.38,-0.15): first
  real ball grasp (60.3 mm held). ZED2 ray-on-plane error A (0.6,-3.4) B (-3.5,-4.0) mm;
  ZED1 -14/-11 mm x. See teleop/object_selector_1121/ball_truth/RESULTS.md. Ball Y locked
  pending spot C held-out check + estimator implementation.
- Second Sprite pick after operator moved the bottle missed (closed empty); recovered
  (S5+S6, open in air). Bottle later not on table (tracker LOST). Reselect after moving.
- Code committed under isaac-ros_ws/src/spot_nvblox/fr3/teleop/object_selector/ (live)
  and object_selector_1121/ (offline tools, peer docs).

---

# SPRITE LIVE GRASP WORKS — 2026-11-21 ~08:02Z (Claude lane cc29b70d)

- Operator moved the bottle closer (axis ~0.483,-0.074) and added an AprilTag (calib ref).
  Sprite planner = horizontal side grasp at label band, approach azimuth searched
  (object_grasps.plan_sprite_side, pitch 0, clearance >=77 mm). Selector Y_VALIDATED
  now {drill, sprite} (sprite = supervised trial note on dashboard).
- Live Y cycle run 20261121_075900_sprite_selector: S1/S2/close/verify/S3 lift ok, held
  model loaded; second Y placed it back, Home, gripper open, idle, no lock. Bottle upright.
- Grasp force is fixed 60 N in joystick_grasp.py: PET squeezed to 24 mm. Lower per-object
  force not implemented (operator decision).
- "robot joint feedback stale" aborts were caused by ros2 CLI polling in fr3-ros2
  (the selector polled every 4 s); polling removed, teleop state read on demand.
- Ball still Y-locked (camera/hand-eye error). AprilTag now on table: use it for ZED1/ZED2.

---

# SELECTOR LIVE + SPRITE/BALL TESTS — 2026-11-21 ~07:55Z (Claude lane cc29b70d)

- LIVE: dashboard :8899 card "Grasp target" (3 buttons + click on ZED1). Service
  VM container vm-object-selector (127.0.0.1:8898, code fr3_real_repro/object_selector/).
  Switch = disengaged+empty only -> park binding/stop vm-regrasp-live -> map gate pause
  -> restart vm-tfproj (immutable seed ref bug) -> SAM2 seed+mask check -> restart
  drill_depth_filter (mask_object_id) + nvblox_curobo + mesh_server -> ESDF carve check.
  Cycle drill->ball->sprite->drill passed live. Y_VALIDATED={drill}.
- Broker (tower grasp_broker.py, backup .bak_selector_20261121) now reads selector
  API; non-drill targets plan via object_selector/object_grasps.py only if selector
  ready+y_enabled+tracker/map on target; writes attached_spheres. Socket needs
  `setfacl -m u:1004:rw grasp.sock` after any broker restart (teleop uid 1004).
  Drill planning-only check through new broker passed.
- Ball: wrist RealSense hand-eye NOT rigid at mm level (5 mm near-top-down, 6-15 mm
  with wrist rotation). See object_selector_1121/ball_calib/RESULTS.md. Ball Y locked.
- Sprite cap grasp live test (user go 07:39Z): S1/S2 ok, close MISSED (0.2 mm), no
  contact/topple. Recovered S5+S6 Home, gripper opened in air. Sprite Y DISARMED.
  STALE: teleop held=true + regrasp.lock HELD_OBJECT until operator presses B.
- New Sprite planner = side/pitched body grasp at label band (user request).
  Flattest collision-free from current bottle spot (x .72) is 45 deg pitch; 15/30
  deg hit table/box. Closing-axis camera error ~9.6 mm vs 7.5 mm/side slack.
  Root blocker for ball+Sprite: ZED1/ZED2 ~17 mm disagreement. Not deployed as Y.

---

# RESUME — 2026-11-21 07:36Z

- Goal: dashboard selector for drill / tennis ball / Sprite; live segmentation,
  target-only map carve, non-target obstacles, validated grasps and less early
  avoidance. This lane made no live writes or robot commands.
- Offline project bundle:
  `isaac-ros_ws/src/spot_nvblox/fr3/teleop/object_selector_1121/`. 106 offline
  checks passed; commands in `README.md`. Repo is broadly dirty; no commit.
- Live selector GET at 07:34:40Z (`100.84.207.115:8899/object-target`) reports
  Sprite selected, generation 7, ready/Y enabled; 1.35% mask, 883 cloud pts,
  TRACKING age .3s, mask age .2s, map age .1s. API reports 100% target carve,
  zero target points left, other objects retained. This is a status response,
  not independent ESDF verification. The live lane reports a full selection
  cycle and a new Sprite cap planner. No robot state was independently checked.
- IMPORTANT: do not press Y for Sprite yet. My offline RGB cap-centroid
  triangulation on the stored pair estimates (.7318,-.2423,.2434), with
  2.26/3.52px reprojection residual; the lane's new live plan reports cap_top
  (.7212,-.2399,.2504), about 11.3mm XY and 7mm Z away. That's beyond the
  ≤5mm cap-pose objective; color centroids may not be corresponding points.
  The new `object_grasps.py` planner assumes a 32mm body radius and tests seen
  so far use synthetic clouds; no real-cloud replay was found. The bottle's
  earlier stereo/depth analysis found a 23mm full-silhouette height disagreement
  and 18–21cm transparent-depth smear. I sent an urgent note asking the live
  lane to keep Sprite Y locked until pose/model/full-path evidence is reconciled.
  No live change was made by this lane. Details:
  `backend/BOTTLE_VALIDATION.md`.
- Ball remains Y-locked. Two supervised wrist-ball calibration passes found no
  rigid eye-in-hand fit; near-top-down subset RMS 5.2mm, rotated views 6–15mm
  and impossible table heights. Independent pass-2 fit (`run_071727`) had
  13.62mm RMS, 79.68mm leave-one-out, table z 75.42mm. Keep ball locked pending
  ground-truth calibration. Results:
  `/home/hipolito/franka_teleop_checks/object_selector_1121/ball_calib/RESULTS.md`.
- Drill is the only previously planner-validated target. Held-model replay found
  14.8/34.7mm table penetration and 45/60mm overhang; full-body path clearance
  .039m vs planner .113m. Keep the current model and 5cm MPC activation
  distance unchanged pending validated replay.
- Board showed `franka-robot` free; no current robot state verified in this
  lane. Do not trigger Y or move hardware here. The Claude lane has a pending
  request for a live Sprite test; no new operator response is present in its
  session history as of this handoff.

---

# RESUME — three-object readiness (2026-11-21)

- User’s goal: dashboard selection among drill, tennis ball, Sprite; live
  segmentation, selected-object map carve with other objects kept as obstacles,
  per-object grasping and later operator tests. This work is offline. Do not
  infer live robot state from old lane messages.
- Project bundle: `isaac-ros_ws/src/spot_nvblox/fr3/teleop/object_selector_1121/`
  now includes selector backend/HTTP contract, UI/proxy candidate, fail-closed
  map-filter core, object validations, collision contracts and swept replay.
  Offline checks: 74 backend + 9 map-filter + 5 proxy/API + 3 UI + 7 clearance
  contract + 6 swept replay + 2 Sprite keep-out route checks = 106 passed. No generated Python caches retained.
- Map-filter policy: fresh explicit no-selection heartbeat -> raw full depth;
  selected target -> carve only with fresh TRACKING + source-matched mask with
  matching (target,generation,selection_id); stale/lost/missing/mismatched ->
  pause integration. Existing live filter has no target identity, and tracker
  only publishes TRACKING/LOST. ROS adapters/coordinator are not implemented.
- Drill only is planner-validated. Tennis ball remains blocked: simultaneous
  camera fits disagree 16.6 mm XY and 0/800 GraspGen candidates meet the measured
  budget. Sprite grasp is unsupported because clear-PET depth is smeared 18–21cm;
  an offline mask-derived keep-out candidate exists (center .7285,-.2345,.1577,
  dims .120,.204,.261m, base frame) but has unbounded common-mode TF error and
  is not integrated. Recorded drill-route separation from this candidate is
  >=.227m in the reproducible offline sphere/AABB replay only.
- Held-drill full-body replay shows current models extend 14.8mm (run 050322)
  and 34.7mm (053830) below the table at grasp, with 45/60mm lateral overhang;
  planner waypoint checks report .113m while swept full-body reaches .039m.
  This can cause premature avoidance while carrying, but is not proven the sole
  cause. Do not lower the 5cm activation distance based on this. The offline held-model fitting attempt found no acceptable replacement; its
  only nominal pass missed one third of the bottom support column and increased
  lateral overhang. See `object_selector_1121/swept_replay/HELD_MODEL_FIT.md`.
  Keep the 5 cm activation distance and current model unchanged.
- No live dashboard, tracker, map, grasp binding, Y, or robot change was made;
  no robot state is known from this work. Live readiness requires a production
  identity bridge, target-specific tracker reseed, single-map rebuild/status,
  full-path/payload/unknown-space validation, and fresh broker preflight.
- Detailed object metrics/tests: `object_selector_1121/README.md`,
  `backend/BALL_SPRITE_VALIDATION.md`, `backend/BOTTLE_VALIDATION.md`, and
  `swept_replay/FINDINGS.md`. All buttons must display not-ready for ball and
  Sprite until evidence supports those paths.

---

# SWEPT REPLAY — 2026-11-21 ~06:40Z (Claude lane, offline only)

- New offline full-body replay /home/hipolito/franka_teleop_checks/object_selector_1121/
  swept_replay/{fr3_spheres.py,replay_plan.py,test_swept_replay.py,FINDINGS.md};
  6 tests pass. URDF/fr3_mesh.yml sphere FK == planner DH fk (1e-15 m).
  One read-only GET 127.0.0.1:8788/mesh.bin 06:35Z; no ROS/map/robot/broker use.
- Held drill model extends BELOW TABLE at grasp: 050322 by 14.8mm (in .05
  activation until lift>=6.5cm), 053830 by 34.7mm (needs >=8.5cm of 10cm lift);
  up to 45/60mm beyond observed drill laterally; 050322 also misses 32mm of
  cloud one side. Measured contributor to early carried avoidance; fix belongs
  in held-model generation (z floor, tighter lateral), NOT activation. Unvalidated.
- Full-body swept S2-S5 obstacle 0.039m (fr3_hand) vs planner 0.113m; link1
  78mm from vertex (0.142,0.01,0.21) at Home (self geometry?). Joint-linear
  single-waypoint S2 TCP deviates 4.6mm from line: ball needs dense waypoints.
- Still nothing deployed; Y remains drill-bound; known space unverified.

---

# RESUME — three-object selector and live grasp, 2026-11-21 ~06:15Z

- User requests complete drill/ball/bottle dashboard target selection, live
  segmentation, map exclusion, autonomous grasp and less premature avoidance.
  No target switch, map change, new grasp or motion performed in this phase.
- Claude child 31bb4cd940fc4e5cb52c2e9e1d3b5d07 built staged backend and ball
  checks; top-level lane request d05dc981df3d4752af7278a7e5318767 remains
  pending, explicit user approval relayed Hub41db8e5ca2cf4f49bc3ec297dca99000.
- Staged code /home/hipolito/franka_teleop_checks/object_selector_1121/:
  backend/{selector_backend.py,selector_http.py,ball_grasp.py,tests},
  candidate/{lab_dashboard.html,object_selector.js,lab_live_server.py,tests},
  collision_validation/swept_esdf.py. 62 backend +4 proxy +1 HTTP integration
  +7 swept-check +3 UI tests pass; none is wired to tracker, nvblox or Y.
  Ball/sprite planner_validated false; old live Y remains drill-bound.
- Near-synchronous (63.7ms) ZED1/ZED2 capture twocam/*.npz and SAM2 masks:
  fitted ball centres 16.61mm apart XY; assumed fixed radius33.5mm & table
  z28.7mm still 16.34mm apart, residual RMS4.51/6.52mm; script
  twocam/fixed_radius_fit.py. Gripper open79.97mm vs estimated ball diameter
  ~70mm -> ~5mm each side. Source of discrepancy unresolved; do not mark
  ball ready. Bottle stereo cloud heavily smeared; validate conservative
  geometry/keep-out before target-specific map carve.
- Baseline planner only gates obstacle clearance S1/S6, samples centreline,
  and omits full hand/payload in checks; Y direct-JTC and ESDF unknown=free.
  Staged swept checker requires actual kinematic sphere/ESDF adapters (absent).
  No real collision margin changes: staged .04m override and sphere refit
  experimental only; validate full observed/unobserved geometry and replay.
- Further: no production object identity enforcement, stale-map clear strategy,
  live mask seeding/tracking check or object-specific grasp binding. Do not
  connect snapshot masks directly to map or use drill binding for ball/bottle.
  No live selector deployed. Existing old drill Y remains enabled; warn user.
- Quick checks (local; no robot commands):
  `cd /home/hipolito/franka_teleop_checks/object_selector_1121/backend &&
  python3 -m unittest -q test_ball_grasp test_selector_backend test_selector_http`;
  `cd ../candidate && python3 -m unittest -q test_object_proxy test_end_to_end
  && node test_selector_ui.js`; `cd ../collision_validation && python3 -m
  unittest -q`; `python3 ../twocam/fixed_radius_fit.py`.
- Live reference: VM nexus@100.84.207.115 repo /home/nexus/spot-teleop;
  domain8 tracker vm-tracker-prod, /vm/view_mask and target state bridge;
  single tower isaac-build /nvblox_curobo consumes /wrist_map/drill_free_depth.
  Current filter mask_object_id=drill. cuRobo domain77 /fr3_joystick_mpc;
  direct-JTC Y broker lives /home/murilo/franka_ws/lab/fr3_zed_runtime/.
  VM dashboard /home/nexus/spot-teleop/fr3_real_repro/dashboard/ has a fixed
  historical drill bbox in /regrasp; never reuse it for another object.
- Existing vm-regrasp-live and held_model_sync were started previously for
  drill-only dynamic binding; claims are advisory and can expire while services
  run. Check actual processes/status before modifying, stopping or taking over.
  Production router and Y have no selection identity enforcement today.
- All resource claims expired/free; inspect board and claim before use. The
  previous attempted fixed-radius command hit an automatic-review usage limit
  but later same read-only local analysis succeeded after user resumed.

---

# THREE OBJECTS — ~06:02Z

- Hub lane request d05dc981df3d4752af7278a7e5318767 pending; user explicitly
  approved, approval note sent to Hub41db8e5ca2cf4f49bc3ec297dca99000.
  Claude child31bb4cd940fc4e5cb52c2e9e1d3b5d07 produced offline backend + HTTP
  wrapper in object_selector_1121/backend; current 37 tests pass. Follow-up
  dispatch reported failed/no activity; its state should be rechecked.
- Staged dashboard candidate/lab_dashboard.html has 3 buttons and candidate/
  object_selector.js; JS syntax +3 UI behavior tests pass. Staged VM proxy
  candidate/lab_live_server.py has safe fixed-loopback /object-target proxy;
  4 tests pass. Nothing deployed; real route/ROS adapters absent.
- Offline ZED1 and ZED2 RGB/depth synchronized individually, captured minutes
  apart. SAM2 snapshot ZED2/overlay.jpg shows 3 objects; ZED1 ball/bottle
  masks. Ball ZED1 cleaned cluster813pts extent(.062,.060,.053)m. Cross-camera
  nearest-median9mm p90 17mm. Bottle ZED1->ZED2 14mm/p90 160mm, transparent
  depth outliers; cannot assume full cloud. Raw images/data under object_selector.
- 200 GraspGen proposals for offline ball: drill selector no feasible candidate;
  candidate idx14 depth0 has tip1.5cm vs drill-only 3cm gate, plan problems[]
  but lower grasp and lift stages lack full obstacle checks; no binding issued.
  Existing plan_grasp enforces obstacle gate ONLY S1/S6, and hand/held-object
  world collisions are missing from intermediate stages. This is a blocking
  safety/functional gap, not evidence that obstacles are avoided by Y.
- Staged collision_validation/swept_esdf.py requires all six stages, full robot
  plus held-object spheres, current identity-matched map and non-target
  obstacles, table clearance and known ESDF cells. 7 unit tests pass. It is an
  offline evidence contract with injectable query, NOT wired to live kinematics,
  ESDF or Y. Planner, selected-object map carve/rebuild, ROS tracker switch,
  held-object model and ball/bottle object identity remain unfinished.
- Live robot and dashboard untouched; Y remains drill-bound. No ball grasp or
  map writes. Avoidance 4cm candidate staged only (not validated/deployed).

---

# AVOIDANCE MARGIN — 2026-11-21 ~05:49Z

- User reports avoidance too early while carrying drill, wants three-object
  dashboard ready. Live node uses node_config/mpc_override.yml: collision cost
  activation distance .05m, weight50000; robot config sphere buffer .004m.
  No live settings changed. Staged .04m candidate under
  /home/hipolito/franka_teleop_checks/object_selector_1121/candidate/;
  syntax parsed, no replay/motion validation. No simrep containers currently up.
- Held spheres on run050322 radii .070/.088/.120/.120m. Capping radii at .06
  covers only56.8%/65.9%/80.7% of three historical observed point clouds.
  Offline 4-cluster refit achieved99.78-99.90% point coverage and 25-59% less
  summed sphere volume, BUT extends up to ~69mm below observed object and when
  constrained to existing far-end rule coverage falls to97.5/97.8/98.8%.
  It is a diagnostic, NOT a deployable safety model. See
  object_selector_1121/{compare_sphere_fits.py,candidate/sphere_fit_comparison.jsonl}.
  Observed cloud coverage is not full object coverage. Keep current live model
  and .05m activation until independent replay/clearance validation.
- Three-object work remains incomplete: snapshot masks verified, staged gate
  tested, no dashboard/API/live tracker/map/ball-or-bottle grasp integration.
  Existing Y still bound to drill. No robot motion or live configuration changed.

---

# THREE-OBJECT SELECTOR — 2026-11-21 ~05:43Z

- User wants dashboard drill/tennis ball/Sprite selector, live masks, checked
  grasps, only selected target excluded from map; other objects remain obstacles.
  Robot Home/disengaged/open, no robot motion or target/map change in this work.
- Reused 11-19 SAM2 snapshot code on cached image ball_after_home.jpg. New
  /home/hipolito/franka_teleop_checks/object_selector_1121/overlay.jpg visually
  confirmed tennis ball and bottle masks. Masks and result.json in that dir;
  1011 and1849 pixels respectively, ~1.26s inference. Snapshot only, manual
  box prompts, not live tracking, grasp validation or map input.
- Existing VM fr3_real_repro/dashboard/lab_live_server.py /regrasp uses FIXED
  drill bbox [462,717,550,911] and label drill; DO NOT route new buttons there.
  Actual map input uses /vm/view_mask and ACTIVE:drill, filter mask_object_id
  drill, single nvblox map; stale geometry stays until rebuilt/cleared. Identity
  mismatch must never silently route a new mask as drill. Y remains bound to
  old drill; user instructed not to press Y for other targets.
- Staged side-effect-free target_gate.py with 7 passing tests in same directory:
  target generation, fresh matching mask, selected-target map validation and
  other obstacle retention, matching broker binding + held-object model; switch
  refused while engaged/busy/held. NOT connected/deployed. Need actual ROS
  target switch with fail-closed pause before reseed, map rebuild/validation,
  object-specific grasp generation and held model, identity-aware broker/Y,
  then dashboard integration. Existing drill ranking/geometry assumptions need
  ball/bottle validation; user-facing buttons must not falsely claim readiness.

---

# TENNIS BALL — ~05:38Z

- User requested Home and tennis-ball autonomous grasp. Called existing
  /teleop/home Trigger; completed at1795239471.603. Fresh mode2/errors{},
  disengaged, gripper open79.97mm, held=false; hand(.30768,.00009,.58977).
- Ball visible in cached ZED1 overlay, near pixel(283,334), image640x360;
  /home/hipolito/franka_teleop_checks/ball_after_home.jpg. Live tracker mask
  was elsewhere, not ball. Existing single tracker, binding, regrasp and
  /wrist_map target_state are drill-specific; filter mask_object_id=drill.
  Changing tracker alone can carve wrong object into map while old obstacles
  remain, and ball grasp and carried model are unvalidated. No ball seed,
  rebind, map alteration or pick issued. Y stays enabled for previous target:
  inform operator not to press Y until ball selection and preflight pass.
- Need coordinated target switch: stop/disengage first (done), select/verify
  ball mask, ensure non-targets map obstacles and stale ball voxels cleared,
  generate/review ball-specific grasp and held-object model, live broker checks
  from measured joints, then test under operator oversight. Planner defaults
  include drill-specific ranking/depth. Do not bypass object or map gates.

---

# Y REFUSAL — ~05:24Z

- User Y reaches teleop; two retries rejected by broker: xy drift3-4mm,
  TOP drift76mm. No motion on rejected Y. Fresh robot mode2/no errors,
  open79.97mm, disengaged, cancelled/held=false; hand(.342,-.273,.472).
- Dynamic loop is alive, current binding052029_drill_loop_regen; sync model
  ready98.8%. decisions.log reports OCCLUDED/trusted=false, cloud~3200pts.
  It correctly holds binding while arm blocks view. User instructed hold
  pad Home2sec to clear cameras, then verify regeneration+broker before retry.
- Resource robot renewed5min; VM/GPU1 reacquired15min for planning loop.
  Do not bypass top-drift or occlusion gates. No autonomous command issued.

---

# LATEST TEST — ~05:17Z (operator-driven, supersedes idle status below)

- Y initially refused stale live cloud; retry completed pick/lift run050322.
  Held model digest10ddb5799e60 loaded; Start engaged immediately afterward
  (log1795238023.932). Thus Y->Start handoff executed; physical secure grip
  is not independently confirmed by these logs.
- During pad drive repeated >50mm following-leash resets (one71mm); do not
  claim smoothness fixed. No parameter changes or commands from our lane.
- Operator released, then pad Home completed1795238158.523. Fresh check:
  mode2/errors{}, gripper79.97mm, held=false/idle, engaged=false,
  attached=false; hand(.3057,-.0001,.5888). Preview-vs-q .064rad while
  disengaged; MPC active. Robot claim renewed max5min for continued test.
- GPU1/VM planning claims expired; no service stopped by expiration. Check
  board/renew before new inference or VM mutations. Read-only log checks
  need no exclusive robot command path changes.

---

# CURRENT — 2026-11-21 ~05:06Z (supersedes all older readiness below)

- Robot freshly verified Home, mode2/errors{}, state age<1ms, OPEN79.97mm,
  idle/held=false, engaged=false, attached=false. User invited to retry Y,
  then physically confirm secure grip and press Start for cuRobo pad control.
  Start is required after lift; automatic engage is NOT implemented. Back stops.
- Previous Y run 20261121_044956_4e47ef9a completed lift but drill SLIPPED.
  Width0.216mm exposed empty gripper while held flag stayed true. User confirmed.
  Open+Home recovered; empty teleop restart cleared stale held and canonical
  broker lock release cleared old HELD_OBJECT. No post-lift slip detection yet.
- Fresh binding 20261121_050322_drill_oneshot_regen: broker PASS from measured
  Home, age0.7s, drift0.8mm, problems[], obstacle>=.210m/table>=.173m,
  466 points between pads. Held model v3 automatically uploaded, coverage98.2%.
  Y runtime enabled, idle, ready_reason=null. No pick triggered by our code.
- Dynamic regrasp was NOT running; restored CPU-only VM container
  vm-regrasp-live (image vm-grasp-loop:latest, domain8, loop.py --anchor-run
  binding --zedt-dir .../regrasp/calib_live_20261121). Planning/binding only,
  no goal or robot commands. First automatic regen045938 succeeded but broker
  later rejected palm clearance; manual fresh regen050322 now passes.
- New VM regrasp/held_model_sync.py watches binding each1s, generates v3 model
  through held_spheres_v3.py, validates coverage>=98%, then atomically uploads
  attached_spheres/<run>.json to lab. Host nohup PID/log under regrasp/state/
  held_model_sync.{pid,log}. Latest MODEL_READY050322 confirmed on both hosts.
  Neither service is in stack manifest yet; persist and commit selectively.
- Local helpers: /home/hipolito/franka_teleop_checks/{held_recovery_preflight.py,
  broker_recovery_preflight.py,held_model_sync.py,refresh_grasp_input.py}.
  Broker helper action=plan only; record fr3-ros2:/runs/grasp_broker_preflight_044818.json
  (filename fixed, content now050322). Old log preserved runs/
  joystick_grasp_teleop_pre_slipclear_20261121.log. Motion speed UNCHANGED.
- Resource claims: franka-robot renewed ~05:05Z max5min for user retry; GPU1
  and omnigent-vm held for dynamic planning services. Renew/check board.
  Current services/models not committed. Held-cancel fix remains deployed,
  21/21 tests passed. Grasp slip and held-motion stutter remain open.

---

# CURRENT — 2026-11-21 ~04:49Z (supersedes held-drill notes below)

- RECOVERY DONE. Initial cuRobo ramp aborted at 10 cm tracking lag, no fault.
  Separate 3 cm down / XY alignment / 5 mm/s descent reached lift pose;
  S3 joint correction err .0184 rad, S4 put-down .0119 rad. Operator opened
  and adjusted drill. Open width79.97mm, held=false, attached=false verified.
  HOME viaPRE passed .0147/.0127rad. Mode2/no errors, state stamps <10ms.
- Cancellation fix now DEPLOYED via teleop-only restart while empty/idle.
  Old pid15772 stopped; new node detected js0 XInput, starts disengaged,
  published=0. Pre-restart log preserved at
  /home/murilo/franka_ws/runs/joystick_grasp_teleop_pre_heldcancelfix_20261121.log.
  Source/test backups *.bak_held_cancelfix_20261121; 21/21 tests passed.
- Fresh drill BOUND: 20261121_044818_drill_oneshot_regen (VM runs dir).
  7798 source cloud points, 200 GraspGen samples; visual overlay on drill;
  drift5.2mm/cloud age1.6s, brokerPASS from measured Home, problems[].
  Min obstacle .200m/table .190m; 310 points between pads.
  Binding backup grasp_binding.json.bak_20261121_044820.
  New v3 held model uploaded runtime attached_spheres/<run>.json, coverage
  99.2%, lowest sphere z=.079 at lift. No new pick executed by this lane.
- User told Start joystick ready and Y fresh-ready; do not move the drill.
  Robot claim renewed for operator testing (max5min); GPU1 released.
  Motion speed UNCHANGED. User asks a faster auto cycle; upward stutter
  remains unresolved. Do not claim complete avoidance: S3 lift direct JTC
  without payload collision model; ESDF unknown treated as free.
- Recovery logs host /home/murilo/franka_ws/runs/held_recovery_*_20261121.log;
  broker record fr3-ros2:/runs/grasp_broker_preflight_044818.json.
  Resume helpers /home/hipolito/franka_teleop_checks/{held_recovery_preflight.py,
  broker_recovery_preflight.py}; all read-only. Historical held-plan recovery
  scripts/old S4 below are no longer needed/current.

---

# RESUME — 2026-11-21 ~04:30Z (supersedes runtime statements below)

**Immediate: drill still HELD; do not press B/open or replay old S4.** Coordination
lane 55055385330f4def83027a49ae00119e has handed over and stopped (plan limit).
User goal remains smooth hand/pad cuRobo+nvblox teleop and Y drill pick/place;
first resolve the held-drill recovery, then deploy/test the cancellation fix.

- Latest physical state is PEER-REPORTED, not freshly read here: disengaged,
  hand ~(0.432,-0.127,0.591), held run `20261121_040403_drill_oneshot_regen`.
  Pad dropped at 04:08:55Z (`js0 Errno 19`); cancel cleared `plan=None` while
  `held=True`. Start cannot find collision model; Y refuses. B opens ~25 cm
  above table. MPC v3 drill model attached by direct publication, digest
  `fc5d97ee67c3`; robot claim released. Recheck board/live state before use.
- Murilo said “bring it down” at 04:27Z; coordinator asks a fresh Murilo go
  and e-stop confirmation when running recovery. Acquire `franka-robot`, read
  fresh stamped mode/errors/q/TCP/engaged/gripper/controllers, preview-vs-q and
  ESDF first. No robot command or teleop restart was issued by this lane in
  this fix turn. A restart loses held adoption and cannot rescue this state.
- Prepared recovery artifacts in `fr3-ros2:/tmp`: `coord_plan.json`,
  `coord_panda_fk.py`, `coord_coord_mpc_move.py`, `coord_set_attach.py`.
  Reported S3 end q `[-0.061,0.026,-0.36,-2.18,-0.328,2.39,0.226]`.
  These are leads for live preflight, NOT a validated trajectory from current
  state; latest coordinator explicitly says do not blindly replay old S4.
- Code-only fix STAGED in host runtime
  `/home/murilo/franka_ws/lab/fr3_zed_runtime/joystick_grasp.py`:
  cancel preserves plan while held (idle or busy); place requires width
  10–75 mm and <=0.08 rad from S3_lift end. Missing pose/plan refuses.
  Does not restore the already lost in-memory plan. Deploy after safe set-down.
  Backups: `joystick_grasp.py.bak_held_cancelfix_20261121` and
  `test_joystick_grasp.py.bak_held_cancelfix_20261121`.
  Test: docker exec fr3-ros2 bash -lc 'source /opt/ros/jazzy/setup.bash &&
  source /ros2_ws/install/setup.bash && cd /lab/fr3_zed_runtime &&
  python3 test_joystick_grasp.py' — **21/21 PASS**, isolated DDS193,
  robot/action/subprocess/broker mocked. Older stale test expectations updated.
- `joystick_grasp.yaml input_timeout: 0.0` is LIVE, backup
  `.bak_inputtimeout03_20261121`. Fixed steady LB/stick event timeout;
  three empty-gripper up/down cycles verified (TCP .198→.296 m, ~.04 m/s).
  Later held-drill stutter remains OPEN: inspect MPC/ESDF, preview gate,
  sphere timing, watchdog; do not assume payload/strength or sphere fault.
- Held-object v1: attach topics `~/attached_spheres` + `~/attached_status`,
  4 spheres in fr3_hand. Startup held without known plan refuses; Y place
  pose guard .08 rad in teleop. MPC standby attachment processing fixed.
  `attached_radius_cap` default .15 is a no-op; backup
  `curobo_mpc_node.py.bak_pre_radiuscap_20261121`. Automatic grasp stages use
  DIRECT JTC, not MPC avoidance; model attaches after S3 lift, so lift-stage
  payload collision coverage is not established. ESDF unknown is free.
- Remote authoritative coordination docs (VM103, not local paths):
  `/home/nexus/spot-teleop/HANDOFF_2026-11-21_held-drill-pad.md` and
  `/home/nexus/spot-teleop-worktrees/hub-coordena-o-e-valida-o/HANDOFF_2026-11-21_coord-to-hipolito.md`.
  Runtime handoff also appended locally in `fr3_zed_runtime/HANDOFF_2026-11-13_joystick_grasp.md`.
- No new commit/push for these runtime fixes. Runtime is outside Git; local
  `/home/hipolito/spot-teleop` is extensively dirty. Integrate selectively;
  never broad-stage. Standing commit author: MHC-CodeSmith, no AI trailer.
  Only drill validated; ball/Sprite/Phillips paths still unvalidated. Single
  map policy: carve selected drill only; other objects remain obstacles.

---

# FR3 drill automatic grasp handoff — 2026-11-21

## Result
A real cuRobo/nvblox drill pick-and-place cycle completed successfully under the operator's e-stop supervision. The tracker had drifted onto a box on ZED2, so the tracker container was restarted and reseeded on the drill using ZED1 only. ZED2 cold-init still rejects its reprojection (pixel 388,20; depth 1.38 m vs 0.98 m expected); this is fail-safe and the planning cloud remained ZED1-only.

- Fresh grasp run/binding: `20261121_003600_drill_oneshot_regen` (VM103: `/home/nexus/spot-teleop/fr3_real_repro/grasp_loop/runs/20261121_003600_drill_oneshot_regen`).
- Seed: ZED1 bbox_1000 `336,764,422,998`, seed pixel `(243,328)`; drill mask ~1.6%, live cloud 2,288 points.
- Broker preflight passed: cloud age 0.9 s, cloud/binding XY drift 0.0006 m, no plan problems. Minimum obstacle clearance 0.246 m; minimum table clearance 0.172 m. The plan used the live nvblox mesh and the calibrated table estimate (z=0.0281 m).
- Plan target: hand (0.2341,-0.3243,0.2986) m; TCP (0.2430,-0.2944,0.2000) m; approach (0.086,0.289,-0.953).
- Execution: all stages passed; max joint errors S1/S2/S3/S4/S5/S6 = 0.0157/0.0145/0.0146/0.0143/0.0140/0.0123 rad. Gripper confirmed the pickup before lift. The second Y-equivalent call set the drill down, opened, retreated, and returned home.
- Final fresh read: robot mode 2, no current error flags, TCP (0.3079,-0.0035,0.4855) m, teleop disengaged; grasp status idle, held=false.
- Runtime record: `/runs/joystick_grasp/20261121_004315_44824d75.json` and sibling `.log` in `fr3-ros2`.

## Current setup (latest safety state)
`/home/murilo/franka_ws/lab/fr3_zed_runtime/joystick_grasp.yaml` has `auto_grasp_enabled: false`; backup before this safety edit is `joystick_grasp.yaml.bak_pre_zed1_rereg_20261121`. No `fr3_input_teleop` process is running. `/fr3_joystick_mpc/set_active false` returned `standby: not publishing`, and `/fr3_joystick_mpc/state` read `standby`. The resource board is free. Do not restart teleop or activate MPC until Murilo's final confirmation after reviewing the independent preflight.

The launcher `/home/murilo/franka_ws/lab/fr3_zed_runtime/start_joystick_grasp.sh` starts disengaged and does not switch controllers or command motion; it exits if another teleop is running. The broker is `/home/murilo/franka_ws/lab/fr3_zed_runtime/grasp_broker.py`, socket `/home/murilo/franka_ws/lab/fr3_zed_runtime/grasp.sock`. XInput Y is button 3; DInput/Bluetooth Y is button 4. `/teleop/grasp` triggers the same path as Y. Broker `action=plan` is planning-only: it reads the live cloud, binding, and nvblox mesh; it does not publish robot goals.

## Still open
Only the drill has a validated mask/binding/planner path. Do not reuse the drill binding for the ball, small Sprite bottle, screwdriver, or other table objects. No validated multi-object segmenter or object-specific grasp plan was found for them. Before testing another object: select/seed that exact object, generate and bind a fresh candidate, then require a passing live broker preflight with the other objects remaining collision obstacles.

## Key paths
- Tower runtime: `/home/murilo/franka_ws/lab/fr3_zed_runtime/`
- Y enable config: `joystick_grasp.yaml`; launcher: `start_joystick_grasp.sh`
- Teleop service implementation: `fr3_input_teleop.py`, `joystick_grasp.py`; stage executor: `vendor/grasp_stage_exec.py`
- Planner/binding source on VM103: `/home/nexus/spot-teleop/fr3_real_repro/grasp_loop/grasp_plan_lib.py`
- Robot ROS domain 77; live perception domain 8; single map node `/nvblox_curobo`
- Relevant tracker lane: `55055385330f4def83027a49ae00119e` (Morys5).

No commit was made. The spot-teleop working tree already had extensive unrelated modifications/untracked files; this note is separate and should be reviewed selectively.

## Safety update — ZED1 re-registered; map rebuilt; execution still on hold
A calibration-lane check found ZED1 physically rotated about 6.3°; with the old TF, positions were estimated 7–11 cm off. Morys5 reports that the vision-only ZED1 correction is now deployed with Murilo's go and passed a no-motion check (arm residual 5.88 mm, inlier fraction 0.35, table error -0.4 mm). The merged ZED1/ZED2 drill cloud reports 1.1 cm inter-camera offset, down from 7.7 cm, and ZED2 now tracks the drill. Murilo authorized the map clear and grasp regeneration. Morys5 reports `/nvblox_curobo` restarted with its identical command at 02:04:59Z (PID 3579919); the rebuilt map's depth agreement is 8.5 mm median / 13 mm p90. ZED1/ZED2 overlap within 2 cm improved from 11% to 32%.

Fresh post-fix drill binding from merged cloud: `20261121_020704_drill_oneshot_regen` at `/home/nexus/spot-teleop/fr3_real_repro/grasp_loop/runs/20261121_020704_drill_oneshot_regen`. Cloud n=3718 (ZED2 n=1650); pick index 31, confidence 0.81, score 0.53, top-down approach `(0.103,-0.122,-0.987)`, no plan problems, IK residual 0.01 mm, target TF `(0.306,-0.317,0.246)`, pre-bind drift 6 mm. Previous binding backup: `grasp_binding.json.bak_20261121_020709`.

Murilo waived the spread-pose calibration re-check. A fresh broker preflight against the new binding has now passed from this lane and independently from Morys5; keep Y disabled/MPC in standby until Murilo confirms after reviewing those results. No robot motion yet on the post-recalibration binding.

Murilo requested an independent planning-only preflight before any Y trigger. This lane's broker socket `action=plan` request (no controller goal, no motion) against run `20261121_020704_drill_oneshot_regen` passed: cloud age 0.5 s, XY drift 2.94 mm, no plan problems, IK error 0.01 mm, minimum obstacle/table clearances 0.180/0.167 m. Exact request uses AF_UNIX stream `/home/murilo/franka_ws/lab/fr3_zed_runtime/grasp.sock` and JSON `action=plan`, UUID owner, `time_ns()` sequence, and fresh measured `start_q`; `grasp_broker.py` itself has no plan CLI option. Morys5 independently ran the same VM planner: cloud age 0.60 s, XY drift 5.6 mm, no problems, IK 0.01 mm, minimum obstacle clearance 0.183 m/table clearance 0.167 m. Their visual projection into both ZED1/ZED2 showed the TCP straddling the drill upper body, top-down.

After Murilo's separate-preflight request, this lane has not restored Y, restarted teleop, or activated MPC. Fresh robot-state read was stamped current, mode 2, error flags false, joints near Home. This lane's `controller_manager/list_controllers` request timed out although the service was listed; Morys5 independently reports that it responds and confirms `fr3_arm_controller` active, broadcasters active, impedance inactive, and 100 Hz joint states. Await Murilo's final confirmation after reviewing the passing preflight and visual alignment; no Y trigger until then.

- Tower `joystick_grasp.yaml`: `auto_grasp_enabled: false`; backup is `joystick_grasp.yaml.bak_pre_zed1_rereg_20261121`.
- Teleop was restarted disengaged to load that setting, then stopped while still disengaged because Start could re-enable the MPC path. `/fr3_joystick_mpc/set_active false` returned `standby: not publishing`; `/fr3_joystick_mpc/state` read `standby` afterward. Another lane independently verified no teleop process, MPC standby/no trajectory output, and the free robot resource. Last pre-stop read had `/teleop/engaged=false` and fresh joints near Home.
- No robot motion or controller switch was issued for this safety hold. The `franka-robot` resource is currently free. Manual cuRobo teleop remains stopped pending Murilo's final confirmation after reviewing the independent preflight and visual alignment.
- Calibration/grasp lane: Morys5 session `55055385330f4def83027a49ae00119e`; it reports the map rebuild, fresh binding, independent preflight, and two-camera visual alignment complete. The spread-pose re-check was waived by Murilo. Await his final confirmation before any motion test. Filter and other `wrist_map` components were not changed by that lane.

---

# UPDATE — Sprite real-cloud cross-camera gate (2026-11-21 07:42Z)

- Replayed the live lane's `sprite_cap()` estimator on stored real masked
  clouds. ZED1 cap=(.72138,-.24082,.26009)m (1,662 pts), ZED2
  (.73753,-.23497,.27209)m (3,212 pts), merged (.72881,-.23806,.26987)m.
  Per-camera estimates differ 17.2mm XY / 12.0mm Z; all individually return
  no problems. Live plan cap_top=(.7212,-.2399,.2504)m is ~10mm below ZED1.
- This fails the ≤5mm cap-pose objective. Existing 20mm fit-RMS gate allows
  the inconsistent pose. I sent urgent note `91299a9ea3b4` to Claude lane
  `cc29b70dbb7844e5927a4e714b3f6977`, asking to keep Sprite Y disabled pending
  reconciliation/ground truth. No live mutation or motion from this lane.
- The last GET snapshot (07:34:40Z) reported Sprite selected and Y enabled;
  treat that as stale but safety-relevant until the live lane confirms a lock.
  Verify live target status before any operator test.
- Details recorded in `object_selector_1121/backend/BOTTLE_VALIDATION.md` §8.
- Follow-up read-only check 07:45Z: live API still says Sprite `ready`,
  `y_enabled=true`, fresh TRACKING/mask/map (ages 0.2/0.0/0.0s), and
  `busy=false`. Resource board still shows the Claude lane holding
  `franka-robot` for “Sprite pick in progress / recovery (user go)”. These
  signals do not establish physical robot state or validate the cap pose. I
  sent urgent note `8f9c73fa0994` asking the lane to confirm whether motion
  started and pause/keep Y locked; no response had arrived by 07:45Z. This lane
  issued no robot command. Do not press Y pending explicit status and corrected
  cap validation.
- Full offline suite rerun at 07:43Z: backend 74, map filter 9, proxy/API 5,
  UI 3, clearance 7, swept replay + bottle route 8 = 106 tests passed.

# UPDATE — Sprite live cap pick result (2026-11-21 07:49Z)

- Other lane reports one user-authorized Sprite cap pick: S1/S2 completed, close
  failed at 0.2mm finger width. Bottle remained upright; no grasp. Recovery
  completed S5 retreat + S6 Home (0.009/0.020 rad errors), then gripper opened
  to 80mm in air. That lane released `franka-robot`; board now confirms free.
- Read-only `GET /object-target` at 07:48:58Z independently confirms
  Sprite `y_enabled=false`, ball false, drill true; Sprite grasp reason records
  the cap-pick miss and 10–17mm camera error. Tracker/mask/map report fresh and
  100% target carving. This is selector/API status, not a robot-state read.
- Other lane reports stale `held=true` / `regrasp.lock=HELD_OBJECT` remain after
  the failed close/recovery; it asked operator to clear with B. Do not assume
  this is cleared until read fresh. Target switching remains blocked in API.
- Next proposal is horizontal/pitched Sprite label-band grasp, offline only.
  Existing physical diameter uncertainty (roughly 71–91mm) overlaps/exceeds
  79.9mm gripper opening, and camera estimates disagree; no supported pose or
  clearance proof exists. Do not enable Y or test that candidate until measured
  dimensions + calibrated pose + full grasp/payload path validation pass.

# UPDATE — Sprite side-grasp supervised cycle (2026-11-21 08:03Z)

- The selector lane reports a successful supervised Sprite side-grasp/pick/place
  on run `20261121_075900_sprite_selector`, after the operator moved the bottle
  closer (reported axis near `(0.483,-0.074)`). Approach/descent/close/width
  check/lift and held-model load passed; second trigger placed the bottle. Lane
  reports Home, gripper open, idle, no regrasp lock. `franka-robot` is free.
- Read-only `/object-target` at 08:02:53Z reports generation 8, Sprite selected,
  mask/map fresh, target fully carved, other objects retained; drill and Sprite
  Y enabled, tennis ball disabled. Grasp reason marks Sprite as a supervised
  pitched label-band trial. This is selector status, not independent robot-state
  verification.
- The same status gives camera error ~10mm versus 7.5mm/side slack. Lane reports
  fixed 60N squeezed PET to 24mm. Therefore record this as one successful
  supervised cycle at the moved bottle pose; it does not establish robust grasps
  across bottle poses or justify the fixed force for arbitrary fill/shape. Keep
  it supervised until camera error is bounded and grip force is checked against
  bottle deformation.
- Lane reports the 4s `ros2 topic echo` polling that caused stale-joint aborts
  has been removed, and an AprilTag is now on the table for camera
  reconciliation. Tennis ball remains locked pending valid calibration and a
  grasp path.
- Calibration distinction for ball follow-up: prior `ball_calib/RESULTS.md`
  tested RealSense wrist-camera-to-hand rigid extrinsics (near-top-down RMS
  5.2mm; rotated views 6–15mm and impossible table heights). AprilTag-based
  ZED1/ZED2 reconciliation may reduce the stereo ball-cloud disagreement, but
  does not validate that wrist-to-hand transform. For the ZED ball path, verify
  tag fit on held-out observations and table height, then rerun fixed-radius
  ball fit + measured-budget GraspGen replay (previously 0/800 candidates),
  broker/stage/payload checks. Keep ball Y locked until those gates pass.
