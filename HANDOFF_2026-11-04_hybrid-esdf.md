# Handoff 2026-11-04 ~15:10 UTC — hybrid first-step + ESDF upstream (offline only)

Lane ZED2-nvblox-grasp. No shared changes: no proxy/map/TF/arm/MPC edits, no goals,
no activation, no restarts. Lab scratch CSVs removed. Teleop lane = 7c (Hipolito).

## Hybrid first-step: why 20/20 disarmed checks failed (no guard changed)
- Data: `/tmp/*.csv` copied from lab `codex-fr3-live-mpc-hybrid-1104` (exited) and
  live `codex-fr3-live-mpc` (teleop lane already reverted it to measured).
- Hybrid: warmup diff 0.0000 rad (reseed OK), then cmd-vs-measured walks:
  step10 0.0008, step20 0.0048, step30 0.0142, step40 0.0282; steady median
  0.028, 100% of last 200 > 0.015 guard. Tracking gate 0.025-0.031, ~13% reseeds.
- Measured baseline: warmup 0.0000, steady median 0.0011, 0% > 0.015.
- Cause: hybrid plans open-loop from own last command while the disarmed bridge
  sends 0 (arm static). Constant wrist goal, pose_err grew 0.0023->0.0079, so the
  rollout walks away from the static arm and crosses 0.015 ~0.6-0.8 s after
  activation. Any disarmed check after ~1 s of active hybrid must fail.
- Structural: `reseed_error_rad` 0.03 > bridge guard 0.015 (2x).
- Open: why a hold-pose rollout drifts 0.028 at all (IK branch? collision
  gradient? velocity walk?). coll_cost 12 vs measured 8.2 at same arm pose.
  Direction (not done): sample first preview right at reseed, or arm promptly.

## ESDF candidate review (read-only, /home/hipolito/franka_teleop_checks/esdf_candidate/)
- Sound: exact-bytes key, no cache (fail-closed), 0.7 s < MPC 1.0 s abandon.
- Concern 1: timed-out flight holds the slot until upstream finishes; upstream
  takes up to 8 s (live: 4333/3116 ms + an 8 s timeout today) -> one slow call
  blocks ALL queries up to 8 s. Suggest detach-on-timeout w/ generation id.
- Concern 2: `rclpy.spin` single-threaded server serializes 0.7 s-blocked
  handlers; fine at 4 Hz single client, piles up under concern 1.
- Concern 3 (minor): identical concurrent requests fail instead of sharing.

## Upstream nvblox delays / timestamps (verified read-only)
- `EsdfAndGradients` response has NO stamp (origin/voxel/data/success only).
  Freshness proxy: stamped `/nvblox_curobo/static_esdf_pointcloud`, `/mesh`,
  `/combined_map_slice` (publish time, not compute time).
- Load: GPU1 99%, 84 C; two 2 cm nvblox nodes + tracker (1.5 GB) + isaac-sim
  python share it. MPC hammers update_esdf=True at 4 Hz; each node also
  integrates 5 Hz + mesh/esdf 2 Hz. Response ~350 k vox ~= 5.6 MB over
  cross-host DDS. nvblox served one observed query in ~6 ms -> spikes are
  contention/transport bursts. Live proxy: 290 k calls, 86% cache hits.
- `vm-esdf-proxy` (domain 143) is stale ("live service not ready" spam) but
  harmless: different domain, cannot steal domain-142 calls. Cleanup needs owner.

## Uncached benchmark >5 s: NOT nvblox compute (15:20 UTC, read-only)
- In-node serve over 846 reqs/30 min (vm-nvblox-prod log deltas): p50 26 ms,
  p90 115 ms, p99 171 ms, max 216 ms, ZERO > 1 s. The 3-8 s proxy-upstream
  spikes and the >5 s direct benchmark are downstream of "Successfully wrote":
  ~5.6 MB service response (~350 k vox x 4 floats) over cross-host UDP DDS
  (~90 fragments) + client discovery/matching on first request.
- GPU1 attribution (pmon): tracker sam2 (2424011) 30-45% SM, nvblox-prod
  17-19%, nvblox-seg 10-22%. Tracker is top GPU user, not nvblox.
- Buffers: kernel 8 MB default/16 MB max both hosts; proxy uses 8 MB UDP
  profile; nvblox default profile. Benchmark client config unknown — if it
  ran defaults, fragment-loss stall is the prime suspect for the >5 s first
  request (single cold sample conflates discovery + transport).
- Conservative diagnostics proposed (no map changes): same-host rerun,
  cold-vs-warm x5 sequential, AABB size scaling, update_esdf=false variant,
  client buffer check, proxy-slow vs in-node timestamp correlation.

## Same-host probe 10/10 fast; cross-host path is the staller (15:30 UTC)
- Ran teleop lane's spec verbatim from inside vm-nvblox-prod (domain 8):
  discovery 2.759 s (cold); small 0.5^3: 0.011/0.137/0.030/0.119/0.067 s;
  full 1.4x1.6x1.2: 0.023/0.105/0.053/0.029/0.086 s, 350811 floats each.
  Two "sequence size exceeds remaining buffer" at startup (discovery burst).
- vs their cross-host: 0.163/2.732/0.005/0.071/>5 s timeout on 70 KB replies.
- UDP counters: lab UdpRcvbufErrors 1,016,794,088 (!), VM 2,799,664; InCsum 0
  both. Receiver-overrun drops + ~3 s reliability heartbeat explain the
  2.7 s / >5 s stair-steps. Same-host clean => loss is on the VM<->lab path
  (competes with 2-4 MB depth bursts at 25 Hz). No hold; probe complete.

## ZED2 (unchanged, still open)
- Published TF still yesterday's tag-reg (not validated, NOT integrated).
  Next: 2nd tag spot (Murilo offered to move it) or arm-pose FK check, then
  decide TF; deploy + nvblox/grasp integration need Murilo's go.

## INTERRUPTED ~15:35 UTC — TCP prototype NOT started (plan window 95%)
- Was idle: same-host probe done/reported, holding nothing, nothing running.
- New 7c asks: (1) investigate isolated VM-lab TCP forwarding prototype
  (VM-local d8 query + lab-local d142 server, exact serialization, gen IDs,
  strict deadlines, no stale cache); measure TCP path w/o live duplication
  or report design constraints; touch nothing shared. (2) Caveat: per-socket
  drops=0 on proxy sockets — 1.017B lab RcvbufErrors are system-wide, NOT
  attributable to ESDF alone. UDP-loss claim now HYPOTHESIS, not proven.
- Next step after reset: `sys_duet_inbox` first; then reply to 7c
  (session 7c04690c6bdd4c70aaf56606cb7191e1) with TCP constraints or an
  isolated prototype in scratch only (no shared proxy/map/MPC/robot edits,
  no live queries without coordination).
- Files: this handoff; candidate
  `/home/hipolito/franka_teleop_checks/esdf_candidate/` (`transport_probe.py`);
  probe spec = 7c duet note (5x small AABB + 5x full, d8, update_esdf true).

## TCP comparison DONE (~16:00 UTC) — wire healthy, prototype in scratch
- Synthetic TCP VM->lab, sequential <=1Hz, TCP_NODELAY, 0 stalls (52 reqs):
  cable 10.0.0.3->.2: 70KB 1-6ms, 1.4MB 5-10ms, 5.6MB 17-19ms;
  tailscale: 70KB 2-6ms, 1.4MB 8-14ms, 5.6MB 30-44ms. Connect 2-3ms.
  Raw IP on BOTH paths is fine -> stalls are DDS/ROS-layer. Caveat: probe
  ran without concurrent depth-burst load; loaded comparison needs coordination.
- Prototype (VM `/tmp/tcp_esdf/`, NOT deployed, no ROS tested): `esdf_wire.py`
  exact struct serialization + CRC32 + gen u64 (offline test PASSED incl.
  350,811-float bit-exact round-trip); `tcp_esdf_vm_bridge.py` (TCP srv + d8
  map client; replay mode = 0 map queries; live = claimed/bounded; ROS 0.7s);
  `tcp_esdf_lab_server.py` (isolated `/test/tcp_esdf_forward`, d142,
  conn-per-request, deadline 0.9s, fail-closed, no cache). Loopback replay
  e2e: 4-8ms for 1.4MB. Design + constraints: `/tmp/tcp_esdf/DESIGN.md`.
- Lab `/tmp/tcp_esdf/tcp_resp_server.py` left for 7c, not running. Nothing
  shared touched; holding nothing. 7c data folded in: CDR codec ~1ms,
  proxy-socket/NIC drops 0 -> suspect = DDS reliability/reassembly under
  concurrent load, or matching edge cases.
- ZED2 still open (unchanged): published TF unvalidated, not integrated.

## LIVE TCP comparison DONE (~16:30 UTC) — 14/14 OK, 0 stalls, then cleaned up
- Path: lab `codex-fr3-live-mpc` (isaac python, d142) `/test/tcp_esdf_forward`
  -> TCP 10.0.0.3:48191 -> `vm-nvblox-prod` bridge (d8) -> live
  `/nvblox_curobo/get_esdf_and_gradient`. Full MPC AABB, 15 map queries total.
- Phase A (10x update=true, <=1Hz): 73/257/142/93/196/110/98/179/93/85 ms,
  all success, n=350811, neg drifts 248446-248666 (live map, no stale).
  Discovery of /test service: 1.25s cold, 0.25s warm.
- Phase C fail-closed x2: (1) bridge SIGTERM'd (rclpy shutdown, socket open,
  ROS dead) -> success=false 5ms via dropped conn; (2) bridge SIGKILL'd
  (port closed) -> success=false 4ms via refused. Both well within deadline.
  Bug found+fixed: bridge now sends T_ERR on ROS exceptions (was silent drop).
- Phase D: recovery 2/2 (75/82ms); full-clear sphere r=2m -> neg=350811/350811
  (clear geometry round-trips; -1000=cleared/unknown); noupdate sha1 vs
  same-host direct_ref: dims/origin/voxel/success EXACT, bytes differ because
  full-clear transiently wiped stored ESDF between queries; direct 1min later
  neg=248677 (map self-healed, no restart, MPC in standby throughout).
- vs direct cross-host DDS baseline (7c): 0.163/2.732/0.005/0.071/>5s timeout.
  Verdict: bypassing cross-host DDS removes stalls; guards/freshness untouched.
- Artifacts: VM `/tmp/tcp_esdf/` + lab `/tmp/tcp_esdf/results_from_vm/`
  (prototype, DESIGN.md, phaseA/D.json, direct_ref.json). Lab phaseC.json +
  labserver.log in lab `/tmp/tcp_esdf/`. Scratch bridge+server STOPPED.
  Claims released (lab-nvblox earlier, robot+mpc now). Nothing shared changed.
- Murilo's "no-map joystick" order BLOCKED BY DESIGN: command_guard requires
  esdf_stamp<1.5s + grid_covers_robot (verified, no bypass flag); with
  use_esdf=false every packet rejects. No changes made. Awaiting his call:
  order guard redesign from teleop lane, or run map-ON proxy-vs-TCP A/B
  (needs joystick reconnected; /dev/input/js* absent).
- ~16:05 UTC per Murilo ("run it with the map"): map-ON joystick stack STARTED,
  guards untouched. Measured MPC was already active+publishing (hybrid gone);
  set_active true reseeded OK. HW bridge up in fr3-ros2 (PID 93207,
  /fr3_curobo_hardware, :8784/status: disarmed, sent=0, command_ready=true,
  JTC active, js age 2ms). Teleop up (input_source=joystick, 0.10/0.08,
  scale 0.5, log /tmp/teleop_joy_1104.log in fr3-ros2) waiting for pad.
  Map read-only verify: 350811 floats, 248548 neg (=baseline), 104ms.
  Robot mode 2, no errors. Pad STILL absent -> Murilo to plug + Start.
  Holds: franka-robot, lab-mpc-test, lab-nvblox. Notified 7c (a9a3c25c07b).
  V2 bridge fix (single-spinner/bounded-1/cancel) written to
  /tmp/tcp_esdf/tcp_esdf_vm_bridge_v2.py, load test NOT yet run (paused).
- ~16:15 UTC 7c coverage caveat answered pre-motion (pad still absent,
  published=0, sent=0): 3 read-only update_esdf=false AABB queries;
  layout xyz stride-confirmed; table sheet present z=-0.05..-0.01;
  workspace box 44.5% free / 2.2% near / 0.3% inside / 53.1% unobserved
  (standing unknown-as-free, same as pre-clear); closest obstacle ~0m at
  box floor = table. No pre-clear LOCAL ref exists (global only), so
  structure-sane-now, not a diff. Scripts in /tmp/tcp_esdf/
  (workspace_coverage, esdf_profile, esdf_maps). Reply a3bca27f1682.
- ~16:20 UTC pad js0 plugged (8BitDo Ultimate 2C, dinput map opened by
  teleop). Bridge disarmed sent=0 rej=0 cmd=true. Live watch armed
  (6 min: teleop engaged/published + MPC timeout/stale). Murilo to Start.
  Ack efa04e774a77. Holds: franka-robot, lab-mpc-test, lab-nvblox.
- ~16:25 UTC Start did nothing: my procedure missed bridge arming (dashboard
  Start normally does it). Teleop engage() needs bridge armed; tick ignores
  the (False,'cuRobo output disarmed') → silent. Pad/mapping verified OK
  (BT dinput, js0). Fix: /fr3_curobo/enable true → armed, mode_ready,
  'waiting for joystick Start', sent=0. No guard changed. 7c told.

## ~16:20 UTC speed doubling (Murilo direct go)
- Pre-restart logs saved to teleop_run_1104_mapon/ (teleop 12KB, mpc csv 17.7MB, bridge sent=15059, 3 rejections: 2 stale ESDF + 1 first-step q=0.0168; 3 leash resets).
- Speed formula confirmed in lab /lab/fr3_zed_runtime/fr3_input_teleop.py:711: v = stick * v_lin * speed_scale. Old effective 5/4 cm/s, new 10/8 cm/s (params 0.20/0.16, scale 0.5).
- Teleop restarted (old 93491/93522 dead, new 106925/106959), pad opened, awaiting Start. Bridge armed, MPC active, guards unchanged.

## ~16:45 UTC guards-down (Murilo explicit order B)
- Shadow dir lab curobo_live_noguards/ runs the bridge; Hipolito's files untouched (.bak_guardsdown_1104 beside them).
- Disabled: 5 staleness timeouts, feasible flag, grid coverage, tracking .05, first-step .015, replay/dt, step+accel limits, feedback-stale raise, teleop leash (max_error 10.0). Kept: schema/names/finite, URDF joint limits, 1.201 vel, controller gates, workspace box, MPC internals.
- Revert: kill noguards bridge, relaunch curobo_live/joystick_bridge.py hardware --domain 77 --goal-port 47928 --packet-port 47927, re-arm /fr3_curobo/enable, teleop max_error 0.08.
- Stumbles: self-match kill dropped one ssh (recovered, verified others alive); first shadow launch on host failed (bridge lives in fr3-ros2, /lab path) -- corrected.

## ~17:05 UTC MPC restarted in hybrid (Murilo order, single variable)
- Old container codex-fr3-live-mpc-measured-noguards-1104 kept stopped (rollback = stop hybrid, rename back, start, re-arm). Run script /tmp/mpc_run_hybrid.sh on lab; cmd staged /tmp/mpc_cmd_hybrid.txt.
- Activation via /fr3_joystick_mpc/set_active (domain 142, from fr3-ros2). First-cmd check pre-arm: 0.0255->0.0296 max 0.0301, stable, limit 0.100; hybrid own-command confirmed.
- Bridge re-armed, teleop disengaged awaiting Start. Old csv saved teleop_run_1104_mapon/codex-live-mpc-measured-noguards.csv (24MB).

## ~17:15 UTC hybrid set as default (Murilo order)
- 3 layers: live container explicit (verified), /tmp/mpc_run_hybrid.sh pins it, code default commanded->hybrid in repo vm_real/node:93 + lab hub-curobo/node:101 (backup .bak_hybriddefault_1104, both py_compile OK). Live unaffected (explicit already).
- Motion-matched A/B (goal>0.02): pose_err med 0.376->0.250, p90 0.686->0.413; goal med 0.051->0.119 (drove 2x faster in hybrid). Hybrid n=227 rows.
- No maintained test: one-token default change, direct checks only (durable-test-collateral: config-value scale).

## ~17:55 UTC drill.json live_model (Murilo order)
- seg_view.py: added live_model block (tracker 3D recentered+projected); grasp-time fields frozen. Backup /tmp/seg_view.py.bak_livedrill. Revert: restore + docker restart vm-viewer.
- First viewer restart hit transient DDS mask freeze (~10min, stale); second restart cleared it. Pre-restart ros-thread traceback unrelated (old log).

## ~18:05 UTC drill live_model done; tracker burstiness is the limiter
- live_model works (center follows tracker, err ~8-9px vs static 78px). Static grasp fields frozen by design.
- Root cause of flicker: sam2_tracker_node publishes in bursts (mask gaps to 6.4s, 3D gaps to 12s) -- pre-existing, untouched. RELIABLE-sub trial reverted (worse). QoS stays BE.
- Next if wanted: smooth tracker publish loop (grasp-lane code) or lengthen live gates (cosmetic).

## ~18:15 UTC grasp.json live-recenter (Murilo: cloud must follow drill)
- seg_view grasp.json now rigid-translates cloud/pick/tcp/centroid/gripper by (live_hold - grasp_center) when hold fresh; forensics recoverable via live.offset_m; old ?run= untouched. Verified: pick follows, offset stable 1mm.
- grasp_target.json (pick marker source) deliberately NOT touched: grasp targeting safety, needs explicit go.

## ~18:25 UTC seg starvation root cause: stale mask_sync ROI
- Live drill mask bbox_1000=[316,728,483,997]; mask_sync ROI=[513,565,737,1000] (yesterday's spot) -> ZERO overlap -> /vm/target_mask empty -> seg never sees drill. Raw mask fine (TRACKING 2.4Hz).
- Proposed: restart vm-mask-sync --roi 260,680,540,1000 (live + margin). Needs Murilo go (shared seg map).
- Grasp safe: S1 re-seeds live per run (operator bbox), S2 gates coverage+centroid (fails off-target), S3 captures fresh; stale model + my JSON never feed grasp.

## ~18:35 UTC dashboard round-trip disarm (fixed by re-arm)
- joystick->ZED->joystick via dashboard left bridge disarmed (sent frozen 105279). Cause: fr3_web_dashboard.py _control_zed disengage/home with control_backend==curobo sends enable-False; nothing re-arms (one-way door). Not my bridge/lane.
- Re-armed by my hand; teleop joystick+disengaged, JTC active. Fix needs Hipolito file (mirror enable-True on return, gated disengaged, or Arm button).

## 19:0x-19:20 UTC: HOME + mask transport rabbit hole + patch live
- HOME (Murilo direct order): arm at Paiva home <1° ([0.004,-0.776,-0.009,-2.364,-0.008,1.576,0.771]), mode 2, no reflex. fr3_home_jtc.py + fr3_home_safe.py BOTH require cartesian_impedance_target_controller (not loaded on JTC stack) -> sent single JTC topic goal directly (4s, quintic) after their path model cleared it (+17.7cm table, +8.4cm self). Bridge disarmed during move.
- MPC: restarted (docker restart, first TRUE reseed today - container has NO ros2/python3 on PATH; all earlier "ros2 service call" reseeds there failed silently). Activated via /isaac-sim/python.sh inline rclpy client (domain 142). Active + stepping at home.
- Auto-rearm patch (8a): files LIVE in curobo_live_noguards/ (bridge_rearm.py, mtime 19:02-19:03, deployed by lab-shell holder, not me). Bridge armed, sent=2017 (~40s post-home driving, zero faults/rejections). Auto-vs-manual re-arm indistinguishable from logs; next dashboard-source disarm + Start proves it.
- MASK: /vm/target_mask valid 22Hz at publish (same-container sub 5/5, 360x640x230400B) but cross-container DDS truncates ("sequence size exceeds remaining buffer", ~0.4Hz valid). TRIED: BE probe, RELIABLE probe, pub QoS RELIABLE->BEST_EFFORT (grasp_loop/vm_mask_sync.py: qp added, pub uses qp; subs stay RELIABLE), --ipc container:vm-nvblox-prod recreate, no-ROI test instance. NONE fixed. Depth (RELIABLE, 4x bigger) perfect 22Hz. New topics (even tiny String) invisible cross-container 45s+ -> discovery plane also degraded. PARKED: likely fix = TCP relay (capture same-container, re-publish inside consumer) or DDS transport config. Not blocking teleop.
- Correction: mask_sync ROI was INNOCENT (no-ROI instance fails identically). BE QoS change kept (matches all 4 BE subs).

## 19:35-19:50 UTC: Murilo-ordered restart (tracker + both nvblox + mask-sync)
- All four restarted clean. Tracker re-acquired WITHOUT manual re-seed (mask_sync: sam2 208, published mask 204/empty 3). No re-seed needed.
- Delivery STILL broken post-restart: depth ~17Hz valid (2 transient errors at match), mask ~0 valid in 25s (hz never even matched: "does not appear to be published"). Restart-as-fix DISPROVEN.
- Fanin is ALSO python+rclpy+RELIABLE (cable_fanin.py) and works -> language and reliability-protocol both eliminated. Remaining difference is writer/node-specific, mechanism unknown.
- MPC: fresh boot at home + active + stepping (Step 42060 OK at 19:35). Bridge disarmed. Stack safe.
- NOTE for teleop: nvblox drill carve is stale (mask not delivered) -> static map holds drill at OLD pose (phantom obstacle) and misses it at new pose. Avoidance otherwise live.

## 19:50 UTC: ONE-map enforced (Murilo direct order)
- docker stop vm-nvblox-seg. Only nvblox_curobo remains. Revert: docker start vm-nvblox-seg.
- Prod verified: "Successfully wrote requested ESDF" (MPC consuming cross-host), tracker flowing (sam2 2450).
- Grasp re-point NOT done (needs grasp lane validation): grasp_loop.py:38-39 esdf_seg/mesh_seg -> /nvblox_curobo/... ; run_vm_nvblox.sh documents SEG=1 variant.

## 20:00-20:20 UTC: tracker empty -> reprompt saga (Murilo: "did you do a reprompt?")
- Tracker output 78/78 EMPTY (restart wiped seed and/or drill moved) -> reprompt needed. Murilo right.
- Remote seed attempts: (330,315) then (300,302) via /tracking/seed_pixel bursts -> both yielded 23% TABLE masks, not drill. Scene is MOVING (tag gone between caps, drill shifted) -> seeds likely missed a moving drill, or SAM2 single-point prefers table.
- Seed interfaces found: /tracking/seed_pixel (PointStamped, single pt, re-inits video predictor), /perception/seed_command (JSON bbox_1000 -> SAM multimask point prompts at box center).
- Reverted my BE publisher experiment (broke RELIABLE-reader matching); mask_sync pub back to RELIABLE.
- Overlay proves viewer delivery works when masks non-empty (age 1.7s) -> transport re-evaluation pending after good seed.
- ASKED Murilo: click seed on live dashboard video himself, or hold drill still 60s.

## 20:20-20:45 UTC: reprompt saga RESOLVED
- Root causes (3 stacked): (1) tracker restart wiped seed -> 78/78 empty; (2) warm re-prompt preserves table memory -> need COLD (seed_command retrigger + seed_pixel); (3) cable feed flap (my many rgb/depth probes collapsed it!) -> seeds expired (no fresh frames). Fanin restart restored 24.7Hz.
- Fix: burst seed_command (x13) + seed_pixel burst at last drill fix (249,335) -> "Video predictor initialized score=1.000". Mask now 30/30 valid, centroid (285,318), fill 1.96% = DRILL.
- LESSON: NEVER attach ad-hoc readers to /vm/cable/* (collapses the feed); verify via logs + mask topics only.

## 20:45-21:05 UTC: yaw-follow fix deployed (Murilo: cloud orientation wrong)
- seg_view.py: live mask PCA vs projected model-axis PCA -> yaw stored in _live_hold; _live_shift_grasp rotates cloud/pick/tcp/centroid/gripper-tris about model center by yaw then translates; drill box AABB recomputed from rotated corners. Guards: >=100 mask px, eccentricity>2, try/except fallback to translation. Yaw exposed as live_model.yaw_deg + live.yaw_deg.
- Live: yaw_deg 41.6 (vs earlier offline estimate ~36), status live (err 39.5px, borderline). Viewer restarted.
- Map restarted with live masks (clean static rebuild).

## 21:05-21:20 UTC: carve proven, yaw live
- ESDF at drill center = -1000 (carved hole), table beside = +0.075 (observed surface), air above = -1000 (masked rays integrate nothing). Mask IS reaching nvblox and carving. Static map clean after restart.
- Lesson reinforced: ad-hoc probe nodes don't match reliably (discovery degraded for NEW readers); long-matched consumers (nvblox/mask_sync/tracker) work. Trust consumer behavior over probes.
- drill.json live (yaw 41.6), grasp.json live yaw 41.4. Box AABB rotated corners.

## 21:20-21:40 UTC: lab feed collapse (Murilo: dashboard worse, leak, 2nd nvblox?)
- Verified ONE nvblox (prod only). drill.json stale (mask_age 24.5s), mask_sync empties 29% -> nvblox leak.
- Root cause: LAB PC load 75 (18 users, Chrome gpu-process 320%, chronic per 10-29 notes). Lab publishes 13-14Hz locally; VM fanin gets 0.1-0.7Hz. VM idle (load 2.8), buffers already 8MB, Tailscale direct. Cross-host DDS send starves under lab load.
- quest-fr3-map-sim (started ~20min before death) innocent: 78% CPU, wrist teleop test holding at leash.
- sysctl rmem_max 16->32MB, default 8->4MB (left; harmless). Did NOT fix (buffers weren't the cause).
- Waiting on Murilo/Hipolito for lab load relief. Recovery plan: feed back -> tracker resumes (seed intact) -> masks flow -> restart nvblox once (clean leak) -> green.

## 21:40 UTC: HANDOFF to bypass lane (Murilo direct go)
- Released franka-robot + lab-teleop-bridge-8784 + lab-mpc-test (all FREE).
- Handoff state: arm NOT home ([0.598,-0.311,-0.704,-2.340,-0.293,2.083,0.937], Murilo drove post-home), mode 2, JTC active; bridge ARMED waiting Start, sent 7825, no fault; MPC active hybrid; map single, feed 0.3Hz; tracker seeded, starved. This lane holds NOTHING and commands nothing further.

## 21:45 UTC: feed recovered (Murilo killed Hipolito's Chrome)
- Lab load 75 -> 51 and falling. Fanin back to 24.8Hz x3. Tracker kept drill through outage (followed to (356,299), fill 1.76%).
- Restarted nvblox-prod (clean leaked static). drill.json live, yaw 43.5.

## 22:00 UTC: Murilo screenshot triage (streak + blob)
- Yaw sign VERIFIED correct: projected shifted-cloud axis 156.4 vs live mask 151.4 (err 5.1); flipped would be 84.5 off. Live cloud orientation right.
- Planner map VERIFIED clean: drill free (+0.2), old spots unobserved (no ghosts), table surface present. Mesh X-Map-Age 0.0 (fresh). No leak in ESDF.
- Screenshot streak = mesh built on drill depth silhouette (bleed + holes -> garbage geometry + cyan colors). Known ZED artifact (silhouette bleed, in _target_model docstring). Gray = table/wall (correct). The :8088 page does NOT render the live grasp cloud (only mesh + robot) -> Murilo never sees the correct cloud there.
- Next if streak bothers: erode mask in mask_sync (cut boundary bleed). Offered.

## 21:05-21:20 UTC: Murilo screenshot = real leak + wrong pose (he was right; I argued wrongly)
- LEAK cause (mine): vm-mask-sync --roi 260,680,540,1000 cut the drill mask at x=361px; live drill spans x 314-412 -> right half integrated as static. Also my --max-age 4.0 applied stale masks to fresh depth.
  Fix: vm_mask_sync.py new --dilate N (cv2.dilate ellipse). Running: --max-age 1.5 --dilate 12, NO roi. Map restarted 21:11 (bypass lane cleared it: no motion source).
- POSE cause: grasp.json cloud was yesterday's drill cloud rigidly moved+yawed; drill now lies in another pose -> can never match.
  Fix: seg_view.py _live_cloud(): live mask (eroded 2px) x nearest-stamp /vm/cable/depth (depth band z10-2cm..+20cm, 20cm radius outlier cut) -> fr3_link0, <=4000 pts. grasp.json 'cloud' = live cloud when fresh (<6s; live.cloud='live_mask_depth'); drill.json box = 2-98 pct AABB of live cloud (box_source='live_cloud'). pick/tcp/gripper still yesterday's rigid shift (approximate). Verified: 2514 live pts served. tests 6/6.
- I WAS WRONG earlier: the :8788 page DOES render grasp.json cloud + drill.json box (via mesh_server /graspjson /drilljson). Also my "erode the mask" offer was backwards: carving needs DILATION.
- STILL OPEN: feed flaps 0<->24Hz every ~10s (chronic, lab sender DDS buffers). Fanin restart did not cure. Lab raw depth has extra readers: zed_live_depth_health (BE) + lab 'mask_sync' (RELIABLE, the dead old one) -> prime suspect. Fix is lab-side, needs Murilo go.
