# START HERE — state at 2026-11-04 ~21:20 UTC (lane "ZED2/teleop", session e64db8e7)

Full history: `HANDOFF_2026-11-04_hybrid-esdf.md` (long, chronological). This file = current truth.

## Ownership (updated ~23:05 UTC)
- **Bridge RETIRED** by 8a lane: noguards hardware bridge archived to lab `fr3_zed_runtime/retired_1104_direct/`, process dead, :8784 down — do NOT restart it.
- **Only live control path:** MPC `codex-fr3-live-mpc` (domain 77) → engaged-gated topic forwarder → `fr3_arm_controller` (JTC). Engage via the teleop as usual (direct_mpc gate = JTC active + fresh preview; no bridge status). Forwarder silent while disengaged; Start drives. A/B numbers in the retired notes.
- Claims released by 8a; board free. I hold none.
- Before any vm-nvblox-prod restart: whoever drives must confirm the arm can't move (empty map = MPC sees free space).

## Ownership (historical, superseded)
- I hold NO claims. Robot handed to bypass lane `8a38210dc6d6415b998fa97f237f712f` ("Controle direto cuRobo Franka") at ~21:40 on Murilo's direct go, for his A/B (direct MPC JointTrajectory vs bridge). They relaunch `codex-fr3-live-mpc` and drive; don't touch robot/bridge/MPC.
- Before any vm-nvblox-prod restart: ask 8a lane if the arm can move (empty map = MPC sees free space).

## Decided / done today (verified)
- MPC `feedback_mode` hybrid is default: repo `fr3_real_repro/vm_real/node/curobo_mpc_node.py:93`, lab `hub-curobo/node/curobo_mpc_node.py:101` (backup `.bak_hybriddefault_1104`). Hybrid fixed the teleop lag (Murilo confirmed).
- Guards-down bridge lives in lab `/lab/fr3_zed_runtime/curobo_live_noguards/` (shadow copy; Hipolito's originals untouched). 8a lane added auto-rearm + dashboard "Recover arm + bridge" button.
- ONE map: `vm-nvblox-seg` stopped (revert `docker start vm-nvblox-seg`). Grasp still points at it: `grasp_loop/grasp_loop.py:38-39` → repoint to `/nvblox_curobo/...` (untested).
- vm-mask-sync (VM): `python3 /w/vm_mask_sync.py --depth /vm/cable/depth --max-age 1.5 --dilate 12` (NO --roi; `--dilate` is new; run with `--ipc container:vm-nvblox-prod`). The old ROI cut the drill → leak. Publisher QoS RELIABLE.
- seg_view.py (VM `vm-viewer`, :8791): `_live_cloud()` = live mask × nearest depth → fr3_link0 cloud; grasp.json `cloud` = live (live.cloud='live_mask_depth'); drill.json box from live cloud (box_source='live_cloud'); yaw-follow for pick/gripper (approximate). The :8788 dashboard DOES render these (via lab mesh_server /graspjson /drilljson). tests `vm_offload/tests/test_drilljson_live3d.py` 6/6.
- Tracker seeded on drill. Reseed recipe: `teleop_run_1104_mapon/burstseed.py` (seed_command bbox burst + /tracking/seed_pixel burst) run in vm-viewer, ONLY when feed is up (seed expires otherwise). Find drill px from depth: `teleop_run_1104_mapon/drillfind.py`.

## OPEN (ranked)
1. ~~Camera feed flap~~ FIXED 21:27 (see bottom). Was: **Camera feed lab→VM flaps 0↔24Hz every ~10s** (vm-cable-fanin log). Not VM (idle, fanin restart no help). Lab load 36–75 (18 users). Killing Hipolito's Chrome (320% CPU) helped once. Prime suspect, unverified: lab RELIABLE subscriber `mask_sync` (old dead lab mask_sync) + `zed_live_depth_health` on `/franka_seg_zed1/depth`. Asked Murilo for go to stop the old lab mask_sync. Everything downstream (tracker, map, box) starves when the feed flaps.
2. Map rebuild after 21:11 restart is partial (feed flap).
3. ZED2 not validated/integrated: needs 2nd tag spot or arm poses (Murilo offered to move tag). Candidate `fr3_real_repro/zed2_align/zed2_extrinsic_20261104_planetag_CANDIDATE.json`.
4. Dashboard `_control_zed` disengage/home sends bridge enable-off (Hipolito file `fr3_web_dashboard.py` ~l.223); 8a lane's auto-rearm not yet observed firing.
5. Tracker clear-box warns `"world" frame missing` (target_parent_frame param) — harmless to mask, breaks clear-box.

## Rules learned the hard way
- Don't argue with Murilo's eyes: point samples of ESDF don't prove "no leak"; check the mask extent vs crop.
- Never attach ad-hoc readers to RAW cable topics (`/zed1/.../compressed`, `/franka_seg_zed1/depth`); `/vm/cable/*` is the VM-local bus (OK).
- New ad-hoc probe nodes often fail DDS discovery here; trust long-lived consumers' logs.
- `pkill -f` self-matches: use `[x]` bracket patterns. Lab containers lack `ros2` in some (MPC: use `/isaac-sim/python.sh` rclpy).
- Access: VM = this host (10.0.0.3 / 100.84.207.115). Lab: `ssh -i ~/.ssh/id_ed25519_lab_claude murilo@100.88.134.127`. Bridge status: lab `curl 127.0.0.1:8784/status`.

## 21:17–21:30 UTC feed-flap test (Murilo go: "desliga o antigo e testa")
- STOPPED old lab mask_sync (`python3 -u /tmp/nvblox_real/mask_sync.py` in container `isaac-build`, up 1d19h). Flap UNCHANGED → not the cause. Left stopped (it was dead: VM vm-mask-sync replaces it). Revert: `docker exec -d isaac-build python3 -u /tmp/nvblox_real/mask_sync.py` (domain env unknown).
- Dead-code cleanup NOT done (condition "if it works" failed).
- Proven: lab segmenter `franka-seg-zed1` stats = steady 29.5 Hz in/out the whole time, incl. VM 0 Hz windows → loss is in transit, not at source. VM NIC/UDP counters clean.
- Found: lab `web_video_server` (:8787, container `zed-build`, `ros2 run web_video_server ...`, domain 8, up 20h) drops ~38–41k UDP datagrams/s on its DDS socket (port 9426, /proc/net/udp drops) = ~all of lab UdpRcvbufErrors; ~40 dangling ESTABLISHED HTTP conns on :8787. Constant, not time-correlated with the flap → contributor (RELIABLE NACK storm) suspected, unproven. Restart needs go (not mine).

## 21:27 UTC FEED FLAP FIXED (Murilo go)
- Restarted lab `web_video_server` in `zed-build`: `docker exec -d zed-build bash -c "source /opt/ros/humble/setup.bash; export ROS_DOMAIN_ID=8; exec ros2 run web_video_server web_video_server --ros-args -p port:=8787 > /tmp/web_video_server_1104.log 2>&1"`. It had ~40 dangling HTTP streams and was dropping ~40k UDP pkts/s.
- Result: VM fanin steady 24–25.6 Hz, 0 dropouts over 3+ min (before: to 0 Hz every 10–60 s). Lab UdpRcvbufErrors 40k/s → ~300/s.
- If the flap returns: check lab per-socket drops (`/proc/net/udp` last column, map inode → process via docker exec) — likely web_video_server accumulating dead streams again.
- Dead-code cleanup (Murilo go): stopped old mask_sync (isaac-build), a `ros2 topic hz` probe hung 7 days (zed-build), and the dead `mesh_server --node nvblox_fixed` on :8789 (0 clients, not in dashboard). :8788/:8795 untouched, still 200.
- Kept on purpose: `depth_health.py` (fr3-curobo; feeds the bridge freshness gate via UDP 47929), seg_overlay/ov2.py (dashboard overlay), files in /tmp/nvblox_real (the live dashboard dir).

## ~22:35 UTC drill projection "floating / translated" (Murilo, dashboard :8788) — FIXED server-side
- Cause 1 (dashboard JS, lab /tmp/nvblox_real/lab_dashboard.html ~l.618 applyFollow): moves the cloud by (live_3d world − drill.json center) — meant for yesterday's static cloud. The cloud is now live in fr3_link0 → double shift, measured (−8.7, −0.7, +3.5) cm = floating. Also build() only on a new `run` → cloud froze at page load.
- Cause 2: pick/tcp/gripper anchored on the 2D live_model center (~9 cm off the live cloud in x).
- Fix (VM only, seg_view.py grasp.json): when live cloud fresh, `run` = `<run>@live<wall*10>` (dashboard rebuilds every poll; follow disabled since run != drill.json source_run); pick/gripper anchored via `_live_shift_grasp(anchor=)` on live cloud box xy center + top-surface z. Verified via :8788/graspjson; pick x 0.36 → 0.449 (inside cloud 0.43–0.59). Lab HTML untouched. Reload the page once.
- ~22:50 UTC still floating on :8443 (= tailscale serve → ssh tunnel 127.0.0.1:18089 → lab :8788, same page). Measured: map table z≈0.00 under the drill; live cloud p2 z=0.07. Cause: ZED1 sees the drill at the image's BOTTOM EDGE — base/battery out of frame (snap.jpg). Fix: seg_view `_support_z()` = table height from a 7–30 px depth ring around the mask (same frame/TF); drill box aabb_min z extended to it (drill.json `support_z`, ~0.012). Cloud itself unchanged (only what the camera sees). Backup seg_view.py.bak_support_1104. Real fix for the cloud = camera view that sees the base (ZED2 or move ZED1/drill).
- 22:50:32 UTC (Murilo): seg→nvblox path CUT so the drill integrates as an obstacle. vm-mask-sync now `--sam2 /vm/seg_disabled_none` → publishes EMPTY masks (727/30 s ≈ 24 Hz); tracker + nvblox untouched/alive. REVERT: recreate vm-mask-sync with `--sam2 /zed1/segmentation_mask` (default; drop the --sam2 arg), same docker run as above.
- 22:59:30 UTC (Murilo, tested): seg→nvblox path RESTORED (vm-mask-sync default --sam2; 658 masks / 30 s). Drill voxels already integrated clear again as the carve re-applies.
- ~23:10 UTC "floating again" after carve restore (map no longer drew the drill under the cloud). Real cause: SAM2 mask covered only the RED body; black/yellow battery unmasked (also not carved → map blob). Reprompted: `burstseed.py 350 315 480 730 660 1000` (bbox to image bottom). Mask 1.8%→2.3%, cloud p2 z 0.07→0.01 (table ≈0). Also: box flashed to the 2D-model fallback (~10 cm off) on ticks where the live cloud missed → now keeps last live-cloud box ≤3 s. Verified 10/10 polls box_source=live_cloud, z −0.01..0.235.

## ~23:30 UTC ZED2 calibration re-check (Murilo: "quero usar a ZED2")
- Fresh capture `fr3_real_repro/zed2_align/cap_1104c/` (8 s; ZED2 compressed ~22 Hz over cable, ZED1 fan-in stayed 24.6–25.7 Hz → transport OK since the web_video_server fix).
- `eval_icp_1104c.py` (out-of-sample vs cap_1104b fit): deployed tagreg table tilt 9.2°, nn 21.5 mm, obj 32.9 mm (WRONG); ALT 10.0°/19.3/33.0; **planetag CANDIDATE 2.4°, 10.7 mm, obj 16.3 mm, table dz −3.8 mm** (best). ICP from any start improves table but worsens objects (slides in-plane) → rejected.
- `drillcheck_1104c.py`: planetag drill NN med 14.9 mm (p75 19.9), top z 0.220 vs ZED1 0.235; deployed 37.4 mm.
- Proposed (awaiting Murilo go): A) lab zed-build ZED2 TF → planetag; B) VM fan-in of /franka_seg_zed2/depth; C) ZED2 drill points in seg_view (3D box crop); D) nvblox camera_1 + map restart (arm idle).
- **A done (~23:1x UTC lab clock)**: lab `zed-build` ZED2 TF publisher swapped tagreg → `recovered_1102/zed2_extrinsic_20261104_planetag.json` (copied in via docker cp; md5 f6b8949a). Optical [0.657,−0.010,1.024]. Log `/tmp/nvblox_real/extrinsics_zed2_planetag_1104.log`. Revert: kill it, relaunch with `zed2_extrinsic_20261104_tagreg.json` (cmd in zed2_align/DEPLOY_20261104.md).
- **B done**: `vm-cable-fanin2` (vm_offload/run_vm_fanin2.sh, `FANIN2_DEPTH=compressed`) → /vm/cable2/{rgb,depth}, ZED2 ~10.4 Hz, ZED1 stays ~25 Hz. RAW `/franka_seg_zed2/depth` (FANIN2_DEPTH=seg) COLLAPSED BOTH feeds to 0 Hz within ~60 s (two 0.9 MB raw streams = old sender-buffer limit) → never run seg mode without a lab-side DDS buffer change. Compressed depth is NOT robot-masked.
- **C done**: seg_view `_zed2_points()` adds ZED2 depth cropped to the ZED1 drill box (+4 cm xy/+4 cm top, >1 cm above table) — grasp.json live.cloud=`live_mask_depth+zed2_box`, n_zed2 ~2.4–2.9k, zed2_T=tf. Also depth deque 15→60 (SAM2 masks 0.4–1 s late caused most live-cloud misses) and box hold 3→6 s: 20/20 polls live_cloud.
- **D pending**: ZED2 into nvblox needs robot-masked ZED2 depth (compressed one includes the arm) + map restart with arm idle. Options: lab-side compressedDepth republish of /franka_seg_zed2/depth, or VM-side robot masking.

## 23:46–23:52 UTC CABLE FIX (Murilo go) — two raw ZED depth streams now coexist
- Jumbo frames: pve enp1s0/tap103i1/vmbr1 → 9000 (runtime, via ssh root@10.181.64.131; Murilo added 10.181.64.193 + 10.0.0.3 to the key's from=), lab enp1s0 9000 (Murilo, sudo, runtime), VM enp6s19 9000 (persisted: nmcli cable-lab mtu 9000). 8972-byte DF ping OK.
- Jumbo alone made it WORSE (reassembly fails 37% → 90%, NIC drops 0). Real cause: VM kernel IP reassembly limits — ipfrag_max_dist=64 drops datagrams when two streams interleave fragments, and 4 MB ipfrag_high_thresh. Set high/low 64/48 MB, max_dist 0 → fails 2 / 153k. Persisted `/etc/sysctl.d/90-cable-reasm.conf` on the VM. (Explains the historical ~50 % IpReasmFails / feed flaps.)
- Then `FANIN2_DEPTH=seg` (raw robot-masked /franka_seg_zed2/depth) for 3 min: ZED1 24.2–25.4 Hz, ZED2 22.4–23.6 Hz, reasm fails 37 / 248k. vm-cable-fanin2 now runs in seg mode.
- NOT persisted yet: pve (/etc/network/interfaces `mtu 9000` on enp1s0 + vmbr1; 103.conf net1 `,mtu=1`) and lab (`sudo nmcli con mod Franka 802-3-ethernet.mtu 9000`). A reboot of pve or lab reverts to 1500 → mismatch with VM 9000: then set VM back (`sudo nmcli con mod cable-lab 802-3-ethernet.mtu 1500; sudo ip link set enp6s19 mtu 1500`) or persist the others.
- Step D (ZED2 into nvblox) now unblocked: robot-masked ZED2 depth is on /vm/cable2/depth. Needs map restart with the arm idle.
- ~00:00 UTC persisted pve: /etc/network/interfaces `mtu 9000` on enp1s0 + vmbr1 (backup interfaces.bak_claude_mtu_20261104; `ifreload -a -n` clean; not reloaded — live already 9000). 103.conf net1 NOT changed (live NIC edit can hot-replug; PVE tap_plug takes the bridge MTU on VM start). Lab persistence still pending: `sudo nmcli con mod Franka 802-3-ethernet.mtu 9000` (Murilo).
- Lab persisted (Hipolito ran nmcli): cable profile 1006e4bb (enp1s0) mtu=9000 ✔; robot-link profile da220b7b (enp0s31f6, also named 'Franka') untouched (mtu auto, live 1500). Always reference the cable profile by UUID 1006e4bb-894c-4afb-9a0b-7886fd4afa7a. MTU 9000 now persisted on all three hops.

## ~00:05 UTC 11-05 Murilo: drill cloud DOUBLED with planetag ZED2 (he was right)
- Rolled back: drill cloud ZED2 off (flag file `fr3_real_repro/vm_offload/ZED2_OFF`; rm it to re-enable, no restart); map back to ZED1-only (original container restarted as vm-nvblox-prod; 2-cam one kept stopped as `vm-nvblox-prod-2cam`; vm-mask-sync2 stopped). vm-cable-fanin2 still running (seg mode, harmless).
- Measured (capture_live.py → cap_live_1105a/b, robot-masked VM-local depth, 8-frame median): planetag drill-TOP offset ZED2−ZED1 = (+47, +30, −16) mm (~5.8 cm) although NN medians looked fine (12–16 mm) — NN hides offsets. 3-DOF in-plane refit (refine3dof.py: yaw −7.9°, +20/+70 mm) only reduces it to (26, 30, −16) mm → not a pure in-plane error; suspects: ZED2 depth scale, visibility bias of the metric, or the ZED1 chain itself (old unexplained 13 px ZED1 shift).
- Proposed: calibrate BOTH cameras against the robot (FK meshes from mesh_server /config as ground truth), ideally 4–6 arm poses moved by Murilo.

## ~00:25 UTC 11-05 robot-based calibration (Murilo go "pode rodar")
- Claimed franka-robot + lab-zed-cameras 00:12–00:20Z. Arm had a cartesian_reflex at ~00:02Z (before my window; 8a says not their chain) → I ran /action_server/error_recovery, FrankaHardwareInterface active, fr3_arm_controller ACTIVE (left active, 8a informed + re-verified their chain).
- 6 poses (home + P1–P5, FK-checked ≥0.2 m from map obstacles) via `calib_robot/move_pose.py` (JTC topic, ≤0.12 rad/s); capture `calib_robot/capture_pose.py` (compressedDepth WITH robot, header-quant decode = raw within 0.5 mm, + dashboard SSE link poses). Data `calib_robot/run_1105/`. Fit `calib_robot/fit_robot.py` (mesh ICP, seconds).
- **ZED1 is off 3.49° / 46.9 mm** vs the robot: arm residual 6.3–9.0 → 4.0–5.8 mm, LOO stable (0.2–1.8°). Candidate `zed2_align/zed1_extrinsic_20261105_robotfit_CANDIDATE.json` (NOT deployed; deploying moves the whole map relative to the robot → Murilo go + map restart).
- ZED2 sees almost none of the arm in these poses (100–570 pts) → its robot-fit is unstable, discarded. With robot-fit ZED1 + planetag ZED2 the drill-top offset drops (47,30,−16) → (18,5,−19) mm; in-plane realign didn't help. ZED2 needs its own robot poses with the hand lowered into its view.

## ~00:30–01:00 UTC 11-05: ZED1 deployed + ZED2 robot session + scale finding
- DEPLOYED ZED1 robot-fit (6 poses) at ~00:27Z: lab zed-build TF publisher on `recovered_1102/zed1_extrinsic_20261105_robotfit.json` (md5 4d502b91; optical [0.284,−0.585,0.632]); old file `recovered_1102/zed1_extrinsic.json` untouched = revert. vm-nvblox-prod restarted 00:27:42Z. Repo `zed2_align/live_zed1_extrinsic.json` = robotfit (old kept as `live_zed1_extrinsic_pre1105.json`).
- web_video_server SEGFAULTED ~00:15Z (on stream removal) → relaunched in a `while true` restart loop in zed-build (log /tmp/web_video_server_1104.log).
- 2nd robot session 00:33–00:41Z (claims released, all lanes notified): 4 poses Z1–Z4 with the hand over the table in ZED2's view (FK + path checked; mover now has a foreign-publisher guard after the 00:17Z goal fight: Hipolito's pad was engaged during my 1st window → 00:17:49 cartesian_reflex). Data `calib_robot/run_1105/PZ*`.
- 10-pose fits (`fit_robot.py`, `fit_robot7.py`): ZED1 6DOF moves 1.8°/17 mm from the deployed 6-pose fit (LOO ≤0.23°); ZED2 6DOF arm resid 2–3 mm. 7DOF depth scale: ZED1 0.977 (very stable), ZED2 0.996 (poorly constrained).
- Tag id16 check (`calib_robot/tagcheck.py`, depth-plane corners, size-free): deployed 6-pose ZED1 is the WORST vs any ZED2 (32–43 mm); 7DOF both → 19 mm (best). Tag depth edges: ZED1 ~139 mm, ZED2 ~135 mm (true 138). Arm and tag agree the cameras differ in depth SCALE by ~2–2.5 % relative (ZED1 reads longer than ZED2), disagree on the absolute split. That explains the remaining ~2 cm doubling.
- ZED2 still OFF in drill cloud + map. Decision pending (Murilo).

## ~01:10–01:20 UTC 11-05: dual-camera drill cloud + new grasp (Murilo: "ficou melhor", "gera um grasp novo")
- Viewer-only calib override: `zed2_align/calib_robot/viewer_calib.json` (7DOF robot-fit, ZED1 scale 0.977, ZED2 0.996) used by seg_view for the drill cloud instead of the lab TF (delete file → TF). ZED2 back ON in the cloud (ZED2_OFF removed). Map/TF unchanged (map = ZED1-only on the 6-pose TF).
- seg_view fixes: ZED2 crop floor table+1 cm → +2.5 cm (25 % of the merged cloud was table slab → first GraspGen pick landed off the drill); follow-yaw now relative to the first yaw seen per run (`_yaw0`), was rotating a freshly generated run by 13.6°.
- New grasp run `grasp_loop/runs/20261105_011430_drill_dual` (latest → dashboard serves it): 4 live merged snapshots, 2.5 mm dedupe, table removed, outlier filter (≥8 nbrs/12 mm) → 5944 pts; GraspGen-X on lab GPU0 (lab-gpu0 claimed+released) 100 grasps; S4 filters/ranker of grasp_loop.py → pick idx 39 pos (0.393,−0.325,0.317), conf 0.75, score 0.61, vert 0.92, bilateral 0.82 (top-down on the rear of the drill body). Earlier bad run (with table slab) left as `20261105_011305_drill_dual` (has pick.json but older id).
- NOT executed on the robot.

## ~01:20–01:35 UTC 11-05: calibration DEPLOYED (both cams) + 2-cam map + staged grasp runner
- Lab zed-build TF publishers: `recovered_1102/zed{1,2}_extrinsic_20261105_robotfit7.json` (7DOF pose; optical zed1 [0.260,−0.573,0.633], zed2 [0.661,0.007,1.036]). Revert = relaunch with zed1_extrinsic_20261105_robotfit.json / zed2_extrinsic_20261104_planetag.json (or the originals).
- Depth scale applied in VM fan-ins (`DEPTH_SCALE`, run_vm_fanin.sh default 0.9767, run_vm_fanin2.sh 0.9959 + FANIN2_DEPTH=seg). Fix: Image.data must be `array('B', …)` (bytes → rclpy per-byte check → fan-ins CPU-bound at 11 Hz). Now 22–24 Hz both.
- viewer_calib.json retired (`.retired_deployed_1105`) — TF + scaled depth replace it (would double-scale).
- vm-nvblox-prod = 2-camera container (ZED1 camera_0 + ZED2 camera_1, mask2 = vm-mask-sync2 drill-box projection). ZED1-only kept stopped as `vm-nvblox-prod-1cam`.
- RULE: after any fan-in restart, `docker restart vm-viewer` (it lost the depth subscription: "live cloud miss … depth buf 0"; live cloud stale → grasp fell back to the 2D model, 2.5 cm off). seg_view now logs live-cloud misses.
- Table: cams agree (A/B regions 26–32 mm) but ZED1 near-field warps ~2 cm low → map table ~2 cm thick near ZED1. Both robot-fits put the table at ~+2.8 cm vs base (old calib 1.3 cm): possible common vertical bias — unresolved (is the base on the same tabletop?).
- Staged real grasp runner: `grasp_loop/grasp_test.sh plan|run [Z_OFFSET]` (VM) → grasp_plan.py (IK on FR3 DH, FK verified vs measured hand to 0.001°; GraspGen pose @ Rz(+90); straight-line descents; map/table/drill checks) + grasp_stage_exec.py (lab fr3-ros2; foreign-publisher guard; start-pose check) + grasp_gripper.sh. ENTER before every stage. Plan PASS (≥0.39 m obstacles, ≥0.25 m table clearance of links).
- franka-robot claimed for Murilo's test ~01:35Z; 8a parked.
