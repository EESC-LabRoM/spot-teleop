# Wrist teleop continuation — 2026-11-19 17:59 UTC

## Current task and decisions
Prepare standing hand teleop; show drill, tennis ball, Sprite bottle and Phillips screwdriver segmentation so the operator can inspect map leakage. Latest work is a **read-only inspector**, not a robot test. Keep one map: tower `isaac-build:/nvblox_curobo`; `vm-nvblox-prod` stays STOPPED. No TF/map/driver changes during the other lane’s calibration. Four-object masks are snapshot diagnostics, not deployed tracking or grasp targets.

## Running / approval / ownership
- Inspector: http://127.0.0.1:47814/ (tower only), PID **2742934**, verified 17:59 UTC. Command: `python3 server.py --bind 127.0.0.1 --ttl 1500`; started ~17:52, expires ~18:17 UTC. Verify PID/cmdline before stopping or restarting.
- Runtime: `/home/hipolito/franka_teleop_checks/object_inspector_1119`; `server.log`, `server.pid`, local JPEG/PNG evidence there. Do not commit camera pixels.
- Automatic approval review rejected exposing images/map on Tailscale without address-specific approval. Async question is PENDING for **http://100.88.134.127:47814**. Do not expose/tunnel until answered. If approved, restart with both `--bind 127.0.0.1 --bind 100.88.134.127`.
- Held resources: `hipolito-precision-5860-tower`, `lab-teleop-test-port-47814`, `omnigent-vm` (renewed ~17:55 for 30 min). Stop inspector/poller before releasing: it polls cached VM stats. No robot/camera/GPU/MPC claims from this lane.
- Robot last reported Home, JTC INACTIVE; no own hardware writer. Standing perception and candidate MPC STOPPED. New motion needs queue + fresh physical-state/e-stop confirmation; current task does not require motion.

## Done / measured
- Inspector toggles four masks, original RGB, live canonical overlay, freshness/exclusion audit, existing map iframe. GET-only; no ROS/control imports. Seven audit tests pass, JS parses, endpoints return 200, Chrome screenshot visually checked.
- Installed VM SAM2 Tiny: four manual boxes, 640×360 snapshot; load **0.627 s**, inference **0.952 s**, peak CUDA **483 MB**. Mask pixels: drill 2372, ball 332, Sprite 1066, screwdriver 1053. All visually sensible; not a video-FPS or multi-object tracking benchmark.
- **Current live target mask is EMPTY (0%).** Filter heartbeat is fresh but last accepted source stamp **1795041797.3732457 (~19 h old)**; refusal `empty or oversized target mask`. New integration paused; old map retained. Top-level drill.json `ok/live` is misleading. Inspector correctly says NOT VERIFIED. Historical zeroed-depth/zero-vertex results do not certify current stored mesh clean.
- Real bag tests (Nov18): z06 completed 958 commands, 3 following resets, TCP error p95 **75.1 mm**; z12 combined aborted our 1.2-rad joint-excursion guard (j4 +1.23 rad), 1137 commands, 7 resets, p95 **77.4 mm**. No hardware fault; Home/passive cleanup. Movement quality is NOT solved. See `../wrist_body_calibration_1118/hardware_1118/RESULTS.md`.
- Standing setup was passive-tested: ~30 Hz, acquisition age 60–95 ms, 50/50 native MPC feasible, max joint delta 0.000371 rad. No physical standing follow yet. Fresh shoulder/elbow/wrist calibration required; do not reuse donor bag profile. See `../wrist_standing_1119/README.md` and `CURRENT_STATUS.md`.

## Repo / commands / identifiers
- Main `/home/hipolito/spot-teleop` is shared/dirty at `3d78fd9`: do not reset/stash/bulk-commit.
- Clean publishing worktree **`/tmp/wrist-publish-20261119`**, branch `wrist-publish-20261119`; latest source pushed **8af98d3**. Remote `https://github.com/EESC-LabRoM/spot-teleop.git`, destination `murilo-curobo-nvblox-hand-20261101`.
- Push scoped changes: `git push origin HEAD:refs/heads/murilo-curobo-nvblox-hand-20261101`. Commit author **MHC-CodeSmith <carvalho.matheush@gmail.com>**, no AI trailer. Standing authorization to commit/push teleop work.
- Teleop docs root: `isaac-ros_ws/src/spot_nvblox/fr3/teleop/`. Inspector method/limitations: [README](README.md). Body calibration commit `9c2b0e0`; real replay `3d78fd9`; transport tooling `a452cfc`; standing `0235e74`, `4004843`.
- Shell tool: discover `sys_os_shell` in ALL_TOOLS; `rg` unavailable. VM SSH: `ssh -o BatchMode=yes -o HostKeyAlias=omnigent-vm nexus@100.84.207.115 ...`.
- Inspector sources: local cached `127.0.0.1:8788/segsnap`, VM `10.0.0.3:8791/stats.json`, `isaac-build:/tmp/wrist_map_filter/status.json`; iframe existing `:8788/map`. No new ROS image subscribers.

## Coordination / next actions
- Calibration lane **c66ecc79ac5f4873b1cae5d2c5c617cb** (Murilo/morys5): 14-pose motion finished **14:53 local**, Home/passive, robot claim released. Offline fit running; camera claim held for ZED1/2 readers. No TF/map deploy without Murilo go + notice. Cached HTTP snapshots explicitly allowed. First P2 trigger produced no motion; retry worked, unexplained.
- Preserve ZED3 operator feed/tracker and viewer PNG relay. ZED1 33874668; ZED2 39498278 (filed TF wrong, new fit pending); ZED3 34046618 recovered ~30 Hz. Wrist D435i 138422074833 absent USB, needs physical reseat; hand-eye absent.
- Next: await inspector network approval; keep loopback alternative usable, then stop/release at expiry. Coordinate calibration outcome before map/teleop activation. Fix/reseed canonical tracking only in a coordinated window; then verify *fresh* mask, accepted depth and accumulated map separately.
- Standing test restart instructions are in `../wrist_standing_1119/README.md`; inspector occupies port 47814, so stop it first. Outstanding: delay/following error, range/scaling and posture/twist, standing live validation, autonomous grasp with fresh trusted target. Do not label these complete.
