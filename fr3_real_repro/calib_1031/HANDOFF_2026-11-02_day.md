# Handoff — 2026-11-02 day (driver recovery, zed1-only map, ghost hunt, avoidance run 5)

## State right now
- Arm: idle (`robot_mode` 1), JTC inactive, parked within 0.014 rad of rehearsed start. Last motion: `avoid_20261102_131808` SUCCESS (~80 s, no trip).
- Our lane holds NO resources (claims expired/released). STOOD DOWN: Hipolito lane announced it is taking the arm NOW for a cal sweep — do NOT command arm, switch controllers, or restart zed-build / ZED wrappers / nvblox until he posts done.
- OPEN CONFLICT (flagged urgent to both, both woken): board shows `franka-robot` + `lab-nvblox` + `lab-zed-cameras` HELD by grasp lane `58e05c74` (Murilo at e-stop, live drill grasp), while Hipolito lane `66f62b19` (Hipolito at e-stop) says it is moving the arm now and is NOT on the board. Next agent: check board + inbox before touching anything.
- Map: single zed1-only `nvblox_human` (Spot 2 cm: voxel 0.02, 5 Hz, 6-vox trunc, weight 100/0.0001, `num_cameras: 1`), dashboard `:8788` live, mesh store cleared today. `nvblox_curobo` STOPPED by us — BUT grasp lane later posted "planner map restarted and answering (5cm, 8ms)", so a second map may be back. Re-verify one-map status next session.
- zed2 is ON THE FLOOR (fell off mount today after creeping 0.25–0.4°/hr). Any zed2 extrinsics from sweeps are meaningless until remount + tighten. Secure it.
- Planner: staged standby, 12 cm buffer, post config + ns800 override; 143-domain proxy repointed at human node (first ESDF received). 142 proxy still points at dead curobo map — file lives in Hipolito's tree, coordinate before touching.

## What happened today (11-02)
1. **Driver recovery (Murilo go).** `CHECK=1 sweep` refused `robot_mode 0`. Root cause: `FrankaHardwareInterface` lost FCI session 03:23 UTC (`libfranka: UDP receive: Timeout`), deactivated everything; joints + mode 0 were frozen cache (`/joint_states` from publisher, not HW). Desk fine (box OK, driver session dead). Restarted `fr3-ros2` → HW active, mode 1, joints live; restored `:8088`/`:8089`, relay, JTC inactive; clean-restarted `nvblox_curobo` (0 phantom voxels).
2. **`CHECK=1` then refused on scene gate:** zed1 matched (1 pt), zed2 34,547 pts >6 cm off at ~(0.67,-0.17,0.30) — known creep, not table. No bypass; v5 plan built on old geometry.
3. **Odom debate settled with data.** zed2 epochs 63→84 in a day, jumps 0.3–1.4 m / up to 34° with bench static and NO restarts; zed1 6 jumps 0.3–1.0 m. Odom noise 10–100× the 0.25°/hr creep signal. Verdict stands: odom relative-only within one epoch over seconds (anchor logger already does this, 0.30 m/15° jump reject), creep detection via reference image + tag. VIO is built for moving robots, not mm-static tables; dark + shared-USB corruption starves it.
4. **zed1-only single map (Murilo go).** Stopped `nvblox_curobo`; `nvblox_human` cut to `num_cameras: 1` (zed1 seg depth+color+mask), Spot params. Depth callbacks flowing, mesh age ~0.5 s, `:8788` 200. MPC had no ESDF until step 8 repoint.
5. **Ghost hunt (drill artifact).** User screenshot showed translucent column by drill. I was wrong twice: (a) "shiny-metal smear" — drill is matte plastic, and per-pixel check showed mesh 0.57–0.62 m vs LIVE depth clean background 0.83–0.86 m (mask clean), plus 8-frame/40 s flicker test all-background; (b) "hand during rebuild". True cause: dashboard **mesh_server had run 9 days** and survived every map restart, serving fossil blocks from the old two-camera map (zed2's offset double). Restarted mesh server on fresh store → 0 verts at spot, ghost gone. Live node was clean the whole time. (My first projection was also broken — wrong transform direction; redid vs live TF.)
6. **Paint page:** served 3 renders (zed1 view, orbit≈dashboard angle, top-down) at `http://100.88.134.127:8791/ghost.html`; user marked 11 pts, all converged to one veil at ~(0.72,-0.27,0.28), 15–35 cm above table, ~65 verts. Method worked; keep it for future "show me where" questions.
7. **Segmenter: HEALTHY (user was right).** ~9 ms/frame, joint-synced, masks ~19% (the arm); static mesh 214 low verts, no arm. My "empty mask" alarm was a misread: it marks masked pixels 0, not NaN. Empty `target_mask` = no grasp target selected (normal idle). Container "unhealthy" label is a red herring.
8. **Avoidance rehearsal fresh (Murilo go):** capture via proxy (fell back to direct domain 8 — same map bytes) + `rehearse.sh` 84 s sim from live joints: hold 1.1 mm max, 0 contacts/4200 steps, map clearance 14.2 cm min, posture 0.61 rad ending home. Script verdict FAIL only on missing labeled clouds → measured directly: whole-cloud min 8.6 cm (drill ~14.3, bottle ~8.6, box ~35). `CHECK=1 avoid_run.sh` green (joints 0.000, mode 1, buffer/cfg match, JTC inactive).
9. **Run 5a `avoid_20261102_130837`: clean abort, arm never moved.** Race I introduced: `set_active` returns at warm-start, `traj` saw "warming" → refused "MPC not active (go first)". Release path perfect (guard 1.9 N). Fixed `avoid_run.sh`: wait up to 3 min for planner ACTIVE after `go`, else release + quit.
10. **Run 5b `avoid_20261102_131808`: SUCCESS.** Traj `20261102_131857`, 3999 preview msgs, peak excursion 0.55 rad (~31°), reparked within 0.014 rad, guard max dF 4.37 N / dτ 1.05 Nm, 8782 notes, tripped False. Video `avoid_20261102_131808_h264.mp4` (8.9 MB) in run folder.
11. **RealSense scoped (not started):** SDK present, NO ros wrapper, NO extrinsics (only ZEDs calibrated), NO arm masking (would fuse arm into map), USB contention risk. Path: install wrapper → calibrate → extend masker → add camera_2 → restart map. Days, not a toggle. Recommend after avoidance campaign.
12. **Duet traffic:** (a) Grasp lane `58e05c74` going live (drill cloud + 100 GraspGen-X, top 0.87, selected vertical 0.83 at (0.516,-0.286,0.345), runner `/home/murilo/franka_ws/lab/claude_trial/run_grasp_drill.sh`; fixed MPC joint starvation via 47901 relay + stale-hold abort). Replied: our arm free, controller inactive, `cranky_tesla` not ours. (b) Hipolito lane `66f62b19`: replay finished NOT driving (home err 2.6 mm); `franka-teleop` rewritten, same trees, new hashes `133ce84`/`1cfd456`/`bbea50b` (was `f66a201`); then TAKING ROBOT NOW for cal sweep (`fr3_workspace_sweep.py --box`, 23 wp, 0.04 m/s, live_recorder, `hec_live3_1101` refit, republish extrinsics, nvblox/ESDF up). Replied urgent: we stand down + conflict flag + zed2-on-floor warning. Also warned grasp lane. Both woken.

## What's left
1. WAIT for Hipolito's "done" post. Do not touch arm/controllers/zed/nvblox until then. Check `sys_duet_inbox` + resource board first thing next session.
2. Resolve grasp-vs-Hipolito arm conflict (two e-stops, one arm) before any motion from our side.
3. Secure/remount/tighten zed2. Until then: zed1-only everything; decline any zed2 refit/apply.
4. Score run 5b real clearances (guard CSV vs object clouds) — offered, Murilo hadn't confirmed.
5. Re-verify one-map status (grasp lane may have restarted a planner map) and repoint/align before next avoidance run.
6. RealSense integration (needs Murilo go + Hipolito coord — touches shared dashboard map).
7. 142 proxy still on dead map name — coordinate with Hipolito before touching his file.
8. Re-acquire `franka-robot` (+ `lab-nvblox`, `lab-zed-cameras`) before any future motion; needs Murilo go + FRESH named e-stop attestation (~20–30 min).

## Where things are
- Runs: `vm_real/saved_runs/avoid_20261102_130837/` (aborted race — no traj), `avoid_20261102_131808/` (SUCCESS: h264 8.9 MB, guard csv/log, traj `20261102_131857/`).
- Video: `/home/nexus/spot-teleop/fr3_real_repro/vm_real/saved_runs/avoid_20261102_131808/avoid_20261102_131808_h264.mp4`
- Fix: `vm_real/avoid_run.sh` (3-min ACTIVE wait after `go`).
- Sweep plan v5 still on disk (`vm_real/calib_sweep/sweep_map_v5.csv`) but geometry STALE (zed2 moved, then fell) — re-capture + re-plan (v6) before any sweep; needs go.
- Ghost paint page: `http://100.88.134.127:8791/ghost.html` (if server still up).
- Rules: `/home/nexus/spot-teleop/CLAUDE.md`; prior handoffs `calib_1031/HANDOFF_2026-10-31_calibration.md`, `HANDOFF_2026-11-01_night.md`.

## Safety rules (still in force)
- Lab PC read-only unless Murilo says go. Arm moves ONLY with Murilo's explicit go + named e-stop person at that moment. Claim resources first, release after. Never stop/kill/restart others' processes without go.
