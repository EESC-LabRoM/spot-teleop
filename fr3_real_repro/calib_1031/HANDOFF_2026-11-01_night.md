# Handoff — 2026-11-01 night (posture fix, run 4, EasyHeC sweep + solver)

## State right now
- Arm is HOME, idle (`robot_mode` 1), JTC inactive. Last move: direct 13 s home after the aborted sweep, guard clean (max 3.3 N), parked within 0.014 rad of `rehearsal_q0.txt`.
- Planner (`vm-mpc`) staged in standby with post config + `mpc_override_ns800.yml` + 12 cm buffer. Nothing applied to live camera transforms. All resource claims released.
- Live maps (TWO — one-map rule still unmet): `nvblox_curobo` (5 cm, planner) and `nvblox_human` (2 cm, dashboard `:8788`/`:8795`, restarted 23:06 UTC, pid 671836). Dead `mesh_server` for `nvblox_fixed` on `:8789` shows a 9-day-old frozen map — ignore it.
- EasyHeC solver finished 22:44 lab time on 26 sweep frames. NOTHING applied.

## What happened today (11-01)
1. **Posture cost applied + staged.** `node/curobo_mpc_node.py` loads `mpc_override` param; real config `node_config/fr3_vcap1.2_acc10_post.yml` (retract = rehearsed start) + `mpc_override_ns800.yml` (null_space_weight 800). Sim: max excursion 1.95 → 0.50 rad, final back at start. Guard gained `--qtrip 1.0` posture stop; `avoid_run.sh` CHECK verifies staged config.
2. **Run 4** (`vm_real/saved_runs/avoid_20261101_193411/`): full 80 s, no trip, max dF 6.1 N / dτ 1.5 Nm, 0 contacts. Posture fix worked (max excursion 0.57 rad, ended 0.012 rad from start) but hand stayed 14–28 cm short of low goals (mean goal error 17.6 cm vs 10.9 in run 3). Bottle real gap only **4.1 cm** at t=105 s while map said 13 cm → do NOT lower buffer on 5 cm.
3. **Map restart + fresh clouds.** `nvblox_curobo` restarted clean 22:45 UTC. Fresh capture: 18 occupied voxels in work area; floater B gone, floater A (0.42,0.25,0.58) persists, zed1-only (12 depth pts) → zed1 depth artifact. Bottle 9 voxels, drill 9, drill/box coverage 65–69%. Table plane measured z=46 mm (was assumed 42).
4. **One-map rule** (Murilo's): exactly ONE nvblox for FR3 cameras, planner ESDF + dashboard mesh share it, voxel size chosen by measurement. Written in `/home/nexus/spot-teleop/CLAUDE.md` + memory `one-nvblox-map-rule.md`. Status: NOT satisfied.
5. **2 cm vs 5 cm sim** (same scene/config/12 cm buffer): step median/p95 5.6/6.1 ms (5 cm) vs 5.5/6.0 ms (2 cm); node accepts 71×81×61 grid; real gaps bottle 8.7→12.9 cm, drill 6.6→12.9 cm. 2 cm at buf 0.08: bottle 9.6/drill 8.9 cm, goal err 171 mm; at 0.06: 7.4/7.7 cm, err 146 mm. Planner compute unaffected; unmeasured risk is 1.4 MB/req × ~4 Hz over lab proxies. Floaters persist at 2 cm.
6. **Dashboard map restart + calibration finding.** zed1–zed2 disagree 82 mm (drill) / 66 mm (bottle); table 17 mm tilt 2.6° (zed1) vs 44 mm tilt 0.7° (zed2). ICP zed2→zed1: 7.8°/80 mm, only 43% agree <1 cm. zed1 unmoved since 10-31 anchor (0.0 px, depth median 4 mm); zed2 moved.
7. **zed2 creep dating** (tag angle in zed2 image): 10-31 14:03 −60.7° → 11-01 22:5x −52.5° (8.2° total, ~0.25°/hr, still moving at ~0.4°/hr late). Rotates about its (near-vertical) optical axis → table looks flat, objects slide sideways. Runs 2/3/4 carry ~4/4.5/6–7 cm sideways error; their gap numbers uncertain by cm; trust guard (no force trip), not gaps. Mount must be tightened.
8. **Tag re-anchor test:** zed2 tag error 27 px / 8.1° vs calibration; tag-PnP fit 0.3 px but object gaps barely move (drill 82→77 mm, bottle 62→60 mm) because tag corners inherit zed1's errors + assumed table height. ICP-only fit rejected (165 px tag error). Conclusion: rerun EasyHeC for BOTH cameras, not ICP/tag alone.
9. **EasyHeC sweep planning saga.** Objects stay on table per Murilo; planned around live 2 cm map + raw depth (margins: map 8 cm, raw 6 cm, table 10 cm). Found + fixed: cuRobo silently ignored map (grid 1 cell larger than data) — first paths cut objects by 5.5 cm; fixed, verified it flags them as collisions. Final plan `calib_sweep/sweep_map_v5.csv`: **25 poses, 29 legs (21 direct, 4 via home), 419 s, 3 s holds, peak 0.251 rad/s**; independent check: ≥8.4 cm map, ≥8.5 cm raw, ≥10 cm table, joint-limit margin 0.38 rad. 11/36 candidates dropped (no home route). Poses: all 25 good in zed1, 16 forearm+hand-clear in zed2.
10. **Sweep run 1 (aborted):** bizarre-looking but faithful tracking (within 0.011 rad at sample 2908, ~58 s in); early legs swing elbow/wrist hard by design. User Ctrl+C → stream held, arm released cleanly, no guard trip. Then direct home (13 s).
11. **Sweep run 2 (success)** (`saved_runs/sweep_20261101_221332/`): all 25 poses + home, ~7 min, self-parked within 0.015 rad, JTC inactive. Guard never tripped: max dF 10.6 N, dτ 3.9 Nm. 26 frame sets recorded (both cams every hold), copied to lab `/home/murilo/easyhec/data/live_all/live_calib_1101_20261101_221332` + `/home/murilo/franka_ws/lab/calib_1031/sweep_20261101_221332/`. First attempt refused on scene gate (transient DDS cloud-grab fail); retry passed unchanged.
12. **EasyHeC solver on 26 frames** (`run_1101.sh`: held-out nfit=11 `_ho_1101` then all `_all_1101`, parallel cams on lab GPU):
    - **zed1: GOOD.** 26/26 usable. Moved 31.6 mm / 2.0° from live. Held-out depth median 17.2→8.0 mm, 60% px <1 cm, residual 4.3 mm. Trust it — but apply together with zed2, not alone.
    - **zed2: OVERFIT, DO NOT APPLY.** Only 16/26 frames show arm (matches user's observation). Scene near-black (lab dark). Overlay: neither red nor green contour tracks arm. Held-out depth 9.2→6.4 mm but pose residual 114→179 mm (worse than recorded). Needs bad-frame purge + refit, or rerun with lights.
    - Overlays local: `saved_runs/sweep_20261101_221332/easyhec_1101/v2_zed{1,2}_depth_s0_w1_ho_1101_overlay.png`. Full logs on lab: `/home/murilo/easyhec/run_1101_{zed1,zed2}.out`, results `/work/results_live_1031/log_{zed1,zed2}_*1101.log` in `easyhec-build`.

## What's left (do tomorrow, lights ON)
1. Tighten zed2 mount (Hipólito, at camera) — it creeps ~0.25–0.4°/hr; any calibration decays in hours otherwise. Leave it alone after.
2. Re-run sweep + solver with lights on: `cd fr3_real_repro/vm_real && CHECK=1 ./sweep_run.sh` then `ESTOP=1 EP="<name-at-estop-now>" ./sweep_run.sh`. Needs Murilo go + named e-stop person (old attestations stale after ~20–30 min). Claim `franka-robot` (+ `lab-zed-cameras`, `lab-nvblox`) first; release after.
3. Before applying: inspect overlays + per-frame residuals, drop bad zed2 frames, refit; require cameras agree on objects ≤2 cm and tag reprojection clean. Applying needs Murilo go; then restart nvblox (single-map migration still needs go — dashboard shared with Hipólito).
4. Then: single-map migration (stop `nvblox_curobo`, serve planner from 2 cm node, planner `voxel_size:=0.03` + centre x 0.34), new goal set that sweeps around objects (not into drill) at 2 cm buf 0.06–0.08, rehearse in sim vs fresh map+clouds, then real avoidance run.
5. Open ideas (not approved): tag-reprojection drift gate in `avoid_run.sh` CHECK (refuse if >5 px / 1°); zed1-only map stopgap (needs go).

## Where things are
- Sweep plan: `fr3_real_repro/vm_real/calib_sweep/sweep_map_v5.csv` (+ `_summary.txt`); tooling `calib_poses_map.py`, `calib_legs_map.py`, `calib_sweep.py`, `live_recorder2.py`, `scene_match.py`; debug outs `legs_map_v{1-5}.out`.
- Runner: `fr3_real_repro/vm_real/sweep_run.sh` (scene-match gate, `--qtrip 0`, Ctrl+C/e-stop/`/tmp/sweep_stop` handling).
- Runs: `vm_real/saved_runs/avoid_20261101_193411/` (run 4: mp4, h264, guard csv/log, traj/), `sweep_20261101_215732/` (aborted, plan only), `home_20261101_220454/` (guard csv/log), `sweep_20261101_221332/` (guard csv 30 MB/log, recorder log, plan copy, `easyhec_1101/` overlays + `run_1101.sh`).
- Planner staging: `vm_real/node_config/fr3_vcap1.2_acc10_post.yml`, `mpc_override_ns800.yml`; `rehearsal_{q0,buf,cfg}.txt`.
- Rules: `/home/nexus/spot-teleop/CLAUDE.md`; memory `one-nvblox-map-rule.md` (+ `camera-calibration-stale-2026-10-31.md` has run history).
- Reference launch: lab `/home/hipolito/spot-teleop/isaac-ros_ws/src/spot_nvblox/launch/fr3_nvblox.launch.py`.

## Safety rules (still in force)
- Lab PC read-only unless Murilo says go. Arm moves ONLY with Murilo's explicit go + named e-stop person at that moment. Never stop/kill/restart others' processes without go. No return-move videos unless asked; "go home" = `home_run.sh` direct.
