# HANDOFF 2026-11-03 — FR3 bag-as-camera teleop validation (lane: Validação teleop bags ZED)

Reproduce without guessing. Information only; nothing here moves the robot.

## 1. Package (source of truth: VM, deployed: lab)

VM dir (NOT a git repo — no commit/diff; files + md5 are the version):
`/home/nexus/spot-teleop/fr3_teleop_bagtest/`

| file | md5 | role |
|---|---|---|
| run_take.py | 983f3155a6776025eaa6ba19e69ebcb6 | one-stage orchestrator (preflight→play→engage→stop→dwell→home→CHECK) |
| bagtime.py | c1a303c32a6143c5a608135c15d0d2e4 | ROS-free stage specs, metrics, CHECKs |
| bag_camera.py | 6feafe9c3cb1fd8929d7a98634597205 | bag image publisher + viz page |
| record_jsonl.py | e398198463b113eb5c0f2058c4a44644 | topic→jsonl recorder |
| rescore.py | 278f97310e2ed3d32c23ff802a72eea6 | re-apply metric to a finished take |
| bagtest.sh | 4f2b6fdd4117f07699303e5d16982418 | THE script: `bash bagtest.sh STAGE` / `bash bagtest.sh down` |
| run_session.sh | d7c4435dc36f7519346b4bca0317343f | all-takes driver (superseded by bagtest.sh singles) |
| test_bagtime.py | e6df646fe56082718aa45d99206edfe4 | unit tests (VM) |
| test_runtake.py | 9ce41722aee8e85502abdf29320c9c70 | unit tests (VM, 45 tests) |

Lab deploy: `/home/murilo/fr3_bagtest/` (identical md5 except test_bagtime.py,
lab has older 372f512b — lab never runs tests; harmless).
2026-11-03 ~22:42 -0300 fix: cleanup lists now reap `bagtest_robot_side.py
serve` (RobotLink.close killed only the docker-exec client; 4 strays x ~43%
CPU leaked from the morning exec session were stopped live on lab; load 37→27
and falling). run_take.py md5 above is the fixed+deployed copy; takes paused so
the new code has not run a take yet.
Deploy cmd (from VM): `scp -r fr3_teleop_bagtest murilo@100.88.134.127:/home/murilo/fr3_bagtest`
SSH key: `~/.ssh/id_ed25519_lab_claude`, host `murilo@100.88.134.127`.

## 2. Inputs (all on lab)

- Bags (13 takes / 12 movements, 05 repeated): `/home/hipolito/spot-teleop/bags/zed_protocol_20261030_005244/<stage>/d8`
- Sim validation: `/home/hipolito/spot-teleop/clips_out/zed_protocol_analysis/simulation_review.json`
  (+ REPORT.md, SIMULATION_RESULTS.md, comparison.json in same dir)
- Stage specs/checks derived from it: `STAGE_SPEC` in bagtime.py.
- Hipólito teleop repo: `/home/hipolito/spot-teleop`, tip `bbea50b`
  `FR3: cuRobo MPC + nvblox live-ZED teleop sources and install notes`, WITH local
  modifications (incl. fr3/teleop/fr3_input_teleop.py, joystick_bridge.py, lab_dashboard.html,
  start_zed3_wrist.sh — see `git status --porcelain`). Running copy was synced with the
  2 files where it lagged (relay v3 samples, deploy review states) before execute takes.

## 3. Environment / isolation

- Containers: `fr3-curobo` (ROS Humble, test perception) + `fr3-ros2` (ROS Jazzy, robot side).
- Domains: 8 = test perception, 77 = robot/impedance, 142 = grasp/MPC (not ours).
- Test-only: UDP 47817 (live teleop 47813), tracker calib 47818 (live 47815),
  cam page :8793, tracker overlay :8790, topics `/fr3_bagtest/*`. Live tracker/relay/teleop
  are never restarted or remapped. Preflight refuses unless: no teleop proc runs,
  `equilibrium_pose` idle, cartesian impedance active, live hand topics idle, arm homes cleanly.

## 4. Commands (run on lab, `cd /home/murilo/fr3_bagtest`)

- One take (~3-4 min, foreground): `bash bagtest.sh 04_frente_tras`
- End of session: `bash bagtest.sh down` (stops test chain)
- Dry (no motion, safe anytime): `python3 run_take.py --stage <s> --out takes/dry_<s>`
- Rescore: `python3 rescore.py takes/<take>`
- Unit (VM, no ROS): `cd fr3_teleop_bagtest && python3 -m unittest test_runtake test_bagtime` (37 tests)
- Execute needs: person at e-stop + `franka-robot` resource held + grasp-lane coordination.

## 5. Results so far

Dry baseline `takes/dry_session3/` — 13/13 recognition_ok (position ≤12cm / orientation ≤15° / gestures):
00 11.56cm, 01 7.37/4.2°, 02 10.75/5.5°, 03 5.0/8.9°, 04 9.85cm, 05 10.35cm,
05_2 7.59cm, 06 5.04cm, 07 gestures, 08 5.51cm, 09 5.14cm, 10 4.48cm, 11 4.81cm.

Execute (robot follows bag, homes between takes):
- `takes/exec_20261103_151015_00_neutro` PASS: neutral, spans [0,0,0]cm, oriexc 0.0°,
  travel 0.04m, n=82. Stop: track_err 0.00cm, drift 0.00cm (n=993), cmds_after_stop 0.
- `takes/exec_20261103_151816_04_frente_tras` PASS: height, span_z 6.8cm (min 5.0),
  offmax 0.4cm, oriexc 2.8°, travel 0.17m, n=65, home OK. Stop: track_err 0.00cm,
  drift 0.00cm (n=598), cmds_after_stop 0.
- Both stop blocks are 1s-window backfills (takes ran pre-dwell code); takes from 05 on
  print a live 5s post-stop dwell line. Post-take idle verified both times: no teleop proc,
  equilibrium_pose has zero publishers, impedance active holding last pose.

Per-take artifacts: `summary.json` (check + post_stop), `robot.jsonl` (TCP/command/engaged),
`samples.jsonl` (new vs recorded), `logs/teleop.log` (engage/stop/home lines).

## 6. Deviations / limitations / pending

- Exec-04 reinfer drift 14.35cm vs dry-04 9.85cm (informational; execute CHECK is
  motion-isolation and passed; recognition gate is dry-only).
- Absolute hand coordinates are NOT compared (p/q reference changed since October
  recording; ~2m mean offset on identical images; teleop uses Start-relative motion).
- Pending execute: 01,02,03,05,05_2,06,07,08,09,10,11. Note: stage 11 never engaged live yet.
- Operator handoff still ahead: stop test chain, arm home, start operator teleop (UDP 47813)
  disengaged, dashboards :8088/:8089 green, **ZED3 needs physical replug** (wrapper runs,
  no images) — verify `ros2 topic hz /zed3/zed_node/rgb/image_rect_color` + `:8786/snapshot`.

## 7. Current state (2026-11-03 ~15:25 -0300)

- Lab: test chain UP (tracker+relay+camera, last loaded 04), no teleop proc,
  cartesian_impedance_target_controller active, fr3_arm_controller inactive,
  equilibrium_pose idle, arm at home. Grasp-lane no-motion dry finished 15:23 (`.../claude_trial/runs/20261103_152348_dry`).
- Resources: `franka-robot` HELD by this lane (renewed, 120min TTL); grasp lane holds
  lab-nvblox + lab-zed-cameras (perception-only, cleared our window; keep ZED1 rig untouched).
- Suggested next: `bash bagtest.sh 05_direita_esquerda`, then 06, 01, 02, 03, 07, 08, 09, 10, 11.

## 2026-11-04 addendum — VM103 offline replica validated (takes paused, teleop-first)
- Replica: `~/track_replay/` + image `track-replay:local` (mediapipe 0.10.14, numpy 1.26.4,
  cv2 4.8.1); ROS-free adapter `local_replay.py` md5 `5992c579` (rosbags via ROS2_HUMBLE
  typestore). Runs actual tracker vs guard variants, `--cpus=2 --network=none`, ~40 s/run.
- 08 scored-subset validation vs lab baseline (412 stamps, seed 0.5009): run1 412/412 matched,
  0 drops, 359 valid (lab 358; only diff row297 two-hands flip), 5 jump-rej + 45 no-assoc exact,
  same 4 jump pairs at identical timestamps (pair1 endpoints agree 3-9 mm).
- FINDING: probe is run-to-run nondeterministic — run2 gave 360 valid, pair1 exact (0.427/0.510),
  pair2 vanished (rows 251/252 flipped). Cause: wall-clock boundary gates in `frame()`
  (pose discard >0.35 s, input-age drop >0.25 s) + scheduling jitter under 1x pacing.
  Guard claims near 0.30 m need multi-run confirmation; pair-1-class events stable 3/3.
- Traces mirrored: `/home/murilo/fr3_bagtest/takes/track_profile/vm103_val08/`
  (`val08_trace.jsonl` 2310e72f, `val08b_trace.jsonl` 5ad61f59 + summaries).
- Offered repro lane 7c the repeated-runs matrix (N x variants x 13 stages) from VM103.
