# HANDOFF 2026-11-04 — bag-as-camera teleop validation (lane: Validação teleop bags ZED)

Information only; nothing here moves the robot. Successor to
`fr3_teleop_bagtest/HANDOFF_2026-11-03_bagtest.md` (read that first for the
package reference; this note covers what changed since + full carry-on state).

## 0. Status at close (2026-11-04 ~17:25 -0300)

- Execute takes: **PAUSED** (00 + 04 PASS, rest pending) per Hipólito's owning
  lane until a tracker gate + stop behavior are validated. Nothing moves.
- Teleop-first stands: Hipólito's lane 7c owns the live window — a joystick
  cuRobo teleop (`fr3_input_teleop_review.py`, `input_source:=joystick`) IS
  running on lab right now (PIDs 1566207/1566257, log
  `/tmp/teleop_joy_1104_noguards.log`). Our preflight refuses while it runs.
- Our parked test chain (tracker 1080717 + relay 1080777) is still up on lab,
  ~5% CPU, consuming NO live feeds (bag replay → `/fr3_bagtest/*` only).
- Lab deploy matches VM on all runtime files (verified read-only at close).
- Grasp lane finished (grip confirmed, no further motion). ZED3 streams fine.
  ZED2 is OUT of the live map with a stale TF; multi-point validation pending.
- No tracker fix is validated or deployed. Guard replays stopped at bag 11 —
  nothing is waiting to be rescored.

## 1. What was done this session

1. **Test package** `fr3_teleop_bagtest` (VM source, lab deploy): bag player →
   test tracker → relay → test teleop → real FR3 on test-only topics/ports.
   One script: `bash bagtest.sh <stage>` (one take, ~3–4 min, foreground,
   ends in sim-anchored CHECK) / `bash bagtest.sh down` (parks chain).
   45 unit tests green (`python3 -m unittest test_runtake test_bagtime`).
2. **Takes**: dry 13/13 flow; honest scoring (0.30 m jump gate) passes 7
   (00, 01, 03, 05, 05_2, 07, 09), flags 6 (02, 04, 06, 08, 10, 11).
   Exec on robot: 00 PASS (neutral still), 04 PASS (z-span 6.8 cm, off-axis
   0.4 cm), both with clean-but-thin 1 s-window stop evidence.
3. **Tracking investigation (all motion types)**: traced tracker→relay→teleop→arm.
   Position jumps on 02/04/06/08/10/11 (worst 06: twelve jumps, 0.64 m max);
   05 under-gate but same on-axis pattern (5× 16–27 cm). Dangerous orientation
   steps (≤0.35 s): five, 32–40° (01×2, 06, 10×2). Root causes verified in code:
   every gate looser than the symptom (in-filter `.03+2·dt`, 40 cm hand
   association, same formula in teleop re-anchor), filter reset accepts any
   jump after 0.65 s gaps, bad samples poison the identity reference, exec
   load triples gaps (box oversubscribed, no CPU quota). Corrected along the
   way (disclosed to peer lane): "impossible velocities" were wall-clock burst
   artifacts (stamp-clock max 2.08 m/s); first bag-time anchors had a
   pass-shift error (fixed); 94–152° orientation figures were reacquisition
   comparisons across gaps, not driven steps (retracted).
4. **Joint diagnosis with repro lane 7c**: 08 jump pairs reproduced from our
   exact frame subset with a fresh tracker → drops + stateful estimation drive
   jumps, not our deployment. Their cand-1 (stricter hand fallback) does NOT
   fix 08. Their guard (pose fallback ≤0.12 m of last accepted hand + ≥2 depth
   sites) removes all dangerous jumps — independently rescored by us: 08: 4→0,
   bag 10: 1→0 (0.47 m → safe 1.03 s gap), bag 11: 1→0 (0.520 m gone; residual
   0.360 m in safe re-anchor class). Yield cost small (−1/−13/−25, gaps in
   limits). Residuals flagged back: sub-gate continuous hand→hand steps,
   orientation unprotected by design, no ROS-timing proof.
5. **VM103 offline replica** (`~/track_replay/` + image `track-replay:local`,
   mediapipe 0.10.14 / numpy 1.26.4 / cv2 4.8.1, `--cpus=2 --network=none`,
   ~40 s/run): 08-subset validation vs lab baseline — 412/412 stamps, 0 drops,
   359 vs 358 valid (sole diff: one two-hands flip), same 4 jump pairs at
   identical timestamps. **New finding**: probe is run-to-run nondeterministic
   (run2: 360 valid, pair 2 vanished) — wall-clock boundary gates in `frame()`
   + scheduling jitter. Guard claims near 0.30 m need multi-run confirmation.
   Traces mirrored to lab `takes/track_profile/vm103_val08/`. Matrix offer
   (N × variants × 13 stages) extended to lane 7c, held during live windows.
6. **Infra**: killed 4 leaked `bagtest_robot_side` helpers (~43% CPU each,
   7.5 h) + fixed the leak at the root (`serve` reaped on all cleanup paths,
   regression test, deployed). Cleared lane 7c's live-teleop preflight from
   our side (no motion controller loaded, zero goal publishers possible,
   no live-feed readers) + flagged state broadcasters inactive at the time.

## 2. Where things are

VM package `/home/nexus/spot-teleop/fr3_teleop_bagtest/` (NOT a git repo;
md5 = version). Fresh md5s at close (2026-11-04):

| file | md5 | note |
|---|---|---|
| run_take.py | 983f3155a6776025eaa6ba19e69ebcb6 | fixed cleanup, +5 s post-stop dwell |
| bagtime.py | 04272577557c9bc7d8dc24432dd756e1 | jump gate + engaged-window scoring |
| bagtest.sh | 40d342d8df88af9233ec4d5f1fd572f2 | THE script |
| rescore.py | eeae1d3a62330abca6e4f993a9fe42dc | rescore w/ same gates |
| bag_camera.py | 6feafe9c3cb1fd8929d7a98634597205 | |
| record_jsonl.py | e398198463b113eb5c0f2058c4a44644 | |
| run_session.sh | d7c4435dc36f7519346b4bca0317343f | superseded by singles |
| test_runtake.py | 9ce41722aee8e85502abdf29320c9c70 | 45 tests green |
| test_bagtime.py | 7c46a7d59de43e1ad03dfe85f064c90f | |

Lab deploy `/home/murilo/fr3_bagtest/`: identical md5 on ALL runtime files
(verified 2026-11-04 ~17:23 -0300); only `test_runtake.py` differs on lab
(`ee33275b`, older — lab never runs tests, harmless).

Key paths (lab, via `ssh -i ~/.ssh/id_ed25519_lab_claude murilo@100.88.134.127`):

- Takes + logs: `/home/murilo/fr3_bagtest/takes/` (`dry_session3/`,
  `exec_20261103_151015_00_neutro`, `exec_20261103_151816_04_frente_tras`,
  `track_profile/` incl. `vm103_val08/` mirror)
- Bags: `/home/hipolito/spot-teleop/bags/zed_protocol_20261030_005244/<stage>/d8`
- Sim validation: `/home/hipolito/spot-teleop/clips_out/zed_protocol_analysis/simulation_review.json`
- Hipólito teleop repo: `/home/hipolito/spot-teleop` @ `bbea50b` + local mods
  (2 synced files pre-exec — see 2026-11-03 handoff §2; do NOT revert blindly)
- Repro-lane workdir: `/home/hipolito/franka_teleop_checks/2026-11-03-execution/tracker_profile_work/`
  (candidate, pose-guard, `profile_08_*`, `guard_08_*`, `guard_11_*` traces)

VM replica: `/home/nexus/track_replay/` (`src/` tracker sources incl.
candidate + pose-guard, `Dockerfile`, `mnt/out/val08*` traces+summaries),
image `track-replay:local` (7ea8f7b3ce3c). GAP: the ROS-free adapter
`local_replay.py` (md5 `5992c579` per 2026-11-03 addendum) is NOT on disk —
it was piped into the container at run time; recover from the 2026-11-03/04
session logs or recreate (stubs rclpy/messages, reads MCAP via `rosbags`
with ROS2_HUMBLE typestore, runs real `frame()`, emits lab-identical
trace.jsonl + summary.json).

Env/isolation: containers `fr3-curobo` (Humble, test perception) +
`fr3-ros2` (Jazzy, robot side); domains 8=test / 77=robot / 142=grasp-MPC;
test UDP 47817 (live 47813), calib 47818 (live 47815), pages :8793/:8790,
topics `/fr3_bagtest/*`.

## 3. What's left

1. **Tracker gate validation (owns: repro lane 7c, we rescore)**: guard needs
   ROS-timing proof, more stages (06 = worst, 10, 02 next), orientation +
   reacquisition protection by design, and multi-run confirmation near 0.30 m
   (nondeterminism finding). Rescore each new trace offline on arrival with
   the same gates; report per-variant jump pairs + yield + max residual.
2. **Stop validation (owns: 7c)**: our 2-take evidence is thin; needs a
   dedicated stop proof before any moving take resumes.
3. **Execute takes resume (needs: 1+2 + Murilo's go + e-stop + board claim)**:
   order 05, 01, 03, 05_2, 07, 09 first (dry-clean), then re-evaluate the
   6 flagged stages against the validated gate. First step after any pause:
   re-home + reactivate cartesian impedance (grasp leaves JTC+cartesian
   inactive, mode 1, arm parked off-home).
4. **Operator handoff (blocked)**: stop test chain → arm home → operator teleop
   (UDP 47813) disengaged → dashboards :8088/:8089 green. ZED3 streams again
   (~14–19.6 Hz) so camera is unblocked; handoff waits on 1+2.
5. **Murilo's pending calls**: (a) stop 2nd nvblox (`nvblox_seg`) to restore
   ONE-map; (b) ZED2 TF choice (deployed stale ~38 mm/3.2°, ALT unvalidated,
   ~44 mm depth disagreement) only after multi-point validation, then ZED2
   map integration + cable-feed fix; never run `cable_fanin2.py`.

## 4. How to resume

- Read-only lab state: `ssh -i ~/.ssh/id_ed25519_lab_claude
  murilo@100.88.134.127 "pgrep -af 'bagtest|fr3_input_teleop|run_take';
  uptime"` — expect our parked tracker+relay, NO `run_take`, and (while the
  live window lasts) lane 7c's joystick teleop. Do NOT touch their procs.
- Before any take: claim `franka-robot` on the board, coordinate grasp lane
  (`lab-nvblox`/`lab-zed-cameras` stay theirs), person at e-stop, run
  `bash bagtest.sh <stage>` from `/home/murilo/fr3_bagtest` on lab.
- Rescore: `python3 rescore.py takes/<take>` on lab; unit tests on VM only.
- Replica runs: only with grasp + live-window clearance (this box serves live
  perception); `--cpus=2 --network=none`, pause during motion windows.
- Duet: check `sys_duet_inbox` at start; peer session IDs — repro/live 7c:
  `7c04690c6bdd4c70aaf56606cb7191e1`; alignment: `c05cade07e4c4abf9d1d5b1b54578570`;
  grasp: `58e05c74ecf7448a86a47e84c72e04cf`; ZED2-integration: `e64db8e713344176bb8c827638762c12`.
  Murilo (user) is go-authority and overrides the board.
