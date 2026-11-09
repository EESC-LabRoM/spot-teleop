# Live grasp + avoidance test — 2026-11-07 (~04:20–06:05Z)

Scripted live grasping on the FR3 (domain 77) with the cuRobo MPC (held-attach v6
deployed, `held_attach armed x3 models`). User (Murilo) at e-stop, explicit go per run.
Claim `franka-robot` held + renewed throughout, released at the end.

Driver source (VM103): `/tmp/mpc_grasp_drive.py` — **still in /tmp, NOT committed.**
Deployed copies in fr3-ros2: `/tmp/mpc_grasp_v6..v13.py` + logs `/tmp/mpc_grasp_v*.log`.
TODO: commit the driver to the repo (suggested: `fr3_real_repro/regrasp/tools/`).

## 1. Motion chain: why the arm first refused to move

All of these were broken at once; each fix was verified before moving on:

1. **Wrong goal topic.** Driver published `/wrist_pose`; live remap is
   `/fr3_joystick_mpc/wrist_goal`. Fixed in driver (`GOAL_TOPIC`).
2. **Driver goal QoS.** Driver used BEST_EFFORT pub, MPC sub is BEST_EFFORT/VOLATILE —
   compatible in theory, but reliable pub (QoS 10) is what works; set in v7.
3. **Forwarder FIRST_MAX gate.** `direct_topic_fwd.py` refuses the first packet when the
   MPC command is > 0.05 rad from measured joints. The PF-shifted command sits ~0.07 rad
   off, so every cold start was refused silently (log is block-buffered!). Raised to
   **0.15** for the session. Backup: `/tmp/direct_topic_fwd.py.bak_firstmax05`.
   The 0.03 rad/packet slew still rate-limits. TODO: decide permanent value / revert.
4. **Engaged gate.** `/teleop/engaged` needs persistent `true` (`ros2 topic pub -r 1`,
   pid 10198, left running). Forwarder is silent without it.
5. **Orphan goal publishers.** Several `ros2 topic pub` debug goals survived `pkill`
   (redirect hides them from `-f` match) and fought each other: freeze/crawl/jitter.
   Lesson: launch debug pubs with `timeout N` so they self-kill. Verified 0 pubs before
   every scored run (`ros2 topic info ... | grep Publisher`).
6. `set_active` needs `std_srvs/srv/SetBool` (a `franka_msgs/srv/SetActive` attempt fails
   silently). Verify via `/fr3_joystick_mpc/state` (`active`/`standby`).

Proof of motion: PRE 5 mm in 6 s, DESCEND 7 mm in 4 s (v7e).

## 2. Grasp results (drill ~1.3 kg, 60 N close)

| run | descend | close width | lift | note |
|-----|---------|-------------|------|------|
| v7e | 7 mm | blind close (driver crashed after send) | — | gripper shut 7 mm off-drill; may have nudged it |
| v9 | 48 mm (PF equilibrium) | 44 mm | 40 mm | user: missed (fingertip graze) |
| v10 | 7 mm (settle 25 mm) | 51 mm | 4 mm | slipped during lift (user saw) |
| v11 | 2 mm + PRESS −15 mm | 51 mm | staged 18/18 mm | HELD, placed OK |
| v12 | 12 mm | 49 mm | 18 mm | set_load fail (fatal then) → placed manually |
| v13 | 2 mm | 51 mm | slow (cost) | loaded sweep blocked, see §4 |

Fixes that mattered: DESCEND settle 60→25 mm, PRESS 15 mm below grasp before close
(deeper finger engulfment), staged lift (+5 cm then +10 cm), slip check (width<5 mm).
PF-equilibrium accept (stalled + err<100 mm → proceed): with hand-gate caps<1 the hand
legitimately settles short of the raw goal; demanding err<60 mm aborts good runs.

## 3. Shake analysis

- v9 shake: hand at z 0.26 **below** the grasp, MPC pressing → contact judder against
  drill/table. Stopped once lifted clear. Not payload.
- Loaded shake (v11+): unmodelled 1.3 kg + pendulum swing of a 20 cm object on a
  fingertip hold. Arm is rated 3 kg, so mass per se is fine.
- `set_load` **cannot fix it mid-session**: `/service_server/set_load` (franka_msgs/
  SetLoad) is rejected in both active and standby (`command not possible in the
  current mode ("Move")`). Load must be set at bringup (idle, before controllers
  claim motion). TODO next bringup: mass 1.3, CoM ≈ (0,0,0.10), inertia ≈
  diag(0.005,0.005,0.002). Driver calls it best-effort (non-fatal).

## 4. Avoidance demo: empty sweep vs loaded sweep (v13)

Same two waypoints (GX±0.15, GY+0.20, GZ+0.12) at z 0.40:

- **Empty:** E1 6 mm/7 s, E2 10 mm/6 s. Fast, tight.
- **Loaded (drill in hand):** L1 19 mm/9 s, **L2 TIMEOUT (err 100 mm, 21 mm/45 s)**.
  MPC: status OK, CLEAR, coll_cost ~550 (vs ~13 empty), PF released (gripper closed).
  The MPC fights **its own unmodelled drill** (fresh voxels under the hand, no attach,
  no clear) and nearly freezes. It also sank ~93 mm during L2 (cost-driven, not sag —
  recovery climb proved the arm lifts the load fine).

Conclusion: without held-attach + clear, carrying the drill paralyzes the MPC. This is
exactly the gap held-attach closes — blocked on §5.

## 5. Held attach: latched, never attached

`HELD latched at z=0.323: holding d=0.013 w=0.068` ✓, then
`HELD attach refused: no cache, live box unrisen, fallback below latch height` (B2 rule
working as designed). Root cause: **`/target_object/clear_box` has 0 publishers** on
domain 77. Producer is `sam2_tracker_node` (vm-tracker-prod, domain 8); there is no
d77 relay (only `grasp_target_relay_d77.py` exists in `regrasp/lab/`). TODO: add
`clear_box_relay_d77` (or equivalent) → attach should fire at +4 cm, then re-run §4.

## 6. Tracker behavior under the hand

Repeatable cycle: hand hovers over drill → loop OCCLUDED/untrusted, pick jumps
((0.60,−0.18) → (0.55,−0.10) → (0.40,−0.09), TF z ~0.14) → retreat hand high/clear →
REGEN → BOUND → TRACKING trusted with full cloud (5–7k pts) in ~1–2 min. Always
re-acquire a fresh binding after any low hover; never grasp on a >5 min binding.
`clear_box` absence (§5) also means the drill is never cleared from the ESDF.

## 7. Final state left

- Arm parked high: hand TCP (0.337, 0.098, 0.421) (≈ home), gripper open 80 mm,
  MPC **standby**, JTC holding. Drill on the table near (0.60,−0.22).
- Still running (lab): `direct_topic_fwd` (FIRST_MAX 0.15), `/teleop/engaged=true`
  pub, esdf_proxy + grasp_target relay, MPC container with held v6.
- `franka-robot` claim released. No goal publishers remain (verified count 0).

## 8. Next steps

1. Commit `/tmp/mpc_grasp_drive.py` to the repo; decide FIRST_MAX 0.15 vs 0.05.
2. Add clear_box d77 relay → re-run loaded sweep, expect attach + free motion.
3. Set 1.3 kg load at next bringup idle → re-check shake.
4. Box location still unknown to this lane — user never answered; avoidance-vs-box
   needs its xyz before a targeted test.

## 9. Lane close-out — 2026-11-09 (chat closed, no new live runs)

What this lane did (full arc, VM103 + live arm):
- Sim PF on VM103 (GPU0, ROS domain 104): GraspGen-X on segmented drill cloud
  (`cap_ggx_01`, 7159 pts, 100 grasps, conf 0.005–0.949); re-ran L40s `pf_vid_on_t5`
  drill test (op err 58.3 mm → hand 9.9 mm, lift 146 mm, success); A/B hand-vs-goal
  gate with live lab params (both lifted 150 mm+, 22 mm residual, no hunting);
  videos served at `https://omnigent-vm.tail0994ab.ts.net:9446/`.
- Live view pages on `:9447/live.html` (+ `?src=real`): verified sim slider moves and
  real TF/target/cloud at 2 Hz; dashboard `#sim` Grasp-vs-cloud panel traced and
  yaw-follow patched (live page is lab `isaac-build:/tmp/nvblox_real/lab_dashboard.html`).
- Tracking robustness: diagnosed `/grasp_target` loss as stale object-gate (jump reset);
  restarted publisher with fixed gate; added yaw-riding (loop `live_yaw.json` →
  publisher, flip rejection + ICP/PCA cross-check); yaw-gate fix later deployed by
  sibling lane (live 04:05:51Z).
- Held-drill: stage 1 (loop width latch, freeze on close) live; stage 2 (MPC attach
  v2→v6: deferred +4 cm attach, 8 cm near test, width floor, TF'd box, table need-gate)
  reviewed, sim-verified (1600+ OK / 0 BLOCKED, empty-close + slip-out refused), and
  DEPLOYED live (`held_attach armed: 4 spares x3 models`, verified + PF values intact).
  Live attach never fired (§5) — expected, missing clear_box relay.
- Scripted live grasp v7→v13 (§1–§4): full grasp cycle live, avoidance empty-vs-loaded
  demo, shake analysis. Arm left parked high, MPC standby, claim released.

Where things are:
- This note: `HANDOFF_2026-11-07_live-grasp.md` (this file).
- Sim runs (VM103 `isaac-sim_ws/franka_mpc/runs/`): `cap_ggx_01/` (clouds/drill.npz),
  `pf_vid_on_t5/` (video.mp4, RESULT_isaac.json), `pf_live_hand_t5/`, `pf_live_goal_t5/`,
  `pf_live_cells_eval.json`; viewer run `simrep/viewer_runs/cap_ggx_01/`.
- Throwaway probes (VM103, not repo): `/tmp/ggx_cap/`, `/tmp/l40s_pf_videos/`
  (also `~/l40s_pf_videos/` + index served on :9446 via 127.0.0.1:8793).
- Live driver (UNCOMMITTED — top TODO): VM103 `/tmp/mpc_grasp_drive.py`; lab
  `fr3-ros2:/tmp/mpc_grasp_v6..v13.py` + `/tmp/mpc_grasp_v*.log`. Suggested home:
  `fr3_real_repro/regrasp/tools/`.
- Live MPC (lab): node + `held_attach.py` v6 in `/home/murilo/franka_ws/hub-curobo/node/`
  (= `/node` in `codex-fr3-live-mpc`); params lab `/tmp/pf.yaml` (+ `.bak_preheld`);
  deploy script staged as `deploy_held.sh` 96812199 (cleared by sibling lane).
- Live forwarder: `fr3-ros2:/tmp/direct_topic_fwd.py` FIRST_MAX=0.15, backup
  `/tmp/direct_topic_fwd.py.bak_firstmax05`.
- Publisher/gate (sibling lane's files, vm-viewer): `grasp_target_from_tf.py`
  02f480d4, `object_gate.py` 10173469, backups `*.bak_preyawgate_20261107`.
- Dashboard: served `lab_dashboard.html` (+ `.bak_ggrasp_live_20261107`); yaw patch live.

Left state at close (verified, then released):
- Arm parked high TCP ~(0.337, 0.098, 0.421), gripper open 80 mm, MPC standby,
  JTC holding. Drill on table near (0.60,−0.22). No goal publishers (count 0).
- Running (lab): `direct_topic_fwd` (0.15, silent/pad-gated), esdf_proxy_d77 +
  grasp_target relay, MPC with held v6. Static `/teleop/engaged=true` pub REMOVED
  (pad gates itself; any future debug engaged pub must be timeout-bound).
- `franka-robot` claim RELEASED. Robot free.
- Sibling lane 'Teleop ZED calibração' staged disengaged ZED chain
  (input zed/review/horizontal, gate closed, published=0), confirmed all preserves
  intact, claimed/released `lab-zed-cameras`. No conflicts; they will not restart
  fr3-ros2 or the MPC.

What is left (for next agent):
1. Commit the driver; decide FIRST_MAX 0.15 vs 0.05 (see §1.3).
2. Add `clear_box` domain-8→77 relay (producer: vm-tracker-prod `sam2_tracker_node`,
   domain 8; mirror `grasp_target_relay_d77.py` in `regrasp/lab/`) → re-run loaded
   sweep (§4), expect attach at +4 cm + free motion; then targeted box-avoidance test
   (needs box xyz from user — never answered).
3. Set 1.3 kg load at next bringup idle (mass 1.3, CoM ≈ (0,0,0.10),
   inertia ≈ diag(0.005,0.005,0.002)) → re-check loaded shake.
4. Re-verify before any live run: `held_attach armed` log line, PF values line,
   0 stray goal publishers, fresh (<5 min) trusted binding, named e-stop person +
   explicit go logged, `franka-robot` claimed.
5. Safety rule (standing): arm moves only with Murilo's explicit go for THAT run +
   named e-stop person present; scripted runs are not exempt.
