# HANDOFF 2026-11-06: Isaac sim replica lane (VM103, domain 103)

Lane "Simulação Isaac ZEDs", session 01a1b0e5-102a-7000-9662-4197f3ef8e7f.
Times UTC unless marked. All work on VM103 (`omnigent-vm`, 100.84.207.115),
ROS_DOMAIN_ID=103 + LOCALHOST, GPU0. Live robot/pipeline NEVER touched
(all live containers still Up at close; sim cleared on the ~00:04-00:08Z
FR3 reflex — cause was real-stack ZED teleop, confirmed back to recovery lane).

## What was built (state at close)

- Full sim replica in `isaac-sim_ws/franka_mpc/simrep/`: warm Isaac session
  mode (one Kit boot, episodes/sweeps reset env only), 2 sim ZEDs at
  robotfit7 extrinsics + robotfit7 static TFs, self-filters, fan-ins,
  RGB relays, seg viewer, sim JTC follower, engaged stub, mesh server.
  No privileged sim data in the loop.
- Controller-swap contract with MPC lane (8a): exact live forwarder
  hash-pinned (`/tmp/sim_contract/direct_topic_fwd.py`, f72d8b1d), bare
  defaults; engaged-stub semantics; preview 50.0 Hz sim-time vs 46-47 live.
- Vendored live MPC brain (`/tmp/sim_contract/`): node 0bc48d5e +
  `mpc_override_live.yml` + `MPC_CLI.txt` (repo copy differs — do NOT use it).
- Calib-error injection: `simrep/calib_err.py` + `calib_err_H1.json`
  (truth-side ~10mm+0.15°/cam, 19.7mm cross-diff = measured AprilTag gap);
  `run_simrep_stack.sh` takes `CALIB_ERR=H1` (default exact). Verified hot 1:1.
  Depth scale in sim = 1.0 (live scales are corrections; applying them to
  true depth double-counts — killed grasp 01, fixed).
- Dashboard sim tab LIVE at https://omnigent-vm.tail0994ab.ts.net:8443/
  ("Simulation · VM103", `#sim` deep-link): ZED snaps, status pills,
  robot-seg overlays, drill mask, grasp-vs-cloud 3D, main 3D (map/robot/
  drill/cloud/TCP/pick). Served from `/tmp/nvblox_real/lab_dashboard.html`
  INSIDE lab container `isaac-build` (host copies are NOT served; do NOT
  copy host over container — they drifted). Tailscale HTTPS :9443 video,
  :9444 drill JSON, :9445 simdash. Backups `.bak_*_20261105` next to both copies.
- Contemporary notes: `runs/simrep/{SIMREP_CONTRACT,LIVE_SPEC,STAND1_REPORT,
  GRASP_SESSION_1105_MIDDAY,DASHBOARD_SIMTAB_1105,STACK_COLORFIX_1105,
  SMEAR_DRILL_1105,GRASP6_BOOT_1105,GRASP7_BOOT_1105,TRUTH_ISOLATION_1106}.md`.

## Key results

- STAND1: ep00 clean (925 cmds, 0.0007 rad track err, drill 0.0mm); ep01
  cold-start latch fixed by mirroring live manual `set_active` practice.
- Cost parity: bin coll_cost by pose_err (e64 method). Tracking bins show a
  real ~4-5x gap, sim HIGHER (~75-86 vs live ~17). sphere59/target_clear
  theory RETRACTED by both lanes (labeling artifact: -1000→free, min_dist=
  max with clear=0). Dont chase sphere 59.
- Grasp midday: 4 attempts, #04 lifted+carried for real then dropped on the
  long carry; drill left LYING. Fixed en route: follower 50Hz hold (RTF
  0.02→0.125), gripper total-width 0.015 + 4x repub, sim-time stage waits.
- grasp6/7: 7 scene objects, nvblox bbox table-only (floor gone), color
  remaps fixed (`rgb/` layout), hand meshes in viewer. MPC reach test
  `mpc_pre01` converged 0.132→0.007, 2890 steps, 0 faults, 0.0mm moved.
- TRUTH ISOLATION (the verdict): minimal Isaac-only stack, perfect ground
  truth — deterministic centered-vertical grasp planned CLEAN through the
  UNMODIFIED gate, executed S1-S6 (<0.01 rad), lifted drill +47mm (privileged
  pose). But GraspGen sampling+rank is a lottery on the SAME perfect input
  (100-roll: 0.71/46mm vs 0.45/129mm; 500-roll top-5 ALL refused; 64-grid
  18/64 CLEAN). Blocker = sampler/ranker, NOT perception/planning/execution.
  Fixing ZED calib alone will NOT fix grasping. Evidence:
  `runs/simrep/truth_{A_full,T1_live,T1b_500,T2_vis}/`, `runs/simrep_truth1/`.

## Left / next (user was offered a/b/c, no answer yet)

1. (a) Deterministic re-rank: keep GraspGen proposals, re-rank by
   gate-predictive features (off-center + verticality + bilateral) or seed
   deterministic centered-vertical candidates alongside (agnostic, no tuning).
2. (b) Sync `simrep/grasp_run_sim.sh` to the 6-stage live plan (it still calls
   7 stage names → any passing plan crashes) + real-home qhome
   (`qhome_pub.py` drives Murilo-home, 0.107 rad off; `truth_qhome.py` is the
   fixed variant). Also `s4_sim.py` density gate uses world-Z not approach
   axis (starves on tilted grasps).
3. Look-pose flow (bigger, real fix for slab clouds): swing arm aside,
   snapshot full drill, then grasp. T2 proved ZED-density visible-surface
   passes the gate when centered (13mm/vert 0.96 CLEAN).
4. Ops items: HD720 restart (~2x fps, ~5min incl. reseed — boot is 3-5min,
   not 15); GPU1 sim-window needs live-lane coordination (GPU1 = live
   nvblox+SAM2); session-stays-alive flag (every wrist-goal MPC test ends
   with session exit + reboot by design); `unlatch.py` empty-output ghost
   (zero impact so far); `simrep-truth-follower` orphan still Up (rm -f it).
5. Watch: robotfit7 STILL deployed (confirmed by grasp lane; calib_robot/
   table_* is offline analysis, ~2-3° tilt, no replacement without Murilo go).
   Sim tab needs one real-browser eyeball (lab headless Chrome TLS broken).
   `ros-domain-103` released at close; re-claim before commanding.

## Where things are

- Stack: `isaac-sim_ws/franka_mpc/simrep/run_simrep_stack.sh`
  (`STACK=simrep_grasp7 EPISODES=1 CALIB_ERR=H1 OBJECTS="" ... --wait 19800`).
- Grasp: `grasp_run_sim.sh <stack> <run>` (agnostic; venv default fixed),
  `grasp_replay_sim.sh` (debug only), `truth_exec_sim.sh` + `truth_*.py`.
- Runs: `runs/simrep_grasp{6,7}/`, `runs/simrep/grasp_g{5,6,7}_*`,
  `runs/simrep/viewer_runs/`, truth runs above. Videos in
  `runs/simrep/grasp_simg4_01/`.
- Live MPC/forwarder pins: `/tmp/sim_contract/` (world-readable, hashed).
- No test harness in this repo — verification was direct (endpoint/mesh/
  log checks). Scratch truth scripts also under `/tmp/truth_*g703*`.
