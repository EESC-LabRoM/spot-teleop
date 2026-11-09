# Newton hybrid: Isaac builds the map once, Newton runs the grasps (started 2026-10-17, overnight)

## RESUME HERE (written 2026-10-17 ~02:50, before a context compaction)
**Goal:** compare the sim at the lab's REAL camera poses + depth settings against the old sim settings, on the 6 drill grasps `runs/b1_grasps`, and make that cheap: Isaac builds the nvblox map ONCE, Newton runs the grasps
(physics + the UNCHANGED cuRobo MPC node + `grasp_goal.py`, map replayed). User (2026-10-17): run overnight, video only for the best runs, invest in Newton for later speed; HiveBoard (physical manipulation benchmark, assets URDF/USD, Isaac Lab envs in `isaaclab-hiveboard`) is the later target, not tonight.
**Decided:** Newton as a ROS drop-in for `franka_ros_arm.py` (same topics, lockstep, episode handshake) + `esdf_replay.py` for the map; MPC node/goal/ROS interface unchanged. Isaac session mode (`run_session.sh`) is NOT equivalent to fresh-per-trial (rs1 2/6, rs2 3/6 vs r1 4/6, r2 3/6).
**Done (see `SESSION_MODE_2026-10-17.md` for the Isaac side, results table):** real_cams.json / `REAL=1`; session mode with `MAPRESET`; `map_real_pristine/esdf_full.npz` (the pristine real-settings map, node-grid crop 222,108 neg voxels vs the node's 222,666); Newton node + `run_newton_session.sh`.
**Newton results so far (g00, g01; map replayed):** newton_prof1: lift 140.6 / 113.1 mm (Isaac 143 / 111-140); hand-target error at close 2.6 mm (Isaac 1-5); finger end 19.8 mm (Isaac 20.3). Reset exact (0.00 mm). g02-g05 NOT yet run in Newton.
**OPEN / next, in order:**
1. `newton_prof2` is RUNNING (PROF=1, py-spy on the node, 20 s at ~90 s): read `runs/newton_prof2/ep00/prof.raw` with `python3 prof_summary.py ...`. QUESTION: why is the node's control loop ~95-135 ms median in Newton (rtf 0.08, 170-190 s per 14 s episode) vs ~22 ms in Isaac (rtf 0.11-0.16)? NOT the ESDF (same without it, USE_ESDF=false), NOT CPU/mem (16 cores, load 4); mpc_step is 4.5 ms; GPU 42 % busy (my own Newton+MPC, `laya-test/reflex` holds 12.9 GB but is idle). If py-spy fails ("Failed to find python version") try `py-spy dump`/`--native`, or time the loop pieces another way.
2. Run all 6 grasps in Newton (`GRASPS=runs/b1_grasps`), compare with r1/r2 (g00,g01,g04 lift; g02,g03 topple the drill = barrel grasp pose; g05 mixed). Then repeats + `JITTERS=` for statistics, then parallel instances (DOM/TAG vary: e.g. DOM=32 TAG=N2) once the per-episode time is understood.
3. The baseline for the original question (REAL=0: NEURAL_PLUS, 1 cm, old poses) was NOT run; needs another pristine map (`run_grasp.sh MAPONLY=1 ESDFFULL=1`, `VOXEL=0.01`) -> Newton batch. Decision to ask the user: replicate study (36 episodes ~1.7 h in Isaac session mode) vs Newton.
4. Commit to the branch (worktree `/home/nexus/spot-teleop-nvblox-lab`, branch `hub/envio-do-nvblox-para-lab`, last commit f93e8f3 + a later one with these files; NOT pushed). Do NOT commit `franka_mpc/cache_newton_home/` (274 MB) or `runs/` heavy files.
**Commands:**
- map once: `REAL=1 FULL=1 LIGHT=1 MASK=1 MASKMODE=human MASKBOX=0 MAPDUMP=1 MAPONLY=1 MAPSECS=30 ESDFFULL=1 REUSE=runs/grasp_full1 ./run_grasp.sh <name>` -> `runs/<name>/esdf_full.npz`
- Newton session: `GRASPS=runs/b1_grasps ESDF=runs/map_real_pristine/esdf_full.npz DOM=31 TAG=N1 [USE_ESDF=false] [PROF=1] [JITTERS='0,0,0;0.01,0,2'] ./run_newton_session.sh <name>` (needs >= 2 grasps; Isaac stack must not run on the same DOM; check `docker ps` clean before launching; kill by PID, never edit the script while it runs)
- Newton tracking self-test: `./run_newton_selftest.sh [--arm-ke ... --arm-kd ...]`; synthetic test map: `make_test_esdf.py`.
**Measured (expensive to find again):** Isaac per-trial 307 s (boot 68 | stack 20 | MPC warm-up+map 77 | sim 95 | teardown 45); session 6 episodes ~15 min. Newton single-world step: 7.3 ms/substep eager -> CUDA graph (collide+solve, 2 x 6 substeps) 0.45 ms (16x), a 14 s episode's physics ~4 s. Arm drive: POSITION-only gives lag kd/kp*v (31 mrad on a 0.5 Hz sine); `JointTargetMode.POSITION_VELOCITY` on the 7 arm joints: 7 mrad; gains x8 unstable; defaults ke 30000/24000/12000, kd 1500/1200/600, 12 substeps per 20 ms. MPC node warm-up is 7-12 s PER STEP if the ESDF exists during warm-up (Isaac warmed up before the first map): `esdf_replay.py --gate-file` withholds the map until "Warm-up done" (the shell touches `ctl/esdf_go`). Isaac `--zed-preview` breaks the ZED handshake in full-stack runs. ZED X in Isaac has no HD720 (HD1080 / downscale 3 = 640x360). nvblox voxel count rule: n = floor(max/v) - floor(min/v) + 1 (81x81x61 for the node grid). Newton sim must HOLD (wait up to 120 s) for the first command once a goal flows (fixed in `lockstep`), else the 14 s window passes before the node answers. Newton in the Isaac image: host site-packages `crawl-lab/IsaacLab/.venv/lib/python3.12/site-packages` first on PYTHONPATH; no git in the image (cached Panda asset used); caches copied to `franka_mpc/cache_newton_home/`.
**Isaac results (6 grasps, lift mm):** r1 143/111/0/0/148/133, r2 145/111/29/32/136/80, rs1 143/140/0/30/0/79 (persistent map), rs2 142/112/0/0/137/32 (map reloaded). Videos: `runs/r2_g0N/video.mp4`, sheets `runs/r2_contact_sheets/`, figure `runs/grasp_timelines_r1_rs1_rs2.png`.


User guidance (2026-10-17): record video only of the best/relevant runs; run overnight; build the map once in Isaac/nvblox, save it, run the grasps in Newton
(or run fully in Newton incl. camera if viable) to escape Isaac's slowness; invest now to gain speed later. HiveBoard (physical manipulation benchmark, Isaac Lab
envs in `isaaclab-hiveboard`, URDF/USD assets in its `Simulation/` folder; no Newton, no Franka in its landing pages) is the later manipulation target: not part of tonight.

## UPDATE 2026-10-17 ~03:00 (after the first Newton 6-grasp batches; supersedes "Newton results so far" below)
**Newton contact calibrated: friction mu 0.8 (new default of newton_ros_arm.py).** Open-loop REPLAY (`run_newton_replay.sh <trace.npz[,trace2...]> [--mu ..]`, `--replay` in newton_ros_arm.py: feeds the recorded q_cmd/qd_cmd + the grasp_goal gripper ramp through the physics only, ~6 s per stream, the gripper ramp is aligned to goal.log's start) of the 12 Isaac r1/r2 command streams:
mu 0.5 (Isaac's PhysX default) fails g04/g05 (drill slips out in the lift); mu 0.65 agrees with Isaac pass/fail on 10/12 streams, **mu 0.8 on 11/12** (miss: r2 g05, Isaac itself only 80 mm), mu 1.0 on 7/12 (lifts g02/g03, which Isaac never lifts). Isaac streams at mu 0.8, 10 repeats each: g04 10/10, g05 10/10, g02 0/10, g03 0/10 (= Isaac r1).
**Closed loop (real MPC node, replayed map), 6 grasps:** mu 0.5 lifts 141/116/26/0/33/33 mm (2/6); mu 0.8 nominal 147/131/26/26/26/32 (2/6: g04/g05 FAIL at the exact nominal pose); mu 0.8 with 3 sessions of small drill offsets (up to 5 mm, 3 deg; `JITTERS`), 3 draws per grasp: **g00 3/3, g01 3/3, g02 0/3, g03 0/3, g04 3/3, g05 2/3 (11/18)** = Isaac's pattern (g04 148/136, g05 mixed 133/80/79/32). g04/g05 are marginal at the mm level: replaying Newton's own nominal g04 command stream gives 7/10, g05 0/10, while the hand-to-drill geometry at closing is within Isaac's r1-vs-r2 scatter. Conclusion: judge grasps by pass RATES over jittered draws, never by one run. Replay is NOT bit-deterministic (GPU atomics): identical inputs flipped pass/fail within one container.
**Throughput:** 3 parallel Newton sessions (DOM 31/32/33, TAG J1..J3) take the same ~175 s per episode as one alone -> 3x. A negative first jitter needs a leading space (`JITTERS=" -0.004,..."`) or argparse reads it as an option. Idle prelude (150 s at retract) before the replay changes nothing.
**Still unexplained:** the node loop is 31 ms median in one episode and 113 ms in the next (same setup); py-spy never produced prof.raw (fell behind, killed with the session). Not blocking: 3 parallel instances hide it.
**Next:** pristine map with the OLD sim settings (`REAL=0 ... ./run_grasp.sh map_old_pristine`, running 03:0x) -> the same jittered Newton batches on it (`ESDF=runs/map_old_pristine/esdf_full.npz`) -> compare pass rates old vs real settings. Disk: 96 % of /home used (23 GB free): check df before big runs.

## RESULT 2026-10-17 ~03:20: sim settings real vs old, Newton (mu 0.8), same 3 jitter sets, 3 draws per grasp
Old = REAL=0 (NEURAL_PLUS, 1 cm voxels, original camera poses; pristine map `runs/map_old_pristine/esdf_full.npz`, 42 MB, captured by `REAL=0 FULL=1 LIGHT=1 MASK=1 MASKMODE=human MASKBOX=0 MAPDUMP=1 MAPONLY=1 MAPSECS=30 ESDFFULL=1 REUSE=runs/grasp_full1 ./run_grasp.sh map_old_pristine`; Newton sessions need `VOXEL=0.01`). Real = REAL=1 (NEURAL_LIGHT, 2 cm, calibrated real cameras).
Passes (lift > 100 mm): **real 11/18, old 10/18**. Per grasp real/old: g00 3/3, 3/3; g01 3/3, 3/3; g02 0/3, 0/3; g03 0/3, 0/3; g04 3/3, 3/3; g05 2/3, 1/3 (the marginal grasp). NOT distinguishable at n=3. Figure: `runs/newton_vs_isaac_lifts.png`.
Caveats: the map is a replayed STATIC map from Isaac, so the settings act only through the MPC's collision map (no perception noise during the run, no human-node dynamics); grasp outcome is dominated by contact, which is the same in both. A perception-driven effect (moving target, ghosts) would need Isaac. The old-settings sessions first failed with `ESDF grid shape mismatch [161,162,121] vs [161,161,121]`: the replay's floor tolerance (1e-6 voxel) was below the float noise of the node's request at 1 cm; now 1e-3 voxel (esdf_replay.py).
Wall per Newton episode ~170 s (rtf 0.08) with 3 parallel sessions = ~1 episode / 57 s overall. Disk 96 % (23 GB free).

## Where the Isaac time goes (measured, per-trial mode, 307 s)
Isaac boot+capture 68 s | perception stack 20 s | MPC warm-up+map 77 s | sim 95 s for 14 s of sim (rtf ~0.16: rendering the ZED streams + lockstep with the MPC) | teardown 45 s.
MPC itself: median step 5 ms, node loop median ~25 ms (mean 65 ms: the 1 Hz ESDF update spikes to 0.5-0.8 s). So a Newton sim without rendering and with a replayed
(static) ESDF should run an episode in roughly 15-25 s, and several can run in parallel (separate ROS domains).

## Design (keeps the MPC node and grasp_goal.py UNCHANGED, so their behaviour is identical to the Isaac runs)
1. `zed_lite/nvblox/esdf_capture.py` (nvblox image): call the nvblox service the way the node does, on a larger box, save `esdf_full.npz`. Hook: `run_grasp.sh ESDFFULL=1`.
2. `esdf_replay.py` (Isaac image python, Jazzy): serves `/nvblox_node/get_esdf_and_gradient` from that npz (crop to the requested AABB on nvblox's voxel lattice).
3. `newton_ros_arm.py` (Isaac image + the host Newton 1.5 site-packages; verified: newton 1.5.0, warp 1.16, mujoco_warp, coacd, rclpy import together): a Newton FR3 + hand + table + drill
   (model from `newton_grasp_batch.py`) with franka_ros_arm.py's ROS interface: /clock, /joint_states_isaac, /joint_command_curobo, /wrist_pose, /gripper_width_cmd, lockstep, episode
   loop with done_/reset_/ready_ handshake, same RESULT_isaac.json / trace.npz keys so `batch_summary.py` and the plot scripts work.
4. `run_newton_session.sh`: esdf_replay + Newton + per grasp the MPC node and goal source.
5. Validate against the Isaac outcomes of the same 6 grasps (r1, r2, rs1, rs2: g00/g01 lift, g02/g03 fail by pushing the drill, g04/g05 mixed/slip). Then scale: many episodes, jitter, parallel domains.
6. Newton camera (SensorTiledCamera: depth/colour/shape-id, ground truth) is viable but gives PERFECT depth: it cannot reproduce NEURAL_LIGHT's errors unless a depth-degradation
   model is fitted from Isaac+ZED depth vs ground truth. Later.

## Progress log
(see the bottom of this file; newest last)

### 2026-10-17 night, log
- Newton 1.5.0 + warp 1.16 + mujoco_warp + rclpy import together in the Isaac image (host site-packages mounted in front). No `git` in the image: the cached Panda asset is used directly.
- `newton_ros_arm.py --selftest` (no ROS): the single-world step was 7.3 ms/substep (138 substeps/s) -> a 14 s episode ~60 s, as slow as Isaac. Capturing collide+solve in a CUDA graph
  (as Newton's hydroelastic examples) gives 2200 substeps/s (16x): a 14 s episode's physics ~4 s. Identical tracking numbers with and without the graph.
- Arm drive: in POSITION-only mode MuJoCo adds kd*(0 - qd): a lag of kd/kp*v (31 mrad mean on a 0.5 Hz sine). `JointTargetMode.POSITION_VELOCITY` on the 7 arm joints adds a velocity actuator
  that tracks the commanded qd (the MPC node sends pos+vel): 7 mrad mean, 3-7 mrad per joint at steady state, spikes <= 36 mrad only in the first steps. Gains x8 go unstable, x1-x2 are the same.
  Defaults kept: ke 30000/24000/12000, kd 1500/1200/600.
- esdf_capture.py / esdf_replay.py written; the replay reproduces nvblox's grid shapes (81x81x61 for the node's 1.6 x 1.6 x 1.2 m at 2 cm; 26x31x23 for the earlier map dumps).
  `make_test_esdf.py` builds a SYNTHETIC map (table slab + the real dumped grasp region) for pipeline tests only.
