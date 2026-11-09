# FR3 avoidance: reproduce the real-robot failures in simulation (VM ubuntu-gpu, A2000)

Copied from nexus on 2026-10-27 (same absolute paths, user nexus). Reason: on the real FR3 (Hipolito's joystick MPC, 10-23)
the arm "stopped beforehand at times, then started hitting things". Findings from the real log are in nexus memory
`fr3-real-avoidance-diagnosis`; the raw log is on the lab machine in ~/franka_ws/hub-curobo/log/joystick_hipolito/.

## Here (checksummed against nexus: 647 code/config files identical)
- ~/spot-teleop/isaac-sim_ws/franka_mpc      sim scripts (run_obstacle.sh = the avoidance sweep), node_config (+ fr3_vcap1.2_acc10.yml from the lab)
- ~/spot-teleop/isaac-sim_ws/curobo-isaac6   cuRobo 0.7.7 source + private CUDA 12.8.93 (nvcc runs). The sm_89 .so files were NOT copied.
- ~/spot-teleop/zed_lite                     ZED receiver + nvblox-lite Dockerfiles/scripts, wrapper build, nvblox_msgs_jazzy
- ~/spot-teleop/spot-ros2_ws/.../curobo_mpc_node.py   the sim's MPC node
- ~/spot-isaac-sim/docker/Dockerfile         spot-isaac-sim base (FROM nvcr.io/nvidia/isaac-sim:6.0.0)
- ~/spot-teleop/fr3_real_repro/hipolito_joystick   his joystick_mpc.py (the node that ran on the robot), joystick_bridge.py, goal_math.py, command_guard.py
- ~/spot-teleop/fr3_real_repro/lab_node_config     the lab's real-run configs

## NOT here yet (rebuilt on the VM from public sources, the link nexus->VM is only ~2.7 MB/s; the VM downloads at 8-14 MB/s)
1. docker pull nvcr.io/nvidia/isaac-sim:6.0.0 (~21 GB, anonymous pull works)
   docker build -t spot-isaac-sim:6.0.0 ~/spot-isaac-sim/docker
   docker build -t spot-isaac-sim:6.0.0-zed -f ~/spot-teleop/isaac-sim_ws/franka_mpc/docker/Dockerfile.isaac-zed ~/spot-teleop/isaac-sim_ws/franka_mpc/docker
2. docker build -t zed-lite:5.4.1-humble -f ~/spot-teleop/zed_lite/Dockerfile ~/spot-teleop/zed_lite/ctx
   docker build -t nvblox-lite:3.2.5-humble -f ~/spot-teleop/zed_lite/nvblox/Dockerfile ~/spot-teleop/zed_lite/nvblox/ctx
3. cuRobo for the A2000 (sm_86 ONLY), inside the spot-isaac-sim container with Isaac's python.sh, CUDA_HOME=/workspace/curobo-isaac6/cuda-12.8.
   INTENDED, NOT YET RUN (the nexus build used uv + a py3.12 venv; the old CUROBO_INSTALL.md recipe is for Isaac 5): TORCH_CUDA_ARCH_LIST=8.6.
4. Caches (Isaac kit/ov) fill on the first run; the ZED SDK re-optimises its depth engines for the A2000 on the first run (minutes).

## Sim vs real: why the sim "worked"
| | sim runs so far (run_obstacle.sh) | real (Hipolito, 10-23) |
|---|---|---|
| ESDF voxel | 0.01 m (older runs 0.02) | 0.05 m |
| collision margin | 4 mm config + 0.03 extra | 4 mm total |
| MPC config | fr3.yml (accel 15, no velocity cap) | fr3_vcap1.2_acc10.yml |
| hand clear sphere | none | r 0.15 m at fr3_hand (permanent) |
| depth | NEURAL_PLUS, full resolution, exact poses | real ZEDs, 3-5 cm cross-camera disagreement |
| node | curobo_mpc_node.py | joystick_mpc.py (skips infeasible steps; collision is a cost, never a constraint) |
| goal | scripted sweep (wrist_path.py) | operator joystick |

## First experiment (8 runs): voxel {0.01, 0.05} x margin {0.004, 0.03} x hand clear sphere {0, 0.15}
Example, real-like: cd ~/spot-teleop/isaac-sim_ws/franka_mpc &&
  VOXEL=0.05 NODE_ARGS="-p extra_collision_sphere_buffer:=0.0 -p robot_config:=/workspace/franka_mpc/node_config/fr3_vcap1.2_acc10.yml -p target_clear_frame:=fr3_hand -p target_clear_radius_m:=0.15" ./run_obstacle.sh real_like_5cm
Score with avoid_check.py / body_contact.py (exact-mesh clearance, not the map). GPU: run only when the Explora benchmark is idle; Isaac 6.0 recommends 16 GB VRAM, the A2000 has 12 GB (use ZED_YAML=sim_static.yaml if it runs out).

## STATUS 2026-10-27 (end of setup)
- Images built from public sources (build_images.sh, ~20 min): spot-isaac-sim:6.0.0, :6.0.0-zed, zed-lite:5.4.1-humble, nvblox-lite:3.2.5-humble (layer counts identical to nexus).
- cuRobo built for sm_86 only (build_curobo_sm86.sh, ~4 min, 5 .so). Verified on the A2000 in the Isaac container with check_curobo_a2000.py
  (cuRobo's mpc_example): converged, pose error 0.0099, 390 steps, mean 6.3 ms/step (L40S: ~3.4 ms).
- NOT done: no Isaac/ZED/nvblox sim run yet. First run will be slow: Isaac shader cache + the ZED SDK optimising its depth engines for the A2000 (ZED_WAIT=1200).
- The Explora benchmark here is CPU-bound (GPU idle): a sim run (Isaac + 2 ZED + nvblox + segmenters + MPC) will perturb its CPU latency numbers. Run when it is idle.

## REAL=1 fidelity switch (added 2026-10-27 evening) -- supersedes the "First experiment" command above
`REAL=1 ./run_obstacle.sh ...` now puts the two virtual ZEDs at the 10-20 EasyHeC-calibrated REAL poses (real_cams.json; the 10-16 fit is kept as
real_cams.json.bak_1016), uses the real depth settings (sim_real.yaml: 640x360 NEURAL_LIGHT) and the real nvblox mapping parameters
(zed_lite/nvblox/franka_zed_nvblox_real.launch.py: all 44 params identical to the lab's nvblox_5cm.launch.py, checked by parsing; the notable ones:
TSDF max weight 10 (sim default 100), ESDF min weight 0.5 (sim: unset, so thin/grazing surfaces seen faintly drop out of the ESDF and count as FREE),
truncation 4 voxels, ESDF 5 Hz). Originals: run_obstacle.sh.orig, run_nvblox.sh.orig.
Real-like first run (grid = Hipolito's, margin = his 4 mm, hand clear sphere = his 0.15 m):
  cd ~/spot-teleop/isaac-sim_ws/franka_mpc && REAL=1 VOXEL=0.05 NODE_ARGS='-p extra_collision_sphere_buffer:=0.0 -p robot_config:=/workspace/franka_mpc/node_config/fr3_vcap1.2_acc10.yml -p target_clear_frame:=fr3_hand -p target_clear_radius_m:=0.15 -p grid_size_m:=[1.4,1.6,1.2] -p grid_center_m:=[0.35,0.0,0.55]' ./run_obstacle.sh real_like_5cm
STILL DIFFERENT FROM THE REAL RUN: (1) the scene: sim table/bottle/beaker/drill, not what was on the real table 10-23 (ask); (2) the sim TF is exact, the real cameras
carried a 3-5 cm cross-camera residual (could be added as a perturbation arm); (3) the cameras may have moved between the 10-20 calibration and the 10-23 trials;
(4) synthetic ZED X (Isaac asset) vs the real ZED 2i, rendered-image depth noise vs real; (5) cuRobo node = curobo_mpc_node.py, not his joystick_mpc.py, and a scripted
sweep instead of an operator (his node as a second pass).

## RUN 1 RESULT (2026-10-27 evening): real-like 5 cm run, REAL=1  (runs/real_like_5cm; copy on nexus: ~/fr3-avoidance-sim/run1_real_like_5cm/)
Arm penetrated objects: whole-arm collision-sphere surfaces min -2.8 cm (inside an object mesh), executed fingertips -1.8 cm, wrist/hand flange +4.3 cm; the blue bottle ends the run lying on its side,
`veja` moved 225 mm, bottle 13.3 mm (centre; the tipping is not counted). Nearest object at the minimum: veja. The node's own min_dist is dead here too (sphere #59, +0.000 on every line, node_min_dist_esdf_m_max 0.0).
CAVEATS: one run, no control yet; the drill was MISSING (spot-sim/assets/drill had not been copied; copied afterwards, the next run has it); exact sim TF; 0.016x real time (18.5 sim s = 19 min wall; a run is ~25 min).
Old nexus numbers from 10-14 (1 cm voxels, 3 cm margin: spheres >= +3.5 cm, nothing moved) are from my notes, not re-measured here.
TRAPS HIT ON THE FIRST RUNS: (1) cold start: the first ZED depth-engine optimisation takes ~15 min and Isaac gives up after --wait 900 s (use --wait 1800 the first time; cached in zed_lite/zed_resources afterwards);
(2) the two ZED wrappers started in parallel on an empty zed_resources race to seed it (cp 'File exists'): seed it once first; (3) if a wrapper hangs after "receiving port 30000 is not available ... Retrying" it picked the SHM-boost backend:
  docker stop zed-lite; sudo mv /dev/shm/sl_local_data_30000{,.hold}; sudo mv /dev/shm/sl_local_video_30000{,.hold}; restart it -> "No SHM-boost topic found ... Using UDP backend" -> connects (delete the .hold files afterwards);
run_obstacle.sh's own retry only fires after ZED_WAIT (so keep ZED_WAIT at the default 240 once the engines are cached); (4) analyze_obstacle.py crashes on an object with an empty mesh (patch: skip them).

## Status 2026-10-27 18:30 (see /home/nexus/fr3-avoidance-sim/HANDOFF.md on nexus for the full story)
Runs on the VM: real_like_5cm (run 1), real_like_5cm_light (run 2, SVGA/15 fps, ~11 min), real_like_5cm_light_m3 (run 3, +3 cm margin). All three hit the veja bottle the same way (spheres -2.8 cm); the 3 cm margin that fixed the old 2 cm sim setup does not fix this stack. Launchers: run_real_like_5cm.sh, run_real_like_5cm_light.sh, run_real_like_5cm_light_m3.sh (zed_lite/sim_real_svga.yaml). Next: MAP_ONLY pre-contact map capture, then bisect toward the old working config. Clear sphere is NOT applied in the sim (frame fr3_hand missing, sim uses arm0_).

## cuRobo MPC RATE (2026-10-29): can the node run at 50 Hz on the A2000? (real: 25-30 Hz on Hipolito's PC, same GPU)
Launcher `franka_mpc/run_newton_rate.sh <run>`: Newton FR3 (lockstep, no Isaac/ZED/nvblox) + saved 5 cm map via esdf_replay.py (runs/real_like_5cm/map_check/esdf.npz)
+ curobo_mpc_node.py with the REAL config (fr3_vcap1.2_acc10.yml, margin 0, grid [1.4,1.6,1.2] @ [0.35,0,0.55]) + wrist_path sweep, 18.5 sim s.
Newton deps on the VM: isaac-sim_ws/crawl-lab/IsaacLab/.venv/lib/python3.12/site-packages -> ~/newton_sp (symlink; its absence killed the 10-28 selftest).
Node: new env switch MPC_EXECUTOR in main() (unset = old behaviour; 'multi:N'; 'events' = rclpy.experimental.EventsExecutor). run_mpc_node.sh: optional DOCKER_EXTRA.
| run (CSV: 1 sample / 10 steps) | mpc.step med/p95 ms | loop med / p90 / p95 ms | loops <= 20 ms | sweep wall s |
| Isaac+2 ZED+nvblox runs (10-27/28, 10 runs) | 13.5-14.9 / 16-23 | 32-36 / - / 37-51 | 0 % | ~1150 |
| Newton, default MultiThreadedExecutor (5 runs) | 5.4-6.0 / 6-8 | 17-24 / 39-107 / 51-183 | 42-59 % | 28-52 |
| Newton, MultiThreadedExecutor(num_threads=2) | 5.8 / 7.0 | 21.1 / 57.5 / 91.2 | 47 % | 45 |
| Newton, EventsExecutor | 8.5 / 10.7 | 12.4 / 13.3 / 13.5 | 100 % | 12.6 |
Profile (py-spy inside the container, runs/rate_newton_5cm_prof4/prof.raw): 55 % of samples = the main thread in rclpy's Python wait-set loop
(executors.py _wait_for_ready_callbacks / event_handler add_to_wait_set), competing for the GIL; inside control_loop, mpc.step = 64 %.
=> the 25-30 Hz is NOT cuRobo: (1) GPU sharing with the camera/mapping stack doubles mpc.step (6 -> 14 ms), (2) the default Python executor adds 10-20 ms + 100-200 ms spikes.
EventsExecutor: same 924 commands, 0 lockstep timeouts, ~20 ESDF updates, median pose error 1.6 cm (default 12.5 cm: the node lagged).
Hipolito's joystick_mpc.py uses the same `MultiThreadedExecutor()` (line 1844) + create_timer at 50 Hz.
NOT yet shown: wall-timer mode (real robot) instead of /clock lockstep; ZED+nvblox on the same GPU with EventsExecutor; the real joint_states rate (franka broadcaster may be far above 100 Hz -> more executor load).

## RATE UNDER CAMERA LOAD (2026-10-29 evening), no Isaac: Newton + the lab's camera stack replayed on the same A2000
Load = per camera zed_lite/svo_stream.py (loops an SVO recorded from the sim ZEDs, runs/svo_record -> zed_lite/zed_settings/svo/rec_zed{,2}.svo2; zed3 replays zed's)
-> a normal sim-mode zed_wrapper with zed_lite/svo_load.yaml (= sim_real.yaml: NEURAL_LIGHT, 640x360 publish; but depth_stabilization 0 because the SVO has no IMU).
15 Hz HD1080 per camera (real: 30 Hz HD720; pixel rate ~93 vs ~83 Mpx/s for 3 cameras). nvblox = franka_zed_nvblox_real.launch.py 5 cm on zed+zed2 (+ self-filters).
The node still uses the replayed map (/esdf_replay/...), so control is the same as the no-load runs. `LOADCAMS="zed zed2 zed3" NVLOAD=1 SEGLOAD=1 ./run_newton_rate.sh <run>`.
| run | GPU util mean | mpc.step med/p95 | loop med / p90 / p95 ms | loops <= 20 ms |
| no load, EventsExecutor (rate_ex_events) | - | 8.5 / 10.7 | 12.4 / 13.3 / 13.5 | 100 % |
| 3 wrappers only, EventsExecutor (load_zed3_events) | 79 % | 12.0 / 14.3 | 15.8 / 17.7 / 19.3 | 97 % |
| 3 wrappers + nvblox + self-filters, EventsExecutor (load_full_events) | 87 % | 14.2 / 19.6 | 19.8 / 23.7 / 24.8 | 53 % |
| 3 wrappers + nvblox + self-filters, default executor (load_full_default) | 77 % | 8.0 / 13.4 | 24.2 / 35.8 / 46.2 | 26 % |
All 924 commands, 0 lockstep timeouts. The full-load default-executor case (~35-40 Hz) is close to the lab's 25-30 Hz (lab adds CPU load: load avg 38 seen there).
=> 50 Hz needs BOTH the EventsExecutor and cuRobo off the nvblox/self-filter GPU; with the 3 wrappers on its GPU it fits (p95 19.3 ms, thin margin).
Lab layout (HANDOFF_REAL_LAB.md): wrappers on GPU 0 (+ desktop), nvblox on GPU 1. nvblox uses 2 of the 3 cameras.
Caveats: lockstep (/clock-stepped node), CSV = 1 sample per 10 steps; one run per row.

> 2026-10-29 end of session: short handoff (state, open items, lab commands) in `HANDOFF_RATE_DRIFT_2026-10-29.md` (this folder).
