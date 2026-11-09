# Real camera setup + real depth settings in the sim, and the one-Isaac-per-batch session mode (2026-10-17)

## What was changed in the sim (L40S)
- `real_cams.json` + `franka_ros_arm.py --real-cams`: the two virtual ZED X left lenses are placed at the EasyHeC-calibrated lab poses (fr3_link0 frame; sim `zed` = lab zed1, sim `zed2` = lab zed2), placement error 1e-16, the sim stands hidden (they are untyped-group children, so the cylinders are hidden directly). The sim TF (fr3_link0 -> *_left_camera_frame_optical) equals the lab's live TF to 3 decimals.
- `zed_lite/sim_real.yaml`: NEURAL_LIGHT, depth stabilization 30, min/max 0.3/10, published 640x360. The Isaac ZED X has no HD720: it streams HD1080 and the wrapper downscales by 3 (DEVIATION: the depth network sees a 1080p grab, the real one a 720p grab). Camera model is ZED X (real: ZED 2i), same 110 deg HFOV.
- `run_grasp.sh REAL=1`: those two + 2 cm voxels; writes `config.txt` into the run. `REAL=0` (default) unchanged.
- TRAP: `--zed-preview` (render products on a streaming camera) makes the ZED wrapper handshake time out; never use it in a full-stack run.
- `run_session.sh` + `franka_ros_arm_session.py`: ONE Isaac + ZED + nvblox + self-filter stack for a whole batch (`GRASPS=<dir> ./run_session.sh <run>`); per grasp the MPC node and goal source are restarted (node warm-up ~11 s), Isaac resets the arm, every object rigid body and the ROS state (worst object offset after reset 0.28 mm). `MAPRESET=1` (default) saves the nvblox static map after the first settle (`save_map`) and re-loads it (`load_map`) before each episode; `MAPRESET=0` lets the map persist. Handshake files in `runs/<run>/ctl`. `franka_ros_arm_session.py` is a copy of `franka_ros_arm.py` with an episode loop (`--episodes 1` = old behaviour); to be folded into the original.
- Analysis scripts: `map_region_stats.py` (free/occupied/unobserved fractions of a map dump), `thin_parts_check.py` (are the objects' tops in the map), `batch_summary.py` now accepts per-episode folders without `isaac.log`.

## Results (6 grasps of runs/b1_grasps, drill, MASKMODE=human, MASKBOX=0, REAL=1, FULL=1, LIGHT=1; n=1 per cell)
| grasp | r1: one Isaac per trial | rs1: session, map persists | rs2: session, map re-loaded |
|---|---|---|---|
| g00 | lift 143 OK | 143 OK | 142 OK |
| g01 | 111 OK | 140 OK | 112 OK |
| g02 | 0 | 0 (hand 73 mm short at close) | 0 |
| g03 | 0 | 30 | 0 |
| g04 | 148 OK | -0.1 (hand 22 mm short) | 137 OK |
| g05 | 133 OK | 79 | 32 |
| success (lift > 10 cm) | 4/6 | 2/6 | 3/6 |

- r1 failures g02/g03: the hand reached the grasp pose (0.4-0.9 mm) but the approach had pushed the drill (xy shift 93 / 43 mm); the gripper closed on air. These two grasps (same hand pose, x = 0.498, at the drill's +x edge) look marginal, independent of perception.
- Maps: fresh = 650-735 occupied / ~700 unobserved voxels in the dump region; persistent map: unobserved 700 -> 238 -> 187 -> 120 -> 31 -> 31 and 840-1090 occupied (the drill's footprint becomes observed-free after the drill is lifted away, +~350 occupied voxels, mostly table-surface layer and around bottle/beaker); `load_map` restores the unobserved footprint (670-704) but occupied stays 700-920.
- MPC collision cost during the approach (first half of the rows): r1 28-53; rs1 63-269; rs2 49-187. Evidence that the session maps put more obstacle cost on the approach; the cause of the residual is NOT identified.
- Thin tops at the real camera poses (static map, n=1): bottle `veja` top 6 cm 97 % mapped (sim cameras, 1 cm: 88 %), beaker rim 55 % (24 %). "mapped" = |ESDF| <= 1.5 voxel, a looser band at 2 cm.
- Timing: per-trial run 307 s (stages: Isaac boot+capture 68 s, perception stack 20 s, MPC warm-up+map 77 s, sim 95 s for 14 s at rtf ~0.16, teardown 45 s). Session: 6 episodes in 901-942 s after the stack was up (~150-165 s per episode with a pre and a post map dump each, MAPPOST=1), about 1.7-2x faster. Per-episode map dumps cost 20-30 s each and can be switched off.

## Not established
- Run-to-run noise (n=1 per cell: g03, g04, g05 flip between runs). The real-vs-sim-settings comparison needs the baseline (REAL=0, 1 cm, NEURAL_PLUS) with the same grasps and replicates; not run yet.
- Whether session mode with `load_map` is equivalent to a fresh run (3/6 vs 4/6, residual collision cost).
