# Grasp loop (one script)

`./run_loop.sh --seed x1,y1,x2,y2 --label drill` runs seed -> mask -> cloud ->
grasp -> carve -> request. Default DRY (no motion). Each stage verifies its
output, saves artifacts to `runs/<ts>_<label>/`, and aborts the run on failure.

## Stages

1. **seed**: publishes ONE seed to `/perception/seed_command`, starts the
   seed-pixel bridge if quiet, waits for `seed_3d` + `TRACKING`. Never reseeds
   blindly (rapid reseeds crash the SAM video predictor).
2. **mask**: waits for a fresh `/zed1/segmentation_mask` with sane coverage
   (default 0.2%-15%). Saves `mask.png`.
3. **cloud**: captures 5 depth+mask pairs, backprojects masked pixels with TF
   into `fr3_link0`, workspace-filters. Saves `cloud.npy` + `cloud.json`.
4. **grasp**: ships the cloud to the lab, runs GraspGen-X on lab GPU1
   (`grasp_infer.py`, `.venv`, franka_panda), ships back `(M,17)` poses.
   Picks max `conf + 0.5*verticality - 0.5*offaxis`. Saves `pick.json`.
5. **carve**: ensures `vm-mask-sync` (`/vm/target_mask`) + `vm-nvblox-seg`
   (segmentation mapper) on the VM, verifies the target reads free and the
   table reads occupied. Saves `carve.json`. Prod map untouched.
6. **request**: DRY (default): reads live joints (read-only) + FK, checks the
   approach/down legs against the seg map (must clear) and the prod map
   (expected blocked at the grasp cell: no carve there), prints the live cmd.
   LIVE (`--live --estop "Name"` + typed `GO`): re-verifies the mask, then
   invokes the lab `run_grasp_drill.sh` with the pick as `GTX/GTY/GTZ`
   (close -> open -> lift, same as the proven drill runs).

## Files

- `grasp_loop.py`: the orchestrator (this is the only thing you run).
- `run_loop.sh`: container launcher (builds `vm-grasp-loop` once).
- `vm_mask_sync.py` + `run_vm_mask_sync.sh`: VM mask restamper.
- `grasp_infer.py`: lab-side inference, pushed per run (always current).
- `runs/`: per-run artifacts (`seed/mask/cloud/grasp/pick/carve/goal.json`,
  `cloud.npy`, `grasps.npy`, `mask.png`, `summary.json`).

## Contracts & safety

- VM ROS: domain 8, `vm_offload/cable.xml` (cable + SHM, self-peer).
- Lab SSH: `murilo@100.88.134.127`, key `~/.ssh/id_ed25519_lab_claude`.
- `--live` requires a named person at the e-stop, typed `GO`, teleop lane idle,
  and the target still tracked. Ctrl-C aborts (lab runner cleans up itself).
- Tracker: seed once per target; the drill moved => old pick/seed are stale.
