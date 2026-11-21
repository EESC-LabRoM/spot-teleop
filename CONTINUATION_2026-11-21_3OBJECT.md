# FR3 selector / perception handoff — 2026-11-21

## Current live state (coordination lane report)

- Murilo chose the original coordinator + VLM stack. vm-object-selector is stopped/removed; dashboard :8899 selector route and Grasp-target card are removed. EESC selector commit d867415 remains pushed; repo removal was not decided.
- vm-coordinator and vm-vlm are Up; tracker and tfproj stayed Up. Latest report: coordinator TRACKING on the tennis ball, mask 0.618% at 21.3 fps, /wrist_map/target_state ACTIVE:tennis_ball. Drill is outside the ZED1 view, so it was not seeded. Tower nvblox_curobo + mesh_server were restarted to clear the erroneous carve. Perception-only; no arm/teleop/MPC commands.
- VLM reprompt is blocked: POST http://100.111.174.61:8000/v1/chat/completions is connection refused. Tailscale maps this address to nexus. vm-vlm already points there; the pipeline works up to that POST.
- Repo docker-compose.yaml defines vllm-server (qwen_server; qwenllm/qwenvl:qwen3vl-cu128; port 8000; NVIDIA; Qwen3-VL-4B-Instruct; GPU memory utilization 0.4). Lane confirms it is the intended Nexus service, but it is not running. Murilo/Nexus GPU owner must claim the GPU and start it, e.g. docker compose up -d vllm-server on Nexus. Do not start from this lane or on the FR3 VM.
- Current live Y/grasp readiness was not rechecked after removing the selector. Do not assume ball Y is enabled or safe. Earlier ball gate failed: 5–16 mm camera error vs 6.5 mm per-side slack.

## Object validation status

- Drill: prior planner validations passed, but historical real attempts included slip and repeated >50 mm leash resets. Drill currently out of view; reseed when visible and revalidate before any pick.
- Sprite bottle: one supervised side grasp/place succeeded at moved pose, run 20261121_075900_sprite_selector. Keep SUPERVISED TRIAL label; 60 N squeezed PET to 24 mm.
- Tennis ball: segmentation/map carve was verified on selector gen11 (mask 0.62%, 1,886 points; ESDF target points 100% carved, 0% remained in mesh, other objects retained). This was perception-only. Grasp is not validated; do not enable Y.
- ZED2 ball ray-plane errors at A=(0.420,-0.270): (+0.6,-3.4) mm; B=(0.380,-0.150): (-3.5,-4.0) mm. ZED1 X bias about -11 to -14 mm. Held-out B→C check is pending in ball_truth/carry_BC.json. Wrist-camera hand-eye is separate and remains poor on rotated views (5.2 mm top-down RMS; 6–15 mm rotated).

## Code, tests, repo

- SAM2 duplicate-seed crash (gen9) fixed: one seed_command publish per attempt. EESC-LabRoM/spot-teleop branch murilo-curobo-nvblox-hand-20261101 at d8674155be87a4b0ce4b081a9a93571967a7eabf, author MHC-CodeSmith, only selector_service.py + test_selector_service.py. Also pushed fork commit 462fa62 to murilo-vinicius04/spot-teleop:frankateleop.
- Tests: python3 -m unittest -v test_selector_service (10/10); py_compile and git diff --check passed. Fix was verified against EESC base 4691f8b in a clean temporary worktree.
- Detailed offline evidence: isaac-ros_ws/src/spot_nvblox/fr3/teleop/object_selector_1121/{backend/BALL_SPRITE_VALIDATION.md,backend/BOTTLE_VALIDATION.md,ball_truth/RESULTS.md,ball_truth/carry_BC.json,ball_calib/RESULTS.md}.
- Main checkout /home/hipolito/spot-teleop is still at local e26e2ec with extensive unrelated dirty/untracked files. Do not broad-stage, reset, or rebase it. This continuation file is an untracked local note.

## Next safe steps

1. Nexus/GPU owner starts the intended vLLM service; lane retests VLM reprompt and confirms a single-seed TRACKING result.
2. Get drill back into ZED1 view before reseeding it; verify mask and map carve.
3. Keep non-target objects in ESDF. Ball grasp remains locked until held-out calibration plus grasp, broker/stage, collision and payload checks pass. Any robot motion needs fresh robot-state checks and current explicit operator authorization/e-stop confirmation.
# Update after Gemini-backed VLM recovery (2026-11-21)

Correction to the earlier Qwen blocker: per Murilo, the original VLM now uses Gemini, not Qwen. The `vm-vlm` transport was changed to Gemini `generateContent` (default `gemini-3-flash-preview`); key comes from `~/.config/spot-teleop/gemini.key` or `GEMINI_API_KEY`, and is never logged. Prompt, parser, retry, services, topics, and coordinator contract remain unchanged; legacy `vlm_url` is unused. `py_compile` passed and `vm-vlm` restarted.

Verified perception-only chain: direct `/vlm/trigger_relocalize` returned bbox `[373,788,418,871]`, 3 grasps, confidence 0.98 in 5.1 s. Full reprompt (`set_object_prompt`, coordinator trigger) applied one tracker seed and initialized the video predictor once. No robot, teleop, or MPC commands. Four reported containers are Up. Details: `REPORT_2026-11-21_early-morning.md`, 16:47Z report.

**Current safety caveat:** the ball moved during the test. A stale snapshot briefly seeded background/table; the tracker was recovered, but the latest ball mask is bloated (~5.1% on `/zed1/segmentation_mask`) and dashboard `view_mask` is stale/zero due to mask-sync ROI. Do not press Y or treat tennis-ball grasp as ready. Latest reported tracking state is TRACKING, seed_3d `(-0.273, 0.146, 0.767)`; reseed from a fresh image after confirming the current ball location, then verify mask and map carve before any grasp assessment. Warm retriggers can stall in the 1.5 s seed-pixel gate; a cold tracker restart plus the coordinator's single-seed path restored tracking. No current robot action was reported; claims released.
## Broker / ball-grasp status update

Read-only lane report: lab `grasp_broker.py` is running and its socket responds, but its non-drill plan path still queries the removed selector at `127.0.0.1:8898/object-target`. It will fall back/fail because no current drill binding is active (binding is parked as `parked_20261121_073350`). `vm-regrasp-live` exited 12h ago; GraspGen runs per `grasp_loop` invocation and there is no persistent inference service. No tennis-ball grasp has been generated. Latest run found was drill-only (`20261121_072358_drill_loop_regen`). `held_model_sync.py` is running. To support ball planning under the original coordinator stack, repoint the broker's target/cloud source to coordinator tracking/live cloud and implement/validate ball grasp generation; tracking alone does not make ball grasp ready. No live changes were made by the reporting lane.
## Tennis-ball grasp adaptation (new lane report)

Current live perception is WRONG for grasping: ZED1 tracker is reportedly TRACKING the TP-Link box (mask coverage ~5.07%); the ball is around pixel (223,233), and the viewer cloud is the box. Keep tennis-ball Y blocked until a fresh Gemini/coordinator reprompt seeds the ball and the mask/cloud are visually and temporally verified. Murilo requested adapting grasping on the original stack (no selector): proposed work is `plan_tennis_ball` in `object_grasps.py` (sphere fit + top-down planner + tests) and repointing lab `grasp_broker.py` non-drill gates from dead selector :8898 to `/tracking_state`, `/wrist_map/target_state`, VLM object prompt, and mask/cloud age. These are planned changes, not yet reported complete. Lane claimed `omnigent-vm` + `lab-zed-cameras`; it said it will not restart teleop or move the arm. Require offline tests and fresh live preflight; never infer grasp readiness from TRACKING alone.
