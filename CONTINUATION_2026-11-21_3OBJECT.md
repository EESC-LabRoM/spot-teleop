# Three-object selector — continuation (2026-11-21 17:06Z)

## Goal and current state
Finish the dashboard selector and safe grasp path for drill, tennis ball and Sprite. Live selector/map switching exists, but target readiness is per-object. No robot commands were issued from this session.

- **Drill:** only established planner-valid grasp. Morning retry `20261121_050322_drill_oneshot_regen` completed pick/lift and loaded held model; Start followed. Secure hold was not independently verified; leash reset repeated (>50 mm, peak 71 mm). Operator released and returned empty/Home (`held=false`, gripper 79.97 mm, attached=false). Earlier `044956` pick slipped.
- **Sprite:** one supervised cap attempt missed (fingers closed to 0.2 mm). Later side-grasp/pick/place run `20261121_075900_sprite_selector` reportedly succeeded at operator-moved pose and returned Home/open. Treat as supervised trial only: API reports ~10 mm camera error versus 7.5 mm/side slack; fixed 60 N compressed PET to 24 mm. Prior real-cloud cap estimates disagreed 17.2 mm XY / 12.0 mm Z. Do not generalize to arbitrary positions or assume that force is acceptable for all bottles.
- **Tennis ball:** real grasp held 60.3 mm. ZED2 ray/ball-centre-plane errors at A=(0.420,-0.270): (+0.6,-3.4) mm; at B=(0.380,-0.150): (-3.5,-4.0) mm. ZED1 X error was -14/-11 mm. Ball Y remains locked pending validation of the ZED2-only path, map carve, grasp replay, payload model and broker/stage checks. This ground truth is from the selector lane, not independently reviewed here. Earlier RealSense wrist-to-hand hand-eye tests failed (5.2 mm top-down RMS; 6–15 mm rotated, impossible table heights); AprilTag ZED reconciliation does not validate that separate wrist transform.

## Latest live snapshot and resources
Read-only GET `100.84.207.115:8899/object-target` at 17:04Z still reports selected Sprite, generation 8, map says target carved/others retained, but tracker/map state `LOST:sprite`; `y_enabled=false`, `y_armed=true`, block reason says tracker/map lost and asks to reselect. Drill `y_validated=true`; ball false (5–16 mm camera error exceeds 6.5 mm/side margin); Sprite marked validated but currently blocked by LOST state. This is API state, not an independent robot-state check. Resource board at 17:03Z: robot, lab-nvblox and lab-zed-cameras free.

## Work/tests and coordination
Offline object-selector suite last run: 106 passed (backend 74, filter 9, proxy/API 5, UI 3, clearance 7, swept replay/route 8). Detailed geometry and caveats are in `isaac-ros_ws/src/spot_nvblox/fr3/teleop/object_selector_1121/backend/{BALL_SPRITE_VALIDATION.md,BOTTLE_VALIDATION.md}` and `object_selector_1121/README.md`.

Claude selector lane: `cc29b70dbb7844e5927a4e714b3f6977`. It reports push complete on `origin/murilo-curobo-nvblox-hand-20261101`: `6ca3371` (selector/Sprite side grasp/ball truth), `86b3513` (Y readiness requires TRACKING + ACTIVE map; y_armed/y_block), `27eef60` (handoff status). Author MHC-CodeSmith, no AI coauthor. Verified local `origin/...` ref at `27eef60`; local `/home/hipolito/spot-teleop` HEAD remains `e26e2ec` and is dirty/behind. Do not pull/rebase the broad dirty checkout. This continuation note is still untracked and is not part of those commits.

Morys report: `/home/nexus/spot-teleop-worktrees/hub-coordena-o-e-valida-o/REPORT_2026-11-21_early-morning.md`; drill retry outcome corrected to “pick/lift + Start, secure grip unconfirmed, empty/Home after leash resets.”

Next: commit/push this continuation note in an isolated worktree if appropriate; keep ball locked until the ZED2 estimator plus held-out `carry_BC.json` check and full grasp/map/payload gates pass. Sprite Y is runtime-blocked while tracker/map is LOST; reselect/reseed only when needed and require fresh TRACKING before Y. No live motion without fresh explicit operator go and current e-stop confirmation.
