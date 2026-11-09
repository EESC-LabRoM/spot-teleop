# regrasp: continuous grasp target for the real FR3 (VM side) + 6D attraction in cuRobo

Owner: RAL lanes (Muse "Grasp e nuvem real" d8b74888..., coordination 67087ec6...). Started 2026-11-06 on Murilo's go:
"Program the loop, keep everything well-structured. The plan is to run this grasp inside cuRobo's potential
fields, replacing it with the 3D affordance so we have a 6D attraction."

## Goal
1. Phase A (this package): keep a VALID, reachable 6D grasp for the drill at all times. Regenerate only when the
   drill really moves; reuse the last grasp through SE(2) ICP when the move is rigid on the table; hold through
   occlusion. Never moves the arm.
2. Phase B: the live cuRobo MPC (lab `codex-fr3-live-mpc`, node `curobo_mpc_node.py`) switches its attraction field
   from `pf_mode:=object` (3D affordance: 20 % pull to the target_object point within 0.4 m, the default running now)
   to `pf_mode:=grasp`. That mode already exists: a 6D pull to `/grasp_target` (PoseStamped, `fr3_hand` in `fr3_link0`),
   with a smoothstep weight from 0.25 m to 0.02 m on |dp| + 0.1 m/rad * angle, lerp + slerp, and a hold after release.
   Sim 10-15: 1/5 lifted without the field, 5/5 with it. Phase B needs Murilo's go (MPC restart = hardware-adjacent).

## Package layout (fr3_real_repro/regrasp/)
| module | does | reuses |
|---|---|---|
| `contracts.py` | dataclasses + JSON (de)serialisers for everything below; the only place topic names live | - |
| `source.py` | polls the fused live cloud at ~1 Hz (`:8791/grasp.json` cloud + live meta), mask-paired ticks only | seg_view output, no viewer changes |
| `accumulator.py` | stacks ticks while static (2.5 mm dedupe, table cut, outlier <8 nbrs/12 mm, retuned for density), capped (~15k pts / 8 ticks) | dual2 recipe |
| `motion.py` | SE(2) ICP vs the ANCHOR (cloud of the last grasp run, fixed); trigger = >15 mm or >10 deg, stable 2 s, median count >= 70 % | ICP from 11-06 reuse152 check |
| `occlusion.py` | FK arm silhouette (from /joint_states) projected in each camera vs the HELD drill box; latch state | grasp_plan.fk, robotfit7_dz TF |
| `graspgen.py` | client: file-drop over scp to the lab server (cloud_<id>.npy in, grasps_<id>.npy out), one-shot fallback | grasp_infer.py |
| `server/graspgen_server.py` | lab GPU1, model loaded once, watches the drop dir; no ports/tunnels. Measure first: today's infers were 1.3-1.7 s incl. model load, so most of the ~35 s per run is ssh/scp/env startup | grasp_infer.py |
| `select.py` | gates (starved=False, score>=0.25, vert>=0.6, wrap 10-70 mm, off<=100 mm) + rank + PLAN CHECK per candidate (IK both flips, PRE reachable, joint margin >= 0.10 rad, >= 50 pts between pads, palm/finger) | grasp_plan.py functions (import, do not copy) |
| `publish.py` | atomic run dir (`grasp_loop/runs/<ts>_drill_loop*`), dashboard pointer, `/drill_cloud_state`, `/grasp_target` | - |
| `loop.py` | state machine below; one process, one log, heartbeat | everything above |
| `tests/` | offline tests on today's recorded runs (lying2 -> lying3 ICP = rigid; maskonly -> lying = tipped; occlusion replay) | - |

Planner = `grasp_loop/grasp_plan_lib.py` (DONE 11-06, bit-identical to the old CLI on 4 cases): `plan_grasp(g, mesh_V, depth=0, z_offset=0, map_z_bias=0, seed_q=None, lift=0.10, vmax=0.10, cart_step=0.01, wp_min_time=0.4) -> plan` (plan['problems'] == [] means valid; plan['hand_R'] + plan['grasp_pos'] = the fr3_hand pose for /grasp_target); raises `PlanError` on IK failure. Helpers `load_grasp(url)`, `load_mesh(url)`, `fk`, `ik`, `hand_checks`. `grasp_plan.py` is now a thin CLI over it (same flags/output); grasp_test.sh copies both into vm-viewer.

## State machine (loop.py)
`TRACKING` (static, accumulating) -> on a confirmed move: `MOVED` -> SE(2) ICP vs the anchor:
  rigid (RMSE < 12 mm, fitness > 0.6, z extent within 15 mm) -> `REUSE` (transform the grasp set, re-run the FULL select gate on the new cloud);
  otherwise or the gate fails -> `REGEN` (GraspGen on the accumulated cloud) -> `TRACKING` with a new anchor.
`OCCLUDED`: arm overlaps the HELD box in a camera, or the count drops below 70 %: freeze the accumulator, grasp and both carve masks. Data held
  indefinitely but marked untrusted (age); on clear -> forced revalidation (ICP vs the held cloud) before the target is valid again.
`LOCKED` (HELD_OBJECT): set by grasp_test.sh / the operator from close until an explicit release; no regeneration, `/grasp_target` NOT published.
Absence of points is never movement.

## Contracts
- `/drill_cloud_state` (std_msgs/String JSON, domain 8, 2 Hz): {state, run_id, anchor_id, age_s, trusted, n_pts, icp:{rmse,fitness,yaw_deg,t}, reason}.
- `/grasp_target` (geometry_msgs/PoseStamped, frame fr3_link0, pose of `fr3_hand` = the MPC ee_link): the PLANNED grasp hand pose
  (plan.json grasp_pos + the chosen flip's hand rotation, incl. --depth), published at 10 Hz ONLY while state in {TRACKING, REUSE-validated}
  and trusted. It stops on LOCKED/OCCLUDED/untrusted, so the MPC field releases (pf_grasp_timeout_s 0.5) and its hold keeps the last correction.
- Run dirs keep the existing format (cloud.npy, grasps.npy, pick.json, rank.json, summary.json, grasp_plan_input.json) + `plan.json` + `loop.json`.
- Lock file: `regrasp/state/regrasp.lock (repo path, visible in vm-viewer)` (VM) with PID + heartbeat; stale > 120 s is broken and logged.

## Phase B (MPC 6D attraction), after A runs clean
1. Relay `/grasp_target` domain 8 (VM) -> domain 77 (lab LOCALHOST): `regrasp/lab/grasp_target_relay_d77.py` (WRITTEN 11-06, not deployed). Two-context pattern of `/tmp/esdf_proxy_d77.py`, separate process in codex-fr3-live-mpc. Drops wrong frame / non-finite / |q| != 1 / outside the box [0.1..0.9, -0.7..0.7, 0..0.8]; never holds or invents a target. Tested on isolated domains 231->232: 2 valid forwarded, 3 invalid dropped.
2. Sim first: replay a recorded `/grasp_target` into the deploy sim with `pf_mode:=grasp`; check the solve rate and the clearance.
3. Real: MPC relaunch with `-p pf_mode:=grasp` (never `off`: parsed as a bool), on Murilo's go with a named e-stop person. Rollback = the same launch without it.

## Safety
The loop never commands the robot. grasp_test.sh keeps asking ENTER per stage. GraspGen server on GPU1 only after measuring the MPC p99 solve time idle vs during inference (claim lab-gpu1).
