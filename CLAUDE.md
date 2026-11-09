# Project rules (FR3 real-robot lane, RA-L revision)

## ONE nvblox map (rule set by Murilo, 2026-11-01)
- Exactly ONE nvblox map instance runs for the FR3 cameras. The cuRobo MPC's ESDF service AND the dashboard mesh (`mesh_server --node ...`) both read that same node. No second `nvblox_node` (viewer, candidate, test) alongside it: it wastes GPU/CPU and the dashboard then shows a map the planner never sees.
- A temporary comparison instance needs Murilo's go and is stopped right after use.
- The voxel size and parameters of the one map are decided by MEASURING the MPC on it (solve rate + clearance in the deploy sim, then real), not assumed. 2 cm for cuRobo has not been measured yet.
- Reference launch to compare against: lab `/home/hipolito/spot-teleop/isaac-ros_ws/src/spot_nvblox/launch/fr3_nvblox.launch.py` (2 cm, static_tsdf, depth masked, node `/nvblox_node`, the name the MPC asks for) and `spot_nvblox.launch.py`.
- Status 2026-11-01: NOT yet satisfied. `nvblox_curobo` (5 cm, planner) and `nvblox_human` (2 cm, dashboard :8788/:8795) are both live; the dead mesh_server for `nvblox_fixed` on :8789 was stopped 2026-11-04. Migration is pending; see calib_1031/HANDOFF_2026-10-31_calibration.md.
- Stopping/restarting either node still needs Murilo's go (the dashboard is shared with Hipolito).

## Long runs: background-first (prevents frozen chat)
- Never run a sweep/pipeline as one blocking tool call. Start it detached (`setsid`, logs + progress file in the run dir) so it survives turn end, then poll with short bounded checks (e.g. `watch_sweep.sh runs/<name>` or `tail` + `grep` with timeouts).
- A turn silent with zero new output for ~10 minutes is treated as stuck: the runner kills it. Keep turns chatty with progress lines so questions get answered mid-run.

## Standing safety rules (unchanged)
- Lab PC is read-only unless Murilo says go. The arm moves only with Murilo's explicit go AND a named person at the e-stop at that moment (re-ask when the attestation is older than ~20-30 min).
- Claim shared resources (`franka-robot`, `lab-nvblox`, `lab-zed-cameras`, `lab-gpu1`, `lab-mpc-test`) with sys_resource_acquire before use and release afterwards.
- Never stop/kill/restart other people's processes without go. No video of the return move unless asked; "go home" = direct move (`home_run.sh`).

## Team mode: RAL + franka_teleop are one team
- Pull `sys_duet_inbox` at the start of every turn and before any arm-adjacent step. A queued note older than one turn is a bug: read it, act, reply.
- Reply to every direct question with `sys_duet_say` to the sender's full session id, same turn if possible. Never leave a question to rot in queue.
- Never end a turn on a promise ("continuing now", "will check next"). Either the next tool call is already issued, or the turn states exactly what trigger resumes it.
- Joint blockers: `bbea50b` contents, MPC-infeasible gate, bridge ownership, tracker topics. If you touch one, post path + result + next step. If asked, check and answer, don't defer to Murilo unless it moves hardware.
- Decisions between lanes: propose option + trade-off, the other lane critiques or confirms. Murilo breaks ties and alone authorizes hardware.
