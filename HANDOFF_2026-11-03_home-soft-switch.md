# Handoff — RAL lane: Comando para posição home / soft-switch (2026-11-03)

Lane: `RAL / Comando para posição home` (hub branch `hub/comando-para-posi-o-home`). Ended with user = Murilo confirmed on the lab machine.

## Starting question and answer

- "What command do you run to go to the home position?" → from `fr3_real_repro/vm_real/`: `ESTOP=1 EP="<name>" ./home_run.sh` (`CHECK=1 ./home_run.sh` for checks only). "Go home" = direct move per [CLAUDE.md](/home/nexus/spot-teleop/CLAUDE.md).
- "Can it be run from Murilo's machine?" → No, not as-is. [home_run.sh](/home/nexus/spot-teleop/fr3_real_repro/vm_real/home_run.sh) is VM-side (local `vm-mpc`/`vm-relay`, local clearance docker, ssh INTO lab). No lab→VM ssh key exists (probed, BatchMode fails).

## What was done

1. **Diagnosed `go` refusal (both-inactive stranded state).** Live run showed `refused: controllers 'fr3_arm_controller=inactive cartesian_impedance_target_controller=inactive '`. Root cause: `vm_real.sh go` only handled SW=0/1/2; both-inactive fell into `else refuse`. Read-only lab check confirmed it (`robot_mode: 1`, only broadcasters active, no stray guard/forwarder).
2. **Auto soft-switch in [vm_real.sh](/home/nexus/spot-teleop/fr3_real_repro/vm_real/vm_real.sh) `go` (repo edit).** New SW=3: disengage, re-activate Cartesian hold, verify, then normal SW=1 swap. Restore runs BEFORE the lab forwarder starts, so a failed restore exits 1 with nothing started and points at lab `real-recover`. Header comment updated.
3. **Regression test [tests/test_go_controllers.sh](/home/nexus/spot-teleop/fr3_real_repro/vm_real/tests/test_go_controllers.sh)** (new, hermetic ssh/docker stubs, byte-compares the copy it runs). 5 cases: SW=3 recovery, SW=0/1 unchanged, JTC-missing still refuses, failed-restore refuses pre-forwarder. Result: pre-fix script fails exactly the new case; fixed script 15/15. Fakes emit the real driver's ANSI colors (cyan 96m names/inactive, green 92m active) after discovering the display-vs-grep pitfall below.
4. **Latent parsing bug caught and fixed (lab script).** First [soft_switch.sh](/home/nexus/spot-teleop/fr3_real_repro/vm_real/lab/soft_switch.sh) copy grepped RAW `list_controllers` output for `name=state` tokens that only exist after awk-tokenizing → refused everything (DRY run caught it; also fixed my probe quoting that masked the exit code). `vm_real.sh` `ctrl()` was already correct (tokenizes). Test fakes hardened to match.
5. **Lab-side [soft_switch.sh](/home/nexus/spot-teleop/fr3_real_repro/vm_real/lab/soft_switch.sh): no-motion hold restore for Murilo.** Installed on lab at `/home/murilo/franka_ws/lab/soft_switch.sh` (executable, `LAB-AST-OK`, DRY passes on live driver). Usage on lab: `ESTOP=1 EP="Murilo" ./soft_switch.sh` (`DRY=1 ./soft_switch.sh` for checks only).
6. **Folded soft-switch INTO lab [fr3_home_jtc.py](/home/murilo/franka_ws/lab/fr3_home_jtc.py) per Murilo ("smart script" like grasp deploy: switch, execute, release).** Entry now: JTC active → refuse; impedance inactive-but-loaded → re-activate + verify, then normal swap→move→swap-back; missing/failed → refuse to `real-recover`. Previously-working inputs hit identical lines. Verified: lab backup `fr3_home_jtc.py.bak_softswitch_20261103`, AST parse OK, container import OK (`IMPORT-OK True`, so `check_home_connection.py`'s `from fr3_home_jtc import Home` is safe), `DRY=1 ./fr3_ros2.sh real-home` → `HOME OK: dry run`. Grasp deploy script (`claude_trial/run_grasp_drill.sh`) does NOT call this file — untouched. Other references are comments, `.bak` files, or the `fr3_home_joints_effort.py` child-runner (benefits automatically).

## Current lab state (last read-only check)

- `fr3_arm_controller=inactive`, `cartesian_impedance_target_controller=inactive`, `robot_mode: 1`. Still stranded — no live restore has been run yet.
- No stray `contact_guard`/`curobo_to_jtc` processes on the driver.

## What is left (for next agent / Murilo)

1. Murilo runs on lab (`~/franka_ws/lab`, e-stop in hand): `./fr3_ros2.sh real-home`. Expect soft-switch log line → `HOME: cartesian... -> fr3_arm_controller` move → swap-back, impedance active. If `did not re-activate` → `./fr3_ros2.sh real-recover` first (Desk: clear error, unlock, Activate FCI).
2. Or re-run VM `ESTOP=1 EP="Murilo" ./home_run.sh` (SW=3 path). Live `go` with SW=3 has NOT been executed yet — stub-tested only.
3. Optional: `soft_switch.sh` remains as the standalone no-motion unstick tool for either flow.

## Key paths

- VM: `fr3_real_repro/vm_real/{home_run.sh,vm_real.sh,home_direct.py,rehearsal_q0.txt,tests/test_go_controllers.sh,lab/soft_switch.sh}`
- Lab: `/home/murilo/franka_ws/lab/{soft_switch.sh,fr3_home_jtc.py,fr3_home_jtc.py.bak_softswitch_20261103,fr3_ros2.sh}`, grasp flow in `claude_trial/` (do not touch; other lane active in `/tmp/franka-teleop`).
- Standing rules still apply: lab writes need Murilo's go; arm moves only with his go + named e-stop person; duet note about the fix already sent to sibling RAL lanes.
