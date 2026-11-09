# HANDOFF 2026-11-06: dashboard executor switch (cuRobo <-> Cartesian) + driver/MPC fixes

Lane: driver/recuperação Franka + dashboard (session 076ac24b270644f8b9eb1a46f6301ab0).
Times UTC. Robot: real FR3, lab host + this VM (omnigent-vm). ROS domains: 77 = control (lab-local), 8 = perception (lab host <-> VM).

## Where things stand (end of session)

- **Robot**: last seen recovered and holding (JTC `fr3_arm_controller` active, Cartesian parked inactive, `robot_mode 2`). Claim RELEASED — grasp lane (`10098820a59f44898914e02d12d03c51`) holds `franka-robot`. Do NOT touch the arm until they release. Final recovery of the night was done by the grasp lane (in-place, joints changed 3e-5 rad).
- **Executor switch is LIVE on the main dashboard**: open https://omnigent-vm.tail099004ab.ts.net:8443/ — Executor card (cuRobo + MPC / Cartesiano), disengaged-only, ~40-60 s per switch (relaunches teleop, no motion). Verified with a full live round trip (cartesian -> curobo, zero arm motion, TCP unchanged).
- **Backup UI**: https://omnigent-vm.tail099004ab.ts.net:8444/ -> lab :8091 sidecar (same code family, older rev). Keep as fallback.
- **Ghost drive (00:05-00:15Z)**: NOT a lane. `real-recover` auto-engage fired with default `input_source=zed` while Murilo stood in ZED view at the e-stop; camera drove 8,448 Cartesian setpoints until Recovery lane disengaged. Structural hazard logged for `fr3_ros2.sh` owner: change default to joystick and/or engage only on Start press. Relayed to Hipolito's teleop lane.

## What was done this session (all verified live)

1. **Driver recovery (morning)**: `error_recovery` SUCCEEDED x2 but `cartesian_reflex` re-latched in ~2 s each time; Desk restart fixed nothing until FCI unlock; then `fr3-ros2` container restart cleared a `TCP I/O error / Net Exception` (stale FCI session). Resume chain green: HW active, JTC active, `robot_mode 2`. Reloaded JTC from lab `fr3_arm_jtc.yaml` with documented soft gains (P 150/150/150/150/80/60/40, D p/10, `allow_nonzero_velocity_at_trajectory_end=true`, `cmd_timeout 0.2`, 0.08 rad tol) after the stock bringup spawned MoveIt gains (600/30, 4x too stiff -> reflex trips).
2. **Relaunched the 8a stack** (closed lane): goal relay + forwarder `/tmp/direct_topic_fwd.py` + teleop with historic flags, inside `fr3-ros2` (host Python has no ROS). Reset stale MPC goal to measured hand pose.
3. **MPC -> GPU1**: cloned `codex-fr3-live-mpc` to GPU1 (`CUDA_VISIBLE_DEVICES=1`), parked old as `codex-fr3-live-mpc-gpu0` (rollback). Preview 44.2 Hz w/ 82 ms spikes -> **49.6 Hz, max gap 44 ms**. Note: MPC boots standby (`start_active:=false`); re-arm with `/fr3_joystick_mpc/set_active := true`.
4. **ESDF bridge restored**: `/tmp/esdf_proxy_d77.py` (domain 77 -> 8) had silently died; restarted inside the new MPC container via `python.sh` (bare kit python lacks numpy). MPC logs `ESDF voxel grid updated` again. The `vm-esdf-proxy /nvblox_human` "not ready" spam is a separate pre-existing quest-sim issue (domain 143) — left alone.
5. **cuRobo vs Cartesian comparison**: recorded run 1 (52.9 s, 5.45 m, peak effort 36 Nm j4), replayed exact TCP on Cartesian impedance (force ceiling 1000 N/m x 0.02 m = 20 N/axis). Result: Cartesian cut 26% path, 24 vs 36 Nm peak, 6 cm RMSE; neither tripped. Caveat: replay was blind (no live avoidance). Bags saved with the run data.
6. **Dashboard executor switch (v9 -> v12)**: staged in `/tmp/dash_sidecar` on lab host (mounted `/lab`), deployed v12 to live `:8089` file `/home/murilo/franka_ws/lab/fr3_zed_runtime/fr3_web_dashboard.py` (= `/lab/...` in container; backups kept). Launcher: `/home/murilo/franka_ws/lab/start_remote_dashboard.sh`; watchdog is nexus-side `/home/nexus/.local/bin/fr3-dash-watchdog.sh` (PID 1151, do NOT fight it — it only restarts the launcher path). v12 enforces: kill-teleop-before-swap, no-swap-while-engaged (re-checked per stage), both controller states verified, MPC state restored on rollback, backend observed, JTC hold re-seed + refuse if > 0.02 rad divergence, fail-closed on sha/ack mismatch.
7. **Dashboard page**: Executor card added to container-canonical `/tmp/nvblox_real/lab_dashboard.html` inside `isaac-build` (mesh `:8788` serves it per-request; host copy intentionally diverged — do NOT copy host-over-container; sim tab lives only in container copy). Backup `.bak_executor_20261105`. Auth: `:8089` allow-list now includes both `murilo.mv4321@gmail.com` and `matheuzu11@gmail.com` (browser tailnet identity, not the person at the keyboard) via `REMOTE_USERS`; fixed the `SyntaxError` crash by rendering server text instead of parsing as JSON.
8. **Forwarder root-cause fix (grasp lane, 21:13:44Z)**: `/tmp/direct_topic_fwd.py` slew limiter kept last pose across disengage -> 1.5 rad/s jump on next Start. Fixed: re-seeds from measured joints on every engage edge, refuses if MPC cmd > 0.05 rad off. Backup `.bak_staleslew_20261105`. Dashboards restarted after to pick up the new sha. **Do NOT restart the forwarder from the old file.**

## Key numbers (don't re-derive)

- Teleop speed caps: linear 0.20, vertical 0.16 m/s x `speed_scale` 0.625 = **0.125 m/s effective**. MPC allows 1.2 rad/s/joint; Cartesian has no speed cap (20 N force ceiling only). Slowness = teleop shaping, not executors.
- Reflexes this period were `cartesian_reflex` (force), never joint-limit violations. Force service knobs exist (`set_full_collision_behavior`, `set_force_torque_collision_behavior`) but left at defaults deliberately.
- Lab host was overloaded during stutter (load ~41-45 on 16 cores, GPU0 88 C throttled, GPU1 idle): sim quest + hand tracker + Chrome + MuJoCo. Ask peer lanes before touching their processes.

## Left / next

- **Ghost-drive hazard**: `fr3_ros2.sh` `real-recover` auto-engage + `INPUT_SRC` default `zed` = unattended self-drive. Needs owner fix (default joystick / Start-press engage). Hipolito's lane has the note.
- **Offer standing with Murilo**: raise teleop speeds (linear 0.30, scale 0.8) — needs his go + e-stop person.
- **`:8091` sidecar vs `:8089`**: copies diverged (sidecar is older rev); future host edits need re-copy into `fr3-ros2`. Watchdog keeps `:8089` on the launcher path — deploy via file swap + launcher restart, never kill-loop.
- **ESDF watch item**: occasional 8 s upstream timeouts from VM nvblox (2/50 calls) — tolerated, but VM nvblox CPU is the next lever if they grow.
- Parked rollback container `codex-fr3-live-mpc-gpu0` can be removed once GPU1 MPC proves stable over a few drives.
- F_ext recorder + collision-threshold work: see [night-grasp handoff](/home/nexus/spot-teleop/HANDOFF_2026-11-05_night-grasp.md) — that file owns calibration/grasp state; this file owns dashboard/driver/MPC state. Where they conflict on tonight's events, this file is newer.
