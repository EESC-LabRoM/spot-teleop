# Handoff — RAL lane: Acesso câmeras e dashboard + real FR3 (2026-10-30)

Lane: `RAL / Acesso câmeras e dashboard`. Chat closed 2026-10-30. Previous lane work was interrupted mid-task (sim→real deploy check); do NOT assume it finished.

## What was done (this lane, as reported in chat)

1. **Lab access IPs (lab PC = `hipolito-Precision-5860-Tower`)**
   - Tailscale IP: `100.88.134.127`, SSH `murilo@100.88.134.127`
   - Cameras (web_video_server): `http://100.88.134.127:8787/` (e.g. `/stream?topic=/zed1/zed_node/left/image_rect_color`)
   - Dashboard static page: `http://100.88.134.127:8788/`
   - 3D calib viewer: `http://100.88.134.127:8790/`, robot SSE `:8088`
   - Source of truth: [zed_lite/HANDOFF_REAL_LAB.md](zed_lite/HANDOFF_REAL_LAB.md:4)
   - Requires Tailscale. Verify: `ping 100.88.134.127`.

2. **HTTPS version via omnigent-vm (user was right, it exists)**
   - URL: `https://omnigent-vm.tail0994ab.ts.net:8443/` — reported HTTP 200, serves `FR3 Lab Live` dashboard, byte-identical to lab page.
   - Chain reported live at the time: `tailscale serve :8443 → http://127.0.0.1:18089` → SSH tunnel (PID 75272, up since 19:05) → `murilo@100.88.134.127:8788`.
   - Dashboard is HTTPS-aware (relative `snap?topic=...` + `fr3/` routes through tunnel, not direct `:8787`/`:8088`).
   - Caveats: page source still mentions dead `nexus:8443` (offline); from the VM itself the MagicDNS name did not resolve (Tailscale DNS disabled there), verified with `curl --resolve omnigent-vm.tail0994ab.ts.net:8443:100.84.207.115`. From a user tailnet device with MagicDNS it should open directly.
   - `https://100.88.134.127:8788/` does NOT work (plain `http.server`, no TLS); no `tailscale serve` on port 443 on the lab machine.

3. **Real-robot driver check → recovery → home (all authorized)**
   - Found: `franka.launch.py` (real hw, `robot_ip:=172.16.0.2`, `use_fake_hardware:=false`) up since Oct 28 in container `fr3-ros2`; control box ping 0% loss; but all 4 controllers `inactive`, mode-switch error in log, `franka_gripper_node` died (NetException); dashboard `:8088/stream` said cartesian impedance not active; lab load ~96.
   - Recovery run (user said "pode rodar"): cartesian_impedance_target_controller ACTIVE (Kt/Kr 1000/40), joint_state + robot_state broadcasters ACTIVE, gripper node restarted, teleop auto-engaged. Final home move SKIPPED at that step (arm held pose).
   - Home run (user said "manda o robo pra home"): reported `HOME OK`, 4 s move, max joint Δ 48°, clearances table +13.7 cm / self +11.4 cm, TCP `[0.309, -0.006, 0.480]` identity orientation, impedance active, teleop disengaged (press Start to drive).
   - Logs claimed on lab machine: `/home/murilo/recover_1793314300.log`, `/home/murilo/home_1793314600.log` — NOT re-verified in closing turn, check before citing.
   - Left known-bad: physical 8BitDo pad not found (`no 8BitDo pad found; waiting`) → drive needs pad reconnect or remote pad `:8089`.

4. **Load question (curobo + nvblox + zeds + segmenter under load?)**
   - Answer given: yes — all four already running (zed1/2/3 wrappers, 2× `franka_segmenter.py`, `nvblox_node` + web_video_server/mesh/seg_overlay, `curobo_mpc_node.py` idle `use_esdf:=false start_active:=false`).
   - Load spike was NOT ours: `phase2.py --num_envs 50` ~560% + Isaac `render6.py` ~74% + `agy models` ~47% + mujoco/Chrome; ours ~25–60% each (normal). GPUs 33%/12% with GBs free, 74 GB RAM free, load fell 96→45 during check.
   - Caveat given (from [fr3_real_repro/README.md](fr3_real_repro/README.md)): perception/mapping fine, but 50 Hz closed-loop needs EventsExecutor + cuRobo off nvblox's GPU; CPU contention adds 100–200 ms spikes. Tight real-time only off-peak / with `phase2.py` paused.

## What is left / open

- **Abandoned (user interrupted, do not resume unasked):** sim→real deploy gap analysis — mapping sim-validated items (GPU split cuRobo/nvblox, EventsExecutor, joystick script) vs lab PC state + what is missing. Last assistant msg only got as far as "Sim side mapped. Now checking lab PC" before interrupt.
- Reconcile with [fr3_real_repro/HANDOFF_RATE_DRIFT_2026-10-29.md](fr3_real_repro/HANDOFF_RATE_DRIFT_2026-10-29.md): that file already tracks EventsExecutor, retract_config drift fix, VM-103 cable runs — read it before redoing any deploy check to avoid duplicate motion commands.
- `nexus:8443` references in dashboard source/comments should be updated to `omnigent-vm` (or removed).
- Lab PC load still high due to someone else's `phase2.py` (Hipolito's) — coordinate before any 50 Hz control test.
- 8BitDo pad reconnect (or `:8089` remote pad) needed for teleop driving.
- Verify the two claimed log files exist before quoting them as evidence.
- Real-robot safety: any further `real-recover` / `real-home` / controller activation needs a person at the e-stop; lab PC is otherwise read-only unless user says go (per prior handoff conventions).

## Where things are

- This repo (`/home/nexus/spot-teleop`): `zed_lite/HANDOFF_REAL_LAB.md` (lab/ZED/nvblox truth), `fr3_real_repro/HANDOFF_RATE_DRIFT_2026-10-29.md` (MPC/rate/drift truth), `fr3_real_repro/README.md` (10-29 rate study).
- Lab machine (`murilo@100.88.134.127`): containers `fr3-ros2` (driver), `zed-build`/`isaac-build` (cameras/nvblox), dashboard on `:8788`, claimed logs `/home/murilo/recover_1793314300.log` + `/home/murilo/home_1793314600.log`.
- HTTPS path: this VM `tailscale serve :8443` + SSH tunnel PID 75272 (`127.0.0.1:18089 → 100.88.134.127:8788`); public name `omnigent-vm.tail0994ab.ts.net`.
