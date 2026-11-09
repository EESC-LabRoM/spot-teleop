# Handoff: direct cable VM 103 <-> lab PC (lane "Deploy via VM 103", 2026-10-29/30)

Goal from the user: use VM 103 to deploy to the Franka (FR3) over the physical cable to Hipolito's lab PC, NOT over Tailscale (Tailscale path is a DERP relay via "sao", ~14 ms, no direct connection).

## State: network link done and tested; nothing ROS-side for the real arm is connected yet

Topology (all on `10.0.0.0/24`, no gateway, no default route):

| Machine | Interface | Address |
|---|---|---|
| Proxmox host `pve` | `vmbr1` (bridge on `enp1s0`, 10G) | 10.0.0.1 |
| Lab PC (hipolito-Precision-5860-Tower) | `enp1s0` (Aquantia 10G) | 10.0.0.2 |
| VM 103 (ubuntu-gpu) | `enp6s19` (virtio, `net1` on `vmbr1`) | 10.0.0.3 |

The Franka link is separate and untouched: lab PC `enp0s31f6` 172.16.0.1 <-> robot 172.16.0.2 (0.1 ms).

## What was changed (all with the user's explicit go)

- **pve** `/etc/network/interfaces`: `enp1s0` now `inet manual`, new `vmbr1` 10.0.0.1/24 with port `enp1s0`. Backup: `/etc/network/interfaces.bak_claude_20261029_222519`. Applied with `ifreload -a` (no reboot; vmbr0 untouched). Revert: restore the backup, `ifreload -a`.
- **VM 103**: `qm set 103 --net1 virtio,bridge=vmbr1,firewall=0` (hotplug, no reboot). NetworkManager profile `cable-lab` (UUID ba476060-987e-4f5e-9b46-dd688b10164b): 10.0.0.3/24, never-default, ipv6 off, autoconnect. Revert: `nmcli con delete cable-lab` and `qm set 103 --delete net1`.
- **Lab PC**: NM profile `Franka` with UUID `1006e4bb-894c-4afb-9a0b-7886fd4afa7a` (the `enp1s0` one) changed from 172.16.0.1/24 to 10.0.0.2/24, never-default yes. Done by the USER with sudo (murilo has no sudo/polkit over SSH). Revert: set the address back to 172.16.0.1/24 and `nmcli con up`. NEVER touch the other `Franka` profile (UUID `da220b7b-5c54-433d-918a-e90661fb786f`, `enp0s31f6`): that is the real robot link. Both profiles share the name "Franka": always use UUIDs.
- **New SSH key for pve**: `~/.ssh/id_ed25519_pve_claude` (comment `claude-vm103-pve-2026-10-29`), authorized for root on pve with `from="10.181.64.191,100.84.207.115",no-port-forwarding,...`. Use: `ssh -i ~/.ssh/id_ed25519_pve_claude -o IdentitiesOnly=yes root@10.181.64.131`. Revoke by deleting that line from `/root/.ssh/authorized_keys` on pve. The promise (not enforced): read first, show the diff and get a go before changing pve network/VM config; do not touch vmbr0, VM 102, storage, firewall, cluster. `~/.ssh/id_labgoto` is NOT the pve key (purpose unknown, was not used).

## Measurements over the cable (2026-10-29)

- Ping VM <-> lab PC, 500 packets at 10 ms: 0 % loss, 0.155 ms avg, 0.37 ms max. (VM <-> pve: ~0.09 ms.)
- TCP via `nc` (no iperf3 on either side): ~8 Gb/s on the 10 Gb link, `nc`-limited.
- ROS 2 ping-pong, isolated domain 199, Fast DDS pinned to 10.0.0.2/.3 (UDPv4 interfaceWhiteList + initialPeers): 100 Hz RTT median 1.55 / p99 3.7 ms, 0.17 % loss; 1 kHz RTT median 0.96 / p99 10 ms, 0.59 % loss. The extra delay vs ping is Python nodes on a loaded lab CPU, not the wire.
- Probe scripts (fastdds.xml, echo.py, client.py) are in `/tmp/claude-1000/-home-nexus-spot-teleop/6e223603-e757-4576-8eb5-a459069d04ab/scratchpad/rosprobe/`. All probe containers/files were removed on the VM and the lab PC.

## Gotchas learned

- The `spot-isaac-sim:6.0.0` image has no `/opt/ros` and no system python. Run rclpy with `--entrypoint bash` and `/isaac-sim/python.sh`, with `ROS_DISTRO=jazzy RMW_IMPLEMENTATION=rmw_fastrtps_cpp LD_LIBRARY_PATH=$J/lib PYTHONPATH=$J/rclpy:/isaac-sim/exts/omni.pip.compute/pip_prebundle`, where `J=/isaac-sim/exts/isaacsim.ros2.core/jazzy`. Without `--entrypoint bash` the image starts Isaac Sim headless and the run hangs (this cost a background-loop mess once; do not use `pkill -f` with a pattern that matches your own shell command).
- Lab ROS layout: `fr3-ros2` (driver, `murilo/franka-ros2:jazzy-3.6.0`) is host-net, domain 77, `ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST` -> invisible from the VM until that changes. `hub-curobo-mpc` is host-net, domain 142, SUBNET, Fast DDS UDPv4. Do not start a second MPC node in domain 142: it would collide with the live one. `claude-esdf-proxy` on the lab PC belongs to another lane (it was already running; left alone).
- The lab PC's `iperf3` is not installed; I did not install anything there.

## What is left (not started)

1. Decide the architecture with the user. My proposal: keep the driver and its relay helpers (`relay_in`, `curobo_to_jtc` inside `fr3-ros2`) on the lab PC; run the MPC node on VM 103 in its own domain with Fast DDS pinned to 10.0.0.3; bridge `/joint_states` and `/curobo/joint_command` over the cable. This needs the driver side to be reachable over 10.0.0.x (the LOCALHOST discovery range blocks it), which means a change on the lab PC that the user must approve.
2. Dry run with the arm inactive (fr3_arm_controller inactive, MPC standby) and measure delivered rate and joint-state age on the VM versus the 49 Hz seen when the node runs on the lab PC (see memory `fr3-mpc-rate-findings`).
3. Only then any motion, and only with a person at the e-stop. Do not run `real_plan_test.sh prep` twice, and never `run`/`joy` unattended.
4. Open safety item from the other lane: the joy-mode drift (hand drifted ~2 mm/s with the MPC active and no goal stream). Do not leave the MPC active on the real arm without a goal stream.
5. Optional: persist DDS settings (profile XML) somewhere in the repo once the setup is chosen; the VM's `cable-lab` profile and the pve bridge already survive reboots (the lab PC profile is also persistent).

## Where things are

- Memory: `/home/nexus/.claude/projects/-home-nexus-spot-teleop/memory/pve-access-and-cable.md` (same facts, short), plus `lab-pc-access.md` and `fr3-mpc-rate-findings.md`.
- Other handoffs (other lanes, not edited): `fr3_real_repro/HANDOFF_RATE_DRIFT_2026-10-29.md`, `zed_lite/HANDOFF_REAL_LAB.md`, `isaac-sim_ws/franka_mpc/HANDOFF.md`.
- No git worktree was created and no repository file was changed by this lane (only this note); `/home/nexus/spot-teleop` is not a git repository.
