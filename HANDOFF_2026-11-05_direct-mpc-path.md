# Direct-MPC teleop path + bridge retirement — handoff 2026-11-05 (lane 8a)

Companion to `HANDOFF_2026-11-05_STATE.md` (e64's map/calib/grasp note).
Branch: `murilo-curobo-nvblox-hand-20261101` on EESC-LabRoM/spot-teleop, commit `66bcd73`
(live teleop file only; `franka-teleop` untouched; merge is Hipolito's call).

## End state (verified 01:55–02:00 UTC)
- User Stop pressed 01:55:20Z: hardware interface INACTIVE, JTC INACTIVE, robot_mode 0,
  arm braked at home. Nothing moves until resume chain runs.
- Resume (person at robot, all in fr3-ros2, domain 77, LOCALHOST): release User Stop →
  `/action_server/error_recovery` → hardware component FrankaHardwareInterface active →
  `switch_controllers --activate fr3_arm_controller`. Then the direct_mpc gate opens
  on its own (checks JTC active + fresh preview). No restarts needed: forwarder, goal
  relay, teleop, MPC all up.
- Claims: NONE held by this lane.

## What is live (all on Murilo's go)
Live path (bridgeless): pad → teleop (`fr3_joy_teleop`, domain 77) → goal relay
`/tmp/direct_goal_fwd.py` (teleop command → `/fr3_joystick_mpc/wrist_goal`, 29 Hz,
only while Start held + fresh command) → `codex-fr3-live-mpc` (domain 77, active,
preview ~46–47 Hz) → arm forwarder `/tmp/direct_topic_fwd` (preview →
`/fr3_arm_controller/joint_trajectory`, engaged-gated, 0.03 rad/packet slew, ref pose
survives disengage; silent otherwise) → JTC.
- Teleop has the `direct_mpc` engage gate (replaces the old bridge-status check;
  original backed up next to it). Pad X / dashboard Home / `real-home` share the
  patched `fr3_home_jtc.py` fallback (homes through already-active JTC, no swap).
- Stock MPC CLI flagged: `node fr3_joystick_mpc`, `robot:=franka`,
  `command_type:=joint_trajectory`, `mpc_on_infeasible:=keep`, voxel 0.02,
  `mpc_override.yml` present (horizon 24, extra smoothing, collision weight 50000,
  detuned MPPI). Full line staged at `/tmp/sim_contract/MPC_CLI.txt` on the lab host.
- ESDF on domain 77 comes from my proxy `/tmp/esdf_proxy_d77.py` (d77 → live nvblox
  in domain 8). The old `claude-esdf-proxy` serves 142 only — do not remove mine
  while the MPC is on 77.

## Retired (do NOT restart the bridge)
- Lab: noguards hardware bridge (`joystick_bridge.py` + auto-rearm patch + backup)
  moved to `fr3_zed_runtime/retired_1104_direct/`. Shared helper modules stay — the
  other (guards) bridge still imports them; that's another lane's stack, untouched.
- This VM: `retired_1104_direct/` holds `zed_wrist_switch/` (switch client, bridge
  endpoint sketch, dashboard toggle, auto-rearm, bypass-flag proposal), the forwarder
  sources, A/B scripts + traces, comparison README. Restore = move two files back +
  one recorded start command (see README there).
- Both lanes (e64, Hipolito alignment) were told: direct path is the only live one.

## Proof it works
- A/B replay of Murilo's real 20:05 drive (same goals): direct 0.64° RMSE vs bridged
  0.91°, zero dropouts/lag/faults. Velocity not comparable (6 Hz vs 30 Hz sampling).
- Joystick drove live through the direct path; home via stock path (0.53° arrival).
- Sim lane (62f98b58, Isaac replica on VM103 domain 103, GPU0) runs the exact vendored
  forwarder (sha verified) + live node + override: preview-rate parity holds
  (50 Hz sim-time vs 46–47 wall, same controller rate). Binned-cost frame is shared;
  their next round boots with the set_active fix.

## The two reflexes (both solved)
- 21:17:49: GOAL FIGHT — pad (forwarder ~45 Hz) + calibration trajectories published
  to JTC at once; joint 1 −0.0805 vs 0.08 → controller abort → libfranka
  cartesian_reflex → hardware inactive. Process gap: pad live during a claimed window.
  PROPOSED, NOT DEPLOYED: process rule (pad dead during claimed windows) + forwarder
  guard that holds when a second publisher appears on the JTC topic.
- 21:02:10: spontaneous idle loss (planner quiet, teleop disengaged), 5th in a
  32-min idle cluster. Robot-side flakiness, not our chain.

## Retracted (do not spend time on)
- Sphere-59 self-clear radius: e64 proved the node already maps −1000 → free
  (curobo_mpc_node.py:1033) and the min_dist-0 readout is a labeling artifact on a
  clear arm (verified l.1766–1769, 1232, 1819). Sim lane was told before burning a run.
- Earlier "hotter from clutter" cost take: wrong; high median is a stale goal the
  stopped arm can't reach. Use pose_error-binned comparison (live holding ≈ 5).
  Optional: reset the MPC goal to the measured hand pose at resume (chasing a stale
  target ~3 h).

## Known live quirks
- Planner skips ESDF while the map refills after restarts (sees free space) — keep
  clear of the rig until the map matures; e64 pings on rebuilds.
- Map is now 2-camera (ZED1+ZED2) on robot-fit-7DOF TFs + depth scale; table ~2 cm
  thick near ZED1 (conservative for planner). Grasp test closed ~3 cm high, no contact.
- MPC goal has been stale since the stop; live teleop runs from Murilo's tree, not
  Hipolito's repo (his tree on the lab box is dirty with uncommitted work — do not
  sweep it into commits).

## Open, Murilo's call
1. Deploy the second-publisher guard in the forwarder.
2. Merge `murilo-curobo-nvblox-hand-20261101` into `franka-teleop` (Hipolito's call).
3. Reset MPC goal at resume; re-verify chain (engaged false, JTC active, preview
   ~47 Hz, single JTC publisher) before anyone drives.
