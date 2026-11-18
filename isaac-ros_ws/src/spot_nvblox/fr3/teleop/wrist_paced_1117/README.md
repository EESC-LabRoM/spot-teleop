# Paced wrist teleop candidate — 2026-11-17

This is the isolated candidate tested on the FR3; existing shared lab teleop/MPC sources were preserved. Source is copied into fr3-ros2:/tmp/wrist_ready_1117 and uses the existing joystick_grasp runtime module. consumer_launch_args.json records the deployed ROS arguments. hand_only.yaml selects ROS Joy input instead of an unstable optional raw joydev reader; it does not replace the physical emergency stop.

Behavior:
- Four or more consistent current position samples spanning at least120ms allow startup during hand motion. Engagement binds the current measured TCP to the current wrist, with the existing measured-joint startup blend downstream. Duplicate/stale frames cannot arm it.
- If position becomes valid before orientation, the first valid orientation binds to the current command instead of a placeholder quaternion. Subsequent tracking gaps preserve that reference and combined motion.
- Translation gain0.5:10cm hand travel commands5cm robot travel. The table-facing preset keeps tool+Z toward base−Z; hand yaw controls tool yaw. Hand roll/pitch are intentionally omitted in this preset; full6D remains a separate mode.
- Cartesian acceleration bound0.5m/s². Following pace1/(1+(command-to-measured-position-error/0.03m)²) scales Cartesian and angular rate caps together. The hand reference is retained; pacing can delay fast input.
- Existing80mm following leash, workspace floor0.25m, stale-input/controller checks, physical e-stop and native collision handling remain. Original MPC/null-space weights and real joint gains were retained after lower posture weights performed worse in simulation.

Checks:
Run python3 test_input.py (NumPy, no ROS required). It exercises the actual consumer methods/target statements for moving startup, duplicates, spikes/gaps, first-orientation binding, reference preservation and combined motion, plus1000 acceleration reversal steps.

Recorded tests and operational state are documented at:
/home/hipolito/franka_teleop_checks/wrist_fix_1117/HANDOFF.md
/home/hipolito/franka_teleop_checks/wrist_fix_1117/hardware_paced/validation_summary.json
/home/hipolito/franka_teleop_checks/wrist_fix_1117/hardware_paced/results/wrist_paced_validation.html

Eight raw RGBD bags were replayed at rate1 through the same debug tracker and the real consumer→native MPC→effort JTC→real empty gripper. These are hardware records, separate from isolated MuJoCo tests. Source/profile hashes are in manifest.json. The first final z07 attempt was aborted on optional raw-pad disconnect; its evidence is retained separately and was followed by a new full replay in hand-only mode.

This validates the recorded envelope and measured robot following, not perfect hand-pose recognition or a full-workspace guarantee. The report distinguishes commanded TCP error from hand-intent fidelity and exposes valid-packet engagement. Actual live-hand validation is separate; consult HANDOFF for whether a visible right hand was observed.

Starting hardware requires the resource-board claim, operator e-stop coverage, fresh passive robot/error/payload checks, a single consumer/goal/output route, and measured-state MPC warmup with the physical forwarder stopped. The recorded args alone do not activate controllers or own resources. The bounded run_live_trial.py and cleanup.py in the artifact directory record the executed workflow. Never restore a stale command or restart the original MPC alongside the candidate writer.
