# ZED -> FR3 cuRobo preview

Prepared 23 October 2026. **This is a planner preview, not a hardware control mode. The existing dashboard Start still drives the direct Cartesian path.**

The preexisting `franka-mpc-real` container was left untouched. It expects 5 cm voxels and was rejecting the 2 cm ESDF. A separate `fr3-zed-mpc-preview` instance uses the same FR3 model and Murilo's MPC source, a 2 cm voxel grid centered at [0.35,0,0.55], size [1.4,1.6,1.2], ESDF service `/nvblox_human/get_esdf_and_gradient`, measured joint feedback and no attraction field. `mpc_node.py` is a source snapshot (only trailing whitespace removed) of `/home/murilo/franka_ws/hub-curobo/node/curobo_mpc_node.py`; its license and authorship remain in the file.

The read-only bridge is two processes:

- `fr3-ros2`: `/lab/fr3_zed_runtime/curobo/zed_goal_bridge.py send` reads `/teleop/command_tool` only while fresh `/teleop/zed_status` says active. It composes the base-frame TCP goal with the actual fixed TF `fr3_hand_tcp -> fr3_hand`, rather than copying a hardcoded offset or treating the raw operator wrist as a robot pose. Sends UDP loopback 47826.
- `fr3-curobo`: the matching `receive` republishes fresh, ordered, finite normalized goals on `/fr3_zed_mpc/wrist_goal` (DDS8). Stale, repeated, malformed or wrong-frame packets are discarded.

The MPC consumes that private goal and publishes **only `/fr3_zed_mpc/joint_preview`**. No process connects it to a robot command topic. Neither the launcher nor bridge loads, switches or commands hardware controllers. Neither calls Start, Home or the gripper. Closing/losing the ZED session stops new goals, but the preview planner may continue solving its last goal; this must not be treated as a physical motion watchdog.

## Verification

- Three transform tests passed, including rotation of the TCP-to-hand offset and invalid quaternion rejection.
- The independent MPC loaded and accepted/updated the live 2 cm ESDF; no voxel-size mismatch in its log. It reported feasible `status=OK` solutions from the measured FR3 state.
- A Humble subscriber received 59 seven-joint preview messages during a three-second observation. A Jazzy subscriber in the driver container received zero during a separate observation, so the output transport to the robot is **not validated**. DDS emitted `sequence size exceeds remaining buffer` warnings.
- No live ZED goal was forwarded during verification because the operator session was disengaged/un-calibrated. Thus the full live hand-to-planner sequence remains to be exercised. No real movement test was performed.

## Before physical control

Do not wire the preview directly to the arm. Complete and test a fresh-command output bridge, joint/velocity/step limits, feasible-result and fresh-map gates, measured-state/watchdog checks, exclusive command ownership, and coordinated transitions between the Cartesian controller and the effort joint controller. Preserve the other session's joint-space Home and Stop semantics. The current Cartesian controller must not command concurrently with a joint controller. Keeping only the MPC's TCP and discarding its joint configuration would not preserve its elbow/self-collision solution.

The preexisting MPC source also publishes a result even when its feasibility metric is false and treats unknown ESDF as free. These behaviors must be considered explicitly before claiming real collision avoidance. This preview does not establish collision safety.

## Runtime

Start a new preview with `python3 run_preview.py` only if its container name is absent; the launcher refuses to replace an existing instance. Logs: `preview_logs/stdout.log` and `preview_logs/mpc_node.csv` (not committed). Existing container: `docker start fr3-zed-mpc-preview`. Stop only this instance with `docker stop fr3-zed-mpc-preview`.

Bridge logs: `fr3-ros2:/runs/fr3-zed-mpc-goal-sender.log` and `fr3-curobo:/tmp/fr3-zed-mpc-goal-receiver.log`. Stop only the processes whose command exactly matches `zed_goal_bridge.py send` / `receive`; leave the other teleoperation, driver and perception processes alone.
