# Current handoff status, Nov19~17:35Z

Standing preparation completed; passive preview50/50 passed and source/configuration pushed0235e74. Fresh person calibration has not run because the operator is not yet in view. No physical motion or hardware output from this preparation.

For Murilo's scheduled14-pose calibration, stopped this lane's temporary standing perception and standby candidate MPC, cancelled warm-up expiry timer, and released camera/CPU/port/GPU/MPC/tower resources after confirming own jobs exited. Original tracker/ZED3 wrapper/map and other-lane jobs were preserved. Do not activate physical teleop until calibration lane c66ecc79 confirms cleanup and the robot resource is handed over. If calibration changes TF/map, revalidate the real map before teleop.

Restart after handoff: acquire resources; docker start wrist-standing-perception-1119; docker start codex-fr3-wrist-response-1117 and its ESDF domain77->8 proxy. Check fresh view/diagnostics and current operator arm calibration. Then current e-stop/empty-gripper/clear-space confirmation plus franka-robot claim before start_supervised_live.sh. No saved bag person's profile; fixed orientation first. See README.md and VM_SEGMENTATION.md for limits.
