#!/usr/bin/env bash
# Wiring test without the robot or the lab: node(JointTrajectory) -> real curobo_to_jtc.py (ros->udp, flags from vm_real.sh) -> udp (127.0.0.1)
# -> real curobo_to_jtc.py (udp->ros) -> stand-in for the controller + Newton sim. localhost-only DDS.
cd "$(dirname "$0")"
RUN=deploy_N_e2e_wiring FEEDBACK=hybrid E2E=1 CMDTYPE=joint_trajectory NODE_OUT=/curobo/joint_trajectory \
  BRIDGE_ARGS="--traj --noslew --cmd-topic /fr3_arm_controller/joint_trajectory" ./run_deploy_sim.sh
echo E2E_DONE
