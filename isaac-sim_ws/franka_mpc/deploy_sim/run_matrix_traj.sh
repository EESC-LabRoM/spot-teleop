#!/usr/bin/env bash
# node publishes JointTrajectory (command_type:=joint_trajectory); same limiter + faults as before, applied after the node like in vm_real
cd "$(dirname "$0")"
export CMDTYPE=joint_trajectory
RUN=deploy_K_traj_clean  FEEDBACK=hybrid BRIDGE_ARGS="--traj --max-accel 10"                  ./run_deploy_sim.sh
RUN=deploy_L_traj_spike  FEEDBACK=hybrid BRIDGE_ARGS="--traj --max-accel 10 --spike 12"       ./run_deploy_sim.sh
RUN=deploy_M_traj_stall  FEEDBACK=hybrid BRIDGE_ARGS="--traj --max-accel 10 --stall 8 25"     ./run_deploy_sim.sh
echo MATRIX_TRAJ_DONE
