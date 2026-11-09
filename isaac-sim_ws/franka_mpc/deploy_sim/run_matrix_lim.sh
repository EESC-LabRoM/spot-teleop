#!/usr/bin/env bash
# same faults as run_matrix.sh with the acceleration limiter (curobo_to_jtc.limit, 10 rad/s^2); J = limiter + no MPC reset on infeasible steps
cd "$(dirname "$0")"
RUN=deploy_F_clean_lim   FEEDBACK=hybrid BRIDGE_ARGS="--max-accel 10"                  ./run_deploy_sim.sh
RUN=deploy_G_spike_lim   FEEDBACK=hybrid BRIDGE_ARGS="--max-accel 10 --spike 12"       ./run_deploy_sim.sh
RUN=deploy_H_stall_lim   FEEDBACK=hybrid BRIDGE_ARGS="--max-accel 10 --stall 8 25"     ./run_deploy_sim.sh
RUN=deploy_I_delay_lim   FEEDBACK=hybrid BRIDGE_ARGS="--max-accel 10 --delay 3"        ./run_deploy_sim.sh
RUN=deploy_J_keep_lim    FEEDBACK=hybrid BRIDGE_ARGS="--max-accel 10" INFEAS=keep      ./run_deploy_sim.sh
echo MATRIX_LIM_DONE
