#!/usr/bin/env bash
# limiter v2: after a gap the limiter restarts from rest (the controller held still)
cd "$(dirname "$0")"
RUN=deploy_H2_stall_lim  FEEDBACK=hybrid BRIDGE_ARGS="--max-accel 10 --stall 8 25"  ./run_deploy_sim.sh
RUN=deploy_G2_spike_lim  FEEDBACK=hybrid BRIDGE_ARGS="--max-accel 10 --spike 12"    ./run_deploy_sim.sh
echo MATRIX_LIM2_DONE
