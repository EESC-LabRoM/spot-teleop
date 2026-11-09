#!/usr/bin/env bash
# stall repeatability: same stall without the limiter (C2) and with it again (H3); compare with analysis of phase in stall_cmp
cd "$(dirname "$0")"
RUN=deploy_C2_stall8_nolim FEEDBACK=hybrid BRIDGE_ARGS="--stall 8 25"                ./run_deploy_sim.sh
RUN=deploy_H3_stall_lim    FEEDBACK=hybrid BRIDGE_ARGS="--max-accel 10 --stall 8 25" ./run_deploy_sim.sh
echo MATRIX_LIM3_DONE
