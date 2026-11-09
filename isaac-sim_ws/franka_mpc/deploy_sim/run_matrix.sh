#!/usr/bin/env bash
# fault matrix on top of run_deploy_sim.sh; each result in runs/deploy_*/summary.txt
cd "$(dirname "$0")"
RUN=deploy_B_spike12  FEEDBACK=hybrid   BRIDGE_ARGS="--spike 12"     ./run_deploy_sim.sh
RUN=deploy_C_stall8   FEEDBACK=hybrid   BRIDGE_ARGS="--stall 8 25"   ./run_deploy_sim.sh
RUN=deploy_D_delay3   FEEDBACK=hybrid   BRIDGE_ARGS="--delay 3"      ./run_deploy_sim.sh
RUN=deploy_E_clean_measured FEEDBACK=measured                        ./run_deploy_sim.sh
echo MATRIX_DONE
