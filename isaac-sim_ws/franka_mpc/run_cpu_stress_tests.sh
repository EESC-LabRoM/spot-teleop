#!/usr/bin/env bash
# CPU-contention rate tests (10-29): the lab PC's 16 cores are saturated (load ~72), the node runs `nice -n -10`. Same full camera stack as the GPU-split runs (layout A),
# plus N CPU burners at nice 0. Scenarios: default executor | EventsExecutor | EventsExecutor + node pinned to cores 0-1 with the burners kept on cores 2-N.
cd /home/nexus/spot-teleop/isaac-sim_ws/franka_mpc || exit 1
NC=$(nproc); ESDF=runs/maponly_hd1080/map_check/esdf.npz
burn() { for i in $(seq 1 $1); do ( timeout 900 taskset -c $2 python3 -c 'while True: pass' >/dev/null 2>&1 & ); done; }
unburn() { pkill -f "while True: pass" 2>/dev/null; sleep 2; }
run() { n=$1; burners=$2; cores=$3; shift 3; burn $burners $cores
  env LOADCAMS="zed zed2 zed3" NVLOAD=1 SEGLOAD=1 G_MPC=0 G_WRAP=0 G_NV=1 G_SIM=1 ESDF=$ESDF MPC_NICE=-10 "$@" ./run_newton_rate.sh $n > runs/$n.log 2>&1; unburn; echo "$n: $(tail -1 runs/$n.log)"; }
trap unburn EXIT
run cpu_A_default_nice  $NC "0-$((NC-1))" DOCKER_EXTRA="--cap-add SYS_NICE"
run cpu_B_events_nice   $NC "0-$((NC-1))" DOCKER_EXTRA="--cap-add SYS_NICE -e MPC_EXECUTOR=events"
run cpu_C_events_pinned $((NC-2)) "2-$((NC-1))" DOCKER_EXTRA="--cap-add SYS_NICE --cpuset-cpus 0-1 -e MPC_EXECUTOR=events"
