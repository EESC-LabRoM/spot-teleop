#!/usr/bin/env bash
# Host side: (re)start THE nvblox for cuRobo on the real FR3. One node, /nvblox_node, masked depth by default.
#   bash start_nvblox_masked.sh           masked (arm + tracked object removed from the depth)   [default]
#   MODE=raw bash start_nvblox_masked.sh  raw ZED depth (the old run_nvblox_real.sh behaviour, same parameters)
#   CAMERAS=zed1,zed2,zed3 ...            optional extra camera (default zed1,zed2; see fr3/zed3/README.md before using zed3)
#   bash start_nvblox_masked.sh stop      stop it;   RESTORE_OLD=1 bash start_nvblox_masked.sh stop   stop and restart Murilo's old raw setup
# Kills nvblox_node processes by PID (inside isaac-build) and checks that none is left before starting.
set -u
HERE=$(cd "$(dirname "$0")" && pwd); C=isaac-build
# launch file: next to this script (Murilo's zed_lite/nvblox/) first, else ../launch/ (spot_nvblox repo layout)
LAUNCH=""; for f in "$HERE/fr3_nvblox.launch.py" "$HERE/../launch/fr3_nvblox.launch.py"; do [ -f "$f" ] && { LAUNCH=$(cd "$(dirname "$f")" && pwd)/fr3_nvblox.launch.py; break; }; done
[ -n "$LAUNCH" ] || { echo "fr3_nvblox.launch.py not found next to $0 or in ../launch/"; exit 1; }
# DRY=1: only print what would be copied/started, touch nothing
if [ "${DRY:-0}" = 1 ]; then echo "DRY: launch=$LAUNCH run=$HERE/run_nvblox_real_masked.sh MODE=${MODE:-masked} arg=${1:-start}"; exit 0; fi
pids() { docker exec $C bash -c "ps -eo pid,args | grep -E '(nvblox_ros/nvblox_node|launch /tmp/fr3_nvblox.launch.py|run_nvblox_real(_masked)?.sh)' | grep -v grep | awk '{print \$1}'"; }
P=$(pids); if [ -n "$P" ]; then
  docker exec $C bash -c "for p in $(echo $P); do kill -INT \$p 2>/dev/null; done; sleep 4; for p in $(echo $P); do kill -TERM \$p 2>/dev/null; done; sleep 2"
fi
[ -z "$(pids)" ] || { echo "nvblox still running: $(pids)"; exit 1; }
if [ "${1:-}" = stop ]; then
  [ "${RESTORE_OLD:-0}" = 1 ] && docker exec -d $C bash -c 'nice -n 19 taskset -c 12-15 bash /tmp/run_nvblox_real.sh > /tmp/nvblox_real.log 2>&1' && echo "old raw /nvblox_node restarted"
  exit 0
fi
docker cp "$HERE/run_nvblox_real_masked.sh" $C:/tmp/run_nvblox_real_masked.sh
docker cp "$LAUNCH" $C:/tmp/fr3_nvblox.launch.py
docker exec -d -e MODE="${MODE:-masked}" -e CAMERAS="${CAMERAS:-zed1,zed2}" $C bash -c 'bash /tmp/run_nvblox_real_masked.sh > /tmp/nvblox_masked.log 2>&1'
echo "started MODE=${MODE:-masked}; node /nvblox_node, service /nvblox_node/get_esdf_and_gradient (log isaac-build:/tmp/nvblox_masked.log)"
