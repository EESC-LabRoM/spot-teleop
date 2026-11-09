#!/usr/bin/env bash
# ZED2 cable fan-in (see cable_fanin2.py header). Depth: FANIN2_DEPTH=seg (default, robot-masked raw) | compressed.
# Compressed legs only; must be the ONLY VM subscriber of the ZED2
# compressed cable topics. Feeds SAM2 #2 + dual-cloud merge on /vm/cable2/*.
set -euo pipefail
D=$(cd "$(dirname "$0")" && pwd)
docker rm -f vm-cable-fanin2 >/dev/null 2>&1 || true
docker run -d --name vm-cable-fanin2 --net host --ipc host \
  -e ROS_DOMAIN_ID=8 -e FANIN2_DEPTH="${FANIN2_DEPTH:-seg}" -e DEPTH_SCALE="${DEPTH_SCALE:-0.9959}" -e RMW_IMPLEMENTATION=rmw_fastrtps_cpp -e PYTHONUNBUFFERED=1 \
  -v "$D/cable_fanin2.py":/cable/cable_fanin2.py:ro \
  --entrypoint bash nvblox-lite:3.2.5-humble -c 'source /opt/ros/humble/setup.bash && python3 /cable/cable_fanin2.py'
