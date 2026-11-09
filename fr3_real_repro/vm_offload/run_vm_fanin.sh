#!/usr/bin/env bash
# Single-reader cable fan-in (see cable_fanin.py header for WHY).
# Must be the ONLY VM subscriber of the raw heavy cable topics.
set -euo pipefail
D=$(cd "$(dirname "$0")" && pwd)
docker rm -f vm-cable-fanin >/dev/null 2>&1 || true
docker run -d --name vm-cable-fanin --net host --ipc host \
  -e ROS_DOMAIN_ID=8 -e DEPTH_SCALE="${DEPTH_SCALE:-0.9767}" -e RMW_IMPLEMENTATION=rmw_fastrtps_cpp -e PYTHONUNBUFFERED=1 \
  -v "$D/cable_fanin.py":/cable/cable_fanin.py:ro \
  --entrypoint bash nvblox-lite:3.2.5-humble -c 'source /opt/ros/humble/setup.bash && python3 /cable/cable_fanin.py'
