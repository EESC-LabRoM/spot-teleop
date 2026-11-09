#!/usr/bin/env bash
# Static-camera stand-in for tf_projection (which runs nowhere): converts the
# tracker's /tracking/seed_3d into /tracking/seed_pixel via live camera_info,
# so the SAM video predictor can (re)initialize. Needed once per seed; this
# runs it persistently (tiny CPU, no GPU). Same domain/XML as the tracker.
set -euo pipefail
D=$(cd "$(dirname "$0")" && pwd)
[ "${1:-}" = "once" ] && DUR=60 || DUR=86400
docker run -d --name vm-px-bridge --net host --ipc host \
  -e ROS_DOMAIN_ID=8 -e RMW_IMPLEMENTATION=rmw_fastrtps_cpp -e PYTHONUNBUFFERED=1 \
  -v "$D/seed_pixel_bridge.py":/cable/seed_pixel_bridge.py:ro \
  --entrypoint bash nvblox-lite:3.2.5-humble -c "source /opt/ros/humble/setup.bash && python3 /cable/seed_pixel_bridge.py $DUR /tracking/seed_3d /tracking/seed_pixel"
