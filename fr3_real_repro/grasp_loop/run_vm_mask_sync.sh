#!/usr/bin/env bash
# VM-side mask restamper for the grasp loop (replaces deaf lab mask_sync).
# /zed1/segmentation_mask + /vm/cable/depth -> /vm/target_mask (depth-stamped).
# NOTE: depth MUST come from the fan-in (/vm/cable/*), never the raw cable
# topic -- the lab sender collapses under 2+ VM readers (see cable_fanin.py).
set -euo pipefail
D=$(cd "$(dirname "$0")" && pwd)
ROI=${ROI:-}
[ -n "$ROI" ] && ROIF="--roi $ROI" || ROIF=""
docker run -d --name vm-mask-sync --net host --ipc host \
  -e ROS_DOMAIN_ID=8 -e RMW_IMPLEMENTATION=rmw_fastrtps_cpp -e PYTHONUNBUFFERED=1 \
  -v "$D/vm_mask_sync.py":/w/vm_mask_sync.py:ro \
  --entrypoint bash nvblox-lite:3.2.5-humble -c "source /opt/ros/humble/setup.bash && python3 /w/vm_mask_sync.py --depth /vm/cable/depth $ROIF"
