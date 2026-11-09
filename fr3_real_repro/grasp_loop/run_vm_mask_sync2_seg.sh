#!/usr/bin/env bash
# 2026-11-06 (Murilo): ZED2 nvblox carve from the ZED2 SAM2 mask (was a drill-box projection, vm_mask_sync2.py).
# /zed2/segmentation_mask (~1.5 Hz) + /vm/cable2/depth -> /vm/target_mask2 (depth-stamped), same restamper as ZED1.
# Rollback: docker rm -f vm-mask-sync2 && docker rename vm-mask-sync2-box vm-mask-sync2 && docker start vm-mask-sync2
set -euo pipefail
D=$(cd "$(dirname "$0")" && pwd)
docker run -d --name vm-mask-sync2 --net host --ipc host \
  -e ROS_DOMAIN_ID=8 -e RMW_IMPLEMENTATION=rmw_fastrtps_cpp -e PYTHONUNBUFFERED=1 \
  -v "$D/vm_mask_sync.py":/w/vm_mask_sync.py:ro \
  --entrypoint bash nvblox-lite:3.2.5-humble -c "source /opt/ros/humble/setup.bash && python3 /w/vm_mask_sync.py --depth /vm/cable2/depth --sam2 /zed2/segmentation_mask --out /vm/target_mask2 --max-age 2.0 --release-after 10 --dilate 12"
