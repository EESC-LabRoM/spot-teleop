#!/usr/bin/env bash
set -euo pipefail
HERE=$(cd "$(dirname "$0")" && pwd)
[ -x "$HERE/bin/zed_rgbd_sender" ] || { echo 'Missing bin/zed_rgbd_sender'; exit 1; }
mkdir -p "$HERE/settings" "$HERE/resources"
DEVICES=()
for p in /dev/v4l/by-id/*ZED*; do
  [ -e "$p" ] && DEVICES+=(--device="$(readlink -f "$p")")
done
[ "${#DEVICES[@]}" -gt 0 ] || { echo 'ZED camera not connected'; exit 1; }
if docker inspect fr3-zed-sender >/dev/null 2>&1; then
  if [ "$(docker inspect -f '{{.State.Running}}' fr3-zed-sender)" = true ]; then
    echo 'ZED sender already running'; exit 0
  fi
  mkdir -p "$HERE/logs"
  docker logs --tail 100 fr3-zed-sender > "$HERE/logs/previous-sender.log" 2>&1 || true
  docker rm fr3-zed-sender
fi
docker run -d --name fr3-zed-sender --network host --restart on-failure:3 \
  --device=nvidia.com/gpu=all "${DEVICES[@]}" --device-cgroup-rule='c 189:* rmw' \
  -v /dev/bus/usb:/dev/bus/usb \
  -v "$HERE/bin:/opt/fr3-zed:ro" -v "$HERE/settings:/usr/local/zed/settings" \
  -v "$HERE/resources:/usr/local/zed/resources" \
  --entrypoint /opt/fr3-zed/zed_rgbd_sender stereolabs/zed:5.0-runtime-cuda12.8-ubuntu22.04 "${BIND_IP:-100.118.129.74}"
echo 'Read status: docker logs --tail 20 fr3-zed-sender'
