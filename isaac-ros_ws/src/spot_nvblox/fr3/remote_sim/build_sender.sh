#!/usr/bin/env bash
set -euo pipefail
HERE=$(cd "$(dirname "$0")" && pwd)
BUILD_CONTAINER=${BUILD_CONTAINER:-zed-build}
mkdir -p "$HERE/bin"
docker cp "$HERE/zed_rgbd_sender.cpp" "$BUILD_CONTAINER:/tmp/fr3_rgbd_sender.cpp"
docker exec "$BUILD_CONTAINER" bash -c 'g++ -std=c++17 -O2 /tmp/fr3_rgbd_sender.cpp -I/usr/local/zed/include -I/usr/local/cuda/include -L/usr/local/zed/lib -L/usr/local/cuda/lib64 -Wl,-rpath,/usr/local/zed/lib -lsl_zed -lcudart -ljpeg -lz -pthread -o /tmp/fr3_rgbd_sender'
docker cp "$BUILD_CONTAINER:/tmp/fr3_rgbd_sender" "$HERE/bin/zed_rgbd_sender"
