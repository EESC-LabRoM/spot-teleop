#!/usr/bin/env bash
# Stop the sim mapping chain in reverse order (receivers before the Isaac streamer).
docker stop franka-seg nvblox-lite zed-lite >/dev/null 2>&1 || true
docker ps -q --filter ancestor=spot-isaac-sim:6.0.0-zed | xargs -r docker stop >/dev/null
docker run --rm -v /home/nexus/spot-teleop/zed_lite:/zl -v /home/nexus/spot-teleop/isaac-sim_ws/franka_mpc/runs:/runs \
  --entrypoint chown zed-lite:5.4.1-humble -R 1000:1000 /zl /runs
echo stopped
