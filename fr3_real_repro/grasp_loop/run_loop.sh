#!/usr/bin/env bash
# Grasp loop runner: builds vm-grasp-loop once, runs grasp_loop.py in it.
# Mounts the repo at its real path (so ds_env.sh + scripts resolve), the lab
# SSH key (read-only), and the docker socket (the loop manages VM containers).
#   ./run_loop.sh --seed 560,470,675,695 --label drill            # dry, all stages
#   ./run_loop.sh --seed ... --from 3 --to 4                       # cloud+grasp only
#   ./run_loop.sh --seed ... --from 6 --run-dir runs/<id> --live --estop "Name"
set -euo pipefail
D=$(cd "$(dirname "$0")" && pwd)
IMG=vm-grasp-loop:latest
docker image inspect "$IMG" >/dev/null 2>&1 || docker build -t "$IMG" -f "$D/Dockerfile.vm-loop" "$D"
exec docker run --rm --net host --ipc host \
  -e ROS_DOMAIN_ID=8 -e RMW_IMPLEMENTATION=rmw_fastrtps_cpp -e PYTHONUNBUFFERED=1 \
  -v /home/nexus/spot-teleop:/home/nexus/spot-teleop:rw \
  -v "$HOME/.ssh/id_ed25519_lab_claude":/ssh/id_lab:ro \
  -v "$HOME/.ssh/known_hosts":/root/.ssh/known_hosts:ro \
  -v /var/run/docker.sock:/var/run/docker.sock \
  --entrypoint bash "$IMG" -c 'source /opt/ros/humble/setup.bash && exec python3 /home/nexus/spot-teleop/fr3_real_repro/grasp_loop/grasp_loop.py "$@"' _ "$@"
