#!/usr/bin/env bash
# newton_render_trace.py in the Isaac image with the host Newton (as run_newton_selftest.sh): mp4 of a run's trace.npz.   ./run_render_trace.sh runs/<run> [--every 2] [--title ...]
WS=/home/nexus/spot-teleop/isaac-sim_ws; FM=$WS/franka_mpc; SP=$WS/crawl-lab/IsaacLab/.venv/lib/python3.12/site-packages
R=$(readlink -f "${1:?run dir}"); shift
docker run --rm --gpus "${GPUS:-all}" -e ACCEPT_EULA=Y -e PYTHONUNBUFFERED=1 -e PYTHONDONTWRITEBYTECODE=1 -e OVRTX_SKIP_USD_CHECK=1 -e LD_LIBRARY_PATH=/home/nexus/newton_usd_sp/usd_core.libs -e PYTHONPATH="$SP:/home/nexus/newton_viewer_sp:/home/nexus/newton_rtx_sp:/home/nexus/newton_usd_sp" -v /home/nexus/newton_rtx_sp:/home/nexus/newton_rtx_sp:rw -v /home/nexus/newton_usd_sp:/home/nexus/newton_usd_sp:ro -v /home/nexus/newton_viewer_sp:/home/nexus/newton_viewer_sp:ro -v "$SP":"$SP":ro -v /home/nexus/newton_sp:/home/nexus/newton_sp:ro \
  -v "$WS":/workspace:rw -v "$FM/cache_newton_home/newton":/root/.cache/newton -v "$FM/cache_newton_home/warp":/root/.cache/warp --entrypoint /isaac-sim/python.sh spot-isaac-sim:6.0.0 \
  /workspace/franka_mpc/newton_render_trace.py "/workspace/franka_mpc/runs/$(basename "$R")" "$@" && sudo chown -R nexus: "$R"/video*.mp4 2>/dev/null; true
