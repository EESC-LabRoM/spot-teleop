#!/usr/bin/env bash
# newton_rtx_probe.py (Newton ViewerRTX/OVRTX vs ViewerGL) in the Isaac image with the host Newton + ovrtx + usd-core (separate site dirs).   ./run_rtx_probe.sh runs/<run> [--frame N] [--env studio]
WS=/home/nexus/spot-teleop/isaac-sim_ws; FM=$WS/franka_mpc; SP=$WS/crawl-lab/IsaacLab/.venv/lib/python3.12/site-packages; H=/home/nexus
R=$(readlink -f "${1:?run dir}"); shift
docker run --rm --gpus "${GPUS:-device=0}" -e ACCEPT_EULA=Y -e PYTHONUNBUFFERED=1 -e PYTHONDONTWRITEBYTECODE=1 -e OVRTX_SKIP_USD_CHECK=1 -e PROBE="${PROBE:-rtx,gl}" -e OVRTX_CACHE="${OVRTX_CACHE:-}" \
  -e PYTHONPATH="$SP:$H/newton_rtx_sp:$H/newton_usd_sp:$H/newton_viewer_sp" -e LD_LIBRARY_PATH="$H/newton_usd_sp/usd_core.libs" \
  -v "$SP":"$SP":ro -v $H/newton_sp:$H/newton_sp:ro -v $H/newton_rtx_sp:$H/newton_rtx_sp:rw -v $H/newton_usd_sp:$H/newton_usd_sp:ro -v $H/newton_viewer_sp:$H/newton_viewer_sp:ro \
  -v "$WS":/workspace:rw -v "$FM/cache_newton_home/root":/root -v "$FM/cache_newton_home/newton":/root/.cache/newton -v "$FM/cache_newton_home/warp":/root/.cache/warp -v "$FM/cache_newton_home/root":/root --entrypoint /isaac-sim/python.sh spot-isaac-sim:6.0.0 \
  /workspace/franka_mpc/newton_rtx_probe.py "/workspace/franka_mpc/runs/$(basename "$R")" "$@"; sudo chown -R nexus: "$R/rtx_probe" 2>/dev/null; true
