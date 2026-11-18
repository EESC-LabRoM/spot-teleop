#!/usr/bin/env bash
# Perception-only, physically isolated replay. This script has no robot backend.
set -euo pipefail
ROOT=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
REPO_ROOT=${REPO_ROOT:-/home/hipolito/spot-teleop}
MODEL_DIR=${MODEL_DIR:-/home/hipolito/franka_teleop_checks/live_wrist_20261116}
args=()
for bag in "$REPO_ROOT"/bags/zed_only_20261113T20*/d8/d8_0.mcap; do
 [[ -f "$bag" ]] || { echo 'Recorded ZED3 bags not found' >&2; exit 1; }
 args+=(--bag "/home/spot-teleop/bags/$(basename "$(dirname "$(dirname "$bag")")")/d8/d8_0.mcap")
done
for label in rgbd_fixed rgbd_body_safe; do
 mode=rgbd_fixed
 extra=()
 if [[ "$label" == rgbd_body_safe ]]; then
  mode=rgbd_body
  extra=(--operator-profile /work/operator_profile.json)
 fi
 docker run --rm --network none --cpus 4 --memory 8g --name "wrist-body-calib-${label}-1118" \
  -v "$REPO_ROOT":/home/spot-teleop:ro -v "$MODEL_DIR":/deploy:ro -v "$ROOT":/work \
  -v "$ROOT/runtime":/work/runtime:ro \
  -e TRACKER_DIR=/work/runtime -e PYTHONDONTWRITEBYTECODE=1 -e OMP_NUM_THREADS=2 -e OPENBLAS_NUM_THREADS=1 \
  -e PYTHONPATH=/home/spot-teleop/tmp/cv_site_hipolito:/home/spot-teleop/tmp/mp_site_hipolito:/home/spot-teleop/spot-ros2_ws/src/arm_pose_estimator \
  spot-teleop-isaac-ros:latest bash -lc 'source /opt/ros/humble/setup.bash; exec python3 /work/replay_session.py "$@"' replay \
  "${args[@]}" --output "/work/results/$label" --duration 60 --pose-source "$mode" "${extra[@]}" > "$ROOT/$label.log" 2>&1
 echo "COMPLETED $label"
done
python3 "$ROOT/analyze.py"
