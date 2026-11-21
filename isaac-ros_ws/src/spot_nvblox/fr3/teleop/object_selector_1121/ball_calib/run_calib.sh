#!/bin/bash
# Wrist-camera hand-eye calibration motion (user go 2026-11-21). Each stage via the
# same JTC executor as Y; capture at each stationary pose. Stops on any failure.
set -u
PLAN=/lab/fr3_zed_runtime/ball_calib/ball_handeye_plan_v2.json
OUT=/home/hipolito/franka_teleop_checks/object_selector_1121/ball_calib/run_$(date -u +%H%M%S)
mkdir -p "$OUT"
EXEC='source /opt/ros/jazzy/setup.bash; source /ros2_ws/install/setup.bash; export ROS_DOMAIN_ID=77; cd /lab/fr3_zed_runtime'
views="["
for S in S1_home_to_pre C2 C3 C4 C5 C6 C7; do
  echo "== $S $(date -u +%T)"
  if ! docker exec fr3-ros2 bash -lc "$EXEC; python3 vendor/grasp_stage_exec.py $PLAN $S 0.05"; then
    echo "STOP: executor failed at $S; arm holds its last JTC target"; echo "${views%,}]" > "$OUT/views.json"; exit 1
  fi
  sleep 1.5
  Q=$(docker exec fr3-ros2 bash -lc "source /opt/ros/jazzy/setup.bash; export ROS_DOMAIN_ID=77; python3 /lab/fr3_zed_runtime/ball_calib/read_joints.py" 2>/dev/null | tail -1 | python3 -c "import sys,json;print(json.dumps(json.loads(sys.stdin.read())['q']))")
  [ -n "$Q" ] || { echo "STOP: joint read failed at $S"; echo "${views%,}]" > "$OUT/views.json"; exit 1; }
  docker exec isaac-build bash -lc "source /opt/ros/humble/setup.bash; export ROS_DOMAIN_ID=8; python3 /tmp/wrist_map_filter/capture_wrist.py /tmp/wrist_map_filter/cal_$S.npz" 2>&1 | grep -v 'sequence size'
  docker cp isaac-build:/tmp/wrist_map_filter/cal_$S.npz "$OUT/" && views="$views{\"npz\":\"$OUT/cal_$S.npz\",\"q\":$Q},"
  echo "   q=$Q"
done
echo "${views%,}]" > "$OUT/views.json"
echo "== Z_home $(date -u +%T)"
docker exec fr3-ros2 bash -lc "$EXEC; python3 vendor/grasp_stage_exec.py $PLAN Z_home 0.05" || { echo "STOP: return home failed"; exit 1; }
echo "DONE $OUT"
