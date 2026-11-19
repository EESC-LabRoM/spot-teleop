#!/usr/bin/env bash
set -euo pipefail
case_name=${1:?z06 or z12}
root=/home/hipolito/franka_teleop_checks/bag_hardware_1118
case "$case_name" in
z06) bag=/home/hipolito/spot-teleop/bags/zed_only_20261113T201545Z_06_cima_baixo_2593924/d8/d8_0.mcap ;;
z12) bag=/home/hipolito/spot-teleop/bags/zed_only_20261113T201801Z_12_todos_combinados_2602550/d8/d8_0.mcap ;;
*) exit 2;; esac
docker run --rm --network none --cpus 4 --memory 8g --name wrist-real-bag-1118 \
 -v /home/hipolito/spot-teleop:/home/spot-teleop:ro \
 -v /home/hipolito/franka_teleop_checks/live_wrist_20261116:/deploy:ro \
 -v "$root":/work -v /home/murilo/franka_ws/lab/wrist_bag_1118:/io \
 -e TRACKER_DIR=/work/runtime -e PYTHONDONTWRITEBYTECODE=1 -e OMP_NUM_THREADS=2 -e OPENBLAS_NUM_THREADS=1 \
 -e PYTHONPATH=/home/spot-teleop/tmp/cv_site_hipolito:/home/spot-teleop/tmp/mp_site_hipolito:/home/spot-teleop/spot-ros2_ws/src/arm_pose_estimator \
 spot-teleop-isaac-ros:latest bash -lc 'source /opt/ros/humble/setup.bash; exec python3 /work/replay_to_socket.py "$@"' replay \
 --bag "${bag/\/home\/hipolito\/spot-teleop/\/home\/spot-teleop}" --output "/work/raw_$case_name" --duration 50 --pose-source rgbd_body --operator-profile /work/operator_profile.json
