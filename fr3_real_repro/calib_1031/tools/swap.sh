set -e
cam=$1; file=$2
PID=$(docker exec zed-build pgrep -f "real_extrinsics_tf.py --calib /tmp/nvblox_real/calib/${file} --cam ${cam}" | head -1)
echo "old publisher pid in container: $PID"; docker exec zed-build bash -c "tr '\0' ' ' < /proc/$PID/cmdline | cut -c1-160; echo"
docker exec zed-build bash -c "tr '\0' ' ' < /proc/$PID/cmdline" | grep -q -- "--cam ${cam}" || { echo "pid does not match ${cam}: abort"; exit 1; }
docker exec zed-build kill $PID; sleep 1
docker exec -d zed-build bash -c "source /opt/ros/humble/setup.bash; export ROS_DOMAIN_ID=8 RMW_IMPLEMENTATION=rmw_fastrtps_cpp; exec python3 /tmp/nvblox_real/real_extrinsics_tf.py --calib /tmp/nvblox_real/calib/${file} --cam ${cam} > /tmp/tf_${cam}_1031.log 2>&1"
for i in $(seq 1 40); do docker exec zed-build grep -q "static TF published" /tmp/tf_${cam}_1031.log 2>/dev/null && break; sleep 1; done
docker exec zed-build sh -c "grep -a -E 'static TF published|error' /tmp/tf_${cam}_1031.log | cut -c1-230"
echo "publishers now:"; docker exec zed-build pgrep -af "real_extrinsics_tf.py" | cut -c1-150
