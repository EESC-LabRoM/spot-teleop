# nvblox_msgs 3.2.5 built on ROS 2 Jazzy (Python 3.12)

Same .msg/.srv as the Humble nvblox server (isaac_ros_nvblox v3.2-14, nvblox-lite image), built so nodes running in
Isaac Sim 6.0's Python 3.12 with its bundled Jazzy rclpy can call the ESDF service. Rebuild (about 10 s):

    W=~/spot-teleop/zed_lite/nvblox_msgs_jazzy
    docker run --rm -v $W:/ws -w /ws ros:jazzy-ros-base bash -c \
      'source /opt/ros/jazzy/setup.bash && colcon build --merge-install && chown -R 1000:1000 /ws'

Use in the Isaac image: ROS_DISTRO=jazzy, LD_LIBRARY_PATH=$J/lib:<install>/lib,
PYTHONPATH=$J/rclpy:<install>/lib/python3.12/site-packages with J=/isaac-sim/exts/isaacsim.ros2.core/jazzy.

Tested 2026-10-14: a Jazzy py3.12 client and a Humble py3.10 client got byte-identical replies from the Humble nvblox
node (76x101x51 grid, same sha1, about 25 ms). Do NOT use NVIDIA's prebuilt ros-jazzy-nvblox-msgs 4.6: its ESDF reply
adds a `header` field, so calls to the 3.2.5 server get no answer.
