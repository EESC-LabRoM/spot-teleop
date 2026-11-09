# Lightweight nvblox: zed-lite (ZED SDK 5.4.1 runtime, CUDA 12.8 runtime, ROS 2 Humble) + only the nvblox ROS node from
# NVIDIA's Isaac ROS 3.2 apt repo (ros-humble-nvblox-ros and what it needs: GXF/NITROS bits + RViz libs, ~0.7 GB).
# nvblox_node also links NPP (CUDA repo, libnpp-12-8) and VPI 3 (NVIDIA's x86_64 Jetson repo, libnvvpi3).
# Not the isaac-ros-nvblox metapackage (that pulls TensorRT, Visual SLAM and a driver package), not the Isaac ROS image.
#   docker build -t nvblox-lite:3.2.5-humble -f zed_lite/nvblox/Dockerfile zed_lite/nvblox/ctx
FROM zed-lite:5.4.1-humble
RUN apt-get update && apt-get install -y --no-install-recommends wget gnupg \
 && wget -qO /usr/share/keyrings/isaac-ros.asc https://isaac.download.nvidia.com/isaac-ros/repos.key \
 && echo "deb [signed-by=/usr/share/keyrings/isaac-ros.asc] https://isaac.download.nvidia.com/isaac-ros/release-3 jammy release-3.0" \
      > /etc/apt/sources.list.d/isaac-ros.list \
 && apt-get update && apt-get install -y --no-install-recommends ros-humble-nvblox-ros ros-humble-tf2-ros-py \
      ros-humble-tf-transformations python3-transforms3d \
 && wget -qO /usr/share/keyrings/jetson-ota.asc https://repo.download.nvidia.com/jetson/jetson-ota-public.asc \
 && echo "deb [signed-by=/usr/share/keyrings/jetson-ota.asc] https://repo.download.nvidia.com/jetson/x86_64/jammy r36.4 main" \
      > /etc/apt/sources.list.d/nvidia-vpi.list \
 && apt-get update && apt-get install -y --no-install-recommends libnpp-12-8 libnvvpi3 \
 && rm -rf /var/lib/apt/lists/*
ENV LD_LIBRARY_PATH=/opt/nvidia/vpi3/lib/x86_64-linux-gnu:/usr/local/cuda/lib64:${LD_LIBRARY_PATH}
