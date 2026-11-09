# Lightweight ZED receiver for the Isaac Sim virtual ZED: ZED SDK 5.4.1 runtime (CUDA 12.8) + ROS 2 Humble ros-base +
# what zed-ros2-wrapper needs to build (nvcc + cudart headers, BLAS, libusb dev, its ROS deps). No full CUDA toolkit, no
# Isaac ROS. The wrapper is built from zed_lite/src (zed-ros2-wrapper v5.4.1, matching SDK 5.4.1; the repo's zed_ws
# submodule is from 2025-07 and no longer compiles against SDK 5.4) into zed_lite/{build,install}.
#   docker build -t zed-lite:5.4.1-humble -f zed_lite/Dockerfile zed_lite/ctx      (empty context)
FROM stereolabs/zed:5.4.1-runtime-cuda12.8-ubuntu22.04
SHELL ["/bin/bash", "-c"]
ENV DEBIAN_FRONTEND=noninteractive LANG=C.UTF-8 LC_ALL=C.UTF-8 ROS_DISTRO=humble \
    RMW_IMPLEMENTATION=rmw_fastrtps_cpp ROS_DOMAIN_ID=8
RUN apt-get update && apt-get install -y --no-install-recommends curl gnupg2 ca-certificates lsb-release \
 && curl -fsSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg \
 && echo "deb [arch=amd64 signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu jammy main" \
      > /etc/apt/sources.list.d/ros2.list \
 && apt-get update && apt-get install -y --no-install-recommends \
      ros-humble-ros-base ros-humble-rmw-fastrtps-cpp python3-colcon-common-extensions build-essential cmake git \
      cuda-nvcc-12-8 cuda-cudart-dev-12-8 libopenblas-dev libusb-1.0-0-dev \
      ros-humble-ament-cmake-auto ros-humble-backward-ros ros-humble-diagnostic-updater ros-humble-diagnostic-msgs \
      ros-humble-geographic-msgs ros-humble-nmea-msgs ros-humble-image-transport ros-humble-compressed-image-transport \
      ros-humble-compressed-depth-image-transport ros-humble-point-cloud-transport ros-humble-stereo-msgs \
      ros-humble-shape-msgs ros-humble-visualization-msgs ros-humble-std-srvs ros-humble-tf2-geometry-msgs \
      ros-humble-robot-localization ros-humble-xacro ros-humble-robot-state-publisher \
      ros-humble-rosidl-default-generators \
 && rm -rf /var/lib/apt/lists/*
# wrapper v5.x: zed_msgs and zed_description come from apt now; nlohmann-json for zed_components
RUN apt-get update && apt-get install -y --no-install-recommends ros-humble-zed-msgs ros-humble-zed-description \
      nlohmann-json3-dev \
 && rm -rf /var/lib/apt/lists/*
ENV PATH=/usr/local/cuda/bin:${PATH}
RUN echo "source /opt/ros/humble/setup.bash" >> /etc/bash.bashrc \
 && echo "[ -f /ws/install/setup.bash ] && source /ws/install/setup.bash" >> /etc/bash.bashrc
WORKDIR /ws
