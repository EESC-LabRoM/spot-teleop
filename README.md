# Spot ROS2 Multi-Workspace

This repository contains multiple ROS2 workspaces for robotics with Boston Dynamics Spot, ZED cameras, RealSense, and Isaac Sim simulation.

## Workspace Structure

* Main workspace for Spot ROS2 + RealSense + MoveIt
* Workspace for ZED + Isaac ROS + NVBlox
* Workspace for Isaac Sim + ZED Isaac Sim

## Cloning

```bash
# Clone with all submodules
git clone --recursive https://github.com/murilo-vinicius04/spot-teleop.git
cd spot-teleop

# Or if you already cloned, initialize the submodules
git submodule update --init --recursive

# Configure Git LFS for isaac_ros_nitros
cd zed_ws/src/isaac_ros_nitros
git lfs install
git lfs pull
cd ../../..
```

## Build with Docker

### Prerequisites

* Docker
* Docker Compose
* NVIDIA Docker Runtime (for GPU)

### Building Containers

```bash
# Build all containers
docker-compose build

# Or build individually
docker-compose build spot-ros2
docker-compose build zed
docker-compose build isaac-sim
```

## Running

### Spot ROS2 + RealSense

```bash
docker-compose up -d spot-ros2
```

### ZED + NVBlox

```bash
docker-compose up -d zed
```

### Isaac Sim

```bash
docker-compose up -d isaac-sim
```

### All services

```bash
docker-compose up -d
```

## Included Submodules

### ZED Workspace

* `isaac_ros_nitros` - NVIDIA Isaac ROS Nitros
* `negotiated` - Negotiated QoS
* `zed-ros2-wrapper` - ZED ROS2 Wrapper
* `isaac_ros_nvblox` - NVIDIA Isaac ROS NVBlox
* `zed-ros2-interfaces` - ZED ROS2 Interfaces
* `isaac_ros_common` - NVIDIA Isaac ROS Common

### Spot ROS2 Workspace

* `spot_ros2` - Boston Dynamics Spot ROS2
* `moveit2` - MoveIt2 Motion Planning
* `moveit_msgs` - MoveIt2 Messages
* `moveit_resources` - MoveIt2 Resources
* `moveit_task_constructor` - MoveIt Task Constructor
* `moveit2_tutorials` - MoveIt2 Tutorials

### Isaac Sim Workspace

* `zed-isaac-sim` - ZED Isaac Sim Integration

---

## Setting Up Isaac Sim with ZED Integration (inside the isaac-sim container)

1. Navigate to the **`zed-isaac-sim`** folder and build:

   ```bash
   cd zed-isaac-sim
   ./build.sh
   ```

2. After the build completes, start Isaac Sim:

   ```bash
   cd /isaac-sim
   ./runapp.sh
   ```

3. When Isaac Sim opens, go to:

   * `Windows -> Extensions -> Third Party -> Settings`
   * Add a **User Dir** pointing to: `/workspace/zed-isaac-sim/exts`

   (Follow the [official ZED Isaac Sim README](https://github.com/stereolabs/zed-isaac-sim) for reference.)

4. Enable the extensions and set them to **Auto Load**.

5. Restart Isaac Sim.

6. Open the provided scene:

   * `File -> Open`
   * Select: `/workspace/zed_streamer_warehouse`

7. The scene should load and be ready for streaming. Click **Play** and check the terminal where Isaac Sim was launched. If you see:

   ```
   [Streaming] Use Transport layer mode : 0
   ```

   The container is correctly set up.

---

## Setting Up ZED Container and NVBlox Mapping

1. Ensure the ROS2 environment is sourced:

   ```bash
   source /opt/ros/humble/setup.bash
   ```

2. Build the workspace with the recommended flags:

   ```bash
   colcon build --merge-install --symlink-install
   ```

3. Source the local install:

   ```bash
   source install/setup.bash
   ```

4. With Isaac Sim streaming, launch the ZED wrapper:

   ```bash
   ros2 launch zed_wrapper zed_camera.launch.py camera_model:=zedx sim_mode:=true
   ```

   > This will optimize the ZED neural mode for your GPU. It may take several minutes on the first run.

5. To visualize the NVBlox map, run the ZED example:

   ```bash
   ros2 launch nvblox_examples_bringup zed_example.launch.py
   ```
