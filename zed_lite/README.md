# zed_lite: virtual ZED X (Isaac Sim 6.0) -> ZED SDK 5.4.1 -> ROS 2 Humble, without the Isaac ROS image

(Real lab cameras on the remote host, streams, nvblox and calibration: see `HANDOFF_REAL_LAB.md`.)

Pieces (2026-10-14):
- Isaac side: image `spot-isaac-sim:6.0.0-zed` (= 6.0.0 + libpng16/libjpeg8/libturbojpeg/libusb, built from
  `isaac-sim_ws/franka_mpc/docker/Dockerfile.isaac-zed`) + Stereolabs `zed-isaac-sim` v5.1.0 (Isaac Sim 6.0.0 / Kit 110.1.1)
  unpacked in `isaac-sim_ws/zed-isaac-sim-5.1.0`. Streamer: `isaac-sim_ws/franka_mpc/zed_stream.py` (workcell, ZED_X, IPC+network).
- Receiver: image `zed-lite:5.4.1-humble` (this Dockerfile; ZED SDK 5.4.1 runtime + ROS 2 Humble ros-base, 5.8 GB) and
  zed-ros2-wrapper v5.4.1 in `src/`, built into `build/` + `install/` (the repo's zed_ws submodule is 2025-07 and does not
  compile against SDK 5.4). `sim_static.yaml`: fixed camera, positional tracking off (the stream has no usable IMU and
  the node aborts otherwise).

Run (Isaac first, then the receiver):
    cd isaac-sim_ws/franka_mpc && IMAGE=spot-isaac-sim:6.0.0-zed ./run_isaac.sh -s zed_stream.py zed_stream --seconds 300 --resolution HD1080
    zed_lite/run_zed.sh                     # docker stop zed-lite to end
Check: `check_topics.py` (rates + one frame), `preview_check.py check <franka_scene run>` (vs Isaac exact depth).

Traps:
- A receiver that dies while connected leaves a held lock in /dev/shm/sl_local_*_30000; the next receiver then hangs after
  "Backward compatibility required". zed_stream.py deletes those files at start; stop the receiver before the streamer.
- The first receiver run optimises the NEURAL_LIGHT model for the GPU (~6 min); cached in `zed_resources/`.
- Wrapper >= 5.1 renamed topics: colour is `/zed/zed_node/rgb/color/rect/image` (was `rgb/image_rect_color`; the
  wrist_detector defaults still use the old name).

## nvblox (2026-10-14): `nvblox/`
- Image `nvblox-lite:3.2.5-humble` = zed-lite + `ros-humble-nvblox-ros` 3.2.5 from NVIDIA's Isaac ROS apt repo
  (release-3) + `libnpp-12-8` + `libnvvpi3` (x86_64 Jetson repo): 7.2 GB total, ~1.4 GB on top of zed-lite. Not the
  `isaac-ros-nvblox` metapackage (TensorRT, Visual SLAM, a driver package).
- `start_mapping.sh` / `stop_mapping.sh`: Isaac stream -> ZED wrapper -> nvblox + static TF, in order; stop in reverse.
- `zed_extrinsics_tf.py`: world -> fr3_link0 (Franka FR3 base; was panda_link0 before 10-14) and fr3_link0 -> zed_camera_link, chosen so the wrapper's
  left optical frame lands on Isaac's CameraLeft. The ZED node republishes `zed_camera_center -> zed_left_camera_frame`
  on /tf from the SDK calibration (lens 10 mm behind centre; the URDF's static says 5 mm) and that wins in steady state,
  so the script re-anchors on it (1 s check). Listeners must spin ~3 s before trusting the lens transform.
- `franka_zed_nvblox.launch.py`: spot_nvblox settings (2 cm, static TSDF, 3D ESDF, 5 Hz integration), global frame
  fr3_link0, the `*_frame_id` params that default to base_link set to the global frame.
- `map_check.py` (in the container) + `map_plot.py` (host): ESDF service grid vs the scene.
- Measured: ESDF service 7-42 ms for 49x91x36 voxels; TF 0.0 mm / 0.0 deg vs Isaac; drill top 0.23 m (true 0.228),
  bottles 0.41/0.43 (0.417), beaker 0.17 (0.15); table top maps 3 cm low because the ZED reads +2.9 cm far on the black
  table (+1.0 cm elsewhere, from the sim's fx 741.6 in the SDK vs 736.0 rendered). The Franka arm is in the map (no
  robot self-filter yet). Unobserved shadows behind objects (one camera).

## Robot self-filter: cuRobo RobotSegmenter (tested offline 2026-10-14)
- cuRobo's `RobotSegmenter` (what Isaac ROS cuMotion's robot_segmenter node wraps) runs in Isaac's Python 3.12 next to
  our existing cuRobo build (curobo-isaac6), with Isaac 6.0's bundled ROS 2 Humble rclpy
  (`/isaac-sim/exts/isaacsim.ros2.core/humble/rclpy`, + `humble/lib` on LD_LIBRARY_PATH) and Warp from
  `/isaac-sim/extscache/omni.warp.core-1.13.0+lx64`. No new container, no PyTorch install, no rebuild.
- `nvblox/segment_test.py` (recorded ZED frame, Franka at retract, franka.yml 61 spheres): masked px 2 cm 41.7k, 5 cm 48.7k,
  10 cm 52.7k, 20 cm 59.5k; ~8 ms per 1920x1080 frame after warm-up. `segment_plot.py` -> out/segtest/segment_test.png.
  Command: docker run --gpus all --network host --ipc host -e ROS_DISTRO=humble -e RMW_IMPLEMENTATION=rmw_fastrtps_cpp
  -e LD_LIBRARY_PATH=/isaac-sim/exts/isaacsim.ros2.core/humble/lib -e PYTHONPATH=<warp>:<ml_archive>:<omni.pip.compute>:
  /workspace/curobo-isaac6/src:<venv site-packages>:/workspace/franka_mpc/pydeps:/isaac-sim/exts/isaacsim.ros2.core/humble/rclpy
  -e SETUPTOOLS_SCM_PRETEND_VERSION=0.7.7.post1.dev5 -v isaac-sim_ws:/workspace:ro -v zed_lite:/zl
  --entrypoint /isaac-sim/python.sh spot-isaac-sim:6.0.0 /zl/nvblox/segment_test.py /zl/nvblox/out/segtest

## Robot self-filter, live (2026-10-14): `nvblox/franka_segmenter.py` + `run_segmenter.sh`
- Chain: Isaac (`zed_stream.py`, ROS=1: /joint_states at 100 Hz from Isaac's bundled Humble rclpy, stamped with sim time,
  the same clock as the ZED images) -> ZED wrapper -> segmenter (Isaac image, Python 3.12, curobo-isaac6 + bundled rclpy;
  cuRobo RobotSegmenter, franka.yml spheres, threshold 0.05 m + sphere buffer 0.01 m, camera pose from TF) ->
  /franka_seg/depth -> nvblox (`self_filter:=true`, default). `start_mapping.sh` runs all of it: `ARM=sweep` swings the arm.
- QoS: the 2 MB depth frames between Isaac's bundled ROS libs and the containers mostly dropped as best effort (2 Hz in);
  reliable in/out gives ~11 Hz through the segmenter, ~13-15 ms per 960x540 frame, joint state within 5 ms of the depth.
- Measured with the arm swinging (map_compare.py): obstacle voxels inside the arm's swept volume 990 (raw depth) -> 39
  (segmenter), 96% removed; all object heights unchanged. Without the filter the lower arm column (links 0-2, always in
  view) is what stays in the map; nvblox clears most of the moving upper arm by itself.
