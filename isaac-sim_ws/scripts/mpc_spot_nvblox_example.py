#!/usr/bin/env python3
#
# Copyright (c) 2023 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
#
# Adapted for Boston Dynamics Spot Arm with Nvblox ROS2 integration
#

try:
    # Third Party
    import isaacsim
except ImportError:
    pass

# Third Party
import torch

a = torch.zeros(4, device="cuda:0")

# Standard Library
import argparse

parser = argparse.ArgumentParser()

parser.add_argument(
    "--headless_mode",
    type=str,
    default=None,
    help="To run headless, use one of [native, websocket], webrtc might not work.",
)
parser.add_argument(
    "--visualize_spheres",
    action="store_true",
    help="When True, visualizes robot spheres",
    default=False,
)
parser.add_argument(
    "--robot", type=str, default="/workspace/config/spot_arm.yml", help="robot configuration to load"
)
parser.add_argument(
    "--esdf_topic",
    type=str,
    default="/nvblox_node/pessimistic_static_esdf_pointcloud",
    help="ESDF pointcloud topic from nvblox",
)
parser.add_argument(
    "--update_hz",
    type=float,
    default=2.0,
    help="How often to update obstacles from nvblox (Hz)",
)
args = parser.parse_args()

###########################################################

# Third Party
from omni.isaac.kit import SimulationApp

simulation_app = SimulationApp(
    {
        "headless": args.headless_mode is not None,
        "width": "1920",
        "height": "1080",
    }
)

# Third Party
import os
import threading
from typing import Optional

import carb
import numpy as np
from helper import add_extensions, add_robot_to_scene
from omni.isaac.core import World
from omni.isaac.core.objects import cuboid
from omni.isaac.core.utils.types import ArticulationAction

# Enable ROS2 bridge extension
import omni.ext

ext_manager = omni.kit.app.get_app().get_extension_manager()
ext_manager.set_extension_enabled_immediate("omni.isaac.ros2_bridge", True)

# ROS2 imports (available after bridge is enabled)
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
# Note: tf2_ros requires full ROS2 install. Uncomment when ROS2 is available:
# from tf2_ros import Buffer, TransformListener, TransformException
import struct

# CuRobo
from curobo.geom.sdf.world import CollisionCheckerType
from curobo.geom.types import WorldConfig
from curobo.rollout.rollout_base import Goal
from curobo.types.base import TensorDeviceType
from curobo.types.math import Pose
from curobo.types.robot import JointState
from curobo.types.state import JointState
from curobo.util.logger import setup_curobo_logger
from curobo.util.usd_helper import UsdHelper
from curobo.util_file import get_robot_configs_path, get_world_configs_path, join_path, load_yaml
from curobo.wrap.reacher.mpc import MpcSolver, MpcSolverConfig

############################################################


def pointcloud2_to_array(msg: PointCloud2) -> np.ndarray:
    """Convert PointCloud2 message to numpy array with x, y, z, intensity."""
    # Get field offsets
    fields = {f.name: (f.offset, f.datatype) for f in msg.fields}
    
    point_step = msg.point_step
    data = msg.data
    n_points = msg.width * msg.height
    
    if n_points == 0:
        return np.zeros((0, 4), dtype=np.float32)
    
    points = np.zeros((n_points, 4), dtype=np.float32)
    
    for i in range(n_points):
        offset = i * point_step
        # x, y, z
        points[i, 0] = struct.unpack_from('f', data, offset + fields['x'][0])[0]
        points[i, 1] = struct.unpack_from('f', data, offset + fields['y'][0])[0]
        points[i, 2] = struct.unpack_from('f', data, offset + fields['z'][0])[0]
        # intensity (distance to nearest obstacle)
        if 'intensity' in fields:
            points[i, 3] = struct.unpack_from('f', data, offset + fields['intensity'][0])[0]
        else:
            points[i, 3] = 0.0
    
    # Filter out NaN and Inf values
    valid_mask = np.isfinite(points).all(axis=1)
    return points[valid_mask]


class NvbloxEsdfSubscriber(Node):
    """ROS2 subscriber for nvblox ESDF pointcloud."""
    
    def __init__(self, topic: str):
        super().__init__('curobo_nvblox_subscriber')
        self.subscription = self.create_subscription(
            PointCloud2,
            topic,
            self.esdf_callback,
            10
        )
        self.latest_esdf = None
        self.latest_frame_id = None
        self.esdf_lock = threading.Lock()
        self.get_logger().info(f"Subscribed to {topic}")
    
    def esdf_callback(self, msg: PointCloud2):
        esdf_array = pointcloud2_to_array(msg)
        with self.esdf_lock:
            self.latest_esdf = esdf_array
            self.latest_frame_id = msg.header.frame_id
    
    def get_esdf(self) -> Optional[np.ndarray]:
        with self.esdf_lock:
            return self.latest_esdf.copy() if self.latest_esdf is not None else None
    
    def get_frame_id(self) -> Optional[str]:
        with self.esdf_lock:
            return self.latest_frame_id



def transform_points(points: np.ndarray, transform) -> np.ndarray:
    """
    Apply a TF transform to points.
    
    Args:
        points: Nx3 or Nx4 array of points
        transform: geometry_msgs/TransformStamped
    
    Returns:
        Transformed points (same shape as input)
    """
    if points is None or len(points) == 0:
        return points
    
    # Extract translation and rotation from transform
    t = transform.transform.translation
    q = transform.transform.rotation
    
    # Convert quaternion to rotation matrix
    # q = [x, y, z, w]
    qx, qy, qz, qw = q.x, q.y, q.z, q.w
    
    # Rotation matrix from quaternion
    R = np.array([
        [1 - 2*(qy**2 + qz**2), 2*(qx*qy - qz*qw), 2*(qx*qz + qy*qw)],
        [2*(qx*qy + qz*qw), 1 - 2*(qx**2 + qz**2), 2*(qy*qz - qx*qw)],
        [2*(qx*qz - qy*qw), 2*(qy*qz + qx*qw), 1 - 2*(qx**2 + qy**2)]
    ])
    
    translation = np.array([t.x, t.y, t.z])
    
    # Apply transform to xyz
    transformed = points.copy()
    xyz = points[:, :3]
    transformed[:, :3] = (R @ xyz.T).T + translation
    
    return transformed


def esdf_to_cuboid_world(esdf_points: np.ndarray, 
                          voxel_size: float = 0.05,
                          distance_threshold: float = 0.01,
                          max_obstacles: int = 100) -> Optional[WorldConfig]:
    """
    Convert ESDF pointcloud to cuRobo WorldConfig with Cuboid obstacles.
    
    Args:
        esdf_points: Nx4 array with [x, y, z, distance]
        voxel_size: Size of cuboid obstacles
        distance_threshold: Points with distance < threshold are obstacles
        max_obstacles: Maximum number of obstacles to create
    
    Returns:
        WorldConfig with Cuboid obstacles, or None if no obstacles
    """
    if esdf_points is None or len(esdf_points) == 0:
        return None
    
    # Filter points that are obstacles (distance < threshold)
    # For pessimistic ESDF, smaller/negative values = inside/near obstacle
    occupied_mask = esdf_points[:, 3] < distance_threshold
    occupied_points = esdf_points[occupied_mask, :3]
    
    if len(occupied_points) == 0:
        return None
    
    # Voxelize: round points to voxel grid and get unique centers
    voxel_centers = np.round(occupied_points / voxel_size) * voxel_size
    unique_centers = np.unique(voxel_centers, axis=0)
    
    # Limit number of obstacles for performance
    if len(unique_centers) > max_obstacles:
        # Sample randomly
        indices = np.random.choice(len(unique_centers), max_obstacles, replace=False)
        unique_centers = unique_centers[indices]
    
    # Create Cuboid obstacles
    from curobo.geom.types import Cuboid
    cuboids = []
    for i, center in enumerate(unique_centers):
        cuboid = Cuboid(
            name=f"nvblox_obs_{i}",
            dims=[voxel_size, voxel_size, voxel_size],
            pose=[center[0], center[1], center[2], 1.0, 0.0, 0.0, 0.0],
        )
        cuboids.append(cuboid)
    
    return WorldConfig(cuboid=cuboids)


def draw_points(rollouts: torch.Tensor):
    if rollouts is None:
        return
    import random

    try:
        from omni.isaac.debug_draw import _debug_draw
    except ImportError:
        from isaacsim.util.debug_draw import _debug_draw
    draw = _debug_draw.acquire_debug_draw_interface()
    N = 100
    draw.clear_points()
    cpu_rollouts = rollouts.cpu().numpy()
    b, h, _ = cpu_rollouts.shape
    point_list = []
    colors = []
    for i in range(b):
        point_list += [
            (cpu_rollouts[i, j, 0], cpu_rollouts[i, j, 1], cpu_rollouts[i, j, 2]) for j in range(h)
        ]
        colors += [(1.0 - (i + 1.0 / b), 0.3 * (i + 1.0 / b), 0.0, 0.1) for _ in range(h)]
    sizes = [10.0 for _ in range(b * h)]
    draw.draw_points(point_list, colors, sizes)


def draw_esdf_points(esdf_points: np.ndarray, threshold: float = 0.0):
    """Draw ESDF occupied points in blue for visualization."""
    if esdf_points is None or len(esdf_points) == 0:
        return
    
    try:
        from omni.isaac.debug_draw import _debug_draw
    except ImportError:
        from isaacsim.util.debug_draw import _debug_draw
    
    draw = _debug_draw.acquire_debug_draw_interface()
    
    # Filter occupied points
    occupied_mask = esdf_points[:, 3] < threshold
    occupied = esdf_points[occupied_mask, :3]
    
    if len(occupied) == 0:
        return
    
    # Limit number of points for performance
    max_points = 5000
    if len(occupied) > max_points:
        indices = np.random.choice(len(occupied), max_points, replace=False)
        occupied = occupied[indices]
    
    point_list = [(p[0], p[1], p[2]) for p in occupied]
    colors = [(0.0, 0.5, 1.0, 0.5) for _ in range(len(occupied))]
    sizes = [5.0 for _ in range(len(occupied))]
    
    draw.draw_points(point_list, colors, sizes)


def main():
    # Initialize ROS2
    rclpy.init()
    nvblox_sub = NvbloxEsdfSubscriber(args.esdf_topic)
    
    # Create executor for spinning ROS2 in background
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(nvblox_sub)
    
    def spin_ros2():
        while rclpy.ok():
            executor.spin_once(timeout_sec=0.01)
    
    ros2_thread = threading.Thread(target=spin_ros2, daemon=True)
    ros2_thread.start()
    
    # Note: TF2 requires full ROS2 install. For sim, assuming odom == body.
    # Uncomment when ROS2 is available:
    # tf_buffer = Buffer()
    # tf_listener = TransformListener(tf_buffer, nvblox_sub)
    
    # Setup Isaac Sim world
    my_world = World(stage_units_in_meters=1.0)
    stage = my_world.stage

    xform = stage.DefinePrim("/World", "Xform")
    stage.SetDefaultPrim(xform)
    stage.DefinePrim("/curobo", "Xform")
    stage = my_world.stage
    my_world.scene.add_default_ground_plane()

    # Make a target to follow - positioned in front of the robot
    target = cuboid.VisualCuboid(
        "/World/target",
        position=np.array([0.5, 0.0, 0.5]),
        orientation=np.array([0, 1, 0, 0]),
        color=np.array([1.0, 0, 0]),
        size=0.05,
    )

    setup_curobo_logger("warn")
    past_pose = None

    usd_help = UsdHelper()

    tensor_args = TensorDeviceType()

    robot_cfg = load_yaml(args.robot)["robot_cfg"]

    j_names = robot_cfg["kinematics"]["cspace"]["joint_names"]
    default_config = robot_cfg["kinematics"]["cspace"]["retract_config"]
    robot_base_frame = robot_cfg["kinematics"]["base_link"]  # e.g., 'body' for Spot
    robot_cfg["kinematics"]["collision_sphere_buffer"] += 0.02

    # Add robot to scene
    robot, robot_prim_path = add_robot_to_scene(robot_cfg, my_world)

    articulation_controller = robot.get_articulation_controller()

    # Initial empty world config for voxel-based collision
    # Will be updated with nvblox data
    world_cfg = WorldConfig()

    init_curobo = False

    tensor_args = TensorDeviceType()
    robot_cfg = load_yaml(args.robot)["robot_cfg"]

    j_names = robot_cfg["kinematics"]["cspace"]["joint_names"]
    default_config = robot_cfg["kinematics"]["cspace"]["retract_config"]

    # MPC Configuration with VOXEL collision checker
    mpc_config = MpcSolverConfig.load_from_robot_config(
        robot_cfg,
        world_cfg,
        use_cuda_graph=True,
        use_cuda_graph_metrics=True,
        use_cuda_graph_full_step=False,
        self_collision_check=True,
        collision_checker_type=CollisionCheckerType.PRIMITIVE,
        collision_cache={"obb": 250},  # Large cache for nvblox obstacles
        use_mppi=True,
        use_lbfgs=False,
        use_es=False,
        store_rollouts=True,
        step_dt=0.03,
    )

    mpc = MpcSolver(mpc_config)

    retract_cfg = mpc.rollout_fn.dynamics_model.retract_config.clone().unsqueeze(0)
    joint_names = mpc.rollout_fn.joint_names

    state = mpc.rollout_fn.compute_kinematics(
        JointState.from_position(retract_cfg, joint_names=joint_names)
    )
    current_state = JointState.from_position(retract_cfg, joint_names=joint_names)
    retract_pose = Pose(state.ee_pos_seq, quaternion=state.ee_quat_seq)
    goal = Goal(
        current_state=current_state,
        goal_state=JointState.from_position(retract_cfg, joint_names=joint_names),
        goal_pose=retract_pose,
    )

    goal_buffer = mpc.setup_solve_single(goal, 1)
    mpc.update_goal(goal_buffer)
    mpc_result = mpc.step(current_state, max_attempts=2)

    usd_help.load_stage(my_world.stage)

    init_world = False
    cmd_state_full = None
    step = 0
    add_extensions(simulation_app, args.headless_mode)
    
    # Calculate update interval based on Hz
    update_interval = int(60 / args.update_hz)  # Assuming ~60 Hz sim rate
    
    print("=" * 60)
    print("Spot Arm MPC Example with Nvblox Integration")
    print("=" * 60)
    print(f"Robot: {args.robot}")
    print(f"Joint names: {j_names}")
    print(f"ESDF topic: {args.esdf_topic}")
    print(f"Obstacle update rate: {args.update_hz} Hz")
    print("Move the red target cube to control the arm!")
    print("=" * 60)
    
    nvblox_initialized = False
    
    while simulation_app.is_running():
        if not init_world:
            for _ in range(10):
                my_world.step(render=True)
            init_world = True
        draw_points(mpc.get_visual_rollouts())

        my_world.step(render=True)
        if not my_world.is_playing():
            continue

        step_index = my_world.current_time_step_index

        if step_index <= 10:
            robot._articulation_view.initialize()
            idx_list = [robot.get_dof_index(x) for x in j_names]
            robot.set_joint_positions(default_config, idx_list)
            robot._articulation_view.set_max_efforts(
                values=np.array([5000 for i in range(len(idx_list))]), joint_indices=idx_list
            )
            # Aumentar damping dos joints para reduzir oscilações
            robot._articulation_view.set_gains(
                kps=np.array([1000.0 for i in range(len(idx_list))]),
                kds=np.array([100.0 for i in range(len(idx_list))]),
                joint_indices=idx_list
            )

        if not init_curobo:
            init_curobo = True
        step += 1
        step_index = step
        
        # Update obstacles from nvblox
        if step_index % update_interval == 0:
            esdf_data = nvblox_sub.get_esdf()
            if esdf_data is not None and len(esdf_data) > 0:
                if not nvblox_initialized:
                    print(f"Nvblox connected! Received {len(esdf_data)} ESDF points")
                    nvblox_initialized = True
                
                # Visualize ESDF points
                draw_esdf_points(esdf_data, threshold=0.01)
                
                # Note: For simulation, assuming odom == body (no transform needed)
                # When ROS2 is fully installed, uncomment TF lookup below:
                # source_frame = nvblox_sub.get_frame_id()
                # if source_frame and source_frame != robot_base_frame:
                #     try:
                #         transform = tf_buffer.lookup_transform(
                #             robot_base_frame, source_frame,
                #             rclpy.time.Time(),
                #             timeout=rclpy.duration.Duration(seconds=0.1)
                #         )
                #         esdf_data = transform_points(esdf_data, transform)
                #     except TransformException as e:
                #         print(f"TF warning: {e}")
                
                # Convert ESDF to cuboid obstacles and update collision model
                nvblox_world = esdf_to_cuboid_world(
                    esdf_data, 
                    voxel_size=0.05,  # 5cm cuboids
                    distance_threshold=0.01,  # Points with ESDF < 1cm are obstacles
                    max_obstacles=200  # Limit for performance
                )
                
                if nvblox_world is not None and len(nvblox_world.cuboid) > 0:
                    try:
                        mpc.world_coll_checker.load_collision_model(nvblox_world)
                        if step_index % (update_interval * 10) == 0:
                            print(f"Updated collision model with {len(nvblox_world.cuboid)} obstacles")
                    except Exception as e:
                        if step_index % (update_interval * 10) == 0:
                            print(f"Warning: Could not update collision model: {e}")
                
                if step_index % (update_interval * 10) == 0:
                    print(f"ESDF points: {len(esdf_data)}, "
                          f"Occupied: {np.sum(esdf_data[:, 3] < 0.01)}")


        # Get target position and orientation
        cube_position, cube_orientation = target.get_world_pose()

        if past_pose is None:
            past_pose = cube_position + 1.0

        if np.linalg.norm(cube_position - past_pose) > 1e-3:
            ee_translation_goal = cube_position
            ee_orientation_teleop_goal = cube_orientation
            ik_goal = Pose(
                position=tensor_args.to_device(ee_translation_goal),
                quaternion=tensor_args.to_device(ee_orientation_teleop_goal),
            )
            goal_buffer.goal_pose.copy_(ik_goal)
            mpc.update_goal(goal_buffer)
            past_pose = cube_position

        # Get robot current state
        sim_js = robot.get_joints_state()
        if sim_js is None:
            print("sim_js is None")
            continue
        js_names = robot.dof_names
        sim_js_names = robot.dof_names

        cu_js = JointState(
            position=tensor_args.to_device(sim_js.positions),
            velocity=tensor_args.to_device(sim_js.velocities) * 0.7,
            acceleration=tensor_args.to_device(sim_js.velocities) * 0.0,
            jerk=tensor_args.to_device(sim_js.velocities) * 0.0,
            joint_names=sim_js_names,
        )
        cu_js = cu_js.get_ordered_joint_state(mpc.rollout_fn.joint_names)
        if cmd_state_full is None:
            current_state.copy_(cu_js)
        else:
            current_state_partial = cmd_state_full.get_ordered_joint_state(
                mpc.rollout_fn.joint_names
            )
            current_state.copy_(current_state_partial)
            current_state.joint_names = current_state_partial.joint_names
        common_js_names = []
        current_state.copy_(cu_js)

        mpc_result = mpc.step(current_state, max_attempts=2)

        succ = True
        cmd_state_full = mpc_result.js_action
        common_js_names = []
        idx_list = []
        for x in sim_js_names:
            if x in cmd_state_full.joint_names:
                idx_list.append(robot.get_dof_index(x))
                common_js_names.append(x)

        cmd_state = cmd_state_full.get_ordered_joint_state(common_js_names)
        cmd_state_full = cmd_state

        art_action = ArticulationAction(
            cmd_state.position.view(-1).cpu().numpy(),
            joint_indices=idx_list,
        )
        
        if step_index % 1000 == 0:
            print(f"Feasible: {mpc_result.metrics.feasible.item()}, "
                  f"Pose error: {mpc_result.metrics.pose_error.item():.4f}")

        if succ:
            for _ in range(1):
                articulation_controller.apply_action(art_action)
        else:
            carb.log_warn("No action is being taken.")
    
    # Cleanup ROS2
    nvblox_sub.destroy_node()
    rclpy.shutdown()


############################################################

if __name__ == "__main__":
    main()
    simulation_app.close()
