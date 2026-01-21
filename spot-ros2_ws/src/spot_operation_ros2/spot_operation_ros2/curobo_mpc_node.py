#!/usr/bin/env python3
"""
cuRobo MPC Node - Standalone motion planning for Spot Arm

Subscribes to pose goals from wrist detector and publishes joint commands
with position, velocity, and effort (via Pinocchio inverse dynamics).

Subscribed Topics:
    /wrist_pose (geometry_msgs/PoseStamped): Target pose (frame: body)
    /joint_states_isaac (sensor_msgs/JointState): Current joint states

Published Topics:
    /joint_command_curobo (sensor_msgs/JointState): pos + vel + effort

Parameters:
    debug_mode (bool): If true, generates test poses automatically without /wrist_pose
"""

import os
import math
import random
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from geometry_msgs.msg import PoseStamped, TransformStamped
from sensor_msgs.msg import JointState, PointCloud2
import struct
import threading

# TF2 for coordinate transformations
import tf2_ros
from tf2_ros import Buffer, TransformListener

import torch
import numpy as np

# cuRobo imports
from curobo.geom.sdf.world import CollisionCheckerType
from curobo.geom.types import WorldConfig, Cuboid
from curobo.rollout.rollout_base import Goal
from curobo.types.base import TensorDeviceType
from curobo.types.math import Pose
from curobo.types.state import JointState as CuJointState
from curobo.util.logger import setup_curobo_logger
from curobo.util_file import load_yaml, get_world_configs_path, join_path
from curobo.wrap.reacher.mpc import MpcSolver, MpcSolverConfig

# Pinocchio for inverse dynamics
try:
    import pinocchio as pin
    PINOCCHIO_AVAILABLE = True
except ImportError:
    PINOCCHIO_AVAILABLE = False


class CuroboMpcNode(Node):
    """ROS2 Node for cuRobo MPC-based motion planning."""

    def __init__(self):
        super().__init__('curobo_mpc_node')

        # Declare parameters
        self.declare_parameter('robot_config', '')
        self.declare_parameter('urdf_path', '')
        self.declare_parameter('control_rate', 30.0)
        self.declare_parameter('step_dt', 0.03)
        self.declare_parameter('use_effort', True)
        self.declare_parameter('debug_mode', False)
        self.declare_parameter('debug_pose_duration', 3.0)  # seconds between pose changes
        
        # nvblox ESDF parameters
        self.declare_parameter('esdf_topic', '/nvblox_node/pessimistic_static_esdf_pointcloud')
        self.declare_parameter('esdf_update_rate', 2.0)  # Hz
        self.declare_parameter('esdf_voxel_size', 0.05)  # meters
        self.declare_parameter('esdf_distance_threshold', 0.02)  # meters
        self.declare_parameter('esdf_max_obstacles', 150)

        # Get parameters
        robot_config_path = self.get_parameter('robot_config').get_parameter_value().string_value
        urdf_path = self.get_parameter('urdf_path').get_parameter_value().string_value
        control_rate = self.get_parameter('control_rate').get_parameter_value().double_value
        step_dt = self.get_parameter('step_dt').get_parameter_value().double_value
        self.use_effort = self.get_parameter('use_effort').get_parameter_value().bool_value
        self.debug_mode = self.get_parameter('debug_mode').get_parameter_value().bool_value
        self.debug_pose_duration = self.get_parameter('debug_pose_duration').get_parameter_value().double_value
        
        # nvblox parameters
        self.esdf_topic = self.get_parameter('esdf_topic').get_parameter_value().string_value
        self.esdf_update_rate = self.get_parameter('esdf_update_rate').get_parameter_value().double_value
        self.esdf_voxel_size = self.get_parameter('esdf_voxel_size').get_parameter_value().double_value
        self.esdf_distance_threshold = self.get_parameter('esdf_distance_threshold').get_parameter_value().double_value
        self.esdf_max_obstacles = self.get_parameter('esdf_max_obstacles').get_parameter_value().integer_value

        # Default paths
        if not robot_config_path:
            try:
                from ament_index_python.packages import get_package_share_directory
                pkg_share = get_package_share_directory('spot_operation_ros2')
                robot_config_path = os.path.join(pkg_share, 'config', 'spot_arm.yml')
                os.environ['CUROBO_CONFIG_PATH'] = os.path.join(pkg_share, 'config')
            except Exception:
                robot_config_path = '/home/nexus/spot-teleop/isaac-sim_ws/config/spot_arm.yml'
                os.environ['CUROBO_CONFIG_PATH'] = '/home/nexus/spot-teleop/isaac-sim_ws/config'

        if not urdf_path:
            try:
                from ament_index_python.packages import get_package_share_directory
                urdf_path = os.path.join(
                    get_package_share_directory('spot_description'),
                    'urdf', 'standalone_arm_fixed.urdf'
                )
            except Exception:
                urdf_path = '/home/nexus/spot-teleop/spot-ros2_ws/src/spot_ros2/spot_description/spot_description/urdf/standalone_arm_fixed.urdf'

        os.environ['SPOT_URDF_PATH'] = os.path.dirname(urdf_path)

        self.get_logger().info('=== cuRobo MPC Node Starting ===')
        self.get_logger().info(f'Robot config: {robot_config_path}')
        self.get_logger().info(f'URDF path: {urdf_path}')
        self.get_logger().info(f'Control rate: {control_rate} Hz')
        self.get_logger().info(f'Use effort (Pinocchio): {self.use_effort}')
        self.get_logger().info(f'Debug mode: {self.debug_mode}')
        self.get_logger().info(f'ESDF topic: {self.esdf_topic}')
        self.get_logger().info(f'ESDF update rate: {self.esdf_update_rate} Hz')

        # Initialize cuRobo
        setup_curobo_logger("warn")
        self.tensor_args = TensorDeviceType()

        # Load robot config - need to resolve env vars in paths
        robot_cfg_raw = load_yaml(robot_config_path)["robot_cfg"]
        
        # Update paths in config
        robot_cfg_raw["kinematics"]["external_asset_path"] = os.path.dirname(urdf_path)
        spheres_path = os.path.join(os.environ['CUROBO_CONFIG_PATH'], 'spheres', 'spot_arm.yml')
        robot_cfg_raw["kinematics"]["collision_spheres"] = spheres_path
        
        self.robot_cfg = robot_cfg_raw
        self.j_names = self.robot_cfg["kinematics"]["cspace"]["joint_names"]
        self.default_config = self.robot_cfg["kinematics"]["cspace"]["retract_config"]
        self.robot_cfg["kinematics"]["collision_sphere_buffer"] += 0.02

        self.get_logger().info(f'Joint names: {self.j_names}')

        # Initialize Pinocchio for inverse dynamics
        self.pin_model = None
        self.pin_data = None
        if self.use_effort and PINOCCHIO_AVAILABLE:
            try:
                self.pin_model = pin.buildModelFromUrdf(urdf_path)
                self.pin_data = self.pin_model.createData()
                self.get_logger().info('Pinocchio model loaded for inverse dynamics')
            except Exception as e:
                self.get_logger().warn(f'Failed to load Pinocchio model: {e}')
                self.use_effort = False
        elif self.use_effort and not PINOCCHIO_AVAILABLE:
            self.get_logger().warn('Pinocchio not available, effort will be empty')
            self.use_effort = False

        # World config (empty for now)
        world_cfg = WorldConfig()

        # MPC Configuration
        self.get_logger().info('Loading MPC solver...')
        mpc_config = MpcSolverConfig.load_from_robot_config(
            self.robot_cfg,
            world_cfg,
            use_cuda_graph=True,
            use_cuda_graph_metrics=True,
            use_cuda_graph_full_step=False,
            self_collision_check=True,
            collision_checker_type=CollisionCheckerType.PRIMITIVE,
            collision_cache={"obb": 200},  # Large cache for nvblox dynamic obstacles
            collision_activation_distance=0.03,  # Start avoiding obstacles from 3cm away
            use_mppi=True,
            use_lbfgs=False,
            use_es=False,
            store_rollouts=False,
            step_dt=step_dt,
        )

        self.mpc = MpcSolver(mpc_config)
        self.get_logger().info('MPC solver loaded!')

        # Initialize state
        retract_cfg = self.mpc.rollout_fn.dynamics_model.retract_config.clone().unsqueeze(0)
        joint_names = self.mpc.rollout_fn.joint_names

        state = self.mpc.rollout_fn.compute_kinematics(
            CuJointState.from_position(retract_cfg, joint_names=joint_names)
        )
        self.current_state = CuJointState.from_position(retract_cfg, joint_names=joint_names)
        retract_pose = Pose(state.ee_pos_seq, quaternion=state.ee_quat_seq)

        goal = Goal(
            current_state=self.current_state,
            goal_state=CuJointState.from_position(retract_cfg, joint_names=joint_names),
            goal_pose=retract_pose,
        )

        self.goal_buffer = self.mpc.setup_solve_single(goal, 1)
        self.mpc.update_goal(self.goal_buffer)
        _ = self.mpc.step(self.current_state, max_attempts=2)  # Warm up

        # State variables
        self.last_goal_pose = None
        self.current_joint_state = None
        self.cmd_state_full = None
        self.goal_received = False
        self.joints_received = False

        # nvblox ESDF state
        self.latest_esdf = None
        self.latest_esdf_frame = 'odom'  # Will be updated from message
        self.esdf_lock = threading.Lock()
        self.nvblox_initialized = False
        self.obstacle_update_count = 0
        
        # TF2 for frame transformations
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.target_frame = 'body'  # cuRobo expects obstacles in base_link frame

        # Debug mode state
        self.debug_pose_index = 0
        self.debug_last_pose_time = self.get_clock().now()
        self.debug_test_poses = self._generate_test_poses()

        # QoS for real-time
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=1
        )

        # Publishers/Subscribers
        self.cmd_pub = self.create_publisher(JointState, '/joint_command_curobo', 10)
        self.pose_sub = self.create_subscription(PoseStamped, '/wrist_pose', self.pose_callback, qos)
        self.joint_sub = self.create_subscription(JointState, '/joint_states_isaac', self.joint_state_callback, qos)
        
        # nvblox ESDF subscriber
        self.esdf_sub = self.create_subscription(
            PointCloud2, 
            self.esdf_topic, 
            self.esdf_callback, 
            qos
        )
        self.get_logger().info(f'Subscribed to ESDF topic: {self.esdf_topic}')

        # Control timer
        self.control_timer = self.create_timer(1.0 / control_rate, self.control_loop)
        self.step_count = 0
        
        # Obstacle update timer (slower than control loop)
        self.obstacle_update_timer = self.create_timer(
            1.0 / self.esdf_update_rate, 
            self.update_obstacles_callback
        )

        self.get_logger().info('=== cuRobo MPC Node Ready ===')
        if self.debug_mode:
            self.get_logger().info('DEBUG MODE ACTIVE - Using test poses')
            self.get_logger().info(f'Will cycle through {len(self.debug_test_poses)} test positions')
            self.goal_received = True  # Auto-ready in debug mode
        else:
            self.get_logger().info('Waiting for /wrist_pose and /joint_states_isaac...')

    def _generate_test_poses(self):
        """Generate a list of test poses for debug mode."""
        # Define test positions in front of the robot (x forward, z up)
        # These are reachable positions for the Spot arm
        test_poses = [
            # (x, y, z, roll, pitch, yaw) - position + euler angles
            (0.5, 0.0, 0.0, 0.0, 0.0, 0.0),      # Forward center
            (0.4, 0.2, 0.1, 0.0, 0.0, 0.0),      # Forward right up
            (0.4, -0.2, 0.1, 0.0, 0.0, 0.0),     # Forward left up
            (0.5, 0.0, -0.2, 0.0, 0.5, 0.0),     # Forward center down (pitched)
            (0.3, 0.3, 0.2, 0.0, 0.0, 0.5),      # Right high
            (0.3, -0.3, 0.2, 0.0, 0.0, -0.5),    # Left high
            (0.6, 0.0, 0.1, 0.0, -0.3, 0.0),     # Extended forward
            (0.35, 0.0, 0.3, 0.0, -0.8, 0.0),    # High center
        ]
        return test_poses

    def _euler_to_quaternion(self, roll, pitch, yaw):
        """Convert euler angles to quaternion (w, x, y, z)."""
        cr = math.cos(roll / 2)
        sr = math.sin(roll / 2)
        cp = math.cos(pitch / 2)
        sp = math.sin(pitch / 2)
        cy = math.cos(yaw / 2)
        sy = math.sin(yaw / 2)

        w = cr * cp * cy + sr * sp * sy
        x = sr * cp * cy - cr * sp * sy
        y = cr * sp * cy + sr * cp * sy
        z = cr * cp * sy - sr * sp * cy
        return (w, x, y, z)

    def _get_debug_pose(self):
        """Get current debug pose, cycling through test poses."""
        now = self.get_clock().now()
        elapsed = (now - self.debug_last_pose_time).nanoseconds / 1e9

        if elapsed >= self.debug_pose_duration:
            self.debug_pose_index = (self.debug_pose_index + 1) % len(self.debug_test_poses)
            self.debug_last_pose_time = now
            pose_data = self.debug_test_poses[self.debug_pose_index]
            self.get_logger().info(
                f'DEBUG: Switching to pose {self.debug_pose_index}: '
                f'pos=({pose_data[0]:.2f}, {pose_data[1]:.2f}, {pose_data[2]:.2f})'
            )

        pose_data = self.debug_test_poses[self.debug_pose_index]
        x, y, z, roll, pitch, yaw = pose_data
        w, qx, qy, qz = self._euler_to_quaternion(roll, pitch, yaw)

        position = self.tensor_args.to_device([x, y, z])
        quaternion = self.tensor_args.to_device([w, qx, qy, qz])

        return Pose(position=position, quaternion=quaternion)

    def pose_callback(self, msg: PoseStamped):
        """Handle incoming pose goal."""
        if self.debug_mode:
            return  # Ignore external poses in debug mode

        pos = msg.pose.position
        ori = msg.pose.orientation

        position = self.tensor_args.to_device([pos.x, pos.y, pos.z])
        quaternion = self.tensor_args.to_device([ori.w, ori.x, ori.y, ori.z])

        self.last_goal_pose = Pose(position=position, quaternion=quaternion)
        self.goal_received = True

    def joint_state_callback(self, msg: JointState):
        """Handle incoming joint state."""
        self.current_joint_state = msg
        self.joints_received = True

    def esdf_callback(self, msg: PointCloud2):
        """Handle incoming ESDF pointcloud from nvblox."""
        esdf_array = self._pointcloud2_to_array(msg)
        with self.esdf_lock:
            self.latest_esdf = esdf_array
            self.latest_esdf_frame = msg.header.frame_id

    def _pointcloud2_to_array(self, msg: PointCloud2) -> np.ndarray:
        """Convert PointCloud2 message to numpy array with x, y, z, intensity."""
        fields = {f.name: (f.offset, f.datatype) for f in msg.fields}
        point_step = msg.point_step
        data = msg.data
        n_points = msg.width * msg.height
        
        if n_points == 0:
            return np.zeros((0, 4), dtype=np.float32)
        
        points = np.zeros((n_points, 4), dtype=np.float32)
        
        for i in range(n_points):
            offset = i * point_step
            points[i, 0] = struct.unpack_from('f', data, offset + fields['x'][0])[0]
            points[i, 1] = struct.unpack_from('f', data, offset + fields['y'][0])[0]
            points[i, 2] = struct.unpack_from('f', data, offset + fields['z'][0])[0]
            if 'intensity' in fields:
                points[i, 3] = struct.unpack_from('f', data, offset + fields['intensity'][0])[0]
        
        # Filter NaN/Inf
        valid_mask = np.isfinite(points).all(axis=1)
        return points[valid_mask]

    def _transform_points(self, esdf_data: np.ndarray, transform: TransformStamped) -> np.ndarray:
        """Transform ESDF points from source frame to target frame."""
        t = transform.transform.translation
        q = transform.transform.rotation
        
        # Quaternion to rotation matrix
        # q = [w, x, y, z] but ROS uses [x, y, z, w]
        qx, qy, qz, qw = q.x, q.y, q.z, q.w
        
        # Rotation matrix from quaternion
        rot = np.array([
            [1 - 2*(qy**2 + qz**2), 2*(qx*qy - qz*qw), 2*(qx*qz + qy*qw)],
            [2*(qx*qy + qz*qw), 1 - 2*(qx**2 + qz**2), 2*(qy*qz - qx*qw)],
            [2*(qx*qz - qy*qw), 2*(qy*qz + qx*qw), 1 - 2*(qx**2 + qy**2)]
        ], dtype=np.float32)
        
        translation = np.array([t.x, t.y, t.z], dtype=np.float32)
        
        # Transform xyz (keep intensity as-is)
        transformed = esdf_data.copy()
        xyz = esdf_data[:, :3]
        transformed[:, :3] = (rot @ xyz.T).T + translation
        
        return transformed

    def _esdf_to_cuboid_world(self, esdf_points: np.ndarray):
        """Convert ESDF pointcloud to cuRobo WorldConfig with Cuboid obstacles."""
        if esdf_points is None or len(esdf_points) == 0:
            return None
        
        # Filter obstacles (ESDF < threshold = inside/near obstacle)
        occupied_mask = esdf_points[:, 3] < self.esdf_distance_threshold
        occupied_points = esdf_points[occupied_mask, :3]
        
        if len(occupied_points) == 0:
            return None
        
        # Voxelize to get unique centers
        voxel_centers = np.round(occupied_points / self.esdf_voxel_size) * self.esdf_voxel_size
        unique_centers = np.unique(voxel_centers, axis=0)
        
        # Limit for performance
        if len(unique_centers) > self.esdf_max_obstacles:
            indices = np.random.choice(len(unique_centers), self.esdf_max_obstacles, replace=False)
            unique_centers = unique_centers[indices]
        
        # Create cuboids
        cuboids = []
        for i, center in enumerate(unique_centers):
            cuboid = Cuboid(
                name=f"nvblox_obs_{i}",
                dims=[self.esdf_voxel_size, self.esdf_voxel_size, self.esdf_voxel_size],
                pose=[float(center[0]), float(center[1]), float(center[2]), 1.0, 0.0, 0.0, 0.0],
            )
            cuboids.append(cuboid)
        
        return WorldConfig(cuboid=cuboids)

    def update_obstacles_callback(self):
        """Periodically update collision model with nvblox ESDF data."""
        with self.esdf_lock:
            esdf_data = self.latest_esdf.copy() if self.latest_esdf is not None else None
            source_frame = self.latest_esdf_frame
        
        if esdf_data is None or len(esdf_data) == 0:
            return
        
        # Transform points from ESDF frame to body frame
        if source_frame != self.target_frame:
            try:
                transform = self.tf_buffer.lookup_transform(
                    self.target_frame,
                    source_frame,
                    rclpy.time.Time(),
                    timeout=rclpy.duration.Duration(seconds=0.1)
                )
                esdf_data = self._transform_points(esdf_data, transform)
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, 
                    tf2_ros.ExtrapolationException) as e:
                if not self.nvblox_initialized:
                    self.get_logger().warn(f'Waiting for TF {source_frame} -> {self.target_frame}: {e}')
                return
        
        if not self.nvblox_initialized:
            self.get_logger().info(f'nvblox connected! Received {len(esdf_data)} ESDF points')
            self.get_logger().info(f'Transforming obstacles from frame \"{source_frame}\" -> \"{self.target_frame}\"')
            self.nvblox_initialized = True
        
        # Convert ESDF to cuboid obstacles
        nvblox_world = self._esdf_to_cuboid_world(esdf_data)
        
        if nvblox_world is not None and len(nvblox_world.cuboid) > 0:
            try:
                self.mpc.update_world(nvblox_world)
                self.obstacle_update_count += 1
                
                # Log every 10 updates with obstacle bounds
                if self.obstacle_update_count % 10 == 0:
                    occupied = np.sum(esdf_data[:, 3] < self.esdf_distance_threshold)
                    occupied_pts = esdf_data[esdf_data[:, 3] < self.esdf_distance_threshold, :3]
                    if len(occupied_pts) > 0:
                        mins = occupied_pts.min(axis=0)
                        maxs = occupied_pts.max(axis=0)
                        self.get_logger().info(
                            f'Obstacle update #{self.obstacle_update_count}: '
                            f'{len(nvblox_world.cuboid)} cuboids from {occupied} pts | '
                            f'Bounds: X[{mins[0]:.2f},{maxs[0]:.2f}] Y[{mins[1]:.2f},{maxs[1]:.2f}] Z[{mins[2]:.2f},{maxs[2]:.2f}]'
                        )
                    else:
                        self.get_logger().info(
                            f'Obstacle update #{self.obstacle_update_count}: '
                            f'{len(nvblox_world.cuboid)} cuboids from {occupied} occupied points'
                        )
            except Exception as e:
                self.get_logger().warn(f'Failed to update collision model: {e}')

    def compute_effort(self, q, v, a):
        """Compute torques using Pinocchio inverse dynamics."""
        if not self.use_effort or self.pin_model is None:
            return []

        try:
            # Need to match joint order - Pinocchio uses URDF order
            q_pin = np.array(q)
            v_pin = np.array(v)
            a_pin = np.array(a)

            # RNEA: τ = M(q)·a + C(q,v)·v + g(q)
            tau = pin.rnea(self.pin_model, self.pin_data, q_pin, v_pin, a_pin)
            return tau.tolist()
        except Exception as e:
            self.get_logger().warn(f'Effort computation failed: {e}')
            return []

    def control_loop(self):
        """Main MPC control loop."""
        # In debug mode, we don't need external goal
        if self.debug_mode:
            self.last_goal_pose = self._get_debug_pose()
            # In debug mode, if no joints received, use retract config
            if not self.joints_received:
                # Use default joint state for debugging
                if self.current_joint_state is None:
                    self.current_joint_state = JointState()
                    self.current_joint_state.name = list(self.j_names)
                    self.current_joint_state.position = list(self.default_config)
                    self.current_joint_state.velocity = [0.0] * len(self.j_names)
                    self.joints_received = True
        else:
            if not self.goal_received or not self.joints_received:
                return
            if self.current_joint_state is None or self.last_goal_pose is None:
                return

        try:
            positions = list(self.current_joint_state.position)
            velocities = list(self.current_joint_state.velocity) if self.current_joint_state.velocity else [0.0] * len(positions)
            joint_names_msg = list(self.current_joint_state.name)

            cu_js = CuJointState(
                position=self.tensor_args.to_device(positions),
                velocity=self.tensor_args.to_device(velocities) * 0.7,
                acceleration=self.tensor_args.to_device(velocities) * 0.0,
                jerk=self.tensor_args.to_device(velocities) * 0.0,
                joint_names=joint_names_msg,
            )

            cu_js = cu_js.get_ordered_joint_state(self.mpc.rollout_fn.joint_names)

            if self.cmd_state_full is None:
                self.current_state.copy_(cu_js)
            else:
                current_state_partial = self.cmd_state_full.get_ordered_joint_state(
                    self.mpc.rollout_fn.joint_names
                )
                self.current_state.copy_(current_state_partial)

            self.current_state.copy_(cu_js)

            self.goal_buffer.goal_pose.copy_(self.last_goal_pose)
            self.mpc.update_goal(self.goal_buffer)

            mpc_result = self.mpc.step(self.current_state, max_attempts=2)
            
            # Get collision/feasibility info
            is_feasible = mpc_result.metrics.feasible.item()
            coll_cost = 0.0
            coll_constraint = 0.0
            if mpc_result.metrics.cost is not None:
                coll_cost = mpc_result.metrics.cost.item()
            if mpc_result.metrics.constraint is not None:
                coll_constraint = mpc_result.metrics.constraint.item()
            
            self.step_count += 1
            
            # Always use MPC result - it will slide along obstacle surfaces
            self.cmd_state_full = mpc_result.js_action

            ordered_names = [n for n in joint_names_msg if n in self.cmd_state_full.joint_names]
            if not ordered_names:
                return

            cmd_state = self.cmd_state_full.get_ordered_joint_state(ordered_names)

            pos_list = cmd_state.position.view(-1).cpu().numpy().tolist()
            vel_list = cmd_state.velocity.view(-1).cpu().numpy().tolist()
            acc_list = cmd_state.acceleration.view(-1).cpu().numpy().tolist()

            # Compute effort via inverse dynamics
            effort_list = self.compute_effort(pos_list, vel_list, acc_list)

            joint_cmd = JointState()
            joint_cmd.header.stamp = self.get_clock().now().to_msg()
            joint_cmd.name = ordered_names
            joint_cmd.position = pos_list
            joint_cmd.velocity = vel_list
            joint_cmd.effort = effort_list

            self.cmd_pub.publish(joint_cmd)
            
            # Rate-limited logging
            if self.step_count % 100 == 0:
                status = "BLOCKED" if not is_feasible else "OK"
                self.get_logger().info(
                    f'Step {self.step_count} [{status}]: '
                    f'Error={mpc_result.metrics.pose_error.item():.4f}, '
                    f'Constraint={coll_constraint:.4f}'
                )

        except Exception as e:
            self.get_logger().error(f'Control loop error: {e}')


def main(args=None):
    rclpy.init(args=args)

    if not torch.cuda.is_available():
        print("ERROR: CUDA not available! cuRobo requires CUDA.")
        return

    node = CuroboMpcNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
