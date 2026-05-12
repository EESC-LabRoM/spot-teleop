import os
import csv
import tempfile
import yaml
import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from geometry_msgs.msg import PoseStamped, TransformStamped, Point, Vector3, PointStamped
from sensor_msgs.msg import JointState, PointCloud2
import threading
import torch
import numpy as np
import time
import sys
from datetime import datetime

# Ensure nvblox_msgs is findable when running inside the cuRobo venv
_NVBLOX_PATH = '/home/spot-teleop/spot-ros2_ws/install/nvblox_msgs/local/lib/python3.10/dist-packages'
if _NVBLOX_PATH not in sys.path:
    sys.path.append(_NVBLOX_PATH)

import tf2_ros
from tf2_ros import Buffer, TransformListener

# cuRobo imports
from curobo.geom.sdf.world import CollisionCheckerType, CollisionQueryBuffer
from curobo.geom.types import VoxelGrid as CuVoxelGrid, WorldConfig
from curobo.rollout.rollout_base import Goal
from curobo.types.base import TensorDeviceType
from curobo.types.math import Pose
from curobo.types.state import JointState as CuJointState
from curobo.util.logger import setup_curobo_logger
from curobo.util_file import load_yaml
from curobo.wrap.reacher.mpc import MpcSolver, MpcSolverConfig

from nvblox_msgs.srv import EsdfAndGradients

try:
    from spot_msgs.msg import JointCommand
except ImportError:
    JointCommand = None



class CuroboMpcNode(Node):
    """ROS2 Node for cuRobo MPC-based motion planning."""

    def __init__(self):
        super().__init__('curobo_mpc_node')

        # Declare parameters
        self.declare_parameter('robot_config', '')
        self.declare_parameter('urdf_path', '')
        self.declare_parameter('control_rate', 50.0)
        self.declare_parameter('step_dt', 0.02)
        self.declare_parameter('log_csv_path', '')

        self.declare_parameter('debug_mode', False)
        self.declare_parameter('debug_pose_duration', 3.0)  # seconds between pose changes
        self.declare_parameter('use_sim', True)  # True=sim (arm0_ prefix), False=real (arm_ prefix)
        self.declare_parameter('use_ros2_control', False)  # True=publish JointCommand to spot_joint_controller
        self.declare_parameter('startup_delay', 5.0)  # Wait N seconds before considering obstacles

        # ESDF / nvblox parameters
        self.declare_parameter('use_esdf', True)
        self.declare_parameter('esdf_service_name', '/nvblox_node/get_esdf_and_gradient')
        self.declare_parameter('esdf_update_rate', 1.0)   # Hz
        self.declare_parameter('voxel_size', 0.05)
        self.declare_parameter('grid_center_m', [0.0, 0.0, 0.0])
        self.declare_parameter('grid_size_m', [4.0, 4.0, 2.0])
        self.declare_parameter('esdf_frame_id', 'body')   # robot base frame (cuRobo planning frame)
        self.declare_parameter('esdf_global_frame', 'odom')  # nvblox global frame (odom in sim, vision on real)

        # Get parameters
        robot_config_path = self.get_parameter('robot_config').get_parameter_value().string_value
        urdf_path = self.get_parameter('urdf_path').get_parameter_value().string_value
        self.control_rate = self.get_parameter('control_rate').get_parameter_value().double_value
        self.step_dt = self.get_parameter('step_dt').get_parameter_value().double_value
        self.log_csv_path = self.get_parameter('log_csv_path').get_parameter_value().string_value

        self._init_csv_logger()

        self.debug_mode = self.get_parameter('debug_mode').get_parameter_value().bool_value
        self.debug_pose_duration = self.get_parameter('debug_pose_duration').get_parameter_value().double_value
        self.use_sim = self.get_parameter('use_sim').get_parameter_value().bool_value
        self.use_ros2_control = self.get_parameter('use_ros2_control').get_parameter_value().bool_value
        self.startup_delay = self.get_parameter('startup_delay').get_parameter_value().double_value

        self.use_esdf = self.get_parameter('use_esdf').get_parameter_value().bool_value
        self.esdf_service_name = self.get_parameter('esdf_service_name').get_parameter_value().string_value
        self.esdf_update_rate = self.get_parameter('esdf_update_rate').get_parameter_value().double_value
        self.__voxel_size = self.get_parameter('voxel_size').get_parameter_value().double_value
        self.__grid_center_m = self.get_parameter('grid_center_m').get_parameter_value().double_array_value
        self.__grid_size_m = list(self.get_parameter('grid_size_m').get_parameter_value().double_array_value)
        self.target_frame = 'body'  # Frame for target object transforms
        self._esdf_query_frame = self.get_parameter('esdf_frame_id').get_parameter_value().string_value
        self._esdf_global_frame = self.get_parameter('esdf_global_frame').get_parameter_value().string_value

        # ... (Keeping default paths logic)
        os.environ['SPOT_URDF_PATH'] = os.path.dirname(urdf_path)

        self._log_info('cuRobo MPC Node Starting', event='startup')
        self._log_info(f'Robot config: {robot_config_path}', event='startup')
        self._log_info(f'URDF path: {urdf_path}', event='startup')
        self._log_info(f'Control rate: {self.control_rate} Hz', event='startup')
        self._log_info(f'Use sim: {self.use_sim} ({"arm0_ prefix" if self.use_sim else "arm_ prefix"})', event='startup')
        self._log_info(f'Use ESDF: {self.use_esdf} ({self.esdf_service_name} @ {self.esdf_update_rate} Hz)', event='startup')

        # Initialize cuRobo
        setup_curobo_logger("warn")
        self.tensor_args = TensorDeviceType()

        # Load robot config - need to resolve env vars in paths
        robot_cfg_raw = load_yaml(robot_config_path)["robot_cfg"]
        
        # Update paths in config
        robot_cfg_raw["kinematics"]["external_asset_path"] = os.path.dirname(urdf_path)
        spheres_path = os.path.join(os.environ['CUROBO_CONFIG_PATH'], 'spheres', 'spot_arm.yml')

        if self.use_sim:
            # Sim uses arm0_ joint names but standalone_arm_fixed.urdf has arm_ prefix links/joints.
            # Generate a temp URDF with arm_ -> arm0_ so cuRobo's kinematic chain resolves correctly.
            urdf_src = os.path.join(os.path.dirname(urdf_path), 'standalone_arm_fixed.urdf')
            with open(urdf_src, 'r') as f:
                urdf_text = f.read()
            urdf_text = urdf_text.replace('arm_', 'arm0_')
            tmp_urdf = tempfile.NamedTemporaryFile(mode='w', suffix='.urdf', delete=False)
            tmp_urdf.write(urdf_text)
            tmp_urdf.close()
            robot_cfg_raw["kinematics"]["external_asset_path"] = os.path.dirname(tmp_urdf.name)
            robot_cfg_raw["kinematics"]["urdf_path"] = os.path.basename(tmp_urdf.name)
            robot_cfg_raw["kinematics"]["collision_spheres"] = spheres_path
            self._log_info(f'Sim: generated arm0_-prefixed URDF at {tmp_urdf.name}', event='config')
        else:
            # Real: prefix arm_. Remap config arm0_ -> arm_ and temp spheres.
            robot_cfg_raw = self._remap_config_prefix(robot_cfg_raw, 'arm0_', 'arm_')
            spheres_data = load_yaml(spheres_path)
            spheres_data = self._remap_config_prefix(spheres_data, 'arm0_', 'arm_')
            tmp_spheres = tempfile.NamedTemporaryFile(mode='w', suffix='.yml', delete=False)
            yaml.dump(spheres_data, tmp_spheres)
            tmp_spheres.close()
            robot_cfg_raw["kinematics"]["collision_spheres"] = tmp_spheres.name
            self._log_info(f'Real: remapped config arm0_ -> arm_. Temp spheres: {tmp_spheres.name}', event='config')
        
        self.robot_cfg = robot_cfg_raw
        self.j_names = self.robot_cfg["kinematics"]["cspace"]["joint_names"]
        self.default_config = self.robot_cfg["kinematics"]["cspace"]["retract_config"]
        self.robot_cfg["kinematics"]["collision_sphere_buffer"] += 0.02

        self._log_info(f'Joint names (URDF): {self.j_names}', event='config')

        # TF buffer (needed for AABB query transformation)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Voxel world: pre-allocates the GPU ESDF buffer — update_voxel_data() fills it in-place
        # Use identity pose - ESDF data coordinates are transformed during AABB query
        world_cfg = WorldConfig.from_dict({
            'voxel': {
                'world_voxel': {
                    'dims': self.__grid_size_m,
                    'pose': [0, 0, 0, 1, 0, 0, 0],  # Identity pose
                    'voxel_size': self.__voxel_size,
                    'feature_dtype': torch.float32,
                },
            },
        })

        self._log_info('Loading MPC solver...', event='mpc')
        mpc_config = MpcSolverConfig.load_from_robot_config(
            self.robot_cfg,
            world_cfg,
            use_cuda_graph=True,
            use_cuda_graph_metrics=False,
            use_cuda_graph_full_step=False,
            self_collision_check=True,
            collision_checker_type=CollisionCheckerType.VOXEL,
            override_particle_file=os.path.join(os.path.dirname(robot_config_path), 'mpc_override.yml'),
            use_mppi=True,
            use_lbfgs=False,
            use_es=False,
            store_rollouts=False,
            step_dt=self.step_dt,
        )

        self.mpc = MpcSolver(mpc_config)
        self._log_info('MPC solver loaded!', event='mpc')

        # Last tracked targets for CSV observability
        self._last_wrist_target = (float('nan'), float('nan'), float('nan'))
        self._last_target_object = (float('nan'), float('nan'), float('nan'))
        self._last_target_dist = float('nan')

        # ... (Keeping MPC init state)
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

        self.__world_collision = self.mpc.world_coll_checker
        self.__cumotion_grid_shape = self.__world_collision.get_voxel_grid('world_voxel').get_grid_shape()[0]

        # Initialize goal pose to use current robot position initially
        self.last_goal_pose = retract_pose

        # State variables
        self.last_goal_pose = None
        self.current_joint_state = None
        self._pose_msg = None
        self.cmd_state_full = None
        self.goal_received = False
        self.joints_received = False
        self._esdf_initialized = not self.use_esdf  # skip gate if ESDF disabled
        self._esdf_call_pending = False  # guard against overlapping async calls
        self._last_esdf_wait_log = 0.0
        self._robot_pos_global = None  # Store body position in global frame for ESDF transform

        self._state_lock = threading.Lock()  # protects current_joint_state and _pose_msg

        # Debug mode state
        self.debug_pose_index = 0
        self.debug_last_pose_time = self.get_clock().now()
        self.debug_test_poses = self._generate_test_poses()

        # Callback groups
        self._control_cb_group = MutuallyExclusiveCallbackGroup()
        self._sensor_cb_group = MutuallyExclusiveCallbackGroup()
        self._esdf_cb_group = MutuallyExclusiveCallbackGroup()

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=1
        )

        # Publishers/Subscribers
        if self.use_sim:
            cmd_topic = '/joint_command_curobo'
            joint_topic = '/joint_states_isaac'
        elif self.use_ros2_control:
            cmd_topic = '/spot_joint_controller/joint_commands'
            joint_topic = '/joint_states'
        else:
            cmd_topic = '/arm_joint_trajectory_commands'
            joint_topic = '/joint_states'

        if self.use_ros2_control and not self.use_sim:
            if JointCommand is None:
                self._log_fatal('use_ros2_control=True but spot_msgs.msg.JointCommand not found!')
                raise ImportError('spot_msgs.msg.JointCommand not available')
            self.cmd_pub = self.create_publisher(JointCommand, cmd_topic, 10)
        else:
            self.cmd_pub = self.create_publisher(JointState, cmd_topic, 10)

        # ESDF visualization publisher
        self.esdf_pub = self.create_publisher(PointCloud2, '/esdf_viz', 10)

        # Subscribe to sensors (fast updates)
        self.pose_sub = self.create_subscription(
            PoseStamped,
            '/wrist_pose',
            self.pose_callback,
            qos,
            callback_group=self._sensor_cb_group,
        )
        self.joint_sub = self.create_subscription(
            JointState,
            joint_topic,
            self.joint_state_callback,
            qos,
            callback_group=self._sensor_cb_group,
        )
        
        self._log_info(f'Subscribed to Joint topic: {joint_topic}')
        self._log_info(f'Publishing to Command topic: {cmd_topic}')

        self.timer = self.create_timer(
            1.0 / self.control_rate,
            self.control_loop,
            callback_group=self._control_cb_group,
        )
        self.step_count = 0
        self.start_time = time.time()

        # ESDF service client — runs in a raw Python thread (NOT an executor callback)
        # call_async inside executor callbacks deadlocks; threading.Event avoids it
        if self.use_esdf:
            self.__esdf_client = self.create_client(
                EsdfAndGradients, self.esdf_service_name,
                callback_group=self._esdf_cb_group
            )
            self.__esdf_req = EsdfAndGradients.Request()

            # ESDF update timer - like upstream mesh updates in control loop
            self.esdf_timer = self.create_timer(
                1.0 / self.esdf_update_rate,
                self._update_esdf,
                callback_group=self._esdf_cb_group,
            )
            self._log_info(f'ESDF service: {self.esdf_service_name} @ {self.esdf_update_rate} Hz')

        # Performance monitoring
        self._last_loop_time = time.time()
        self._loop_times = []  # track last N loop durations
        self._step_times = []  # track mpc.step() durations
        self._publish_count = 0
        self._joint_state_age = 0.0  # how old is the joint state data
        self._last_joint_stamp = None

        # GPU monitoring via pynvml - use same device as torch
        self._gpu_monitor_available = False
        try:
            import pynvml
            pynvml.nvmlInit()
            gpu_idx = torch.cuda.current_device()
            self._gpu_handle = pynvml.nvmlDeviceGetHandleByIndex(gpu_idx)
            gpu_name = pynvml.nvmlDeviceGetName(self._gpu_handle)
            self._gpu_monitor_available = True
            self._log_info(f'GPU monitoring enabled: GPU {gpu_idx} ({gpu_name})')
        except Exception as e:
            self._log_warn(f'pynvml not available, using torch.cuda only: {e}')

        self._log_info('=== cuRobo MPC Node Ready ===')
        if self.debug_mode:
            self._log_info('DEBUG MODE ACTIVE - Using test poses')
            self._log_info(f'Will cycle through {len(self.debug_test_poses)} test positions')
            self.goal_received = True  # Auto-ready in debug mode
        else:
            # Get joint topic name for the log message
            joint_topic = '/joint_states_isaac' if self.use_sim else '/joint_states'
            self._log_info(f'Waiting for /wrist_pose and {joint_topic}...')

    # ... (Keeping _generate_test_poses, _euler_to_quaternion, _get_debug_pose, pose_callback, joint_state_callback)

    def _remap_config_prefix(self, config, old_prefix, new_prefix):
        """Recursively remap string prefixes in a config dict/list."""
        if isinstance(config, dict):
            return {
                (k.replace(old_prefix, new_prefix) if isinstance(k, str) else k):
                self._remap_config_prefix(v, old_prefix, new_prefix)
                for k, v in config.items()
            }
        elif isinstance(config, list):
            return [self._remap_config_prefix(item, old_prefix, new_prefix) for item in config]
        elif isinstance(config, str):
            return config.replace(old_prefix, new_prefix)
        return config

    def _init_csv_logger(self):
        if not self.log_csv_path:
            stamp = datetime.now().strftime('%Y%m%d_%H%M%S')
            self.log_csv_path = os.path.join(os.getcwd(), f'curobo_mpc_log_{stamp}.csv')

        log_dir = os.path.dirname(self.log_csv_path)
        if log_dir:
            os.makedirs(log_dir, exist_ok=True)

        self._csv_fields = [
            'timestamp', 'level', 'event', 'message', 'step',
            'wrist_target_x', 'wrist_target_y', 'wrist_target_z',
            'target_object_x', 'target_object_y', 'target_object_z', 'target_distance',
            'pose_error', 'constraint', 'coll_cost', 'evasion_dev',
            'loop_ms', 'mpc_step_ms', 'gap_ms',
            'avg_loop_ms', 'avg_step_ms', 'effective_hz',
            'gpu_util', 'gpu_mem_used_mb', 'gpu_mem_total_mb', 'gpu_temp_c',
            'torch_alloc_mb', 'torch_reserved_mb',
            'js_age_ms', 'publish_count', 'min_dist', 'closest_sphere',
            'cmd_pos', 'curr_pos', 'details'
        ]

        self._csv_lock = threading.Lock()
        self._csv_fh = open(self.log_csv_path, 'a', newline='', encoding='utf-8')
        self._csv_writer = csv.DictWriter(self._csv_fh, fieldnames=self._csv_fields)
        if self._csv_fh.tell() == 0:
            self._csv_writer.writeheader()
            self._csv_fh.flush()

    def _write_csv_log(self, level: str, message: str, event: str = 'general', **fields):
        logger = self.get_logger()
        if level == 'DEBUG':
            logger.debug(message)
        elif level == 'WARN':
            logger.warn(message)
        elif level == 'ERROR':
            logger.error(message)
        elif level == 'FATAL':
            logger.fatal(message)
        else:
            logger.info(message)

        if not hasattr(self, '_csv_writer'):
            return

        row = {k: '' for k in self._csv_fields}
        row['timestamp'] = datetime.now().isoformat(timespec='milliseconds')
        row['level'] = level
        row['event'] = event
        row['message'] = message

        details_parts = []
        for key, value in fields.items():
            if key in row:
                row[key] = value
            else:
                details_parts.append(f'{key}={value}')
        row['details'] = ';'.join(details_parts)

        with self._csv_lock:
            self._csv_writer.writerow(row)
            self._csv_fh.flush()

    def _log_info(self, message: str, event: str = 'general', **fields):
        fields.pop('once', None)
        self._write_csv_log('INFO', message, event=event, **fields)

    def _log_warn(self, message: str, event: str = 'general', **fields):
        fields.pop('once', None)
        self._write_csv_log('WARN', message, event=event, **fields)

    def _log_error(self, message: str, event: str = 'general', **fields):
        fields.pop('once', None)
        self._write_csv_log('ERROR', message, event=event, **fields)

    def _log_debug(self, message: str, event: str = 'general', **fields):
        fields.pop('once', None)
        self._write_csv_log('DEBUG', message, event=event, **fields)

    def _log_fatal(self, message: str, event: str = 'general', **fields):
        fields.pop('once', None)
        self._write_csv_log('FATAL', message, event=event, **fields)



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
            self._log_info(
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
        """Handle incoming pose goal (position + orientation)."""
        if self.debug_mode:
            return

        # Respect startup delay — tensor update happens in control_loop
        if time.time() - self.start_time < self.startup_delay:
            return

        with self._state_lock:
            self._pose_msg = msg
            self.goal_received = True

    def joint_state_callback(self, msg: JointState):
        """Handle incoming joint state."""
        with self._state_lock:
            self.current_joint_state = msg
            self._last_joint_stamp = time.time()
            self.joints_received = True

    def _update_esdf(self):
        """Query nvblox in its global frame, computing AABB from current robot TF."""
        if time.time() - self.start_time < self.startup_delay:
            return
        if not self.__esdf_client.service_is_ready():
            self.get_logger().warn('ESDF service not ready, skipping.', skip_first=True)
            return
        if self._esdf_call_pending:
            return

        # Look up robot base position in nvblox's global frame
        try:
            tf_global_from_base = self.tf_buffer.lookup_transform(
                self._esdf_global_frame,
                self._esdf_query_frame,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.1),
            )
        except Exception as e:
            self._log_warn(
                f'TF {self._esdf_query_frame}→{self._esdf_global_frame} unavailable: {e}',
                event='esdf'
            )
            return

        t = tf_global_from_base.transform.translation
        q = tf_global_from_base.transform.rotation
        robot_pos = np.array([t.x, t.y, t.z])

        # Rotate grid_center_m offset by robot orientation, then add robot position
        q_arr = np.array([q.x, q.y, q.z, q.w])
        gc = np.array(self.__grid_center_m, dtype=float)
        gc_rotated = self._rotate_by_quat(gc, q_arr)
        grid_center_global = robot_pos + gc_rotated

        # Store robot_pos for ESDF origin transformation in _parse_esdf_response
        self._robot_pos_global = robot_pos.copy()

        min_corner = grid_center_global - np.array(self.__grid_size_m) / 2.0

        self._log_info(
            f'ESDF AABB in {self._esdf_global_frame}: '
            f'robot=({robot_pos[0]:.2f},{robot_pos[1]:.2f},{robot_pos[2]:.2f}) '
            f'min=({min_corner[0]:.2f},{min_corner[1]:.2f},{min_corner[2]:.2f})',
            event='esdf'
        )

        self.__esdf_req.update_esdf = True
        self.__esdf_req.visualize_esdf = False
        self.__esdf_req.use_aabb = True
        self.__esdf_req.frame_id = self._esdf_global_frame
        self.__esdf_req.aabb_min_m = Point(
            x=float(min_corner[0]), y=float(min_corner[1]), z=float(min_corner[2]))
        self.__esdf_req.aabb_size_m = Vector3(
            x=float(self.__grid_size_m[0]),
            y=float(self.__grid_size_m[1]),
            z=float(self.__grid_size_m[2]),
        )

        self._esdf_call_pending = True
        t0 = time.time()
        future = self.__esdf_client.call_async(self.__esdf_req)
        future.add_done_callback(lambda f: self._on_esdf_response(f, t0))

    @staticmethod
    def _rotate_by_quat(v: np.ndarray, q: np.ndarray) -> np.ndarray:
        """Rotate vector v by quaternion q (x,y,z,w)."""
        qx, qy, qz, qw = q
        # v' = v + 2*qw*(q_vec × v) + 2*(q_vec × (q_vec × v))
        u = np.array([qx, qy, qz])
        return v + 2.0 * qw * np.cross(u, v) + 2.0 * np.cross(u, np.cross(u, v))

    @staticmethod
    def _rotate_by_quat_inverse(v: np.ndarray, q: np.ndarray) -> np.ndarray:
        """Rotate vector v by inverse of quaternion q (x,y,z,w)."""
        qx, qy, qz, qw = q
        # Inverse rotation uses negative vector part (conjugate)
        u = np.array([-qx, -qy, -qz])
        return v + 2.0 * qw * np.cross(u, v) + 2.0 * np.cross(u, np.cross(u, v))

    def _on_esdf_response(self, future, t0: float):
        self._esdf_call_pending = False
        try:
            response = future.result()
        except Exception as e:
            self._log_error(f'ESDF service call failed: {e}')
            return
        elapsed_ms = (time.time() - t0) * 1000
        self._log_info(
            f'ESDF service responded in {elapsed_ms:.0f}ms | queried frame: {self._esdf_global_frame}',
            event='esdf'
        )

        if response is None or not response.success:
            try:
                frames = self.tf_buffer.all_frames_as_string()
                self._log_error(
                    f'ESDF service returned failure (queried frame_id={self._esdf_global_frame})\n'
                    f'TF tree:\n{frames}'
                )
            except Exception:
                self._log_error(f'ESDF service returned failure (queried frame_id={self._esdf_global_frame})')
            return

        esdf_grid = self._parse_esdf_response(response)
        if esdf_grid is None:
            return

        self.__world_collision.update_voxel_data(esdf_grid)
        if not self._esdf_initialized:
            self._esdf_initialized = True
            self._log_info('First ESDF map received — control loop released', event='esdf')
        self._log_info('ESDF voxel grid updated', event='esdf')

    def _parse_esdf_response(self, response) -> CuVoxelGrid:
        """Convert EsdfAndGradients response to a cuRobo CuVoxelGrid, or None on error."""
        if abs(response.voxel_size_m - self.__voxel_size) > 1e-4:
            self._log_error(f'ESDF voxel size mismatch: {response.voxel_size_m} vs {self.__voxel_size}')
            return None

        arr = response.esdf_and_gradients
        shape = [arr.layout.dim[i].size for i in range(3)]
        data = np.array(arr.data, dtype=np.float32)

        if data.shape[0] <= 0:
            self._log_error('ESDF response data is empty')
            return None
        if shape != self.__cumotion_grid_shape:
            self._log_error(f'ESDF grid shape mismatch: {shape} vs {self.__cumotion_grid_shape}')
            return None

        total_voxels = data.shape[0]
        obstacle_count = int(np.sum(data < 0.0))  # nvblox: negative = inside obstacle
        self._log_info(f'ESDF: {obstacle_count}/{total_voxels} occupied voxels', event='esdf')

        # Publish ESDF visualization
        self._publish_esdf_viz(data, shape, response.origin_m)

        data = torch.as_tensor(data).view(shape[0], shape[1], shape[2]).reshape(-1, 1)
        data[data < -999.9] = 1000.0   # unobserved → free
        data = -data                    # nvblox sign convention → cuRobo
        data += 0.5 * self.__voxel_size # surface offset

        # origin_m comes back in esdf_global_frame (odom/vision).
        # Transform it to esdf_query_frame (base/body) for cuRobo.
        origin_global = response.origin_m
        self._log_info(
            f'ESDF origin_m in {self._esdf_global_frame}: '
            f'x={origin_global.x:.3f} y={origin_global.y:.3f} z={origin_global.z:.3f}',
            event='esdf'
        )

        # Use stored robot_pos_global from _update_esdf instead of looking up inverse transform
        if self._robot_pos_global is None:
            self._log_error('robot_pos_global not set, ESDF transform failed')
            return None

        # Get rotation from vision to body (need to look this up for the quaternion)
        try:
            tf_base_from_global = self.tf_buffer.lookup_transform(
                self._esdf_query_frame,
                self._esdf_global_frame,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.1),
            )
            q_arr = np.array([
                tf_base_from_global.transform.rotation.x,
                tf_base_from_global.transform.rotation.y,
                tf_base_from_global.transform.rotation.z,
                tf_base_from_global.transform.rotation.w,
            ])
        except Exception as e:
            self._log_error(f'TF {self._esdf_global_frame}→{self._esdf_query_frame} failed: {e}')
            return None

        origin_vec = np.array([origin_global.x, origin_global.y, origin_global.z])
        # CORRECT transformation: p_base = R^T * (p_vision - robot_pos)
        origin_relative_vision = origin_vec - self._robot_pos_global
        origin_base = self._rotate_by_quat_inverse(origin_relative_vision, q_arr)

        self._log_info(
            f'ESDF origin_m in {self._esdf_query_frame}: '
            f'x={origin_base[0]:.3f} y={origin_base[1]:.3f} z={origin_base[2]:.3f}',
            event='esdf'
        )

        grid_center = [
            origin_base[0] + self.__grid_size_m[0] / 2.0,
            origin_base[1] + self.__grid_size_m[1] / 2.0,
            origin_base[2] + self.__grid_size_m[2] / 2.0,
        ]
        self._log_info(
            f'ESDF grid_center (cuRobo pose in {self._esdf_query_frame}): '
            f'x={grid_center[0]:.3f} y={grid_center[1]:.3f} z={grid_center[2]:.3f}',
            event='esdf'
        )

        return CuVoxelGrid(
            name='world_voxel',
            dims=self.__grid_size_m,
            pose=grid_center + [1, 0.0, 0.0, 0.0],  # x, y, z, qw, qx, qy, qz
            voxel_size=self.__voxel_size,
            feature_dtype=torch.float32,
            feature_tensor=data,
        )

    def _publish_esdf_viz(self, data, shape, origin):
        """Publish ESDF as colored PointCloud2 for RViz2 visualization.

        Color coding:
        - Red: Close to obstacle (distance < 0.2m)
        - Yellow: Medium distance (0.2m < distance < 0.5m)
        - Green: Far from obstacle (distance > 0.5m)
        """
        try:
            points = []
            colors = []

            # Downsample for visualization (every 2nd voxel)
            step = 2
            for i in range(0, shape[0], step):
                for j in range(0, shape[1], step):
                    for k in range(0, shape[2], step):
                        idx = i * shape[1] * shape[2] + j * shape[2] + k
                        distance = data[idx]

                        # Skip unobserved or very far voxels
                        if distance > 2.0 or distance < -10.0:
                            continue

                        x = origin.x + i * self.__voxel_size
                        y = origin.y + j * self.__voxel_size
                        z = origin.z + k * self.__voxel_size

                        points.append([x, y, z])

                        # Color based on distance
                        if distance < 0.2:
                            colors.append([255, 0, 0, 255])  # Red - close to obstacle
                        elif distance < 0.5:
                            colors.append([255, 255, 0, 255])  # Yellow
                        else:
                            colors.append([0, 255, 0, 255])  # Green

            if points:
                msg = PointCloud2()
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.header.frame_id = self._esdf_query_frame
                msg.height = 1
                msg.width = len(points)

                # Create PointField objects correctly
                from sensor_msgs.msg import PointField
                msg.fields = [
                    PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
                    PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
                    PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
                    PointField(name='r', offset=12, datatype=PointField.UINT8, count=1),
                    PointField(name='g', offset=13, datatype=PointField.UINT8, count=1),
                    PointField(name='b', offset=14, datatype=PointField.UINT8, count=1),
                    PointField(name='a', offset=15, datatype=PointField.UINT8, count=1),
                ]
                msg.is_bigendian = False
                msg.point_step = 16
                msg.row_step = msg.point_step * msg.width

                # Create point cloud data
                import struct
                point_data = []
                for (x, y, z), (r, g, b, a) in zip(points, colors):
                    point_data.append(struct.pack('fffBBBB', x, y, z, r, g, b, a))

                msg.data = b''.join(point_data)
                self.esdf_pub.publish(msg)

        except Exception as e:
            self._log_warn(f'Failed to publish ESDF visualization: {e}', event='esdf')



    def compute_attractive(self, current: np.ndarray, target: np.ndarray, k_att: float = 0.1) -> np.ndarray:
        return k_att * (target - current)

    def _get_gpu_stats(self):
        """Get GPU utilization and memory stats."""
        gpu_util = -1
        gpu_mem_used = 0
        gpu_mem_total = 0
        gpu_temp = -1
        
        if self._gpu_monitor_available:
            try:
                import pynvml
                util = pynvml.nvmlDeviceGetUtilizationRates(self._gpu_handle)
                mem = pynvml.nvmlDeviceGetMemoryInfo(self._gpu_handle)
                temp = pynvml.nvmlDeviceGetTemperature(self._gpu_handle, pynvml.NVML_TEMPERATURE_GPU)
                gpu_util = util.gpu
                gpu_mem_used = mem.used // (1024 * 1024)  # MB
                gpu_mem_total = mem.total // (1024 * 1024)  # MB
                gpu_temp = temp
            except Exception:
                pass
        
        # Also get torch CUDA memory
        torch_alloc = torch.cuda.memory_allocated() // (1024 * 1024)  # MB
        torch_reserved = torch.cuda.memory_reserved() // (1024 * 1024)  # MB
        
        return gpu_util, gpu_mem_used, gpu_mem_total, gpu_temp, torch_alloc, torch_reserved

    def _get_sphere_distances(self) -> list:
        """Returns distance from each collision sphere to nearest obstacle."""
        # Access world collision checker inside MPC
        world_coll = None
        if hasattr(self.mpc.rollout_fn, 'world_coll_checker'):
            world_coll = self.mpc.rollout_fn.world_coll_checker

        # Fallback search if path varies
        if world_coll is None:
            # self._log_debug('Searching for world_coll_checker...')
            for attr in dir(self.mpc.rollout_fn):
                if 'prim' in attr.lower():
                    prim = getattr(self.mpc.rollout_fn, attr)
                    if hasattr(prim, 'world_coll_checker'):
                        world_coll = prim.world_coll_checker
                        break

        if world_coll is None:
            if not hasattr(self, '_coll_checker_none_logged'):
                self._log_error("ERROR: world_coll_checker is None! Collision detection not working!", event='collision')
                self._coll_checker_none_logged = True
            return []

        # Log collision checker type for debugging
        if not hasattr(self, '_coll_type_logged'):
            coll_types = getattr(world_coll, 'collision_types', {})
            self._log_info(f"Collision checker types: {coll_types}", event='collision')
            self._coll_type_logged = True

        # FK to get current sphere positions
        # rollout_fn.kinematics is CudaRobotModel which returns CudaRobotModelState with spheres
        pos = self.current_state.position.view(-1)  # [dof]
        if len(pos.shape) == 1:
            pos = pos.unsqueeze(0)  # [1, dof]

        # Using kinematics directly
        kin_state = self.mpc.rollout_fn.kinematics.get_state(pos)

        # Sphere positions: (batch, n_spheres, 4) -> [x, y, z, radius]
        spheres = kin_state.link_spheres_tensor

        if spheres is None:
             if not hasattr(self, '_sphere_none_logged'):
                 self._log_error("sphere_dist error: link_spheres_tensor is None")
                 self._sphere_none_logged = True
             return []

        # Log sphere positions for debugging (first few steps only)
        if not hasattr(self, '_sphere_positions_logged') and self.step_count % 100 == 0:
            sphere_positions = spheres[0, :, :3].cpu().numpy()  # First batch, all spheres, xyz
            self._log_info(f"Sphere positions (first 3): {sphere_positions[:3]}", event='collision')
            self._sphere_positions_logged = True

        # CollisionQueryBuffer expects [batch, horizon, n_spheres, 4]
        # Current shape is [batch, n_spheres, 4] (batch=1)
        if len(spheres.shape) == 3:
            spheres = spheres.unsqueeze(1)

        # Query buffer (lazy init)
        if not hasattr(self, '_query_buffer') or self._query_buffer is None:
            # Increase max_distance so ESDF reports distances beyond 10cm (default 0.1)
            world_coll.max_distance = self.tensor_args.to_device([0.5])
            try:
                # Initialize buffer for ALL types (voxel, mesh, primitive) to prevent fallbacks crashing
                self._query_buffer = CollisionQueryBuffer.initialize_from_shape(
                    spheres.shape, self.tensor_args,
                    collision_types={'voxel': True, 'mesh': True, 'primitive': True}
                )
            except Exception as e:
                self._log_error(f"CollisionQueryBuffer init error: {e}")
                return []

        # MANUAL FALLBACK: Ensure buffers exist if init failed to create them
        from curobo.geom.sdf.world import CollisionBuffer
        if self._query_buffer.voxel_collision_buffer is None:
             self._query_buffer.voxel_collision_buffer = CollisionBuffer.initialize_from_shape(
                 spheres.shape, self.tensor_args
             )
             self._query_buffer.voxel_collision_buffer.sparsity_index_buffer[:] = 0
             
        if self._query_buffer.mesh_collision_buffer is None:
             self._query_buffer.mesh_collision_buffer = CollisionBuffer.initialize_from_shape(
                 spheres.shape, self.tensor_args
             )

        if self._query_buffer.primitive_collision_buffer is None:
             self._query_buffer.primitive_collision_buffer = CollisionBuffer.initialize_from_shape(
                 spheres.shape, self.tensor_args
             )

        # act_distance = search radius (returns 0 outside range)
        act_dist = 0.5  # 50cm
        # Activation distance for Mesh Collision (Warp) expects 1D tensor [n_spheres]
        act_distance = torch.full(
            (spheres.shape[2],), 
            act_dist, 
            device=self.tensor_args.device, 
            dtype=self.tensor_args.dtype
        )
        
        # Weight for Mesh Collision (Warp) expects 1D tensor [n_spheres]
        weight = torch.ones(
            (spheres.shape[2],), 
            device=self.tensor_args.device, 
            dtype=self.tensor_args.dtype
        )

        try:
            # Reset distance buffer before each query to avoid stale values
            self._query_buffer.mesh_collision_buffer.distance_buffer.zero_()
            if self._query_buffer.primitive_collision_buffer is not None:
                self._query_buffer.primitive_collision_buffer.distance_buffer.zero_()

            # compute_esdf=True -> returns real distance (not just collision bool)
            # Args order: spheres, buffer, weight, activation_distance
            dist = world_coll.get_sphere_distance(
                spheres, self._query_buffer, weight, act_distance, compute_esdf=True
            )
            
            result_list = dist.squeeze().cpu().tolist()
            
            # If dist is scalar (0-d), tolist returns float, not list
            if not isinstance(result_list, list):
                result_list = [result_list]
                
            return result_list
        except Exception as e:
            if not hasattr(self, '_sphere_dist_calc_logged'):
                self._log_error(f"get_sphere_distance error: {e}")
                import traceback
                self._log_error(traceback.format_exc())
                self._sphere_dist_calc_logged = True
            return []

    def control_loop(self):
        """Main MPC control loop."""
        t_loop_start = time.time()
        loop_gap = t_loop_start - self._last_loop_time  # time since last call
        self._last_loop_time = t_loop_start

        # Snapshot shared sensor state — brief lock, no GPU work inside
        with self._state_lock:
            _js_msg = self.current_joint_state
            _pose_msg = self._pose_msg

        # Apply latest pose goal (tensor conversion stays in control thread, not callback)
        if not self.debug_mode and _pose_msg is not None:
            pos = _pose_msg.pose.position
            ori = _pose_msg.pose.orientation
            position = self.tensor_args.to_device([pos.x, pos.y, pos.z])
            quaternion = self.tensor_args.to_device([ori.w, ori.x, ori.y, ori.z])
            if self.last_goal_pose is not None:
                self.last_goal_pose.position.copy_(position)
                self.last_goal_pose.quaternion.copy_(quaternion)
            else:
                self.last_goal_pose = Pose(position=position, quaternion=quaternion)
            self._last_wrist_target = (pos.x, pos.y, pos.z)

        # In debug mode, we don't need external goal
        if self.debug_mode:
            self.last_goal_pose = self._get_debug_pose()
            # In debug mode, if no joints received, use retract config
            if not self.joints_received:
                if _js_msg is None:
                    fake_js = JointState()
                    fake_js.name = list(self.j_names)
                    fake_js.position = list(self.default_config)
                    fake_js.velocity = [0.0] * len(self.j_names)
                    with self._state_lock:
                        self.current_joint_state = fake_js
                    _js_msg = fake_js
                    self.joints_received = True
        else:
            if not self.goal_received and (time.time() - self.start_time >= self.startup_delay):
                # Before external goals arrive, maintain current EE pose to prevent jerking
                if self.joints_received and _js_msg is not None:
                    # Initialize goal_pose to current FK pose exactly once
                    if getattr(self, '_init_goal_set', False) is False:
                        positions = list(_js_msg.position)
                        joint_names_msg = list(_js_msg.name)
                        cu_js_init = CuJointState(
                            position=self.tensor_args.to_device(positions),
                            joint_names=joint_names_msg
                        ).get_ordered_joint_state(self.mpc.rollout_fn.joint_names)
                        
                        fk_init = self.mpc.rollout_fn.compute_kinematics(cu_js_init)
                        self.last_goal_pose = Pose(fk_init.ee_pos_seq, quaternion=fk_init.ee_quat_seq)
                        
                        # === WARM-UP MPC with REAL joint state ===
                        # The MPC was initialized and warmed up with retract_config in __init__,
                        # so its internal particles/rollouts are optimized around a completely
                        # different joint configuration. We must re-warm the solver around the
                        # actual physical joint state to prevent the first commands from jumping
                        # to a different IK solution (which causes a dangerous jerk).
                        self._log_info(
                            f'Warm-up MPC with real joint state: '
                            f'[{", ".join(f"{p:.3f}" for p in cu_js_init.position.view(-1).cpu().numpy())}]'
                        )
                        self.current_state.copy_(cu_js_init)
                        self.goal_buffer.goal_pose.copy_(self.last_goal_pose)
                        self.mpc.update_goal(self.goal_buffer)
                        
                        WARMUP_ITERS = 10
                        for i in range(WARMUP_ITERS):
                            warmup_result = self.mpc.step(self.current_state, max_attempts=2)
                            # Feed MPC output back as current state so particles converge
                            warmup_ordered = warmup_result.js_action.get_ordered_joint_state(
                                self.mpc.rollout_fn.joint_names
                            )
                            # But keep overriding with real state to anchor around it
                            self.current_state.copy_(cu_js_init)
                        
                        # Log the first command after warm-up to verify convergence
                        warmup_cmd = warmup_ordered.position.view(-1).cpu().numpy()
                        real_pos = cu_js_init.position.view(-1).cpu().numpy()
                        max_diff = max(abs(warmup_cmd - real_pos))
                        self._log_info(
                            f'Warm-up done ({WARMUP_ITERS} iters). Max joint diff: {max_diff:.4f} rad\n'
                            f'  Real:    [{", ".join(f"{p:.3f}" for p in real_pos)}]\n'
                            f'  MPC cmd: [{", ".join(f"{p:.3f}" for p in warmup_cmd)}]'
                        )
                        
                        self.goal_received = True
                        self._init_goal_set = True
                        self._log_info('Initialized goal to current physical pose to prevent jumping.')
                return
            if not self.joints_received or _js_msg is None or self.last_goal_pose is None:
                return
            if not self._esdf_initialized:
                now = time.time()
                if now - self._last_esdf_wait_log >= 2.0:
                    self._last_esdf_wait_log = now
                    self._log_info('Waiting for first ESDF map...', event='esdf')
                return

        # Guard: skip MPC if joint state is too stale (Isaac Sim stopped publishing)
        JS_STALE_THRESHOLD = 2.0  # seconds
        if self._last_joint_stamp is not None:
            js_age_now = time.time() - self._last_joint_stamp
            if js_age_now > JS_STALE_THRESHOLD:
                if self.step_count % 30 == 0:
                    self._log_warn(
                        f'⚠️ Joint state is STALE ({js_age_now*1000:.0f}ms)! '
                        f'Isaac Sim may have stopped publishing. Skipping MPC step.'
                    )
                return

        try:
            positions = list(_js_msg.position)
            velocities = list(_js_msg.velocity) if _js_msg.velocity else [0.0] * len(positions)
            joint_names_msg = list(_js_msg.name)

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

            self.goal_buffer.goal_pose.copy_(self.last_goal_pose)
            
            # --- Attractive Potential Field Logic ---
            try:
                # 1. Get current EE position in robot frame (body)
                # We use the kinematics from the current state (latest joint sensors)
                fk_result = self.mpc.rollout_fn.compute_kinematics(self.current_state)
                # fk_result.ee_pos_seq shape is (batch, horizon, 3) -> usually (1, 1, 3) here
                current_ee_pos_tensor = fk_result.ee_pos_seq
                current_ee_pos_np = current_ee_pos_tensor.cpu().numpy().flatten()[:3]
                
                # 2. Get Target Object in 'body' frame
                # We need to transform from 'target_object' frame to 'body' frame
                # 'target_object' is published by detect_qwen.py
                target_tf = self.tf_buffer.lookup_transform(
                    self.target_frame, # 'body'
                    'target_object',
                    rclpy.time.Time()
                )
                
                target_pos_np = np.array([
                    target_tf.transform.translation.x,
                    target_tf.transform.translation.y,
                    target_tf.transform.translation.z
                ])
                self._last_target_object = (
                    float(target_pos_np[0]),
                    float(target_pos_np[1]),
                    float(target_pos_np[2]),
                )
                
                # 3. Compute Distance
                dist = np.linalg.norm(target_pos_np - current_ee_pos_np)
                self._last_target_dist = float(dist)
                
                # 4. Apply Attraction if close
                if dist < 0.40:
                    delta = self.compute_attractive(current_ee_pos_np, target_pos_np, k_att=0.2)
                    new_pos_np = current_ee_pos_np + delta
                    
                    # Update the goal position (keep orientation from last_goal_pose)
                    new_pos_tensor = self.tensor_args.to_device(new_pos_np)
                    self.goal_buffer.goal_pose.position.copy_(new_pos_tensor)
                    
                    if self.step_count % 30 == 0: # Log occasionally
                        self._log_info(
                            f'Attractive field active. Dist={dist:.3f}m',
                            event='target_object',
                            step=self.step_count,
                            target_object_x=self._last_target_object[0],
                            target_object_y=self._last_target_object[1],
                            target_object_z=self._last_target_object[2],
                            target_distance=self._last_target_dist,
                            wrist_target_x=self._last_wrist_target[0],
                            wrist_target_y=self._last_wrist_target[1],
                            wrist_target_z=self._last_wrist_target[2],
                        )
                elif self.step_count % 30 == 0:
                     self._log_info(
                         f'Target detected. Dist={dist:.3f}m',
                         event='target_object',
                         step=self.step_count,
                         target_object_x=self._last_target_object[0],
                         target_object_y=self._last_target_object[1],
                         target_object_z=self._last_target_object[2],
                         target_distance=self._last_target_dist,
                         wrist_target_x=self._last_wrist_target[0],
                         wrist_target_y=self._last_wrist_target[1],
                         wrist_target_z=self._last_wrist_target[2],
                     )

            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
                # Target not detected or TF not ready
                if self.step_count % 100 == 0:
                    self._log_info(
                        f'Target TF not found: {e}',
                        event='target_object',
                        step=self.step_count,
                        target_object_x=self._last_target_object[0],
                        target_object_y=self._last_target_object[1],
                        target_object_z=self._last_target_object[2],
                        target_distance=self._last_target_dist,
                    )
                pass
            except Exception as e:
                self._log_warn(f'Potential field error: {e}')
            
            # ----------------------------------------

            self.mpc.update_goal(self.goal_buffer)

            t_step_start = time.time()
            mpc_result = self.mpc.step(self.current_state, max_attempts=2)
            torch.cuda.synchronize()  # Ensure GPU work is done before timing
            t_step_end = time.time()
            step_dt = t_step_end - t_step_start
            self._step_times.append(step_dt)
            
            # Get collision/feasibility info
            is_feasible = mpc_result.metrics.feasible.item()
            coll_cost = 0.0
            coll_constraint = 0.0
            if mpc_result.metrics.cost is not None:
                coll_cost = mpc_result.metrics.cost.item()
            if mpc_result.metrics.constraint is not None:
                coll_constraint = mpc_result.metrics.constraint.item()
            
            self.step_count += 1
            self._publish_count += 1
            
            # --- Deviation Cost Logic ---
            evasion_dev = 0.0
            try:
                # We need the current EE pos
                fk_current = self.mpc.rollout_fn.compute_kinematics(self.current_state)
                current_ee_pos_tensor = fk_current.ee_pos_seq
                current_ee_pos_np = current_ee_pos_tensor.cpu().numpy().flatten()[:3]
                
                # And the goal pos
                goal_pos_tensor = self.goal_buffer.goal_pose.position
                goal_pos_np = goal_pos_tensor.cpu().numpy().flatten()[:3]
                
                # If goal shifted by more than 5cm, or no anchor exists, reset the straight-line anchor
                if not hasattr(self, 'anchor_goal_np') or np.linalg.norm(goal_pos_np - self.anchor_goal_np) > 0.05:
                    self.anchor_goal_np = goal_pos_np.copy()
                    self.anchor_start_np = current_ee_pos_np.copy()
                
                # The ideal Euclidean path is from anchor_start to anchor_goal
                line_vec = self.anchor_goal_np - self.anchor_start_np
                line_len = np.linalg.norm(line_vec)
                
                if line_len > 1e-3:
                    line_dir = line_vec / line_len
                    # Vector from start to current
                    point_vec = current_ee_pos_np - self.anchor_start_np
                    # Project current position onto the ideal line
                    proj = np.dot(point_vec, line_dir)
                    closest_pt = self.anchor_start_np + max(0.0, min(line_len, proj)) * line_dir
                    
                    # Deflection distance from the ideal straight line
                    evasion_dev = np.linalg.norm(current_ee_pos_np - closest_pt)
            except Exception as e:
                pass
            # ---------------------------

            # Always use MPC result - it will slide along obstacle surfaces
            self.cmd_state_full = mpc_result.js_action

            # Enforce using ONLY the configured controlled joints (e.g. 6 arm joints)
            # This explicitly filters out locked joints (like arm_f1x) from the command
            ordered_names = list(self.j_names)

            cmd_state = self.cmd_state_full.get_ordered_joint_state(ordered_names)

            pos_list = cmd_state.position.view(-1).cpu().numpy().tolist()
            vel_list = cmd_state.velocity.view(-1).cpu().numpy().tolist()
            # acc_list = cmd_state.acceleration.view(-1).cpu().numpy().tolist()

            if self.use_ros2_control and not self.use_sim:
                # Publish JointCommand for spot_joint_controller
                joint_cmd = JointCommand()
                joint_cmd.name = ordered_names
                joint_cmd.position = pos_list
                joint_cmd.velocity = vel_list
                # Leave effort, k_q_p, k_qd_p empty to use SDK default gains
            else:
                # Publish JointState for sim or legacy real robot path
                joint_cmd = JointState()
                joint_cmd.header.stamp = self.get_clock().now().to_msg()
                if self.use_sim:
                    joint_cmd.name = [n.replace('arm_', 'arm0_') for n in ordered_names]
                else:
                    joint_cmd.name = ordered_names
                joint_cmd.position = pos_list
                joint_cmd.velocity = vel_list
                joint_cmd.effort = []

            self.cmd_pub.publish(joint_cmd)
            
            t_loop_end = time.time()
            total_loop_dt = t_loop_end - t_loop_start
            self._loop_times.append(total_loop_dt)

            # Compute joint state age
            js_age = -1.0
            if self._last_joint_stamp is not None:
                js_age = t_loop_start - self._last_joint_stamp

            # Keep only last 50 samples
            if len(self._loop_times) > 50:
                self._loop_times = self._loop_times[-50:]
            if len(self._step_times) > 50:
                self._step_times = self._step_times[-50:]

            # DETAILED LOGGING every 10 steps
            if self.step_count % 10 == 0:
                avg_loop = sum(self._loop_times) / len(self._loop_times)
                avg_step = sum(self._step_times) / len(self._step_times)
                effective_hz = 1.0 / avg_loop if avg_loop > 0 else 0
                
                gpu_util, gpu_mem_used, gpu_mem_total, gpu_temp, torch_alloc, torch_reserved = self._get_gpu_stats()
                
                # Get sphere distances
                sphere_dists = self._get_sphere_distances()
                # cuRobo ESDF: positive = inside obstacle, negative = outside
                # max() gives the sphere CLOSEST to (or inside) an obstacle  
                min_dist = max(sphere_dists) if sphere_dists else -1.0
                closest_sphere = sphere_dists.index(max(sphere_dists)) if sphere_dists else -1
                
                # Get current joint positions for logging
                curr_js_ordered = cu_js.get_ordered_joint_state(ordered_names)
                curr_pos_list = curr_js_ordered.position.view(-1).cpu().numpy().tolist()
                
                status = "BLOCKED" if not is_feasible else "OK"
                step_msg = f'Step {self.step_count} status={status}'
                self._log_info(
                    step_msg,
                    event='control_step',
                    step=self.step_count,
                    wrist_target_x=self._last_wrist_target[0],
                    wrist_target_y=self._last_wrist_target[1],
                    wrist_target_z=self._last_wrist_target[2],
                    target_object_x=self._last_target_object[0],
                    target_object_y=self._last_target_object[1],
                    target_object_z=self._last_target_object[2],
                    target_distance=self._last_target_dist,
                    pose_error=float(mpc_result.metrics.pose_error.item()),
                    constraint=coll_constraint,
                    coll_cost=coll_cost,
                    evasion_dev=evasion_dev,
                    loop_ms=total_loop_dt * 1000.0,
                    mpc_step_ms=step_dt * 1000.0,
                    gap_ms=loop_gap * 1000.0,
                    avg_loop_ms=avg_loop * 1000.0,
                    avg_step_ms=avg_step * 1000.0,
                    effective_hz=effective_hz,
                    gpu_util=gpu_util,
                    gpu_mem_used_mb=gpu_mem_used,
                    gpu_mem_total_mb=gpu_mem_total,
                    gpu_temp_c=gpu_temp,
                    torch_alloc_mb=torch_alloc,
                    torch_reserved_mb=torch_reserved,
                    js_age_ms=js_age * 1000.0,
                    publish_count=self._publish_count,
                    min_dist=min_dist,
                    closest_sphere=closest_sphere,
                    cmd_pos='|'.join(f'{v:.6f}' for v in pos_list),
                    curr_pos='|'.join(f'{v:.6f}' for v in curr_pos_list),
                )
                self.get_logger().info(step_msg)

        except Exception as e:
            import traceback
            self._log_error(f'Control loop error: {e}\n{traceback.format_exc()}')


def main(args=None):
    rclpy.init(args=args)

    if not torch.cuda.is_available():
        stamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        csv_path = os.path.join(os.getcwd(), f'curobo_mpc_log_{stamp}.csv')
        with open(csv_path, 'w', newline='', encoding='utf-8') as fh:
            writer = csv.DictWriter(fh, fieldnames=['timestamp', 'level', 'event', 'message'])
            writer.writeheader()
            writer.writerow({
                'timestamp': datetime.now().isoformat(timespec='milliseconds'),
                'level': 'ERROR',
                'event': 'startup',
                'message': 'CUDA not available! cuRobo requires CUDA.',
            })
        return

    node = CuroboMpcNode()

    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        if hasattr(node, '_csv_fh') and node._csv_fh is not None:
            node._csv_fh.close()
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
