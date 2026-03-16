import os
import tempfile
import yaml
import math
import random
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from geometry_msgs.msg import PoseStamped, TransformStamped, Quaternion
from sensor_msgs.msg import JointState, PointCloud2
import struct
import threading

# TF2 for coordinate transformations
import tf2_ros
from tf2_ros import Buffer, TransformListener
from tf2_geometry_msgs import do_transform_point

import torch
import numpy as np
import time
import sys

# Hack to find nvblox_msgs if running in venv
# Assuming workspace is /home/spot-teleop/spot-ros2_ws
NVBLOX_MSGS_PATH = '/home/spot-teleop/spot-ros2_ws/install/nvblox_msgs/local/lib/python3.10/dist-packages'
if NVBLOX_MSGS_PATH not in sys.path:
    sys.path.append(NVBLOX_MSGS_PATH)

# cuRobo imports
from curobo.geom.sdf.world import CollisionCheckerType, CollisionQueryBuffer
from curobo.geom.types import WorldConfig, Mesh
from curobo.rollout.rollout_base import Goal
from curobo.types.base import TensorDeviceType
from curobo.types.math import Pose
from curobo.types.state import JointState as CuJointState
from curobo.util.logger import setup_curobo_logger
from curobo.util_file import load_yaml, get_robot_configs_path
from curobo.wrap.reacher.mpc import MpcSolver, MpcSolverConfig

# nvblox msgs
try:
    from nvblox_msgs.msg import Mesh as NvbloxMesh
except ImportError:
    # Try one more path common in colcon builds
    NVBLOX_MSGS_PATH_2 = '/home/spot-teleop/spot-ros2_ws/install/nvblox_msgs/lib/python3.10/site-packages'
    if NVBLOX_MSGS_PATH_2 not in sys.path:
        sys.path.append(NVBLOX_MSGS_PATH_2)
    from nvblox_msgs.msg import Mesh as NvbloxMesh

# spot_msgs for spot_joint_controller integration
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

        self.declare_parameter('debug_mode', False)
        self.declare_parameter('debug_pose_duration', 3.0)  # seconds between pose changes
        self.declare_parameter('use_sim', True)  # True=sim (arm0_ prefix), False=real (arm_ prefix)
        self.declare_parameter('use_ros2_control', False)  # True=publish JointCommand to spot_joint_controller
        self.declare_parameter('startup_delay', 5.0)  # Wait N seconds before considering obstacles
        
        # nvblox Mesh parameters
        self.declare_parameter('use_nvblox', False)  # If True, subscribe to mesh
        self.declare_parameter('mesh_topic', '/nvblox_node/mesh')
        self.declare_parameter('mesh_update_rate', 2.0)  # Hz (limit updates to curobo)
        self.declare_parameter('static_mesh', True)  # If True, update mesh only once

        # Get parameters
        robot_config_path = self.get_parameter('robot_config').get_parameter_value().string_value
        urdf_path = self.get_parameter('urdf_path').get_parameter_value().string_value
        self.control_rate = self.get_parameter('control_rate').get_parameter_value().double_value
        self.step_dt = self.get_parameter('step_dt').get_parameter_value().double_value

        self.debug_mode = self.get_parameter('debug_mode').get_parameter_value().bool_value
        self.debug_pose_duration = self.get_parameter('debug_pose_duration').get_parameter_value().double_value
        self.use_sim = self.get_parameter('use_sim').get_parameter_value().bool_value
        self.use_ros2_control = self.get_parameter('use_ros2_control').get_parameter_value().bool_value
        self.startup_delay = self.get_parameter('startup_delay').get_parameter_value().double_value
        
        # nvblox parameters
        self.use_nvblox = self.get_parameter('use_nvblox').get_parameter_value().bool_value
        self.mesh_topic = self.get_parameter('mesh_topic').get_parameter_value().string_value
        self.mesh_update_rate = self.get_parameter('mesh_update_rate').get_parameter_value().double_value
        self.static_mesh = self.get_parameter('static_mesh').get_parameter_value().bool_value

        # ... (Keeping default paths logic)
        os.environ['SPOT_URDF_PATH'] = os.path.dirname(urdf_path)

        self.get_logger().info('=== cuRobo MPC Node Starting ===')
        self.get_logger().info(f'Robot config: {robot_config_path}')
        self.get_logger().info(f'URDF path: {urdf_path}')
        self.get_logger().info(f'Control rate: {self.control_rate} Hz')
        self.get_logger().info(f'Use sim: {self.use_sim} ({"arm0_ prefix" if self.use_sim else "arm_ prefix"})')
        self.get_logger().info(f'Use sim: {self.use_sim} ({"arm0_ prefix" if self.use_sim else "arm_ prefix"})')
        self.get_logger().info(f'Use nvblox: {self.use_nvblox}')
        if self.use_nvblox:
            self.get_logger().info(f'Mesh topic: {self.mesh_topic}')
            self.get_logger().info(f'Static mesh: {self.static_mesh}')

        # Initialize cuRobo
        setup_curobo_logger("warn")
        self.tensor_args = TensorDeviceType()

        # Load robot config - need to resolve env vars in paths
        robot_cfg_raw = load_yaml(robot_config_path)["robot_cfg"]
        
        # Update paths in config
        robot_cfg_raw["kinematics"]["external_asset_path"] = os.path.dirname(urdf_path)
        spheres_path = os.path.join(os.environ['CUROBO_CONFIG_PATH'], 'spheres', 'spot_arm.yml')

        # Config and spheres use arm0_ (sim naming). URDF always uses arm_.
        # Remap config arm0_ -> arm_ to match URDF, and create temp spheres file.
        robot_cfg_raw = self._remap_config_prefix(robot_cfg_raw, 'arm0_', 'arm_')
        spheres_data = load_yaml(spheres_path)
        spheres_data = self._remap_config_prefix(spheres_data, 'arm0_', 'arm_')
        tmp_spheres = tempfile.NamedTemporaryFile(mode='w', suffix='.yml', delete=False)
        yaml.dump(spheres_data, tmp_spheres)
        tmp_spheres.close()
        robot_cfg_raw["kinematics"]["collision_spheres"] = tmp_spheres.name
        self.get_logger().info(f'Remapped config arm0_ -> arm_ (URDF match). Temp spheres: {tmp_spheres.name}')
        
        self.robot_cfg = robot_cfg_raw
        self.j_names = self.robot_cfg["kinematics"]["cspace"]["joint_names"]
        self.default_config = self.robot_cfg["kinematics"]["cspace"]["retract_config"]
        self.robot_cfg["kinematics"]["collision_sphere_buffer"] += 0.02

        self.get_logger().info(f'Joint names (URDF): {self.j_names}')



        # World config (empty for now)
        world_cfg = WorldConfig()
        self.obstacle_update_count = 0


        # MPC Configuration
        # NOTE: RTX A2000 12GB has 70W TDP limit -> thermal throttling at ~80C.
        # Reduce GPU load: smaller collision cache, fewer seeds.
        self.get_logger().info('Loading MPC solver...')
        mpc_config = MpcSolverConfig.load_from_robot_config(
            self.robot_cfg,
            world_cfg,
            use_cuda_graph=True,
            use_cuda_graph_metrics=True,
            use_cuda_graph_full_step=False,
            self_collision_check=True,
            collision_checker_type=CollisionCheckerType.VOXEL,  # Switch to Voxel (GPU ESDF) for performance
            collision_cache={"obb": 10, "mesh": 10}, # Voxelization needs obb+mesh cache
            override_particle_file=os.path.join(os.path.dirname(robot_config_path), 'mpc_override.yml'),
            use_mppi=True,
            use_lbfgs=False,
            use_es=False,
            store_rollouts=False,
            step_dt=self.step_dt,
        )

        self.mpc = MpcSolver(mpc_config)
        self.get_logger().info('MPC solver loaded!')

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

        # Initialize goal pose to use current robot position initially
        self.last_goal_pose = retract_pose

        # State variables
        self.last_goal_pose = None
        self.current_joint_state = None
        self.cmd_state_full = None
        self.goal_received = False
        self.joints_received = False

        # nvblox Mesh state
        self.mesh_blocks = {}  # {block_index (tuple): (vertices, triangles, block_size)}
        self.mesh_lock = threading.Lock()
        self.nvblox_initialized = False
        self.last_mesh_update_time = 0
        
        # TF2 for frame transformations
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.target_frame = 'body'  # cuRobo solve frame
        self.source_frame = 'odom' # Assumed nvblox frame, updated from msg

        # Debug mode state
        self.debug_pose_index = 0
        self.debug_last_pose_time = self.get_clock().now()
        self.debug_test_poses = self._generate_test_poses()
        
        # Mesh settings
        self.MAX_MESH_VERTICES = 15000


        # QoS for real-time sensors (joint states, pose)
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=1
        )

        # QoS for nvblox mesh - MUST be RELIABLE to match nvblox publisher
        # nvblox publishes with RELIABLE+VOLATILE; BEST_EFFORT subscriber won't connect!
        qos_mesh = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
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
                self.get_logger().fatal('use_ros2_control=True but spot_msgs.msg.JointCommand not found!')
                raise ImportError('spot_msgs.msg.JointCommand not available')
            self.cmd_pub = self.create_publisher(JointCommand, cmd_topic, 10)
        else:
            self.cmd_pub = self.create_publisher(JointState, cmd_topic, 10)
        
        # Subscribe to sensors (fast updates)
        self.pose_sub = self.create_subscription(
            PoseStamped, 
            '/wrist_pose', 
            self.pose_callback, 
            qos
        )
        self.joint_sub = self.create_subscription(
            JointState, 
            joint_topic, 
            self.joint_state_callback, 
            qos
        )
        
        # nvblox Mesh subscriber - RELIABLE QoS to match nvblox publisher
        if self.use_nvblox:
            self.mesh_sub = self.create_subscription(
                NvbloxMesh, 
                self.mesh_topic, 
                self.mesh_callback, 
                qos_mesh
            )
            self.get_logger().info(f'Subscribed to Mesh topic: {self.mesh_topic} (separate thread)')
        else:
            self.mesh_sub = None
            self.get_logger().info('Nvblox disabled (use_nvblox=False). Mesh updates OFF.')
        self.get_logger().info(f'Subscribed to Joint topic: {joint_topic}')
        self.get_logger().info(f'Publishing to Command topic: {cmd_topic}')

        # Latest joint states used for tracking age
        self.latest_js_age = 0.0

        self.timer = self.create_timer(1.0 / self.control_rate, self.control_loop)
        self.step_count = 0
        self.start_time = time.time()
        
        # Mesh update is done INLINE in control_loop (cuRobo is NOT thread-safe)
        self.mesh_update_interval = 1.0 / self.mesh_update_rate  # seconds between updates
        self.last_mesh_update_time = time.time()
        self.pending_mesh_update = False  # flag set by mesh_callback

        # Performance monitoring
        self._last_loop_time = time.time()
        self._loop_times = []  # track last N loop durations
        self._step_times = []  # track mpc.step() durations
        self._publish_count = 0
        self._joint_state_age = 0.0  # how old is the joint state data
        self._last_joint_stamp = None

        # CSV auto-save
        import csv as _csv, datetime as _dt
        _ts = _dt.datetime.now().strftime('%Y%m%d_%H%M%S')
        self._csv_path = os.path.join(
            os.path.expanduser('~'), f'curobo_run_{_ts}.csv')
        self._csv_file = open(self._csv_path, 'w', newline='')
        self._csv_writer = _csv.writer(self._csv_file)
        self._csv_writer.writerow([
            'step', 'status', 'loop_time_ms', 'mpc_step_ms',
            'gpu_util', 'js_age_ms', 'min_dist',
            'mpc_error', 'mpc_constraint', 'coll_cost', 'evasion_dev',
            'joint_tracking_err',
        ])
        self.get_logger().info(f'CSV logging -> {self._csv_path}')

        # GPU monitoring via pynvml - use same device as torch
        self._gpu_monitor_available = False
        try:
            import pynvml
            pynvml.nvmlInit()
            gpu_idx = torch.cuda.current_device()
            self._gpu_handle = pynvml.nvmlDeviceGetHandleByIndex(gpu_idx)
            gpu_name = pynvml.nvmlDeviceGetName(self._gpu_handle)
            self._gpu_monitor_available = True
            self.get_logger().info(f'GPU monitoring enabled: GPU {gpu_idx} ({gpu_name})')
        except Exception as e:
            self.get_logger().warn(f'pynvml not available, using torch.cuda only: {e}')

        self.get_logger().info('=== cuRobo MPC Node Ready ===')
        if self.debug_mode:
            self.get_logger().info('DEBUG MODE ACTIVE - Using test poses')
            self.get_logger().info(f'Will cycle through {len(self.debug_test_poses)} test positions')
            self.goal_received = True  # Auto-ready in debug mode
        else:
            # Get joint topic name for the log message
            joint_topic = '/joint_states_isaac' if self.use_sim else '/joint_states'
            self.get_logger().info(f'Waiting for /wrist_pose and {joint_topic}...')

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
        """Handle incoming pose goal (position + orientation)."""
        if self.debug_mode:
            return  # Ignore external poses in debug mode

        # Only accept external goals after startup delay to prevent sudden jumps
        if time.time() - self.start_time < self.startup_delay:
            return

        pos = msg.pose.position
        ori = msg.pose.orientation
        
        position = self.tensor_args.to_device([pos.x, pos.y, pos.z])
        quaternion = self.tensor_args.to_device([ori.w, ori.x, ori.y, ori.z])

        if self.last_goal_pose is not None:
            # Update both position and orientation
            self.last_goal_pose.position.copy_(position)
            self.last_goal_pose.quaternion.copy_(quaternion)
        else:
            self.last_goal_pose = Pose(position=position, quaternion=quaternion)

        self.goal_received = True

    def joint_state_callback(self, msg: JointState):
        """Handle incoming joint state."""
        # In sim, joints arrive as arm0_* -> remap to arm_ to match URDF/cuRobo
        if self.use_sim:
            msg.name = [n.replace('arm0_', 'arm_') for n in msg.name]
        self.current_joint_state = msg
        self._last_joint_stamp = time.time()
        self.joints_received = True

    def mesh_callback(self, msg: NvbloxMesh):
        """Handle incoming Mesh message from nvblox."""
        with self.mesh_lock:
            # Update blocks in the dictionary
            # msg.block_indices and msg.blocks are parallel arrays
            # If clear is True, clear the map first (though nvblox might partial update)
            if msg.clear:
                self.mesh_blocks.clear()
            
            for i, idx in enumerate(msg.block_indices):
                # idx is Index3D (x,y,z)
                block = msg.blocks[i]
                # Store block if it has vertices
                if len(block.vertices) > 0:
                    idx_tuple = (idx.x, idx.y, idx.z)
                    self.mesh_blocks[idx_tuple] = block
                else:
                    # If empty, remove it if it exists (released block)
                    idx_tuple = (idx.x, idx.y, idx.z)
                    if idx_tuple in self.mesh_blocks:
                        del self.mesh_blocks[idx_tuple]
            
            self.pending_mesh_update = True  # Signal control loop to update
            if not self.nvblox_initialized and len(self.mesh_blocks) > 0:
                self.nvblox_initialized = True
                self.get_logger().info(f'Nvblox mesh received! Frame: {msg.header.frame_id}')
                self.source_frame = msg.header.frame_id

    def _transform_mesh_vertices(self, vertices: np.ndarray, transform: TransformStamped) -> np.ndarray:
        """Transform mesh vertices from source frame to target frame."""
        t = transform.transform.translation
        q = transform.transform.rotation
        
        # Quaternion to rotation matrix
        qx, qy, qz, qw = q.x, q.y, q.z, q.w
        
        # Rotation matrix from quaternion
        rot = np.array([
            [1 - 2*(qy**2 + qz**2), 2*(qx*qy - qz*qw), 2*(qx*qz + qy*qw)],
            [2*(qx*qy + qz*qw), 1 - 2*(qx**2 + qz**2), 2*(qy*qz - qx*qw)],
            [2*(qx*qz - qy*qw), 2*(qy*qz + qx*qw), 1 - 2*(qx**2 + qy**2)]
        ], dtype=np.float32)
        
        translation = np.array([t.x, t.y, t.z], dtype=np.float32)
        
        # Transform vertices (N, 3)
        transformed = (rot @ vertices.T).T + translation
        return transformed

    def _build_curobo_mesh(self, blocks_snapshot: dict, transform: TransformStamped = None):
        """Reconstruct full mesh from blocks snapshot and convert to cuRobo Mesh.
        
        Args:
            blocks_snapshot: A copied dict of {idx: block} (so we don't hold the lock).
            transform: Optional TF to apply.
        """
        all_vertices = []
        all_triangles = []
        vertex_offset = 0

        # Iterate over all stored blocks
        for idx, block in blocks_snapshot.items():
            # Vertices
            # block.vertices is list of Point32
            verts = np.array([[p.x, p.y, p.z] for p in block.vertices], dtype=np.float32)
            
            # Triangles
            # block.triangles is list of int32
            tris = np.array(block.triangles, dtype=np.int32).reshape(-1, 3)
            
            # Adjust triangle indices
            tris += vertex_offset
            
            all_vertices.append(verts)
            all_triangles.append(tris)
            
            vertex_offset += len(verts)

        if not all_vertices:
            return None

        # Concatenate
        full_vertices = np.concatenate(all_vertices, axis=0)
        full_triangles = np.concatenate(all_triangles, axis=0)

        # Decimate if too many vertices
        total_verts = len(full_vertices)
        if total_verts > self.MAX_MESH_VERTICES:
            # Uniform subsampling: keep every N-th vertex
            stride = total_verts // self.MAX_MESH_VERTICES
            # Build a mask of which vertices to keep
            keep_mask = np.zeros(total_verts, dtype=bool)
            keep_mask[::stride] = True
            
            # Remap vertex indices
            new_indices = np.full(total_verts, -1, dtype=np.int32)
            new_indices[keep_mask] = np.arange(keep_mask.sum(), dtype=np.int32)
            
            # Filter triangles: keep only those where all 3 vertices are kept
            tri_mask = (
                keep_mask[full_triangles[:, 0]] &
                keep_mask[full_triangles[:, 1]] &
                keep_mask[full_triangles[:, 2]]
            )
            full_triangles = new_indices[full_triangles[tri_mask]]
            full_vertices = full_vertices[keep_mask]
            
            self.get_logger().debug(
                f'Decimated mesh: {total_verts} -> {len(full_vertices)} verts, '
                f'{tri_mask.sum()} faces'
            )

        # Transform if needed (transform is Target <- Source)
        if transform:
            full_vertices = self._transform_mesh_vertices(full_vertices, transform)

        # Create Curobo Mesh
        # We assume standard 1.0 scale as nvblox is metric
        mesh = Mesh(
            name="nvblox_mesh",
            vertices=full_vertices,
            faces=full_triangles,
            pose=[0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0], # Identity pose in target frame
            color=[0.5, 0.5, 0.5, 1.0], # Grey color
        )
        return mesh

    def _try_update_mesh(self):
        """Try to update mesh world inline (called from control_loop). cuRobo is NOT thread-safe."""
        if not self.nvblox_initialized or not self.pending_mesh_update:
            return

        # If static mesh mode is enabled and we already have a mesh, skip
        if self.static_mesh and self.obstacle_update_count > 0:
            return

        # Snapshot blocks under lock (fast)
        with self.mesh_lock:
            if not self.mesh_blocks:
                return
            blocks_snapshot = dict(self.mesh_blocks)

        self.pending_mesh_update = False
        
        # Check startup delay before adding obstacles
        if time.time() - self.start_time < self.startup_delay:
            self.get_logger().info('Startup delay active: ignoring obstacles for now.', once=True)
            return

        # TF lookup
        try:
            transform = self.tf_buffer.lookup_transform(
                self.target_frame,
                self.source_frame,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.05)
            )
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException) as e:
            if self.obstacle_update_count == 0:
                self.get_logger().warn(f'Waiting for TF {self.source_frame} -> {self.target_frame}: {e}')
            return

        # Build and update (same thread as mpc.step - safe!)
        try:
            t0 = time.time()
            curobo_mesh = self._build_curobo_mesh(blocks_snapshot, transform)

            if curobo_mesh is not None:
                mesh_world = WorldConfig(mesh=[curobo_mesh])
                self.mpc.update_world(mesh_world)
                self.obstacle_update_count += 1
                elapsed = time.time() - t0
                self.last_mesh_update_time = time.time()

                self.get_logger().info(
                    f'Mesh update #{self.obstacle_update_count}: '
                    f'{len(curobo_mesh.vertices)} verts, {len(curobo_mesh.faces)} faces '
                    f'({elapsed:.3f}s)'
                )

                if self.static_mesh and self.obstacle_update_count > 0:
                    self.get_logger().info("Mesh update COMPLETED. FREEZING updates (static_mesh=True).")

        except Exception as e:
            self.get_logger().warn(f'Failed to update mesh world: {e}')



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
            # self.get_logger().debug('Searching for world_coll_checker...')
            for attr in dir(self.mpc.rollout_fn):
                if 'prim' in attr.lower():
                    prim = getattr(self.mpc.rollout_fn, attr)
                    if hasattr(prim, 'world_coll_checker'):
                        world_coll = prim.world_coll_checker
                        break

        if world_coll is None:
            return []

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
                 self.get_logger().error("sphere_dist error: link_spheres_tensor is None")
                 self._sphere_none_logged = True
             return []
        
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
                self.get_logger().error(f"CollisionQueryBuffer init error: {e}")
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
                self.get_logger().error(f"get_sphere_distance error: {e}")
                import traceback
                self.get_logger().error(traceback.format_exc())
                self._sphere_dist_calc_logged = True
            return []

    def control_loop(self):
        """Main MPC control loop."""
        t_loop_start = time.time()
        loop_gap = t_loop_start - self._last_loop_time  # time since last call
        self._last_loop_time = t_loop_start

        # Update mesh world inline (cuRobo is NOT thread-safe for world updates)
        # The heavy deserialization happens in mesh_callback (separate thread),
        # so this just takes the lock and updates the world (fast).
        t_mesh_start = time.time()
        self._try_update_mesh()
        t_mesh_end = time.time()

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
            if not self.goal_received and (time.time() - self.start_time >= self.startup_delay):
                # Before external goals arrive, maintain current EE pose to prevent jerking
                if self.joints_received and self.current_joint_state is not None:
                    # Initialize goal_pose to current FK pose exactly once
                    if getattr(self, '_init_goal_set', False) is False:
                        positions = list(self.current_joint_state.position)
                        joint_names_msg = list(self.current_joint_state.name)
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
                        self.get_logger().info(
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
                        self.get_logger().info(
                            f'Warm-up done ({WARMUP_ITERS} iters). Max joint diff: {max_diff:.4f} rad\n'
                            f'  Real:    [{", ".join(f"{p:.3f}" for p in real_pos)}]\n'
                            f'  MPC cmd: [{", ".join(f"{p:.3f}" for p in warmup_cmd)}]'
                        )
                        
                        self.goal_received = True
                        self._init_goal_set = True
                        self.get_logger().info('Initialized goal to current physical pose to prevent jumping.')
                return
            if not self.joints_received or self.current_joint_state is None or self.last_goal_pose is None:
                return

        # Guard: skip MPC if joint state is too stale (Isaac Sim stopped publishing)
        JS_STALE_THRESHOLD = 2.0  # seconds
        if self._last_joint_stamp is not None:
            js_age_now = time.time() - self._last_joint_stamp
            if js_age_now > JS_STALE_THRESHOLD:
                if self.step_count % 30 == 0:
                    self.get_logger().warn(
                        f'⚠️ Joint state is STALE ({js_age_now*1000:.0f}ms)! '
                        f'Isaac Sim may have stopped publishing. Skipping MPC step.'
                    )
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
                
                # 3. Compute Distance
                dist = np.linalg.norm(target_pos_np - current_ee_pos_np)
                
                # 4. Apply Attraction if close
                if dist < 0.40:
                    delta = self.compute_attractive(current_ee_pos_np, target_pos_np, k_att=0.2)
                    new_pos_np = current_ee_pos_np + delta
                    
                    # Update the goal position (keep orientation from last_goal_pose)
                    new_pos_tensor = self.tensor_args.to_device(new_pos_np)
                    self.goal_buffer.goal_pose.position.copy_(new_pos_tensor)
                    
                    if self.step_count % 30 == 0: # Log occasionally
                        self.get_logger().info(
                            f'🧲 Attractive Field Active! Dist={dist:.3f}m -> Pulling to object.'
                        )
                elif self.step_count % 30 == 0:
                     self.get_logger().info(f'👀 Target detected. Dist={dist:.3f}m (Approaching...)')

            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
                # Target not detected or TF not ready
                if self.step_count % 100 == 0:
                    self.get_logger().info(f'⚠️ Target TF not found: {e}')
                pass
            except Exception as e:
                self.get_logger().warn(f'Potential field error: {e}')
            
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

            # Joint tracking error (mean |cmd - curr| over arm joints)
            try:
                curr_ordered = cu_js.get_ordered_joint_state(ordered_names)
                curr_arr = curr_ordered.position.view(-1).cpu().numpy()
                _jtrack = float(np.mean(np.abs(np.array(pos_list) - curr_arr)))
            except Exception:
                _jtrack = 0.0

            t_loop_end = time.time()
            total_loop_dt = t_loop_end - t_loop_start
            mesh_dt = t_mesh_end - t_mesh_start
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

            # CSV row every step
            if hasattr(self, '_csv_writer'):
                _gpu_u, _, _, _, _, _ = self._get_gpu_stats() if self.step_count % 10 == 0 else (-1, 0, 0, 0, 0, 0)
                _sd = self._get_sphere_distances() if self.step_count % 10 == 0 else []
                _min_d = max(_sd) if _sd else -1.0
                self._csv_writer.writerow([
                    self.step_count,
                    'OK' if is_feasible else 'BLOCKED',
                    int(total_loop_dt * 1000),
                    int(step_dt * 1000),
                    _gpu_u, int(js_age * 1000) if js_age >= 0 else -1,
                    round(_min_d, 4),
                    round(mpc_result.metrics.pose_error.item(), 4),
                    round(coll_constraint, 4),
                    round(coll_cost, 4),
                    round(evasion_dev, 4),
                    round(_jtrack, 4),
                ])
                if self.step_count % 100 == 0:
                    self._csv_file.flush()

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
                self.get_logger().info(
                    f'\n'
                    f'  === Step {self.step_count} [{status}] ===\n'
                    f'  MPC: error={mpc_result.metrics.pose_error.item():.4f}, constraint={coll_constraint:.4f}, coll_cost={coll_cost:.4f}, evasion_dev={evasion_dev:.4f}m\n'
                    f'  TIMING: loop={total_loop_dt*1000:.0f}ms, mpc.step={step_dt*1000:.0f}ms, mesh={mesh_dt*1000:.0f}ms, gap={loop_gap*1000:.0f}ms\n'
                    f'  AVG: loop={avg_loop*1000:.0f}ms, step={avg_step*1000:.0f}ms, effective={effective_hz:.1f}Hz\n'
                    f'  GPU: util={gpu_util}%, mem={gpu_mem_used}/{gpu_mem_total}MB, temp={gpu_temp}C\n'
                    f'  TORCH: alloc={torch_alloc}MB, reserved={torch_reserved}MB\n'
                    f'  DATA: js_age={js_age*1000:.0f}ms, pub_count={self._publish_count}\n'
                    f'  OBST: min_dist={min_dist:.4f}m, sphere_idx={closest_sphere}\n'
                    f'  CMD:  pos=[{pos_list[0]:.3f},{pos_list[1]:.3f},{pos_list[2]:.3f},{pos_list[3]:.3f},{pos_list[4]:.3f},{pos_list[5]:.3f}]\n'
                    f'  CURR: pos=[{curr_pos_list[0]:.3f},{curr_pos_list[1]:.3f},{curr_pos_list[2]:.3f},{curr_pos_list[3]:.3f},{curr_pos_list[4]:.3f},{curr_pos_list[5]:.3f}]'
                )

        except Exception as e:
            import traceback
            self.get_logger().error(f'Control loop error: {e}\n{traceback.format_exc()}')


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
        if hasattr(node, '_csv_file'):
            node._csv_file.flush()
            node._csv_file.close()
            print(f"CSV salvo: {node._csv_path}")
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass  # Already shut down


if __name__ == '__main__':
    main()
