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
from curobo.geom.sdf.world import CollisionCheckerType
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



class CuroboMpcNode(Node):
    """ROS2 Node for cuRobo MPC-based motion planning."""

    def __init__(self):
        super().__init__('curobo_mpc_node')

        # Declare parameters
        self.declare_parameter('robot_config', '')
        self.declare_parameter('urdf_path', '')
        self.declare_parameter('control_rate', 30.0)
        self.declare_parameter('step_dt', 0.03)

        self.declare_parameter('debug_mode', False)
        self.declare_parameter('debug_pose_duration', 3.0)  # seconds between pose changes
        
        # nvblox Mesh parameters
        self.declare_parameter('mesh_topic', '/nvblox_node/mesh')
        self.declare_parameter('mesh_update_rate', 2.0)  # Hz (limit updates to curobo)

        # Get parameters
        robot_config_path = self.get_parameter('robot_config').get_parameter_value().string_value
        urdf_path = self.get_parameter('urdf_path').get_parameter_value().string_value
        control_rate = self.get_parameter('control_rate').get_parameter_value().double_value
        step_dt = self.get_parameter('step_dt').get_parameter_value().double_value

        self.debug_mode = self.get_parameter('debug_mode').get_parameter_value().bool_value
        self.debug_pose_duration = self.get_parameter('debug_pose_duration').get_parameter_value().double_value
        
        # nvblox parameters
        self.mesh_topic = self.get_parameter('mesh_topic').get_parameter_value().string_value
        self.mesh_update_rate = self.get_parameter('mesh_update_rate').get_parameter_value().double_value

        # ... (Keeping default paths logic)
        os.environ['SPOT_URDF_PATH'] = os.path.dirname(urdf_path)

        self.get_logger().info('=== cuRobo MPC Node Starting ===')
        self.get_logger().info(f'Robot config: {robot_config_path}')
        self.get_logger().info(f'URDF path: {urdf_path}')
        self.get_logger().info(f'Control rate: {control_rate} Hz')
        # ... (Logging)
        self.get_logger().info(f'Mesh topic: {self.mesh_topic}')

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



        # World config (empty for now)
        world_cfg = WorldConfig()
        self.obstacle_update_count = 0


        # MPC Configuration
        self.get_logger().info('Loading MPC solver...')
        mpc_config = MpcSolverConfig.load_from_robot_config(
            self.robot_cfg,
            world_cfg,
            use_cuda_graph=True,
            use_cuda_graph_metrics=True,
            use_cuda_graph_full_step=False,
            self_collision_check=True,
            collision_checker_type=CollisionCheckerType.MESH,
            collision_cache={"mesh": 20}, # Adjust cache for mesh
            collision_activation_distance=0.03,  # Start avoiding obstacles from 3cm away
            use_mppi=True,
            use_lbfgs=False,
            use_es=False,
            store_rollouts=False,
            step_dt=step_dt,
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
        
        # nvblox Mesh subscriber
        self.mesh_sub = self.create_subscription(
            NvbloxMesh, 
            self.mesh_topic, 
            self.mesh_callback, 
            qos
        )
        self.get_logger().info(f'Subscribed to Mesh topic: {self.mesh_topic}')

        # Control timer
        self.control_timer = self.create_timer(1.0 / control_rate, self.control_loop)
        self.step_count = 0
        
        # Mesh update timer
        self.mesh_update_timer = self.create_timer(
            1.0 / self.mesh_update_rate, 
            self.update_mesh_world_callback
        )

        self.get_logger().info('=== cuRobo MPC Node Ready ===')
        if self.debug_mode:
            self.get_logger().info('DEBUG MODE ACTIVE - Using test poses')
            self.get_logger().info(f'Will cycle through {len(self.debug_test_poses)} test positions')
            self.goal_received = True  # Auto-ready in debug mode
        else:
            self.get_logger().info('Waiting for /wrist_pose and /joint_states_isaac...')

    # ... (Keeping _generate_test_poses, _euler_to_quaternion, _get_debug_pose, pose_callback, joint_state_callback)


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
            
            self.last_mesh_update_time = time.time()
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

    def _build_curobo_mesh(self, transform: TransformStamped = None):
        """Reconstruct full mesh from blocks and convert to cuRobo Mesh."""
        all_vertices = []
        all_triangles = []
        vertex_offset = 0

        # Iterate over all stored blocks
        for idx, block in self.mesh_blocks.items():
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

    def update_mesh_world_callback(self):
        """Periodically update collision model with reconstructed mesh."""
        if not self.nvblox_initialized:
            return

        with self.mesh_lock:
             # Check if we have blocks
            if not self.mesh_blocks:
                return
            
            # Get latest transform
            # We want to transform FROM nvblox frame (odom) TO curobo frame (body)
            # So looking up transform: target=body, source=odom
            try:
                transform = self.tf_buffer.lookup_transform(
                    self.target_frame,
                    self.source_frame,
                    rclpy.time.Time(),
                    timeout=rclpy.duration.Duration(seconds=0.1)
                )
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, 
                    tf2_ros.ExtrapolationException) as e:
                self.get_logger().warn(f'Waiting for TF {self.source_frame} -> {self.target_frame}: {e}')
                return

            # Build and update world
            try:
                curobo_mesh = self._build_curobo_mesh(transform)
                
                if curobo_mesh is not None:
                    # Create new world config with just this mesh
                    mesh_world = WorldConfig(mesh=[curobo_mesh])
                    
                    self.mpc.update_world(mesh_world)
                    self.obstacle_update_count += 1
                    
                    if self.obstacle_update_count % 10 == 0:
                        self.get_logger().info(
                            f'Mesh update #{self.obstacle_update_count}: '
                            f'{len(curobo_mesh.vertices)} vertices, {len(curobo_mesh.faces)} faces'
                        )

            except Exception as e:
                self.get_logger().warn(f'Failed to update mesh world: {e}')



    def compute_attractive(self, current: np.ndarray, target: np.ndarray, k_att: float = 0.1) -> np.ndarray:
        return k_att * (target - current)

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



            joint_cmd = JointState()
            joint_cmd.header.stamp = self.get_clock().now().to_msg()
            joint_cmd.name = ordered_names
            joint_cmd.position = pos_list
            joint_cmd.velocity = vel_list
            joint_cmd.effort = []

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
