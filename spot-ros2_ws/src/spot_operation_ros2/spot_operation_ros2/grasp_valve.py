#!/usr/bin/env python3

"""
Valve Grasping Node for Boston Dynamics Spot Robot
Performs 6D pose-based grasping of valves using FoundationPose detection and MoveIt motion planning.
Uses pure ROS2 Action API without moveit_commander.
"""

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.action import ActionClient
from std_srvs.srv import Trigger
from std_msgs.msg import Float64
from vision_msgs.msg import Detection3DArray
from geometry_msgs.msg import PoseStamped, Pose, TransformStamped
from tf2_ros import Buffer, TransformListener, LookupException, ConnectivityException, ExtrapolationException, TransformException
from tf2_geometry_msgs import do_transform_pose
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    Constraints, PositionConstraint, OrientationConstraint, 
    BoundingVolume, MotionPlanRequest, RobotState
)
from shape_msgs.msg import SolidPrimitive
import yaml
import numpy as np
from pathlib import Path
import time
from collections import deque
import math
from moveit_msgs.srv import GetCartesianPath
from moveit_msgs.action import ExecuteTrajectory
from control_msgs.action import FollowJointTrajectory
import tf_transformations


class ValveGraspNode(Node):
    """
    ROS2 Node for grasping valves using 6D pose estimation and MoveIt motion planning.
    
    Subscribes to /valve_pose for 6D detection, uses TF tree to compute grasp poses,
    and executes grasping motions using MoveIt Action API with separate gripper control.
    """
    
    # Moving average window size
    POSE_WINDOW_SIZE = 5
    
    # MoveIt configuration
    PLANNING_GROUP = "arm"
    END_EFFECTOR_LINK = "arm_link_wr1"  # From spot.srdf
    PLANNING_FRAME = "body"
    
    # Planning tolerances
    POSITION_TOLERANCE = 0.001  # 1mm
    ORIENTATION_TOLERANCE = 0.01  # ~0.57 degrees
    PLANNING_TIME = 5.0  # seconds
    
    def __init__(self):
        super().__init__('valve_grasp_node')
        
        # Callback group for parallel execution
        self.callback_group = ReentrantCallbackGroup()
        
        # Create MoveGroup action client
        self.get_logger().info("Creating MoveGroup action client...")
        self.move_group_client = ActionClient(
            self,
            MoveGroup,
            '/move_action',
            callback_group=self.callback_group
        )
        
        # Wait for action server
        self.get_logger().info("Waiting for /move_action server...")
        if not self.move_group_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error("MoveGroup action server not available!")
            raise RuntimeError("MoveGroup action server not available")
        
        self.get_logger().info(f"Connected to MoveGroup action server successfully!")
        self.get_logger().info(f"Planning group: {self.PLANNING_GROUP}")
        self.get_logger().info(f"End effector: {self.END_EFFECTOR_LINK}")
        self.get_logger().info(f"Planning frame: {self.PLANNING_FRAME}")
        
        # Cartesian path service
        self.cartesian_client = self.create_client(
            GetCartesianPath,
            '/compute_cartesian_path',
            callback_group=self.callback_group
        )
        self.get_logger().info("Waiting for /compute_cartesian_path service...")
        if not self.cartesian_client.wait_for_service(timeout_sec=10.0):
            self.get_logger().error("GetCartesianPath service not available!")
            raise RuntimeError("GetCartesianPath service not available")
        
        # ExecuteTrajectory action
        self.exec_traj_client = ActionClient(
            self,
            ExecuteTrajectory,
            '/execute_trajectory',
            callback_group=self.callback_group
        )
        self.get_logger().info("Waiting for /execute_trajectory action server...")
        if not self.exec_traj_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error("ExecuteTrajectory action server not available!")
            raise RuntimeError("ExecuteTrajectory action server not available")
        
        # FollowJointTrajectory action client
        self.get_logger().info("Creating FollowJointTrajectory action client...")
        self.fjt_client = ActionClient(
            self,
            FollowJointTrajectory,
            '/arm_controller/follow_joint_trajectory',
            callback_group=self.callback_group
        )
        self.get_logger().info("Waiting for /arm_controller/follow_joint_trajectory action server...")
        if not self.fjt_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error("FollowJointTrajectory action server not available!")
            raise RuntimeError("FollowJointTrajectory action server not available")
        
        # TF2 setup for coordinate transformations
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Load grasp configuration from YAML
        self.grasp_config = self._load_grasp_config()
        
        # Extract gripper angles from YAML (dynamic per grasp configuration)
        self.gripper_open_angle = None
        self.gripper_close_angle = None
        if self.grasp_config:
            self._extract_gripper_angles()
        
        # Valve pose tracking with moving average
        self.valve_poses = deque(maxlen=self.POSE_WINDOW_SIZE)
        self.latest_valve_detection = None
        self.detection_lock = False
        
        # Store last planned grasp for execute-only service
        self.last_planned_goal = None
        self.last_grasp_pose = None
        
        # Subscribe to valve pose detections
        self.valve_pose_sub = self.create_subscription(
            Detection3DArray,
            '/valve_pose',
            self.valve_pose_callback,
            10,
            callback_group=self.callback_group
        )
        
        # Gripper command publisher (publishes to gripper_controller node)
        self.gripper_cmd_pub = self.create_publisher(
            Float64,
            'gripper/command',
            10
        )
        
        # Services for grasp control (debug-friendly)
        self.plan_service = self.create_service(
            Trigger,
            'plan_grasp',
            self.plan_grasp_callback,
            callback_group=self.callback_group
        )
        
        self.execute_service = self.create_service(
            Trigger,
            'execute_grasp',
            self.execute_grasp_callback,
            callback_group=self.callback_group
        )
        
        self.grasp_service = self.create_service(
            Trigger,
            'grasp_valve',
            self.grasp_service_callback,
            callback_group=self.callback_group
        )
        
        self.close_valve_service = self.create_service(
            Trigger,
            'close_valve',
            self.close_valve_cartesian_callback,
            callback_group=self.callback_group
        )
        
        self.get_logger().info("Valve Grasp Node initialized successfully!")
        self.get_logger().info("Available services:")
        self.get_logger().info("  - /plan_grasp: Plan motion only (no execution, no gripper)")
        self.get_logger().info("  - /execute_grasp: Execute planned motion with pre-grasp")
        self.get_logger().info("  - /grasp_valve: Complete grasp (plan + execute + gripper)")
        self.get_logger().info("  - /close_valve: Close valve using Cartesian path")
        
        # Open gripper to pre-grasp position on startup
        self.get_logger().info("=" * 60)
        self.get_logger().info("Opening gripper to pre-grasp position on startup...")
        self.open_gripper()
        self.get_logger().info("Gripper ready! Node is now ready for grasp commands.")
        self.get_logger().info("=" * 60)
    
    def _load_grasp_config(self):
        """Load grasp configuration from spot_grasp.yaml"""
        package_path = Path(__file__).parent
        yaml_path = package_path / "spot_grasp.yaml"
        
        try:
            with open(yaml_path, 'r') as f:
                config = yaml.safe_load(f)
            self.get_logger().info(f"Loaded grasp config from {yaml_path}")
            return config
        except Exception as e:
            self.get_logger().error(f"Failed to load grasp config: {e}")
            return None
    
    def _extract_gripper_angles(self):
        """
        Extract gripper angles from loaded YAML configuration.
        These values may vary depending on the grasp configuration.
        """
        try:
            grasp_data = self.grasp_config['grasps']['grasp_0']
            
            # Extract pre-grasp (open) and grasp (close) positions
            self.gripper_open_angle = grasp_data['pregrasp_cspace_position']['arm0_f1x']
            self.gripper_close_angle = grasp_data['cspace_position']['arm0_f1x']
            
            self.get_logger().info("=" * 60)
            self.get_logger().info("Gripper angles extracted from YAML:")
            self.get_logger().info(f"  Open (pre-grasp): {self.gripper_open_angle:.4f} rad")
            self.get_logger().info(f"  Close (grasp):    {self.gripper_close_angle:.4f} rad")
            self.get_logger().info("=" * 60)
            
        except KeyError as e:
            self.get_logger().error(f"Failed to extract gripper angles from YAML: {e}")
            # Fallback to default values
            self.gripper_open_angle = -1.5707999467849731
            self.gripper_close_angle = -0.19896012544631958
            self.get_logger().warn("Using default gripper angles as fallback")
    
    def valve_pose_callback(self, msg: Detection3DArray):
        """
        Callback for valve pose detections. Stores poses for moving average filtering.
        """
        if len(msg.detections) == 0:
            return
        
        # Take the first (best) detection
        detection = msg.detections[0]
        
        # Store pose for moving average
        pose_stamped = PoseStamped()
        pose_stamped.header = detection.header
        pose_stamped.pose = detection.results[0].pose.pose
        
        self.valve_poses.append(pose_stamped)
        self.latest_valve_detection = detection
        
        # Log detection count
        if len(self.valve_poses) == 1:
            self.get_logger().info(f"First valve detection received in frame: {detection.header.frame_id}")
    
    def get_averaged_valve_pose(self):
        """
        Compute averaged valve pose from recent detections.
        Returns PoseStamped in the detection frame.
        """
        if len(self.valve_poses) == 0:
            return None
        
        # Average position
        positions = np.array([[p.pose.position.x, p.pose.position.y, p.pose.position.z] 
                              for p in self.valve_poses])
        avg_position = np.mean(positions, axis=0)
        
        # For quaternions, use the most recent one (averaging quaternions properly requires slerp)
        # Simple approach: use latest quaternion
        latest_orientation = self.valve_poses[-1].pose.orientation
        
        avg_pose = PoseStamped()
        avg_pose.header = self.valve_poses[-1].header
        avg_pose.pose.position.x = avg_position[0]
        avg_pose.pose.position.y = avg_position[1]
        avg_pose.pose.position.z = avg_position[2]
        avg_pose.pose.orientation = latest_orientation
        
        return avg_pose
    
    def lookup_transform(self, target_frame, source_frame, timeout=5.0):
        """
        Look up transform from source_frame to target_frame.
        Uses Time(0) which explicitly requests the latest available transform.
        """
        try:
            # Time(0) explicitly requests the latest available transform without interpolation
            transform = self.tf_buffer.lookup_transform(
                target_frame,
                source_frame,
                rclpy.time.Time(seconds=0, nanoseconds=0),
                timeout=rclpy.duration.Duration(seconds=timeout)
            )
            return transform
        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            self.get_logger().error(f"TF lookup failed ({source_frame} -> {target_frame}): {e}")
            return None
    
    def _quat_to_rot(self, q):
        """Convert quaternion to 3x3 rotation matrix."""
        x, y, z, w = q.x, q.y, q.z, q.w
        xx, yy, zz = x*x, y*y, z*z
        xy, xz, yz = x*y, x*z, y*z
        wx, wy, wz = w*x, w*y, w*z
        return np.array([
            [1-2*(yy+zz), 2*(xy-wz),   2*(xz+wy)],
            [2*(xy+wz),   1-2*(xx+zz), 2*(yz-wx)],
            [2*(xz-wy),   2*(yz+wx),   1-2*(xx+yy)]
        ])
    
    def _rot_to_quat(self, R):
        """Convert 3x3 rotation matrix to quaternion."""
        from geometry_msgs.msg import Quaternion
        
        t = np.trace(R)
        if t > 0:
            s = math.sqrt(t+1.0)*2
            w = 0.25*s
            x = (R[2,1]-R[1,2])/s
            y = (R[0,2]-R[2,0])/s
            z = (R[1,0]-R[0,1])/s
        else:
            i = np.argmax([R[0,0], R[1,1], R[2,2]])
            if i == 0:
                s = math.sqrt(1.0+R[0,0]-R[1,1]-R[2,2])*2
                w = (R[2,1]-R[1,2])/s
                x = 0.25*s
                y = (R[0,1]+R[1,0])/s
                z = (R[0,2]+R[2,0])/s
            elif i == 1:
                s = math.sqrt(1.0+R[1,1]-R[0,0]-R[2,2])*2
                w = (R[0,2]-R[2,0])/s
                x = (R[0,1]+R[1,0])/s
                y = 0.25*s
                z = (R[1,2]+R[2,1])/s
            else:
                s = math.sqrt(1.0+R[2,2]-R[0,0]-R[1,1])*2
                w = (R[1,0]-R[0,1])/s
                x = (R[0,2]+R[2,0])/s
                y = (R[1,2]+R[2,1])/s
                z = 0.25*s
        
        q = Quaternion()
        q.x, q.y, q.z, q.w = x, y, z, w
        return q
    
    def _transform_to_matrix(self, tf):
        """Convert Transform message to 4x4 homogeneous matrix."""
        t = tf.translation
        q = tf.rotation
        R = self._quat_to_rot(q)
        T = np.eye(4)
        T[:3,:3] = R
        T[:3, 3] = np.array([t.x, t.y, t.z])
        return T
    
    def _pose_to_matrix(self, pose):
        """Convert Pose message to 4x4 homogeneous matrix."""
        T = np.eye(4)
        T[:3,:3] = self._quat_to_rot(pose.orientation)
        T[:3, 3] = np.array([pose.position.x, pose.position.y, pose.position.z])
        return T
    
    def _matrix_to_pose(self, T):
        """Convert 4x4 homogeneous matrix to Pose message."""
        from geometry_msgs.msg import Pose
        
        p = Pose()
        p.position.x, p.position.y, p.position.z = T[0,3], T[1,3], T[2,3]
        q = self._rot_to_quat(T[:3,:3])
        p.orientation = q
        return p
    
    def _axis_rot(self, theta, axis='z'):
        """
        Create a 4x4 rotation matrix around a given axis.
        
        Args:
            theta: Rotation angle in radians
            axis: 'x', 'y', or 'z'
        
        Returns:
            4x4 homogeneous rotation matrix
        """
        R = np.eye(4)
        c, s = math.cos(theta), math.sin(theta)
        
        if axis == 'x':
            R[:3,:3] = np.array([[1, 0, 0],
                                 [0, c, -s],
                                 [0, s, c]])
        elif axis == 'y':
            R[:3,:3] = np.array([[c, 0, s],
                                 [0, 1, 0],
                                 [-s, 0, c]])
        else:  # 'z'
            R[:3,:3] = np.array([[c, -s, 0],
                                 [s, c, 0],
                                 [0, 0, 1]])
        return R
    
    def construct_goal_constraints(self, pose_stamped: PoseStamped, 
                                   tolerance_pos: float = None, 
                                   tolerance_angle: float = None) -> Constraints:
        """
        Construct goal constraints from a PoseStamped message.
        Mimics kinematic_constraints::constructGoalConstraints from MoveIt C++ API.
        
        Args:
            pose_stamped: Target pose for the end effector
            tolerance_pos: Position tolerance in meters (default: self.POSITION_TOLERANCE)
            tolerance_angle: Orientation tolerance in radians (default: self.ORIENTATION_TOLERANCE)
            
        Returns:
            Constraints message with position and orientation constraints
        """
        if tolerance_pos is None:
            tolerance_pos = self.POSITION_TOLERANCE
        if tolerance_angle is None:
            tolerance_angle = self.ORIENTATION_TOLERANCE
        
        constraints = Constraints()
        
        # Position constraint (sphere around target position)
        pos_constraint = PositionConstraint()
        pos_constraint.header = pose_stamped.header
        pos_constraint.link_name = self.END_EFFECTOR_LINK
        pos_constraint.target_point_offset.x = 0.0
        pos_constraint.target_point_offset.y = 0.0
        pos_constraint.target_point_offset.z = 0.0
        
        # Constraint region: sphere with radius = tolerance_pos
        bounding_volume = BoundingVolume()
        sphere = SolidPrimitive()
        sphere.type = SolidPrimitive.SPHERE
        sphere.dimensions = [tolerance_pos]  # SPHERE_RADIUS
        
        bounding_volume.primitives = [sphere]
        bounding_volume.primitive_poses = [Pose()]
        bounding_volume.primitive_poses[0].position = pose_stamped.pose.position
        bounding_volume.primitive_poses[0].orientation.w = 1.0
        
        pos_constraint.constraint_region = bounding_volume
        pos_constraint.weight = 1.0
        
        constraints.position_constraints = [pos_constraint]
        
        # Orientation constraint
        orient_constraint = OrientationConstraint()
        orient_constraint.header = pose_stamped.header
        orient_constraint.link_name = self.END_EFFECTOR_LINK
        orient_constraint.orientation = pose_stamped.pose.orientation
        orient_constraint.absolute_x_axis_tolerance = tolerance_angle
        orient_constraint.absolute_y_axis_tolerance = tolerance_angle
        orient_constraint.absolute_z_axis_tolerance = tolerance_angle
        orient_constraint.weight = 1.0
        
        constraints.orientation_constraints = [orient_constraint]
        
        return constraints
    
    def wait_for_future(self, future, timeout_sec=30.0):
        """
        Wait for a future to complete without blocking the executor.
        Uses polling instead of spin_until_future_complete to avoid conflicts.
        
        Args:
            future: The future to wait for
            timeout_sec: Maximum time to wait in seconds
            
        Returns:
            The result of the future, or None if timeout
        """
        start_time = time.time()
        rate = self.create_rate(10)  # 10 Hz polling
        
        while time.time() - start_time < timeout_sec:
            if future.done():
                return future.result()
            rate.sleep()
        
        return None
    
    def compute_grasp_pose_in_planning_frame(self):
        """
        Compute the grasp pose in the planning frame (body).
        
        Uses TF tree: body -> lever_pivot
        Applies grasp offset from spot_grasp.yaml
        Returns PoseStamped in planning frame.
        """
        # Wait for averaged pose
        if len(self.valve_poses) < 3:
            self.get_logger().warn(f"Insufficient detections for averaging: {len(self.valve_poses)}/3")
            return None
        
        # Get lever_pivot frame from config
        if not self.grasp_config:
            self.get_logger().error("Grasp config not loaded!")
            return None
        
        grasp_data = self.grasp_config['grasps']['grasp_0']
        
        # Extract grasp offset relative to lever_pivot
        offset_position = grasp_data['position']
        offset_orientation = grasp_data['orientation']
        
        # Create grasp pose in lever_pivot frame
        grasp_in_lever_pivot = PoseStamped()
        grasp_in_lever_pivot.header.frame_id = "lever_pivot"
        grasp_in_lever_pivot.header.stamp = self.get_clock().now().to_msg()
        grasp_in_lever_pivot.pose.position.x = offset_position[0]
        grasp_in_lever_pivot.pose.position.y = offset_position[1]
        grasp_in_lever_pivot.pose.position.z = offset_position[2]
        grasp_in_lever_pivot.pose.orientation.w = offset_orientation['w']
        grasp_in_lever_pivot.pose.orientation.x = offset_orientation['xyz'][0]
        grasp_in_lever_pivot.pose.orientation.y = offset_orientation['xyz'][1]
        grasp_in_lever_pivot.pose.orientation.z = offset_orientation['xyz'][2]
        
        # Transform to planning frame (body)
        try:
            transform = self.lookup_transform(self.PLANNING_FRAME, "lever_pivot", timeout=5.0)
            if transform is None:
                return None
            
            grasp_in_body = do_transform_pose(grasp_in_lever_pivot.pose, transform)
            
            grasp_pose = PoseStamped()
            grasp_pose.header.frame_id = self.PLANNING_FRAME
            grasp_pose.header.stamp = self.get_clock().now().to_msg()
            grasp_pose.pose = grasp_in_body
            
            self.get_logger().info(f"Computed grasp pose in {self.PLANNING_FRAME} frame")
            self.get_logger().info(f"Position: [{grasp_pose.pose.position.x:.3f}, "
                                   f"{grasp_pose.pose.position.y:.3f}, "
                                   f"{grasp_pose.pose.position.z:.3f}]")
            
            return grasp_pose
            
        except Exception as e:
            self.get_logger().error(f"Failed to transform grasp pose: {e}")
            return None
    
    def open_gripper(self):
        """
        Open gripper to pre-grasp position by publishing to gripper_controller.
        Waits for motion to complete.
        """
        if self.gripper_open_angle is None:
            self.get_logger().error("Gripper open angle not configured!")
            return
        
        self.get_logger().info(f"Opening gripper to {self.gripper_open_angle:.4f} rad...")
        cmd_msg = Float64()
        cmd_msg.data = self.gripper_open_angle
        self.gripper_cmd_pub.publish(cmd_msg)
        # Wait for gripper to reach position (duration + buffer)
        time.sleep(2.0)
    
    def close_gripper(self):
        """
        Close gripper to grasp position by publishing to gripper_controller.
        Waits for motion to complete.
        """
        if self.gripper_close_angle is None:
            self.get_logger().error("Gripper close angle not configured!")
            return
        
        self.get_logger().info(f"Closing gripper to {self.gripper_close_angle:.4f} rad...")
        cmd_msg = Float64()
        cmd_msg.data = self.gripper_close_angle
        self.gripper_cmd_pub.publish(cmd_msg)
        # Wait for gripper to reach position (duration + buffer)
        time.sleep(2.0)
    
    def rotate_valve(self, theta_deg=90.0, steps=96, axis='z', max_step=0.001, avoid_collisions=False):
        """
        Rotate the valve by describing a circular arc around the valve axis.
        Uses EE position as the robust reference, computing radial distance in body frame.
        
        This approach is more robust than using lever_pivot_aligned directly because:
        - EE position (arm_link_wr1) is always consistent (from robot odometry)
        - lever_pivot_aligned is used only to define rotation center (once)
        - All waypoints are computed in body frame (consistent coordinate system)
        - Small errors in pivot detection only affect initial position, not arc shape
        
        Args:
            theta_deg: Rotation angle in degrees (default: 90.0)
            steps: Number of waypoints for smooth motion (default: 96)
            axis: Rotation axis ('z' for vertical valve rotation, default: 'z')
            max_step: Cartesian interpolation resolution in meters (default: 0.001)
            avoid_collisions: Whether to check collisions during planning (default: False)
        
        Returns:
            bool: True if rotation executed successfully, False otherwise
        """
        try:
            theta_rad = math.radians(theta_deg)
            
            # Get current EE position in body frame (always consistent)
            tf_body_ee = self.lookup_transform(self.PLANNING_FRAME, self.END_EFFECTOR_LINK, timeout=3.0)
            if tf_body_ee is None:
                self.get_logger().error("Failed to get TF body->EE")
                return False
            
            # Get pivot center in body frame (used only to define rotation center)
            tf_body_pivot = self.lookup_transform(self.PLANNING_FRAME, "lever_pivot_aligned", timeout=3.0)
            if tf_body_pivot is None:
                self.get_logger().error("Failed to get TF body->lever_pivot_aligned")
                return False
            
            # Extract rotation axis from lever_pivot_aligned frame
            # The Y axis of lever_pivot_aligned is the valve rotation axis (horizontal valve)
            pivot_rotation = self._quat_to_rot(tf_body_pivot.transform.rotation)
            axis_vector = pivot_rotation[:, 1]  # Y axis (2nd column) of pivot frame in body coordinates
            
            self.get_logger().info(f"Valve axis in body frame: [{axis_vector[0]:.3f}, {axis_vector[1]:.3f}, {axis_vector[2]:.3f}]")
            
            # Extract positions in body frame
            ee_x = tf_body_ee.transform.translation.x
            ee_y = tf_body_ee.transform.translation.y
            ee_z = tf_body_ee.transform.translation.z
            
            pivot_x = tf_body_pivot.transform.translation.x
            pivot_y = tf_body_pivot.transform.translation.y
            pivot_z = tf_body_pivot.transform.translation.z
            
            # Compute radius vector (center → EE) in body frame
            dx = ee_x - pivot_x
            dy = ee_y - pivot_y
            dz = ee_z - pivot_z
            
            # For valve rotation: use ONLY the distance along the valve axis as radius
            # If valve is aligned with Y, radius = |dy|
            # Movement will be in the plane perpendicular to X (i.e., YZ plane)
            
            # Determine which axis component to use as radius based on valve orientation
            axis_components = np.abs(axis_vector)
            dominant_axis_idx = np.argmax(axis_components)
            
            # Map to axis names
            axis_names = ['X', 'Y', 'Z']
            valve_axis_name = axis_names[dominant_axis_idx]
            
            # Radius is the distance along the valve axis
            if dominant_axis_idx == 0:  # X axis
                raio = abs(dx)
            elif dominant_axis_idx == 1:  # Y axis
                raio = abs(dy)
            else:  # Z axis
                raio = abs(dz)
            
            if raio < 0.005:  # Too small (< 5mm)
                self.get_logger().error(f"Radial distance too small: {raio*1000:.1f} mm")
                return False
            
            self.get_logger().info(f"Valve rotation parameters:")
            self.get_logger().info(f"  Center (pivot):     [{pivot_x:.3f}, {pivot_y:.3f}, {pivot_z:.3f}]")
            self.get_logger().info(f"  EE position:        [{ee_x:.3f}, {ee_y:.3f}, {ee_z:.3f}]")
            self.get_logger().info(f"  Valve axis:         {valve_axis_name} (vector: [{axis_vector[0]:.3f}, {axis_vector[1]:.3f}, {axis_vector[2]:.3f}])")
            self.get_logger().info(f"  Radial distance:    {raio:.4f} m ({raio*1000:.1f} mm)")
            
            # Get initial orientation to maintain it
            ee_orientation = tf_body_ee.transform.rotation
            
            # Generate waypoints: circular arc in plane perpendicular to valve axis
            # For Y-axis valve (horizontal): arc in YZ plane, X constant
            waypoints = []
            
            if dominant_axis_idx == 1:  # Y axis (horizontal valve)
                # Movement in YZ plane, X constant
                for k in range(1, steps+1):
                    angle = theta_rad * (k / steps)
                    
                    # Circular arc: start at (pivot_y + raio, ee_z), rotate towards +Z
                    new_x = ee_x  # CONSTANT!
                    new_y = pivot_y + raio * math.cos(angle)
                    new_z = ee_z + raio * math.sin(angle)
                    
                    waypoint = Pose()
                    waypoint.position.x = float(new_x)
                    waypoint.position.y = float(new_y)
                    waypoint.position.z = float(new_z)
                    waypoint.orientation = ee_orientation
                    
                    waypoints.append(waypoint)
                    
            elif dominant_axis_idx == 2:  # Z axis (vertical valve)
                # Movement in XY plane, Z constant
                for k in range(1, steps+1):
                    angle = theta_rad * (k / steps)
                    
                    # Circular arc in XY plane
                    angle_initial = math.atan2(dy, dx)
                    angle_current = angle_initial + angle
                    
                    new_x = pivot_x + raio * math.cos(angle_current)
                    new_y = pivot_y + raio * math.sin(angle_current)
                    new_z = ee_z  # CONSTANT!
                    
                    waypoint = Pose()
                    waypoint.position.x = float(new_x)
                    waypoint.position.y = float(new_y)
                    waypoint.position.z = float(new_z)
                    waypoint.orientation = ee_orientation
                    
                    waypoints.append(waypoint)
                    
            else:  # X axis
                # Movement in XZ plane, Y constant
                for k in range(1, steps+1):
                    angle = theta_rad * (k / steps)
                    
                    new_x = pivot_x + raio * math.cos(angle)
                    new_y = ee_y  # CONSTANT!
                    new_z = ee_z + raio * math.sin(angle)
                    
                    waypoint = Pose()
                    waypoint.position.x = float(new_x)
                    waypoint.position.y = float(new_y)
                    waypoint.position.z = float(new_z)
                    waypoint.orientation = ee_orientation
                    
                    waypoints.append(waypoint)
            
            # Build GetCartesianPath request
            req = GetCartesianPath.Request()
            req.header.frame_id = self.PLANNING_FRAME  # "body" - critical for correct interpretation!
            req.header.stamp = self.get_clock().now().to_msg()
            req.start_state.is_diff = True
            req.group_name = self.PLANNING_GROUP
            req.link_name = self.END_EFFECTOR_LINK
            req.waypoints = waypoints
            req.max_step = float(max_step)         # Interpolation resolution (m)
            req.jump_threshold = 0.0               # Disable jump filtering (or set > 0)
            req.avoid_collisions = bool(avoid_collisions)
            
            self.get_logger().info(f"Requesting cartesian path: {steps} waypoints, "
                                   f"+{theta_deg:.1f}° circular arc in XY plane...")
            
            # Call service
            future = self.cartesian_client.call_async(req)
            res = self.wait_for_future(future, timeout_sec=10.0)
            
            if res is None:
                self.get_logger().error("Timeout calling GetCartesianPath service")
                return False
            
            # Check fraction of path achieved
            frac = getattr(res, 'fraction', 0.0)
            self.get_logger().info(f"Cartesian path fraction achieved: {frac*100:.1f}%")
            
            if frac < 0.9:
                self.get_logger().warn("Cartesian path incomplete (<90%). "
                                       "May fail or stop before completion.")
            
            # Execute the returned trajectory
            if res.solution.joint_trajectory.points:
                exec_goal = ExecuteTrajectory.Goal()
                exec_goal.trajectory = res.solution
                
                self.get_logger().info("Executing valve rotation trajectory...")
                send_exec = self.exec_traj_client.send_goal_async(exec_goal)
                handle = self.wait_for_future(send_exec, timeout_sec=2.0)
                
                if handle is None or not handle.accepted:
                    self.get_logger().error("Failed to accept trajectory execution")
                    return False
                
                result_future = handle.get_result_async()
                result = self.wait_for_future(result_future, timeout_sec=30.0)
                
                if result is None:
                    self.get_logger().error("Timeout executing cartesian trajectory")
                    return False
                
                self.get_logger().info("Valve rotation executed successfully! ✅")
                return True
            else:
                self.get_logger().error("Empty trajectory returned by GetCartesianPath")
                return False
        
        except Exception as e:
            self.get_logger().error(f"rotate_valve failed: {e}", exc_info=True)
            return False
    
    def _generate_rotation_waypoints(
        self,
        start_pose_stamped: PoseStamped,
        pivot_frame: str,
        rotation_axis: str = "z",
        total_angle_rad: float = -math.pi / 2.0,  # -90 deg for closing
        num_steps: int = 10,
    ) -> list:
        """
        Generates a list of Cartesian waypoints for a rotation around a pivot.
        
        Args:
            start_pose_stamped: Starting pose of the end effector
            pivot_frame: Frame to rotate around (e.g., "lever_pivot")
            rotation_axis: Axis to rotate around ('x', 'y', or 'z')
            total_angle_rad: Total rotation angle in radians
            num_steps: Number of waypoints to generate
            
        Returns:
            List of Pose messages representing the waypoints in body frame
        """
        self.get_logger().info(
            f"Generating {num_steps} waypoints for {total_angle_rad:.2f} rad "
            f"rotation around {pivot_frame}..."
        )
        waypoints_in_body_frame = []

        try:
            transform_body_to_pivot = self.lookup_transform(
                pivot_frame, self.PLANNING_FRAME, timeout=2.0
            )
            transform_pivot_to_body = self.lookup_transform(
                self.PLANNING_FRAME, pivot_frame, timeout=2.0
            )
            if not transform_body_to_pivot or not transform_pivot_to_body:
                self.get_logger().error("Failed to get pivot transforms.")
                return None

            start_pose_in_pivot_msg = do_transform_pose(
                start_pose_stamped.pose, transform_body_to_pivot
            )

            for i in range(1, num_steps + 1):
                angle = (total_angle_rad / num_steps) * i
                if rotation_axis == "x":
                    q_rot = tf_transformations.quaternion_from_euler(angle, 0, 0)
                elif rotation_axis == "y":
                    q_rot = tf_transformations.quaternion_from_euler(0, angle, 0)
                else:  # 'z'
                    q_rot = tf_transformations.quaternion_from_euler(0, 0, angle)

                rot_transform_stamped_msg = TransformStamped()
                rot_transform_stamped_msg.transform.translation.x = 0.0
                rot_transform_stamped_msg.transform.translation.y = 0.0
                rot_transform_stamped_msg.transform.translation.z = 0.0
                rot_transform_stamped_msg.transform.rotation.x = q_rot[0]
                rot_transform_stamped_msg.transform.rotation.y = q_rot[1]
                rot_transform_stamped_msg.transform.rotation.z = q_rot[2]
                rot_transform_stamped_msg.transform.rotation.w = q_rot[3]

                waypoint_in_pivot_msg = do_transform_pose(
                    start_pose_in_pivot_msg, rot_transform_stamped_msg
                )
                waypoint_in_body_msg = do_transform_pose(
                    waypoint_in_pivot_msg, transform_pivot_to_body
                )
                waypoints_in_body_frame.append(waypoint_in_body_msg)

            self.get_logger().info(
                f"Successfully generated {len(waypoints_in_body_frame)} waypoints."
            )
            return waypoints_in_body_frame

        except (TransformException, Exception) as e:
            self.get_logger().error(f"Failed to generate waypoints: {e}")
            return None

    def close_valve_cartesian_callback(self, request, response):
        """
        Service callback for closing the valve using a computed Cartesian path.
        
        Uses the stored grasp pose from a previous grasp_valve call and generates
        a Cartesian path that rotates the valve around the pivot point.
        """
        self.get_logger().info("=" * 50)
        self.get_logger().info("CLOSE VALVE (CARTESIAN) SERVICE TRIGGERED")
        self.get_logger().info("=" * 50)

        try:
            if self.last_grasp_pose is None:
                response.success = False
                response.message = "No grasp pose recorded. Run /grasp_valve first."
                self.get_logger().error(response.message)
                return response

            waypoints = self._generate_rotation_waypoints(
                start_pose_stamped=self.last_grasp_pose,
                pivot_frame="lever_pivot",
                rotation_axis="y",
                total_angle_rad=-math.pi / 1.0,  # -90 degrees
                num_steps=15,
            )
            if not waypoints:
                response.success = False
                response.message = "Failed to generate Cartesian waypoints."
                self.get_logger().error(response.message)
                return response

            cartesian_req = GetCartesianPath.Request()
            cartesian_req.header.frame_id = self.PLANNING_FRAME
            cartesian_req.header.stamp = self.get_clock().now().to_msg()
            cartesian_req.start_state.is_diff = True
            cartesian_req.group_name = self.PLANNING_GROUP
            cartesian_req.link_name = self.END_EFFECTOR_LINK
            cartesian_req.waypoints = waypoints
            cartesian_req.max_step = 0.10
            cartesian_req.avoid_collisions = False

            future = self.cartesian_client.call_async(cartesian_req)
            cartesian_result = self.wait_for_future(future, timeout_sec=10.0)
            if cartesian_result is None:
                response.success = False
                response.message = "Cartesian path service timed out."
                self.get_logger().error(response.message)
                return response

            if cartesian_result.fraction < 0.40:
                response.success = False
                response.message = f"Failed to compute full Cartesian path (fraction: {cartesian_result.fraction:.2f})."
                self.get_logger().error(response.message)
                return response

            robot_trajectory = cartesian_result.solution.joint_trajectory
            fjt_goal = FollowJointTrajectory.Goal()
            fjt_goal.trajectory = robot_trajectory
            send_goal_future = self.fjt_client.send_goal_async(fjt_goal)
            goal_handle = self.wait_for_future(send_goal_future, timeout_sec=5.0)
            if goal_handle is None or not goal_handle.accepted:
                response.success = False
                response.message = "Trajectory execution goal was rejected or timed out."
                self.get_logger().error(response.message)
                return response

            result_future = goal_handle.get_result_async()
            fjt_result = self.wait_for_future(result_future, timeout_sec=45.0)
            if fjt_result is None:
                response.success = False
                response.message = "Trajectory execution timed out."
                self.get_logger().error(response.message)
                return response

            if fjt_result.result.error_code != fjt_result.result.SUCCESSFUL:
                response.success = False
                response.message = f"Trajectory execution failed with code: {fjt_result.result.error_code}"
                self.get_logger().error(response.message)
                return response

            response.success = True
            response.message = "Valve closed successfully using Cartesian path!"
            self.get_logger().info("VALVE CLOSED SUCCESSFULLY!")

        except Exception as e:
            response.success = False
            response.message = f"Close valve failed with exception: {str(e)}"
            self.get_logger().error(response.message)
        return response
    
    def plan_grasp_callback(self, request, response):
        """
        Service callback for planning only (no execution, no gripper control).
        Useful for debugging and visualization.
        
        Plans the motion and stores it for later execution via execute_grasp.
        """
        self.get_logger().info("=" * 50)
        self.get_logger().info("PLAN GRASP SERVICE TRIGGERED (planning only)")
        self.get_logger().info("=" * 50)
        
        try:
            # Compute grasp pose from valve detection
            self.get_logger().info("Computing grasp pose from valve detection...")
            grasp_pose = self.compute_grasp_pose_in_planning_frame()
            
            if grasp_pose is None:
                response.success = False
                response.message = "Failed to compute grasp pose. Check valve detection and TF tree."
                self.get_logger().error(response.message)
                return response
            
            # Construct goal constraints
            goal_constraints = self.construct_goal_constraints(grasp_pose)
            
            # Build MoveGroup.Goal
            goal_msg = MoveGroup.Goal()
            goal_msg.request.group_name = self.PLANNING_GROUP
            goal_msg.request.num_planning_attempts = 10
            goal_msg.request.allowed_planning_time = self.PLANNING_TIME
            goal_msg.request.max_velocity_scaling_factor = 0.1
            goal_msg.request.max_acceleration_scaling_factor = 0.1
            goal_msg.request.start_state.is_diff = True
            goal_msg.request.goal_constraints = [goal_constraints]
            
            # PLAN ONLY - don't execute
            goal_msg.planning_options.plan_only = True
            goal_msg.planning_options.planning_scene_diff.is_diff = True
            goal_msg.planning_options.planning_scene_diff.robot_state.is_diff = True
            
            self.get_logger().info("Sending plan-only goal to MoveGroup...")
            send_goal_future = self.move_group_client.send_goal_async(goal_msg)
            
            # Wait for goal acceptance
            goal_handle = self.wait_for_future(send_goal_future, timeout_sec=2.0)
            
            if goal_handle is None:
                response.success = False
                response.message = "Timed out waiting for goal acceptance"
                self.get_logger().error(response.message)
                return response
            
            if not goal_handle.accepted:
                response.success = False
                response.message = "Goal rejected by action server"
                self.get_logger().error(response.message)
                return response
            
            self.get_logger().info("Goal accepted! Planning...")
            
            # Wait for result
            result_future = goal_handle.get_result_async()
            result = self.wait_for_future(result_future, timeout_sec=30.0)
            
            if result is None:
                response.success = False
                response.message = "Planning timed out"
                self.get_logger().error(response.message)
                return response
            
            # Check result
            error_code = result.result.error_code.val
            
            if error_code != result.result.error_code.SUCCESS:
                response.success = False
                response.message = f"Motion planning failed with error code: {error_code}"
                self.get_logger().error(response.message)
                return response
            
            # Store the successful plan for later execution
            self.last_planned_goal = goal_msg
            self.last_grasp_pose = grasp_pose
            
            response.success = True
            response.message = "Motion planned successfully! Use /execute_grasp to execute."
            self.get_logger().info("=" * 50)
            self.get_logger().info("PLANNING SUCCESSFUL!")
            self.get_logger().info("Pose stored. Call /execute_grasp to execute.")
            self.get_logger().info("=" * 50)
            
        except Exception as e:
            response.success = False
            response.message = f"Planning failed with exception: {str(e)}"
            self.get_logger().error(response.message)
            self.get_logger().error(f"Exception details: {e}", exc_info=True)
        
        return response
    
    def execute_grasp_callback(self, request, response):
        """
        Service callback for executing a previously planned motion.
        Opens gripper (pre-grasp) and executes the stored plan.
        Does NOT close gripper - use grasp_valve for complete grasp.
        """
        self.get_logger().info("=" * 50)
        self.get_logger().info("EXECUTE GRASP SERVICE TRIGGERED")
        self.get_logger().info("=" * 50)
        
        try:
            # Check if we have a plan
            if self.last_planned_goal is None:
                response.success = False
                response.message = "No plan available! Call /plan_grasp first."
                self.get_logger().error(response.message)
                return response
            
            # Open gripper to pre-grasp position
            self.get_logger().info("Opening gripper to pre-grasp position...")
            self.open_gripper()
            time.sleep(0.5)
            
            # Re-send goal but now with execution enabled
            goal_msg = self.last_planned_goal
            goal_msg.planning_options.plan_only = False  # Now execute!
            
            self.get_logger().info("Executing planned motion...")
            send_goal_future = self.move_group_client.send_goal_async(goal_msg)
            
            # Wait for goal acceptance
            goal_handle = self.wait_for_future(send_goal_future, timeout_sec=2.0)
            
            if goal_handle is None:
                response.success = False
                response.message = "Timed out waiting for execution goal acceptance"
                self.get_logger().error(response.message)
                return response
            
            if not goal_handle.accepted:
                response.success = False
                response.message = "Execution goal rejected by action server"
                self.get_logger().error(response.message)
                return response
            
            self.get_logger().info("Goal accepted! Executing...")
            
            # Wait for result
            result_future = goal_handle.get_result_async()
            result = self.wait_for_future(result_future, timeout_sec=30.0)
            
            if result is None:
                response.success = False
                response.message = "Execution timed out"
                self.get_logger().error(response.message)
                return response
            
            # Check result
            error_code = result.result.error_code.val
            
            if error_code != result.result.error_code.SUCCESS:
                response.success = False
                response.message = f"Motion execution failed with error code: {error_code}"
                self.get_logger().error(response.message)
                return response
            
            response.success = True
            response.message = "Motion executed successfully! (gripper still open)"
            self.get_logger().info("=" * 50)
            self.get_logger().info("EXECUTION SUCCESSFUL!")
            self.get_logger().info("Gripper is in pre-grasp position (open).")
            self.get_logger().info("=" * 50)
            
        except Exception as e:
            response.success = False
            response.message = f"Execution failed with exception: {str(e)}"
            self.get_logger().error(response.message)
            self.get_logger().error(f"Exception details: {e}", exc_info=True)
        
        return response
    
    def grasp_service_callback(self, request, response):
        """
        Service callback for grasp execution.
        
        Sequence:
        1. Compute grasp pose from valve detection
        2. Plan and execute motion to grasp pose using MoveGroup action
        3. Close gripper
        
        Note: Gripper is already open from startup initialization.
        """
        self.get_logger().info("=" * 50)
        self.get_logger().info("GRASP SERVICE TRIGGERED")
        self.get_logger().info("=" * 50)
        
        try:
            # Step 1: Compute grasp pose
            self.get_logger().info("Step 1: Computing grasp pose from valve detection...")
            grasp_pose = self.compute_grasp_pose_in_planning_frame()
            
            if grasp_pose is None:
                response.success = False
                response.message = "Failed to compute grasp pose. Check valve detection and TF tree."
                self.get_logger().error(response.message)
                return response
            
            # Step 2: Plan and execute motion using MoveGroup action
            self.get_logger().info("Step 2: Planning and executing motion to grasp pose...")
            
            # Construct goal constraints
            goal_constraints = self.construct_goal_constraints(grasp_pose)
            
            # Build MoveGroup.Goal
            goal_msg = MoveGroup.Goal()
            goal_msg.request.group_name = self.PLANNING_GROUP
            goal_msg.request.num_planning_attempts = 10
            goal_msg.request.allowed_planning_time = self.PLANNING_TIME
            goal_msg.request.max_velocity_scaling_factor = 0.1
            goal_msg.request.max_acceleration_scaling_factor = 0.1
            
            # Set start state (empty = current state)
            goal_msg.request.start_state.is_diff = True
            
            # Set goal constraints
            goal_msg.request.goal_constraints = [goal_constraints]
            
            # Plan and execute
            goal_msg.planning_options.plan_only = False  # Plan AND execute
            goal_msg.planning_options.planning_scene_diff.is_diff = True
            goal_msg.planning_options.planning_scene_diff.robot_state.is_diff = True
            
            # Store for debugging
            self.last_planned_goal = goal_msg
            self.last_grasp_pose = grasp_pose
            
            self.get_logger().info("Sending goal to MoveGroup action server...")
            send_goal_future = self.move_group_client.send_goal_async(goal_msg)
            
            # Wait for goal acceptance
            goal_handle = self.wait_for_future(send_goal_future, timeout_sec=2.0)
            
            if goal_handle is None:
                response.success = False
                response.message = "Timed out waiting for goal acceptance"
                self.get_logger().error(response.message)
                return response
            
            if not goal_handle.accepted:
                response.success = False
                response.message = "Goal rejected by action server"
                self.get_logger().error(response.message)
                return response
            
            self.get_logger().info("Goal accepted! Waiting for result...")
            
            # Wait for result
            result_future = goal_handle.get_result_async()
            result = self.wait_for_future(result_future, timeout_sec=30.0)
            
            if result is None:
                response.success = False
                response.message = "Action timed out"
                self.get_logger().error(response.message)
                return response
            
            # Check result
            error_code = result.result.error_code.val
            
            if error_code != result.result.error_code.SUCCESS:
                response.success = False
                response.message = f"Motion planning/execution failed with error code: {error_code}"
                self.get_logger().error(response.message)
                return response
            
            self.get_logger().info("Reached grasp pose successfully!")
            time.sleep(0.1)
            
            # Step 3: Close gripper
            self.get_logger().info("Step 3: Closing gripper to grasp...")
            self.close_gripper()
            time.sleep(0.5)
            
            # Step 4: Rotate the valve (+90° circular arc)
            self.get_logger().info("Step 4: Rotating the valve (+90°)...")
            ok = self.rotate_valve(theta_deg=90.0, steps=128, axis='z', max_step=0.0008, avoid_collisions=False)
            if not ok:
                self.get_logger().warn("Rotation did not complete fully. Check limits/collisions/TF.")
            
            # Success!
            response.success = True
            response.message = "Grasp and valve rotation executed successfully!"
            self.get_logger().info("=" * 50)
            self.get_logger().info("GRASP COMPLETED SUCCESSFULLY!")
            self.get_logger().info("=" * 50)
            
        except Exception as e:
            response.success = False
            response.message = f"Grasp failed with exception: {str(e)}"
            self.get_logger().error(response.message)
            self.get_logger().error(f"Exception details: {e}", exc_info=True)
        
        return response


def main(args=None):
    rclpy.init(args=args)
    
    node = ValveGraspNode()
    
    # Use multi-threaded executor for parallel callbacks
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info("Shutting down Valve Grasp Node...")
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
