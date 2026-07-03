#!/usr/bin/env python3
"""
ROS2 Node for Right Wrist Detection using MediaPipe Pose.

This node subscribes to the ZED camera image and depth topics, detects the right wrist
using MediaPipe Pose (landmark index 16), and visualizes coordinate axes using real depth.

Subscribed Topics:
    /zed/zed_node/rgb/image_rect_color (sensor_msgs/Image): RGB image from ZED camera
    /zed/zed_node/depth/depth_registered (sensor_msgs/Image): Depth image from ZED camera
    /zed/zed_node/depth/camera_info (sensor_msgs/CameraInfo): Camera info from ZED camera

Author: Generated for spot-teleop
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped, TransformStamped, Quaternion
import tf2_ros
from cv_bridge import CvBridge
import cv2
import cv2.aruco as aruco
import mediapipe as mp
import numpy as np
import message_filters


class WristDetector(Node):
    """ROS2 node for detecting and visualizing the right wrist using MediaPipe."""
    
    def __init__(self):
        super().__init__('wrist_detector')
        
        # Initialize MediaPipe Pose
        self.mp_pose = mp.solutions.pose
        self.mp_drawing = mp.solutions.drawing_utils
        self.mp_drawing_styles = mp.solutions.drawing_styles
        
        # Create pose detector with optimized settings for real-time detection
        self.pose = self.mp_pose.Pose(
            static_image_mode=False,
            model_complexity=1,  # 0=Lite, 1=Full, 2=Heavy
            smooth_landmarks=True,
            enable_segmentation=False,
            min_detection_confidence=0.5,
            min_tracking_confidence=0.5
        )
        
        # Initialize CV bridge
        self.bridge = CvBridge()
        
        # Camera info
        self.camera_info = None
        self.fx = None
        self.fy = None
        self.cx = None
        self.cy = None
        
        # Depth image
        self.depth_image = None
        
        # AprilTag detector setup (36h11 family)
        self.aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_APRILTAG_36h11)
        self.aruco_params = aruco.DetectorParameters()
        self.aruco_detector = aruco.ArucoDetector(self.aruco_dict, self.aruco_params)
        
        # Declare parameters
        self.declare_parameter('color_topic', '/zed/zed_node/rgb/image_rect_color')
        self.declare_parameter('depth_topic', '/zed/zed_node/depth/depth_registered')
        self.declare_parameter('camera_info_topic', '/zed/zed_node/depth/camera_info')
        self.declare_parameter('show_all_landmarks', False)
        self.declare_parameter('wrist_circle_radius', 10)
        self.declare_parameter('wrist_circle_color', [0, 255, 0])  # Green in BGR
        self.declare_parameter('apriltag_size', 0.16)  # AprilTag size in meters (default 10cm)
        
        # Get parameters
        color_topic = self.get_parameter('color_topic').value
        depth_topic = self.get_parameter('depth_topic').value
        camera_info_topic = self.get_parameter('camera_info_topic').value
        self.show_all_landmarks = self.get_parameter('show_all_landmarks').value
        self.wrist_radius = self.get_parameter('wrist_circle_radius').value
        color_param = self.get_parameter('wrist_circle_color').value
        self.wrist_color = tuple(color_param)
        self.tag_size = self.get_parameter('apriltag_size').value
        
        # Scale factor to compensate human arm vs Spot arm length
        # Spot arm reach: ~984mm, Human arm (shoulder to wrist): ~650mm
        # Used at startup and as fallback when online estimation is disabled
        # or has not converged yet.
        self.declare_parameter('scale_factor', 984.0 / 650.0)
        self.scale_factor = self.get_parameter('scale_factor').value

        # Online arm-length estimation: estimate the operator's arm length as
        # the segment sum ||shoulder->elbow|| + ||elbow->wrist||, which is
        # pose-invariant (rigid segments), so no calibration pose is needed.
        self.declare_parameter('online_scale_estimation', True)
        self.online_scale_estimation = self.get_parameter('online_scale_estimation').value
        self.declare_parameter('robot_reach', 0.984)  # Spot arm reach in meters
        self.robot_reach = self.get_parameter('robot_reach').value
        
        # Output frame for wrist pose (robot's body frame)
        self.declare_parameter('output_frame', 'body')
        self.output_frame = self.get_parameter('output_frame').value
        
        # Shoulder offset: offset from body to arm_link_sh0 in body frame (REP-103: X=forward, Y=left, Z=up)
        # From URDF: arm_sh0 is at xyz="0.292 0.0 0.188" relative to body
        # Set to [0,0,0] to disable offset, or [0.292, 0.0, 0.188] to reference from shoulder
        self.declare_parameter('shoulder_offset', [0.292, 0.0, 0.188])
        self.shoulder_offset = np.array(self.get_parameter('shoulder_offset').value)
        
        # Publisher for wrist pose in body frame
        self.wrist_pose_pub = self.create_publisher(PoseStamped, '/wrist_pose', 10)
        
        # TF broadcaster for wrist target frame
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        
        # Create synchronized subscribers for color and depth
        self.color_sub = message_filters.Subscriber(self, Image, color_topic)
        self.depth_sub = message_filters.Subscriber(self, Image, depth_topic)
        
        # Synchronize color and depth messages
        self.sync = message_filters.ApproximateTimeSynchronizer(
            [self.color_sub, self.depth_sub],
            queue_size=10,
            slop=0.1
        )
        self.sync.registerCallback(self.synced_callback)
        
        # Camera info subscriber (not synchronized, just need it once)
        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            camera_info_topic,
            self.camera_info_callback,
            10
        )
        
        # Statistics
        self.frame_count = 0
        self.detection_count = 0
        
        # EMA filter parameters
        self.declare_parameter('filter_alpha_axes', 0.15)  # Lower = smoother body frame
        self.declare_parameter('filter_alpha_wrist', 0.2)  # Higher = more responsive wrist
        self.alpha_axes = self.get_parameter('filter_alpha_axes').value
        self.alpha_wrist = self.get_parameter('filter_alpha_wrist').value
        
        # Jump filter parameters (max allowed movement per frame in meters)
        self.declare_parameter('jump_threshold', 0.40)  # Increased to 40cm for more fluid body follow
        self.jump_threshold = self.get_parameter('jump_threshold').value
        
        # Wrist jump filter (higher threshold since wrist moves faster)
        self.declare_parameter('wrist_jump_threshold', 0.40)  # 40cm max jump per frame
        self.wrist_jump_threshold = self.get_parameter('wrist_jump_threshold').value
        
        # Angular jump filter (max allowed rotation per frame in degrees)
        self.declare_parameter('axis_jump_threshold_deg', 30.0)  # Increased to 30 degrees
        self.axis_jump_threshold = np.radians(self.get_parameter('axis_jump_threshold_deg').value)
        
        # Filtered states (EMA)
        self.filtered_origin = None
        self.filtered_axis_x = None
        self.filtered_axis_y = None
        self.filtered_axis_z = None
        self.filtered_wrist_in_body = None
        
        # Previous landmark positions for jump filter
        self.prev_landmarks_3d = {}
        
        # Previous wrist_in_body for jump filter
        self.prev_wrist_in_body = None

        # Online arm-length estimation state
        self.arm_length_samples = []       # rolling window of segment-sum lengths (m)
        self.arm_length_window = 150       # ~5 s at 30 Hz
        self.arm_length_min_samples = 90   # samples required before latching
        self.arm_length_max_spread = 0.03  # IQR convergence threshold (m)
        self.online_scale = None           # latched scale; None until converged
        self.current_r_sh_3d = None        # right shoulder 3D of the current frame
        
        # Previous axes for angular jump filter
        self.prev_axes = {}
        
        # Max plausible velocity for body landmarks (m/s)
        # Shoulders/hips don't move faster than ~1.5 m/s in normal use
        self.declare_parameter('max_landmark_velocity', 1.5)
        self.max_landmark_velocity = self.get_parameter('max_landmark_velocity').value
        
        # Timestamps for velocity-based convergence
        self.prev_landmark_times = {}
        
        # Last valid body frame (used when any landmark is rejected)
        self.last_valid_origin = None
        self.last_valid_R = None  # Rotation matrix for body frame
        
        # Flag to track if any body landmark was rejected this frame
        self.body_landmark_rejected = False
        
        # Hand orientation (roll)
        self.hand_quat = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        self.hand_roll_sub = self.create_subscription(
            Quaternion,
            '/hand_roll_quat',
            self.hand_roll_callback,
            10
        )
        
        self.get_logger().info('=== Wrist Detector Node Started ===')
        self.get_logger().info(f'Subscribing to color topic: {color_topic}')
        self.get_logger().info(f'Subscribing to depth topic: {depth_topic}')
        self.get_logger().info(f'Subscribing to camera info: {camera_info_topic}')
        self.get_logger().info(f'Right wrist landmark index: 16 (MediaPipe Pose)')
        self.get_logger().info(f'Show all landmarks: {self.show_all_landmarks}')
        self.get_logger().info(f'AprilTag size: {self.tag_size} meters')
        self.get_logger().info(f'Output frame for wrist pose: {self.output_frame}')
        self.get_logger().info(f'Filter alpha (axes): {self.alpha_axes}, (wrist): {self.alpha_wrist}')
        self.get_logger().info(f'Jump threshold: {self.jump_threshold*100:.1f} cm/frame')
        self.get_logger().info(f'Max landmark velocity: {self.max_landmark_velocity:.2f} m/s')
        self.get_logger().info(f'Wrist jump threshold: {self.wrist_jump_threshold*100:.1f} cm/frame')
        self.get_logger().info(f'Axis jump threshold: {np.degrees(self.axis_jump_threshold):.1f} deg/frame')
        self.get_logger().info(f'Scale factor (human to Spot arm): {self.scale_factor:.3f}')
        self.get_logger().info(
            f'Online scale estimation: {self.online_scale_estimation} '
            f'(robot reach: {self.robot_reach:.3f} m)')
        self.get_logger().info(f'Shoulder offset (body->sh0): X={self.shoulder_offset[0]:.3f}, Y={self.shoulder_offset[1]:.3f}, Z={self.shoulder_offset[2]:.3f}')
        
        # Store last comparison results for logging
        self.last_position_error = None
        self.last_angle_errors = None

    def hand_roll_callback(self, msg):
        """Update hand orientation (roll) from hand_orientation_estimator."""
        self.hand_quat = msg

    def camera_info_callback(self, msg):
        """Store camera info and extract intrinsics."""
        if self.camera_info is None:
            self.camera_info = msg
            # Extract camera intrinsics from K matrix
            # K = [fx, 0, cx, 0, fy, cy, 0, 0, 1]
            self.fx = msg.k[0]
            self.fy = msg.k[4]
            self.cx = msg.k[2]
            self.cy = msg.k[5]
            self.get_logger().info(f'Received camera info: {msg.width}x{msg.height}')
            self.get_logger().info(f'Intrinsics: fx={self.fx:.2f}, fy={self.fy:.2f}, cx={self.cx:.2f}, cy={self.cy:.2f}')

    def get_depth_at_pixel(self, depth_image, u, v, window_size=5):
        """Get depth at pixel using median of a window to reduce noise."""
        h, w = depth_image.shape[:2]
        half = window_size // 2
        
        # Clamp window to image bounds
        u_min = max(0, u - half)
        u_max = min(w, u + half + 1)
        v_min = max(0, v - half)
        v_max = min(h, v + half + 1)
        
        # Extract window and compute median of valid depths
        window = depth_image[v_min:v_max, u_min:u_max]
        valid_depths = window[(window > 0) & (np.isfinite(window))]
        
        if len(valid_depths) > 0:
            return np.median(valid_depths)
        return None

    def deproject_pixel_to_3d(self, u, v, depth):
        """Convert pixel coordinates + depth to 3D point in camera frame."""
        if self.fx is None or depth is None or depth <= 0:
            return None
        
        X = (u - self.cx) * depth / self.fx
        Y = (v - self.cy) * depth / self.fy
        Z = depth
        
        return np.array([X, Y, Z])

    def project_3d_to_pixel(self, point_3d):
        """Project 3D point back to pixel coordinates."""
        if self.fx is None or point_3d[2] <= 0:
            return None
        
        u = int(self.fx * point_3d[0] / point_3d[2] + self.cx)
        v = int(self.fy * point_3d[1] / point_3d[2] + self.cy)
        
        return (u, v)

    def apply_ema(self, new_value, filtered_value, alpha):
        """Apply Exponential Moving Average filter.
        
        Args:
            new_value: New measurement (numpy array)
            filtered_value: Previous filtered value (numpy array or None)
            alpha: Filter coefficient (0-1). Higher = more responsive, Lower = smoother
        
        Returns:
            Filtered value
        """
        if filtered_value is None:
            return new_value.copy()
        return alpha * new_value + (1 - alpha) * filtered_value

    def apply_jump_filter(self, landmark_name, new_pos):
        """Apply pure rejection jump filter for body landmarks.
        
        Natural movement at detection frequency always produces intermediate
        points within the threshold. If a reading exceeds the threshold,
        it's bad data (occlusion, depth noise) — reject entirely and keep
        the previous valid position. No convergence toward bad readings.
        
        Args:
            landmark_name: Identifier for the landmark (e.g., 'l_shoulder')
            new_pos: New 3D position (numpy array)
        
        Returns:
            Previous valid position if jump detected, otherwise new_pos
        """
        if landmark_name not in self.prev_landmarks_3d:
            self.prev_landmarks_3d[landmark_name] = new_pos.copy()
            return new_pos
        
        prev_pos = self.prev_landmarks_3d[landmark_name]
        delta = new_pos - prev_pos
        distance = np.linalg.norm(delta)
        
        if distance > self.jump_threshold:
            # Bad data — reject entirely, keep previous valid position
            self.get_logger().warn(
                f'JUMP REJECTED on {landmark_name}: {distance*100:.1f}cm — keeping previous'
            )
            self.body_landmark_rejected = True
            return prev_pos
        else:
            # Normal movement — accept and update baseline
            self.prev_landmarks_3d[landmark_name] = new_pos.copy()
            return new_pos

    def update_arm_length(self, sh_3d, el_3d, wr_3d):
        """Accumulate arm-length samples and latch the scale on convergence.

        The upper-arm and forearm segment lengths are pose-invariant, so
        ||elbow - shoulder|| + ||wrist - elbow|| estimates the operator's
        arm length at any flexion, with no calibration pose. The median of
        a rolling window rejects depth outliers; once enough samples agree,
        the scale is latched so the hand-to-robot mapping does not keep
        drifting during operation.

        Args:
            sh_3d: Right shoulder 3D position, camera frame (numpy array)
            el_3d: Right elbow 3D position, camera frame (numpy array)
            wr_3d: Right wrist 3D position, camera frame (numpy array)
        """
        length = np.linalg.norm(el_3d - sh_3d) + np.linalg.norm(wr_3d - el_3d)
        if not (0.3 < length < 1.2):
            return  # implausible arm length — depth artifact

        self.arm_length_samples.append(length)
        if len(self.arm_length_samples) > self.arm_length_window:
            self.arm_length_samples.pop(0)

        if len(self.arm_length_samples) < self.arm_length_min_samples:
            return

        q1, median, q3 = np.percentile(self.arm_length_samples, [25, 50, 75])
        if (q3 - q1) < self.arm_length_max_spread:
            self.online_scale = self.robot_reach / median
            self.get_logger().info(
                f'Arm length converged: {median*100:.1f} cm '
                f'(IQR {(q3-q1)*100:.1f} cm, {len(self.arm_length_samples)} samples) '
                f'-> scale latched at {self.online_scale:.3f}')
        elif self.frame_count % 150 == 0:
            self.get_logger().info(
                f'Estimating arm length: {len(self.arm_length_samples)} samples, '
                f'median {median*100:.1f} cm, IQR {(q3-q1)*100:.1f} cm '
                f'(need < {self.arm_length_max_spread*100:.1f} cm)')

    def apply_axis_jump_filter(self, axis_name, new_axis):
        """Apply angular jump filter to limit sudden axis rotations.
        
        If the axis rotates more than axis_jump_threshold, clamp the rotation
        using linear interpolation towards the new direction.
        
        Args:
            axis_name: Identifier for the axis (e.g., 'axis_x')
            new_axis: New unit axis vector (numpy array)
        
        Returns:
            Filtered axis (clamped if angular jump detected)
        """
        if axis_name not in self.prev_axes:
            self.prev_axes[axis_name] = new_axis.copy()
            return new_axis
        
        prev_axis = self.prev_axes[axis_name]
        
        # Calculate angle between axes
        cos_angle = np.clip(np.dot(new_axis, prev_axis), -1.0, 1.0)
        angle = np.arccos(cos_angle)  # in radians
        
        if angle > self.axis_jump_threshold:
            # REJECT the new axis - keep previous (don't interpolate towards bad value!)
            self.get_logger().warn(
                f'AXIS JUMP REJECTED on {axis_name}: {np.degrees(angle):.1f}° (threshold: {np.degrees(self.axis_jump_threshold):.1f}°) - keeping previous'
            )
            # Don't update prev_axes - keep the old good value
            return prev_axis
        else:
            # Accept new axis and update previous
            self.prev_axes[axis_name] = new_axis.copy()
            return new_axis

    def synced_callback(self, color_msg, depth_msg):
        """Process synchronized color and depth images with body frame persistence."""
        try:
            # Convert ROS Images to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(color_msg, "bgr8")
            
            # Handle different depth encodings
            if depth_msg.encoding == '32FC1':
                depth_image = self.bridge.imgmsg_to_cv2(depth_msg, "32FC1")
            elif depth_msg.encoding == '16UC1':
                depth_image = self.bridge.imgmsg_to_cv2(depth_msg, "16UC1")
                depth_image = depth_image.astype(np.float32) / 1000.0
            else:
                depth_image = self.bridge.imgmsg_to_cv2(depth_msg, "passthrough")
                if depth_image.dtype == np.uint16:
                    depth_image = depth_image.astype(np.float32) / 1000.0
            
            # Convert BGR to RGB for MediaPipe
            rgb_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
            results = self.pose.process(rgb_image)
            display_image = cv_image.copy()
            height, width, _ = cv_image.shape
            
            # Reset rejection flag for this frame
            self.body_landmark_rejected = False
            
            if results.pose_landmarks:
                self.detection_count += 1
                landmarks = results.pose_landmarks.landmark

                # Shoulder of the current frame only (for arm-length estimation);
                # never pair a stale shoulder with the current wrist
                self.current_r_sh_3d = None

                # Draw landmarks if requested
                if self.show_all_landmarks:
                    self.mp_drawing.draw_landmarks(
                        display_image, results.pose_landmarks, self.mp_pose.POSE_CONNECTIONS,
                        landmark_drawing_spec=self.mp_drawing_styles.get_default_pose_landmarks_style())
                
                # 1. Update Body Frame with Hierarchical Fallback
                l_shoulder, r_shoulder = landmarks[11], landmarks[12]
                l_hip, r_hip = landmarks[23], landmarks[24]
                l_ankle, r_ankle = landmarks[27], landmarks[28]
                
                # Check visibility for different levels
                vis_sh = l_shoulder.visibility > 0.5 and r_shoulder.visibility > 0.5
                vis_hp = l_hip.visibility > 0.5 and r_hip.visibility > 0.5
                vis_ak = l_ankle.visibility > 0.5 and r_ankle.visibility > 0.5
                
                if vis_sh and self.fx is not None:
                    # Basic points for all levels
                    sh_l_px = (int(l_shoulder.x * width), int(l_shoulder.y * height))
                    sh_r_px = (int(r_shoulder.x * width), int(r_shoulder.y * height))
                    d_sh_l = self.get_depth_at_pixel(depth_image, sh_l_px[0], sh_l_px[1])
                    d_sh_r = self.get_depth_at_pixel(depth_image, sh_r_px[0], sh_r_px[1])
                    
                    if d_sh_l and d_sh_r:
                        l_sh_3d = self.apply_jump_filter('l_sh', self.deproject_pixel_to_3d(sh_l_px[0], sh_l_px[1], d_sh_l))
                        r_sh_3d = self.apply_jump_filter('r_sh', self.deproject_pixel_to_3d(sh_r_px[0], sh_r_px[1], d_sh_r))
                        self.current_r_sh_3d = r_sh_3d

                        # Initialize vectors
                        up_vec = np.array([0.0, -1.0, 0.0]) # Default up (camera frame)
                        torso_center = (l_sh_3d + r_sh_3d) / 2
                        
                        best_level = "None"
                        
                        # --- LEVEL 1: Full Body (Ankles) ---
                        if vis_hp and vis_ak:
                            hp_l_px = (int(l_hip.x * width), int(l_hip.y * height))
                            hp_r_px = (int(r_hip.x * width), int(r_hip.y * height))
                            ak_l_px = (int(l_ankle.x * width), int(l_ankle.y * height))
                            ak_r_px = (int(r_ankle.x * width), int(r_ankle.y * height))
                            d_hp_l = self.get_depth_at_pixel(depth_image, hp_l_px[0], hp_l_px[1])
                            d_hp_r = self.get_depth_at_pixel(depth_image, hp_r_px[0], hp_r_px[1])
                            d_ak_l = self.get_depth_at_pixel(depth_image, ak_l_px[0], ak_l_px[1])
                            d_ak_r = self.get_depth_at_pixel(depth_image, ak_r_px[0], ak_r_px[1])
                            
                            if all([d_hp_l, d_hp_r, d_ak_l, d_ak_r]):
                                l_hp_3d = self.apply_jump_filter('l_hp', self.deproject_pixel_to_3d(hp_l_px[0], hp_l_px[1], d_hp_l))
                                r_hp_3d = self.apply_jump_filter('r_hp', self.deproject_pixel_to_3d(hp_r_px[0], hp_r_px[1], d_hp_r))
                                l_ak_3d = self.apply_jump_filter('l_ak', self.deproject_pixel_to_3d(ak_l_px[0], ak_l_px[1], d_ak_l))
                                r_ak_3d = self.apply_jump_filter('r_ak', self.deproject_pixel_to_3d(ak_r_px[0], ak_r_px[1], d_ak_r))
                                
                                torso_center = (l_sh_3d + r_sh_3d + l_hp_3d + r_hp_3d) / 4
                                up_vec = (l_sh_3d + r_sh_3d)/2 - (l_ak_3d + r_ak_3d)/2
                                best_level = "FULL_BODY"
                        
                        # --- LEVEL 2: Torso Only (Hips) ---
                        if best_level == "None" and vis_hp:
                            hp_l_px = (int(l_hip.x * width), int(l_hip.y * height))
                            hp_r_px = (int(r_hip.x * width), int(r_hip.y * height))
                            d_hp_l = self.get_depth_at_pixel(depth_image, hp_l_px[0], hp_l_px[1])
                            d_hp_r = self.get_depth_at_pixel(depth_image, hp_r_px[0], hp_r_px[1])
                            
                            if d_hp_l and d_hp_r:
                                l_hp_3d = self.apply_jump_filter('l_hp', self.deproject_pixel_to_3d(hp_l_px[0], hp_l_px[1], d_hp_l))
                                r_hp_3d = self.apply_jump_filter('r_hp', self.deproject_pixel_to_3d(hp_r_px[0], hp_r_px[1], d_hp_r))
                                torso_center = (l_sh_3d + r_sh_3d + l_hp_3d + r_hp_3d) / 4
                                up_vec = (l_sh_3d + r_sh_3d)/2 - (l_hp_3d + r_hp_3d)/2
                                best_level = "TORSO"
                                
                        # --- LEVEL 3: Shoulders Only ---
                        if best_level == "None":
                            best_level = "SHOULDERS_ONLY"
                            # up_vec is already set to camera default [0, -1, 0]
                        
                        # Common Frame Calculation
                        axis_x = (r_sh_3d - l_sh_3d) / (np.linalg.norm(r_sh_3d - l_sh_3d) + 1e-6)
                        up_vec /= (np.linalg.norm(up_vec) + 1e-6)
                        axis_z = -np.cross(axis_x, up_vec)
                        axis_z /= (np.linalg.norm(axis_z) + 1e-6)
                        axis_y = np.cross(axis_x, axis_z)
                        
                        # Origin aligned with right shoulder
                        R_temp = np.column_stack([axis_z, -axis_x, axis_y])
                        r_sh_in_body = R_temp.T @ (r_sh_3d - torso_center)
                        origin_3d = torso_center + R_temp @ np.array([0.0, r_sh_in_body[1], 0.0])
                        
                        # Filter and store ONLY if no landmark jumped
                        # This keeps the axis frame consistent (no weird twists from partial updates)
                        if not self.body_landmark_rejected:
                            self.filtered_origin = self.apply_ema(origin_3d, self.filtered_origin, self.alpha_axes)
                            self.filtered_axis_x = self.apply_ema(axis_x, self.filtered_axis_x, self.alpha_axes)
                            self.filtered_axis_y = self.apply_ema(axis_y, self.filtered_axis_y, self.alpha_axes)
                            self.filtered_axis_z = self.apply_ema(axis_z, self.filtered_axis_z, self.alpha_axes)
                            
                            self.last_valid_origin = self.filtered_origin.copy()
                            self.last_valid_R = np.column_stack([self.filtered_axis_z, -self.filtered_axis_x, self.filtered_axis_y])
                            
                            if self.frame_count % 60 == 0:
                                self.get_logger().info(f'Body tracking ACTIVE (Level: {best_level})')
                        else:
                            if self.frame_count % 30 == 0:
                                self.get_logger().warn('Body jump detected - FREEZING axes update for this frame.')
                
                # 2. Process Wrist independently using last valid body frame
                right_wrist = landmarks[self.mp_pose.PoseLandmark.RIGHT_WRIST.value]
                if right_wrist.visibility > 0.5 and self.last_valid_origin is not None:
                    w_x, w_y = int(right_wrist.x * width), int(right_wrist.y * height)
                    cv2.circle(display_image, (w_x, w_y), self.wrist_radius, self.wrist_color, -1)
                    
                    w_depth = self.get_depth_at_pixel(depth_image, w_x, w_y)
                    if w_depth is not None and 0.1 < w_depth < 10.0:
                        w_3d_cam = self.deproject_pixel_to_3d(w_x, w_y, w_depth)
                        if w_3d_cam is not None:
                            # Online arm-length estimation: needs shoulder, elbow,
                            # and wrist from the same frame. The elbow is not
                            # jump-filtered (that would freeze the body frame on
                            # elbow noise); the median window rejects outliers.
                            if (self.online_scale_estimation and self.online_scale is None
                                    and self.current_r_sh_3d is not None):
                                r_elbow = landmarks[self.mp_pose.PoseLandmark.RIGHT_ELBOW.value]
                                if r_elbow.visibility > 0.5:
                                    e_x, e_y = int(r_elbow.x * width), int(r_elbow.y * height)
                                    e_depth = self.get_depth_at_pixel(depth_image, e_x, e_y)
                                    if e_depth is not None and 0.1 < e_depth < 10.0:
                                        e_3d_cam = self.deproject_pixel_to_3d(e_x, e_y, e_depth)
                                        if e_3d_cam is not None:
                                            self.update_arm_length(
                                                self.current_r_sh_3d, e_3d_cam, w_3d_cam)

                            w_in_body_raw = self.last_valid_R.T @ (w_3d_cam - self.last_valid_origin)
                            
                            # Jump filter: reject jumped values from EMA entirely
                            wrist_jumped = False
                            if self.prev_wrist_in_body is not None:
                                wrist_delta = np.linalg.norm(w_in_body_raw - self.prev_wrist_in_body)
                                if wrist_delta >= self.wrist_jump_threshold:
                                    wrist_jumped = True
                                    if self.frame_count % 15 == 0:
                                        self.get_logger().warn(
                                            f'Wrist jump rejected: {wrist_delta*100:.1f}cm — keeping filtered')
                                else:
                                    self.prev_wrist_in_body = w_in_body_raw.copy()
                            else:
                                self.prev_wrist_in_body = w_in_body_raw.copy()
                            
                            if not wrist_jumped:
                                self.filtered_wrist_in_body = self.apply_ema(
                                    w_in_body_raw, self.filtered_wrist_in_body, self.alpha_wrist)
                
                # 3. Visualization and Logging
                if self.filtered_origin is not None:
                    o_px = self.project_3d_to_pixel(self.filtered_origin)
                    if o_px:
                        for axis, color, label in [(self.filtered_axis_x, (0,0,255), 'X'), (self.filtered_axis_y, (0,255,0), 'Y'), (self.filtered_axis_z, (255,0,0), 'Z')]:
                            e_px = self.project_3d_to_pixel(self.filtered_origin + axis * 0.3)
                            if e_px: 
                                cv2.arrowedLine(display_image, o_px, e_px, color, 3)
                                cv2.putText(display_image, label, (e_px[0]+5, e_px[1]), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)
                
            # ALWAYS publish last valid wrist (persists through occlusion and jumps)
            if self.filtered_wrist_in_body is not None:
                scale = self.online_scale if self.online_scale is not None else self.scale_factor
                wrist_final = self.filtered_wrist_in_body * scale + self.shoulder_offset
                
                stamp = self.get_clock().now().to_msg()
                
                pose_msg = PoseStamped()
                pose_msg.header.stamp = stamp
                pose_msg.header.frame_id = self.output_frame
                pose_msg.pose.position.x, pose_msg.pose.position.y, pose_msg.pose.position.z = wrist_final
                pose_msg.pose.orientation = self.hand_quat
                self.wrist_pose_pub.publish(pose_msg)
                
                tf_msg = TransformStamped()
                tf_msg.header.stamp = stamp
                tf_msg.header.frame_id = self.output_frame
                tf_msg.child_frame_id = "wrist_target"
                tf_msg.transform.translation.x, tf_msg.transform.translation.y, tf_msg.transform.translation.z = wrist_final
                tf_msg.transform.rotation.x, tf_msg.transform.rotation.y, tf_msg.transform.rotation.z, tf_msg.transform.rotation.w = \
                    self.hand_quat.x, self.hand_quat.y, self.hand_quat.z, self.hand_quat.w
                self.tf_broadcaster.sendTransform(tf_msg)
                
                if self.frame_count % 30 == 0:
                    self.get_logger().info(f'Wrist in Body: {self.filtered_wrist_in_body}')
            
            self.frame_count += 1
            cv2.imshow('Right Wrist Detection', display_image)
            cv2.waitKey(1)
            
        except Exception as e:
            self.get_logger().error(f'Error processing image: {str(e)}')

    def destroy_node(self):
        """Clean up resources when node is destroyed."""
        self.get_logger().info('Shutting down wrist detector...')
        cv2.destroyAllWindows()
        self.pose.close()
        super().destroy_node()


def main(args=None):
    """Main entry point for the wrist detector node."""
    rclpy.init(args=args)
    node = WristDetector()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
