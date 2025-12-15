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
from geometry_msgs.msg import PoseStamped, TransformStamped
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
        
        # Output frame for wrist pose (robot's body frame)
        self.declare_parameter('output_frame', 'body')
        self.output_frame = self.get_parameter('output_frame').value
        
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
        self.declare_parameter('filter_alpha_axes', 0.3)  # Lower = smoother body frame
        self.declare_parameter('filter_alpha_wrist', 0.5)  # Higher = more responsive wrist
        self.alpha_axes = self.get_parameter('filter_alpha_axes').value
        self.alpha_wrist = self.get_parameter('filter_alpha_wrist').value
        
        # Jump filter parameters (max allowed movement per frame in meters)
        self.declare_parameter('jump_threshold', 0.05)  # 5cm max jump per frame
        self.jump_threshold = self.get_parameter('jump_threshold').value
        
        # Wrist jump filter (higher threshold since wrist moves faster)
        self.declare_parameter('wrist_jump_threshold', 0.15)  # 15cm max jump per frame
        self.wrist_jump_threshold = self.get_parameter('wrist_jump_threshold').value
        
        # Angular jump filter (max allowed rotation per frame in degrees)
        self.declare_parameter('axis_jump_threshold_deg', 5.0)  # 5 degrees max per frame
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
        
        # Previous axes for angular jump filter
        self.prev_axes = {}
        
        # Last valid body frame (used when any landmark is rejected)
        self.last_valid_origin = None
        self.last_valid_R = None  # Rotation matrix for body frame
        
        # Flag to track if any body landmark was rejected this frame
        self.body_landmark_rejected = False
        
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
        self.get_logger().info(f'Wrist jump threshold: {self.wrist_jump_threshold*100:.1f} cm/frame')
        self.get_logger().info(f'Axis jump threshold: {np.degrees(self.axis_jump_threshold):.1f} deg/frame')
        
        # Store last comparison results for logging
        self.last_position_error = None
        self.last_angle_errors = None

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
        """Apply jump filter to limit sudden position changes.
        
        If the landmark moves more than jump_threshold, clamp the movement
        to prevent sudden jumps (e.g., when MediaPipe pulls shoulders with arm).
        
        Args:
            landmark_name: Identifier for the landmark (e.g., 'l_shoulder')
            new_pos: New 3D position (numpy array)
        
        Returns:
            Filtered position (clamped if jump detected)
        """
        if landmark_name not in self.prev_landmarks_3d:
            self.prev_landmarks_3d[landmark_name] = new_pos.copy()
            return new_pos
        
        prev_pos = self.prev_landmarks_3d[landmark_name]
        delta = new_pos - prev_pos
        distance = np.linalg.norm(delta)
        
        if distance > self.jump_threshold:
            # REJECT the new value - keep previous position (don't interpolate towards bad value!)
            self.get_logger().warn(
                f'JUMP REJECTED on {landmark_name}: {distance*100:.1f}cm (threshold: {self.jump_threshold*100:.1f}cm) - keeping previous'
            )
            # Set flag so we use the previous complete body frame
            self.body_landmark_rejected = True
            # Don't update prev_landmarks_3d - keep the old good value
            return prev_pos
        else:
            # Accept new value and update previous
            self.prev_landmarks_3d[landmark_name] = new_pos.copy()
            return new_pos

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
        """Process synchronized color and depth images."""
        try:
            # Convert ROS Images to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(color_msg, "bgr8")
            
            # Handle different depth encodings
            if depth_msg.encoding == '32FC1':
                depth_image = self.bridge.imgmsg_to_cv2(depth_msg, "32FC1")
            elif depth_msg.encoding == '16UC1':
                depth_image = self.bridge.imgmsg_to_cv2(depth_msg, "16UC1")
                depth_image = depth_image.astype(np.float32) / 1000.0  # Convert mm to meters
            else:
                depth_image = self.bridge.imgmsg_to_cv2(depth_msg, "passthrough")
                if depth_image.dtype == np.uint16:
                    depth_image = depth_image.astype(np.float32) / 1000.0
            
            # Convert BGR to RGB for MediaPipe
            rgb_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
            
            # Process the image with MediaPipe Pose
            results = self.pose.process(rgb_image)
            
            # Create a copy for visualization
            display_image = cv_image.copy()
            
            # Check if pose landmarks were detected
            if results.pose_landmarks:
                self.detection_count += 1
                
                # Get image dimensions
                height, width, _ = cv_image.shape
                
                # Draw all pose landmarks if requested
                if self.show_all_landmarks:
                    self.mp_drawing.draw_landmarks(
                        display_image,
                        results.pose_landmarks,
                        self.mp_pose.POSE_CONNECTIONS,
                        landmark_drawing_spec=self.mp_drawing_styles.get_default_pose_landmarks_style()
                    )
                
                # Extract right wrist landmark (index 16)
                landmarks = results.pose_landmarks.landmark
                right_wrist = landmarks[self.mp_pose.PoseLandmark.RIGHT_WRIST.value]
                
                # Check if the landmark is visible (visibility > 0.5)
                if right_wrist.visibility > 0.5:
                    # Convert normalized coordinates to pixel coordinates
                    wrist_x = int(right_wrist.x * width)
                    wrist_y = int(right_wrist.y * height)
                    
                    # Draw a circle at the wrist position
                    cv2.circle(
                        display_image,
                        (wrist_x, wrist_y),
                        self.wrist_radius,
                        self.wrist_color,
                        -1  # Filled circle
                    )
                    
                    # Draw a larger outline circle
                    cv2.circle(
                        display_image,
                        (wrist_x, wrist_y),
                        self.wrist_radius + 3,
                        (255, 255, 255),  # White outline
                        2
                    )
                    
                    # Log detection periodically
                    if self.frame_count % 30 == 0:
                        self.get_logger().info(
                            f'Right wrist detected at ({wrist_x}, {wrist_y}) - '
                            f'Visibility: {right_wrist.visibility:.2f}'
                        )
                else:
                    # Wrist not visible enough
                    cv2.putText(
                        display_image,
                        'Right wrist not visible',
                        (10, 60),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.7,
                        (0, 0, 255),
                        2
                    )
                
                # Calculate and draw body origin (centroid of shoulders, hips, and ankles)
                l_shoulder = landmarks[11]
                r_shoulder = landmarks[12]
                l_hip = landmarks[23]
                r_hip = landmarks[24]
                l_ankle = landmarks[27]
                r_ankle = landmarks[28]
                
                # Only draw origin if all 6 landmarks are visible and camera intrinsics are available
                if (l_shoulder.visibility > 0.5 and r_shoulder.visibility > 0.5 and
                    l_hip.visibility > 0.5 and r_hip.visibility > 0.5 and
                    l_ankle.visibility > 0.5 and r_ankle.visibility > 0.5 and self.fx is not None):
                    
                    # Convert landmarks to pixel coordinates
                    l_sh_px = (int(l_shoulder.x * width), int(l_shoulder.y * height))
                    r_sh_px = (int(r_shoulder.x * width), int(r_shoulder.y * height))
                    l_hp_px = (int(l_hip.x * width), int(l_hip.y * height))
                    r_hp_px = (int(r_hip.x * width), int(r_hip.y * height))
                    l_ak_px = (int(l_ankle.x * width), int(l_ankle.y * height))
                    r_ak_px = (int(r_ankle.x * width), int(r_ankle.y * height))
                    
                    # Get depth for each landmark using median filter
                    l_sh_depth = self.get_depth_at_pixel(depth_image, l_sh_px[0], l_sh_px[1])
                    r_sh_depth = self.get_depth_at_pixel(depth_image, r_sh_px[0], r_sh_px[1])
                    l_hp_depth = self.get_depth_at_pixel(depth_image, l_hp_px[0], l_hp_px[1])
                    r_hp_depth = self.get_depth_at_pixel(depth_image, r_hp_px[0], r_hp_px[1])
                    l_ak_depth = self.get_depth_at_pixel(depth_image, l_ak_px[0], l_ak_px[1])
                    r_ak_depth = self.get_depth_at_pixel(depth_image, r_ak_px[0], r_ak_px[1])
                    
                    # Check if all depths are valid
                    all_depths = [l_sh_depth, r_sh_depth, l_hp_depth, r_hp_depth, l_ak_depth, r_ak_depth]
                    if all(d is not None and d > 0.1 and d < 10.0 for d in all_depths):
                        
                        # Reset body landmark rejection flag for this frame
                        self.body_landmark_rejected = False
                        
                        # Deproject pixels to 3D points in camera frame
                        l_sh_3d_raw = self.deproject_pixel_to_3d(l_sh_px[0], l_sh_px[1], l_sh_depth)
                        r_sh_3d_raw = self.deproject_pixel_to_3d(r_sh_px[0], r_sh_px[1], r_sh_depth)
                        l_hp_3d_raw = self.deproject_pixel_to_3d(l_hp_px[0], l_hp_px[1], l_hp_depth)
                        r_hp_3d_raw = self.deproject_pixel_to_3d(r_hp_px[0], r_hp_px[1], r_hp_depth)
                        l_ak_3d_raw = self.deproject_pixel_to_3d(l_ak_px[0], l_ak_px[1], l_ak_depth)
                        r_ak_3d_raw = self.deproject_pixel_to_3d(r_ak_px[0], r_ak_px[1], r_ak_depth)
                        
                        # Apply jump filter to body landmarks (not wrist - it needs to move freely)
                        l_sh_3d = self.apply_jump_filter('l_shoulder', l_sh_3d_raw)
                        r_sh_3d = self.apply_jump_filter('r_shoulder', r_sh_3d_raw)
                        l_hp_3d = self.apply_jump_filter('l_hip', l_hp_3d_raw)
                        r_hp_3d = self.apply_jump_filter('r_hip', r_hp_3d_raw)
                        l_ak_3d = self.apply_jump_filter('l_ankle', l_ak_3d_raw)
                        r_ak_3d = self.apply_jump_filter('r_ankle', r_ak_3d_raw)
                        
                        # Calculate origin as center of torso (average of shoulders and hips)
                        torso_center = (l_sh_3d + r_sh_3d + l_hp_3d + r_hp_3d) / 4
                        
                        # Calculate torso center pixel for visibility check
                        torso_center_px = self.project_3d_to_pixel(torso_center)
                        
                        if torso_center_px is not None:
                            # ========== CALCULATE COORDINATE AXES IN 3D ==========
                            # Axis X (Lateral): Vector from left shoulder to right shoulder
                            axis_x = r_sh_3d - l_sh_3d
                            axis_x = axis_x / (np.linalg.norm(axis_x) + 1e-6)  # Normalize
                            
                            # Preliminary Up vector: from ankles center to shoulders center
                            # Using ankles gives a longer, more stable vector
                            shoulders_center = (l_sh_3d + r_sh_3d) / 2
                            ankles_center = (l_ak_3d + r_ak_3d) / 2
                            up_preliminary = shoulders_center - ankles_center
                            up_preliminary = up_preliminary / (np.linalg.norm(up_preliminary) + 1e-6)
                            
                            # Axis Z (Frontal): Normal to the plane (cross product of X and up)
                            # Points forward (out of the body, towards camera)
                            # Note: We negate because cross(right, up) points backwards
                            axis_z = -np.cross(axis_x, up_preliminary)
                            axis_z = axis_z / (np.linalg.norm(axis_z) + 1e-6)  # Normalize
                            
                            # Axis Y (Vertical): Cross product of Z and X (ensures orthogonality)
                            axis_y = np.cross(axis_z, axis_x)
                            axis_y = axis_y / (np.linalg.norm(axis_y) + 1e-6)  # Normalize
                            
                            # ========== CALCULATE ORIGIN ALIGNED WITH RIGHT SHOULDER ==========
                            # Build temporary rotation matrix to transform to body frame
                            R_temp = np.column_stack([axis_z, -axis_x, axis_y])
                            
                            # Transform right shoulder to body frame (relative to torso center)
                            r_sh_in_body = R_temp.T @ (r_sh_3d - torso_center)
                            
                            # New origin in body frame: Y from shoulder (lateral alignment), X and Z = 0 (centered)
                            # X = forward, Y = left, Z = up
                            # We want to move origin laterally to align with right shoulder
                            new_origin_offset_body = np.array([0.0, r_sh_in_body[1], 0.0])
                            
                            # Convert offset back to camera frame and add to torso center
                            origin_3d = torso_center + R_temp @ new_origin_offset_body
                            
                            # ========== APPLY ANGULAR JUMP FILTER TO AXES ==========
                            axis_x = self.apply_axis_jump_filter('axis_x', axis_x)
                            axis_y = self.apply_axis_jump_filter('axis_y', axis_y)
                            axis_z = self.apply_axis_jump_filter('axis_z', axis_z)
                            
                            # ========== APPLY EMA FILTER TO BODY FRAME ==========
                            self.filtered_origin = self.apply_ema(origin_3d, self.filtered_origin, self.alpha_axes)
                            self.filtered_axis_x = self.apply_ema(axis_x, self.filtered_axis_x, self.alpha_axes)
                            self.filtered_axis_y = self.apply_ema(axis_y, self.filtered_axis_y, self.alpha_axes)
                            self.filtered_axis_z = self.apply_ema(axis_z, self.filtered_axis_z, self.alpha_axes)
                            
                            # Re-normalize filtered axes (EMA can break unit length)
                            self.filtered_axis_x = self.filtered_axis_x / (np.linalg.norm(self.filtered_axis_x) + 1e-6)
                            self.filtered_axis_y = self.filtered_axis_y / (np.linalg.norm(self.filtered_axis_y) + 1e-6)
                            self.filtered_axis_z = self.filtered_axis_z / (np.linalg.norm(self.filtered_axis_z) + 1e-6)
                            
                            # Use filtered values for visualization and calculations
                            origin_3d = self.filtered_origin
                            axis_x = self.filtered_axis_x
                            axis_y = self.filtered_axis_y
                            axis_z = self.filtered_axis_z
                            
                            # Calculate origin pixel for visualization (after filtering)
                            origin_px = self.project_3d_to_pixel(origin_3d)
                            
                            # Build rotation matrix for body frame (REP-103 convention)
                            # REP-103: X=forward, Y=left, Z=up
                            # Our axes: axis_x=right, axis_y=up, axis_z=forward
                            # So: X_rep103=axis_z, Y_rep103=-axis_x, Z_rep103=axis_y
                            R_human_rep103 = np.column_stack([axis_z, -axis_x, axis_y])
                            
                            # ========== HANDLE BODY FRAME CONSISTENCY ==========
                            # If any body landmark was rejected, use the PREVIOUS complete body frame
                            # This prevents using inconsistent landmarks from different frames
                            if self.body_landmark_rejected and self.last_valid_origin is not None:
                                self.get_logger().warn('Using previous body frame due to landmark rejection')
                                origin_for_wrist = self.last_valid_origin
                                R_for_wrist = self.last_valid_R
                            else:
                                # All landmarks accepted - update the last valid body frame
                                origin_for_wrist = origin_3d
                                R_for_wrist = R_human_rep103
                                self.last_valid_origin = origin_3d.copy()
                                self.last_valid_R = R_human_rep103.copy()
                            
                            # Scale factor for axis length in meters
                            axis_length_m = 0.3  # 30 cm axes
                            
                            # ========== CALCULATE WRIST IN BODY FRAME (REP-103) ==========
                            # Get wrist 3D position in camera frame
                            wrist_depth = self.get_depth_at_pixel(depth_image, wrist_x, wrist_y)
                            wrist_3d = None
                            wrist_in_body = None
                            
                            if wrist_depth is not None and 0.1 < wrist_depth < 10.0:
                                wrist_3d = self.deproject_pixel_to_3d(wrist_x, wrist_y, wrist_depth)
                                
                                if wrist_3d is not None:
                                    # Transform wrist to body frame using CONSISTENT body frame
                                    # wrist_in_body = R^T @ (wrist_camera - origin_camera)
                                    wrist_in_body_raw = R_for_wrist.T @ (wrist_3d - origin_for_wrist)
                                    
                                    # ========== APPLY JUMP FILTER TO WRIST_IN_BODY ==========
                                    # Reject wrist if it jumps too much (protects against bad body frame)
                                    if self.prev_wrist_in_body is not None:
                                        wrist_delta = np.linalg.norm(wrist_in_body_raw - self.prev_wrist_in_body)
                                        if wrist_delta > self.wrist_jump_threshold:
                                            self.get_logger().warn(
                                                f'WRIST JUMP REJECTED: {wrist_delta*100:.1f}cm (threshold: {self.wrist_jump_threshold*100:.1f}cm) - keeping previous'
                                            )
                                            wrist_in_body_raw = self.prev_wrist_in_body.copy()
                                        else:
                                            self.prev_wrist_in_body = wrist_in_body_raw.copy()
                                    else:
                                        self.prev_wrist_in_body = wrist_in_body_raw.copy()
                                    
                                    # Apply EMA filter to wrist position
                                    self.filtered_wrist_in_body = self.apply_ema(
                                        wrist_in_body_raw, 
                                        self.filtered_wrist_in_body, 
                                        self.alpha_wrist
                                    )
                                    wrist_in_body = self.filtered_wrist_in_body
                                    
                                    # Publish PoseStamped
                                    pose_msg = PoseStamped()
                                    pose_msg.header.stamp = self.get_clock().now().to_msg()
                                    pose_msg.header.frame_id = self.output_frame
                                    pose_msg.pose.position.x = float(wrist_in_body[0])  # forward
                                    pose_msg.pose.position.y = float(wrist_in_body[1])  # left
                                    pose_msg.pose.position.z = float(wrist_in_body[2])  # up
                                    # Orientation: identity (no wrist orientation yet)
                                    pose_msg.pose.orientation.w = 1.0
                                    pose_msg.pose.orientation.x = 0.0
                                    pose_msg.pose.orientation.y = 0.0
                                    pose_msg.pose.orientation.z = 0.0
                                    
                                    self.wrist_pose_pub.publish(pose_msg)
                                    
                                    # Publish TF: body → wrist_target
                                    tf_msg = TransformStamped()
                                    tf_msg.header.stamp = pose_msg.header.stamp
                                    tf_msg.header.frame_id = self.output_frame  # "body"
                                    tf_msg.child_frame_id = "wrist_target"
                                    tf_msg.transform.translation.x = float(wrist_in_body[0])
                                    tf_msg.transform.translation.y = float(wrist_in_body[1])
                                    tf_msg.transform.translation.z = float(wrist_in_body[2])
                                    tf_msg.transform.rotation.w = 1.0
                                    tf_msg.transform.rotation.x = 0.0
                                    tf_msg.transform.rotation.y = 0.0
                                    tf_msg.transform.rotation.z = 0.0
                                    self.tf_broadcaster.sendTransform(tf_msg)
                            
                            # Calculate 3D endpoints of each axis
                            axis_x_end_3d = origin_3d + axis_x * axis_length_m
                            axis_y_end_3d = origin_3d + axis_y * axis_length_m
                            axis_z_end_3d = origin_3d + axis_z * axis_length_m
                            
                            # Project axis endpoints to 2D
                            axis_x_end = self.project_3d_to_pixel(axis_x_end_3d)
                            axis_y_end = self.project_3d_to_pixel(axis_y_end_3d)
                            axis_z_end = self.project_3d_to_pixel(axis_z_end_3d)
                            
                            # Draw axes if all projections are valid
                            if all(p is not None for p in [axis_x_end, axis_y_end, axis_z_end]):
                                # X axis - Red (lateral, pointing right)
                                cv2.arrowedLine(display_image, origin_px, axis_x_end, 
                                               (0, 0, 255), 4, tipLength=0.15)
                                cv2.putText(display_image, 'X', 
                                           (axis_x_end[0] + 8, axis_x_end[1]),
                                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
                                
                                # Y axis - Green (vertical, pointing up)
                                cv2.arrowedLine(display_image, origin_px, axis_y_end,
                                               (0, 255, 0), 4, tipLength=0.15)
                                cv2.putText(display_image, 'Y',
                                           (axis_y_end[0] + 8, axis_y_end[1]),
                                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                                
                                # Z axis - Blue (frontal, pointing forward/towards camera)
                                cv2.arrowedLine(display_image, origin_px, axis_z_end,
                                               (255, 0, 0), 4, tipLength=0.15)
                                cv2.putText(display_image, 'Z',
                                           (axis_z_end[0] + 8, axis_z_end[1]),
                                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 0, 0), 2)
                                
                                # Display wrist position in body frame
                                if wrist_in_body is not None:
                                    # Background for wrist info
                                    cv2.rectangle(display_image, (5, 5), (320, 85), (0, 0, 0), -1)
                                    cv2.rectangle(display_image, (5, 5), (320, 85), (0, 255, 0), 1)
                                    
                                    cv2.putText(display_image, 
                                               f'Wrist in {self.output_frame} frame (REP-103):',
                                               (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
                                    cv2.putText(display_image, 
                                               f'X (fwd): {wrist_in_body[0]*100:+6.1f} cm',
                                               (10, 45), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
                                    cv2.putText(display_image, 
                                               f'Y (left): {wrist_in_body[1]*100:+6.1f} cm',
                                               (10, 65), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
                                    cv2.putText(display_image, 
                                               f'Z (up): {wrist_in_body[2]*100:+6.1f} cm',
                                               (170, 45), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
                                    
                                    # Log periodically
                                    if self.frame_count % 30 == 0:
                                        self.get_logger().info(
                                            f'Wrist in body: X={wrist_in_body[0]:.3f}, Y={wrist_in_body[1]:.3f}, Z={wrist_in_body[2]:.3f} m'
                                        )
                                
                                # Log depth info periodically
                                if self.frame_count % 60 == 0:
                                    self.get_logger().info(
                                        f'Origin 3D: ({origin_3d[0]:.2f}, {origin_3d[1]:.2f}, {origin_3d[2]:.2f}) m'
                                    )
                                
                                # ========== APRILTAG DETECTION AND COMPARISON ==========
                                # Detect AprilTags in the image
                                gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
                                corners, ids, rejected = self.aruco_detector.detectMarkers(gray)
                                
                                if ids is not None and 0 in ids:
                                    # Find tag ID 0
                                    tag_idx = np.where(ids == 0)[0][0]
                                    tag_corners = corners[tag_idx]
                                    
                                    # Draw detected tag
                                    aruco.drawDetectedMarkers(display_image, [tag_corners], np.array([[0]]))
                                    
                                    # Camera matrix and distortion coefficients
                                    camera_matrix = np.array([
                                        [self.fx, 0, self.cx],
                                        [0, self.fy, self.cy],
                                        [0, 0, 1]
                                    ], dtype=np.float64)
                                    dist_coeffs = np.zeros(5)  # Assuming no distortion (rectified image)
                                    
                                    # Estimate pose of the AprilTag
                                    rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(
                                        [tag_corners], self.tag_size, camera_matrix, dist_coeffs
                                    )
                                    
                                    rvec = rvecs[0][0]
                                    tvec = tvecs[0][0]
                                    
                                    # Convert rotation vector to rotation matrix
                                    R_tag, _ = cv2.Rodrigues(rvec)
                                    
                                    # AprilTag axes in camera frame (columns of rotation matrix)
                                    # Use raw axes without inversion for honest comparison
                                    tag_axis_x = R_tag[:, 0]  # X axis of tag
                                    tag_axis_y = R_tag[:, 1]  # Y axis of tag
                                    tag_axis_z = R_tag[:, 2]  # Z axis of tag
                                    
                                    # AprilTag position in camera frame
                                    tag_position = tvec
                                    
                                    # ========== CALCULATE ERRORS ==========
                                    # Position error (Euclidean distance)
                                    position_error = np.linalg.norm(origin_3d - tag_position)
                                    
                                    # Angular errors between corresponding axes (in degrees)
                                    # Treat ±axis as equivalent (same axis but opposite direction)
                                    def angle_between_vectors(v1, v2):
                                        """Calculate angle between two vectors in degrees, treating ±axis as equivalent."""
                                        cos_angle = np.clip(np.dot(v1, v2), -1.0, 1.0)
                                        ang = np.degrees(np.arccos(cos_angle))
                                        return min(ang, 180.0 - ang)  # Treat opposite directions as equivalent
                                    
                                    angle_error_x = angle_between_vectors(axis_x, tag_axis_x)
                                    angle_error_y = angle_between_vectors(axis_y, tag_axis_y)
                                    angle_error_z = angle_between_vectors(axis_z, tag_axis_z)
                                    
                                    # Store for logging
                                    self.last_position_error = position_error
                                    self.last_angle_errors = (angle_error_x, angle_error_y, angle_error_z)
                                    
                                    # ========== DISPLAY COMPARISON RESULTS ==========
                                    # Background for text
                                    cv2.rectangle(display_image, (5, 95), (350, 230), (0, 0, 0), -1)
                                    cv2.rectangle(display_image, (5, 95), (350, 230), (255, 255, 255), 1)
                                    
                                    # Position comparison
                                    cv2.putText(display_image, 
                                               f'Landmark pos: ({origin_3d[0]:.3f}, {origin_3d[1]:.3f}, {origin_3d[2]:.3f})',
                                               (10, 115), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 255), 1)
                                    cv2.putText(display_image, 
                                               f'AprilTag pos: ({tag_position[0]:.3f}, {tag_position[1]:.3f}, {tag_position[2]:.3f})',
                                               (10, 135), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 0, 255), 1)
                                    
                                    # Position error with color coding
                                    error_color = (0, 255, 0) if position_error < 0.05 else (0, 165, 255) if position_error < 0.1 else (0, 0, 255)
                                    cv2.putText(display_image, 
                                               f'Position error: {position_error*100:.1f} cm',
                                               (10, 160), cv2.FONT_HERSHEY_SIMPLEX, 0.5, error_color, 2)
                                    
                                    # Angular errors
                                    cv2.putText(display_image, 
                                               f'Angle errors (X,Y,Z): {angle_error_x:.1f}, {angle_error_y:.1f}, {angle_error_z:.1f} deg',
                                               (10, 185), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1)
                                    
                                    # Average angular error
                                    avg_angle_error = (angle_error_x + angle_error_y + angle_error_z) / 3
                                    angle_color = (0, 255, 0) if avg_angle_error < 10 else (0, 165, 255) if avg_angle_error < 20 else (0, 0, 255)
                                    cv2.putText(display_image, 
                                               f'Avg angle error: {avg_angle_error:.1f} deg',
                                               (10, 210), cv2.FONT_HERSHEY_SIMPLEX, 0.5, angle_color, 2)
                                    
                                    # Log periodically
                                    if self.frame_count % 30 == 0:
                                        self.get_logger().info(
                                            f'Position error: {position_error*100:.2f} cm | '
                                            f'Angle errors (X,Y,Z): {angle_error_x:.1f}, {angle_error_y:.1f}, {angle_error_z:.1f} deg'
                                        )
                                else:
                                    # AprilTag not detected
                                    cv2.putText(display_image, 
                                               'AprilTag ID 0 not detected',
                                               (10, 115), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 165, 255), 1)
                    else:
                        # Show message if depth is invalid
                        cv2.putText(
                            display_image,
                            'Invalid depth for landmarks',
                            (10, 90),
                            cv2.FONT_HERSHEY_SIMPLEX,
                            0.6,
                            (0, 165, 255),
                            2
                        )
            else:
                # No pose detected
                cv2.putText(
                    display_image,
                    'No pose detected',
                    (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.7,
                    (0, 0, 255),
                    2
                )
            
            # Add frame info
            self.frame_count += 1
            info_text = f'Frame: {self.frame_count} | Detections: {self.detection_count}'
            cv2.putText(
                display_image,
                info_text,
                (10, height - 10),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (255, 255, 255),
                1
            )
            
            # Display the image
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
