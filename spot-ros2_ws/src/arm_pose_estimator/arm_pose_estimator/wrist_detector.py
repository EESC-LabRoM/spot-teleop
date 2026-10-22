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

import json
import os
from types import SimpleNamespace
from std_msgs.msg import String
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image, CameraInfo
from array import array
from geometry_msgs.msg import PoseStamped, TransformStamped, Quaternion
import tf2_ros
from cv_bridge import CvBridge
import cv2
import cv2.aruco as aruco
import mediapipe as mp
import numpy as np
import message_filters
from arm_pose_estimator.hand_frame import palm_rotation, tool_orientation, quaternion_matrix, OrientationFilter, fingers_open, hand_roi, robust_depth, proper_rotation, GestureOrientationLock


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
        self.declare_parameter('visualize', True)  # false → headless (no cv2 preview window)

        # Get parameters
        color_topic = self.get_parameter('color_topic').value
        depth_topic = self.get_parameter('depth_topic').value
        camera_info_topic = self.get_parameter('camera_info_topic').value
        self.show_all_landmarks = self.get_parameter('show_all_landmarks').value
        self.visualize = self.get_parameter('visualize').value
        # Opt-in FR3 contract: never make a cached/occluded wrist look current.
        self.require_fresh_tracking = self.declare_parameter('require_fresh_tracking', False).value
        self.body_frame_timeout = self.declare_parameter('body_frame_timeout', 0.6).value
        self.max_capture_age = float(self.declare_parameter('max_capture_age', 0.30).value)
        if not 0 < self.max_capture_age <= 1.: raise ValueError('max_capture_age must be within (0, 1]')
        self.last_body_time = None
        self.orientation_mode = self.declare_parameter('orientation_mode', 'roll').value
        if self.orientation_mode not in ('roll', 'full_6d'): raise ValueError('orientation_mode must be roll or full_6d')
        self.hand_gesture, self.hand_gesture_score = None, 0.
        self.gesture_recognizer = None
        self.hand_capture_ms = -1
        if self.orientation_mode == 'full_6d' and self.declare_parameter('hand_gestures', False).value:
            # Same pretrained Open_Palm / Closed_Fist model used by Spot's hand_pose_estimator.
            model = os.path.join(os.path.dirname(os.path.dirname(__file__)), 'models', 'gesture_recognizer.task')
            options = mp.tasks.vision.GestureRecognizerOptions(
                base_options=mp.tasks.BaseOptions(model_asset_path=model),
                running_mode=mp.tasks.vision.RunningMode.VIDEO, num_hands=2,
                min_hand_detection_confidence=.6, min_hand_presence_confidence=.6,
                min_tracking_confidence=.5)
            self.gesture_recognizer = mp.tasks.vision.GestureRecognizer.create_from_options(options)
        sample_topic = self.declare_parameter('teleop_sample_topic', '').value
        self.teleop_sample_pub = self.create_publisher(String, sample_topic, 1) if sample_topic else None
        self.hand_6d = (mp.solutions.hands.Hands(static_image_mode=False, max_num_hands=2,
                        min_detection_confidence=.7, min_tracking_confidence=.7)
                        if self.orientation_mode == 'full_6d' and self.gesture_recognizer is None else None)
        self.hand_geometry_source = self.declare_parameter('hand_geometry_source', 'depth').value
        if self.hand_geometry_source not in ('depth', 'model'): raise ValueError('invalid hand geometry source')
        self.orientation_filter = OrientationFilter()
        self.gesture_orientation_lock = GestureOrientationLock()
        self.use_hand_roi = self.declare_parameter('use_hand_roi', False).value
        self.body_frame_mode = self.declare_parameter('body_frame_mode', 'hierarchical').value
        if self.body_frame_mode not in ('hierarchical', 'shoulders'): raise ValueError('invalid body frame mode')
        self.robust_depth_enabled = self.declare_parameter('robust_depth', False).value
        self.operator_depth_range = tuple(self.declare_parameter('operator_depth_range', [.1,10.]).value)
        self.last_capture_ns = -1
        self.hand_wrist_camera = None
        self.hand_wrist_pixel = None
        self.hand_reason = 'hand not tracked'
        self.hand_rpy = None
        overlay_topic = self.declare_parameter('overlay_topic', '').value
        self.overlay_pub = self.create_publisher(Image, overlay_topic, 1) if overlay_topic else None
        self.wrist_radius = self.get_parameter('wrist_circle_radius').value
        color_param = self.get_parameter('wrist_circle_color').value
        self.wrist_color = tuple(color_param)
        self.tag_size = self.get_parameter('apriltag_size').value
        
        # Scale factor to compensate human arm vs Spot arm length
        # Spot arm reach: ~984mm, Human arm (shoulder to wrist): ~650mm
        self.declare_parameter('scale_factor', 984.0 / 650.0)
        self.scale_factor = self.get_parameter('scale_factor').value
        # Official origin/dev segment-sum estimator, opt-in to preserve local Spot defaults.
        self.online_scale_estimation = self.declare_parameter('online_scale_estimation', False).value
        self.robot_reach = self.declare_parameter('robot_reach', 0.984).value
        self.arm_length_samples = []
        self.arm_length_window, self.arm_length_min_samples = 150, 90
        self.arm_length_max_spread = 0.03
        self.online_scale = None
        self.arm_length_range = self.declare_parameter('operator_arm_length_range', [.3, 1.2]).value
        self.arm_segment_range = self.declare_parameter('operator_arm_segment_range', [0., 1.2]).value
        
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
        input_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT) if self.require_fresh_tracking else 10
        self.color_sub = message_filters.Subscriber(self, Image, color_topic, qos_profile=input_qos)
        self.depth_sub = message_filters.Subscriber(self, Image, depth_topic, qos_profile=input_qos)
        
        # Synchronize color and depth messages
        self.sync = message_filters.ApproximateTimeSynchronizer(
            [self.color_sub, self.depth_sub],
            queue_size=2 if self.require_fresh_tracking else 10,
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
        self.declare_parameter('filter_alpha_axes', 0.08)  # Lower = smoother/steadier body frame
        self.declare_parameter('filter_alpha_wrist', 0.13)  # Higher = more responsive, lower = smoother
        self.alpha_axes = self.get_parameter('filter_alpha_axes').value
        self.alpha_wrist = self.get_parameter('filter_alpha_wrist').value
        
        # Jump filter parameters.
        # The gate is VELOCITY based: allowed = velocity * dt, clamped to [min, max].
        # This makes it physically meaningful (a shoulder can't move 40cm in one 33ms
        # frame) instead of a single huge fixed budget that lets bad depth through.
        self.declare_parameter('jump_threshold', 0.40)  # absolute hard cap (m)
        self.jump_threshold = self.get_parameter('jump_threshold').value

        # Floor for the velocity gate so detector jitter at high frame rate isn't
        # rejected (m). Body landmarks get this tight floor; the wrist floor is larger.
        self.declare_parameter('min_jump_threshold', 0.06)
        self.min_jump_threshold = self.get_parameter('min_jump_threshold').value

        # Re-seed only on a CONFIRMED stable relocation, never on a transient occlusion.
        # A rejected reading starts/extends a "candidate" baseline; we only adopt it once
        # the candidate has held still (within the gate of itself) for this many frames.
        # Erratic occlusion readings (e.g. your arm crossing in front of a hip) never
        # stay consistent long enough, so the jump filter just holds the frame — which is
        # exactly what should happen for a near-static body.
        self.declare_parameter('reseed_confirm_frames', 12)
        self.reseed_confirm_frames = self.get_parameter('reseed_confirm_frames').value

        # The body is near-static, so the frame does NOT need to be recomputed at wrist
        # rate. Sampling it slowly makes it much stickier (fewer chances for depth noise
        # to leak in) and is cheaper. The wrist still updates every frame against the
        # last held frame. Set to 0.0 to update every frame.
        self.declare_parameter('axes_update_period', 0.2)  # seconds (~5 Hz)
        self.axes_update_period = self.get_parameter('axes_update_period').value

        # Wrist re-seed: simpler consecutive-rejection recovery (wrist moves fast and
        # genuinely; we don't want it frozen for long).
        self.declare_parameter('max_consecutive_rejections', 8)
        self.max_consecutive_rejections = self.get_parameter('max_consecutive_rejections').value

        # Wrist jump filter (higher threshold since wrist moves faster)
        self.declare_parameter('wrist_jump_threshold', 0.40)  # absolute hard cap (m)
        self.wrist_jump_threshold = self.get_parameter('wrist_jump_threshold').value
        self.declare_parameter('max_wrist_velocity', 3.0)  # m/s, wrist moves fast
        self.max_wrist_velocity = self.get_parameter('max_wrist_velocity').value
        self.declare_parameter('min_wrist_jump_threshold', 0.10)  # m, velocity-gate floor
        self.min_wrist_jump_threshold = self.get_parameter('min_wrist_jump_threshold').value

        # Hard slew-rate cap on the FINAL published wrist (m/frame). Guarantees the
        # output can never snap regardless of any residual upstream jump — the strongest
        # robustness/smoothness guarantee, independent of the body-frame quality.
        self.declare_parameter('max_wrist_step', 0.04)
        self.max_wrist_step = self.get_parameter('max_wrist_step').value
        
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
        
        # Previous axes for angular jump filter
        self.prev_axes = {}
        
        # Max plausible velocity for body landmarks (m/s)
        # Shoulders/hips don't move faster than ~1.5 m/s in normal use
        self.declare_parameter('max_landmark_velocity', 1.5)
        self.max_landmark_velocity = self.get_parameter('max_landmark_velocity').value
        
        # Timestamps for velocity-based gating
        self.prev_landmark_times = {}

        # Candidate baselines for confirmed re-seed: {name: {'pos': np.array, 'count': int}}
        self.reseed_candidates = {}

        # Low-rate body-frame update clock
        self.last_axes_update_time = None

        # Wrist gate state
        self.prev_wrist_time = None
        self.reject_count_wrist = 0

        # Final slew-rate-limited published wrist position
        self.published_wrist_in_body = None
        
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
        self.get_logger().info(f'Jump threshold: {self.jump_threshold*100:.1f} cm/frame (max)')
        self.get_logger().info(f'Min jump threshold (body): {self.min_jump_threshold*100:.1f} cm/frame')
        self.get_logger().info(f'Max landmark velocity: {self.max_landmark_velocity:.2f} m/s')
        self.get_logger().info(f'Re-seed confirm frames: {self.reseed_confirm_frames}')
        self.get_logger().info(f'Axes update period: {self.axes_update_period:.2f} s (~{1.0/self.axes_update_period:.1f} Hz)')
        self.get_logger().info(f'Wrist jump threshold: {self.wrist_jump_threshold*100:.1f} cm/frame')
        self.get_logger().info(f'Max wrist step (final slew): {self.max_wrist_step*100:.1f} cm/frame')
        self.get_logger().info(f'Axis jump threshold: {np.degrees(self.axis_jump_threshold):.1f} deg/frame')
        self.get_logger().info(f'Scale factor (human to Spot arm): {self.scale_factor:.3f}')
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
        if getattr(self, 'robust_depth_enabled', False):
            return robust_depth(depth_image, int(u), int(v), window_size//2, self.operator_depth_range)
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

    def apply_jump_filter(self, landmark_name, new_pos, now_sec):
        """Velocity-gated rejection filter for body landmarks, with stuck-recovery.

        A shoulder/hip physically cannot move more than `max_landmark_velocity * dt`
        between two frames, so the allowed budget scales with the actual frame interval
        (clamped to [min_jump_threshold, jump_threshold]). Anything larger is bad data
        (occlusion, depth noise) and is rejected, keeping the previous valid position.

        Pure rejection alone gets stuck forever once the operator genuinely moves (the
        baseline never advances). But we must NOT re-seed on a transient occlusion (e.g.
        an arm crossing in front of a hip), or the frame snaps onto the arm and back.
        So we only re-seed when the rejected readings form a STABLE cluster held for
        `reseed_confirm_frames` frames — a genuine relocation. Erratic occlusion readings
        reset the candidate and never confirm, so the frame is simply held.

        Args:
            landmark_name: Identifier for the landmark (e.g., 'l_sh')
            new_pos: New 3D position (numpy array)
            now_sec: Current time in seconds (for the velocity gate)

        Returns:
            Position to use for the body frame this frame.
        """
        if landmark_name not in self.prev_landmarks_3d:
            self.prev_landmarks_3d[landmark_name] = new_pos.copy()
            self.prev_landmark_times[landmark_name] = now_sec
            self.reseed_candidates.pop(landmark_name, None)
            return new_pos

        prev_pos = self.prev_landmarks_3d[landmark_name]
        dt = max(now_sec - self.prev_landmark_times.get(landmark_name, now_sec), 1e-3)
        allowed = np.clip(self.max_landmark_velocity * dt,
                          self.min_jump_threshold, self.jump_threshold)
        distance = np.linalg.norm(new_pos - prev_pos)

        if distance <= allowed:
            # Normal movement — accept, update baseline, drop any pending candidate.
            self.prev_landmarks_3d[landmark_name] = new_pos.copy()
            self.prev_landmark_times[landmark_name] = now_sec
            self.reseed_candidates.pop(landmark_name, None)
            return new_pos

        # Rejected: hold the frame this cycle no matter what.
        self.body_landmark_rejected = True

        # Confirmed-relocation logic: only adopt a NEW baseline if the rejected readings
        # cluster (stay consistent with each other) for `reseed_confirm_frames` frames.
        # Transient occlusion is erratic, so it resets the candidate and never confirms.
        cand = self.reseed_candidates.get(landmark_name)
        if cand is not None and np.linalg.norm(new_pos - cand['pos']) <= allowed:
            cand['pos'] = new_pos.copy()
            cand['count'] += 1
            if cand['count'] >= self.reseed_confirm_frames:
                self.get_logger().warn(
                    f'{landmark_name}: re-seeding baseline — stable new position held '
                    f'{cand["count"]} frames ({distance*100:.1f}cm from old)'
                )
                self.prev_landmarks_3d[landmark_name] = new_pos.copy()
                self.prev_landmark_times[landmark_name] = now_sec
                self.reseed_candidates.pop(landmark_name, None)
                return prev_pos  # still freeze this frame; next frame uses new baseline
        else:
            # New / inconsistent reading — start a fresh candidate.
            self.reseed_candidates[landmark_name] = {'pos': new_pos.copy(), 'count': 1}

        self.get_logger().warn(
            f'JUMP REJECTED on {landmark_name}: {distance*100:.1f}cm '
            f'(>{allowed*100:.1f}cm) — keeping previous'
        )
        return prev_pos

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
        upper = np.linalg.norm(el_3d - sh_3d)
        fore = np.linalg.norm(wr_3d - el_3d)
        length = upper + fore
        lo, hi = getattr(self, 'arm_length_range', (.3, 1.2))
        segment_lo, segment_hi = getattr(self, 'arm_segment_range', (0., 1.2))
        if not (lo < length < hi and segment_lo < upper < segment_hi and segment_lo < fore < segment_hi):
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

    def _hand_orientation_6d(self, rgb, depth, landmarks, display, stamp):
        """Associate the hand with the right Pose wrist, reconstruct the palm from aligned depth."""
        self.hand_reason = 'show right wrist and open palm'
        self.hand_rpy = None
        self.hand_wrist_camera, self.hand_wrist_pixel = None, None
        if landmarks is None or landmarks[16].visibility < .5 or self.last_valid_R is None:
            return False
        self.hand_gesture, self.hand_gesture_score = None, 0.
        recognition = None
        height, width = depth.shape[:2]
        roi = (0,0,width,height)
        hand_rgb = rgb
        if getattr(self, 'use_hand_roi', False):
            roi = hand_roi(landmarks,width,height)
            x,y,rw,rh = roi
            hand_rgb = cv2.resize(rgb[y:y+rh,x:x+rw], (256,256), interpolation=cv2.INTER_LINEAR)
        if self.gesture_recognizer is not None:
            capture_ms = int(stamp*1000)
            if capture_ms <= self.hand_capture_ms:
                self.hand_reason = 'hand frame out of order'; return False
            self.hand_capture_ms = capture_ms
            recognition = self.gesture_recognizer.recognize_for_video(
                mp.Image(image_format=mp.ImageFormat.SRGB, data=np.ascontiguousarray(hand_rgb)), capture_ms)
            from mediapipe.framework.formats import landmark_pb2
            result = SimpleNamespace(
                multi_hand_landmarks=[landmark_pb2.NormalizedLandmarkList(landmark=[landmark_pb2.NormalizedLandmark(x=v.x,y=v.y,z=v.z) for v in h]) for h in recognition.hand_landmarks],
                multi_hand_world_landmarks=[landmark_pb2.LandmarkList(landmark=[landmark_pb2.Landmark(x=v.x,y=v.y,z=v.z) for v in h]) for h in recognition.hand_world_landmarks])
        else:
            result = self.hand_6d.process(hand_rgb)
        if not result.multi_hand_landmarks: return False
        if getattr(self, 'use_hand_roi', False):
            x,y,rw,rh = roi
            for detected in result.multi_hand_landmarks:
                for landmark in detected.landmark:
                    landmark.x = (x+landmark.x*rw)/width
                    landmark.y = (y+landmark.y*rh)/height
                    landmark.z *= rw/width
        wrist = np.array([landmarks[16].x*width, landmarks[16].y*height])
        candidates = sorted((np.linalg.norm(np.array([h.landmark[0].x*width,h.landmark[0].y*height])-wrist), i)
                            for i,h in enumerate(result.multi_hand_landmarks))
        if candidates[0][0] > .08*min(width,height):
            self.hand_reason = 'hand does not match right wrist'; return False
        if len(candidates)>1 and candidates[1][0]-candidates[0][0] < .02*min(width,height):
            self.hand_reason = 'separate hands for tracking'; return False
        hand = result.multi_hand_landmarks[candidates[0][1]]
        if recognition is not None and len(recognition.gestures) > candidates[0][1]:
            categories = recognition.gestures[candidates[0][1]]
            if categories and categories[0].category_name in ('Open_Palm','Closed_Fist') and categories[0].score >= .75:
                self.hand_gesture = categories[0].category_name
                self.hand_gesture_score = float(categories[0].score)
        self.mp_drawing.draw_landmarks(display,hand,mp.solutions.hands.HAND_CONNECTIONS)
        points=[]
        if self.hand_geometry_source == 'model':
            # The hand model supplies metric 3D geometry already rotated into image axes.
            # Use ZED depth for the wrist translation; tiny MCP depth patches are unreliable
            # when a hand turns sideways. Never switch geometry sources within a session.
            worlds = result.multi_hand_world_landmarks
            if not worlds or len(worlds) <= candidates[0][1]:
                self.hand_reason = '3D hand geometry unavailable'; return False
            world = worlds[candidates[0][1]].landmark
            if self.gesture_recognizer is not None and fingers_open([[v.x,v.y,v.z] for v in world]):
                # Open fingers can be clear in 3D even when the palm faces sideways
                # and the canned classifier reports None or another open-hand gesture.
                self.hand_gesture = 'Open_Palm'
                self.hand_gesture_score = 1.  # geometry gate, not a classifier probability
            u, v = int(hand.landmark[0].x*width), int(hand.landmark[0].y*height)
            if not (1 <= u < width-1 and 1 <= v < height-1): return False
            distance = self.get_depth_at_pixel(depth,u,v,window_size=5)
            if distance is None or not np.isfinite(distance):
                self.hand_reason = 'wrist depth unavailable'; return False
            origin = self.deproject_pixel_to_3d(u,v,distance)
            self.hand_wrist_camera, self.hand_wrist_pixel = origin.copy(), (u,v)
            w0 = np.array([world[0].x,world[0].y,world[0].z])
            points = [origin + np.array([world[i].x,world[i].y,world[i].z])-w0 for i in (0,5,9,13,17)]
        else:
            for i in (0,5,9,13,17):
                lm=hand.landmark[i];u,v=int(lm.x*width),int(lm.y*height)
                if not (1 <= u < width-1 and 1 <= v < height-1): return False
                distance=self.get_depth_at_pixel(depth,u,v,window_size=3)
                if distance is None or not np.isfinite(distance):
                    self.hand_reason='palm depth unavailable'; return False
                points.append(self.deproject_pixel_to_3d(u,v,distance))
        try:
            raw=tool_orientation(points,self.last_valid_R)
            if self.gesture_recognizer is not None:
                raw=self.gesture_orientation_lock.update(raw,self.hand_gesture,stamp,anchor=self.orientation_filter.q)
            if self.gesture_recognizer is not None and self.gesture_orientation_lock.held is not None and self.orientation_filter.q is not None:
                q=self.orientation_filter.hold(stamp)
            else:
                q=self.orientation_filter.update(raw,stamp)
            camera_hand=palm_rotation(points)
        except ValueError as e:
            self.hand_reason=str(e); return False
        self.hand_quat=Quaternion(x=float(q[0]),y=float(q[1]),z=float(q[2]),w=float(q[3]))
        r=quaternion_matrix(q)
        self.hand_rpy=np.degrees([np.arctan2(r[2,1],r[2,2]),
                                np.arctan2(-r[2,0],np.hypot(r[2,1],r[2,2])),
                                np.arctan2(r[1,0],r[0,0])])
        start=self.project_3d_to_pixel(points[0])
        if start:
            for i,color in enumerate(((0,0,255),(0,255,0),(255,0,0))):
                end=self.project_3d_to_pixel(points[0]+.12*camera_hand[:,i])
                if end: cv2.arrowedLine(display,start,end,color,2)
        self.hand_reason='6D hand tracked'
        return True

    def synced_callback(self, color_msg, depth_msg):
        """Process synchronized color and depth images with body frame persistence."""
        try:
            # Convert ROS Images to OpenCV format
            wrist_fresh = False
            hand_fresh = self.orientation_mode != 'full_6d'
            if self.orientation_mode == 'full_6d':
                self.hand_reason = 'show right wrist and palm'
                self.hand_rpy = None
                self.hand_gesture, self.hand_gesture_score = None, 0.
            capture_ns = color_msg.header.stamp.sec*10**9 + color_msg.header.stamp.nanosec
            if self.require_fresh_tracking:
                age = (self.get_clock().now().nanoseconds-capture_ns)*1e-9
                if capture_ns <= self.last_capture_ns or not 0 <= age <= getattr(self, 'max_capture_age', .30): return
                self.last_capture_ns = capture_ns
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

            # Single timestamp for all velocity gates this frame
            now_sec = capture_ns*1e-9 if self.require_fresh_tracking else self.get_clock().now().nanoseconds*1e-9
            
            if results.pose_landmarks:
                self.detection_count += 1
                landmarks = results.pose_landmarks.landmark
                
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
                if self.body_frame_mode == 'shoulders':
                    # Fixed operator camera: shoulder line and camera up define the frame.
                    # Hidden hips/ankles must not veto otherwise valid shoulder tracking.
                    vis_hp, vis_ak = False, False

                # The body is near-static: only recompute the frame at a low rate. Between
                # updates the last held frame is reused for the wrist (computed every frame).
                update_axes = (self.last_axes_update_time is None or
                               (now_sec - self.last_axes_update_time) >= self.axes_update_period)

                if update_axes and vis_sh and self.fx is not None:
                    self.last_axes_update_time = now_sec
                    # Basic points for all levels
                    sh_l_px = (int(l_shoulder.x * width), int(l_shoulder.y * height))
                    sh_r_px = (int(r_shoulder.x * width), int(r_shoulder.y * height))
                    d_sh_l = self.get_depth_at_pixel(depth_image, sh_l_px[0], sh_l_px[1])
                    d_sh_r = self.get_depth_at_pixel(depth_image, sh_r_px[0], sh_r_px[1])
                    
                    if d_sh_l and d_sh_r:
                        l_sh_3d = self.apply_jump_filter('l_sh', self.deproject_pixel_to_3d(sh_l_px[0], sh_l_px[1], d_sh_l), now_sec)
                        r_sh_3d = self.apply_jump_filter('r_sh', self.deproject_pixel_to_3d(sh_r_px[0], sh_r_px[1], d_sh_r), now_sec)
                        
                        if self.require_fresh_tracking and not .20 < np.linalg.norm(l_sh_3d-r_sh_3d) < .75:
                            self.body_landmark_rejected = True
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
                                l_hp_3d = self.apply_jump_filter('l_hp', self.deproject_pixel_to_3d(hp_l_px[0], hp_l_px[1], d_hp_l), now_sec)
                                r_hp_3d = self.apply_jump_filter('r_hp', self.deproject_pixel_to_3d(hp_r_px[0], hp_r_px[1], d_hp_r), now_sec)
                                l_ak_3d = self.apply_jump_filter('l_ak', self.deproject_pixel_to_3d(ak_l_px[0], ak_l_px[1], d_ak_l), now_sec)
                                r_ak_3d = self.apply_jump_filter('r_ak', self.deproject_pixel_to_3d(ak_r_px[0], ak_r_px[1], d_ak_r), now_sec)
                                
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
                                l_hp_3d = self.apply_jump_filter('l_hp', self.deproject_pixel_to_3d(hp_l_px[0], hp_l_px[1], d_hp_l), now_sec)
                                r_hp_3d = self.apply_jump_filter('r_hp', self.deproject_pixel_to_3d(hp_r_px[0], hp_r_px[1], d_hp_r), now_sec)
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
                            self.last_body_time = now_sec
                            self.last_valid_R = np.column_stack([self.filtered_axis_z, -self.filtered_axis_x, self.filtered_axis_y])
                            if self.orientation_mode == 'full_6d':
                                # Position and orientation must use the same orthonormal body frame.
                                self.last_valid_R = proper_rotation(self.last_valid_R)
                            
                            if self.frame_count % 60 == 0:
                                self.get_logger().info(f'Body tracking ACTIVE (Level: {best_level})')
                        else:
                            if self.frame_count % 30 == 0:
                                self.get_logger().warn('Body jump detected - FREEZING axes update for this frame.')
                
                if self.orientation_mode == 'full_6d':
                    hand_fresh = self._hand_orientation_6d(rgb_image,depth_image,landmarks,
                        display_image,capture_ns*1e-9)
                # 2. Process Wrist independently using last valid body frame
                right_wrist = landmarks[self.mp_pose.PoseLandmark.RIGHT_WRIST.value]
                if right_wrist.visibility > 0.5 and self.last_valid_origin is not None:
                    w_x, w_y = int(right_wrist.x * width), int(right_wrist.y * height)
                    if self.orientation_mode == 'full_6d' and self.hand_wrist_pixel is not None:
                        # The hand's precise wrist also supplies position, avoiding two
                        # independent landmark estimates for one synchronized 6D pose.
                        w_x, w_y = self.hand_wrist_pixel
                    cv2.circle(display_image, (w_x, w_y), self.wrist_radius, self.wrist_color, -1)
                    
                    w_depth = self.get_depth_at_pixel(depth_image, w_x, w_y)
                    if w_depth is not None and 0.1 < w_depth < 10.0:
                        w_3d_cam = self.deproject_pixel_to_3d(w_x, w_y, w_depth)
                        if w_3d_cam is not None:
                            if self.online_scale_estimation and self.online_scale is None:
                                shoulder, elbow = landmarks[12], landmarks[14]
                                if shoulder.visibility > 0.5 and elbow.visibility > 0.5:
                                    sx, sy = int(shoulder.x * width), int(shoulder.y * height)
                                    ex, ey = int(elbow.x * width), int(elbow.y * height)
                                    sd = self.get_depth_at_pixel(depth_image, sx, sy)
                                    ed = self.get_depth_at_pixel(depth_image, ex, ey)
                                    if sd is not None and ed is not None and 0.1 < sd < 10 and 0.1 < ed < 10:
                                        sh = self.deproject_pixel_to_3d(sx, sy, sd)
                                        stable_sh = self.prev_landmarks_3d.get('r_sh')
                                        # Do not calibrate limb length from a rejected shoulder/background depth.
                                        consistent = (not self.body_landmark_rejected and stable_sh is not None and
                                                      np.linalg.norm(sh-stable_sh) < .08)
                                        if self.orientation_mode != 'full_6d' or consistent:
                                            self.update_arm_length(sh, self.deproject_pixel_to_3d(ex, ey, ed), w_3d_cam)
                            w_in_body_raw = self.last_valid_R.T @ (w_3d_cam - self.last_valid_origin)

                            # Velocity-gated rejection: the wrist can move fast, so the
                            # budget scales with the frame interval (clamped) instead of a
                            # flat 40cm that lets body-frame-induced shifts through.
                            wrist_jumped = False
                            if self.prev_wrist_in_body is not None:
                                w_dt = max(now_sec - (self.prev_wrist_time or now_sec), 1e-3)
                                w_allowed = np.clip(self.max_wrist_velocity * w_dt,
                                                    self.min_wrist_jump_threshold,
                                                    self.wrist_jump_threshold)
                                wrist_delta = np.linalg.norm(w_in_body_raw - self.prev_wrist_in_body)
                                if wrist_delta >= w_allowed:
                                    self.reject_count_wrist += 1
                                    # Re-seed after a sustained run so we don't freeze forever.
                                    if self.reject_count_wrist >= self.max_consecutive_rejections:
                                        self.prev_wrist_in_body = w_in_body_raw.copy()
                                        self.prev_wrist_time = now_sec
                                        self.reject_count_wrist = 0
                                    wrist_jumped = True
                                    if self.frame_count % 15 == 0:
                                        self.get_logger().warn(
                                            f'Wrist jump rejected: {wrist_delta*100:.1f}cm '
                                            f'(>{w_allowed*100:.1f}cm) — keeping filtered')
                                else:
                                    self.prev_wrist_in_body = w_in_body_raw.copy()
                                    self.prev_wrist_time = now_sec
                                    self.reject_count_wrist = 0
                            else:
                                self.prev_wrist_in_body = w_in_body_raw.copy()
                                self.prev_wrist_time = now_sec

                            if not wrist_jumped:
                                wrist_fresh = True
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
                
            # In full 6D mode the quaternion comes from the same RGB-D pair and body frame.
            # Legacy roll behavior is preserved; FR3 full pose requires a fresh palm.
            body_fresh = (self.last_body_time is not None and
                          0 <= now_sec - self.last_body_time <= self.body_frame_timeout)
            scale_ready = not self.online_scale_estimation or self.online_scale is not None
            current_scale = self.online_scale if self.online_scale is not None else self.scale_factor
            if self.filtered_wrist_in_body is not None and scale_ready and hand_fresh and (
                    not self.require_fresh_tracking or (wrist_fresh and body_fresh and hand_fresh)):
                # Hard slew-rate limit on the published target: bounds the change per
                # frame so NOTHING upstream (a body-frame wobble, a depth spike that
                # slipped the gates) can ever snap the commanded pose. This is the final
                # smoothness/robustness guarantee, decoupled from body-frame quality.
                if self.published_wrist_in_body is None:
                    self.published_wrist_in_body = self.filtered_wrist_in_body.copy()
                else:
                    step = self.filtered_wrist_in_body - self.published_wrist_in_body
                    step_norm = np.linalg.norm(step)
                    if step_norm > self.max_wrist_step:
                        step *= self.max_wrist_step / step_norm
                    self.published_wrist_in_body = self.published_wrist_in_body + step

                wrist_final = self.published_wrist_in_body * current_scale + self.shoulder_offset
                
                stamp = color_msg.header.stamp if self.require_fresh_tracking else self.get_clock().now().to_msg()
                
                pose_msg = PoseStamped()
                pose_msg.header.stamp = stamp
                pose_msg.header.frame_id = self.output_frame
                pose_msg.pose.position.x, pose_msg.pose.position.y, pose_msg.pose.position.z = wrist_final
                pose_msg.pose.orientation = self.hand_quat
                self.wrist_pose_pub.publish(pose_msg)
                if self.teleop_sample_pub is not None and self.orientation_mode == 'full_6d':
                    ns = stamp.sec*10**9 + stamp.nanosec
                    q = self.hand_quat
                    packet = dict(version=2,orientation_mode='full_6d',stamp_ns=ns,orientation_stamp_ns=ns,
                                  frame=self.output_frame,p=[float(v) for v in wrist_final],q=[q.x,q.y,q.z,q.w],
                                  gesture=self.hand_gesture,gesture_score=self.hand_gesture_score)
                    self.teleop_sample_pub.publish(String(data=json.dumps(packet,allow_nan=False)))
                
                tf_msg = TransformStamped()
                tf_msg.header.stamp = stamp
                tf_msg.header.frame_id = self.output_frame
                tf_msg.child_frame_id = "wrist_target"
                tf_msg.transform.translation.x, tf_msg.transform.translation.y, tf_msg.transform.translation.z = wrist_final
                tf_msg.transform.rotation.x, tf_msg.transform.rotation.y, tf_msg.transform.rotation.z, tf_msg.transform.rotation.w = \
                    self.hand_quat.x, self.hand_quat.y, self.hand_quat.z, self.hand_quat.w
                self.tf_broadcaster.sendTransform(tf_msg)
                
                if self.frame_count % 30 == 0:
                    self.get_logger().info(f'Wrist in Body: {self.published_wrist_in_body}')
            
            if self.overlay_pub is not None:
                tracking = wrist_fresh and body_fresh and scale_ready and hand_fresh
                self.get_logger().info('6D status: ' + (self.hand_reason if not hand_fresh else ('wrist/body unavailable' if not tracking else 'tracking')), throttle_duration_sec=2.) if self.orientation_mode == 'full_6d' else None
                label = ('6D WRIST + HAND TRACKING' if self.orientation_mode == 'full_6d' else 'WRIST TRACKING') if tracking else 'TRACKING PAUSED'
                if self.orientation_mode == 'full_6d' and not hand_fresh:
                    label = '6D: ' + self.hand_reason
                if not scale_ready:
                    label = (f'ARM SCALE {len(self.arm_length_samples)}/90: SHOW SHOULDER, ELBOW, WRIST'
                             if len(self.arm_length_samples) < 90 else 'ARM SCALE: DEPTH UNSTABLE - FACE CAMERA')
                cv2.putText(display_image, label, (8, 22), cv2.FONT_HERSHEY_SIMPLEX,
                            0.48, (40, 220, 40) if tracking else (30, 30, 255), 1)
                if self.online_scale is not None:
                    cv2.putText(display_image, f'Arm {self.robot_reach / self.online_scale:.2f} m -> FR3 {self.robot_reach:.3f} m | scale {self.online_scale:.3f}',
                                (8, 42), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (255, 255, 255), 1)
                if self.gesture_recognizer is not None:
                    cv2.putText(display_image, 'Grip: ' + (self.hand_gesture or 'uncertain'), (8,82),
                                cv2.FONT_HERSHEY_SIMPLEX,.43,(255,255,255),1)
                if self.hand_rpy is not None:
                    cv2.putText(display_image, 'Hand R/P/Y [deg]: ' + ' '.join(f'{v:+.0f}' for v in self.hand_rpy),
                                (8, 62), cv2.FONT_HERSHEY_SIMPLEX, .43, (255,255,255), 1)
                if tracking and self.published_wrist_in_body is not None:
                    xyz = self.published_wrist_in_body * current_scale + self.shoulder_offset
                    cv2.putText(display_image, 'FR3 reference [m]: ' + ' '.join(f'{v:+.2f}' for v in xyz),
                                (8, height - 12), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1)
                # array('B') bypasses ROS's per-byte Python validation (~100 ms per frame).
                overlay = Image()
                overlay.height, overlay.width = display_image.shape[:2]
                overlay.encoding = 'bgr8'
                overlay.step = overlay.width * 3
                overlay.data = array('B', np.ascontiguousarray(display_image).tobytes())
                overlay.header = color_msg.header
                self.overlay_pub.publish(overlay)
            self.frame_count += 1
            if self.visualize:
                cv2.imshow('Right Wrist Detection', display_image)
                cv2.waitKey(1)
            
        except Exception as e:
            self.get_logger().error(f'Error processing image: {str(e)}')

    def destroy_node(self):
        """Clean up resources when node is destroyed."""
        self.get_logger().info('Shutting down wrist detector...')
        if self.visualize: cv2.destroyAllWindows()
        self.pose.close()
        if self.hand_6d is not None: self.hand_6d.close()
        if self.gesture_recognizer is not None: self.gesture_recognizer.close()
        return super().destroy_node()


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
