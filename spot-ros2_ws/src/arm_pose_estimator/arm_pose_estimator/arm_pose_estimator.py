#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import cv2
import cv2.aruco as aruco
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo
from message_filters import ApproximateTimeSynchronizer, Subscriber
import tf2_ros
import geometry_msgs.msg
import mediapipe as mp
import os
from geometry_msgs.msg import Quaternion

class ArmPoseEstimator(Node):
    def __init__(self):
        super().__init__('arm_pose_estimator')
        self.get_logger().info("Initializing Arm Pose Estimator...")
    
        self.bridge = CvBridge()
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # Set DISPLAY environment variable if running headless
        if 'DISPLAY' not in os.environ:
            os.environ['DISPLAY'] = ':0'
        
        # Calibration flag and transformation from marker to camera frame
        self.calibrated = False
        self.calib_transform = None

        # Camera intrinsics (to be set from CameraInfo)
        self.camera_info_received = False
        self.camera_matrix = None
        self.dist_coeffs = None
        self.fx = None
        self.fy = None
        self.cx = None
        self.cy = None

        # Variables for temporal smoothing and jump filtering of wrist position
        self.filtered_point = None    # last valid wrist position (in marker frame)
        self.alpha = 0.5              # smoothing factor for exponential moving average
        self.change_threshold = 0.25  # maximum allowed jump (in meters) between frames - INCREASED
        
        # Variables for prediction when depth is missing
        self.prediction_count = 0
        self.max_predictions = 5      # Maximum number of frames to predict
        
        # Variables for improved depth sampling
        self.depth_window_size = 5    # Size of window to sample depth values (5x5 pixels)

        # Check for available topics
        self.get_logger().info("Waiting for topics to be available...")
        
        # Get parameters if specified in launch file
        self.declare_parameter('color_topic', '/camera/color/image_raw')
        self.declare_parameter('depth_topic', '/camera/aligned_depth_to_color/image_raw')
        self.declare_parameter('camera_info_topic', '/camera/aligned_depth_to_color/camera_info')
        self.declare_parameter('marker_size', 0.2)
        self.declare_parameter('debug_visualization', False)
        
        color_topic = self.get_parameter('color_topic').value
        depth_topic = self.get_parameter('depth_topic').value
        camera_info_topic = self.get_parameter('camera_info_topic').value
        
        # Log the selected topics
        self.get_logger().info(f"Using depth topic: {depth_topic}")
        self.get_logger().info(f"Using color topic: {color_topic}")
        self.get_logger().info(f"Using camera info topic: {camera_info_topic}")

        # Subscribe to the camera info topic to get the intrinsics
        self.create_subscription(
            CameraInfo,
            camera_info_topic,
            self.camera_info_callback,
            10)

        # Use message_filters to synchronize the color and depth images
        self.color_sub = Subscriber(self, Image, color_topic)
        self.depth_sub = Subscriber(self, Image, depth_topic)
        self.ats = ApproximateTimeSynchronizer([self.color_sub, self.depth_sub],
                                               queue_size=10, slop=0.1)
        self.ats.registerCallback(self.image_callback)

        # Initialize MediaPipe Pose for wrist detection
        self.mp_pose = mp.solutions.pose
        self.pose = self.mp_pose.Pose(
            min_detection_confidence=0.5,
            min_tracking_confidence=0.5,
            model_complexity=1  # Use a more accurate model
        )
        
        # Get marker size from parameter
        self.marker_length = self.get_parameter('marker_size').value
        
        # Debug visualization flag
        self.debug_visualization = self.get_parameter('debug_visualization').value
        
        self.hand_quat = [0.0, 0.0, 0.0, 1.0]  # Default quaternion
        self.create_subscription(
            Quaternion,
            '/hand_roll_quat',
            self._orientation_callback,
            10)
        
        self.get_logger().info("Arm Pose Estimator Initialized.")

    def camera_info_callback(self, msg):
        # Only update (and log) intrinsics once.
        if not self.camera_info_received:
            self.camera_matrix = np.array(msg.k).reshape(3, 3)
            self.dist_coeffs = np.array(msg.d)
            self.fx = self.camera_matrix[0, 0]
            self.fy = self.camera_matrix[1, 1]
            self.cx = self.camera_matrix[0, 2]
            self.cy = self.camera_matrix[1, 2]
            self.camera_info_received = True
            self.get_logger().info("Camera intrinsics received.")
            self.get_logger().info(f"Camera matrix: \n{self.camera_matrix}")
            self.get_logger().info(f"Distortion coeffs: {self.dist_coeffs}")

    def image_callback(self, color_msg, depth_msg):
        if not self.camera_info_received:
            self.get_logger().warn("Waiting for camera info...")
            return

        try:
            color_image = self.bridge.imgmsg_to_cv2(color_msg, desired_encoding="bgr8")
            depth_image = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding="passthrough")
            self.get_logger().debug("Successfully converted images")
        except Exception as e:
            self.get_logger().error("CV Bridge error: %s" % str(e))
            return

        # Run calibration using ArUco until a marker is detected
        if not self.calibrated:
            self.calibrate(color_image)
        else:
            self.process_pose(color_image, depth_image, color_msg.header)

        # Ensure windows are created and updated if debug visualization is enabled
        if self.debug_visualization:
            try:
                # Wait for a small amount of time to process GUI events
                key = cv2.waitKey(1)
                
                # Optional: Add key handling
                if key == ord('q'):  # Quit on 'q' key
                    try:
                        rclpy.shutdown()
                    except Exception:
                        pass

                    
            except Exception as e:
                self.get_logger().error(f"Error displaying windows: {e}")

    def calibrate(self, color_image):
        # Detect an ArUco marker to define the reference (calibration) frame.
        aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
        # Tenta usar o método criador, mas cai pro construtor direto se não existir
        try:
            parameters = aruco.DetectorParameters_create()
        except AttributeError:
            parameters = aruco.DetectorParameters()
        corners, ids, _ = aruco.detectMarkers(color_image, aruco_dict, parameters=parameters)

        # Create a copy of the image for visualization
        display_image = color_image.copy()

        if ids is not None and len(ids) > 0:
            # Draw detected markers for visualization
            aruco.drawDetectedMarkers(display_image, corners, ids)

            # Use the camera intrinsics obtained from CameraInfo
            rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(corners, self.marker_length,
                                                               self.camera_matrix,
                                                               self.dist_coeffs)
            rvec = rvecs[0][0]
            tvec = tvecs[0][0]
            
            # Draw coordinate axes on the marker
            cv2.drawFrameAxes(display_image, self.camera_matrix, self.dist_coeffs, 
                              rvec, tvec, self.marker_length/2)
            
            # Calculate the transform matrix
            R, _ = cv2.Rodrigues(rvec)
            transform = np.eye(4)
            transform[0:3, 0:3] = R
            transform[0:3, 3] = tvec
            self.calib_transform = transform
            self.calibrated = True
            self.get_logger().info("Calibration successful using ArUco marker.")
            self.get_logger().info(f"Marker position: {tvec}")
        else:
            # Add text to the image to guide the user
            cv2.putText(display_image, "No ArUco marker detected. Please show marker.", 
                        (20, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
            self.get_logger().info("Calibration: No ArUco marker detected. Please hold the marker in view.")

    def process_pose(self, color_image, depth_image, header):
        # Convert the BGR image to RGB for MediaPipe processing.
        image_rgb = cv2.cvtColor(color_image, cv2.COLOR_BGR2RGB)
        results = self.pose.process(image_rgb)

        # Create a copy of the image for visualization
        display_image = color_image.copy()

        if results.pose_landmarks:
            # Use the right wrist landmark (MediaPipe index 16) as an example.
            landmark = results.pose_landmarks.landmark[self.mp_pose.PoseLandmark.RIGHT_WRIST]

            # Check landmark visibility to filter out uncertain detections.
            if landmark.visibility < 0.5:  # Reduced from 0.6 to 0.5
                cv2.putText(display_image, f"Wrist visibility low: {landmark.visibility:.2f}", 
                            (20, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
                self.get_logger().warn("Wrist landmark visibility low (%.2f). Skipping frame." % landmark.visibility)
                
                # Use prediction if we have a previous position
                if self.filtered_point is not None and self.prediction_count < self.max_predictions:
                    self.prediction_count += 1
                    filtered_point_hom = np.array([self.filtered_point[0],
                                                self.filtered_point[1],
                                                self.filtered_point[2], 1]).reshape(4, 1)
                    self.publish_tf(filtered_point_hom, header)
                    
                    # Add prediction indicator to display
                    if self.debug_visualization:
                        cv2.putText(display_image, f"PREDICTING ({self.prediction_count}/{self.max_predictions})", 
                                    (20, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
                return

            h, w, _ = color_image.shape
            pixel_x = int(landmark.x * w)
            pixel_y = int(landmark.y * h)

            # Get the depth value at the wrist pixel using a window of pixels
            if pixel_y >= depth_image.shape[0] or pixel_x >= depth_image.shape[1]:
                self.get_logger().warn("Pixel coordinates out of depth image bounds!")
                return
                
            # Sample depth from a small window around the wrist point
            half_size = self.depth_window_size // 2
            depth_window = depth_image[
                max(0, pixel_y-half_size):min(depth_image.shape[0], pixel_y+half_size+1), 
                max(0, pixel_x-half_size):min(depth_image.shape[1], pixel_x+half_size+1)
            ]
            valid_depths = depth_window[depth_window > 0]
            
            if valid_depths.size > 0:
                depth_mm = np.median(valid_depths)  # Use median to filter outliers
            else:
                # If no valid depth, try to predict
                if self.filtered_point is not None and self.prediction_count < self.max_predictions:
                    self.prediction_count += 1
                    filtered_point_hom = np.array([self.filtered_point[0],
                                                self.filtered_point[1],
                                                self.filtered_point[2], 1]).reshape(4, 1)
                    self.publish_tf(filtered_point_hom, header)
                    
                    # Add prediction indicator to display
                    if self.debug_visualization:
                        cv2.putText(display_image, f"No depth - PREDICTING ({self.prediction_count}/{self.max_predictions})", 
                                    (20, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
                else:
                    cv2.putText(display_image, "No depth data at wrist", 
                                (20, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
                    self.get_logger().warn("Depth value at wrist is zero. Skipping this frame.")
                return
                
            # Reset prediction counter since we have valid data
            self.prediction_count = 0
            
            depth = depth_mm / 1000.0  # Convert depth from mm to meters

            # Back-project the 2D pixel to a 3D point using the intrinsics.
            X = (pixel_x - self.cx) * depth / self.fx
            Y = (pixel_y - self.cy) * depth / self.fy
            Z = depth
            point_cam = np.array([X, Y, Z, 1]).reshape(4, 1)

            # Transform the 3D point from the camera frame to the calibration (marker) frame.
            T_inv = np.linalg.inv(self.calib_transform)
            point_marker = T_inv.dot(point_cam)
            new_point = point_marker[:3, 0]

            # Sanity check: if the new wrist position is absurdly far, skip the update.
            MAX_DISTANCE = 3.0  # adjust based on your expected scene scale (meters)
            if np.linalg.norm(new_point) > MAX_DISTANCE:
                cv2.putText(display_image, f"Wrist too far: {np.linalg.norm(new_point):.2f}m", 
                            (20, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
                self.get_logger().warn("New wrist position is too far (%.2f m). Skipping update." % np.linalg.norm(new_point))
                
                # Use prediction
                if self.filtered_point is not None and self.prediction_count < self.max_predictions:
                    self.prediction_count += 1
                    filtered_point_hom = np.array([self.filtered_point[0],
                                                self.filtered_point[1],
                                                self.filtered_point[2], 1]).reshape(4, 1)
                    self.publish_tf(filtered_point_hom, header)
                    
                    # Add prediction indicator to display
                    if self.debug_visualization:
                        cv2.putText(display_image, f"Too far - PREDICTING ({self.prediction_count}/{self.max_predictions})", 
                                    (20, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
                return

            # Filter out sudden jumps (likely misclassifications).
            if self.filtered_point is not None:
                diff = np.linalg.norm(new_point - self.filtered_point)
                if diff > self.change_threshold:
                    cv2.putText(display_image, f"Large jump: {diff:.3f}m", 
                                (20, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
                    self.get_logger().warn("Large jump in wrist position detected (%.3f m). Ignoring update." % diff)
                    
                    # Use prediction
                    if self.filtered_point is not None and self.prediction_count < self.max_predictions:
                        self.prediction_count += 1
                        filtered_point_hom = np.array([self.filtered_point[0],
                                                    self.filtered_point[1],
                                                    self.filtered_point[2], 1]).reshape(4, 1)
                        self.publish_tf(filtered_point_hom, header)
                        
                        # Add prediction indicator to display
                        if self.debug_visualization:
                            cv2.putText(display_image, f"Jump - PREDICTING ({self.prediction_count}/{self.max_predictions})", 
                                        (20, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
                    return

            # Apply exponential moving average filtering for smoothing.
            if self.filtered_point is None:
                self.filtered_point = new_point
            else:
                self.filtered_point = self.alpha * new_point + (1 - self.alpha) * self.filtered_point

            # Rebuild homogeneous coordinates for the filtered point.
            filtered_point_hom = np.array([self.filtered_point[0],
                                           self.filtered_point[1],
                                           self.filtered_point[2], 1]).reshape(4, 1)
                                           
            self.publish_tf(filtered_point_hom, header)
            
            # Reset prediction counter since we have valid data
            self.prediction_count = 0

            # Draw a circle on the wrist in the visualization.
            cv2.circle(display_image, (pixel_x, pixel_y), 5, (0, 255, 0), -1)
            
            # Draw position text
            cv2.putText(display_image, f"X: {self.filtered_point[0]:.3f}", 
                        (20, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            cv2.putText(display_image, f"Y: {self.filtered_point[1]:.3f}", 
                        (20, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            cv2.putText(display_image, f"Z: {self.filtered_point[2]:.3f}", 
                        (20, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            
            # Draw MediaPipe pose landmarks for visualization
            self.draw_pose_landmarks(display_image, results)
            
        else:
            cv2.putText(display_image, "No pose landmarks detected", 
                        (20, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
            self.get_logger().info("No pose landmarks detected.")
            
            # Use prediction if we have a previous position
            if self.filtered_point is not None and self.prediction_count < self.max_predictions:
                self.prediction_count += 1
                filtered_point_hom = np.array([self.filtered_point[0],
                                            self.filtered_point[1],
                                            self.filtered_point[2], 1]).reshape(4, 1)
                self.publish_tf(filtered_point_hom, header)
                
                # Add prediction indicator to display
                if self.debug_visualization:
                    cv2.putText(display_image, f"No landmarks - PREDICTING ({self.prediction_count}/{self.max_predictions})", 
                                (20, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)

    def draw_pose_landmarks(self, image, results):
        """Draw the pose landmarks on the image."""
        if not results.pose_landmarks:
            return
            
        mp_drawing = mp.solutions.drawing_utils
        mp_drawing_styles = mp.solutions.drawing_styles
        
        # Draw the pose landmarks
        mp_drawing.draw_landmarks(
            image,
            results.pose_landmarks,
            self.mp_pose.POSE_CONNECTIONS,
            landmark_drawing_spec=mp_drawing_styles.get_default_pose_landmarks_style())

    def publish_tf(self, point_marker, header):
        scale_factor = 984.0 / 650.0  # Scale factor to match dimensions

        t = geometry_msgs.msg.TransformStamped()
        t.header.stamp = header.stamp
        t.header.frame_id = "body"
        t.child_frame_id = "wrist"

        # Position
        t.transform.translation.x = float(point_marker[2, 0] * scale_factor) + 0.292
        t.transform.translation.y = float(point_marker[0, 0] * scale_factor)
        t.transform.translation.z = float(point_marker[1, 0] * scale_factor) + 0.188

        # Use the quaternion received from /hand_orientation
        t.transform.rotation.x = self.hand_quat[0]
        t.transform.rotation.y = self.hand_quat[1]
        t.transform.rotation.z = self.hand_quat[2]
        t.transform.rotation.w = self.hand_quat[3]

        self.tf_broadcaster.sendTransform(t)
        self.get_logger().info(
            "Published wrist TF pos=[%.3f, %.3f, %.3f] quat=[%.3f, %.3f, %.3f, %.3f]",
            t.transform.translation.x,
            t.transform.translation.y,
            t.transform.translation.z,
            self.hand_quat[0], self.hand_quat[1], self.hand_quat[2], self.hand_quat[3]
        )

    def _orientation_callback(self, msg: Quaternion):
        # Update the quaternion when a new message is received
        self.hand_quat = [msg.x, msg.y, msg.z, msg.w]

def main():
    rclpy.init()
    try:
        estimator = ArmPoseEstimator()
        rclpy.spin(estimator)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main() 