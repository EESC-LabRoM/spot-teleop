#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np
import cv2
import mediapipe as mp

class HandOrientationEstimator(Node):
    def __init__(self):
        super().__init__('hand_orientation_estimator')
        
        # Initialize MediaPipe
        self.mp_hands = mp.solutions.hands
        self.hands = self.mp_hands.Hands(
            static_image_mode=False,
            max_num_hands=2,
            min_detection_confidence=0.7,
            min_tracking_confidence=0.5
        )
        
        # Initialize CV bridge
        self.bridge = CvBridge()
        
        # Create subscribers
        self.image_sub = self.create_subscription(
            Image,
            '/camera/color/image_raw',
            self.image_callback,
            10
        )
        
        # Create publishers
        self.hand_pose_pub = self.create_publisher(
            PoseStamped,
            'hand_orientation',
            10
        )
        
        self.get_logger().info('Hand orientation estimator started')

    def image_callback(self, msg):
        try:
            # Convert ROS Image to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            
            # Convert BGR to RGB
            rgb_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
            
            # Process the image and detect hands
            results = self.hands.process(rgb_image)
            
            if results.multi_hand_landmarks:
                for hand_landmarks in results.multi_hand_landmarks:
                    # Calculate hand orientation using wrist and middle finger MCP
                    wrist = hand_landmarks.landmark[self.mp_hands.HandLandmark.WRIST]
                    middle_finger_mcp = hand_landmarks.landmark[self.mp_hands.HandLandmark.MIDDLE_FINGER_MCP]
                    
                    # Create and publish PoseStamped message
                    pose_msg = PoseStamped()
                    pose_msg.header = msg.header
                    
                    # Set position (wrist position)
                    pose_msg.pose.position.x = wrist.x
                    pose_msg.pose.position.y = wrist.y
                    pose_msg.pose.position.z = wrist.z
                    
                    # Calculate orientation (simplified - just using direction from wrist to MCP)
                    direction = np.array([
                        middle_finger_mcp.x - wrist.x,
                        middle_finger_mcp.y - wrist.y,
                        middle_finger_mcp.z - wrist.z
                    ])
                    direction = direction / np.linalg.norm(direction)
                    
                    # Set a simple orientation (this could be improved with proper quaternion calculation)
                    pose_msg.pose.orientation.w = 1.0
                    pose_msg.pose.orientation.x = direction[0]
                    pose_msg.pose.orientation.y = direction[1]
                    pose_msg.pose.orientation.z = direction[2]
                    
                    self.hand_pose_pub.publish(pose_msg)
                    
        except Exception as e:
            self.get_logger().error(f'Error processing image: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    node = HandOrientationEstimator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.hands.close()
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main() 