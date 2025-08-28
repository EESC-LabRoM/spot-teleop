#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import mediapipe as mp
import numpy as np
import os

class HandPoseEstimator(Node):
    def __init__(self):
        super().__init__('hand_pose_estimator')
        
        # Initialize MediaPipe
        model_path = os.path.join(os.path.dirname(os.path.dirname(__file__)), 
                                'models', 'gesture_recognizer.task')
        
        self.recognizer = mp.tasks.vision.GestureRecognizer.create_from_model_path(model_path)
        
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
        self.gesture_pub = self.create_publisher(
            String,
            'hand_gesture',
            10
        )
        
        self.get_logger().info('Hand pose estimator started')

    def image_callback(self, msg):
        try:
            # Convert ROS Image to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            
            # Convert to RGB for MediaPipe
            rgb_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
            
            # Create MediaPipe image
            mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=rgb_image)
            
            # Detect gestures
            recognition_result = self.recognizer.recognize(mp_image)
            
            if recognition_result.gestures:
                # Get the most confident gesture
                top_gesture = recognition_result.gestures[0][0]
                
                # Publish the gesture
                msg = String()
                msg.data = top_gesture.category_name
                self.gesture_pub.publish(msg)
                
                self.get_logger().info(f'Detected gesture: {top_gesture.category_name} '
                                     f'(score: {top_gesture.score:.2f})')
            
        except Exception as e:
            self.get_logger().error(f'Error processing image: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    node = HandPoseEstimator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.recognizer.close()
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass

if __name__ == '__main__':
    main() 