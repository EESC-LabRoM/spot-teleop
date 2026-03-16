#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String, Float64
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
        self.declare_parameter('color_topic', '/camera/camera/color/image_raw')
        self.color_topic = self.get_parameter('color_topic').value
        
        self.get_logger().info(f'Subscribing to color topic: {self.color_topic}')
        
        self.image_sub = self.create_subscription(
            Image,
            self.color_topic,
            self.image_callback,
            10
        )
        
        # Create publishers
        self.gesture_pub = self.create_publisher(
            String,
            'hand_gesture',
            10
        )
        
        self.gripper_pub = self.create_publisher(
            Float64,
            'gripper/goal',
            10
        )
        
        self.current_state = -1 # -1: unknown, 0: open, 1: closed
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
            
            gesture_name = "None"
            if recognition_result.gestures:
                # Get the most confident gesture
                top_gesture = recognition_result.gestures[0][0]
                gesture_name = top_gesture.category_name
            
            # Publish the gesture string
            gesture_msg = String()
            gesture_msg.data = gesture_name
            self.gesture_pub.publish(gesture_msg)
            
            # Logic for gripper (binary)
            new_state = self.current_state
            if "Closed_Fist" in gesture_name:
                new_state = 1
            elif "Open_Palm" in gesture_name:
                new_state = 0
            
            # Publish command only on transition
            if new_state != self.current_state and new_state != -1:
                self.current_state = new_state
                gripper_cmd = Float64()
                # 0.0 = Closed, -1.57 = Open
                gripper_cmd.data = 0.0 if self.current_state == 1 else -1.57
                self.gripper_pub.publish(gripper_cmd)
                
                state_str = "CLOSED" if self.current_state == 1 else "OPEN"
                self.get_logger().info(f'Gesture: {gesture_name} -> Commanding Gripper: {state_str}')
            
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