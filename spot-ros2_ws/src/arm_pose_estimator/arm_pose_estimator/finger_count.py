#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from std_msgs.msg import Int32

class FingerCountNode(Node):
    def __init__(self):
        super().__init__('finger_count_node')
        
        # Create service
        self.srv = self.create_service(Trigger, 'count_fingers', self.count_fingers_callback)
        
        # Create publisher for finger count
        self.finger_count_pub = self.create_publisher(Int32, 'finger_count', 10)
        
        self.get_logger().info('Finger count service started')

    def count_fingers_callback(self, request, response):
        try:
            # TODO: Implement finger counting logic using MediaPipe hand landmarks
            # For now, just return a dummy value
            count = Int32()
            count.data = 5  # Example: 5 fingers
            self.finger_count_pub.publish(count)
            
            response.success = True
            response.message = f"Detected {count.data} fingers"
        except Exception as e:
            response.success = False
            response.message = f"Error counting fingers: {str(e)}"
        
        return response

def main(args=None):
    rclpy.init(args=args)
    node = FingerCountNode()
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