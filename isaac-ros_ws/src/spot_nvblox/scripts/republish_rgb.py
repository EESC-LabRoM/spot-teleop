#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class BgrToRgb(Node):
    def __init__(self):
        super().__init__('bgr_to_rgb')
        # We will use remappings instead of parameters for topics/node name usually,
        # but parameters are easier for explicit multi-node launch.
        # Actually standard remappings are better.
        # Input: 'image_in'
        # Output: 'image_out'
        
        self.sub = self.create_subscription(
            Image, 'image_in', self.callback, 10
        )
        self.pub = self.create_publisher(Image, 'image_out', 10)
        self.bridge = CvBridge()

    def callback(self, msg):
        try:
            # Convert ANY input to rgb8
            # cv_bridge handles the color conversion logic
            cv_img = self.bridge.imgmsg_to_cv2(msg, "rgb8")
            out_msg = self.bridge.cv2_to_imgmsg(cv_img, "rgb8")
            out_msg.header = msg.header
            self.pub.publish(out_msg)
        except Exception as e:
            self.get_logger().error(f'Conversion error: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    node = BgrToRgb()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
