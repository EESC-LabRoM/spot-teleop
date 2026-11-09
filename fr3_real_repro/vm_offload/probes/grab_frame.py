import time
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image

rclpy.init()
node = Node('frame_grab')
qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
                 history=HistoryPolicy.KEEP_LAST)
got = {}
node.create_subscription(Image, '/zed1/zed_node/left/image_rect_color',
                         lambda m: got.setdefault('m', m), qos)
t0 = time.time()
while 'm' not in got and time.time() - t0 < 20:
    rclpy.spin_once(node, timeout_sec=0.2)
m = got.get('m')
if m is None:
    print('NO FRAME')
else:
    arr = np.frombuffer(m.data, dtype=np.uint8).reshape(m.height, m.width, 4)
    cv2.imwrite('/shadow_out/frame.jpg', cv2.cvtColor(arr, cv2.COLOR_BGRA2BGR))
    print(f'frame {m.width}x{m.height} saved')
node.destroy_node()
rclpy.shutdown()
