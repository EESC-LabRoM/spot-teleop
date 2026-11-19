"""Read-only timestamps from the depth streams actually used by nvblox."""
import json
import os
from pathlib import Path
import socket
os.environ.setdefault('FASTRTPS_DEFAULT_PROFILES_FILE', str(Path(__file__).with_name('live_transport.xml')))
import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

rclpy.init(domain_id=8)
node = rclpy.create_node('zed_live_depth_health')
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
subs = []
# VM103's current 2 cm mapper consumes ZED1 filtered depth exclusively.
# A different camera must never refresh this map's hardware freshness gate.
for camera in ('zed1',):
    def callback(message, camera=camera):
        ns = message.header.stamp.sec * 10**9 + message.header.stamp.nanosec
        packet = dict(kind='segmented_depth', camera=camera, stamp_ns=ns)
        sock.sendto(json.dumps(packet).encode(), ('127.0.0.1', 47929))
    subs.append(node.create_subscription(Image, '/franka_seg_' + camera + '/depth', callback, qos_profile_sensor_data))
try:
    rclpy.spin(node)
finally:
    sock.close()
    node.destroy_node()
    rclpy.shutdown()
