#!/usr/bin/env python3
"""Read-only Humble ZED subscriptions -> loopback UDP; never publishes robot commands."""
import argparse
import json
import math
import socket
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import PoseStamped, Quaternion
from std_msgs.msg import String


def main():
    ap=argparse.ArgumentParser(); ap.add_argument('--port',type=int,default=47813); ap.add_argument('--orientation-mode',choices=['roll','full_6d'],default='full_6d'); a,_=ap.parse_known_args()
    rclpy.init(); n=Node('fr3_zed_input_relay'); sock=socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
    roll=[0]
    def hand(m):
        if all(math.isfinite(x) for x in (m.x,m.y,m.z,m.w)):
            roll[0]=n.get_clock().now().nanoseconds
    def pose(m):
        p,q=m.pose.position,m.pose.orientation
        d=dict(version=1,stamp_ns=m.header.stamp.sec*10**9+m.header.stamp.nanosec,
               roll_stamp_ns=roll[0],frame=m.header.frame_id,p=[p.x,p.y,p.z],q=[q.x,q.y,q.z,q.w])
        if a.orientation_mode == 'full_6d':
            d.update(version=2,orientation_mode='full_6d',orientation_stamp_ns=d['stamp_ns'])
        sock.sendto(json.dumps(d,allow_nan=False).encode(),('127.0.0.1',a.port))
    if a.orientation_mode == 'roll':
        n.create_subscription(Quaternion,'/hand_roll_quat',hand,qos_profile_sensor_data)
    def sample(m):
        try:
            d=json.loads(m.data)
            if d.get('version') != 2 or d.get('orientation_mode') != 'full_6d': return
            sock.sendto(json.dumps(d,allow_nan=False).encode(),('127.0.0.1',a.port))
        except (ValueError,TypeError,OSError): pass
    if a.orientation_mode == 'full_6d':
        # Pose, hand orientation and gesture come from one timestamped perception result.
        n.create_subscription(String,'/fr3_operator/teleop_sample',sample,qos_profile_sensor_data)
        def diagnostics(m):
            try:
                d=json.loads(m.data)
                packet=dict(kind='tracking',version=1,stamp_ns=d['stamp_ns'],diagnostics=d)
                sock.sendto(json.dumps(packet,allow_nan=False).encode(),('127.0.0.1',a.port))
            except (ValueError,TypeError,KeyError,OSError):pass
        n.create_subscription(String,'/fr3_operator/tracking',diagnostics,qos_profile_sensor_data)
    else:
        n.create_subscription(PoseStamped,'/fr3_operator/wrist_pose',pose,qos_profile_sensor_data)
    n.get_logger().info('ZED '+a.orientation_mode+' -> localhost UDP '+str(a.port))
    try:rclpy.spin(n)
    except KeyboardInterrupt:pass
    finally:
        sock.close();n.destroy_node()
        if rclpy.ok():rclpy.shutdown()
if __name__=='__main__':main()
