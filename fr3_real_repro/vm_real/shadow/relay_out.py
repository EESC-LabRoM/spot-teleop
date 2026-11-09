"""Shadow relay, MPC side: own container, ROS domain 78 (the driver is on 77). Republishes what relay_in.py forwards:
/joint_states and /wrist_pose (hand goal). Logs the relay latency (same host clock).
python3 relay_out.py [port]"""
import json
import os
import socket
import sys
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState

PORT = int(sys.argv[1]) if len(sys.argv) > 1 else 47801
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((os.environ.get("RELAY_BIND", "127.0.0.1"), PORT))
sock.settimeout(0.5)
rclpy.init()
n = rclpy.create_node("shadow_relay_out")
pub_js = n.create_publisher(JointState, "/joint_states", 10)
pub_goal = n.create_publisher(PoseStamped, "/wrist_pose", 10)
lat, cnt, t_rep = [], {"js": 0, "goal": 0}, time.monotonic()
print(f"shadow relay out: udp {PORT} -> /joint_states, /wrist_pose (domain 78)", flush=True)
while rclpy.ok():
    try:
        d = json.loads(sock.recv(65536))
    except socket.timeout:
        d = None
    if d:
        lat.append(1000 * (time.time() - d["t"]))
        cnt[d["k"]] += 1
        if d["k"] == "js":
            m = JointState()
            m.header.stamp = n.get_clock().now().to_msg()
            m.name, m.position, m.velocity = d["name"], d["pos"], d["vel"]
            pub_js.publish(m)
        else:
            m = PoseStamped()
            m.header.stamp = n.get_clock().now().to_msg()
            m.header.frame_id = d["frame"] or "fr3_link0"
            m.pose.position.x, m.pose.position.y, m.pose.position.z = d["p"]
            m.pose.orientation.x, m.pose.orientation.y, m.pose.orientation.z, m.pose.orientation.w = d["q"]
            pub_goal.publish(m)
    if time.monotonic() - t_rep > 10.0:
        t_rep = time.monotonic()
        s = sorted(lat) or [0.0]
        print(f"relay last 10 s: joint_states {cnt['js']}, goals {cnt['goal']}, latency median {s[len(s) // 2]:.2f} ms, "
              f"max {s[-1]:.2f} ms", flush=True)
        lat, cnt = [], {"js": 0, "goal": 0}
