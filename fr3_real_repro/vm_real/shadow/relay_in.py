"""Shadow relay, driver side: runs INSIDE the fr3-ros2 container (docker exec), where the driver's data is readable
(separate containers get no data from it: shared-memory transport, measured 10-16). Read-only: it only subscribes.
Forwards /joint_states, and the teleop's equilibrium pose converted to the hand pose (/wrist_pose for the MPC, TCP ->
fr3_hand = 0.1034 m back along the tool z), as JSON over UDP to 127.0.0.1:<port> (the container uses the host network).
python3 relay_in.py [port]"""
import json
import socket
import sys
import time

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState

PORT = int(sys.argv[1]) if len(sys.argv) > 1 else 47801
GOALS = not (len(sys.argv) > 2 and sys.argv[2] == "nogoals")   # nogoals: the teleop's pose is not forwarded as an MPC goal
HAND_OFFSET = 0.1034
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
rclpy.init()
n = rclpy.create_node("shadow_relay_in")
cnt = {"js": 0, "goal": 0}


def qrot(q, v):  # q = [x, y, z, w]
    x, y, z, w = q
    R = np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                  [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                  [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])
    return R @ v


def on_js(m):
    cnt["js"] += 1
    sock.sendto(json.dumps({"k": "js", "t": time.time(), "name": list(m.name), "pos": list(m.position),
                            "vel": list(m.velocity)}).encode(), ("127.0.0.1", PORT))


def on_eq(m):
    cnt["goal"] += 1
    p, o = m.pose.position, m.pose.orientation
    q = [o.x, o.y, o.z, o.w]
    hp = np.array([p.x, p.y, p.z]) - qrot(q, np.array([0.0, 0.0, HAND_OFFSET]))
    sock.sendto(json.dumps({"k": "goal", "t": time.time(), "frame": m.header.frame_id, "p": hp.tolist(), "q": q}).encode(),
                ("127.0.0.1", PORT))


n.create_subscription(JointState, "/joint_states", on_js, QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
if GOALS:
    n.create_subscription(PoseStamped, "/cartesian_impedance_target_controller/equilibrium_pose", on_eq, 10)
n.create_timer(10.0, lambda: n.get_logger().info(f"forwarded joint_states {cnt['js']}, goals {cnt['goal']}"))
print(f"shadow relay in: /joint_states + equilibrium_pose -> udp 127.0.0.1:{PORT}", flush=True)
rclpy.spin(n)
