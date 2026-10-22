#!/usr/bin/env python3
"""Drive the SIMULATED FR3 (Newton sim, Cartesian impedance controller) through a sweep over the bench, so the arm passes in front of
both simulated cameras and over the obstacles: the TCP equilibrium target moves through WAYPOINTS at --speed, keeping the start
orientation. Sim domain only (it publishes /cartesian_impedance_target_controller/equilibrium_pose; never run it on the robot domain).
  ROS_DOMAIN_ID=79 python3 sweep_motion.py [--speed 0.08] [--loops 1]
"""
import argparse
import os
import sys
import time

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from sensor_msgs.msg import JointState

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))
from fr3_arm_mask_node import Chain  # noqa: E402

# TCP positions in fr3_link0: over the obstacles (tops at z <= 0.22) and around them, back to the start
WAYPOINTS = [(0.40, 0.25, 0.40), (0.60, 0.28, 0.36), (0.72, 0.10, 0.34), (0.72, -0.10, 0.34), (0.60, -0.28, 0.36),
             (0.40, -0.25, 0.40), (0.50, 0.00, 0.42), (0.31, 0.00, 0.49)]


def mat2quat(R):
    t = np.trace(R)
    if t > 0:
        s = 2 * np.sqrt(t + 1); return np.array([(R[2, 1] - R[1, 2]) / s, (R[0, 2] - R[2, 0]) / s, (R[1, 0] - R[0, 1]) / s, s / 4])
    i = int(np.argmax(np.diag(R))); j, k = (i + 1) % 3, (i + 2) % 3
    s = 2 * np.sqrt(1 + R[i, i] - R[j, j] - R[k, k]); q = np.zeros(4); q[i] = s / 4
    q[j] = (R[j, i] + R[i, j]) / s; q[k] = (R[k, i] + R[i, k]) / s; q[3] = (R[k, j] - R[j, k]) / s; return q


def main():
    ap = argparse.ArgumentParser(); ap.add_argument("--speed", type=float, default=0.08); ap.add_argument("--loops", type=int, default=1)
    a, ros = ap.parse_known_args()
    rclpy.init(args=ros); n = Node("sim_sweep_motion"); js = {}
    pub = n.create_publisher(PoseStamped, "/cartesian_impedance_target_controller/equilibrium_pose", 1)
    n.create_subscription(JointState, "/joint_states", lambda m: js.update(dict(zip(m.name, m.position))), 20)
    t0 = time.monotonic()
    while "fr3_joint7" not in js and time.monotonic() - t0 < 10: rclpy.spin_once(n, timeout_sec=0.1)
    if "fr3_joint7" not in js: print("no /joint_states from the sim"); return 1
    chain = Chain(os.path.join(os.path.dirname(HERE), "fr3.urdf"))
    T = chain.poses({**js, "fr3_finger_joint1": js.get("fr3_finger_joint1", 0.04)}, ["fr3_hand_tcp"])["fr3_hand_tcp"]
    p, q = T[:3, 3].copy(), mat2quat(T[:3, :3])
    print(f"start TCP {np.round(p, 3).tolist()}, {len(WAYPOINTS)} waypoints x {a.loops}, {a.speed} m/s", flush=True)
    for _ in range(a.loops):
        for w in WAYPOINTS:
            w = np.array(w); d = np.linalg.norm(w - p); steps = max(1, int(d / a.speed * 50))
            for k in range(1, steps + 1):
                s = k / steps; e = 10 * s ** 3 - 15 * s ** 4 + 6 * s ** 5; x = p + (w - p) * e
                m = PoseStamped(); m.header.frame_id = "fr3_link0"; m.header.stamp = n.get_clock().now().to_msg()
                m.pose.position.x, m.pose.position.y, m.pose.position.z = (float(v) for v in x)
                m.pose.orientation.x, m.pose.orientation.y, m.pose.orientation.z, m.pose.orientation.w = (float(v) for v in q)
                pub.publish(m); rclpy.spin_once(n, timeout_sec=0.0); time.sleep(0.02)
            p = w
            for _ in range(50): pub.publish(m); time.sleep(0.02)                              # 1 s dwell
            print(f"reached {np.round(w, 2).tolist()}", flush=True)
    print("sweep done", flush=True)


if __name__ == "__main__":
    sys.exit(main())
