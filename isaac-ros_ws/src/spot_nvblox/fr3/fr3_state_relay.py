#!/usr/bin/env python3
"""Relay the FR3 robot state from the robot's ROS domain (77, franka_ros2 Jazzy) to the perception domain (8, ZED + nvblox Humble).
Read-only on the robot side: it only subscribes there, it never publishes on domain 77.

  domain 77 -> domain 8:  /joint_states (100 Hz), /robot_description (latched)
/tf is NOT relayed: Jazzy -> Humble TFMessage relaying dropped to ~17 Hz with "sequence size exceeds remaining buffer". A Humble
robot_state_publisher on domain 8 rebuilds the FR3 TF tree from the relayed joint states and URDF instead (launch: fr3_perception.launch.py).
fr3-ros2 runs with a private IPC namespace, so run this with FASTRTPS_DEFAULT_PROFILES_FILE=fastdds_udp_only.xml (no shared memory).
  python3 fr3_state_relay.py [--from 77] [--to 8]      (Humble rclpy; one process, two DDS contexts)
"""
import argparse
import os
import threading
import time

import rclpy
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import String

ap = argparse.ArgumentParser()
ap.add_argument("--from", dest="src", type=int, default=77)
ap.add_argument("--to", dest="dst", type=int, default=8)
ap.add_argument("--urdf-out", default="/tmp/fr3_perception/fr3.urdf", help="also write the received URDF here ('' = off)")
a, _ = ap.parse_known_args()

LATCHED = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)

ctx_in, ctx_out = Context(), Context()
rclpy.init(context=ctx_in, domain_id=a.src)
rclpy.init(context=ctx_out, domain_id=a.dst)
n_in = Node("fr3_state_relay_in", context=ctx_in)
n_out = Node("fr3_state_relay_out", context=ctx_out)

pub_js = n_out.create_publisher(JointState, "/joint_states", 10)
pub_desc = n_out.create_publisher(String, "/robot_description", LATCHED)

count = {"js": 0, "desc": 0}


def on_js(m): pub_js.publish(m); count["js"] += 1
def on_desc(m):
    pub_desc.publish(m); count["desc"] += 1
    if a.urdf_out:                                            # for the Humble robot_state_publisher (takes the URDF as a parameter)
        with open(a.urdf_out + ".tmp", "w") as f: f.write(m.data)
        os.replace(a.urdf_out + ".tmp", a.urdf_out)


n_in.create_subscription(JointState, "/joint_states", on_js, 10)
n_in.create_subscription(String, "/robot_description", on_desc, LATCHED)

ex_in, ex_out = SingleThreadedExecutor(context=ctx_in), SingleThreadedExecutor(context=ctx_out)
ex_in.add_node(n_in); ex_out.add_node(n_out)
threading.Thread(target=ex_out.spin, daemon=True).start()
t_log, last = time.monotonic(), dict(count)
try:
    while rclpy.ok(context=ctx_in):
        ex_in.spin_once(timeout_sec=0.1)
        if time.monotonic() - t_log > 10.0:
            dt = time.monotonic() - t_log
            n_out.get_logger().info(f"relay {a.src}->{a.dst}: joint_states {(count['js'] - last['js']) / dt:.0f} Hz, "
                                    f"robot_description {'yes' if count['desc'] else 'no'}")
            t_log, last = time.monotonic(), dict(count)
except KeyboardInterrupt:
    pass
