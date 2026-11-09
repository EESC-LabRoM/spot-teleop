#!/usr/bin/env python3
"""Append subscribed ROS topics as JSON lines (one recorder per container).

Perception (fr3-curobo, domain 8):
  record_jsonl.py --domain 8 --out /tmp/bagtest/samples.jsonl \\
      --str /fr3_bagtest/teleop_sample /fr3_bagtest/recorded_sample /fr3_bagtest/tracking
Robot (fr3-ros2, domain 77):
  record_jsonl.py --domain 77 --out /tmp/bagtest/robot.jsonl \\
      --str /teleop/zed_status --bool /teleop/engaged \\
      --pose /franka_robot_state_broadcaster/current_pose /teleop/command_tool \\
      --joints /joint_states /franka_gripper/joint_states
Stops cleanly on SIGTERM/SIGINT (flushes and closes the file).
"""
import argparse
import json
import signal
import sys
import time

import rclpy
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, String

stop = False


def handle(signum, frame):
    global stop
    stop = True


def pose_dict(m):
    p, q = m.pose.position, m.pose.orientation
    return dict(frame=m.header.frame_id, p=[p.x, p.y, p.z], q=[q.x, q.y, q.z, q.w])


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--domain', type=int, required=True)
    ap.add_argument('--out', required=True)
    ap.add_argument('--str', dest='strs', nargs='*', default=[])
    ap.add_argument('--bool', dest='bools', nargs='*', default=[])
    ap.add_argument('--pose', dest='poses', nargs='*', default=[])
    ap.add_argument('--joints', dest='joints', nargs='*', default=[])
    a = ap.parse_args()
    rclpy.init(domain_id=a.domain)
    node = rclpy.create_node('bagtest_recorder')
    out = open(a.out, 'w')
    counts = {}

    def emit(topic, obj):
        counts[topic] = counts.get(topic, 0) + 1
        out.write(json.dumps(dict(wall_t=time.time(), topic=topic, data=obj)) + '\n')

    def as_json(text):
        try:
            return json.loads(text)
        except ValueError:
            return dict(raw=text[:200])

    for t in a.strs:
        node.create_subscription(String, t, (lambda t: lambda m: emit(t, as_json(m.data)))(t),
                                 qos_profile_sensor_data)
    for t in a.bools:
        node.create_subscription(Bool, t, (lambda t: lambda m: emit(t, m.data))(t), 10)
    for t in a.poses:
        node.create_subscription(PoseStamped, t, (lambda t: lambda m: emit(t, pose_dict(m)))(t),
                                 qos_profile_sensor_data)
    for t in a.joints:
        node.create_subscription(JointState, t,
                                 (lambda t: lambda m: emit(t, dict(
                                     name=list(m.name), position=list(m.position))))(t),
                                 qos_profile_sensor_data)
    signal.signal(signal.SIGTERM, handle)
    signal.signal(signal.SIGINT, handle)
    print('recording %d topics -> %s' % (len(a.strs) + len(a.bools) + len(a.poses) + len(a.joints), a.out),
          flush=True)
    try:
        while rclpy.ok() and not stop:
            rclpy.spin_once(node, timeout_sec=0.2)
    finally:
        out.flush()
        out.close()
        print('recorded: ' + json.dumps(counts), flush=True)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
