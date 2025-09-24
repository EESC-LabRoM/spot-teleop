#!/usr/bin/env python3
# python
# Lightweight joint state mapper: subscribes /joint_states_isaac and publishes /joint_states_mapped
# Intended for sim-only use (used by launch condition sim==True)

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

MAP = {
    'arm0_sh0': 'arm_sh0', 'arm0_sh1': 'arm_sh1', 'arm0_el0': 'arm_el0', 'arm0_el1': 'arm_el1',
    'arm0_wr0': 'arm_wr0', 'arm0_wr1': 'arm_wr1', 'arm0_f1x': 'arm_f1x',
    'fl_hx': 'front_left_hip_x', 'fl_hy': 'front_left_hip_y', 'fl_kn': 'front_left_knee',
    'fr_hx': 'front_right_hip_x', 'fr_hy': 'front_right_hip_y', 'fr_kn': 'front_right_knee',
    'hl_hx': 'rear_left_hip_x', 'hl_hy': 'rear_left_hip_y', 'hl_kn': 'rear_left_knee',
    'hr_hx': 'rear_right_hip_x', 'hr_hy': 'rear_right_hip_y', 'hr_kn': 'rear_right_knee',
}

# Target ordering: legs then arm (matching URDF/MoveIt expectations)
TARGET = [
    'front_left_hip_x', 'front_left_hip_y', 'front_left_knee',
    'front_right_hip_x', 'front_right_hip_y', 'front_right_knee',
    'rear_left_hip_x', 'rear_left_hip_y', 'rear_left_knee',
    'rear_right_hip_x', 'rear_right_hip_y', 'rear_right_knee',
    'arm_sh0', 'arm_sh1', 'arm_el0', 'arm_el1', 'arm_wr0', 'arm_wr1', 'arm_f1x',
]

class JointStateMapper(Node):
    def __init__(self):
        super().__init__('joint_state_mapper')
        self.pub = self.create_publisher(JointState, '/joint_states_mapped', 10)
        self.sub = self.create_subscription(JointState, '/joint_states_isaac', self.cb, 10)
        self.get_logger().info('joint_state_mapper started, publishing /joint_states_mapped from /joint_states_isaac')

    def cb(self, msg: JointState):
        src_idx = {n: i for i, n in enumerate(msg.name)}
        out = JointState()
        out.header.stamp = self.get_clock().now().to_msg()
        # use same canonical frame as mock (/joint_states had frame_id base_link)
        out.header.frame_id = 'base_link'
        out.name = TARGET.copy()

        # helper to fetch value or default
        def fetch(arr, name):
            if arr is None:
                return 0.0
            i = src_idx.get(name)
            if i is None:
                # try reverse map: find source name that maps to this target
                for s, t in MAP.items():
                    if t == name:
                        i = src_idx.get(s)
                        break
            if i is None:
                return 0.0
            return arr[i] if i < len(arr) else 0.0

        out.position = [fetch(msg.position, n) for n in TARGET]
        out.velocity = [fetch(msg.velocity, n) for n in TARGET]
        out.effort = [fetch(msg.effort, n) for n in TARGET]

        self.pub.publish(out)

def main(args=None):
    rclpy.init(args=args)
    node = JointStateMapper()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
