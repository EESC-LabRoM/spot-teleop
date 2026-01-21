#!/usr/bin/env python3
"""
Joint State Remapper for robot_state_publisher.

Remaps joint names from arm0_* to arm_* for compatibility with
standalone_arm.urdf.xacro.

Subscribes: /joint_command_curobo (arm0_sh0, arm0_sh1, ...)
Publishes: /joint_states_rsp (arm_sh0, arm_sh1, ...)
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


class JointStateRemapper(Node):
    def __init__(self):
        super().__init__('joint_state_remapper')
        
        # Subscribe to cuRobo joint commands
        self.sub = self.create_subscription(
            JointState,
            '/joint_command_curobo',
            self.callback,
            10
        )
        
        # Publish remapped joint states for robot_state_publisher
        self.pub = self.create_publisher(JointState, '/joint_states_rsp', 10)
        
        self.get_logger().info('Joint State Remapper: arm0_* -> arm_*')
        
    def callback(self, msg: JointState):
        # Create new message with remapped names
        out = JointState()
        out.header = msg.header
        
        # Remap: arm0_sh0 -> arm_sh0, arm0_el0 -> arm_el0, etc.
        out.name = [name.replace('arm0_', 'arm_') for name in msg.name]
        out.position = msg.position
        out.velocity = msg.velocity
        out.effort = msg.effort
        
        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = JointStateRemapper()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
