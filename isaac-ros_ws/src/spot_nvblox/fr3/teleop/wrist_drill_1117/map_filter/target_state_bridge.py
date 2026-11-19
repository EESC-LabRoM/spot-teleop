#!/usr/bin/env python3
"""Convert explicit target selection + SAM2 tracking state to a fail-closed map gate."""
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

from state_logic import resolve_map_state


class TargetStateBridge(Node):
    def __init__(self):
        super().__init__("wrist_map_target_state_bridge")
        self.declare_parameter("initial_target", "")
        self.declare_parameter("selection_topic", "/wrist_map/target_selection")
        self.declare_parameter("tracking_topic", "/tracking_state")
        self.declare_parameter("state_topic", "/wrist_map/target_state")
        self.declare_parameter("tracking_timeout_s", 3.0)
        self.target = str(self.get_parameter("initial_target").value).strip()
        self.tracking = None
        self.tracking_received = 0.0
        self.timeout = float(self.get_parameter("tracking_timeout_s").value)
        reliable = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self.publisher = self.create_publisher(
            String, str(self.get_parameter("state_topic").value), reliable)
        self.create_subscription(
            String, str(self.get_parameter("selection_topic").value),
            self.selection_cb, reliable)
        self.create_subscription(
            String, str(self.get_parameter("tracking_topic").value),
            self.tracking_cb, reliable)
        self.create_timer(0.25, self.publish_state)

    def selection_cb(self, msg):
        value = msg.data.strip()
        self.target = "" if value.upper() in ("", "NONE", "NO_TARGET", "CLEAR") else value

    def tracking_cb(self, msg):
        value = msg.data.strip().upper()
        self.tracking = value if value in ("TRACKING", "LOST") else "UNKNOWN"
        self.tracking_received = time.monotonic()

    def publish_state(self):
        age = None if self.tracking is None else time.monotonic() - self.tracking_received
        msg = String()
        msg.data = resolve_map_state(self.target, self.tracking, age, self.timeout)
        self.publisher.publish(msg)


def main():
    rclpy.init()
    node = TargetStateBridge()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
