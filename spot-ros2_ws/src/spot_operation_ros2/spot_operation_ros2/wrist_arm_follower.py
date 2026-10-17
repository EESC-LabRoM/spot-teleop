#!/usr/bin/env python3

"""
Wrist -> Arm follower (high-level "continuous direct" teleop).

Replicates the arm-following behaviour of the old `continuous_direct_grasp.py`:
it streamed Spot's high-level `arm_pose_command` to make the end-effector track
the operator's wrist target. No cuRobo, no MoveIt.

On the current ROS2 stack the spot_ros2 driver already exposes that exact path:
publishing a PoseStamped to `arm_pose_commands` calls `spot_arm.hand_pose(...)`,
i.e. a high-level ArmCartesianCommand in the pose's header.frame_id. So this node
just relays `/wrist_pose` (from wrist_detector, in the `body` frame) to
`arm_pose_commands`, throttled to a sane rate.

Why throttle: wrist_detector publishes at camera rate (~30 Hz) and every message
makes the driver issue a new robot_command that preempts the previous one. The
old script streamed at 5 Hz; ~10 Hz here keeps following smooth without flooding
the robot_command action server.

The closed-fist grasp (autonomous_grasp_node) is also high-level, so it cleanly
preempts this stream, grasps, and then the stream resumes — no mode switching.

    ros2 run spot_operation_ros2 wrist_arm_follower
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped


class WristArmFollower(Node):
    """Relay /wrist_pose -> arm_pose_commands at a throttled rate."""

    def __init__(self):
        super().__init__('wrist_arm_follower')

        self.declare_parameter('input_topic', '/wrist_pose')
        self.declare_parameter('output_topic', 'arm_pose_commands')
        self.declare_parameter('rate_hz', 10.0)
        # Stop commanding if the wrist target goes stale (operator lost / node died),
        # so the arm holds its last pose instead of chasing an old target.
        self.declare_parameter('stale_timeout_sec', 0.5)

        self.input_topic = str(self.get_parameter('input_topic').value)
        self.output_topic = str(self.get_parameter('output_topic').value)
        self.rate_hz = float(self.get_parameter('rate_hz').value)
        self.stale_timeout = float(self.get_parameter('stale_timeout_sec').value)

        self._latest = None        # latest PoseStamped
        self._latest_rx_sec = None  # wall-ish receive time (node clock)

        self.sub = self.create_subscription(
            PoseStamped, self.input_topic, self._wrist_cb, 10)
        self.pub = self.create_publisher(PoseStamped, self.output_topic, 10)
        self.timer = self.create_timer(1.0 / self.rate_hz, self._tick)

        self.get_logger().info('=' * 60)
        self.get_logger().info('Wrist -> Arm follower (high-level continuous-direct)')
        self.get_logger().info(f'  {self.input_topic}  ->  {self.output_topic}')
        self.get_logger().info(f'  rate: {self.rate_hz:.1f} Hz | stale timeout: {self.stale_timeout:.2f} s')
        self.get_logger().info('=' * 60)

    def _wrist_cb(self, msg: PoseStamped):
        self._latest = msg
        self._latest_rx_sec = self.get_clock().now().nanoseconds * 1e-9

    def _tick(self):
        if self._latest is None or self._latest_rx_sec is None:
            return
        age = self.get_clock().now().nanoseconds * 1e-9 - self._latest_rx_sec
        if age > self.stale_timeout:
            return  # hold last pose; don't chase a stale target
        # Restamp now so the driver/Spot treat it as a fresh command.
        out = PoseStamped()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = self._latest.header.frame_id
        out.pose = self._latest.pose
        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = WristArmFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
