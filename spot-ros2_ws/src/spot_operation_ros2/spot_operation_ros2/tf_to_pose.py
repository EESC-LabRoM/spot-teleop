#!/usr/bin/env python3

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener


class TfToPoseNode(Node):
    def __init__(self):
        super().__init__("tf_to_pose_node")

        # 1. Declare parameters for flexibility
        self.declare_parameter("target_frame", "cube")
        self.declare_parameter(
            "base_frame", "base"
        )  # The frame you want 'cube' measured against
        self.declare_parameter("publish_rate", 50.0)  # Hz

        self.target_frame = (
            self.get_parameter("target_frame").get_parameter_value().string_value
        )
        self.base_frame = (
            self.get_parameter("base_frame").get_parameter_value().string_value
        )
        publish_rate = (
            self.get_parameter("publish_rate").get_parameter_value().double_value
        )

        # 2. Set up TF Buffer and Listener
        # The listener automatically subscribes to /tf and /tf_static and populates the buffer
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # 3. Set up the Publisher
        self.publisher = self.create_publisher(PoseStamped, "/wrist_pose", 10)

        # 4. Set up a Timer to query TF and publish periodically
        self.timer = self.create_timer(1.0 / publish_rate, self.timer_callback)
        self.get_logger().info(
            f"Publishing {self.target_frame} relative to {self.base_frame} on /wrist_pose at {publish_rate}Hz"
        )

    def timer_callback(self):
        try:
            # Look up the latest transform from base_frame to target_frame
            # rclpy.time.Time() means "get the most recent available transform"
            t = self.tf_buffer.lookup_transform(
                self.base_frame, self.target_frame, rclpy.time.Time()
            )
        except TransformException as ex:
            # If the transform isn't available yet, log it and return safely
            self.get_logger().debug(
                f"Could not transform {self.base_frame} to {self.target_frame}: {ex}"
            )
            return

        # 5. Convert TransformStamped to PoseStamped
        pose_msg = PoseStamped()

        # Copy the header (this ensures the timestamp and frame_id match the TF calculation)
        pose_msg.header = t.header

        # Copy the translation into position
        pose_msg.pose.position.x = t.transform.translation.x
        pose_msg.pose.position.y = t.transform.translation.y
        pose_msg.pose.position.z = t.transform.translation.z

        # Copy the rotation (quaternion)
        pose_msg.pose.orientation = t.transform.rotation

        # 6. Publish the message
        self.publisher.publish(pose_msg)


def main(args=None):
    rclpy.init(args=args)
    node = TfToPoseNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
