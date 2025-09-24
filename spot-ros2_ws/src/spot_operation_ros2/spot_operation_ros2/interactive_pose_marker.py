#!/usr/bin/env python3
"""
Interactive Pose Marker Node

Creates an interactive marker in RViz that allows real-time 3D pose manipulation
and publishes the pose to /target_pose topic at 30Hz for compatibility with
arm_pose_estimator frequency.
"""

import rclpy
from rclpy.node import Node
from rclpy.timer import Timer
from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from interactive_markers.menu_handler import MenuHandler
from visualization_msgs.msg import InteractiveMarker, InteractiveMarkerControl, Marker
from geometry_msgs.msg import PoseStamped, Point, TransformStamped
import tf2_ros
from tf2_ros import TransformBroadcaster
import math


class InteractivePoseMarker(Node):
    def __init__(self):
        super().__init__('interactive_pose_marker')

        # Declare parameters
        self.declare_parameter('marker_frame', 'body')
        self.declare_parameter('publish_rate', 30.0)  # Hz
        self.declare_parameter('marker_name', 'target_pose_marker')
        self.declare_parameter('server_name', 'target_pose_marker_server')

        # Get parameters
        self.marker_frame = self.get_parameter('marker_frame').value
        self.publish_rate = self.get_parameter('publish_rate').value
        self.marker_name = self.get_parameter('marker_name').value
        self.server_name = self.get_parameter('server_name').value

        # Publishers
        self.pose_publisher = self.create_publisher(
            PoseStamped,
            '/target_pose',
            10
        )

        # TF broadcaster for visualization
        self.tf_broadcaster = TransformBroadcaster(self)

        # Interactive marker server
        self.server = InteractiveMarkerServer(self, self.server_name)

        # Current pose storage
        self.current_pose = PoseStamped()
        self.current_pose.header.frame_id = self.marker_frame
        self.current_pose.pose.position.x = 0.5
        self.current_pose.pose.position.y = 0.0
        self.current_pose.pose.position.z = 0.3
        self.current_pose.pose.orientation.w = 1.0

        # Create the interactive marker
        self.create_interactive_marker()

        # Timer for continuous publishing at specified rate
        timer_period = 1.0 / self.publish_rate
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.get_logger().info(f'Interactive Pose Marker initialized with {self.publish_rate}Hz publish rate')
        self.get_logger().info(f'Marker frame: {self.marker_frame}')
        self.get_logger().info(f'Marker name: {self.marker_name}')

    def create_interactive_marker(self):
        """Create the interactive marker with 6DOF controls"""
        # Create the interactive marker
        int_marker = InteractiveMarker()
        int_marker.header.frame_id = self.marker_frame
        int_marker.name = self.marker_name
        int_marker.description = "Target Pose (6DOF)"
        int_marker.pose = self.current_pose.pose

        # Create the visual marker (cube)
        box_marker = Marker()
        box_marker.type = Marker.CUBE
        box_marker.scale.x = 0.1
        box_marker.scale.y = 0.1
        box_marker.scale.z = 0.1
        box_marker.color.r = 0.0
        box_marker.color.g = 1.0
        box_marker.color.b = 0.0
        box_marker.color.a = 0.8

        # Create the visual control
        visual_control = InteractiveMarkerControl()
        visual_control.always_visible = True
        visual_control.markers.append(box_marker)
        int_marker.controls.append(visual_control)

        # Create move controls for each axis
        self.add_axis_controls(int_marker)

        # Add the marker to the server
        self.server.insert(int_marker, feedback_callback=self.process_feedback)

        # Add menu handler (optional)
        menu_handler = MenuHandler()
        menu_handler.insert("Reset Pose", callback=self.reset_pose_callback)
        menu_handler.apply(self.server, int_marker.name)

        # Apply changes
        self.server.applyChanges()

    def add_axis_controls(self, int_marker):
        """Add 6DOF control handles to the marker"""
        # Move X axis
        control = InteractiveMarkerControl()
        control.name = "move_x"
        control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        control.orientation.w = 1.0
        control.orientation.x = 1.0
        control.orientation.y = 0.0
        control.orientation.z = 0.0
        control.always_visible = True
        int_marker.controls.append(control)

        # Move Y axis
        control = InteractiveMarkerControl()
        control.name = "move_y"
        control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        control.orientation.w = 1.0
        control.orientation.x = 0.0
        control.orientation.y = 1.0
        control.orientation.z = 0.0
        control.always_visible = True
        int_marker.controls.append(control)

        # Move Z axis
        control = InteractiveMarkerControl()
        control.name = "move_z"
        control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        control.orientation.w = 1.0
        control.orientation.x = 0.0
        control.orientation.y = 0.0
        control.orientation.z = 1.0
        control.always_visible = True
        int_marker.controls.append(control)

        # Rotate X axis
        control = InteractiveMarkerControl()
        control.name = "rotate_x"
        control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        control.orientation.w = 1.0
        control.orientation.x = 1.0
        control.orientation.y = 0.0
        control.orientation.z = 0.0
        control.always_visible = True
        int_marker.controls.append(control)

        # Rotate Y axis
        control = InteractiveMarkerControl()
        control.name = "rotate_y"
        control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        control.orientation.w = 1.0
        control.orientation.x = 0.0
        control.orientation.y = 1.0
        control.orientation.z = 0.0
        control.always_visible = True
        int_marker.controls.append(control)

        # Rotate Z axis
        control = InteractiveMarkerControl()
        control.name = "rotate_z"
        control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        control.orientation.w = 1.0
        control.orientation.x = 0.0
        control.orientation.y = 0.0
        control.orientation.z = 1.0
        control.always_visible = True
        int_marker.controls.append(control)

    def process_feedback(self, feedback):
        """Process feedback from marker interaction"""
        # Update current pose
        self.current_pose.pose = feedback.pose

        # Publish immediately when marker is moved
        self.publish_pose()

        # Update the server with new pose
        self.server.setPose(feedback.marker_name, feedback.pose)
        self.server.applyChanges()

    def reset_pose_callback(self, feedback):
        """Reset marker to default pose"""
        self.current_pose.pose.position.x = 0.5
        self.current_pose.pose.position.y = 0.0
        self.current_pose.pose.position.z = 0.3
        self.current_pose.pose.orientation.w = 1.0
        self.current_pose.pose.orientation.x = 0.0
        self.current_pose.pose.orientation.y = 0.0
        self.current_pose.pose.orientation.z = 0.0

        # Update server
        self.server.setPose(self.marker_name, self.current_pose.pose)
        self.server.applyChanges()

        self.get_logger().info("Marker pose reset to default")

    def timer_callback(self):
        """Timer callback for continuous publishing"""
        self.publish_pose()

    def publish_pose(self):
        """Publish current pose to /target_pose topic"""
        # Update timestamp
        self.current_pose.header.stamp = self.get_clock().now().to_msg()

        # Publish pose
        self.pose_publisher.publish(self.current_pose)

        # Broadcast TF for visualization
        self.broadcast_tf()

    def broadcast_tf(self):
        """Broadcast TF transform for the target pose"""
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.marker_frame
        t.child_frame_id = "target_pose_frame"
        t.transform.translation.x = self.current_pose.pose.position.x
        t.transform.translation.y = self.current_pose.pose.position.y
        t.transform.translation.z = self.current_pose.pose.position.z
        t.transform.rotation = self.current_pose.pose.orientation

        self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    try:
        node = InteractivePoseMarker()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
