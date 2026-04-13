#!/usr/bin/env python3

"""
Gripper Controller Node for Boston Dynamics Spot Robot
Subscribes to gripper goal commands and smoothly actuates the gripper.
Other nodes can publish to /gripper/goal to request a gripper position.
Publishes smoothed position to /gripper/command for the driver.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
import time
import threading

try:
    from spot_msgs.msg import JointCommand
except ImportError:
    JointCommand = None


class GripperControllerNode(Node):
    """
    ROS2 Node for continuous gripper control.
    
    Subscribes to /gripper/goal (Float64) and smoothly moves gripper to commanded position.
    Publishes smoothed commands to /gripper/command for the driver.
    """
    
    # Gripper constants
    GRIPPER_JOINT_NAME = "arm_f1x"
    
    # Motion parameters
    DEFAULT_DURATION = 0.7  # seconds
    CONTROL_FREQUENCY = 50.0  # Hz
    
    def __init__(self):
        super().__init__('gripper_controller_node')

        self.declare_parameter('use_ros2_control', False)
        self.use_ros2_control = self.get_parameter('use_ros2_control').value

        # Current state
        self.current_goal = None
        self.is_moving = False
        self.motion_lock = threading.Lock()
        self.latest_joint_state = None
        self.last_state_log_time = 0
        self.state_msg_count = 0

        # Publisher
        if self.use_ros2_control:
            if JointCommand is None:
                self.get_logger().fatal('use_ros2_control=True but spot_msgs.msg.JointCommand not found!')
                raise ImportError('spot_msgs.msg.JointCommand not available')
            self.gripper_pub = self.create_publisher(
                JointCommand,
                '/spot_joint_controller/joint_commands',
                10
            )
            cmd_topic = '/spot_joint_controller/joint_commands'
        else:
            self.gripper_pub = self.create_publisher(
                Float64,
                'gripper/command',
                10
            )
            cmd_topic = 'gripper/command'

        joint_topic = '/low_level/joint_states' if self.use_ros2_control else '/joint_states'
        self.joint_state_sub = self.create_subscription(
            JointState,
            joint_topic,
            self.joint_state_callback,
            10
        )

        self.command_sub = self.create_subscription(
            Float64,
            'gripper/goal',
            self.command_callback,
            10
        )

        mode = "ros2_control (JointCommand)" if self.use_ros2_control else "spot_driver (Float64)"
        self.get_logger().info("=" * 60)
        self.get_logger().info(f"Gripper Controller Node initialized! Mode: {mode}")
        self.get_logger().info("=" * 60)
        self.get_logger().info(f"Subscribing to: /gripper/goal (std_msgs/Float64)")
        self.get_logger().info(f"Publishing to:  {cmd_topic}")
        self.get_logger().info(f"Joint states:   {joint_topic}")
        self.get_logger().info("")
        self.get_logger().info("Usage examples:")
        self.get_logger().info("  Open:  ros2 topic pub --once /gripper/goal std_msgs/Float64 'data: -1.57'")
        self.get_logger().info("  Close: ros2 topic pub --once /gripper/goal std_msgs/Float64 'data: 0.0'")
        self.get_logger().info("=" * 60)
    
    def _publish_gripper(self, angle: float):
        """Publish gripper command in the appropriate format."""
        if self.use_ros2_control:
            msg = JointCommand()
            msg.name = [self.GRIPPER_JOINT_NAME]
            msg.position = [angle]
        else:
            msg = Float64()
            msg.data = angle
        self.gripper_pub.publish(msg)

    def joint_state_callback(self, msg: JointState):
        """Store latest joint state message."""
        self.state_msg_count += 1
        self.latest_joint_state = msg
        
        # Log every 2 seconds or every 100 messages to avoid spam but confirm receipts
        now = time.time()
        if now - self.last_state_log_time > 2.0:
            self.get_logger().info(f"Received joint state update #{self.state_msg_count} (joints: {len(msg.name)})")
            self.last_state_log_time = now
    
    def get_gripper_joint_angle(self):
        """Get current gripper joint angle from joint states"""
        try:
            # Use cached joint state instead of synchros2 unwrap_future
            joint_state = self.latest_joint_state
            if joint_state is None:
                self.get_logger().warn(f"No joint state received yet. Subscriber to: {self.joint_state_sub.topic_name}")
                return None
            
            if self.GRIPPER_JOINT_NAME not in joint_state.name:
                self.get_logger().error(f"Gripper joint {self.GRIPPER_JOINT_NAME} not found in joint state!")
                self.get_logger().debug(f"Available joints: {joint_state.name}")
                return None
            
            gripper_index = joint_state.name.index(self.GRIPPER_JOINT_NAME)
            gripper_position = joint_state.position[gripper_index]
            return gripper_position
            
        except Exception as e:
            self.get_logger().error(f"Failed to get gripper joint angle: {e}")
            return None
    
    def command_callback(self, msg: Float64):
        """
        Callback for gripper position commands.
        Spawns a thread to smoothly move gripper to commanded position.
        """
        goal_angle = msg.data
        
        # Validate command (reasonable range for Spot gripper)
        if goal_angle < -1.6 or goal_angle > 0.1:
            self.get_logger().warn(f"Command {goal_angle:.3f} rad is outside typical range [-1.57, 0.0]. Executing anyway...")
        
        self.get_logger().info(f"Received gripper command: {goal_angle:.3f} rad")
        
        # Start motion in separate thread to avoid blocking
        motion_thread = threading.Thread(
            target=self.move_gripper_smooth,
            args=(goal_angle,),
            daemon=True
        )
        motion_thread.start()
    
    def move_gripper_smooth(self, goal_angle, duration_sec=None):
        """
        Smoothly move gripper to goal angle with linear interpolation.
        
        Args:
            goal_angle: Target angle in radians
            duration_sec: Duration of motion (uses default if None)
        """
        with self.motion_lock:
            if duration_sec is None:
                duration_sec = self.DEFAULT_DURATION
            
            self.is_moving = True
            
            # Get current position
            current_angle = self.get_gripper_joint_angle()
            if current_angle is None:
                self.get_logger().warn("Could not get current gripper angle, commanding goal directly")
                current_angle = 0.0
            
            # Calculate motion parameters
            npoints = int(duration_sec * self.CONTROL_FREQUENCY)
            dt = 1.0 / self.CONTROL_FREQUENCY
            step_size = (goal_angle - current_angle) / npoints if npoints > 0 else 0
            
            self.get_logger().info(f"Moving gripper: {current_angle:.3f} -> {goal_angle:.3f} rad")
            
            # Smooth motion with linear interpolation
            for i in range(npoints):
                target_angle = current_angle + i * step_size
                self._publish_gripper(target_angle)
                time.sleep(dt)

            # Final position - ensure we reach exact goal
            self._publish_gripper(goal_angle)
            
            # Short wait for the robot to respond and joint states to update
            time.sleep(0.2)
            
            # Check actual position to verify
            final_angle = self.get_gripper_joint_angle()
            if final_angle is not None:
                error = abs(final_angle - goal_angle)
                if error < 0.05:
                    self.get_logger().info(f"Gripper reached goal: {final_angle:.3f} rad")
                else:
                    self.get_logger().warn(
                        f"Gripper motion ended at {final_angle:.3f} rad (Goal: {goal_angle:.3f}, Error: {error:.3f})"
                    )
            else:
                self.get_logger().info(f"Gripper command {goal_angle:.3f} rad sent, but feedback is unavailable")
                
            self.is_moving = False


def main(args=None):
    rclpy.init(args=args)
    
    node = GripperControllerNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info("Shutting down Gripper Controller Node...")
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
