#!/usr/bin/env python3

"""
Gripper Controller Node for Boston Dynamics Spot Robot
Subscribes to gripper position commands and smoothly actuates the gripper.
Other nodes can publish to /gripper/command to control gripper position.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
from spot_msgs.msg import JointCommand
import time
import threading


class GripperControllerNode(Node):
    """
    ROS2 Node for continuous gripper control.
    
    Subscribes to /gripper/command (Float64) and smoothly moves gripper to commanded position.
    Publishes commands to spot_joint_controller.
    """
    
    # Gripper constants
    GRIPPER_JOINT_NAME = "arm_f1x"
    
    # Gripper gains (from set_gripper_gains.py example)
    K_Q_P = 16.0
    K_QD_P = 0.32
    
    # Motion parameters
    DEFAULT_DURATION = 1.5  # seconds
    CONTROL_FREQUENCY = 50.0  # Hz
    
    def __init__(self):
        super().__init__('gripper_controller_node')
        
        # Current state
        self.current_goal = None
        self.is_moving = False
        self.motion_lock = threading.Lock()
        self.latest_joint_state = None
        
        # Publisher for gripper commands
        self.gripper_pub = self.create_publisher(
            JointCommand,
            'spot_joint_controller/joint_commands',
            10
        )
        
        # Joint state subscription for feedback (standard ROS2 subscriber)
        self.joint_state_sub = self.create_subscription(
            JointState,
            'joint_states_mapped',
            self.joint_state_callback,
            10
        )
        
        # Subscribe to gripper command topic
        self.command_sub = self.create_subscription(
            Float64,
            'gripper/command',
            self.command_callback,
            10
        )
        
        self.get_logger().info("=" * 60)
        self.get_logger().info("Gripper Controller Node initialized!")
        self.get_logger().info("=" * 60)
        self.get_logger().info("Subscribing to: /gripper/command (std_msgs/Float64)")
        self.get_logger().info("Publishing to: /spot_joint_controller/joint_commands")
        self.get_logger().info("")
        self.get_logger().info("Usage examples:")
        self.get_logger().info("  Open:  ros2 topic pub --once /gripper/command std_msgs/Float64 'data: -1.57'")
        self.get_logger().info("  Close: ros2 topic pub --once /gripper/command std_msgs/Float64 'data: 0.0'")
        self.get_logger().info("=" * 60)
    
    def joint_state_callback(self, msg: JointState):
        """Store latest joint state message."""
        self.latest_joint_state = msg
    
    def get_gripper_joint_angle(self):
        """Get current gripper joint angle from joint states"""
        try:
            # Use cached joint state instead of synchros2 unwrap_future
            joint_state = self.latest_joint_state
            if joint_state is None:
                self.get_logger().warn("No joint state received yet")
                return None
            
            if self.GRIPPER_JOINT_NAME not in joint_state.name:
                self.get_logger().error(f"Gripper joint {self.GRIPPER_JOINT_NAME} not found!")
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
            
            # Create JointCommand message
            joint_cmd = JointCommand()
            joint_cmd.name = [self.GRIPPER_JOINT_NAME]
            joint_cmd.k_q_p = [self.K_Q_P]
            joint_cmd.k_qd_p = [self.K_QD_P]
            
            self.get_logger().info(f"Moving gripper: {current_angle:.3f} -> {goal_angle:.3f} rad")
            
            # Smooth motion with linear interpolation
            for i in range(npoints):
                target_angle = current_angle + i * step_size
                joint_cmd.position = [target_angle]
                self.gripper_pub.publish(joint_cmd)
                time.sleep(dt)
            
            # Final position - ensure we reach exact goal
            joint_cmd.position = [goal_angle]
            self.gripper_pub.publish(joint_cmd)
            
            self.get_logger().info(f"Gripper reached: {goal_angle:.3f} rad")
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
