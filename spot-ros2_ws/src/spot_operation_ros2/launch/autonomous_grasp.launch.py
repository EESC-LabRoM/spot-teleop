#!/usr/bin/env python3

"""
Launch the autonomous grasp node.

Replicates the old `continuous_direct_grasp.py` autonomous-grasp mode using
Spot's native PickObject grasp via the spot_ros2 driver's `manipulation` action.
The object point is taken from the `target_object` TF that the perception stack
(VLM / SAM2) already publishes.

Prerequisites (must already be running):
    - spot_ros2 driver (holds the lease; provides the `manipulation` action)
    - perception that publishes the `target_object` TF, e.g.:
        ros2 launch spot_operation_ros2 perception_minimal.launch.py \
            sim:=false object_prompt:="red bottle"

Trigger a grasp:
    ros2 service call /autonomous_grasp std_srvs/srv/Trigger

NOTE: This is Spot's HIGH-LEVEL grasp. Do not run it while cuRobo MPC is
streaming low-level joint commands to the arm.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    grasp_frame_arg = DeclareLaunchArgument(
        'grasp_frame', default_value='vision',
        description="Bosdyn-known world frame the grasp point is expressed in. Default "
                    "'vision' matches target_object's parent on the real robot "
                    "(perception_minimal target_parent_frame:=vision). 'odom' also works.")
    target_frame_arg = DeclareLaunchArgument(
        'target_frame', default_value='target_object',
        description='TF child frame of the detected object.')
    auto_trigger_arg = DeclareLaunchArgument(
        'auto_trigger', default_value='false',
        description='If true, fire one grasp automatically on startup (standalone mode).')
    gesture_trigger_arg = DeclareLaunchArgument(
        'gesture_trigger', default_value='true',
        description="If true, the closed-fist gesture (hand_gesture topic) fires the grasp, "
                    "same signal that closes the gripper. Re-armed by an open palm.")
    log_level_arg = DeclareLaunchArgument(
        'log_level', default_value='info',
        description='Logging level (debug, info, warn, error).')

    grasp_node = Node(
        package='spot_operation_ros2',
        executable='autonomous_grasp_node',
        name='autonomous_grasp_node',
        output='screen',
        emulate_tty=True,
        parameters=[{
            'grasp_frame': LaunchConfiguration('grasp_frame'),
            'target_frame': LaunchConfiguration('target_frame'),
            'auto_trigger': LaunchConfiguration('auto_trigger'),
            'gesture_trigger': LaunchConfiguration('gesture_trigger'),
        }],
        arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
    )

    return LaunchDescription([
        grasp_frame_arg,
        target_frame_arg,
        auto_trigger_arg,
        gesture_trigger_arg,
        log_level_arg,
        grasp_node,
    ])
