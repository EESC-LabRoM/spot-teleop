#!/usr/bin/env python3

"""
Continuous-direct teleop + autonomous grasp (SDK-direct, one lease).

Replicates the old `continuous_direct_grasp.py` SpotController on the ROS2 stack:
a single node holds Spot's lease (via the Bosdyn SDK directly) and does BOTH
  * wrist-follow: streams /wrist_pose -> SDK arm_pose_command (EE follows the operator),
  * fist-grasp: the closed-fist gesture fires Spot's native PickObject at target_object,
    pausing the follow during the grasp and resuming after (now carrying the object).

It does NOT go through the spot_ros2 driver's control APIs, which churn/contend the
lease (LeaseUseError). The driver runs only for CAMERAS / state / TF.

Run alongside (separately):
  * spot_ros2 driver — cameras/state/TF only. With auto_claim:=false + start_estop:=false
    it never grabs the lease, so this node can own it. Do NOT also run wrist_arm_follower,
    cuRobo, or control_mode_switcher (they route control through the driver -> lease fight).
  * perception:  ros2 launch spot_operation_ros2 perception_minimal.launch.py
                 sim:=false object_prompt:="red bottle"
  * wrist detector (publishes /wrist_pose) + gesture node (publishes hand_gesture).
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    hostname_arg = DeclareLaunchArgument(
        'hostname', default_value='10.0.0.3', description='Spot IP.')
    grasp_frame_arg = DeclareLaunchArgument(
        'grasp_frame', default_value='vision',
        description="World frame target_object is published in (vision/odom).")
    follow_arg = DeclareLaunchArgument(
        'follow', default_value='true',
        description='Stream /wrist_pose to the arm before/after the grasp.')
    lock_arg = DeclareLaunchArgument(
        'lock_xyz_on_grasp', default_value='false',
        description='Freeze EE position after a grasp (rotation-only) until release.')
    grasp_orientation_arg = DeclareLaunchArgument(
        'grasp_orientation', default_value='frontal',
        description="Grasp approach: frontal (horizontal) | top_down | auto.")
    log_level_arg = DeclareLaunchArgument('log_level', default_value='info')

    grasp = Node(
        package='spot_operation_ros2',
        executable='autonomous_grasp_node',
        name='autonomous_grasp_node',
        output='screen',
        emulate_tty=True,
        parameters=[{
            'hostname': LaunchConfiguration('hostname'),
            'grasp_frame': LaunchConfiguration('grasp_frame'),
            'follow': LaunchConfiguration('follow'),
            'lock_xyz_on_grasp': LaunchConfiguration('lock_xyz_on_grasp'),
            'grasp_orientation': LaunchConfiguration('grasp_orientation'),
        }],
        arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
    )

    return LaunchDescription([
        hostname_arg,
        grasp_frame_arg,
        follow_arg,
        lock_arg,
        grasp_orientation_arg,
        log_level_arg,
        grasp,
    ])
