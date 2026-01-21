#!/usr/bin/env python3

"""
Launch file for Valve Grasping System
Launches gripper controller, grasp valve nodes, and TF bridges.
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """Generate launch description for valve grasping system"""
    
    # Declare launch arguments
    log_level_arg = DeclareLaunchArgument(
        'log_level',
        default_value='info',
        description='Logging level (debug, info, warn, error)'
    )
    
    # Static TF Bridge 1: fp_object ↔ Mesh (FoundationPose ↔ Isaac Sim)
    # Connects FoundationPose detection to Isaac model hierarchy
    tf_bridge_fp_mesh = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_fp_mesh',
        arguments=['0', '0', '0', '0', '0', '0', '1', 'fp_object', 'Mesh'],
        output='screen',
        parameters=[{'use_sim_time': True}],
    )
    
    # Static TF Bridge 2: arm_link_fngr ↔ arm0_link_fngr (ROS ↔ Isaac nomenclature)
    # Connects MoveIt gripper frame to Isaac gripper frame
    tf_bridge_gripper = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_gripper',
        arguments=['0', '0', '0', '0', '0', '0', 'arm_link_fngr', 'arm0_link_fngr'],
        output='screen',
        parameters=[{'use_sim_time': True}],
    )
    
    # Static TF Bridge 3: lever_pivot ↔ lever_pivot_aligned (90° rotation on Z-axis)
    # Aligns lever pivot frame for grasping operations
    tf_bridge_lever_pivot = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_lever_pivot',
        arguments=['0', '0', '0', '0', '0', '0.70710678', '0.70710678', 'lever_pivot', 'lever_pivot_aligned'],
        output='screen',
        parameters=[{'use_sim_time': True}],
    )
    
    # Gripper controller node
    gripper_controller_node = Node(
        package='spot_operation_ros2',
        executable='gripper_controller',
        name='gripper_controller',
        output='screen',
        parameters=[{'use_sim_time': True}],
        arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
        emulate_tty=True,
    )
    
    # Grasp valve node
    grasp_valve_node = Node(
        package='spot_operation_ros2',
        executable='grasp_valve',
        name='valve_grasp_node',
        output='screen',
        parameters=[{'use_sim_time': True}],
        arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
        emulate_tty=True,
    )
    
    # Startup info
    startup_info = LogInfo(
        msg=[
            '\n',
            '=' * 70, '\n',
            '  VALVE GRASPING SYSTEM LAUNCHED\n',
            '=' * 70, '\n',
            '  Nodes:\n',
            '    - gripper_controller: Subscribes to /gripper/command\n',
            '    - valve_grasp_node:   Provides /grasp_valve service\n',
            '\n',
            '  TF Bridges:\n',
            '    - fp_object ↔ Mesh (FoundationPose ↔ Isaac Sim)\n',
            '    - arm_link_wr1 ↔ arm0_link_wr1 (ROS ↔ Isaac nomenclature)\n',
            '    - lever_pivot ↔ lever_pivot_aligned (90° Z-axis rotation)\n',
            '\n',
            '  Usage:\n',
            '    ros2 service call /grasp_valve std_srvs/srv/Trigger\n',
            '\n',
            '  Manual gripper control:\n',
            '    ros2 topic pub --once /gripper/command std_msgs/Float64 "data: -1.57"\n',
            '\n',
            '=' * 70, '\n',
        ]
    )
    
    return LaunchDescription([
        log_level_arg,
        startup_info,
        tf_bridge_fp_mesh,
        tf_bridge_gripper,
        tf_bridge_lever_pivot,
        gripper_controller_node,
        grasp_valve_node,
    ])
