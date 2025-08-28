#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    """
    Simplified launch file that only launches moveit_servo for debugging purposes.
    This includes the necessary robot_description and robot_description_semantic parameters.
    """
    
    # Build the MoveIt configuration to get the robot description and SRDF
    moveit_cfg = (
        MoveItConfigsBuilder("spot", package_name="spot_moveit_config")
        .robot_description(
            mappings={"arm": "true", "add_ros2_control_tag": "false"}
        )
        .robot_description_semantic(file_path="config/spot.srdf")
        .to_moveit_configs()
    )

    # Robot State Publisher - needed to publish robot description
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[moveit_cfg.robot_description],
    )

    # MoveIt Servo Node with all necessary parameters
    servo_node = Node(
        package="moveit_servo",
        executable="servo_node_main",
        name="servo_node",
        output="screen",
        parameters=[
            # Servo configuration file
            PathJoinSubstitution([
                FindPackageShare("spot_moveit_config"), 
                "config", 
                "spot_servo_config.yaml"
            ]),
            # Robot description (URDF)
            moveit_cfg.robot_description,
            # Robot description semantic (SRDF)  
            moveit_cfg.robot_description_semantic,
            # Enable intra-process communication to avoid the warning
            {"use_intra_process_comms": True},
        ],
        arguments=['--ros-args', '--log-level', 'INFO'],
    )

    return LaunchDescription([
        robot_state_publisher_node,
        servo_node,
    ])
