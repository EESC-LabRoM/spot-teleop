#!/usr/bin/env python3
"""
Launch file for Wrist Detector with ZED camera.

This launch file starts the wrist_detector node configured
for ZED camera topics to detect the right wrist using MediaPipe Pose.

Usage:
    # Basic usage:
    ros2 launch arm_pose_estimator wrist_detector_zed.launch.py
    
    # Show all pose landmarks:
    ros2 launch arm_pose_estimator wrist_detector_zed.launch.py show_all:=true
    
    # Custom wrist marker size and color:
    ros2 launch arm_pose_estimator wrist_detector_zed.launch.py radius:=15
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Declare launch arguments
    color_topic_arg = DeclareLaunchArgument(
        'color_topic',
        default_value='/zed/zed_node/rgb/image_rect_color',
        description='Color image topic from ZED camera'
    )
    
    depth_topic_arg = DeclareLaunchArgument(
        'depth_topic',
        default_value='/zed/zed_node/depth/depth_registered',
        description='Depth image topic from ZED camera'
    )
    
    camera_info_topic_arg = DeclareLaunchArgument(
        'camera_info_topic',
        default_value='/zed/zed_node/depth/camera_info',
        description='Camera info topic from ZED camera'
    )
    
    show_all_arg = DeclareLaunchArgument(
        'show_all',
        default_value='false',
        description='Show all pose landmarks (not just the wrist)'
    )
    
    radius_arg = DeclareLaunchArgument(
        'radius',
        default_value='10',
        description='Radius of the wrist marker circle in pixels'
    )
    
    apriltag_size_arg = DeclareLaunchArgument(
        'tag_size',
        default_value='0.2',
        description='AprilTag size in meters (default 10cm)'
    )

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo/Isaac) clock if true'
    )

    # Wrist Detector Node
    wrist_detector_node = Node(
        package='arm_pose_estimator',
        executable='wrist_detector',
        name='wrist_detector',
        output='screen',
        parameters=[{
            'color_topic': LaunchConfiguration('color_topic'),
            'depth_topic': LaunchConfiguration('depth_topic'),
            'camera_info_topic': LaunchConfiguration('camera_info_topic'),
            'show_all_landmarks': LaunchConfiguration('show_all'),
            'wrist_circle_radius': LaunchConfiguration('radius'),
            'wrist_circle_color': [0, 255, 0],  # Green in BGR
            'apriltag_size': LaunchConfiguration('tag_size'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }],
        emulate_tty=True,
    )

    return LaunchDescription([
        color_topic_arg,
        depth_topic_arg,
        camera_info_topic_arg,
        show_all_arg,
        radius_arg,
        apriltag_size_arg,
        use_sim_time_arg,
        wrist_detector_node,
    ])
