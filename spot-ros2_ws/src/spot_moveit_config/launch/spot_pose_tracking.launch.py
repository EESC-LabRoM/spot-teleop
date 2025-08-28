#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    
    # Argumentos do launch
    sim_arg = DeclareLaunchArgument(
        'sim',
        default_value='true',
        description='Use simulation time'
    )
    
    # Nó do pose tracker
    pose_tracker_node = Node(
        package='spot_moveit_config',
        executable='spot_pose_tracker.py',
        name='spot_pose_tracker',
        output='screen',
        parameters=[
            {'use_sim_time': LaunchConfiguration('sim')}
        ]
    )
    
    return LaunchDescription([
        sim_arg,
        pose_tracker_node
    ])
