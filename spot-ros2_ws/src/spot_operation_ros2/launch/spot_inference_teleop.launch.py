from launch import LaunchDescription
from launch.actions import ExecuteProcess, DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os


def generate_launch_description():
    # Declare launch arguments
    model_path_arg = DeclareLaunchArgument(
        'model_path',
        default_value='/home/spot-teleop/spot-ros2_ws/src/spot_operation_ros2/models/policy.pt',
        description='Path to the policy model file'
    )
    
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )

    mode_arg = DeclareLaunchArgument(
        'mode',
        default_value='real',
        description='Operation mode: sim or real'
    )

    action_topic_arg = DeclareLaunchArgument(
        'action_topic',
        default_value='/joint_command_isaac',
        description='Topic to publish joint targets to'
    )
    
    # Teleop node in a separate terminal
    teleop_node = Node(
        package='teleop_twist_keyboard',
        executable='teleop_twist_keyboard',
        name='teleop_twist_keyboard',
        prefix='xterm -e',
        output='screen'
    )
    
    # Inference node - run as process since it's not installed as entry point
    inference_node = ExecuteProcess(
        cmd=[
            'python3',
            '/home/spot-teleop/spot-ros2_ws/src/spot_operation_ros2/spot_operation_ros2/play_inference_ros.py',
            '--model_path', LaunchConfiguration('model_path'),
            '--mode', LaunchConfiguration('mode'),
            '--action_topic', LaunchConfiguration('action_topic'),
            '--ros-args', '-p', 'use_sim_time:=true'
        ],
        output='screen',
        shell=False
    )
    
    return LaunchDescription([
        model_path_arg,
        mode_arg,
        action_topic_arg,
        use_sim_time_arg,
        teleop_node,
        inference_node,
    ])
