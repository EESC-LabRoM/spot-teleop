import os
import yaml
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder("spot", package_name="spot_moveit_config")
        .robot_description(file_path="config/spot.urdf.xacro")
        .to_moveit_configs()
    )

    # Load servo configuration from YAML file
    servo_config_file = os.path.join(
        get_package_share_directory("spot_moveit_config"),
        "config",
        "spot_servo_config.yaml"
    )
    
    with open(servo_config_file, 'r') as file:
        servo_yaml = yaml.safe_load(file)

    # Load pose tracking configuration and merge with servo params
    pose_tracking_config_file = os.path.join(
        get_package_share_directory("spot_moveit_config"),
        "config",
        "pose_tracking_settings.yaml",
    )
    with open(pose_tracking_config_file, 'r') as file:
        pose_tracking_yaml = yaml.safe_load(file)
    
    servo_params = {
        "moveit_servo": {
            **servo_yaml["/**"]["ros__parameters"],
            **pose_tracking_yaml["/**"]["ros__parameters"],
        }
    }

    # ros2_control using spot_ros2_control
    ros2_control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("spot_ros2_control"),
                "launch",
                "spot_ros2_control.launch.py",
            ])
        ),
        launch_arguments={
            "hardware_interface": "mock",
            "mock_arm": "true",
            "auto_start": "true",
            "launch_rviz": "false",
            "spot_name": "",
            "config_file": "",
            "robot_controllers": "arm_controller",
            "control_only": "false",
        }.items(),
    )

    # RViz
    rviz_config_file = (
        get_package_share_directory("spot_moveit_config")
        + "/config/moveit.rviz"
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        # prefix=['xterm -e gdb -ex run --args'],
        output="log",
        arguments=["-d", rviz_config_file],
        parameters=[moveit_config.to_dict()],
    )

    # A node to publish world -> body transform
    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="log",
        arguments=["0.0", "0.0", "0.0", "0.0", "0.0", "0.0", "world", "body"],
    )

    pose_tracking_node = Node(
        package="spot_moveit_config",
        executable="servo_pose_tracking_spot",
        # prefix=['xterm -e gdb -ex run --args'],
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            servo_params,
        ],
    )

    # ros2_control using FakeSystem as hardware
    ros2_controllers_path = os.path.join(
        get_package_share_directory("moveit_resources_panda_moveit_config"),
        "config",
        "ros2_controllers.yaml",
    )
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[moveit_config.robot_description, ros2_controllers_path],
        output="screen",
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager-timeout",
            "300",
            "--controller-manager",
            "/controller_manager",
        ],
    )

    panda_arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["panda_arm_controller", "-c", "/controller_manager"],
    )

    return LaunchDescription(
        [
            rviz_node,
            static_tf,
            ros2_control_launch,
            pose_tracking_node,
        ]
    )
