import os
from moveit_configs_utils import MoveItConfigsBuilder
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

def generate_launch_description():
    # Define o nome do robô (pode deixar vazio se não quiser namespace)
    spot_name = ""

    # Inclui o launch do spot_ros2_control
    spot_ros2_control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("spot_ros2_control"),
                "launch",
                "spot_ros2_control_with_arm.launch.py"
            ])
        ),
        launch_arguments={
            "hardware_interface": "mock",     # ou "robot"              # true se quiser simular o braço
            "auto_start": "true",             # ativa controladores na hora
            "launch_rviz": "false",           # RViz será lançado pelo demo
            "spot_name": spot_name             # namespace vazio
        }.items(),
    )

    # Gera config do MoveIt (sem robot_description, já vem do ros2_control)
    moveit_config = (
        MoveItConfigsBuilder("spot", package_name="spot_moveit_config")
        .trajectory_execution(
            file_path=os.path.join(
                get_package_share_directory("spot_moveit_config"),
                "config",
                "moveit_controllers.yaml"
            )
        )
        .robot_description(mappings={
            "arm": "true",
            "add_ros2_control_tag": "false",  # já foi adicionado no outro launch
        })  # ainda precisa passar algo pra não quebrar
        .to_moveit_configs()
    )

    # Lança o robot_state_publisher
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[moveit_config.robot_description],
    )

    # Lança o Move Group
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        name="move_group",
        output="screen",
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.joint_limits,
            moveit_config.planning_pipelines,
            {
                "moveit_controller_manager": "moveit_simple_controller_manager/MoveItSimpleControllerManager",
                "moveit_simple_controller_manager.controller_names": ["arm_controller"],
                "moveit_simple_controller_manager.arm_controller.type": "FollowJointTrajectory",
                "moveit_simple_controller_manager.arm_controller.action_ns": "follow_joint_trajectory",
                "moveit_simple_controller_manager.arm_controller.default": True,
                "moveit_simple_controller_manager.arm_controller.joints": [
                    "arm_sh0", "arm_sh1", "arm_el0", "arm_el1", "arm_wr0", "arm_wr1"
                ]
            }
        ],
    )

    # Lança o RViz com MoveIt
    rviz_config_file = os.path.join(
        get_package_share_directory("spot_moveit_config"),
        "config",
        "moveit.rviz"
    )
    
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config_file],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.joint_limits,
            moveit_config.planning_pipelines,
            moveit_config.trajectory_execution,
        ],
    )

    return LaunchDescription([
        spot_ros2_control_launch,
        robot_state_publisher,
        move_group_node,
        rviz_node,
    ])
