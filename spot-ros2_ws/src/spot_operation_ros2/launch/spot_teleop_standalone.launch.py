import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    GroupAction,
    ExecuteProcess,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, SetParameter
from launch_ros.substitutions import FindPackageShare


def launch_setup(context, *args, **kwargs):
    sim = LaunchConfiguration("sim").perform(context).lower() in ("true", "1", "yes")
    cfg_file = LaunchConfiguration("config_file").perform(context)
    spot_name = ""

    # --- spot_ros2_control ---
    ros2_control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("spot_ros2_control"),
                "launch",
                "spot_ros2_control.launch.py",
            ])
        ),
        launch_arguments={
            "hardware_interface": "mock" if sim else "robot",
            "mock_arm": "true" if sim else "false",
            "auto_start": "true",
            "launch_rviz": "false",
            "spot_name": spot_name,
            "config_file": cfg_file,
            "robot_controllers": "arm_controller spot_joint_controller",
            "control_only": "true" if sim else "false",
        }.items(),
    )

    # --- robot_state_publisher (manual, for sim with remapping) ---
    remappings = [('/joint_states', '/joint_states_mapped')] if sim else []
    
    # Get robot description from spot_description package
    from moveit_configs_utils import MoveItConfigsBuilder
    moveit_cfg = (
        MoveItConfigsBuilder("spot", package_name="spot_moveit_config")
        .robot_description(mappings={"arm": "true", "add_ros2_control_tag": "false"})
        .to_moveit_configs()
    )
    
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[moveit_cfg.robot_description, {'ignore_timestamp': True, 'use_sim_time': sim}],
        remappings=remappings,
        condition=None,  # Always launch when sim=true (control_only=true)
    ) if sim else None

    # --- isaac_publisher (sim only) ---
    isaac_pub_node = None
    if sim:
        try:
            get_package_share_directory("spot_operation_ros2")
            isaac_pub_node = Node(
                package="spot_operation_ros2",
                executable="isaac_publisher",
                output="screen",
                parameters=[{'use_sim_time': True}],
            )
        except Exception:
            isaac_pub_node = ExecuteProcess(
                cmd=['/home/spot_ws/spot-ros2_ws/install/spot_operation_ros2/lib/spot_operation_ros2/isaac_publisher'],
                output='screen',
            )

    # --- joint_state_mapper (sim only) ---
    mapper_node = None
    if sim:
        try:
            get_package_share_directory("spot_operation_ros2")
            mapper_node = Node(
                package="spot_operation_ros2",
                executable="joint_state_mapper",
                output="screen",
                parameters=[{'use_sim_time': True}],
            )
        except Exception:
            mapper_node = ExecuteProcess(
                cmd=['/home/spot_ws/spot-ros2_ws/install/spot_operation_ros2/lib/spot_operation_ros2/joint_state_mapper'],
                output='screen',
            )

    # --- wrist_detector ---
    wrist_detector_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("arm_pose_estimator"),
                "launch",
                "wrist_detector_zed.launch.py",
            ])
        ),
        launch_arguments={
            "use_sim_time": "true" if sim else "false",
        }.items(),
    )

    # --- RViz ---
    rviz_config_file = PathJoinSubstitution([
        FindPackageShare("spot_operation_ros2"),
        "rviz",
        "spot_teleop.rviz",
    ])
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        output="screen",
        arguments=["-d", rviz_config_file],
        parameters=[{'use_sim_time': sim}],
        remappings=remappings,
    )

    # --- Build action groups ---
    sim_nodes = []
    if sim:
        if isaac_pub_node:
            sim_nodes.append(isaac_pub_node)
        if mapper_node:
            sim_nodes.append(mapper_node)
        if robot_state_publisher_node:
            sim_nodes.append(robot_state_publisher_node)
    
    sim_nodes.append(wrist_detector_launch)
    sim_nodes.append(rviz_node)

    sim_group = GroupAction([
        SetParameter(name='use_sim_time', value=sim),
        *sim_nodes,
    ])

    wall_group = GroupAction([
        SetParameter(name='use_sim_time', value=False),
        ros2_control_launch,
    ])

    return [sim_group, wall_group]


def generate_launch_description():
    declare_sim_arg = DeclareLaunchArgument(
        "sim",
        default_value="true",
        description="true = ros2_control mock | false = Spot real via driver",
    )
    
    declare_config_file_arg = DeclareLaunchArgument(
        "config_file",
        default_value="",
        description="YAML com credenciais/ganhos do Spot real (se sim=false)",
    )

    return LaunchDescription([
        declare_sim_arg,
        declare_config_file_arg,
        OpaqueFunction(function=launch_setup),
    ])
