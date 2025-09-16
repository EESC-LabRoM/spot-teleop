import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    TimerAction,
    ExecuteProcess,
    GroupAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, SetParameter
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder
from launch_ros.parameter_descriptions import ParameterValue
import yaml


def launch_setup(context, *args, **kwargs):
    # As configurações iniciais continuam as mesmas
    sim = LaunchConfiguration("sim").perform(context).lower() in ("true", "1", "yes")
    cfg_file = LaunchConfiguration("config_file").perform(context)
    spot_name = ""
    servo = LaunchConfiguration("servo").perform(context).lower() in ("true", "1", "yes")

    # --- Fase 1: Iniciar o back-end (driver/mock) primeiro ---
    # Esta ação será iniciada imediatamente
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
            "robot_controllers": "arm_controller",
            "control_only": "true" if sim else "false",
        }.items(),
    )

    # --- Fase 2: Preparar as ações do MoveIt, que serão atrasadas ---
    # Primeiro, preparamos as configurações do MoveIt como antes
    moveit_cfg = (
        MoveItConfigsBuilder("spot", package_name="spot_moveit_config")
        .robot_description(
            mappings={"arm": "true", "add_ros2_control_tag": "false"}
        )
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_scene_monitor(
            publish_planning_scene=True,
            publish_geometry_updates=True,
            publish_state_updates=True,
            publish_transforms_updates=True,
        )
        .to_moveit_configs()
    )

    # Agora, criamos os nós do MoveIt
    remappings = [('/joint_states', '/joint_states_mapped')] if sim else []
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[moveit_cfg.robot_description, {'ignore_timestamp': True, 'use_sim_time': True}],
        remappings=remappings,
    )


    # Adiciona de volta o move_group_node pro planning e interactive markers
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_cfg.to_dict(), {'wait_for_complete_state_timeout': 5.0, 'use_sim_time': True}],
        remappings=remappings,
    )

    # Isaac publisher node: publica /joint_states_isaac (installed entrypoint)
    try:
        get_package_share_directory("spot_operation")
        isaac_pub_node = Node(
            package="spot_operation",
            executable="isaac_publisher",
            output="screen",
            parameters=[{'use_sim_time': True}],
        )
    except Exception:
        isaac_pub_node = ExecuteProcess(
            cmd=['/home/spot_ws/spot-ros2_ws/install/spot_operation/lib/spot_operation/isaac_publisher'],
            output='screen',
        )

    # Mapper node: quando em sim, converte /joint_states_isaac -> /joint_states_mapped
    # Prefer launching the installed entrypoint via package lookup; if the package
    # cannot be resolved in this environment, fall back to executing the installed
    # entrypoint binary directly so the launch still works.
    try:
        get_package_share_directory("spot_operation")
        mapper_node = Node(
            package="spot_operation",
            executable="joint_state_mapper",
            output="screen",
            parameters=[{'use_sim_time': True}],
        )
    except Exception:
        mapper_node = ExecuteProcess(
            cmd=['/home/spot_ws/spot-ros2_ws/install/spot_operation/lib/spot_operation/joint_state_mapper'],
            output='screen',
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

    # Use MoveIt Servo pose tracking demo
    servo_node = Node(
        package="moveit_servo",
        executable="servo_pose_tracking_demo",
        name="servo_server",
        output="screen",
        parameters=[
            moveit_cfg.to_dict(),
            servo_params,
            {'use_sim_time': False},
        ],
        remappings=remappings,
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        output="screen",
        arguments=["-d", PathJoinSubstitution([FindPackageShare("spot_moveit_config"), "config", "moveit.rviz"])],
        parameters=[moveit_cfg.to_dict(), {'use_sim_time': True}],
        remappings=remappings,
    )
    
    # Timer para o refresh do RViz (continua importante)
    rviz_refresh_timer = TimerAction(
        period=7.0, # Aumentei um pouco pra dar tempo de tudo subir antes do refresh
        actions=[
            ExecuteProcess(
                cmd=['ros2', 'service', 'call', '/rviz/motion_planning/update_start_state', 'std_srvs/srv/Empty', '{}'],
                output='screen',
            )
        ],
    )

    # Grupo com use_sim_time=True para nós que consomem estado/TF/sensores
    sim_timer_actions = [
        robot_state_publisher_node,
        rviz_node,
        rviz_refresh_timer,
    ]
    if not servo:
        # inserir move_group entre RSP e RViz para manter ordem lógica
        sim_timer_actions.insert(1, move_group_node)

    sim_nodes = [
        TimerAction(period=2.0, actions=sim_timer_actions),
    ]
    
    if sim:
        # /clock e mapper sobem antes do TimerAction
        sim_nodes = [isaac_pub_node, mapper_node] + sim_nodes
    
    sim_group = GroupAction([
        SetParameter(name='use_sim_time', value=True),
        *sim_nodes,
    ])

    # Grupo com use_sim_time=False para nós que publicam comandos em wall time
    wall_actions = [
        SetParameter(name='use_sim_time', value=False),
        ros2_control_launch,   # controller_manager e controladores em wall time
    ]
    if servo:
        wall_actions.append(servo_node)

    wall_group = GroupAction(wall_actions)

    actions = [
        sim_group,
        wall_group,
    ]
    
    return actions



def generate_launch_description():
    # Declare launch arguments
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

    declare_servo_arg = DeclareLaunchArgument(
        "servo",
        default_value="false",
        description="true = usar MoveIt Servo (traj) | false = usar state (plan&execute)",
    )
    
    return LaunchDescription([
        declare_sim_arg,
        declare_config_file_arg,
        declare_servo_arg,
        OpaqueFunction(function=launch_setup),
    ])
