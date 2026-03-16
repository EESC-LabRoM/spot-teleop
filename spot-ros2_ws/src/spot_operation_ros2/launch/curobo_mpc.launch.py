"""
Launch file for cuRobo MPC teleop mode.

Launches:
  - curobo_mpc_node: cuRobo MPC motion planner (uses venv with cuRobo)
  - isaac_publisher (teleop=true): relay to Isaac Sim
  - joint_state_mapper: maps Isaac joints to ROS naming
  - joint_state_remapper: remaps arm0_* to arm_* for robot_state_publisher
  - robot_state_publisher: publishes TF for visualization in RViz
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext, LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.parameter_descriptions import ParameterValue


def launch_setup(context: LaunchContext):
    # Get xacro path for robot_state_publisher - use full Spot with arm
    spot_description_dir = get_package_share_directory('spot_description')
    xacro_file = os.path.join(spot_description_dir, 'urdf', 'spot.urdf.xacro')

    # Config paths
    spot_config_dir = get_package_share_directory('spot_operation_ros2')
    robot_config_path = os.path.join(spot_config_dir, 'config', 'spot_arm.yml')

    # cuRobo MPC Node - Must use venv Python because cuRobo is installed there
    use_sim = LaunchConfiguration('use_sim')
    
    # Build command list - add joint_states remapping when using ros2_control
    # (spot_ros2_control publishes to /low_level/joint_states)
    curobo_cmd = [
            '/home/spot-teleop/spot-ros2_ws/curobo_venv/bin/python',
            '/home/spot-teleop/spot-ros2_ws/src/spot_operation_ros2/spot_operation_ros2/curobo_mpc_node.py',
            '--ros-args',
            '-r', '__node:=curobo_mpc_node',
            '-p', ['control_rate:=', LaunchConfiguration('control_rate')],
            '-p', ['use_effort:=', LaunchConfiguration('use_effort')],
            '-p', ['debug_mode:=', LaunchConfiguration('debug_mode')],
            '-p', ['debug_pose_duration:=', LaunchConfiguration('debug_pose_duration')],
            '-p', ['esdf_topic:=', LaunchConfiguration('esdf_topic')],
            '-p', ['esdf_update_rate:=', LaunchConfiguration('esdf_update_rate')],
            '-p', f'robot_config:={robot_config_path}',
            '-p', f'urdf_path:={xacro_file}',
            '-p', ['use_sim_time:=', LaunchConfiguration('use_sim_time')],
            '-p', ['use_sim:=', use_sim],
            '-p', ['use_nvblox:=', LaunchConfiguration('use_nvblox')],
            '-p', ['static_mesh:=', LaunchConfiguration('static_mesh')],
            '-p', ['mesh_topic:=', LaunchConfiguration('mesh_topic')],
            '-p', ['use_ros2_control:=', LaunchConfiguration('use_ros2_control')],
    ]

    # Remap /joint_states -> /low_level/joint_states when using ros2_control
    use_ros2_control_val = LaunchConfiguration('use_ros2_control').perform(context)
    if use_ros2_control_val.lower() in ('true', '1', 'yes'):
        curobo_cmd.extend(['-r', '/joint_states:=/low_level/joint_states'])

    curobo_mpc_process = ExecuteProcess(
        cmd=curobo_cmd,
        name='curobo_mpc_node',
        output='screen',
        additional_env={
            'PYTHONPATH': ':'.join([
                os.environ.get('PYTHONPATH', ''),
                # Add all ROS prefix paths (including local workspace install folders)
                *[os.path.join(p, 'lib/python3.10/site-packages') for p in os.environ.get('AMENT_PREFIX_PATH', '').split(':')],
                *[os.path.join(p, 'local/lib/python3.10/dist-packages') for p in os.environ.get('AMENT_PREFIX_PATH', '').split(':')],
                '/opt/ros/humble/lib/python3.10/site-packages',
                '/opt/ros/humble/local/lib/python3.10/dist-packages'
            ]),
            'PATH': '/usr/local/cuda-12.8/bin:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin',
            'CUDA_HOME': '/usr/local/cuda-12.8',
            'CUROBO_CONFIG_PATH': os.path.join(spot_config_dir, 'config'),
        }
    )

    # Isaac Publisher in teleop mode
    isaac_publisher_node = Node(
        package='spot_operation_ros2',
        executable='isaac_publisher',
        name='isaac_publisher',
        output='screen',
        parameters=[{'teleop': True, 'use_sim_time': LaunchConfiguration('use_sim_time')}],
        condition=IfCondition(use_sim)
    )

    # Joint State Mapper
    joint_state_mapper_node = Node(
        package='spot_operation_ros2',
        executable='joint_state_mapper',
        name='joint_state_mapper',
        output='screen',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        condition=IfCondition(use_sim)
    )

    # Joint State Remapper - converts arm0_* to arm_* for robot_state_publisher
    joint_state_remapper_node = Node(
        package='spot_operation_ros2',
        executable='joint_state_remapper',
        name='joint_state_remapper',
        output='screen',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        condition=IfCondition(use_sim)
    )

    # Robot State Publisher - publishes TF from URDF + joint states
    # Uses xacro to process the URDF with package:// paths
    # Subscribes to /joint_states_rsp (remapped from arm0_* to arm_*)
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': ParameterValue(
                Command(['xacro ', xacro_file, ' arm:=true']),
                value_type=str
            ),
            'publish_frequency': 50.0,
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }],
        remappings=[
            ('/joint_states', '/joint_states_rsp'),
        ],
        condition=IfCondition(use_sim)
    )

    # Static TF: base -> body (simulation uses 'base', cuRobo expects 'body')
    # Identity transform - they are the same frame with different names
    base_to_body_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_body_tf',
        arguments=['0', '0', '0', '0', '0', '0', 'base', 'body'],
        output='screen',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )

    gripper_remappings = []
    if use_ros2_control_val.lower() in ('true', '1', 'yes'):
        gripper_remappings = [('/joint_states', '/low_level/joint_states')]

    gripper_controller_node = Node(
        package='spot_operation_ros2',
        executable='gripper_controller',
        name='gripper_controller',
        output='screen',
        parameters=[{
            'use_ros2_control': LaunchConfiguration('use_ros2_control'),
        }],
        remappings=gripper_remappings,
    )

    return [
        base_to_body_tf,
        curobo_mpc_process,
        gripper_controller_node,
        isaac_publisher_node,
        joint_state_mapper_node,
        joint_state_remapper_node,
        robot_state_publisher_node
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('control_rate', default_value='30.0',
                              description='MPC control rate in Hz'),
        DeclareLaunchArgument('use_effort', default_value='false',
                              description='Use Pinocchio for effort computation'),
        DeclareLaunchArgument('debug_mode', default_value='false',
                              description='Enable debug mode with test poses (no /wrist_pose required)'),
        DeclareLaunchArgument('debug_pose_duration', default_value='3.0',
                              description='Seconds between pose changes in debug mode'),
        DeclareLaunchArgument('esdf_topic',
                              default_value='/nvblox_node/pessimistic_static_esdf_pointcloud',
                              description='nvblox ESDF pointcloud topic'),
        DeclareLaunchArgument('esdf_update_rate', default_value='2.0',
                              description='Obstacle update rate from ESDF in Hz'),
        DeclareLaunchArgument('use_sim_time', default_value='false',
                              description='Use simulation (Gazebo/Isaac) clock if true'),
        DeclareLaunchArgument('use_sim', default_value='true',
                              description='True for sim (arm0_ joint prefix), False for real robot (arm_ joint prefix)'),
        DeclareLaunchArgument('use_nvblox', default_value='true',
                              description='Enable nvblox mesh subscription for obstacle avoidance'),
        DeclareLaunchArgument('static_mesh', default_value='false',
                              description='If true, freeze mesh after first update. If false, update continuously.'),
        DeclareLaunchArgument('mesh_topic', default_value='/nvblox_node/mesh',
                              description='nvblox mesh topic'),
        DeclareLaunchArgument('use_ros2_control', default_value='false',
                              description='If true, publish JointCommand to spot_joint_controller instead of JointState'),
        OpaqueFunction(function=launch_setup),
    ])
