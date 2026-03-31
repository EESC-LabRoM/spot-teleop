from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    executor_threads = LaunchConfiguration("executor_threads")
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
                description="Use simulation clock if true.",
            ),
            DeclareLaunchArgument(
                "executor_threads",
                default_value="3",
                description="Perception node executor thread count.",
            ),
            Node(
                package="spot_operation_ros2",
                executable="sam2_tracker_node",
                name="sam2_tracker_node",
                parameters=[{"use_sim_time": use_sim_time}],
                output="screen",
            ),
            Node(
                package="spot_operation_ros2",
                executable="tf_projection_node",
                name="tf_projection_node",
                parameters=[{"use_sim_time": use_sim_time}],
                output="screen",
            ),
            Node(
                package="spot_operation_ros2",
                executable="roll_image_node",
                name="roll_image_node",
                parameters=[{"use_sim_time": use_sim_time}],
                output="screen",
            ),
            Node(
                package="spot_operation_ros2",
                executable="vlm_relocalize_node",
                name="vlm_relocalize_node",
                parameters=[{"use_sim_time": use_sim_time}],
                output="screen",
            ),
            Node(
                package="spot_operation_ros2",
                executable="coordinator_node",
                name="coordinator_node",
                parameters=[{"use_sim_time": use_sim_time}],
                output="screen",
            ),
        ]
    )

