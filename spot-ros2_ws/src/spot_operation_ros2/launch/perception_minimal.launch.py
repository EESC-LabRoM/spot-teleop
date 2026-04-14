from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    executor_threads = LaunchConfiguration("executor_threads")
    secondary_cameras = LaunchConfiguration("secondary_cameras")
    
    object_prompt = LaunchConfiguration("object_prompt")
    sim = LaunchConfiguration("sim")
    rgb_topic = LaunchConfiguration("rgb_topic")
    camera_info_topic = LaunchConfiguration("camera_info_topic")
    depth_topic = LaunchConfiguration("depth_topic")
    depth_info_topic = LaunchConfiguration("depth_info_topic")
    secondary_rgb_topic_pattern = LaunchConfiguration("secondary_rgb_topic_pattern")
    secondary_camera_info_topic_pattern = LaunchConfiguration("secondary_camera_info_topic_pattern")
    
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "object_prompt",
                default_value="wheel valve",
                description="Object to detect with the VLM.",
            ),
            DeclareLaunchArgument(
                "sim",
                default_value="true",
                description="Use simulation topics and configurations.",
            ),
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
            DeclareLaunchArgument(
                "secondary_cameras",
                default_value="frontleft,frontright",
                description="CSV of secondary camera names for SAM2 tracking. Set to '' to disable.",
            ),
            DeclareLaunchArgument(
                "rgb_topic",
                default_value=PythonExpression(["'/hand/rgb' if '", sim, "' == 'true' else '/camera/hand/image'"]),
                description="Topic for RGB image.",
            ),
            DeclareLaunchArgument(
                "camera_info_topic",
                default_value=PythonExpression(["'/hand/camera_info' if '", sim, "' == 'true' else '/camera/hand/camera_info'"]),
                description="Topic for RGB camera info.",
            ),
            DeclareLaunchArgument(
                "depth_topic",
                default_value=PythonExpression(["'/hand/depth' if '", sim, "' == 'true' else '/depth_registered/hand/image'"]),
                description="Topic for depth image.",
            ),
            DeclareLaunchArgument(
                "depth_info_topic",
                default_value=PythonExpression(["'/hand/camera_info' if '", sim, "' == 'true' else '/depth_registered/hand/camera_info'"]),
                description="Topic for depth camera info.",
            ),
            DeclareLaunchArgument(
                "secondary_rgb_topic_pattern",
                default_value=PythonExpression(["'/{cam}/rgb' if '", sim, "' == 'true' else '/camera/{cam}/image'"]),
                description="Pattern for secondary RGB topics. Use {cam} placeholder.",
            ),
            DeclareLaunchArgument(
                "secondary_camera_info_topic_pattern",
                default_value=PythonExpression(["'/{cam}/camera_info' if '", sim, "' == 'true' else '/camera/{cam}/camera_info'"]),
                description="Pattern for secondary camera_info topics. Use {cam} placeholder.",
            ),
            Node(
                package="spot_operation_ros2",
                executable="sam2_tracker_node",
                name="sam2_tracker_node",
                parameters=[{
                    "use_sim_time": use_sim_time,
                    "secondary_cameras": secondary_cameras,
                    "rgb_topic": rgb_topic,
                    "depth_topic": depth_topic,
                    "depth_info_topic": depth_info_topic,
                    "secondary_rgb_topic_pattern": secondary_rgb_topic_pattern,
                }],
                output="screen",
            ),
            Node(
                package="spot_operation_ros2",
                executable="tf_projection_node",
                name="tf_projection_node",
                parameters=[{
                    "use_sim_time": use_sim_time,
                    "secondary_cameras": secondary_cameras,
                    "camera_info_topic": camera_info_topic,
                    "secondary_camera_info_topic_pattern": secondary_camera_info_topic_pattern,
                    "target_parent_frame": "vision",
                }],
                output="screen",
            ),
            Node(
                package="spot_operation_ros2",
                executable="roll_image_node",
                name="roll_image_node",
                parameters=[{
                    "use_sim_time": use_sim_time,
                    "rgb_topic": rgb_topic,
                    "camera_info_topic": camera_info_topic,
                }],
                output="screen",
            ),
            Node(
                package="spot_operation_ros2",
                executable="vlm_relocalize_node",
                name="vlm_relocalize_node",
                parameters=[{
                    "use_sim_time": use_sim_time,
                    "object_prompt": object_prompt,
                }],
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

