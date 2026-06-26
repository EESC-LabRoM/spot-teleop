from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    use_secondaries = LaunchConfiguration("use_secondaries")
    secondary_cameras = LaunchConfiguration("secondary_cameras")
    
    object_prompt = LaunchConfiguration("object_prompt")
    sim = LaunchConfiguration("sim")
    rgb_topic = LaunchConfiguration("rgb_topic")
    camera_info_topic = LaunchConfiguration("camera_info_topic")
    depth_topic = LaunchConfiguration("depth_topic")
    depth_info_topic = LaunchConfiguration("depth_info_topic")
    secondary_rgb_topic_pattern = LaunchConfiguration("secondary_rgb_topic_pattern")
    secondary_depth_topic_pattern = LaunchConfiguration("secondary_depth_topic_pattern")
    secondary_camera_info_topic_pattern = LaunchConfiguration("secondary_camera_info_topic_pattern")
    secondary_mask_topic_pattern = LaunchConfiguration("secondary_mask_topic_pattern")
    segmentation_mask_topic = LaunchConfiguration("segmentation_mask_topic")
    target_parent_frame = LaunchConfiguration("target_parent_frame")

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
                "use_secondaries",
                default_value="true",
                description="Enable secondary-camera SAM2 tracking. false → hand-only (frees GPU).",
            ),
            DeclareLaunchArgument(
                "visualize",
                default_value="true",
                description="Show cv2 preview windows. false → headless (removes GUI/GIL overhead).",
            ),
            DeclareLaunchArgument(
                "secondary_cameras",
                default_value=PythonExpression(
                    ["'frontleft,frontright' if '", use_secondaries, "' == 'true' else ''"]
                ),
                description="CSV of secondary camera names for SAM2 tracking. Empty → disabled.",
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
                "secondary_depth_topic_pattern",
                default_value=PythonExpression(["'/{cam}/depth' if '", sim, "' == 'true' else '/depth_registered/{cam}/image'"]),
                description="Pattern for secondary depth topics (for the seed depth gate). Use {cam} placeholder.",
            ),
            DeclareLaunchArgument(
                "secondary_camera_info_topic_pattern",
                default_value=PythonExpression(["'/{cam}/camera_info' if '", sim, "' == 'true' else '/camera/{cam}/camera_info'"]),
                description="Pattern for secondary camera_info topics. Use {cam} placeholder.",
            ),
            DeclareLaunchArgument(
                "target_parent_frame",
                default_value="world",
                description="Parent TF frame for the projected target_object transform.",
            ),
            DeclareLaunchArgument(
                "secondary_mask_topic_pattern",
                default_value=PythonExpression(["'/{cam}/segmentation_mask' if '", sim, "' == 'true' else '/camera/{cam}/segmentation_mask'"]),
                description="Pattern for secondary segmentation mask topics. Use {cam} placeholder.",
            ),
            DeclareLaunchArgument(
                "segmentation_mask_topic",
                default_value=PythonExpression(["'/hand/segmentation_mask' if '", sim, "' == 'true' else '/camera/hand/segmentation_mask'"]),
                description="Topic for the HAND segmentation mask. Must match where nvblox subscribes the hand mask (real: /camera/hand/segmentation_mask).",
            ),
            DeclareLaunchArgument(
                "mask_dilation_px",
                default_value="7",
                description=(
                    "Dilate the mask sent to nvblox by N px so the object's rim is fully "
                    "covered and doesn't fuse into the static TSDF (which makes cuRobo "
                    "avoid the target). Keep tight (rim width); 0 = off."
                ),
            ),
            DeclareLaunchArgument(
                "hand_deroll_enabled",
                default_value="true",
                description="De-roll the hand frame to gravity-upright before SAM2 so the "
                            "video predictor holds lock through wrist roll. false = native.",
            ),
            DeclareLaunchArgument(
                "hand_deroll_min_deg",
                default_value="3.0",
                description="Deadband (deg): below this roll the hand frame is fed native (no warp).",
            ),
            Node(
                package="spot_operation_ros2",
                executable="sam2_tracker_node",
                name="sam2_tracker_node",
                parameters=[{
                    "use_sim_time": use_sim_time,
                    "mask_dilation_px": ParameterValue(
                        LaunchConfiguration("mask_dilation_px"), value_type=int),
                    "hand_deroll_enabled": ParameterValue(
                        LaunchConfiguration("hand_deroll_enabled"), value_type=bool),
                    "hand_deroll_min_deg": ParameterValue(
                        LaunchConfiguration("hand_deroll_min_deg"), value_type=float),
                    "secondary_cameras": ParameterValue(secondary_cameras, value_type=str),
                    "rgb_topic": rgb_topic,
                    "depth_topic": depth_topic,
                    "depth_info_topic": depth_info_topic,
                    "secondary_rgb_topic_pattern": secondary_rgb_topic_pattern,
                    "secondary_depth_topic_pattern": secondary_depth_topic_pattern,
                    "secondary_mask_topic_pattern": secondary_mask_topic_pattern,
                    "segmentation_mask_topic": segmentation_mask_topic,
                    "secondary_camera_info_topic_pattern": secondary_camera_info_topic_pattern,
                    "target_object_frame": "target_object",
                    "visualize": LaunchConfiguration("visualize"),
                }],
                output="screen",
            ),
            Node(
                package="spot_operation_ros2",
                executable="tf_projection_node",
                name="tf_projection_node",
                parameters=[{
                    "use_sim_time": use_sim_time,
                    "secondary_cameras": ParameterValue(secondary_cameras, value_type=str),
                    "camera_info_topic": camera_info_topic,
                    "secondary_camera_info_topic_pattern": secondary_camera_info_topic_pattern,
                    "target_parent_frame": target_parent_frame,
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
                    "rgb_topic": rgb_topic,
                    "camera_info_topic": camera_info_topic,
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

