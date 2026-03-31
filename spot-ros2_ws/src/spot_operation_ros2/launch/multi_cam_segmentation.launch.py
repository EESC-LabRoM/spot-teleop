#!/usr/bin/env python3
"""
Segmentação só na mão (detect_qwen) + um masked_depth_pointcloud_node.

A nuvem do objeto é latched em odom a partir da máscara+mão; o mesmo nó reprojeta para
frontleft e frontright (máscaras sintéticas + depth refinado/reprojetado), sem segundo
detect_qwen nas câmaras frontais.

use_sim_time deve coincidir com o stack (Spot sim + /clock → true; robô real sem /clock → false).
Mistura de relógios causa TF “atrasado” vs stamp das imagens e reprojeção deslocada.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    log_level = DeclareLaunchArgument(
        "log_level", default_value="info", description="Nível de log ROS"
    )
    odom_frame = DeclareLaunchArgument(
        "odom_frame", default_value="odom", description="Frame para latch e TF"
    )
    object_prompt = DeclareLaunchArgument(
        "object_prompt",
        default_value="wheel valve",
        description="Prompt Qwen (câmara mão)",
    )
    robot_base_frame = DeclareLaunchArgument(
        "robot_base_frame",
        default_value="base",
        description="Frame base TF (Spot: base)",
    )
    target_frame = DeclareLaunchArgument(
        "target_frame",
        default_value="target_object",
        description="Frame TF do alvo publicado pelo detect_qwen",
    )
    secondary_reproject_cameras = DeclareLaunchArgument(
        "secondary_reproject_cameras",
        default_value="frontleft,frontright",
        description="Lista separada por vírgulas: reprojetar nuvem odom nestes tópicos",
    )
    idle_tracking_interval = DeclareLaunchArgument(
        "idle_tracking_interval",
        default_value="0.4",
        description="Intervalo SAM2 em idle; reduzir para manter /hand/depth_reprojected atualizando",
    )
    active_tracking_interval = DeclareLaunchArgument(
        "active_tracking_interval",
        default_value="0.2",
        description="Intervalo SAM2 em modo ativo",
    )
    enable_open3d = DeclareLaunchArgument(
        "enable_open3d",
        default_value="false",
        description="Janela Open3D (só visualização da nuvem da mão)",
    )
    use_sim_time = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="true se o sistema publica /clock (Isaac, Gazebo, etc.); alinhar com o driver",
    )

    detect_qwen_node = Node(
        package="spot_operation_ros2",
        executable="detect_qwen",
        name="detect_qwen",
        output="screen",
        parameters=[
            {
                "use_sim_time": ParameterValue(
                    LaunchConfiguration("use_sim_time"), value_type=bool
                ),
                "rgb_topic": "/hand/rgb",
                "depth_topic": "/hand/depth",
                "camera_info_topic": "/hand/camera_info",
                "depth_info_topic": "/hand/camera_info",
                "depth_segmented_topic": "/hand/depth_segmented",
                "segmentation_mask_topic": "/hand/segmentation_mask",
                "target_frame": LaunchConfiguration("target_frame"),
                "object_prompt": LaunchConfiguration("object_prompt"),
                "robot_base_frame": LaunchConfiguration("robot_base_frame"),
                "idle_tracking_interval": LaunchConfiguration("idle_tracking_interval"),
                "active_tracking_interval": LaunchConfiguration("active_tracking_interval"),
            }
        ],
        arguments=["--ros-args", "--log-level", LaunchConfiguration("log_level")],
        emulate_tty=True,
    )

    masked_depth_node = Node(
        package="spot_operation_ros2",
        executable="masked_depth_pointcloud_node",
        name="masked_depth_pointcloud_node",
        output="screen",
        parameters=[
            {
                "use_sim_time": ParameterValue(
                    LaunchConfiguration("use_sim_time"), value_type=bool
                ),
                "mask_topic": "/hand/segmentation_mask",
                "depth_topic": "/hand/depth",
                "camera_info_topic": "/hand/camera_info",
                "pointcloud_topic": "/hand/segmented_pointcloud",
                "reprojected_depth_topic": "/hand/depth_reprojected",
                "refined_depth_topic": "/hand/depth_refined",
                "object_cloud_odom_topic": "/hand/object_cloud_odom",
                "odom_frame": LaunchConfiguration("odom_frame"),
                "secondary_reproject_cameras": LaunchConfiguration(
                    "secondary_reproject_cameras"
                ),
                # Isaac / sim: voxel 0.012 reduz a nuvem a ~80 pts e desloca reprojeção nas câmaras frontais.
                "enable_latch_voxel_downsample": False,
                "enable_visualizer": LaunchConfiguration("enable_open3d"),
                "visualizer_window_title": "Masked PCD (hand)",
            }
        ],
        arguments=["--ros-args", "--log-level", LaunchConfiguration("log_level")],
        emulate_tty=True,
    )

    info = LogInfo(
        msg=(
            "multi_cam_segmentation: 1x detect_qwen (mão) + masked_depth com "
            "secondary_reproject_cameras → máscaras sintéticas em /frontleft|frontright/..."
        )
    )

    return LaunchDescription(
        [
            log_level,
            odom_frame,
            object_prompt,
            robot_base_frame,
            target_frame,
            secondary_reproject_cameras,
            idle_tracking_interval,
            active_tracking_interval,
            enable_open3d,
            use_sim_time,
            info,
            detect_qwen_node,
            masked_depth_node,
        ]
    )
