"""Launch file para nvblox com a FR3 (lab real, ZED 2i fixas), no formato do spot_nvblox.launch.py.

Fusão das câmeras: um camera_i por ZED, mapa no frame da base da FR3 (fr3_link0).
Pré-requisitos no domínio da percepção (ver isaac-ros_ws/src/spot_nvblox/fr3/):
  - TF estático fr3_link0 -> zedN_camera_link (calibração EasyHeC, real_extrinsics_tf.py);
  - fr3_state_relay.py (joint_states + URDF do domínio do robô) + robot_state_publisher;
  - fr3_arm_mask_node.py quando arm_filter:=true (profundidade com o braço = NaN, para o braço não entrar no mapa).
Exemplos:
  cameras:=zed1,zed2                 (frontal + superior, padrão)
  cameras:=zed1 arm_filter:=false    (só a frontal, profundidade crua)
  node_name:=fr3_nvblox              (ao lado de outro nvblox_node: /fr3_nvblox/get_esdf_and_gradient)

Default = the ONE nvblox for the cuRobo on the real FR3: node /nvblox_node (service /nvblox_node/get_esdf_and_gradient, the name the cuRobo
MPC node asks for), fr3_link0 frame, 2 cm voxels, fed with the masked depth (arm + tracked object = NaN). arm_filter:=false = the
raw-depth behaviour of run_nvblox_real.sh (same parameters, raw ZED depth).
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# Same base files as the lab's run_nvblox_real.sh; the explicit values below override them.
NVBLOX_CFG = "/workspaces/isaac_ros-dev/src/isaac_ros_nvblox/nvblox_examples/nvblox_examples_bringup/config/nvblox"

VALID_CAMERAS = {"zed1", "zed2", "zed3"}


def launch_setup(context, *args, **kwargs):
    camera_names = [c.strip() for c in LaunchConfiguration("cameras").perform(context).split(",") if c.strip()]
    invalid = [c for c in camera_names if c not in VALID_CAMERAS]
    if invalid:
        raise RuntimeError(f"Câmera(s) inválida(s): {invalid}. Válidas: {sorted(VALID_CAMERAS)}")
    if not camera_names:
        raise RuntimeError("Parâmetro 'cameras' está vazio.")

    arm_filter = LaunchConfiguration("arm_filter").perform(context) == "true"
    global_frame = LaunchConfiguration("global_frame").perform(context)

    remappings = []
    for i, cam in enumerate(camera_names):
        depth_ns = f"/{cam}/arm_filter/depth" if arm_filter else f"/{cam}/zed_node/depth/depth_registered"
        depth_info = f"/{cam}/arm_filter/depth/camera_info" if arm_filter else f"/{cam}/zed_node/depth/camera_info"
        remappings.extend(
            [
                (f"camera_{i}/depth/image", depth_ns),
                (f"camera_{i}/depth/camera_info", depth_info),
                (f"camera_{i}/color/image", f"/{cam}/zed_node/rgb/image_rect_color"),
                (f"camera_{i}/color/camera_info", f"/{cam}/zed_node/rgb/camera_info"),
            ]
        )

    # Mesmos valores do spot_nvblox (voxel 2 cm, 5 Hz, esdf 3d), com o volume da bancada da FR3
    nvblox_params = {
        "num_cameras": len(camera_names),
        "global_frame": global_frame,
        "pose_frame": global_frame,
        "use_tf_transforms": True,
        "voxel_size": 0.02,
        "mapping_type": "static_tsdf",
        "use_segmentation": False,
        "integrate_depth_rate_hz": 5.0,
        "integrate_color_rate_hz": 5.0,
        "update_mesh_rate_hz": 2.0,
        "update_esdf_rate_hz": 2.0,
        "use_depth": True,
        "use_color": True,
        "use_lidar": False,
        "esdf_mode": "3d",
        "static_mapper.esdf_slice_min_height": 0.02,
        "static_mapper.esdf_slice_max_height": 0.8,
        "static_mapper.esdf_slice_height": 0.15,
        "input_qos": "SENSOR_DATA",
        # Base fixa: nada de limpeza por distância
        "map_clearing_frame_id": global_frame,
        "map_clearing_radius_m": -1.0,
        "esdf_slice_bounds_visualization_attachment_frame_id": global_frame,
        "workspace_height_bounds_visualization_attachment_frame_id": global_frame,
        "max_back_projection_distance": 7.0,
        # A bancada está a < 2.5 m das câmeras
        "static_mapper.projective_integrator_max_integration_distance_m": 2.5,
        "static_mapper.projective_integrator_truncation_distance_vox": 6.0,
        "layer_streamer_bandwidth_limit_mbps": -1.0,
        "decay_tsdf_rate_hz": 0.0,
        "print_rates_to_console": True,
    }

    nvblox_node = Node(
        package="nvblox_ros",
        executable="nvblox_node",
        name=LaunchConfiguration("node_name").perform(context),
        output="screen",
        parameters=[f"{NVBLOX_CFG}/nvblox_base.yaml", f"{NVBLOX_CFG}/specializations/nvblox_zed.yaml", nvblox_params],
        remappings=remappings,
    )
    return [nvblox_node]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("global_frame", default_value="fr3_link0", description="Global frame for nvblox (FR3 base)"),
            DeclareLaunchArgument(
                "node_name",
                default_value="nvblox_node",
                description="Node name; its topics and the ESDF service live under it. nvblox_node = the name cuRobo asks for (stop any other nvblox_node first)",
            ),
            DeclareLaunchArgument(
                "arm_filter",
                default_value="true",
                description="true (default): integrate /<cam>/arm_filter/depth (arm + tracked object masked out by fr3_arm_mask_node.py). false: raw ZED depth",
            ),
            DeclareLaunchArgument(
                "cameras",
                default_value="zed1,zed2",
                description=f"Comma-separated list of cameras to use. Valid: {sorted(VALID_CAMERAS)}",
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
