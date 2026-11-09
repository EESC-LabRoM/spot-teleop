# REAL mapping parameters (as the lab's nvblox_5cm.launch.py of 2026-10-23, Hipolito's joystick MPC map); generated from franka_zed_nvblox.launch.py 2026-10-27
# nvblox for the Franka workcell: one or two external ZEDs (the zed_lite wrappers' topics), static scene, ESDF in the robot base
# frame for cuRobo (service /nvblox_node/get_esdf_and_gradient, as on Spot). Parameters follow spot_nvblox.launch.py
# (2 cm voxels, static TSDF, 3D ESDF, 5 Hz integration); Spot-only parts dropped (masks, hand clip, body map clearing).
#   ros2 launch /ws/nvblox/franka_zed_nvblox.launch.py [global_frame:=fr3_link0] [self_filter:=true] [cameras:=zed,zed2]
#   (TF: zed_extrinsics_tf.py per camera; robot-free depth per camera from franka_segmenter.py: /franka_seg/depth for zed,
#   /franka_seg_<name>/depth for the others)
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def setup(context):
    frame = LaunchConfiguration("global_frame").perform(context)
    cams = [c for c in LaunchConfiguration("cameras").perform(context).split(",") if c]
    seg = LaunchConfiguration("self_filter").perform(context) == "true"
    mask = LaunchConfiguration("mask").perform(context) == "true"
    nt = LaunchConfiguration("filter_target").perform(context) == "true"      # depth with the target removed (target_mask.py --mode filter)
    params = {
        "num_cameras": len(cams), "global_frame": frame, "use_tf_transforms": True,
        "voxel_size": float(LaunchConfiguration("voxel_size").perform(context)), "mapping_type": "human_with_static_tsdf" if mask else "static_tsdf", "use_segmentation": mask,
        "integrate_depth_rate_hz": 5.0, "maximum_input_queue_length": 3, "static_mapper.do_depth_preprocessing": False, "static_mapper.esdf_integrator_min_weight": 0.5, "integrate_color_rate_hz": 2.0,
        "update_mesh_rate_hz": 1.0, "update_esdf_rate_hz": 5.0, "publish_layer_rate_hz": 1.0,
        "use_depth": True, "use_color": True, "use_lidar": False,
        "esdf_mode": "3d", "publish_esdf_distance_slice": True,
        "static_mapper.esdf_slice_min_height": 0.02, "static_mapper.esdf_slice_max_height": 0.8,
        "static_mapper.esdf_slice_height": 0.15,                # in the base frame: 15 cm above the table top
        "input_qos": "SENSOR_DATA",
        "map_clearing_radius_m": -1.0,                          # fixed camera, small scene: keep everything
        "map_clearing_frame_id": frame, "esdf_slice_bounds_visualization_attachment_frame_id": frame,
        "workspace_height_bounds_visualization_attachment_frame_id": frame, "pose_frame": frame,  # defaults: base_link
        "layer_visualization_exclusion_height_m": 100.0, "layer_visualization_exclusion_radius_m": 0.0,
        "layer_visualization_min_tsdf_weight": 0.0001, "max_back_projection_distance": 7.0,
        "static_mapper.projective_integrator_max_integration_distance_m": 2.5,   # ZED is 1.0-1.6 m from the table
        "static_mapper.projective_integrator_truncation_distance_vox": 4.0,
        "static_mapper.projective_integrator_max_weight": 10.0, "static_mapper.mesh_integrator_min_weight": 0.5,
        # mask:=true: the paper's setup (spot_nvblox.launch.py): masked pixels (the grasp target) -> the dynamic/human layer, the rest -> the
        # static TSDF the ESDF comes from. remove_small_connected_components MUST stay on: nvblox 3.2's ImageMasker splits depth by the mask
        # on its OWN CUDA stream, and the only sync with the node stream (where this frame's depth + mask were just uploaded) is the
        # cuda_stream_->synchronize() inside MaskPreprocessor::removeSmallConnectedComponents. Turned off (as here until 10-15), the split
        # sometimes reads the PREVIOUS frame's mask, i.e. the other camera's: measured 6/24 zed2 frames split with zed's mask, and the
        # target was back in the static map (898 voxels, run mask_2cam_noocc). Threshold 0 = that path is a plain copy + sync (the mask
        # is not altered; > 0 would downscale it and delete blobs under the threshold, default 2000 px as in the paper's release).
        **({"multi_mapper.remove_small_connected_components": True, "multi_mapper.connected_mask_component_size_threshold": 0,
            # as spot_nvblox.launch.py (paper release): the dynamic/foreground mappers' truncation (>= occupied_region_half_width_m / voxel)
            "dynamic_mapper.projective_integrator_truncation_distance_vox": 6.0, "foreground_mapper.projective_integrator_truncation_distance_vox": 10.0} if mask else {}),
        # nvblox's CUDA stream: 1 = its own blocking stream (default), 0 = the legacy default stream. 0 makes the human-mode mask split
        # (ImageMasker, which runs on a SEPARATE stream of its own) wait for the node's async depth/mask uploads; test 10-15 for the
        # 2-camera leak (each camera alone keeps the target out of the static map, both together do not)
        "cuda_stream_type": int(LaunchConfiguration("cuda_stream_type").perform(context)),
        "layer_streamer_bandwidth_limit_mbps": -1.0, "decay_tsdf_rate_hz": 0.0, "print_rates_to_console": True,
    }
    remaps = []
    for i, c in enumerate(cams):
        z = f"/{c}/zed_node"
        depth = f"/{c}/depth_no_target" if nt else (("/franka_seg/depth" if c == "zed" else f"/franka_seg_{c}/depth") if seg else f"{z}/depth/depth_registered")
        remaps += [(f"camera_{i}/depth/image", depth), (f"camera_{i}/depth/camera_info", f"{z}/depth/camera_info"),
                   (f"camera_{i}/color/image", f"{z}/rgb/color/rect/image"), (f"camera_{i}/color/camera_info", f"{z}/rgb/color/rect/camera_info")]
        if mask:                                               # target_mask.py: one mask per depth frame, same header, in /<cam>/target_mask
            remaps += [(f"camera_{i}/mask/image", f"/{c}/target_mask{LaunchConfiguration('mask_suffix').perform(context)}"), (f"camera_{i}/mask/camera_info", f"{z}/depth/camera_info")]
    return [Node(package="nvblox_ros", executable="nvblox_node", name=LaunchConfiguration("node_name").perform(context), output="screen",
                 parameters=[params], remappings=remaps)]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("global_frame", default_value="fr3_link0", description="nvblox global frame (robot base; FR3)"),
        DeclareLaunchArgument("voxel_size", default_value="0.02", description="TSDF/ESDF voxel (m)"),
        DeclareLaunchArgument("node_name", default_value="nvblox_node", description="a second instance needs its own name"),
        DeclareLaunchArgument("cameras", default_value="zed", description="comma list of ZED camera names (zed, zed2)"),
        DeclareLaunchArgument("mask", default_value="false",
                              description="true: human_with_static_tsdf, the target's mask (/<cam>/target_mask) keeps it out of the static map"),
        DeclareLaunchArgument("filter_target", default_value="false", description="true: integrate /<cam>/depth_no_target (the target's pixels removed by target_mask.py --mode filter)"),
        DeclareLaunchArgument("mask_suffix", default_value="", description="mask topic /<cam>/target_mask<suffix> (target_mask.py --extra-dilate variants: _d12)"),
        DeclareLaunchArgument("cuda_stream_type", default_value="1", description="nvblox node CUDA stream: 0 legacy default, 1 blocking (nvblox default), 2 non-blocking, 3 per-thread"),
        DeclareLaunchArgument("self_filter", default_value="true",
                              description="true: integrate franka_segmenter.py's robot-free depth; false: the raw ZED depth"),
        OpaqueFunction(function=setup)])
