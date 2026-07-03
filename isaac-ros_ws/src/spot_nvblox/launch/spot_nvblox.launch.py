# SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
# Copyright (c) 2024 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# SPDX-License-Identifier: Apache-2.0

"""Launch file para nvblox com Spot.

Câmeras disponíveis: frontleft, frontright, back, hand, left, right
Exemplos:
  cameras:=hand                              (apenas câmera do braço)
  cameras:=frontleft                         (apenas uma câmera frontal)
  cameras:=frontleft,frontright              (duas câmeras frontais — padrão)
  cameras:=frontleft,frontright,hand         (frontais + braço)
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

VALID_CAMERAS = {"frontleft", "frontright", "back", "hand", "left", "right"}


def launch_setup(context, *args, **kwargs):
    cameras_str = LaunchConfiguration("cameras").perform(context)
    camera_names = [c.strip() for c in cameras_str.split(",") if c.strip()]

    invalid = [c for c in camera_names if c not in VALID_CAMERAS]
    if invalid:
        raise RuntimeError(
            f"Câmera(s) inválida(s): {invalid}. Válidas: {sorted(VALID_CAMERAS)}"
        )
    if not camera_names:
        raise RuntimeError("Parâmetro 'cameras' está vazio.")

    sim = LaunchConfiguration("sim").perform(context)
    use_sim_time = LaunchConfiguration("use_sim_time").perform(context)
    use_segmentation = LaunchConfiguration("use_segmentation").perform(context)
    global_frame = LaunchConfiguration("global_frame").perform(context)

    remappings = []
    for i, cam in enumerate(camera_names):
        if sim == "true":
            depth_topic = f"/{cam}/depth"
            color_topic = f"/{cam}/rgb"
            info_topic = f"/{cam}/camera_info"
            depth_info_topic = f"/{cam}/camera_info"
        else:
            depth_topic = f"/depth_registered/{cam}/image"
            color_topic = f"/camera/{cam}/image_rgb"
            info_topic = f"/camera/{cam}/camera_info"
            depth_info_topic = f"/depth_registered/{cam}/camera_info"

        remappings.extend(
            [
                (f"camera_{i}/depth/image", depth_topic),
                (f"camera_{i}/depth/camera_info", depth_info_topic),
                (f"camera_{i}/color/image", color_topic),
                (f"camera_{i}/color/camera_info", info_topic),
            ]
        )

        if use_segmentation == "true":
            if sim == "true":
                mask_topic = f"/{cam}/segmentation_mask"
            else:
                mask_topic = f"/camera/{cam}/segmentation_mask"

            remappings.extend(
                [
                    (f"camera_{i}/mask/image", mask_topic),
                    (f"camera_{i}/mask/camera_info", info_topic),
                ]
            )

    # Parâmetros do nvblox - MODO ALTA PRECISÃO (SAFE)
    nvblox_params = {
        "num_cameras": len(camera_names),
        "global_frame": global_frame,
        "use_tf_transforms": True,
        # --- O SEGREDO DA ALTA RESOLUÇÃO ---
        "voxel_size": 0.02,  # Mantém os 2cm que a gente quer
        "mapping_type": (
            "human_with_static_tsdf" if use_segmentation == "true" else "static_tsdf"
        ),
        "use_segmentation": use_segmentation == "true",
        # --- SEGURA A EMOÇÃO NOS HZ (Isso evita o crash) ---
        # 40Hz é insanidade pra voxel de 2cm.
        "integrate_depth_rate_hz": 5.0,  # 5 FPS é suficiente pra mapear
        "integrate_color_rate_hz": 5.0,
        "update_mesh_rate_hz": 2.0,  # Mesh é pesado, faz devagar
        "update_esdf_rate_hz": 2.0,  # ESDF também pesa
        "publish_layer_rate_hz": 2.0,
        "use_depth": True,
        "use_color": True,
        "use_lidar": False,
        # ESDF
        "esdf_mode": "3d",
        "publish_esdf_distance_slice": True,
        "static_mapper.esdf_slice_min_height": 0.1,  # Ajustado pra braço (não pegar chão)
        "static_mapper.esdf_slice_max_height": 1.5,
        "static_mapper.esdf_slice_height": 0.5,
        "input_qos": "SENSOR_DATA",
        # Limpeza
        "map_clearing_radius_m": 5.0,  # Limpa o que ficar muito longe pra liberar RAM
        "map_clearing_frame_id": "body",
        # Visualização
        "layer_visualization_exclusion_height_m": 100.0,
        "layer_visualization_exclusion_radius_m": 0.0,
        "layer_visualization_min_tsdf_weight": 0.0001,
        "max_back_projection_distance": 7.0,  # Diminuir um pouco
        # --- AQUI TÁ O PULO DO GATO PRA NÃO CRASHAR ---
        # Não integra nada além de 4.5 metros.
        # Pra manipulação vc precisa do que tá na sua cara, não na parede do fundo.
        "static_mapper.projective_integrator_max_integration_distance_m": 4.5,
        # Truncation: 6 voxels * 2cm = 12cm, com margem acima de occupied_region_half_width_m.
        "static_mapper.projective_integrator_truncation_distance_vox": 6.0,
        "dynamic_mapper.projective_integrator_truncation_distance_vox": 6.0,
        "static_mapper.projective_integrator_max_weight": 100.0,
        "static_mapper.mesh_integrator_min_weight": 0.0001,
        # Foreground mapper (regiões mascaradas) — truncation precisa ser >= occupied_region_half_width_m / voxel_size
        # 0.2m / 0.02m = 10 voxels mínimo para silenciar o warning
        "foreground_mapper.projective_integrator_truncation_distance_vox": 10.0,
        "layer_streamer_bandwidth_limit_mbps": -1.0,
        "decay_tsdf_rate_hz": 0.0,
        "print_rates_to_console": True,
        "use_sim_time": use_sim_time == "true",
    }

    nvblox_node = Node(
        package="nvblox_ros",
        executable="nvblox_node",
        name="nvblox_node",
        output="screen",
        parameters=[nvblox_params],
        remappings=remappings,
    )

    # RGB Conversion Nodes (BGR8 -> RGB8) — only for real robot
    rgb_converters = []
    if sim != "true":
        for cam in camera_names:
            rgb_converters.append(
                Node(
                    package="spot_nvblox",
                    executable="republish_rgb.py",
                    name=f"rgb_converter_{cam}",
                    remappings=[
                        ("image_in", f"/camera/{cam}/image"),
                        ("image_out", f"/camera/{cam}/image_rgb"),
                    ],
                    parameters=[{"use_sim_time": use_sim_time == "true"}],
                    output="screen",
                )
            )

    return [*rgb_converters, nvblox_node]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time", default_value="false", description="Use simulation time"
            ),
            DeclareLaunchArgument(
                "global_frame", default_value="vision", description="Global frame for nvblox"
            ),
            DeclareLaunchArgument(
                "sim", default_value="true", description="Whether to use simulation"
            ),
            DeclareLaunchArgument(
                "use_segmentation", default_value="true", description="Use segmentation masks"
            ),
            DeclareLaunchArgument(
                "cameras",
                default_value="frontleft,frontright",
                description=(
                    "Comma-separated list of cameras to use. "
                    f"Valid: {sorted(VALID_CAMERAS)}"
                ),
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
