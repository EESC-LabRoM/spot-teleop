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

"""
Launch file para nvblox com 2 câmeras do Boston Dynamics Spot.
Câmeras: frontleft, frontright
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch.conditions import UnlessCondition
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    
    # Argumentos
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )
    
    global_frame_arg = DeclareLaunchArgument(
        'global_frame',
        default_value='odom',
        description='Global frame for nvblox'
    )

    sim_arg = DeclareLaunchArgument(
        'sim',
        default_value='true',
        description='Whether to use simulation'
    )

    # Launch Configurations
    sim = LaunchConfiguration('sim')
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Remappings das câmeras do Spot para o formato do nvblox
    # Sim: /{camera}/rgb, /{camera}/depth, /{camera}/camera_info
    # Real: /camera/{camera}/image, /depth/{camera}/image, /camera/{camera}/camera_info
    camera_names = ['frontleft', 'frontright']
    remappings = []
    for i, cam in enumerate(camera_names):
        # Simulation topics
        sim_depth = f'/{cam}/depth'
        sim_rgb = f'/{cam}/rgb'
        sim_info = f'/{cam}/camera_info'

        # Real topics (from user ros2 topic list)
        real_depth = f'/depth_registered/{cam}/image'
        real_rgb = f'/camera/{cam}/image_rgb' # Converted to RGB8
        real_rgb_info = f'/camera/{cam}/camera_info'
        real_depth_info = f'/depth_registered/{cam}/camera_info'



        remappings.extend([
            (f'camera_{i}/depth/image', PythonExpression([f"'{sim_depth}' if '", sim, "' == 'true' else '", real_depth, "'"])),
            (f'camera_{i}/depth/camera_info', PythonExpression([f"'{sim_info}' if '", sim, "' == 'true' else '", real_depth_info, "'"])),
            (f'camera_{i}/color/image', PythonExpression([f"'{sim_rgb}' if '", sim, "' == 'true' else '", real_rgb, "'"])),
            (f'camera_{i}/color/camera_info', PythonExpression([f"'{sim_info}' if '", sim, "' == 'true' else '", real_rgb_info, "'"]))
        ])

    # Parâmetros do nvblox - MODO ALTA PRECISÃO (SAFE)
    nvblox_params = {
        # Número de câmeras
        'num_cameras': 2,
        
        'global_frame': LaunchConfiguration('global_frame'),
        'use_tf_transforms': True,
        
        # --- O SEGREDO DA ALTA RESOLUÇÃO ---
        'voxel_size': 0.02,  # Mantém os 2cm que a gente quer
        
        'mapping_type': 'static_tsdf',
        
        # --- SEGURA A EMOÇÃO NOS HZ (Isso evita o crash) ---
        # 40Hz é insanidade pra voxel de 2cm.
        'integrate_depth_rate_hz': 5.0,   # 5 FPS é suficiente pra mapear
        'integrate_color_rate_hz': 5.0,
        'update_mesh_rate_hz': 2.0,       # Mesh é pesado, faz devagar
        'update_esdf_rate_hz': 2.0,       # ESDF também pesa
        'publish_layer_rate_hz': 2.0,
        
        'use_depth': True,
        'use_color': True,
        'use_lidar': False,
        
        # ESDF
        'esdf_mode': '2d',
        'publish_esdf_distance_slice': True,
        'static_mapper.esdf_slice_min_height': 0.1,  # Ajustado pra braço (não pegar chão)
        'static_mapper.esdf_slice_max_height': 1.5,
        'static_mapper.esdf_slice_height': 0.5,
        
        'input_qos': 'SENSOR_DATA',
        
        # Limpeza
        'map_clearing_radius_m': 5.0,  # Limpa o que ficar muito longe pra liberar RAM
        'map_clearing_frame_id': 'body',
        
        # Visualização
        'layer_visualization_exclusion_height_m': 100.0,
        'layer_visualization_exclusion_radius_m': 0.0,
        'layer_visualization_min_tsdf_weight': 0.0001,
        'max_back_projection_distance': 7.0,  # Diminuir um pouco
        
        # --- AQUI TÁ O PULO DO GATO PRA NÃO CRASHAR ---
        # Não integra nada além de 4.5 metros.
        # Pra manipulação vc precisa do que tá na sua cara, não na parede do fundo.
        'static_mapper.projective_integrator_max_integration_distance_m': 4.5,
        
        # Truncation: 4 voxels * 2cm = 8cm de banda. Suficiente e economiza RAM.
        'static_mapper.projective_integrator_truncation_distance_vox': 4.0,
        
        'static_mapper.projective_integrator_max_weight': 100.0,
        'static_mapper.mesh_integrator_min_weight': 0.0001,
        
        'layer_streamer_bandwidth_limit_mbps': -1.0,
        'decay_tsdf_rate_hz': 0.0,
        'print_rates_to_console': True,
        'use_sim_time': LaunchConfiguration('use_sim_time'),
    }

    nvblox_node = Node(
        package='nvblox_ros',
        executable='nvblox_node',
        name='nvblox_node',
        output='screen',
        parameters=[nvblox_params],
        remappings=remappings,
    )



    # RGB Conversion Nodes (BGR8 -> RGB8) for real cameras
    rgb_converters = []
    for cam in camera_names:
        rgb_converters.append(
            Node(
                package='spot_nvblox',
                executable='republish_rgb.py',
                name=f'rgb_converter_{cam}',
                remappings=[
                    ('image_in', f'/camera/{cam}/image'),
                    ('image_out', f'/camera/{cam}/image_rgb'),
                ],
                parameters=[{'use_sim_time': use_sim_time}],
                condition=UnlessCondition(sim),
                output='screen'
            )
        )

    return LaunchDescription([
        use_sim_time_arg,
        sim_arg,
        global_frame_arg,

        *rgb_converters,
        nvblox_node,
    ])
