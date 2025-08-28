# src/arm_pose_estimator/launch/start_arm_pose_estimator.launch.py

from launch import LaunchDescription
from launch.actions import TimerAction, ExecuteProcess
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([

        # 1) Nodo da câmera RealSense
        Node(
            package='realsense2_camera',
            executable='realsense2_camera_node',
            name='realsense2_camera',
            parameters=[{
                'enable_color': True,
                'enable_depth': True,
                'align_depth': True,
                'color_width': 640,
                'color_height': 480,
                'color_fps': 30,
                'depth_width': 640,
                'depth_height': 480,
                'depth_fps': 30,
            }],
            output='screen'
        ),

        # 2) Aguarda 2 segundos para a câmera inicializar
        TimerAction(
            period=2.0,
            actions=[

                # 2.1) Estimador de pose (Python module)
                ExecuteProcess(
                    cmd=['python3', '-m', 'arm_pose_estimator.arm_pose_estimator'],
                    output='screen'
                ),

                # 2.2) Contador de dedos
                ExecuteProcess(
                    cmd=['python3', '-m', 'arm_pose_estimator.finger_count'],
                    output='screen'
                ),

                # 2.3) Orientação da mão
                ExecuteProcess(
                    cmd=['python3', '-m', 'arm_pose_estimator.hand_orientation_estimator'],
                    output='screen'
                ),

                # 2.4) Pose da mão
                ExecuteProcess(
                    cmd=['python3', '-m', 'arm_pose_estimator.hand_pose_estimator'],
                    output='screen'
                ),

            ]
        ),
    ])
