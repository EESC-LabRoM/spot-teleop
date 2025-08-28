from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_demo_launch
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    urdf_path = os.path.join(
        get_package_share_directory("spot_description"),
        "urdf",
        "spot.urdf.xacro"
    )

    moveit_config = (
        MoveItConfigsBuilder("spot", package_name="spot_moveit_config")
        .robot_description(
            file_path=urdf_path,
            mappings={
                "arm": "true",
                "add_ros2_control_tag": "true",
                "hardware_interface_type": "mock",
                "mock_arm": "true"  # ativa o braço no modo mock
            }
        )
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .to_moveit_configs()
    )

    return generate_demo_launch(moveit_config)
