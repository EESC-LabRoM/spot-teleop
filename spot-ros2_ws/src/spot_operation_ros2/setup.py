from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'spot_operation_ros2'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=[
        'setuptools',
        'opencv-python',
        'ultralytics',
        'numpy',
    ],
    zip_safe=True,
    maintainer='root',
    maintainer_email='murilo.mv4321@usp.br',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'isaac_publisher = spot_operation_ros2.isaac_publisher:main',
            'joint_state_mapper = spot_operation_ros2.joint_state_mapper:main',
            'interactive_pose_marker = spot_operation_ros2.interactive_pose_marker:main',
            'test_servo_joint_publisher = spot_operation_ros2.test_servo_joint_publisher:main',
            'detect_valve = spot_operation_ros2.detect_valve:main',
            'grasp_valve = spot_operation_ros2.grasp_valve:main',
            'gripper_controller = spot_operation_ros2.gripper_controller:main',
        ],
    },
)
