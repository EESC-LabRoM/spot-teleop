from setuptools import setup
import os
from glob import glob

package_name = 'arm_pose_estimator'

setup(
    name=package_name,
    version='0.1.0',
    # lista os pacotes Python que existem sob a pasta arm_pose_estimator/
    packages=[package_name],
    # agora sem package_dir!
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name,
         ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
         glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'models'),
         glob('models/*')),
    ],
    install_requires=[
        'setuptools',
        'rclpy',
        'opencv-python',
        'opencv-contrib-python',
        'numpy',
        'cv_bridge',
        'message_filters',
        'mediapipe',
        'tf2_ros',
        'geometry_msgs',
        'sensor_msgs',
    ],
    zip_safe=True,
    maintainer='Matheus Hipólito',
    maintainer_email='seu.email@exemplo.com',
    description='Estimador de pose de braço para ROS 2',
    license='Apache License 2.0',
    entry_points={
        'console_scripts': [
            'arm_pose_estimator = arm_pose_estimator.realsense_pose_node:main',
            'hand_pose_estimator = arm_pose_estimator.hand_pose_estimator:main',
            'hand_orientation_estimator = arm_pose_estimator.hand_orientation_estimator:main',
            'finger_count = arm_pose_estimator.finger_count:main',
        ],
    },
)
