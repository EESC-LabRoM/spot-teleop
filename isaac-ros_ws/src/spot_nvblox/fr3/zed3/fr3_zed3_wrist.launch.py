"""Operator-wrist perception on zed3 for the FR3 (perception only; commands nothing).

Runs the Spot branch's arm_pose_estimator nodes from source (no colcon install needed) with the FR3 params in fr3_operator_wrist.yaml:
  wrist_detector            zed3 rgb+depth -> /fr3_operator/wrist_pose (PoseStamped, frame fr3_link0) + TF fr3_link0 -> wrist_target
  hand_orientation_estimator  (hand_orientation:=true) zed3 rgb -> /hand_roll_quat, which wrist_detector puts in the pose orientation
The pose topic is remapped away from /wrist_pose (the Spot curobo_mpc_node's input) on purpose.
  ros2 launch fr3_zed3_wrist.launch.py [params_file:=...] [hand_orientation:=true] [pose_topic:=/fr3_operator/wrist_pose]
Needs mediapipe on PYTHONPATH (start_zed3_wrist.sh handles it) and the zed3 wrapper up in the same ROS domain.
"""
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration

HERE = os.path.dirname(os.path.abspath(__file__))


def _node(module, name, params, remaps=()):
    cmd = ["python3", "-m", module, "--ros-args", "-r", f"__node:={name}", "--params-file", params]
    for a, b in remaps:
        cmd += ["-r", f"{a}:={b}"]
    return cmd


def generate_launch_description():
    params = LaunchConfiguration("params_file")
    return LaunchDescription(
        [
            DeclareLaunchArgument("params_file", default_value=os.path.join(HERE, "fr3_operator_wrist.yaml"),
                                  description="FR3 params for wrist_detector / hand_orientation_estimator (zed3 topics, FR3 scale/offset)"),
            DeclareLaunchArgument("pose_topic", default_value="/fr3_operator/wrist_pose",
                                  description="Output PoseStamped topic (NOT /wrist_pose: that is the Spot MPC's input)"),
            DeclareLaunchArgument("hand_orientation", default_value="false",
                                  description="Legacy roll only: also run hand_orientation_estimator (hand roll from MediaPipe Hands)"),
            ExecuteProcess(
                cmd=["python3", "-m", "arm_pose_estimator.wrist_detector", "--ros-args", "-r", "__node:=wrist_detector",
                     "--params-file", params, "-r", ["/wrist_pose:=", LaunchConfiguration("pose_topic")]],
                name="wrist_detector", output="screen"),
            ExecuteProcess(
                cmd=_node("arm_pose_estimator.hand_orientation_estimator", "hand_orientation_estimator", params),
                name="hand_orientation_estimator", output="screen", condition=IfCondition(LaunchConfiguration("hand_orientation"))),
        ]
    )
