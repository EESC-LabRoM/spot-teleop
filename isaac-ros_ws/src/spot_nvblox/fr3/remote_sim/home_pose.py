"""Franka FR3 lab HOME configuration ('Paiva' home) and exact MuJoCo FK TCP pose.

Pure constants module (no ROS or MuJoCo imports).
"""
import math
import numpy as np

HOME_Q = [
    0.0,
    -math.pi / 4.0,
    0.0,
    -3.0 * math.pi / 4.0,
    0.0,
    math.pi / 2.0,
    math.pi / 4.0,
]
HOME_FINGER = 0.04

# Exact FK of MujocoArm.tcp_pose() at HOME_Q read from mujoco/workcell.xml in fr3_link0 frame:
# TCP position (m) and fr3_hand link quaternion in xyzw order (tool pointing straight down).
HOME_TCP_P = np.array(
    [0.30689056659294095, 1.814615319072546e-33, 0.4868820523028392],
    dtype=np.float64,
)
HOME_TCP_Q = np.array(
    [
        0.9999999999999994,
        -1.7459913959072537e-08,
        -7.850462293418873e-17,
        1.3706839562344069e-24,
    ],
    dtype=np.float64,
)
