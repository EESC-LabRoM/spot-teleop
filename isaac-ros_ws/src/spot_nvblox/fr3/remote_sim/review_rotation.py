"""Signed review angles from recorded hand and measured robot quaternions."""
import math
import numpy as np
from cartesian_review import projected_angle
from teleop_mapping import _quat_mul


def profile_for_label(label):
    if 'Inclinar dedos' in label:
        return 'pitch'
    if 'Virar a palma' in label:
        return 'yaw'
    return None


def hand_angle(q0, q, profile):
    q0 = np.asarray(q0, float)
    delta = _quat_mul(q, np.r_[-q0[:3], q0[3]])
    axis = np.eye(3)[1 if profile == 'pitch' else 2]
    return math.degrees(projected_angle(delta, axis))


def tool_angle(q0, q, profile):
    q0 = np.asarray(q0, float)
    delta = _quat_mul(np.r_[-q0[:3], q0[3]], q)
    axis = np.eye(3)[1 if profile == 'pitch' else 0]
    return math.degrees(projected_angle(delta, axis))


def direction(angle):
    return 'POSITIVO' if angle > 3 else 'NEGATIVO' if angle < -3 else 'NEUTRO'
