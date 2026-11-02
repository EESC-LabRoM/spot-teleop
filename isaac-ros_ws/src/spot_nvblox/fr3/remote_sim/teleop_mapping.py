"""Validated relative mapping shared by bag inspection and the simulation."""
import math
import numpy as np


def paired_sample(packet):
    """Position-only v3 packets must never renew full-pose tracking."""
    if not isinstance(packet, dict):
        return None
    try:
        stamp = int(packet['stamp_ns'])
        p, q = np.asarray(packet['p'], float), np.asarray(packet['q'], float)
        if packet.get('version') not in (2, 3) or packet.get('frame') != 'fr3_link0':
            return None
        if packet.get('orientation_mode') != 'full_6d' or packet.get('orientation_stamp_ns') != stamp:
            return None
        if packet.get('position_valid', True) is not True or packet.get('orientation_valid', True) is not True:
            return None
        if p.shape != (3,) or q.shape != (4,) or not np.all(np.isfinite(np.r_[p, q])):
            return None
        if abs(np.linalg.norm(q) - 1) > .01:
            return None
        return stamp, p, q / np.linalg.norm(q)
    except (ValueError, TypeError, KeyError, OverflowError):
        return None


def mapping_settings(gains, rotation_gain, tilt_gain=None):
    gains = np.asarray(gains, float)
    if gains.shape != (3,) or not np.all(np.isfinite(gains)) or np.any(np.abs(gains) > 5):
        raise ValueError('Position gains require three finite values in [-5, 5]')
    rotation_gain = float(rotation_gain)
    if not math.isfinite(rotation_gain) or not 0 <= rotation_gain <= 3:
        raise ValueError('Rotation gain must be in [0, 3]')
    if tilt_gain is None:
        return gains, rotation_gain
    tilt_gain = float(tilt_gain)
    if not math.isfinite(tilt_gain) or not 0 <= tilt_gain <= 3:
        raise ValueError('Tilt gain must be in [0, 3]')
    return gains, rotation_gain, tilt_gain


def control_sample(packet, previous_q=None):
    """Accept fresh position-only v3 with a held, previously measured orientation."""
    full = paired_sample(packet)
    if full is not None:
        return (*full, True)
    if not isinstance(packet, dict) or previous_q is None:
        return None
    try:
        if (packet.get('version') != 3 or packet.get('frame') != 'fr3_link0'
                or packet.get('orientation_mode') != 'full_6d'
                or packet.get('position_valid') is not True
                or packet.get('orientation_valid') is not False
                or packet.get('q') is not None or packet.get('orientation_stamp_ns') is not None):
            return None
        p = np.asarray(packet['p'], float)
        if p.shape != (3,) or not np.all(np.isfinite(p)):
            return None
        return int(packet['stamp_ns']), p, previous_q.copy(), False
    except (ValueError, TypeError, KeyError, OverflowError):
        return None


def control_gesture(packet):
    """A fresh accepted wrist packet can confirm fingers even if palm rotation is held.

    Freshness and active authorization are checked by the caller; do not couple
    gesture confidence to depth-based palm orientation validity.
    """
    try:
        score=float(packet.get('gesture_score',0))
        gesture=packet.get('gesture')
        if packet.get('position_valid',packet.get('version')==2) is True and math.isfinite(score) and .75<=score<=1. and gesture in ('Open_Palm','Closed_Fist'):
            return gesture
    except (TypeError,ValueError,AttributeError):
        pass
    return None


def _quat_mul(q1, q2):
    x1, y1, z1, w1 = q1
    x2, y2, z2, w2 = q2
    return np.array([
        w1*x2 + x1*w2 + y1*z2 - z1*y2,
        w1*y2 - x1*z2 + y1*w2 + z1*x2,
        w1*z2 + x1*y2 - y1*x2 + z1*w2,
        w1*w2 - x1*x2 - y1*y2 - z1*z2,
    ])


def rotation_delta(q, q0, gain):
    """Shortest spatial quaternion delta, with an optional angle multiplier."""
    x, y, z, w = q
    a, b, c, d = -q0[0], -q0[1], -q0[2], q0[3]
    delta = np.array([w*a+x*d+y*c-z*b, w*b-x*c+y*d+z*a,
                      w*c+x*b-y*a+z*d, w*d-x*a-y*b-z*c])
    delta /= np.linalg.norm(delta)
    if delta[3] < 0:
        delta = -delta
    length = np.linalg.norm(delta[:3])
    if length < 1e-10:
        return np.array([0., 0., 0., 1.])
    half_angle = math.atan2(length, delta[3]) * gain
    return np.r_[delta[:3] * (math.sin(half_angle) / length), math.cos(half_angle)]


def palm_normal(q):
    """Palm +z normal in the packet frame: -(first column of R(q))."""
    q = np.asarray(q, float)
    x, y, z, w = q / np.linalg.norm(q)
    return -np.array([1. - 2.*(y*y + z*z), 2.*(x*y + z*w), 2.*(x*z - y*w)])


def map_target(anchor_p, anchor_q, hand_p0, hand_q0, hand_p, hand_q,
               position_gains, roll_gain, tilt_gain=0.0):
    """Map relative hand motion in packet frame to Franka TCP target (p_target, q_target)."""
    ap = np.asarray(anchor_p, float)
    aq = np.asarray(anchor_q, float)
    aq = aq / np.linalg.norm(aq)
    p0 = np.asarray(hand_p0, float)
    q0 = np.asarray(hand_q0, float)
    q0 = q0 / np.linalg.norm(q0)
    p = np.asarray(hand_p, float)
    q = np.asarray(hand_q, float)
    q = q / np.linalg.norm(q)
    gx, gy, gz = np.asarray(position_gains, float)
    roll_gain = float(roll_gain)
    tilt_gain = float(tilt_gain)

    dx, dy, dz = p - p0
    r0 = math.hypot(ap[0], ap[1])
    dr = gx * dz
    if dr == 0.0 and 0.15 <= r0 <= 0.80:
        x_rad, y_rad = ap[0], ap[1]
    else:
        r = float(np.clip(r0 + dr, 0.15, 0.80))
        if r0 > 1e-9:
            scale = r / r0
            x_rad, y_rad = ap[0] * scale, ap[1] * scale
        else:
            x_rad, y_rad = r, 0.0

    dtheta = gy * dy / max(r0, 0.2)
    c, s = math.cos(dtheta), math.sin(dtheta)
    p_target = np.array([
        c * x_rad - s * y_rad,
        s * x_rad + c * y_rad,
        ap[2] - gz * dx,
    ], dtype=float)

    n0 = palm_normal(q0)
    # Camera forward in the packet frame is [-1, 0, 0] (-X).
    dot_cam = float(np.dot(n0, [-1.0, 0.0, 0.0]))
    if abs(dot_cam) >= 0.2 and dot_cam < 0.0:
        n0 = -n0

    dq = _quat_mul(q, np.array([-q0[0], -q0[1], -q0[2], q0[3]]))
    dq /= np.linalg.norm(dq)
    angle = 2.0 * math.atan2(float(np.dot(dq[:3], n0)), float(dq[3]))
    angle = (angle + math.pi) % (2.0 * math.pi) - math.pi
    if angle <= -math.pi:
        angle += 2.0 * math.pi

    half_roll = 0.5 * roll_gain * angle
    rz = np.array([0.0, 0.0, math.sin(half_roll), math.cos(half_roll)])
    q_target = _quat_mul(aq, rz)

    if tilt_gain != 0.0:
        half_twist = 0.5 * angle
        twist = np.r_[n0 * math.sin(half_twist), math.cos(half_twist)]
        swing = _quat_mul(dq, np.array([-twist[0], -twist[1], -twist[2], twist[3]]))
        swing /= np.linalg.norm(swing)
        swing_delta = rotation_delta(swing, np.array([0.0, 0.0, 0.0, 1.0]), tilt_gain)
        q_target = _quat_mul(swing_delta, q_target)

    q_target /= np.linalg.norm(q_target)
    return p_target, q_target
