"""Body-referenced operator frame (port of the original wrist_detector frame).

The frame is built from whichever Pose landmarks are visible, in the same
hierarchy as spot-ros2_ws/src/arm_pose_estimator/wrist_detector.py:

  FULL_BODY  shoulders + hips + ankles: up = shoulder-mid - ankle-mid
  TORSO      shoulders + hips:          up = shoulder-mid - hip-mid
  SHOULDERS  shoulders only:            up = camera up (fallback)

Axes (camera frame): x = right_shoulder - left_shoulder, z = -x cross up,
y = x cross z. The robot-from-camera rotation is R.T with
R = [z, -x, y] (columns), which reduces to CAMERA_TO_ROBOT for an upright
operator facing the camera. Origin = torso centre (mean of shoulders and,
when visible, hips) shifted along the body Y so it aligns with the right
shoulder, as in the original. Filtering is the original's sticky EMA over
origin and axes, so a transient occlusion holds the last frame instead of
twisting the output.
"""
import numpy as np

CAMERA_UP = np.array([0., -1., 0.])


def _unit(v):
    n = np.linalg.norm(v)
    return None if n < 1e-9 else v / n


def _proper_rotation(m):
    u, _, vt = np.linalg.svd(np.asarray(m, float))
    r = u @ vt
    if np.linalg.det(r) < 0:
        u[:, -1] *= -1
        r = u @ vt
    return r


TORSO_DEPTH_TOL_M = .10


def _sanitize_torso_depths(points):
    """Reproject torso points whose depth disagrees with the torso median.

    A landmark pixel can sit on the raised arm in front of the torso (the
    z04 right shoulder reads 0.89-1.40 m while neck/hips/other shoulder
    agree at 1.16-1.29 m). That single depth invents a shoulder axis with
    a huge camera-Z component and rotates the whole frame 30-45 deg.
    Points deviating more than TORSO_DEPTH_TOL_M from the torso median
    depth are moved along their own camera ray to the median depth:
    pixel direction (the reliable part) is kept, the occluded depth is
    not. Torso yaw beyond ~15 deg of shoulder asymmetry is therefore
    anchored to the camera heading, as in the fixed-axis tracker; tilt
    (the height-leak term) is still fully body-referenced.
    """
    pts = [np.asarray(p, float) for p in points if p is not None and np.all(np.isfinite(p))]
    if len(pts) < 3:
        return points
    ref = float(np.median([p[2] for p in pts]))
    if not (.3 < ref < 3.):
        return points
    out = []
    for p in points:
        if p is None or not np.all(np.isfinite(p)) or abs(float(p[2]) - ref) <= TORSO_DEPTH_TOL_M or p[2] <= 0:
            out.append(p)
        else:
            out.append(np.asarray(p, float) * (ref / float(p[2])))
    return out


def make_body_frame(shoulder_l, shoulder_r, hip_l=None, hip_r=None,
                    ankle_l=None, ankle_r=None, camera_up=CAMERA_UP):
    """Raw (unfiltered) frame from camera-frame 3D landmarks.

    Returns (R, origin, level) with robot_from_camera = R.T, or None when
    the shoulders are unusable. Points are (3,) arrays or None.
    """
    if shoulder_l is None or shoulder_r is None:
        return None
    shoulder_l, shoulder_r, hip_l, hip_r = _sanitize_torso_depths(
        [shoulder_l, shoulder_r, hip_l, hip_r])
    sl, sr = np.asarray(shoulder_l, float), np.asarray(shoulder_r, float)
    if not (np.all(np.isfinite(sl)) and np.all(np.isfinite(sr))):
        return None
    if not .20 < np.linalg.norm(sr - sl) < .75:
        return None
    axis_x = _unit(sr - sl)
    sh_mid = (sl + sr) / 2
    torso_pts = [sl, sr]
    up, level = np.asarray(camera_up, float), 'SHOULDERS'

    def mid(a, b):
        if a is None or b is None:
            return None
        a, b = np.asarray(a, float), np.asarray(b, float)
        return (a + b) / 2 if np.all(np.isfinite(a)) and np.all(np.isfinite(b)) else None

    hip_mid = mid(hip_l, hip_r)
    if hip_mid is not None:
        torso_pts += [np.asarray(hip_l, float), np.asarray(hip_r, float)]
        ankle_mid = mid(ankle_l, ankle_r)
        if ankle_mid is not None:
            up, level = sh_mid - ankle_mid, 'FULL_BODY'
        else:
            up, level = sh_mid - hip_mid, 'TORSO'
    axis_z = _unit(-np.cross(axis_x, up))
    if axis_z is None:
        return None
    axis_y = np.cross(axis_x, axis_z)
    R = _proper_rotation(np.column_stack([axis_z, -axis_x, axis_y]))
    torso_center = np.mean(torso_pts, 0)
    r_sh_in_body = R.T @ (sr - torso_center)
    origin = torso_center + R @ np.array([0., r_sh_in_body[1], 0.])
    return R, origin, level


class BodyFrameFilter:
    """Sticky EMA over make_body_frame output (original: alpha_axes .08,
    axes recomputed at low rate; here the caller controls the cadence).

    Landmark jump gate ported from the original detector: a body landmark
    cannot move faster than 1.5 m/s (budget clamped to [0.06, 0.40] m), so
    a reading beyond that (occlusion depth, arm crossing a hip) holds the
    frame for that cycle instead of twisting it. A rejected reading only
    re-seeds its landmark after 12 consecutive mutually consistent
    readings -- a genuine relocation, never a transient occlusion.
    """

    def __init__(self, alpha=.15):
        self.alpha = float(alpha)
        self.R = None
        self.origin = None
        self.level = None
        self.last_raw_valid = False
        self._accepted = {}
        self._accepted_at = {}
        self._candidates = {}

    def reset(self):
        self.R = self.origin = self.level = None
        self.last_raw_valid = False
        self._accepted = {}
        self._accepted_at = {}
        self._candidates = {}

    def _gate_landmarks(self, points, now):
        """Apply the jump gate to named points; True if any was rejected."""
        rejected = False
        for name, point in list(points.items()):
            if point is None:
                continue
            point = np.asarray(point, float)
            if not np.all(np.isfinite(point)):
                continue
            prev = self._accepted.get(name)
            if prev is None:
                self._accepted[name] = point.copy()
                self._accepted_at[name] = now
                continue
            dt = .33 if now is None or self._accepted_at.get(name) is None else max(now - self._accepted_at[name], 1e-3)
            allowed = float(np.clip(1.5 * dt, .06, .40))
            if np.linalg.norm(point - prev) <= allowed:
                self._accepted[name] = point.copy()
                self._accepted_at[name] = now
                self._candidates.pop(name, None)
                continue
            rejected = True
            points[name] = prev
            cand = self._candidates.get(name)
            if cand is not None and np.linalg.norm(point - cand[0]) <= allowed:
                cand = (point.copy(), cand[1] + 1)
            else:
                cand = (point.copy(), 1)
            if cand[1] >= 12:
                self._accepted[name] = point.copy()
                self._accepted_at[name] = now
                self._candidates.pop(name, None)
            else:
                self._candidates[name] = cand
        return rejected

    def update(self, *args, now=None, **kwargs):
        if self._gate_landmarks(kwargs, now):
            self.last_raw_valid = False
            return None if self.R is None else (self.R, self.origin, self.level)
        got = make_body_frame(*args, **kwargs)
        if got is None:
            self.last_raw_valid = False
            return None if self.R is None else (self.R, self.origin, self.level)
        self.last_raw_valid = True
        R, origin, level = got
        if self.R is None:
            self.R, self.origin = R, origin
        else:
            a = self.alpha
            self.origin = a * origin + (1 - a) * self.origin
            # EMA on the rotation via its columns, re-orthonormalised; the
            # sign guard keeps the average from flipping an axis.
            m = a * R + (1 - a) * self.R
            if np.trace(m) < 0:
                m = a * R - (1 - a) * self.R
            self.R = _proper_rotation(m)
        self.level = level
        return self.R, self.origin, self.level

    @property
    def robot_from_camera(self):
        """Rotation mapping camera-frame vectors to robot (operator) axes."""
        return None if self.R is None else self.R.T
