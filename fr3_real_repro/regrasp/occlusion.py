"""Occlusion check: does the arm hide the drill from a camera? Pure numpy.

The FK arm silhouette is projected into each camera and tested against the
projected HELD drill box (never the live box: under occlusion the live box
is garbage, which is the circularity this avoids). A link point occludes
only when it lands inside the box footprint AND sits between the camera
and the box in depth. No ROS here; loop.py feeds q + camera models.
"""
import numpy as np

BOX_DILATE_PX = 15.0    # footprint margin for silhouette thickness
DEPTH_MARGIN_M = 0.05   # arm must be this far in front of the box far plane
PTS_PER_LINK = 6


class CamModel:
    """Pinhole + base-from-camera extrinsic (robotfit7_dz numbers live)."""

    def __init__(self, fx, fy, cx, cy, width, height, R_base_cam, t_base_cam):
        self.fx, self.fy, self.cx, self.cy = fx, fy, cx, cy
        self.width, self.height = width, height
        self.R = np.asarray(R_base_cam, float)
        self.t = np.asarray(t_base_cam, float)

    def project(self, P):
        """Base-frame Nx3 -> (u, v, z_cam). Behind-camera rows get z <= 0."""
        Pc = (np.asarray(P, float) - self.t) @ self.R
        z = Pc[:, 2]
        with np.errstate(divide='ignore', invalid='ignore'):
            u = self.fx * Pc[:, 0] / z + self.cx
            v = self.fy * Pc[:, 1] / z + self.cy
        return u, v, z


def arm_points(q, fk):
    """Sampled points along the arm in fr3_link0. fk is grasp_plan_lib.fk
    (injected so tests don't import the planner through this module)."""
    _, P = fk(np.asarray(q, float), pts=True)
    P = np.asarray(P, float)
    out = [P]
    for a, b in zip(P[:-1], P[1:]):
        s = np.linspace(0, 1, PTS_PER_LINK + 2)[1:-1]
        out.append(a[None, :] * (1 - s[:, None]) + b[None, :] * s[:, None])
    return np.vstack(out)


def _box_corners(lo, hi):
    lo, hi = np.asarray(lo, float), np.asarray(hi, float)
    return np.array([[x, y, z] for x in (lo[0], hi[0])
                     for y in (lo[1], hi[1]) for z in (lo[2], hi[2])])


def occludes(q, cam, box_lo, box_hi, fk):
    """True if any arm point falls on the held-box footprint in front of it."""
    cu, cv, cz = cam.project(_box_corners(box_lo, box_hi))
    if bool((cz <= 0).all()):
        return False                    # box behind this camera: no overlap
    u1, u2 = cu.min() - BOX_DILATE_PX, cu.max() + BOX_DILATE_PX
    v1, v2 = cv.min() - BOX_DILATE_PX, cv.max() + BOX_DILATE_PX
    zfar = cz.max() + DEPTH_MARGIN_M
    au, av, az = cam.project(arm_points(q, fk))
    inside = (au >= u1) & (au <= u2) & (av >= v1) & (av <= v2)
    return bool(((inside) & (az > 0) & (az < zfar)).any())
