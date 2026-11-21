"""Offline transparent-bottle keep-out from RGB silhouettes (no depth, no ROS).

Clear PET defeats stereo depth (fragmented / smeared clouds), but the SAM2
silhouettes are clean. A visual hull (voxels whose projection lands inside
every camera's mask) is depth-free. With two views it is NOT conservative by
itself: the cameras disagree by ~16.6 mm on the same ball, so the raw hull
cuts the object. The mask dilation is therefore calibrated on a co-captured
known-geometry control (the tennis ball): the smallest dilation at which the
hull contains BOTH cameras' sphere fits.

Output is an OBSTACLE candidate (axis-aligned cuboid) for when the bottle is
NOT the target. No grasp is produced here; see assess_bottle_grasp().
Projection convention matches build_snapshot_clouds.py / twocam/analyze.py:
camera = (x, y, z, qx, qy, qz, qw), R maps optical -> base.
"""
from __future__ import annotations

from dataclasses import dataclass

import cv2
import numpy as np

H_IMG, W_IMG = 360, 640


@dataclass(frozen=True)
class Camera:
    K: np.ndarray
    R: np.ndarray
    t: np.ndarray

    @staticmethod
    def from_npz(path):
        a = np.load(path)
        x, y, z, qx, qy, qz, qw = a['camera']
        R = np.array([
            [1 - 2 * (qy * qy + qz * qz), 2 * (qx * qy - qz * qw), 2 * (qx * qz + qy * qw)],
            [2 * (qx * qy + qz * qw), 1 - 2 * (qx * qx + qz * qz), 2 * (qy * qz - qx * qw)],
            [2 * (qx * qz - qy * qw), 2 * (qy * qz + qx * qw), 1 - 2 * (qx * qx + qy * qy)]])
        return Camera(a['K'].reshape(3, 3), R, np.array([x, y, z], float))

    def project(self, X):
        Pc = (np.asarray(X, float) - self.t) @ self.R
        z = Pc[:, 2]
        with np.errstate(divide='ignore', invalid='ignore'):
            uv = np.c_[self.K[0, 0] * Pc[:, 0] / z + self.K[0, 2],
                       self.K[1, 1] * Pc[:, 1] / z + self.K[1, 2]]
        return uv, z


def _dilate(mask, k):
    m = np.asarray(mask, np.uint8)
    if k <= 0:
        return m > 0
    return cv2.dilate(m, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (2 * k + 1, 2 * k + 1))) > 0


def visual_hull(masks: dict, cams: dict, k_px: int, lo, hi, voxel=0.003):
    """Voxel centres inside every (dilated) silhouette. masks/cams keyed alike."""
    if set(masks) != set(cams) or len(masks) < 2:
        raise ValueError('need >= 2 cameras with matching masks')
    axes = [np.arange(lo[i], hi[i], voxel) for i in range(3)]
    X = np.stack(np.meshgrid(*axes, indexing='ij'), -1).reshape(-1, 3)
    keep = np.ones(len(X), bool)
    for key, cam in cams.items():
        m = _dilate(masks[key], k_px)
        if m.shape != (H_IMG, W_IMG) or not m.any():
            raise ValueError(f'mask {key} empty or wrong shape')
        uv, z = cam.project(X)
        u = np.round(np.nan_to_num(uv, nan=-1)).astype(int)
        ok = (z > 0) & (u[:, 0] >= 0) & (u[:, 0] < W_IMG) & (u[:, 1] >= 0) & (u[:, 1] < H_IMG)
        inside = np.zeros(len(X), bool)
        inside[ok] = m[u[ok, 1], u[ok, 0]]
        keep &= inside
    return X[keep]


def contained_fraction(H, P, voxel=0.003):
    if len(H) == 0:
        return 0.0
    occ = set(map(tuple, np.round(H / voxel).astype(int)))
    return float(np.mean([tuple(k) in occ for k in np.round(np.asarray(P) / voxel).astype(int)]))


def sphere_surface(center, r, table_z, n=4000, seed=0):
    u = np.random.default_rng(seed).normal(size=(n, 3))
    u /= np.linalg.norm(u, axis=1)[:, None]
    S = np.asarray(center) + u * r
    return S[S[:, 2] >= table_z + 0.002]


def calibrate_dilation(ball_masks, cams, sphere_fits, table_z, k_max=15,
                       need=1.0, voxel=0.002):
    """Smallest k (px) whose ball hull contains every camera's sphere fit.
    Returns (k, table of (k, fractions)). Raises if none up to k_max."""
    centers = np.array([f.center for f in sphere_fits])
    mid = centers.mean(0)
    span = 0.08 + float(np.ptp(centers, 0).max())
    surfaces = [sphere_surface(f.center, f.radius, table_z) for f in sphere_fits]
    rows = []
    for k in range(0, k_max + 1):
        H = visual_hull(ball_masks, cams, k, mid - span, mid + span, voxel)
        fr = [contained_fraction(H, S, voxel) for S in surfaces]
        rows.append((k, [round(f, 3) for f in fr]))
        if min(fr) >= need:
            return k, rows
    raise ValueError(f'no dilation <= {k_max}px contains all sphere fits: {rows[-1]}')


# The calibrated threshold moved 8 <-> 9 px between two surface samplings of
# the same control, so the applied dilation adds that observed 1 px spread.
DILATION_SAFETY_PX = 1


def bottle_keepout(bottle_masks, cams, ball_masks, sphere_fits, table_z, lo, hi, voxel=0.003):
    """End-to-end offline keep-out: calibrate on the ball, carve the bottle."""
    k_cal, rows = calibrate_dilation(ball_masks, cams, sphere_fits, table_z)
    k = k_cal + DILATION_SAFETY_PX
    H = visual_hull(bottle_masks, cams, k, lo, hi, voxel)
    cov = {key: round(reprojection_coverage(H, bottle_masks[key], cams[key]), 3) for key in cams}
    if min(cov.values()) < 0.99:
        raise ValueError(f'keep-out does not cover every silhouette: {cov}')
    return dict(keepout_cuboid(H, table_z, voxel), k_calibrated_px=k_cal, k_applied_px=k,
                calibration=rows, mask_coverage=cov, n_voxels=int(len(H)),
                scope='offline obstacle candidate; bottle must NOT be the target')


def keepout_cuboid(H, table_z, voxel=0.003):
    """Axis-aligned cuboid around the hull (voxel half-size padded), base at table."""
    if len(H) == 0:
        raise ValueError('empty hull')
    lo = H.min(0) - voxel / 2
    hi = H.max(0) + voxel / 2
    lo[2] = min(lo[2], table_z)
    return dict(center=np.round((lo + hi) / 2, 4).tolist(),
                dims=np.round(hi - lo, 4).tolist(), frame='fr3_link0')


def reprojection_coverage(H, mask, cam):
    uv, _ = cam.project(H)
    u = np.round(uv).astype(int)
    ok = (u[:, 0] >= 0) & (u[:, 0] < W_IMG) & (u[:, 1] >= 0) & (u[:, 1] < H_IMG)
    img = np.zeros((H_IMG, W_IMG), np.uint8)
    img[u[ok, 1], u[ok, 0]] = 1
    img = cv2.dilate(img, np.ones((3, 3), np.uint8)) > 0
    m = np.asarray(mask) > 0
    return float((img & m).sum() / m.sum())


def top_ray_height(mask, cam, axis_xy):
    """z where the ray through the top-most silhouette pixel passes the
    vertical line at axis_xy (closest approach). Depth-free height estimate."""
    ys, xs = np.nonzero(mask)
    y0 = ys.min()
    x0 = xs[ys == y0].mean()
    d = cam.R @ np.array([(x0 - cam.K[0, 2]) / cam.K[0, 0], (y0 - cam.K[1, 2]) / cam.K[1, 1], 1.0])
    d /= np.linalg.norm(d)
    w = cam.t[:2] - np.asarray(axis_xy)
    s = -(w @ d[:2]) / (d[:2] @ d[:2])
    return float((cam.t + s * d)[2])


GRIPPER_OPEN = 0.0799


def assess_bottle_grasp(*, diameter_bounds_m, lateral_budget_m, top_z_by_cam,
                        depth_pts_between_pads=0):
    """Fail-closed verdict for a bottle grasp from offline evidence.
    Returns dict(ok=False, reasons=[...]) unless every requirement is met."""
    reasons = []
    dlo, dhi = diameter_bounds_m
    if not (np.isfinite(dlo) and np.isfinite(dhi) and dhi >= dlo > 0):
        reasons.append('bottle diameter unknown')
    else:
        cap = (GRIPPER_OPEN - dhi) / 2
        if cap <= 0:
            reasons.append(f'body diameter up to {dhi * 1e3:.0f} mm may not fit {GRIPPER_OPEN * 1e3:.1f} mm opening')
        elif lateral_budget_m > cap:
            reasons.append(f'lateral uncertainty {lateral_budget_m * 1e3:.1f} mm > body clearance {cap * 1e3:.1f} mm')
    tops = list(top_z_by_cam.values())
    if len(tops) < 2:
        reasons.append('cap height needs >= 2 cameras')
    elif max(tops) - min(tops) > 0.010:
        reasons.append(f'cap height disagrees by {(max(tops) - min(tops)) * 1e3:.0f} mm across cameras')
    if depth_pts_between_pads < 50:
        reasons.append('no trustworthy depth points between pads (transparent); planner contract unmet')
    return dict(ok=not reasons, reasons=reasons)
