"""Held-drill attach helpers (pure numpy, no ROS/torch at import).

The regrasp loop freezes while the gripper is closed (see loop.py 1b); this
module is the MPC side: it decides WHEN the closed gripper holds the drill
(close NEAR the last grasp target -- an empty close elsewhere must never
attach) and fits the drill's collision spheres in the fr3_hand frame.

Config precedent (fr3.yml, both trees): link 'attached_object' is FIXED to
fr3_hand with identity transform and carries 4 pre-allocated spare spheres
(extra_collision_spheres), parked at radius -10 (disabled) until attach.
Sphere values written here are therefore plain fr3_hand coordinates.
"""
import numpy as np


class HeldLatch:
    """Width hysteresis: latch below close_below, release at/above open_min.

    Mirrors the TF publisher bars (close <0.070, open >=0.075). width None
    (no joint_states yet) leaves the latch untouched: never latch without
    evidence, never release without evidence.
    """

    def __init__(self, close_below=0.070, open_min=0.075):
        self.close_below = close_below
        self.open_min = open_min
        self.held = False

    def update(self, width):
        """Returns 'latched', 'released', or None (no transition)."""
        if width is None or not np.isfinite(width):
            return None
        if not self.held and width < self.close_below:
            self.held = True
            return 'latched'
        if self.held and width >= self.open_min:
            self.held = False
            return 'released'
        return None


def should_attach(hand_pos, target_pos, near_m=0.10):
    """True when the hand closed AT the grasp (the success proxy).

    hand_pos/target_pos: length-3 in the same frame (fr3_link0). Anything
    non-finite or farther than near_m refuses: an empty close elsewhere
    must never attach a phantom drill.
    """
    try:
        h = np.asarray(hand_pos, float).reshape(-1)
        t = np.asarray(target_pos, float).reshape(-1)
    except (TypeError, ValueError):
        return False
    if h.shape != (3,) or t.shape != (3,):
        return False
    if not (np.all(np.isfinite(h)) and np.all(np.isfinite(t))):
        return False
    return bool(float(np.linalg.norm(h - t)) <= near_m)


def holding_verdict(width, hand_pos, target_pos, near_m=0.08,
                  min_width_m=0.010):
    """Close verdict: (candidate, reason). Holding needs all three: the
    fingers held apart by an object (width >= min_width_m: a missed grasp
    closes to ~0), a finite grasp target, and the hand at it (<= near_m).
    Pure so the node and the tests share it."""
    if width is None:
        return False, 'no-width'
    try:
        w = float(width)
    except (TypeError, ValueError):
        return False, 'no-width'
    if not np.isfinite(w):
        return False, 'no-width'
    if w < min_width_m:
        return False, 'empty-close'
    try:
        h = np.asarray(hand_pos, float).reshape(-1)
        t = np.asarray(target_pos, float).reshape(-1)
    except (TypeError, ValueError):
        return False, 'no-target'
    if h.shape != (3,) or not np.all(np.isfinite(h)):
        return False, 'no-hand'
    if t.shape != (3,) or not np.all(np.isfinite(t)):
        return False, 'no-target'
    d = float(np.linalg.norm(h - t))
    if d > near_m:
        return False, 'far-from-target'
    return True, f'holding d={d:.3f} w={w:.3f}'


def attach_floor_ok(width, min_width_m=0.010):
    """Attach-time width floor: (ok, reason). The latch edge always samples
    mid-travel, so this runs on the SETTLED width at attach height. None /
    non-finite -> (False, 'no-width') (caller waits: no evidence, no
    attach); below the floor -> (False, 'empty-settled'); at/above the
    floor -> (True, ...)."""
    if width is None:
        return False, 'no-width'
    try:
        w = float(width)
    except (TypeError, ValueError):
        return False, 'no-width'
    if not np.isfinite(w):
        return False, 'no-width'
    if w < min_width_m:
        return False, 'empty-settled'
    return True, f'floor-ok w={w:.3f}'


def attached_holds(width, min_width_m=0.010):
    """While-attached floor: False (detach) only on a DEFINITE below-floor
    reading -- the grasp was empty or the drill slipped out. None /
    non-finite HOLDS collision: a dead width stream must not drop
    protection."""
    if width is None:
        return True
    try:
        w = float(width)
    except (TypeError, ValueError):
        return True
    if not np.isfinite(w):
        return True
    return w >= min_width_m


def fit_hand_spheres(box_center, box_half, T_hand_from_world, n=4, pad_m=0.005,
                     min_radius_m=0.020):
    """Fit n spheres over a world-frame box, expressed in the hand frame.

    box_center/box_half: length-3 (the /target_object/clear_box cube).
    T_hand_from_world: 4x4 (world -> fr3_hand at latch; the drill is rigid
    in the hand afterwards, so one fit per grasp suffices).
    Returns (n,4) [x,y,z,r]: centers spread along the box's longest axis
    in the hand frame. INSET fit: end centers sit r inside the box ends, so
    nothing bulges past the spread-axis faces (a corner-covering fit dips
    7-12 cm into the table at latch height and fights the lift). r covers
    the larger perpendicular half-extent + pad, so the smaller perp face
    bulges by design; the attach need-gate (lowest world point vs the
    table) is the real guard there, plus the MPC collision buffer.
    Raises ValueError on bad inputs (caller falls back, never attaches NaN).
    """
    c = np.asarray(box_center, float).reshape(-1)
    h = np.asarray(box_half, float).reshape(-1)
    T = np.asarray(T_hand_from_world, float)
    if c.shape != (3,) or h.shape != (3,) or T.shape != (4, 4):
        raise ValueError('bad box/transform shapes')
    if not (np.all(np.isfinite(c)) and np.all(np.isfinite(h))
            and np.all(np.isfinite(T)) and np.all(h >= 0)):
        raise ValueError('non-finite or negative box')
    if n < 1:
        raise ValueError('need at least 1 sphere')
    # Box corners -> hand frame -> hand-frame AABB (orientation-free).
    corners = np.stack(np.meshgrid(*[[-1.0, 1.0]] * 3)).reshape(3, -1).T
    pts = c + corners * h
    homo = np.concatenate([pts, np.ones((8, 1))], axis=1) @ T.T
    lo, hi = homo[:, :3].min(axis=0), homo[:, :3].max(axis=0)
    ext = hi - lo
    ax = int(np.argmax(ext))
    half = ext / 2.0
    r = max(float(max(half[(ax + 1) % 3], half[(ax + 2) % 3])) + pad_m,
            min_radius_m)
    out = np.zeros((n, 4))
    mid = (lo + hi) / 2.0
    # Inset the end centers by r so no sphere bulges past the box ends. A
    # squat box (span < 2r) collapses toward mid; the attach need-gate
    # (lowest world point vs the table) catches any dip that remains.
    lo_c = min(lo[ax] + r, mid[ax])
    hi_c = max(hi[ax] - r, mid[ax])
    for i in range(n):
        f = lo_c + (hi_c - lo_c) * i / (n - 1) if n > 1 else mid[ax]
        p = mid.copy()
        p[ax] = f
        out[i, :3], out[i, 3] = p, r
    return out


def _rot_from_quat_wxyz(q):
    q = np.asarray(q, float).reshape(-1)
    w, x, y, z = q / (np.linalg.norm(q) or 1.0)
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def lowest_world_point(spheres_hand, hand_pos, hand_quat_wxyz):
    """Lowest world z reached by hand-frame spheres: min over (R*c + p).z - r.
    The attach need-gate: attaching with this below the table + margin
    would press the table and fight the lift, so the node stays pending."""
    S = np.asarray(spheres_hand, float)
    p = np.asarray(hand_pos, float).reshape(-1)
    if S.ndim != 2 or S.shape[1] != 4 or p.shape != (3,):
        raise ValueError('bad spheres/hand shapes')
    R = _rot_from_quat_wxyz(hand_quat_wxyz)
    if not (np.all(np.isfinite(S)) and np.all(np.isfinite(p))
            and np.all(np.isfinite(R))):
        raise ValueError('non-finite spheres/hand')
    return float((((R @ S[:, :3].T).T + p)[:, 2] - S[:, 3]).min())


def fallback_spheres(n=4, radius_m=0.045, z_m=0.055):
    """Symmetric blob about the TCP when no fresh box exists at latch.

    Two spheres above, two below the palm along hand Z (sign-convention
    free); covers a fumbled fit conservatively until the next grasp.
    """
    if n < 1:
        raise ValueError('need at least 1 sphere')
    zs = np.linspace(-z_m, z_m, n)
    out = np.zeros((n, 4))
    out[:, 2], out[:, 3] = zs, radius_m
    return out
