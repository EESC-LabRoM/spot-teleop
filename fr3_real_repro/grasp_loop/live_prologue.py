#!/usr/bin/env python3
"""Live object-relative prologue for the grasp (offline-safe).

Same grasp, shifted by the target's live rigid motion as measured by
spot-teleop's video predictor (/tracking_3d_point). Translation-only;
orientation untouched. Fully object-agnostic: no object-specific numbers.

    pick_live = pick_grasp + (T_live_link0 - C_grasp_link0)

T_live is the median live tracker 3D point transformed to fr3_link0 via
the STATIC camera chain (static_tf_zed1_link0.json; camera is bolted);
A_grasp is the grasp-time tracker seed point (seed.json seed_3d, same
optical frame) through the same chain. Same quantity, same estimator,
both ends - no cross-estimator bias. Residual assumption: the live
/tracking_3d_point follows the same object point the seed marked;
backstopped by the overlay check + the runner's 20 mm close-gate.

Guards (fail fast, never guess): mask fresh (age < 2 s), >= 3 samples,
jump since grasp <= 0.15 m.

Usage:
  python3 live_prologue.py --run <run_dir> [--replay samples.json]
--replay runs fully offline (no ROS) for dry validation.
--use-tf forces a live TF lookup instead of the static file (debug).
"""

import argparse
import json
import math
import os
import sys

SOURCE_FRAME = 'zed1_left_camera_optical_frame'
TARGET_FRAME = 'fr3_link0'
TRACK_TOPIC = '/tracking_3d_point'
MASK_TOPIC = '/zed1/segmentation_mask'
MAX_JUMP_M = 0.15
MAX_MASK_AGE_S = 2.0
MIN_SAMPLES = 3
CHAIN_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                           'static_tf_zed1_link0.json')


class Fail(Exception):
    pass


# ---------------------------------------------------------------- pure --
def _median(v):
    s = sorted(v)
    n = len(s)
    return s[n // 2] if n % 2 else 0.5 * (s[n // 2 - 1] + s[n // 2])


def median_point(points):
    """Component-wise median of [(x, y, z), ...]. Needs >= MIN_SAMPLES."""
    if len(points) < MIN_SAMPLES:
        raise Fail(f'need >={MIN_SAMPLES} live samples, got {len(points)}')
    xs, ys, zs = zip(*points)
    return [_median(xs), _median(ys), _median(zs)]


def quat_rotate(q, p):
    """Rotate point p by quaternion q_xyzw (pure python)."""
    x, y, z, w = q
    px, py, pz = p
    # q * p * q_conj, p as pure quaternion
    ix = w * px + y * pz - z * py
    iy = w * py + z * px - x * pz
    iz = w * pz + x * py - y * px
    iw = -x * px - y * py - z * pz
    rx = ix * w - iw * x - iy * z + iz * y
    ry = iy * w - iw * y - iz * x + ix * z
    rz = iz * w - iw * z - ix * y + iy * x
    return [rx, ry, rz]


def apply_transform(t, q_xyzw, p):
    """Rigid transform: rotate by q then translate by t."""
    r = quat_rotate(q_xyzw, p)
    return [r[0] + t[0], r[1] + t[1], r[2] + t[2]]


def compose_chain(hops):
    """Compose hops (each {t, q_xyzw}) given in APPLICATION order.

    Returns single (t, q) mapping input-frame points to output frame.
    """
    Rt, qt = [0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0]
    for h in hops:
        th, qh = h['t'], h['q_xyzw']
        Rt = apply_transform(th, qh, Rt)
        qt = quat_mul(qh, qt)
    return Rt, qt


def quat_mul(q2, q1):
    """Quaternion product q2*q1 (apply q1 first)."""
    x1, y1, z1, w1 = q1
    x2, y2, z2, w2 = q2
    return [w2 * x1 + x2 * w1 + y2 * z1 - z2 * y1,
            w2 * y1 - x2 * z1 + y2 * w1 + z2 * x1,
            w2 * z1 + x2 * y1 - y2 * x1 + z2 * w1,
            w2 * w1 - x2 * x1 - y2 * y1 - z2 * z1]


def quat_to_matrix(q):
    """Quaternion xyzw -> 3x3 rotation matrix (rows)."""
    x, y, z, w = q
    n = math.sqrt(x * x + y * y + z * z + w * w)
    x, y, z, w = x / n, y / n, z / n, w / n
    return [[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]]


def invert_transform(t, q):
    """Inverse of rigid (t, q): returns (t_inv, q_inv) mapping back."""
    R = quat_to_matrix(q)
    Rt = [[R[j][i] for j in range(3)] for i in range(3)]  # transpose
    ti = [-sum(Rt[i][j] * t[j] for j in range(3)) for i in range(3)]
    x, y, z, w = q
    return ti, [-x, -y, -z, w]


def load_chain(path=CHAIN_FILE):
    d = json.load(open(path))
    if d.get('from') != SOURCE_FRAME or d.get('to') != TARGET_FRAME:
        raise Fail(f'chain file {path} covers '
                   f'{d.get("from")}->{d.get("to")}, need '
                   f'{SOURCE_FRAME}->{TARGET_FRAME}')
    return compose_chain(d['hops'])


def shift_pick(pick_pos, grasp_centroid, live_link0):
    """Translate the grasp pick by the live object delta. Returns (P, delta)."""
    delta = [b - a for a, b in zip(grasp_centroid, live_link0)]
    jump = math.sqrt(sum(d * d for d in delta))
    if jump > MAX_JUMP_M:
        raise Fail(f'target moved {jump:.2f} m since grasp '
                   f'(>{MAX_JUMP_M}): re-run S1-S4')
    P = [a + d for a, d in zip(pick_pos, delta)]
    return P, delta


def check_mask_age(stamp_sec, now_sec):
    age = now_sec - stamp_sec
    if age < 0 or age > MAX_MASK_AGE_S:
        raise Fail(f'mask stale (age {age:.1f} s > {MAX_MASK_AGE_S}): '
                   f'verify tracker input before firing')
    return age


def load_grasp_refs(run_dir, t, q):
    pick = json.load(open(os.path.join(run_dir, 'pick.json')))
    seed = json.load(open(os.path.join(run_dir, 'seed.json')))
    anchor = apply_transform(t, q, seed['seed_3d'])
    return pick['pos'], pick['quat_xyzw'], anchor


def write_export(out_path, P):
    line = f'GTX={P[0]:.4f} GTY={P[1]:.4f} GTZ={P[2]:.4f}'
    with open(out_path, 'w') as f:
        f.write(line + '\n')
    return line


# ------------------------------------------------------------------ ROS --
def capture_live(node, seconds=2.0):
    """Median live tracker point + mask age. Only ROS-touching capture."""
    import time
    from geometry_msgs.msg import PointStamped
    from sensor_msgs.msg import Image
    pts = []
    mask_stamp = [None]

    def on_point(m):
        pts.append((m.point.x, m.point.y, m.point.z))

    def on_mask(m):
        mask_stamp[0] = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9

    sub = node.create_subscription(PointStamped, TRACK_TOPIC, on_point, 10)
    msub = node.create_subscription(Image, MASK_TOPIC, on_mask, 10)
    t0 = time.monotonic()
    import rclpy
    while time.monotonic() - t0 < seconds:
        rclpy.spin_once(node, timeout_sec=0.1)
    node.destroy_subscription(sub)
    node.destroy_subscription(msub)
    if mask_stamp[0] is None:
        raise Fail('no mask received during capture (tracker starved?)')
    now_ros = node.get_clock().now().nanoseconds * 1e-9
    age = check_mask_age(mask_stamp[0], now_ros)
    med = median_point([p[:3] for p in pts])
    return med, age


def lookup_tf():
    """Live TF lookup (debug/fallback; static file is primary)."""
    import rclpy
    from tf2_ros import Buffer, TransformListener
    node = rclpy.create_node('prologue_tf')
    buf = Buffer()
    TransformListener(buf, node)
    t0 = node.get_clock().now().nanoseconds
    while (node.get_clock().now().nanoseconds - t0) * 1e-9 < 10.0:
        try:
            tr = buf.lookup_transform(TARGET_FRAME, SOURCE_FRAME,
                                      rclpy.time.Time())
            t = tr.transform.translation
            q = tr.transform.rotation
            node.destroy_node()
            return [t.x, t.y, t.z], [q.x, q.y, q.z, q.w]
        except Exception:
            import time
            time.sleep(0.2)
    node.destroy_node()
    raise Fail(f'no TF {SOURCE_FRAME} -> {TARGET_FRAME} within 10 s')


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument('--run', required=True, help='grasp run dir')
    ap.add_argument('--replay', default=None,
                    help='offline samples json (no ROS)')
    ap.add_argument('--out', default=None, help='prologue json path')
    ap.add_argument('--tf-json', default=CHAIN_FILE, help='static chain file')
    ap.add_argument('--use-tf', action='store_true',
                    help='live TF lookup instead of static file')
    a = ap.parse_args(argv)
    if a.replay:
        rep = json.load(open(a.replay))
        med = median_point([tuple(p) for p in rep['points']])
        check_mask_age(rep['mask_stamp'], rep['now'])
        t, q = rep['tf_t'], rep['tf_q']
        mode = 'replay'
    elif a.use_tf:
        import rclpy
        rclpy.init()
        node = rclpy.create_node('live_prologue')
        try:
            med, age = capture_live(node)
            t, q = lookup_tf()
            mode = f'live-tf (mask age {age:.1f} s)'
        finally:
            node.destroy_node()
            rclpy.shutdown()
    else:
        t, q = load_chain(a.tf_json)
        import rclpy
        rclpy.init()
        node = rclpy.create_node('live_prologue')
        try:
            med, age = capture_live(node)
            mode = f'live (mask age {age:.1f} s)'
        finally:
            node.destroy_node()
            rclpy.shutdown()
    pick_pos, pick_quat, A0 = load_grasp_refs(a.run, t, q)
    live_link0 = apply_transform(t, q, med)
    P, delta = shift_pick(pick_pos, A0, live_link0)
    out = dict(mode=mode, grasp_anchor=A0, live_link0=live_link0,
               delta_m=delta, pick_live_m=P, quat_xyzw=pick_quat)
    out_path = a.out or os.path.join(a.run, 'prologue.json')
    json.dump(out, open(out_path, 'w'), indent=1)
    exp_path = os.path.join(os.path.dirname(out_path) or '.', 'prologue.env')
    line = write_export(exp_path, P)
    print(f'prologue [{mode}]: pick_live={[round(v, 4) for v in P]}')
    print(f'source {exp_path}: {line}')
    return 0


if __name__ == '__main__':
    try:
        sys.exit(main())
    except Fail as e:
        print(f'PROLOGUE FAIL: {e}', file=sys.stderr)
        sys.exit(2)
