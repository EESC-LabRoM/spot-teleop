#!/usr/bin/env python3
"""Grasp loop: seed -> mask -> cloud -> grasp -> carve -> request. THE one script.

Runs inside vm-grasp-loop on VM103 (domain 8 over the cable). Lab is touched only
via explicit ssh steps (GraspGen infer; live request). Default is DRY (no motion).

  # full dry loop on the drill (bbox_1000 x1,y1,x2,y2):
  ./run_loop.sh --seed 560,470,675,695 --label drill
  # single stage / subset:
  ./run_loop.sh --seed ... --from 3 --to 4
  # LIVE request (person at e-stop, interactive GO):
  ./run_loop.sh --seed ... --live --estop "Name"
"""
import argparse
import json
import os
import subprocess
import sys
import time
from datetime import datetime, timezone

import numpy as np

LAB = 'murilo@100.88.134.127'
KEY = '/ssh/id_lab'
SSH = ['ssh', '-i', KEY, '-o', 'IdentitiesOnly=yes', '-o', 'BatchMode=yes',
       '-o', 'ConnectTimeout=10', LAB]

T = dict(rgb='/vm/cable/rgb',
         depth='/vm/cable/depth',
         depth_info='/zed1/zed_node/depth/camera_info',
         mask='/zed1/segmentation_mask',
         vm_mask='/vm/target_mask',
         seed_cmd='/perception/seed_command',
         seed_3d='/tracking/seed_3d',
         state='/tracking_state',
         esdf_prod='/nvblox_curobo/get_esdf_and_gradient',
         esdf_seg='/nvblox_seg/get_esdf_and_gradient',
         mesh_seg='/nvblox_seg/mesh')


class Fail(Exception):
    pass


def log(s):
    print(f'[loop {datetime.now(timezone.utc).strftime("%H:%M:%S")}] {s}', flush=True)


def sh(cmd, timeout=120, check=False):
    p = subprocess.run(cmd, capture_output=True, text=True, timeout=timeout)
    if check and p.returncode != 0:
        raise Fail(f'cmd failed {p.returncode}: {" ".join(cmd[:6])}: {p.stderr.strip()[:300]}')
    return p


def ssh(cmd, timeout=300):
    return sh(SSH + [cmd], timeout=timeout, check=True)


def erode_mask_np(M, px=2):
    """Binary erosion (pure numpy, no cv2): trims mask edge bleed that admits
    table/tag pixels at the silhouette border. M: uint8 0/255."""
    B = (M > 0)
    if px <= 0 or not B.any():
        return (B * 255).astype(np.uint8)
    K = B.copy()
    for d in range(1, px + 1):
        K[ d:, :] &= B[:-d, :]; K[:-d, :] &= B[d:, :]
        K[:,  d:] &= B[:, :-d]; K[:, :-d] &= B[:, d:]
    return (K * 255).astype(np.uint8)


def floor_cut(P, z_floor=0.015, max_drop_frac=0.6):
    """Drop table points by known geometry (table surface ~= z 0 in fr3_link0).

    Returns (kept, dropped_frac). Raises Fail when most of the cloud is below
    the floor: the "object" is table, i.e. the tracker lost it.
    """
    keep = P[:, 2] >= z_floor
    frac = 1.0 - float(keep.mean()) if len(P) else 1.0
    if frac > max_drop_frac:
        raise Fail(f'cloud {100*frac:.0f}% below table floor '
                   f'({len(P)} pts) — tracker lost the object?')
    return P[keep], frac


def tcp_of_pose(R44):
    """Fingertip (TCP) = hand origin + 103 mm along the approach (+tool-z)."""
    R44 = np.asarray(R44, float).reshape(4, 4)
    return R44[:3, 3] + R44[:3, 2] * 0.103


def grasp_metrics(Pc, R44, center_xy, top_z):
    """Agnostic per-grasp geometry metrics (pure numpy, no ROS).

    Pc: (N,3) object cloud in fr3_link0. R44: 4x4 grasp pose (hand frame).
    center_xy: robust object center (XY). top_z: object top height.
    Closing axis is GraspGen's canonical X (fingers straddle it).
    """
    R44 = np.asarray(R44, float).reshape(4, 4)
    pos = R44[:3, 3]
    tcp = tcp_of_pose(R44)
    d = np.linalg.norm(np.asarray(Pc, float) - tcp, axis=1)
    near = np.asarray(Pc, float)[d < 0.040] - tcp
    nnear = int(len(near))
    if nnear:
        s = near @ R44[:3, 0]
        nL, nR = int((s >= 0).sum()), int((s < 0).sum())
        bil = min(nL, nR) / max(nL, nR, 1)
    else:
        bil = 0.0
    return dict(tcp=tcp, nnear=nnear, bilateral=float(bil),
                wrap=float(top_z - tcp[2]),
                off_xy=float(np.linalg.norm(pos[:2] - np.asarray(center_xy, float)[:2])),
                vert=float(-R44[2, 2]))


def rank_score(conf, m, sigma, cloud_n):
    """Combined agnostic score: confidence x centering x contact.

    sigma: object XY radius (half of max horizontal extent) — the centering
    falloff scales with the object, not a tuned constant. cloud_n sizes the
    density term (10% of the cloud near the TCP = full marks).
    Returns (score, breakdown dict).
    """
    import math
    center_w = math.exp(-(m['off_xy'] ** 2) / (2 * sigma * sigma))
    density_w = min(1.0, m['nnear'] / max(150.0, 0.10 * cloud_n))
    bilateral_w = 0.3 + 0.7 * m['bilateral']
    w = m['wrap']
    if w < 0.0:
        wrap_w = max(0.0, 1.0 + w / 0.015)  # TCP above object top: fingertip graze
    elif w <= 0.060:
        wrap_w = 1.0 if w >= 0.015 else w / 0.015  # 15-60 mm = finger channel
    else:
        wrap_w = max(0.3, 1.0 - (w - 0.060) / 0.090 * 0.7)  # deep: slab/table risk
    contact_w = density_w * bilateral_w * wrap_w
    return float(conf) * center_w * contact_w, dict(
        center_w=center_w, density_w=density_w, bilateral_w=bilateral_w,
        wrap_w=wrap_w, contact_w=contact_w)


def rank_grasps(G, Pc, center_xy, extent_xy, top_z, vert_min=0.7):
    """Rank grasp rows by the combined score. Returns sorted [(idx, score,
    metrics, breakdown)], best first. Side grasps below vert_min are gated
    (the live strategy descends 5 cm straight down, so the approach must be
    near-vertical); the gate relaxes rather than starving the pool."""
    G = np.asarray(G, float)
    Pc = np.asarray(Pc, float)
    sigma = min(0.12, max(0.03, 0.5 * float(np.max(np.asarray(extent_xy, float)))))
    pool = list(range(G.shape[0]))
    gate = vert_min
    for _ in range(3):
        keep = [i for i in pool if float(-G[i, :16].reshape(4, 4)[2, 2]) >= gate]
        if len(keep) >= 3 or gate <= 0.0:
            pool = keep if keep else pool
            break
        gate = 0.5 if gate > 0.5 else 0.0
    out = []
    for i in pool:
        R = G[i, :16].reshape(4, 4)
        m = grasp_metrics(Pc, R, center_xy, top_z)
        s, b = rank_score(float(G[i, 16]), m, sigma, len(Pc))
        out.append((i, s, m, b))
    out.sort(key=lambda t: -t[1])
    return out


def density_gate(G, Pc, cand, radius=0.040, min_pts=150, fallback=5):
    """Keep candidates whose fingertip (TCP) sits in real geometry.

    Density is measured around the TCP, never around the hand origin:
    the fingers close around the TCP, so that is where the object must be.
    If fewer than <fallback> candidates are dense, keep the top-<fallback>
    by confidence and report starved=True (better a flagged grasp than a
    dead run). Returns (kept, starved, scored). Pure numpy, no ROS.
    """
    G = np.asarray(G, float)
    Pc = np.asarray(Pc, float)
    scored = []
    for i in cand:
        R = G[i, :16].reshape(4, 4)
        tcp = tcp_of_pose(R)
        nnear = int((np.linalg.norm(Pc - tcp, axis=1) < radius).sum())
        scored.append((i, nnear))
    dense = [i for i, nnear in scored if nnear >= min_pts]
    if len(dense) >= fallback:
        return dense, False, scored
    scored.sort(key=lambda t: float(G[t[0], 16]), reverse=True)
    return [i for i, _ in scored[:fallback]], True, scored


def quat_from_matrix(R):
    t = np.trace(R)
    if t > 0:
        s = 0.5 / np.sqrt(t + 1.0)
        return np.array([ (R[2, 1] - R[1, 2]) * s, (R[0, 2] - R[2, 0]) * s,
                          (R[1, 0] - R[0, 1]) * s, 0.25 / s ])
    i = int(np.argmax([R[0, 0], R[1, 1], R[2, 2]]))
    if i == 0:
        s = 2.0 * np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2])
        return np.array([0.25 * s, (R[0, 1] + R[1, 0]) / s,
                         (R[0, 2] + R[2, 0]) / s, (R[2, 1] - R[1, 2]) / s])
    if i == 1:
        s = 2.0 * np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2])
        return np.array([(R[0, 1] + R[1, 0]) / s, 0.25 * s,
                         (R[1, 2] + R[2, 1]) / s, (R[0, 2] - R[2, 0]) / s])
    s = 2.0 * np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1])
    return np.array([(R[0, 2] + R[2, 0]) / s, (R[1, 2] + R[2, 1]) / s,
                     0.25 * s, (R[1, 0] - R[0, 1]) / s])


def wait_msg(node, topic, typ, timeout=30.0):
    import rclpy
    from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
    for rel in (ReliabilityPolicy.RELIABLE, ReliabilityPolicy.BEST_EFFORT):
        got = {}
        qos = QoSProfile(depth=10, reliability=rel, history=HistoryPolicy.KEEP_LAST)
        sub = node.create_subscription(
            typ, topic, lambda m: got.setdefault('m', m), qos)
        t0 = time.time()
        while 'm' not in got and time.time() - t0 < timeout / 2:
            rclpy.spin_once(node, timeout_sec=0.2)
        node.destroy_subscription(sub)
        if 'm' in got:
            return got['m']
    return None


def call_esdf(node, service, amin, asize, timeout=30.0):
    import rclpy
    from nvblox_msgs.srv import EsdfAndGradients
    from geometry_msgs.msg import Point, Vector3
    cli = node.create_client(EsdfAndGradients, service)
    if not cli.wait_for_service(timeout_sec=10.0):
        raise Fail(f'no service {service}')
    req = EsdfAndGradients.Request()
    req.update_esdf = True
    req.use_aabb = True
    req.frame_id = 'fr3_link0'
    req.aabb_min_m = Point(x=amin[0], y=amin[1], z=amin[2])
    req.aabb_size_m = Vector3(x=asize[0], y=asize[1], z=asize[2])
    t0 = time.time()
    fut = cli.call_async(req)
    rclpy.spin_until_future_complete(node, fut, timeout_sec=timeout)
    dt = time.time() - t0
    res = fut.result()
    if res is None or not res.success:
        raise Fail(f'esdf call failed on {service}')
    dims = {d.label: d.size for d in res.esdf_and_gradients.layout.dim}
    E = np.array(res.esdf_and_gradients.data, np.float32).reshape(
        dims['x'], dims['y'], dims['z'])
    O = np.array([res.origin_m.x, res.origin_m.y, res.origin_m.z])
    return dict(esdf=E, origin=O, voxel=float(res.voxel_size_m), ms=dt * 1000)


def s1_seed(node, ctx, a):
    """Publish ONE seed, verify seed_3d + TRACKING."""
    import rclpy
    from std_msgs.msg import String
    from geometry_msgs.msg import PointStamped
    x1, y1, x2, y2 = [int(v) for v in a.seed.split(',')]
    # bridge alive?
    from sensor_msgs.msg import Image as _I  # noqa (keeps import surface obvious)
    p = sh(['docker', 'ps', '--filter', 'name=vm-px-bridge', '--format', '{{.Names}}'])
    if 'vm-px-bridge' not in p.stdout:
        log('seed bridge not running, starting it')
        here0 = os.path.dirname(os.path.abspath(__file__))
        off0 = os.path.join(here0, '..', 'vm_offload')
        subprocess.run([f'{off0}/run_vm_seed_bridge.sh'], capture_output=True,
                       timeout=60, check=True)
        time.sleep(5)
    from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
    # subscribe BEFORE seeding: seed_3d is transient (one msg per apply)
    qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE,
                     history=HistoryPolicy.KEEP_LAST)
    got = {}
    node.create_subscription(PointStamped, T['seed_3d'],
                             lambda m: got.setdefault('m', m), qos)
    rclpy.spin_once(node, timeout_sec=3.0)  # let discovery settle
    pub = node.create_publisher(String, T['seed_cmd'], 10)
    seed = json.dumps({'bbox_1000': [x1, y1, x2, y2], 'label': a.label, 'confidence': 1.0})
    s3 = None
    for attempt in range(3):  # fresh tracker may need frames buffered first
        for _ in range(2):
            pub.publish(String(data=seed))
            rclpy.spin_once(node, timeout_sec=0.7)
        log(f'seed sent (try {attempt + 1}) {seed}')
        t0 = time.time()
        while 'm' not in got and time.time() - t0 < 25:
            rclpy.spin_once(node, timeout_sec=0.2)
        s3 = got.get('m')
        if s3 is not None:
            break
        got.clear()
        time.sleep(10)
    if s3 is None:
        raise Fail('no seed_3d after seed (tracker deaf?)')
    # drive video init directly: project seed_3d to pixels (bridge is backup only,
    # it can miss the transient seed_3d)
    from sensor_msgs.msg import CameraInfo
    info = wait_msg(node, T['depth_info'], CameraInfo, timeout=20.0)
    if info is None:
        raise Fail('no camera_info for seed projection')
    K = np.array(info.k).reshape(3, 3)
    u = K[0, 0] * (s3.point.x / s3.point.z) + K[0, 2]
    v = K[1, 1] * (s3.point.y / s3.point.z) + K[1, 2]
    pxpub = node.create_publisher(PointStamped, '/tracking/seed_pixel', 10)
    t0 = time.time()
    while time.time() - t0 < 15:
        m = PointStamped()
        m.header.frame_id = info.header.frame_id
        m.header.stamp = node.get_clock().now().to_msg()
        m.point.x, m.point.y, m.point.z = float(u), float(v), float(s3.point.z)
        pxpub.publish(m)
        rclpy.spin_once(node, timeout_sec=0.1)
    log(f'seed_pixel driven at ({u:.0f}, {v:.0f})')
    t0 = time.time()
    st = None
    while time.time() - t0 < 60:
        m = wait_msg(node, T['state'], String, timeout=6.0)
        if m is not None:
            st = m.data
            if st == 'TRACKING':
                break
    if st != 'TRACKING':
        raise Fail(f'tracker state {st!r}, want TRACKING')
    out = dict(bbox=[x1, y1, x2, y2], label=a.label,
               seed_3d=[s3.point.x, s3.point.y, s3.point.z], state=st)
    json.dump(out, open(f'{ctx["run"]}/seed.json', 'w'), indent=1)
    log(f'S1 PASS tracking, seed_3d ({s3.point.x:.3f}, {s3.point.y:.3f}, {s3.point.z:.3f})')
    return out


def s2_mask(node, ctx, a):
    """Verify fresh, sane-coverage tracker mask; save snapshot."""
    import cv2
    import rclpy
    from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
    from sensor_msgs.msg import Image
    qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE,
                     history=HistoryPolicy.KEEP_LAST)
    cov, last = [], {}
    node.create_subscription(Image, T['mask'], lambda m: (
        cov.append(float((np.frombuffer(m.data, np.uint8) > 0).mean())),
        last.setdefault('m', m)), qos)
    t0 = time.time()
    while time.time() - t0 < 25 and len(cov) < 5:
        rclpy.spin_once(node, timeout_sec=0.2)
    if len(cov) < 3:
        raise Fail('mask too slow (<3 in 25s)')
    c = float(np.mean(cov[-5:]))
    m = last['m']
    arr = np.frombuffer(m.data, np.uint8).reshape(m.height, m.width)
    cv2.imwrite(f'{ctx["run"]}/mask_raw.png', (arr > 0).astype(np.uint8) * 255)
    sj = json.load(open(f'{ctx["run"]}/seed.json'))
    x1, y1, x2, y2 = sj['bbox']
    X1 = max(int(x1 / 1000 * m.width) - 15, 0)
    Y1 = max(int(y1 / 1000 * m.height) - 15, 0)
    X2 = min(int(x2 / 1000 * m.width) + 15, m.width)
    Y2 = min(int(y2 / 1000 * m.height) + 15, m.height)
    roi = np.zeros_like(arr)
    roi[Y1:Y2, X1:X2] = arr[Y1:Y2, X1:X2]
    arr = roi  # effective mask: what s3/s5 actually consume
    cv2.imwrite(f'{ctx["run"]}/mask.png', (arr > 0).astype(np.uint8) * 255)
    c = float((arr > 0).mean())
    if not (a.min_cov <= c <= a.max_cov):
        raise Fail(f'mask-in-ROI coverage {100 * c:.2f}% outside [{100 * a.min_cov:.1f}, {100 * a.max_cov:.0f}]% (snap saved)')
    sp = np.array([(x1 + x2) / 2 / 1000 * m.width, (y1 + y2) / 2 / 1000 * m.height])
    ys, xs = np.nonzero(arr > 0)
    dd = float(np.linalg.norm(np.array([xs.mean(), ys.mean()]) - sp))
    if dd > 100:
        raise Fail(f'mask off-target (centroid {dd:.0f}px from seed; snap saved)')
    out = dict(coverage=c, shape=[m.width, m.height], n=len(cov), centroid_off_px=dd)
    json.dump(out, open(f'{ctx["run"]}/mask.json', 'w'), indent=1)
    log(f'S2 PASS mask {100 * c:.2f}%')
    return out

def s3_cloud(node, ctx, a):
    """Capture masked depth -> object cloud in fr3_link0."""
    import rclpy
    import tf2_ros
    from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
    from sensor_msgs.msg import Image, CameraInfo
    info = wait_msg(node, T['depth_info'], CameraInfo, timeout=20.0)
    if info is None:
        raise Fail('no depth camera_info')
    K = np.array(info.k).reshape(3, 3)
    fx, fy, cx, cy = K[0, 0], K[1, 1], K[0, 2], K[1, 2]
    tfbuf = tf2_ros.Buffer()
    tf2_ros.TransformListener(tfbuf, node)
    qos_be = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT,
                        history=HistoryPolicy.KEEP_LAST)
    qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE,
                     history=HistoryPolicy.KEEP_LAST)
    from collections import deque
    mask_buf = deque(maxlen=12)
    node.create_subscription(Image, T['mask'],
                             lambda m: mask_buf.append(m), qos)
    pairs = []
    dbg = dict(depth=0, mask_seen=0, stamp_ok=0, tf_ok=0)
    def on_depth(d):
        if len(pairs) >= 5:
            return
        dbg['depth'] += 1
        if not mask_buf:
            return
        dbg['mask_seen'] += 1
        ds = d.header.stamp.sec + d.header.stamp.nanosec * 1e-9
        best, bdt = None, 3.0
        for m in mask_buf:
            ms = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
            if abs(ds - ms) < bdt:
                best, bdt = m, abs(ds - ms)
        if best is None:
            return
        m = best
        dbg['stamp_ok'] += 1
        try:
            tf = tfbuf.lookup_transform(
                'fr3_link0', d.header.frame_id,
                rclpy.time.Time(seconds=ds), timeout=rclpy.duration.Duration(seconds=1.0))
        except Exception:
            return
        dbg['tf_ok'] += 1
        pairs.append((d, m, tf))
    node.create_subscription(Image, T['depth'], on_depth, qos_be)
    t0 = time.time()
    while time.time() - t0 < 15:  # discovery settle for the fresh subs
        rclpy.spin_once(node, timeout_sec=0.2)
    log(f's3 settle: depth seen {dbg["depth"]}, mask {dbg["mask_seen"]}')
    dbg['depth'] = 0
    t0 = time.time()
    while time.time() - t0 < 60 and len(pairs) < 5:
        rclpy.spin_once(node, timeout_sec=0.2)
    if len(pairs) < 3:
        raise Fail(f'only {len(pairs)} depth+mask pairs (want >=3): {dbg}')
    sj = json.load(open(f'{ctx["run"]}/seed.json'))
    bx1, by1, bx2, by2 = sj['bbox']
    z0 = float(sj['seed_3d'][2])  # camera-frame depth of the seed point
    pts = []
    RT0 = None
    for d, m, tf in pairs:
        D = np.frombuffer(d.data, np.float32).reshape(d.height, d.width)
        M = np.frombuffer(m.data, np.uint8).reshape(m.height, m.width)
        if M.shape != D.shape:
            import cv2
            M = cv2.resize(M, (d.width, d.height), interpolation=cv2.INTER_NEAREST)
        M = erode_mask_np(M, 2)  # trim silhouette bleed (table/tag at edges)
        # Grown ROI (same +75%/side recipe as mask-sync): the tight seed box is a
        # prompt, not a gate — gating on it clips ~1/3 of valid mask pixels.
        gx, gy = 0.75 * (bx2 - bx1), 0.75 * (by2 - by1)
        X1 = max(int((bx1 - gx) / 1000 * d.width), 0)
        Y1 = max(int((by1 - gy) / 1000 * d.height), 0)
        X2 = min(int((bx2 + gx) / 1000 * d.width), d.width)
        Y2 = min(int((by2 + gy) / 1000 * d.height), d.height)
        inroi = np.zeros_like(M)
        inroi[Y1:Y2, X1:X2] = 1
        ys, xs = np.nonzero((M > 0) & (inroi > 0) & (D > 0.1) & (D < 2.5)
                             & (np.abs(D - z0) < 0.12))
        if len(xs) == 0:
            continue
        z = D[ys, xs]
        pc = np.stack([(xs - cx) * z / fx, (ys - cy) * z / fy, z], axis=1)
        t, q = tf.transform.translation, tf.transform.rotation
        n = np.array([q.x, q.y, q.z, q.w], float)
        n /= np.linalg.norm(n)
        x, y, zc, w = n
        R = np.array([[1 - 2 * (y * y + zc * zc), 2 * (x * y - zc * w), 2 * (x * zc + y * w)],
                      [2 * (x * y + zc * w), 1 - 2 * (x * x + zc * zc), 2 * (y * zc - x * w)],
                      [2 * (x * zc - y * w), 2 * (y * zc + x * w), 1 - 2 * (x * x + y * y)]])
        if RT0 is None:
            RT0 = (R.copy(), np.array([t.x, t.y, t.z]))
        pts.append(pc @ R.T + np.array([t.x, t.y, t.z]))
    P = np.concatenate(pts, axis=0)
    P = P[(P[:, 0] > -0.2) & (P[:, 0] < 1.2) & (P[:, 1] > -1.2) & (P[:, 1] < 0.6)
          & (P[:, 2] > -0.1) & (P[:, 2] < 1.2)]
    # seed-anchored cluster keep: SAM2 masks bleed onto walls/clutter, so keep
    # the connected component around the seed ray (ROI center @ seed depth).
    from collections import deque as _dq
    uc = (bx1 + bx2) / 2 / 1000 * d.width
    vc = (by1 + by2) / 2 / 1000 * d.height
    pc0 = np.array([(uc - cx) * z0 / fx, (vc - cy) * z0 / fy, z0])
    _R0, _t0 = RT0
    anchor = pc0 @ _R0.T + _t0
    vox = np.floor(P / 0.025).astype(np.int64)
    keys, inv = np.unique(vox, axis=0, return_inverse=True)
    k2i = {tuple(k): i for i, k in enumerate(keys)}
    nbrs = [(dx, dy, dz) for dx in (-1, 0, 1) for dy in (-1, 0, 1)
            for dz in (-1, 0, 1) if dx or dy or dz]
    seen = np.zeros(len(keys), bool)
    comps = []
    for i in range(len(keys)):
        if seen[i]:
            continue
        q = _dq([i]); seen[i] = True; cur = [i]
        while q:
            kj = tuple(keys[q.popleft()])
            for ox, oy, oz in nbrs:
                l = k2i.get((kj[0] + ox, kj[1] + oy, kj[2] + oz))
                if l is not None and not seen[l]:
                    seen[l] = True; q.append(l); cur.append(l)
        comps.append(cur)
    best, bi = 1e9, -1
    for ci, cur in enumerate(comps):
        dd = float(np.linalg.norm(keys[cur].mean(axis=0) * 0.025 - anchor))
        if dd < best:
            best, bi = dd, ci
    if bi < 0 or best > 0.15:
        raise Fail(f'no cluster near seed ray (nearest {best:.2f} m, {len(comps)} comps)')
    keep_v = np.zeros(len(keys), bool)
    keep_v[comps[bi]] = True
    P = P[keep_v[inv]]
    log(f'cluster keep: {len(comps)} comps -> {len(P)} pts near seed (d {best:.3f} m)')
    # Table cut by known geometry (table surface ~= z 0 in fr3_link0).
    # RANSAC fitting proved fragile here (degenerate vertical fits on the
    # drill column); a z floor is exact and fails loud when the tracker is
    # lost (see floor_cut).
    n0 = len(P)
    P, cut_frac = floor_cut(P)
    log(f'table cut: z<0.015 drop {n0 - len(P)}/{n0} pts')
    if len(P) > 20000:
        P = P[np.random.choice(len(P), 20000, replace=False)]
    if len(P) < 200:
        raise Fail(f'cloud too small ({len(P)} pts)')
    np.save(f'{ctx["run"]}/cloud.npy', P.astype(np.float32))
    out = dict(n=int(len(P)), centroid=P.mean(axis=0).round(4).tolist(),
               top=float(P[:, 2].max()),
               extent=(P.max(axis=0) - P.min(axis=0)).round(4).tolist())
    json.dump(out, open(f'{ctx["run"]}/cloud.json', 'w'), indent=1)
    log(f'S3 PASS cloud {len(P)} pts centroid {out["centroid"]} top {out["top"]:.3f}')
    return out


def s4_grasp(node, ctx, a, cloud):
    """GraspGen-X on lab GPU -> (M,17) + best pick. Cloud/matrix math on VM."""
    here = os.path.dirname(os.path.abspath(__file__))
    rdir = f'/tmp/grasp_loop_{ctx["id"]}'
    ssh(f'mkdir -p {rdir}')
    sh(['scp', '-i', KEY, '-o', 'IdentitiesOnly=yes', '-o', 'BatchMode=yes',
        f'{ctx["run"]}/cloud.npy', f'{here}/grasp_infer.py', f'{LAB}:{rdir}/'],
       timeout=60, check=True)
    p = ssh(f'cd {rdir} && CUDA_VISIBLE_DEVICES={a.cuda} timeout 600 '
            f'/home/murilo/graspgenx/.venv/bin/python grasp_infer.py cloud.npy grasps.npy --num 100',
            timeout=660)
    log('infer: ' + (p.stdout.strip().splitlines() or ['?'])[-1][:160])
    sh(['scp', '-i', KEY, '-o', 'IdentitiesOnly=yes', '-o', 'BatchMode=yes',
        f'{LAB}:{rdir}/grasps.npy', f'{ctx["run"]}/grasps.npy'], timeout=60, check=True)
    G = np.load(f'{ctx["run"]}/grasps.npy')
    if G.shape[0] == 0:
        raise Fail('GraspGen returned 0 grasps')
    C = np.array(cloud['centroid'])
    E = np.array(cloud.get('extent', [0.3, 0.3, 0.3]))
    lo, hi = C - E / 2 - 0.05, C + E / 2 + np.array([0.05, 0.05, 0.08])
    cand = [i for i in range(G.shape[0])
            if np.all(G[i, :16].reshape(4, 4)[:3, 3] >= lo)
            and np.all(G[i, :16].reshape(4, 4)[:3, 3] <= hi)]
    if not cand:
        raise Fail(f'0/{G.shape[0]} grasps inside cloud extent (sloppy mask?)')
    log(f'pick pool: {len(cand)}/{G.shape[0]} inside extent')
    # Density gate: the TCP (fingertip, 103 mm along the approach) must sit
    # in real geometry, not bleed/air. Drop candidates with <150 cloud
    # points within 40 mm of their TCP; if that starves the pool, keep the
    # top 5 by confidence anyway and warn loudly (better a flagged grasp
    # than a dead run).
    Pc = np.load(f'{ctx["run"]}/cloud.npy')[:, :3].astype(float)
    cand, starved, scored = density_gate(G, Pc, cand)
    if not starved:
        log(f'density gate: {len(cand)}/{len(scored)} with >=150 pts near TCP')
    else:
        n_dense = sum(1 for _, nnear in scored if nnear >= 150)
        log(f'WARN density gate starved ({n_dense} dense): '
            f'falling back to top-5 by confidence')
    # Rank: combined agnostic score (confidence x centering x contact) over
    # the valid pool. Robust center = cloud median (mean is table-biased).
    Cmed = np.median(Pc, axis=0)
    Gc = G[np.array(cand)]
    ranked = rank_grasps(Gc, Pc, Cmed[:2], E[:2], float(cloud['top']))
    order = [cand[i] for i, _, _, _ in ranked]
    for i, s, m, b in ranked[:5]:
        gi = cand[i]
        log(f'rank idx={gi:3d} score={s:.4f} conf={float(G[gi, 16]):.2f} '
            f'off={1000*m["off_xy"]:.0f}mm nnear={m["nnear"]} '
            f'bil={m["bilateral"]:.2f} wrap={1000*m["wrap"]:+.0f}mm vert={m["vert"]:+.2f}')
    bi = order[0]
    _, bs, bm, bb = ranked[0]
    R = G[bi, :16].reshape(4, 4)
    pos = R[:3, 3]
    pick = dict(pos=[float(v) for v in pos],
                quat_xyzw=[float(v) for v in quat_from_matrix(R[:3, :3])],
                conf=float(G[bi, 16]), n_total=int(G.shape[0]), idx=int(bi),
                score=float(bs),
                metrics=dict(off_xy_mm=round(1000 * bm['off_xy'], 1),
                             nnear=bm['nnear'], bilateral=round(bm['bilateral'], 3),
                             wrap_mm=round(1000 * bm['wrap'], 1),
                             vert=round(bm['vert'], 3)),
                weights={k: round(float(v), 4) for k, v in bb.items()})
    json.dump(pick, open(f'{ctx["run"]}/pick.json', 'w'), indent=1)
    json.dump([dict(idx=int(cand[i]), score=round(float(s), 5),
                     conf=round(float(G[cand[i], 16]), 4),
                     off_xy_mm=round(1000 * m['off_xy'], 1), nnear=m['nnear'],
                     bilateral=round(m['bilateral'], 3),
                     wrap_mm=round(1000 * m['wrap'], 1), vert=round(m['vert'], 3))
               for i, s, m, _ in ranked],
              open(f'{ctx["run"]}/rank.json', 'w'), indent=1)
    log(f'S4 PASS {G.shape[0]} grasps, pick ({pos[0]:.3f}, {pos[1]:.3f}, {pos[2]:.3f}) '
        f'conf {pick["conf"]:.2f} score {pick["score"]:.4f} '
        f'vert {bm["vert"]:+.2f} offxy {1000*bm["off_xy"]:.0f}mm')
    _save_pick_overlay(node, ctx, R, pos)
    return pick


def _save_pick_overlay(node, ctx, R, pos):
    """Project hand/TCP/approach onto the live frame (never fails the run)."""
    try:
        import rclpy
        import tf2_ros
        from sensor_msgs.msg import Image, CameraInfo
        rgb = wait_msg(node, T['rgb'], Image, timeout=15.0)
        info = wait_msg(node, T['depth_info'], CameraInfo, timeout=15.0)
        if rgb is None or info is None:
            log('WARN pick overlay: no rgb/camera_info'); return
        import cv2
        fr = np.frombuffer(rgb.data, np.uint8).reshape(rgb.height, rgb.width, 3)
        if rgb.encoding == 'rgb8':
            fr = cv2.cvtColor(fr, cv2.COLOR_RGB2BGR)
        K = np.array(info.k).reshape(3, 3)
        fx, fy, cx, cy = K[0, 0], K[1, 1], K[0, 2], K[1, 2]
        tfbuf = tf2_ros.Buffer()
        tf2_ros.TransformListener(tfbuf, node)
        tfm, t0 = None, time.time()
        while tfm is None and time.time() - t0 < 10:
            try:
                tfm = tfbuf.lookup_transform(rgb.header.frame_id, 'fr3_link0',
                                             rclpy.time.Time())
            except Exception:
                rclpy.spin_once(node, timeout_sec=0.2)
        if tfm is None:
            log('WARN pick overlay: no TF'); return
        t, q = tfm.transform.translation, tfm.transform.rotation
        n = np.array([q.x, q.y, q.z, q.w], float); n /= np.linalg.norm(n)
        x, y, zc, w = n
        Rt = np.array([[1 - 2 * (y * y + zc * zc), 2 * (x * y - zc * w), 2 * (x * zc + y * w)],
                       [2 * (x * y + zc * w), 1 - 2 * (x * x + zc * zc), 2 * (y * zc - x * w)],
                       [2 * (x * zc - y * w), 2 * (y * zc + x * w), 1 - 2 * (x * x + y * y)]])
        tt = np.array([t.x, t.y, t.z])
        def proj(p):
            pc = Rt @ np.asarray(p, float) + tt
            return (int(fx * pc[0] / pc[2] + cx), int(fy * pc[1] / pc[2] + cy))
        hand = proj(pos)
        tcp = pos + R[:3, 2] * 0.103  # 103 mm tool along tool-z
        tc = proj(tcp)
        ap = proj(pos - R[:3, 2] * 0.10)  # 10 cm back along approach
        cv2.arrowedLine(fr, ap, tc, (0, 0, 255), 2, tipLength=0.15)
        cv2.drawMarker(fr, tc, (0, 0, 255), cv2.MARKER_STAR, 22, 2)
        cv2.drawMarker(fr, hand, (255, 0, 255), cv2.MARKER_TILTED_CROSS, 20, 2)
        p = f'{ctx["run"]}/pick_overlay.png'
        cv2.imwrite(p, fr)
        log(f'pick overlay: hand {hand} tcp {tc} -> pick_overlay.png')
    except Exception as e:
        log(f'WARN pick overlay failed: {type(e).__name__}: {str(e)[:120]}')

def _seg_ready(node, extra_settle=10, timeout=220.0):
    """Readiness via ESDF service (mesh subscription is flaky VM-local):
    poll the table region until observed voxels appear."""
    t0 = time.time()
    while time.time() - t0 < timeout:
        try:
            rt = call_esdf(node, T['esdf_seg'], np.array([0.32, -0.03, 0.0]),
                           np.array([0.06, 0.06, 0.05]))
        except Fail:
            time.sleep(10)
            continue
        f = rt['esdf'][np.isfinite(rt['esdf'])]
        if len(f[f > -999.0]) > 0:
            break
        time.sleep(15)
    else:
        raise Fail('seg mapper ESDF never observed the table region')
    time.sleep(extra_settle)


def s5_carve(node, ctx, a, cloud):
    """Seg mapper on VM + verify the target is carved out of it."""
    here = os.path.dirname(os.path.abspath(__file__))
    off = os.path.join(here, '..', 'vm_offload')
    sj = json.load(open(f'{ctx["run"]}/seed.json'))
    # ROI is the carve gate, not the seed prompt: grow the seed box 75% per
    # side so the whole object passes (a tight prompt box would clip limbs).
    x1, y1, x2, y2 = sj['bbox']
    gx, gy = int(0.75 * (x2 - x1)), int(0.75 * (y2 - y1))
    roi = [max(0, x1 - gx), max(0, y1 - gy),
           min(1000, x2 + gx), min(1000, y2 + gy)]
    log(f'(re)starting vm-mask-sync with ROI {roi} (seed {sj["bbox"]})')
    sh(['docker', 'rm', '-f', 'vm-mask-sync'], timeout=30)
    env = dict(os.environ, ROI=','.join(str(v) for v in roi))
    subprocess.run([f'{here}/run_vm_mask_sync.sh'], env=env, capture_output=True,
                   timeout=60, check=True)
    from sensor_msgs.msg import Image
    if wait_msg(node, T['vm_mask'], Image, timeout=40.0) is None:
        raise Fail('no /vm/target_mask (mask sync dead?)')
    # warm-mask gate: a fresh seg mapper started while masks are still empty
    # bakes the target into static geometry permanently. Require 3 non-empty.
    import rclpy as _r
    from rclpy.qos import QoSProfile as _Q, ReliabilityPolicy as _R, HistoryPolicy as _H
    _q = _Q(depth=5, reliability=_R.BEST_EFFORT, history=_H.KEEP_LAST)
    _got = {}
    node.create_subscription(Image, T['vm_mask'], lambda m: _got.setdefault('m', m), _q)
    _full = 0
    t0 = time.time()
    while time.time() - t0 < 60 and _full < 3:
        _r.spin_once(node, timeout_sec=0.2)
        m = _got.pop('m', None)
        if m is not None and (np.frombuffer(m.data, np.uint8) > 0).mean() > 0.0005:
            _full += 1
    if _full < 3:
        raise Fail('target mask never went non-empty (tracker lost it?)')
    log('mask warm (3 non-empty), starting vm-nvblox-seg fresh (SEG=1)')
    sh(['docker', 'rm', '-f', 'vm-nvblox-seg'], timeout=30)
    env = dict(os.environ, SEG='1')
    subprocess.run([f'{off}/run_vm_nvblox.sh'], env=env, capture_output=True,
                   timeout=60, check=True)
    _seg_ready(node)
    def occ_min(rr):
        # -1000 = unknown/unobserved (a successful carve reads all-unknown);
        # occupied geometry reads small/negative. Only observed voxels count.
        f = rr['esdf'][np.isfinite(rr['esdf'])]
        k = f[f > -999.0]
        return (float(k.min()) if len(k) else 1e9), len(k), int(f.size)
    time.sleep(20)
    # verify the PICK cell + approach corridor (what the motion needs), not the
    # centroid: low target voxels persist via truncation bleed, but the grasp
    # works in the carved bubble above them. Fall back to centroid pre-S4.
    _pp = os.path.join(ctx['run'], 'pick.json')
    if os.path.exists(_pp):
        # Carve must cover where the target IS, not where S4 saw it.
        C = reanchor_pick(node, ctx, a, json.load(open(_pp))['pos'])
        amin, asize = C - np.array([0.03, 0.03, 0.03]), np.array([0.06, 0.06, 0.16])
    else:
        C = np.array(cloud['centroid'])
        amin, asize = C - 0.03, np.array([0.06, 0.06, 0.06])
    r = call_esdf(node, T['esdf_seg'], amin, asize)
    dmin, dknown, dtot = occ_min(r)
    if dmin < 0.02:
        log('stale drill geometry? rebuilding seg mapper once with the ROI mask')
        sh(['docker', 'rm', '-f', 'vm-nvblox-seg'], timeout=30)
        env = dict(os.environ, SEG='1')
        subprocess.run([f'{off}/run_vm_nvblox.sh'], env=env, capture_output=True,
                       timeout=60, check=True)
        _seg_ready(node, extra_settle=30)
        r = call_esdf(node, T['esdf_seg'], amin, asize)
        dmin, dknown, dtot = occ_min(r)
    rt = call_esdf(node, T['esdf_seg'], np.array([0.32, -0.03, 0.0]), np.array([0.06, 0.06, 0.05]))
    tmin, tknown, ttot = occ_min(rt)
    if tknown == 0:
        raise Fail('seg map empty (table region all unknown)')
    out = dict(drill_min_dist_m=dmin, drill_known=f'{dknown}/{dtot}',
               table_min_dist_m=tmin, table_known=f'{tknown}/{ttot}', esdf_ms=r['ms'])
    json.dump(out, open(f'{ctx["run"]}/carve.json', 'w'), indent=1)
    if tmin > 0.01:
        log(f'WARN table reads free ({tmin:.3f} m; black-table depth gaps?)')
    if dmin < 0.02:
        raise Fail(f'target NOT carved (min dist {dmin:.3f} m)')
    log(f'S5 PASS carved (target free {dmin:.3f} m known {dknown}/{dtot}, table {tmin:.3f} m)')
    return out


def live_hand():
    """Live hand pose via lab joints (read-only) + cuRobo FK."""
    p = ssh("docker exec fr3-ros2 bash -c 'source /opt/ros/jazzy/setup.bash;"
            " source /ros2_ws/install/setup.bash; export ROS_DOMAIN_ID=77;"
            " timeout 8 ros2 topic echo --once --field position /joint_states'")
    import re
    q = re.findall(r'-?\d+\.\d+(?:e-?\d+)?', p.stdout)[:7]
    if len(q) != 7:
        raise Fail('could not read live joints')
    qs = ','.join(q)
    p = sh(['bash', '-c',
            'cd /home/nexus/spot-teleop/isaac-sim_ws/franka_mpc/deploy_sim && source ds_env.sh && '
            f'"${{DSRUN[@]}}" --gpus device=1 $IMG /workspace/franka_mpc/deploy_sim/fk_q.py "{qs}"'],
           timeout=240, check=True)
    m = secrets_match(p.stdout)
    if m is None:
        raise Fail('FK failed')
    return m


def secrets_match(out):
    import re
    p0 = re.search(r'^P0\s+([-\d.,]+)', out, re.M)
    qu = re.search(r'^QUAT_XYZW\s+([-\d.,]+)', out, re.M)
    if not p0 or not qu:
        return None
    return dict(p0=[float(v) for v in p0.group(1).split(',')],
                quat=[float(v) for v in qu.group(1).split(',')])


def leg_clearance(node, service, legs, step=0.02):
    """Min ESDF along legs (list of (a, b) arrays). One query per leg bbox.
    -1000 (unknown/unobserved) is excluded like the planner treats it as free;
    returns (worst, known_frac)."""
    worst, known, total = 1e9, 0, 0
    for (a, b) in legs:
        L = float(np.linalg.norm(b - a))
        n = max(int(L / step), 1) + 1
        pts = np.linspace(a, b, n)
        lo = pts.min(axis=0) - 0.03
        sz = pts.max(axis=0) - pts.min(axis=0) + 0.06
        r = call_esdf(node, service, lo, sz)
        E, O, V = r['esdf'], r['origin'], r['voxel']
        idx = np.clip(((pts - O) / V).astype(int), 0, np.array(E.shape) - 1)
        d = E[idx[:, 0], idx[:, 1], idx[:, 2]]
        d = d[np.isfinite(d)]
        total += len(d)
        k = d[d > -999.0]
        known += len(k)
        if len(k):
            worst = min(worst, float(k.min()))
    frac = known / total if total else 0.0
    return (worst if known else 1e9), frac


def reanchor_pick(node, ctx, a, pos):
    """Shift a grasp-time position by the target's live rigid motion.

    Re-captures with the SAME s3 estimator into <run>/reanchor/ (never clobbers
    cloud.json) and returns pos + (live_centroid - grasp_centroid). Idempotent:
    always derived from the stable (cloud.json, live) pair, so S5 and S6 can
    both call it. Fails the run if the tracker lost the target or it jumped
    >15 cm (re-run S1-S4 then).
    """
    import shutil
    ra_dir = os.path.join(ctx['run'], 'reanchor')
    os.makedirs(ra_dir, exist_ok=True)
    shutil.copy(f'{ctx["run"]}/seed.json', f'{ra_dir}/seed.json')
    ra_ctx = dict(run=ra_dir, id=ctx['id'] + '_ra')
    try:
        live = s3_cloud(node, ra_ctx, a)
    except Fail as e:
        raise Fail(f're-anchor capture failed (tracker lost target?): {e}')
    C0 = np.array(json.load(open(f'{ctx["run"]}/cloud.json'))['centroid'])
    C1 = np.array(live['centroid'])
    delta = C1 - C0
    djump = float(np.linalg.norm(delta))
    if djump > 0.15:
        raise Fail(f'target moved {djump:.2f} m since grasp (>0.15): re-run S1-S4')
    P = np.array(pos, float) + delta
    json.dump(dict(delta_m=[round(float(v), 4) for v in delta],
                   grasp_centroid=C0.round(4).tolist(),
                   live_centroid=C1.round(4).tolist(),
                   live_n=live['n'], live_top=live['top'],
                   pick_shifted_m=[round(float(v), 4) for v in P]),
              open(f'{ctx["run"]}/reanchor.json', 'w'), indent=1)
    log(f're-anchor: target shift {(delta * 1000).round(1).tolist()} mm, '
        f'pick now {P.round(4).tolist()}')
    return P


def s6_request(node, ctx, a, pick):
    """DRY: validate goal chain vs live maps. LIVE: execute via lab runner (gated)."""
    P = reanchor_pick(node, ctx, a, pick['pos'])
    above, lift = P + np.array([0, 0, 0.05]), P + np.array([0, 0, 0.105])
    hand = live_hand()
    H = np.array(hand['p0'])
    legs = [(H, above), (above, P)]
    log(f'live hand ({H[0]:.3f}, {H[1]:.3f}, {H[2]:.3f}) -> above ({above[0]:.3f}, {above[1]:.3f}, {above[2]:.3f}) -> grasp')
    seg_min, seg_frac = leg_clearance(node, T['esdf_seg'], legs)
    prod_min, prod_frac = leg_clearance(node, T['esdf_prod'], legs)
    out = dict(hand=hand, above=above.tolist(), lift=lift.tolist(),
               seg_min_dist_m=seg_min, seg_known_frac=seg_frac,
               prod_min_dist_m=prod_min, prod_known_frac=prod_frac, live=bool(a.live))
    json.dump(out, open(f'{ctx["run"]}/goal.json', 'w'), indent=1)
    log(f'legs min dist: seg-map {seg_min:.3f} m (known {seg_frac:.0%}), '
        f'prod-map {prod_min:.3f} m (known {prod_frac:.0%})')
    if seg_frac < 0.3:
        # Not fatal: the carve cone above a tracked target is unknown BY DESIGN
        # (masked rays never integrate there), and the planner treats unknown as
        # free. The pick-cell carve (S5) is the hard guarantee; this is advisory.
        log(f'WARN path coverage low (known {seg_frac:.0%}); carve-cone shadow '
            f'is expected above the target')
    if seg_min < 0.0:
        raise Fail('path collides even in the carved map')
    if prod_min >= 0.0:
        log('note: prod map also clear (target absent there?)')
    else:
        log('expected: prod map (no carve) blocks the grasp cell')
    if not a.live:
        log(f'S6 DRY PASS. live cmd: ./run_loop.sh --seed {a.seed} --label {a.label} '
            f'--from 6 --run-dir {ctx["run"]} --live --estop "Name"')
        return out
    if not a.estop:
        raise Fail('--live needs --estop "Name"')
    go = input(f'LIVE grasp at ({P[0]:.3f}, {P[1]:.3f}, {P[2]:.3f}). {a.estop} at e-stop + teleop lane idle? type GO: ')
    if go.strip() != 'GO':
        raise Fail('live aborted by operator')
    s2_mask(node, ctx, a)  # target must still be tracked
    cmd = (f'cd /home/murilo/franka_ws/lab/claude_trial && GTX={P[0]:.4f} GTY={P[1]:.4f} GTZ={P[2]:.4f} '
           f'ESTOP="{a.estop}" printf "MOVE\\n" | ./run_grasp_drill.sh')
    log('executing on lab (streaming): ' + cmd[:120] + '...')
    p = subprocess.run(SSH + [cmd], capture_output=False, timeout=600)
    out['lab_rc'] = p.returncode
    json.dump(out, open(f'{ctx["run"]}/goal.json', 'w'), indent=1)
    if p.returncode != 0:
        raise Fail(f'lab runner exited {p.returncode}')
    log('S6 LIVE PASS')
    return out


STAGES = {1: ('seed', s1_seed), 2: ('mask', s2_mask), 3: ('cloud', s3_cloud),
          4: ('grasp', s4_grasp), 5: ('carve', s5_carve), 6: ('request', s6_request)}


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument('--seed', required=True, help='bbox_1000 x1,y1,x2,y2')
    ap.add_argument('--label', default='target')
    ap.add_argument('--run-dir', default='')
    ap.add_argument('--from', dest='frm', type=int, default=1)
    ap.add_argument('--to', type=int, default=6)
    ap.add_argument('--live', action='store_true')
    ap.add_argument('--estop', default='')
    ap.add_argument('--cuda', default='1')
    ap.add_argument('--min-cov', type=float, default=0.002)
    ap.add_argument('--max-cov', type=float, default=0.15)
    a = ap.parse_args()
    import rclpy
    here = os.path.dirname(os.path.abspath(__file__))
    rid = datetime.now(timezone.utc).strftime('%Y%m%d_%H%M%S')
    rund = a.run_dir or os.path.join(here, 'runs', f'{rid}_{a.label}')
    os.makedirs(rund, exist_ok=True)
    ctx = dict(run=rund, id=os.path.basename(rund))
    if a.frm > 1:
        for f in ('seed.json', 'cloud.json', 'pick.json'):
            p = os.path.join(rund, f)
            if os.path.exists(p):
                ctx[f[:-5]] = json.load(open(p))
    rclpy.init()
    node = rclpy.create_node('grasp_loop')
    results, rc = {}, 0
    try:
        data = {}
        if a.frm <= 1 <= a.to:
            data['seed'] = s1_seed(node, ctx, a)
        if a.frm <= 2 <= a.to:
            data['mask'] = s2_mask(node, ctx, a)
        if a.frm <= 3 <= a.to:
            data['cloud'] = s3_cloud(node, ctx, a)
        elif 'cloud' in ctx:
            data['cloud'] = ctx['cloud']
        if a.frm <= 4 <= a.to:
            if 'cloud' not in data:
                raise Fail('stage 4 needs stage 3 (or --run-dir with cloud.json)')
            data['pick'] = s4_grasp(node, ctx, a, data['cloud'])
        elif 'pick' in ctx:
            data['pick'] = ctx['pick']
        if a.frm <= 5 <= a.to:
            if 'cloud' not in data:
                raise Fail('stage 5 needs stage 3 (or --run-dir with cloud.json)')
            data['carve'] = s5_carve(node, ctx, a, data['cloud'])
        if a.frm <= 6 <= a.to:
            if 'pick' not in data:
                raise Fail('stage 6 needs stage 4 (or --run-dir with pick.json)')
            data['goal'] = s6_request(node, ctx, a, data['pick'])
        results = {k: 'PASS' for k in data}
    except Fail as e:
        log(f'STAGE FAIL: {e}')
        results['FAIL'] = str(e)
        rc = 2
    json.dump(results, open(f'{rund}/summary.json', 'w'), indent=1)
    log(f'done rc={rc}: {rund}')
    return rc


if __name__ == '__main__':
    sys.exit(main())
