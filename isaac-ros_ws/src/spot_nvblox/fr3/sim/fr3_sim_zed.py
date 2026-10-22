#!/usr/bin/env python3
"""Simulated ZED 2i pair for the Newton FR3 sim (Murilo's hub-curobo/newton_sim): renders depth + a shaded colour image of the bench
scene (table top + the plan's fixed obstacles) and of the simulated arm, from the REAL calibrated camera poses and intrinsics
(calib_live1016 + the live wrappers' camera_info), and publishes them with the ZED wrapper's topic names, so the perception
stack (fr3_arm_mask_node.py, nvblox via fr3_nvblox.launch.py) runs unchanged against a scene whose geometry is known exactly.

  - arm: each link's convex hull (fr3_link_hulls.npz, EasyHeC's FR3 meshes) reduced to --hull-points points, posed by FK of the
    sim's /joint_states (URDF chain), ray-cast on the GPU (torch) together with the scene boxes;
  - realism knobs: depth noise sigma = --noise * z^2 (ZED-like), and a calibration error: the images are rendered from a camera
    pose perturbed by --calib-err-mm / --calib-err-deg while the TF says the calibrated pose (as on the real bench, ~5 mm);
  - each frame is stamped with the joint state it was rendered from.
Ground truth for the checker: scene_boxes() below (fr3_link0, metres).
  python3 fr3_sim_zed.py [--rate 5] [--noise 0.004] [--calib-err-mm 5] [--calib-err-deg 0.3]
"""
import argparse
import json
import os
import sys

import numpy as np
import rclpy
import torch
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from scipy.spatial import ConvexHull
from sensor_msgs.msg import CameraInfo, Image, JointState
from tf2_ros import StaticTransformBroadcaster

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))
from fr3_arm_mask_node import Chain  # noqa: E402

CAMS = {"zed1": "zed1_extrinsic_1016.json", "zed2": "zed2_extrinsic_1016.json"}


def scene_boxes():
    """(name, centre xyz, half extents xyz, BGR) in fr3_link0. The table matches the Newton sim's (top at z = 0). Four obstacles
    around a target spot at (0.55, 0) as in JOYSTICK_EXPERIMENT_PLAN section 3: the straight path from the ready pose is blocked."""
    return [
        ("table", (0.30, 0.00, -0.05), (0.80, 0.80, 0.05), (40, 40, 40)),
        ("front_wall", (0.42, 0.00, 0.11), (0.03, 0.10, 0.11), (60, 60, 200)),     # 6 x 20 x 22 cm, between the arm and the target
        ("left_box", (0.56, 0.18, 0.075), (0.05, 0.03, 0.075), (60, 180, 60)),    # 10 x 6 x 15 cm
        ("right_box", (0.56, -0.18, 0.075), (0.05, 0.03, 0.075), (200, 120, 40)),
        ("back_box", (0.69, 0.00, 0.06), (0.03, 0.10, 0.06), (40, 200, 220)),     # 6 x 20 x 12 cm
    ]


def rot(axis, deg):
    a = np.radians(deg); axis = np.asarray(axis, float) / np.linalg.norm(axis); x, y, z = axis; c, s, C = np.cos(a), np.sin(a), 1 - np.cos(a)
    return np.array([[c + x * x * C, x * y * C - z * s, x * z * C + y * s], [y * x * C + z * s, c + y * y * C, y * z * C - x * s],
                     [z * x * C - y * s, z * y * C + x * s, c + z * z * C]])


def mat2quat(R):
    t = np.trace(R)
    if t > 0:
        s = 2 * np.sqrt(t + 1); return np.array([(R[2, 1] - R[1, 2]) / s, (R[0, 2] - R[2, 0]) / s, (R[1, 0] - R[0, 1]) / s, s / 4])
    i = int(np.argmax(np.diag(R))); j, k = (i + 1) % 3, (i + 2) % 3
    s = 2 * np.sqrt(1 + R[i, i] - R[j, j] - R[k, k]); q = np.zeros(4); q[i] = s / 4
    q[j] = (R[j, i] + R[i, j]) / s; q[k] = (R[k, i] + R[i, k]) / s; q[3] = (R[k, j] - R[j, k]) / s; return q


class SimZed(Node):
    def __init__(self, a):
        super().__init__("fr3_sim_zed")
        self.a, self.dev = a, torch.device("cuda")
        rng = np.random.default_rng(a.seed)
        info = json.load(open(os.path.join(HERE, "zed_cam_info.json")))
        self.cams, br, statics = {}, StaticTransformBroadcaster(self), []
        for cam, f in CAMS.items():
            T = np.array(json.load(open(os.path.join(HERE, f)))["T_base_cam"])            # calibrated optical frame in fr3_link0 (TF)
            Tt = T.copy()                                                                   # "true" pose the images are rendered from
            if a.calib_err_mm > 0 or a.calib_err_deg > 0:
                v = rng.normal(size=3); Tt[:3, 3] += a.calib_err_mm * 1e-3 * v / np.linalg.norm(v)
                Tt[:3, :3] = rot(rng.normal(size=3), a.calib_err_deg) @ Tt[:3, :3]
            ci = info[cam]; K = np.array(ci["k"]).reshape(3, 3); W, H = ci["width"], ci["height"]
            u, v = np.meshgrid(np.arange(W) + 0.5, np.arange(H) + 0.5)
            rays_c = np.stack([(u - K[0, 2]) / K[0, 0], (v - K[1, 2]) / K[1, 1], np.ones_like(u)], -1).reshape(-1, 3)   # z = 1
            dirs = rays_c @ Tt[:3, :3].T                                                    # base frame, |dir_z_cam| = 1 -> t = depth
            self.cams[cam] = dict(W=W, H=H, info=ci, o=torch.tensor(Tt[:3, 3], dtype=torch.float32, device=self.dev),
                                  d=torch.tensor(dirs, dtype=torch.float32, device=self.dev),
                                  pubs=(self.create_publisher(Image, f"/{cam}/zed_node/depth/depth_registered", qos_profile_sensor_data),
                                        self.create_publisher(CameraInfo, f"/{cam}/zed_node/depth/camera_info", qos_profile_sensor_data),
                                        self.create_publisher(Image, f"/{cam}/zed_node/rgb/image_rect_color", qos_profile_sensor_data),
                                        self.create_publisher(CameraInfo, f"/{cam}/zed_node/rgb/camera_info", qos_profile_sensor_data)))
            ts = TransformStamped(); ts.header.frame_id = "fr3_link0"; ts.child_frame_id = f"{cam}_left_camera_optical_frame"
            ts.transform.translation.x, ts.transform.translation.y, ts.transform.translation.z = (float(x) for x in T[:3, 3])
            q = mat2quat(T[:3, :3]); ts.transform.rotation.x, ts.transform.rotation.y, ts.transform.rotation.z, ts.transform.rotation.w = (float(x) for x in q)
            statics.append(ts)
            err = np.linalg.norm(Tt[:3, 3] - T[:3, 3]) * 1000
            self.get_logger().info(f"{cam}: {W}x{H} fx {K[0, 0]:.1f}, rendered {err:.1f} mm / {a.calib_err_deg:.2f} deg off the published TF")
        br.sendTransform(statics)
        boxes = scene_boxes()
        self.box_c = torch.tensor([b[1] for b in boxes], dtype=torch.float32, device=self.dev)
        self.box_h = torch.tensor([b[2] for b in boxes], dtype=torch.float32, device=self.dev)
        self.colors = np.array([b[3] for b in boxes] + [(225, 225, 225)], np.uint8)          # last = arm
        h = np.load(os.path.join(os.path.dirname(HERE), "fr3_link_hulls.npz"))
        self.planes = {}
        for n in h["names"]:
            V = h[str(n)].astype(np.float64)
            if len(V) > a.hull_points:                                                       # farthest-point subsample, then re-hull
                idx = [0]; dmin = np.linalg.norm(V - V[0], axis=1)
                for _ in range(a.hull_points - 1): i = int(dmin.argmax()); idx.append(i); dmin = np.minimum(dmin, np.linalg.norm(V - V[i], axis=1))
                V = V[idx]
            self.planes[str(n)] = ConvexHull(V).equations                                    # n.x + d <= 0 inside, link frame
        self.chain = Chain(os.path.join(os.path.dirname(HERE), "fr3.urdf"))
        self.js = None
        self.create_subscription(JointState, "/joint_states", lambda m: setattr(self, "js", m), 20)
        self.create_timer(1.0 / a.rate, self.tick)
        self.n = 0
        self.get_logger().info(f"sim ZEDs up: {len(boxes)} scene boxes, {sum(len(p) for p in self.planes.values())} arm hull planes, "
                               f"{a.rate:.0f} Hz, noise {a.noise} z^2")

    def cast(self, o, d, poses):
        """nearest hit distance (= depth, rays have unit z in the camera) and surface id (box index, or len(boxes) = arm)."""
        inv = 1.0 / torch.where(d.abs() < 1e-9, torch.full_like(d, 1e-9), d)
        lo = ((self.box_c - self.box_h)[None] - o) * inv[:, None]; hi = ((self.box_c + self.box_h)[None] - o) * inv[:, None]
        tn = torch.minimum(lo, hi).amax(-1); tf = torch.maximum(lo, hi).amin(-1)
        tb = torch.where((tn <= tf) & (tf > 0), tn.clamp(min=0), torch.full_like(tn, float("inf")))
        best, sid = tb.min(-1)
        for name, P in self.planes.items():
            T = poses[name]; n = P[:, :3] @ T[:3, :3].T; c = P[:, 3] - n @ T[:3, 3]           # planes in the base frame
            n = torch.tensor(n, dtype=torch.float32, device=self.dev); c = torch.tensor(c, dtype=torch.float32, device=self.dev)
            num = -(o @ n.T + c)                                                              # (P,)
            den = d @ n.T                                                                     # (N, P)
            t = num[None] / torch.where(den.abs() < 1e-9, torch.full_like(den, 1e-9), den)
            t_in = torch.where(den < 0, t, torch.full_like(t, -float("inf"))).amax(-1)
            t_out = torch.where(den > 0, t, torch.full_like(t, float("inf"))).amin(-1)
            outside_parallel = ((den.abs() < 1e-9) & (num[None] < 0)).any(-1)
            hit = (t_in <= t_out) & (t_out > 0) & ~outside_parallel
            th = torch.where(hit, t_in.clamp(min=0), torch.full_like(t_in, float("inf")))
            closer = th < best; best = torch.where(closer, th, best); sid = torch.where(closer, torch.full_like(sid, len(self.colors) - 1), sid)
        return best, sid

    def tick(self):
        m = self.js
        if m is None: return
        q = dict(zip(m.name, m.position))
        if not all(f"fr3_joint{i}" in q for i in range(1, 8)): return
        q.setdefault("fr3_finger_joint1", 0.04)
        poses = self.chain.poses(q, self.planes)
        for cam, c in self.cams.items():
            with torch.no_grad():
                t, sid = self.cast(c["o"], c["d"], poses)
                z = t.clone()
                if self.a.noise > 0: z = z + torch.randn_like(z) * self.a.noise * z * z
                z[~torch.isfinite(t) | (t > self.a.max_range)] = float("nan")
            depth = z.view(c["H"], c["W"]).cpu().numpy().astype(np.float32)
            sid = sid.view(c["H"], c["W"]).cpu().numpy(); miss = ~np.isfinite(depth)
            rgb = self.colors[np.clip(sid, 0, len(self.colors) - 1)].copy()
            shade = np.clip(1.15 - 0.35 * np.nan_to_num(depth, nan=2.0), 0.4, 1.0)[..., None]
            rgb = (rgb * shade).astype(np.uint8); rgb[miss] = (90, 80, 70)
            bgra = np.concatenate([rgb, np.full(rgb.shape[:2] + (1,), 255, np.uint8)], -1)
            hdr = m.header; hdr.frame_id = f"{cam}_left_camera_optical_frame"
            ci = c["info"]
            info = CameraInfo(); info.header = hdr; info.width, info.height = c["W"], c["H"]
            info.k, info.p, info.d, info.distortion_model = ci["k"], ci["p"], ci["d"], ci["distortion_model"]
            info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
            pd, pdi, pc, pci = c["pubs"]
            pd.publish(Image(header=hdr, height=c["H"], width=c["W"], encoding="32FC1", step=c["W"] * 4, data=depth.tobytes()))
            pdi.publish(info)
            pc.publish(Image(header=hdr, height=c["H"], width=c["W"], encoding="bgra8", step=c["W"] * 4, data=bgra.tobytes()))
            pci.publish(info)
        self.n += 1
        if self.n % (int(self.a.rate) * 10) == 0: self.get_logger().info(f"{self.n} frames rendered")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--rate", type=float, default=5.0)
    ap.add_argument("--noise", type=float, default=0.004, help="depth noise sigma = noise * z^2 (m)")
    ap.add_argument("--calib-err-mm", type=float, default=5.0); ap.add_argument("--calib-err-deg", type=float, default=0.3)
    ap.add_argument("--hull-points", type=int, default=96); ap.add_argument("--max-range", type=float, default=3.0)
    ap.add_argument("--seed", type=int, default=0)
    a, ros = ap.parse_known_args()
    rclpy.init(args=ros); n = SimZed(a)
    try: rclpy.spin(n)
    except KeyboardInterrupt: pass


if __name__ == "__main__":
    main()
