#!/usr/bin/env python3
"""Live FR3 self-segmentation for the ZED cameras (same model as the EasyHeC calibration: the URDF visual meshes posed by the robot's
joints and projected with the calibrated camera extrinsics), so nvblox does not map the arm as an obstacle.

Per camera and per depth frame:
  - the joint state at the image stamp is interpolated from a /joint_states history (relayed from the robot, fr3_state_relay.py), and
    the link poses come from forward kinematics over the URDF joint chain (fr3.urdf written by the relay);
  - each link's convex hull (fr3_link_hulls.npz, exported from EasyHeC's FR3 meshes) is projected with the depth camera_info and the
    static camera -> fr3_link0 transform (real_extrinsics_tf.py + the ZED wrapper's own static chain, from /tf_static);
  - the silhouette is grown by --margin metres (in pixels at the link's nearest depth) to cover calibration and timing error;
  - the object to manipulate is removed as well: the latest /<cam>/segmentation_mask from the Spot branch's sam2_tracker_node
    (seeded by the VLM point), if younger than --target-max-age, is OR-ed into the mask;
  - publishes /<cam>/arm_filter/mask (mono8, 255 = robot or target) and /<cam>/arm_filter/depth (+ camera_info): the depth with the robot pixels set
    to NaN (invalid), which nvblox integrates in place of the raw depth.
The dynamic /tf is not used: a Python TF listener fell ~2 s behind the 100 Hz robot TF. If no joint state is close enough to the image
stamp (--max-skew) the frame is dropped, never published unmasked.
  python3 fr3_arm_mask_node.py [--cameras zed1,zed2] [--margin 0.03] [--hulls fr3_link_hulls.npz] [--urdf fr3.urdf]
"""
import argparse
import bisect
import collections
import os
import threading
import time
import xml.etree.ElementTree as ET

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image, JointState
from tf2_msgs.msg import TFMessage
from tf2_ros import Buffer

BASE = "fr3_link0"


def rpy_to_R(r, p, y):
    cr, sr, cp, sp, cy, sy = np.cos(r), np.sin(r), np.cos(p), np.sin(p), np.cos(y), np.sin(y)
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                     [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                     [-sp, cp * sr, cp * cr]])


def axis_angle(ax, a):
    x, y, z = ax; c, s, C = np.cos(a), np.sin(a), 1 - np.cos(a)
    return np.array([[c + x * x * C, x * y * C - z * s, x * z * C + y * s],
                     [y * x * C + z * s, c + y * y * C, y * z * C - x * s],
                     [z * x * C - y * s, z * y * C + x * s, c + z * z * C]])


def hom(R, t): T = np.eye(4); T[:3, :3] = R; T[:3, 3] = t; return T


class Chain:
    """forward kinematics from the URDF joint list (revolute / prismatic / fixed, mimic), base frame fr3_link0."""
    def __init__(self, path):
        root = ET.parse(path).getroot(); self.joints = {}
        for j in root.findall("joint"):
            o = j.find("origin"); ax = j.find("axis"); mm = j.find("mimic")
            xyz = [float(v) for v in (o.get("xyz", "0 0 0") if o is not None else "0 0 0").split()]
            rpy = [float(v) for v in (o.get("rpy", "0 0 0") if o is not None else "0 0 0").split()]
            self.joints[j.find("child").get("link")] = dict(
                name=j.get("name"), type=j.get("type"), parent=j.find("parent").get("link"), T0=hom(rpy_to_R(*rpy), xyz),
                axis=np.array([float(v) for v in (ax.get("xyz") if ax is not None else "1 0 0").split()]),
                mimic=(mm.get("joint"), float(mm.get("multiplier", 1)), float(mm.get("offset", 0))) if mm is not None else None)

    def poses(self, q, links):
        cache = {BASE: np.eye(4)}

        def pose(link):
            if link in cache: return cache[link]
            j = self.joints[link]; T = pose(j["parent"]) @ j["T0"]
            if j["type"] != "fixed":
                v = q[j["mimic"][0]] * j["mimic"][1] + j["mimic"][2] if j["mimic"] else q[j["name"]]
                T = T @ (hom(axis_angle(j["axis"], v), [0, 0, 0]) if j["type"] in ("revolute", "continuous") else hom(np.eye(3), j["axis"] * v))
            cache[link] = T; return T
        return {link: pose(link) for link in links}


class ArmMask(Node):
    def __init__(self, a):
        super().__init__("fr3_arm_mask")
        self.a = a
        h = np.load(a.hulls)
        self.links = {str(n): h[str(n)].astype(np.float64) for n in h["names"]}
        self.chain = Chain(a.urdf)
        self.hist, self.lock = collections.deque(maxlen=400), threading.Lock()   # (stamp s, {joint: pos}), ~4 s at 100 Hz
        self.static = Buffer()
        latched = QoSProfile(depth=100, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(TFMessage, "/tf_static", self.on_static, latched)
        self.create_subscription(JointState, "/joint_states", self.on_js, 50)
        self.info, self.pubs, self.stats, self.skew, self.target = {}, {}, {}, 0.0, {}
        for cam in a.cameras:
            self.pubs[cam] = (self.create_publisher(Image, f"/{cam}/arm_filter/depth", qos_profile_sensor_data),
                              self.create_publisher(CameraInfo, f"/{cam}/arm_filter/depth/camera_info", qos_profile_sensor_data),
                              self.create_publisher(Image, f"/{cam}/arm_filter/mask", qos_profile_sensor_data))
            self.stats[cam] = dict(ok=0, drop=0, frac=0.0, ms=0.0, target=0.0)
            self.create_subscription(CameraInfo, f"/{cam}/zed_node/depth/camera_info", lambda m, c=cam: self.info.__setitem__(c, m),
                                     qos_profile_sensor_data)
            self.create_subscription(Image, f"/{cam}/zed_node/depth/depth_registered", lambda m, c=cam: self.on_depth(c, m),
                                     qos_profile_sensor_data)
            if a.target_mask_topic:                                # the object to manipulate (sam2_tracker_node), kept out of the map too
                self.create_subscription(Image, a.target_mask_topic.replace("{cam}", cam), lambda m, c=cam: self.on_target(c, m), 10)
        self.create_timer(10.0, self.report)
        self.get_logger().info(f"arm mask on {a.cameras}: {len(self.links)} links, margin {a.margin * 100:.0f} cm")

    def on_target(self, cam, m):
        if m.encoding not in ("mono8", "8UC1"): return
        k = np.frombuffer(m.data, np.uint8).reshape(m.height, m.width)
        self.target[cam] = (time.monotonic(), k.copy() if k.any() else None)

    def on_static(self, m):
        for t in m.transforms: self.static.set_transform_static(t, "tf_static")

    def on_js(self, m):
        t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        with self.lock: self.hist.append((t, dict(zip(m.name, m.position))))

    def q_at(self, t):
        """joint positions at time t: linear interpolation in the history; past the newest sample, the newest if within --max-skew."""
        with self.lock: H = list(self.hist)
        if not H: raise LookupError("no /joint_states")
        ts = [h[0] for h in H]; i = bisect.bisect_left(ts, t)
        if i == 0:
            self.skew = t - ts[0]
            if ts[0] - t > self.a.max_skew: raise LookupError(f"image {ts[0] - t:.3f} s older than the joint history")
            return H[0][1]
        if i == len(H):
            self.skew = t - ts[-1]
            if t - ts[-1] > self.a.max_skew: raise LookupError(f"image {t - ts[-1]:.3f} s newer than the last joint state")
            return H[-1][1]
        (t0, q0), (t1, q1) = H[i - 1], H[i]; w = (t - t0) / max(t1 - t0, 1e-6); self.skew = 0.0
        return {k: q0[k] + w * (q1[k] - q0[k]) for k in q0 if k in q1}

    def render(self, frame, t, info):
        K = np.array(info.k).reshape(3, 3); W, H = info.width, info.height
        tr = self.static.lookup_transform(frame, BASE, Time()).transform              # camera <- base, static
        x, y, z, w = tr.rotation.x, tr.rotation.y, tr.rotation.z, tr.rotation.w
        T_cb = hom(np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                             [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                             [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]]),
                   [tr.translation.x, tr.translation.y, tr.translation.z])
        poses = self.chain.poses(self.q_at(t), self.links)
        mask = np.zeros((H, W), np.uint8)
        for name, V in self.links.items():
            T = T_cb @ poses[name]; P = V @ T[:3, :3].T + T[:3, 3]
            P = P[P[:, 2] > 0.05]                                 # in front of the camera
            if len(P) < 3: continue
            uv = (P[:, :2] / P[:, 2:3]) @ K[:2, :2].T + K[:2, 2]
            hull = cv2.convexHull(np.round(uv).astype(np.int32))
            cv2.fillConvexPoly(mask, hull, 255)
            r = int(np.ceil(K[0, 0] * self.a.margin / max(float(P[:, 2].min()), 0.1)))   # margin in px at the link's nearest depth
            if r > 0: cv2.polylines(mask, [hull], True, 255, thickness=2 * r + 1)
        return mask

    def on_depth(self, cam, m):
        info = self.info.get(cam)
        if info is None or m.encoding != "32FC1": return
        t0 = time.monotonic(); st = self.stats[cam]
        try:
            mask = self.render(m.header.frame_id, m.header.stamp.sec + m.header.stamp.nanosec * 1e-9, info)
        except Exception as e:                                    # no robot state / extrinsics for this frame: drop, never pass it unmasked
            st["drop"] += 1
            if st["drop"] % 20 == 1: self.get_logger().warn(f"{cam}: frame dropped ({type(e).__name__}: {str(e)[:160]})")
            return
        d = np.frombuffer(m.data, np.float32).reshape(m.height, m.width).copy()
        if mask.shape != d.shape: mask = cv2.resize(mask, (d.shape[1], d.shape[0]), interpolation=cv2.INTER_NEAREST)
        tgt = self.target.get(cam)
        st["target"] = 0.0
        if tgt is not None and tgt[1] is not None and time.monotonic() - tgt[0] < self.a.target_max_age:
            k = tgt[1] if tgt[1].shape == mask.shape else cv2.resize(tgt[1], (mask.shape[1], mask.shape[0]), interpolation=cv2.INTER_NEAREST)
            st["target"] = float((k > 0).mean()); mask = np.maximum(mask, np.where(k > 0, 255, 0).astype(np.uint8))
        d[mask > 0] = np.nan
        out = Image(header=m.header, height=m.height, width=m.width, encoding="32FC1", is_bigendian=m.is_bigendian, step=m.width * 4,
                    data=d.tobytes())
        ci = CameraInfo(); ci.header = m.header; ci.height, ci.width = info.height, info.width
        ci.distortion_model, ci.d, ci.k, ci.r, ci.p = info.distortion_model, info.d, info.k, info.r, info.p
        mk = Image(header=m.header, height=mask.shape[0], width=mask.shape[1], encoding="mono8", step=mask.shape[1], data=mask.tobytes())
        pd, pi, pm = self.pubs[cam]; pd.publish(out); pi.publish(ci); pm.publish(mk)
        st["ok"] += 1; st["frac"] = float((mask > 0).mean()); st["ms"] = 1000 * (time.monotonic() - t0)

    def report(self):
        s = ", ".join(f"{c}: {v['ok']} ok / {v['drop']} dropped, masked {v['frac'] * 100:.0f}% of image (target {v['target'] * 100:.1f}%), {v['ms']:.0f} ms"
                      for c, v in self.stats.items())
        self.get_logger().info(f"{s}; image-vs-joint-state skew {self.skew * 1000:+.0f} ms")
        for v in self.stats.values(): v["ok"] = v["drop"] = 0


def main():
    here = os.path.dirname(os.path.abspath(__file__))
    ap = argparse.ArgumentParser()
    ap.add_argument("--cameras", default="zed1,zed2")
    ap.add_argument("--margin", type=float, default=0.03, help="m added around the silhouette (calibration + timing error)")
    ap.add_argument("--hulls", default=os.path.join(here, "fr3_link_hulls.npz"))
    ap.add_argument("--urdf", default=os.path.join(here, "fr3.urdf"))
    ap.add_argument("--max-skew", type=float, default=0.1, help="s: largest image-vs-joint-state gap bridged with the nearest state")
    ap.add_argument("--target-mask-topic", default="/{cam}/segmentation_mask",
                    help="object mask per camera (sam2_tracker_node, as in the Spot branch), also removed from the depth; '' = off")
    ap.add_argument("--target-max-age", type=float, default=1.0, help="s: an older object mask is ignored")
    a, ros = ap.parse_known_args()
    a.cameras = [c.strip() for c in a.cameras.split(",") if c.strip()]
    rclpy.init(args=ros); n = ArmMask(a)
    try: rclpy.spin(n)
    except KeyboardInterrupt: pass


if __name__ == "__main__":
    main()
