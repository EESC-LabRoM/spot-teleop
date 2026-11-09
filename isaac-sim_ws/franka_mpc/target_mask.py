"""Target-object mask for nvblox's human/dynamic layer (stands in for the SAM2 tracker's mask in the paper's pipeline).
Per ZED depth frame it publishes a mono8 mask (255 = the target) with the SAME header as the depth frame, so nvblox_node in
`human_with_static_tsdf` mode (franka_zed_nvblox.launch.py mask:=true) pairs them: masked pixels go to the dynamic layer, the rest builds
the static TSDF that the ESDF comes from. The mask is the object's exact mesh (Isaac, objects_start.npz) rasterized with the live
rigid-body pose from /target_object/pose (franka_ros_arm.py --track-object), the camera pose from camera_<cam>.json and the intrinsics
from the depth camera_info; no occlusion test (a hand in front of the object is masked too, the robot self-filter removes it anyway).
An all-zero mask is published when the object is out of view (nvblox integrates nothing from a camera without masks).
Runs in the nvblox-lite image (ROS 2 Humble, python3, cv2):
   python3 target_mask.py --run /run --workcell-scene /fm/runs/workcell/scene.json --cams zed,zed2 [--dilate 3]
"""
import argparse, bisect, collections, json, os, sys, time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker

ap = argparse.ArgumentParser()
ap.add_argument("--run", required=True, help="run dir: camera_<cam>.json, objects_start.npz, object_ref.json (franka_ros_arm.py)")
ap.add_argument("--workcell-scene", required=True, help="scene.json with franka_base (world position of fr3_link0)")
ap.add_argument("--cams", default="zed,zed2")
ap.add_argument("--mode", choices=["human", "filter"], default="human", help="human: publish the mask for nvblox's human_with_static_tsdf layer (/<cam>/target_mask); filter: publish the robot-free depth with the target's pixels removed (/<cam>/depth_no_target) for a plain static nvblox")
ap.add_argument("--clear-box", action="store_true", help="human mode: also publish the target's clear box (Marker CUBE, fr3_link0) from the mask + depth, as the paper's sam2_tracker_node does (mask pixel extent at the median masked depth, 10-90th pct depth spread floored at 4 cm / capped at 0.5 m), for the MPC node's ESDF clear (+ target_clear_padding_m)")
ap.add_argument("--pose-topic", default="/target_object/pose")
ap.add_argument("--dilate", type=int, default=3, help="grow the mask by N pixels (a SAM2 mask is not pixel-exact either)")
ap.add_argument("--extra-dilate", default="", help="more variants, comma list of pixel counts: each one is published on /<cam>/target_mask_d<N> (compare maps with different growth)")
ap.add_argument("--snap-every", type=float, default=0.0, help="save inspection images every N s of sim time (0 = only the first frame)")
ap.add_argument("--occlusion-m", type=float, default=0.03, help="keep a masked pixel only if its measured depth lies within the target's depth range from this camera +- this margin (what a SAM2 mask of the VISIBLE object gives: an arm in front is not part of it); <0 = off")
ap.add_argument("--slop", type=float, default=0.25, help="max |depth stamp - pose stamp| (s)")
a = ap.parse_args(rclpy.utilities.remove_ros_args(sys.argv)[1:])


def stamp(h): return h.stamp.sec + h.stamp.nanosec * 1e-9


def quat_to_R(x, y, z, w):
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


class TargetMask(Node):
    def __init__(self):
        super().__init__("target_mask")
        ref = json.load(open(os.path.join(a.run, "object_ref.json"))); self.obj = ref["object"]
        root = np.array(json.load(open(a.workcell_scene))["franka_base"])
        tri = np.load(os.path.join(a.run, "objects_start.npz"))[self.obj].astype(np.float64) - root       # base frame, at the reference pose
        self.tri = tri; self.p_ref = np.array(ref["p_base"]); self.R_ref = quat_to_R(*ref["quat_xyzw"]); self.n_occl = 0
        self.cams = {}; self.extra = [int(x) for x in a.extra_dilate.split(",") if x]
        os.makedirs(os.path.join(a.run, "mask_inspect"), exist_ok=True)
        img_qos = QoSProfile(depth=2, history=HistoryPolicy.KEEP_LAST, reliability=ReliabilityPolicy.RELIABLE)
        for c in [c for c in a.cams.split(",") if c]:
            Tw = np.array(json.load(open(os.path.join(a.run, f"camera_{c}.json")))["T_world_camera_usd"])
            d = {"R": Tw[:3, :3], "t": Tw[:3, 3] - root, "K": None, "n": 0, "skipped": 0, "empty": 0, "frac": [], "dbg": False, "ms": []}
            d["pub"] = self.create_publisher(Image, f"/{c}/target_mask", img_qos); self.cams[c] = d
            d["xpub"] = {n: self.create_publisher(Image, f"/{c}/target_mask_d{n}", img_qos) for n in self.extra}
            d["rgb"] = None; d["last_snap"] = -1e9; d["snaps"] = 0
            self.create_subscription(Image, f"/{c}/zed_node/rgb/color/rect/image", lambda m, d=d: d.__setitem__("rgb", m), qos_profile_sensor_data)
            self.create_subscription(CameraInfo, f"/{c}/zed_node/depth/camera_info", lambda m, d=d: d.__setitem__("K", np.array(m.k).reshape(3, 3)), qos_profile_sensor_data)
            din = f"/{c}/zed_node/depth/depth_registered" if a.mode == "human" else ("/franka_seg/depth" if c == "zed" else f"/franka_seg_{c}/depth")   # filter: after the robot self-filter
            d["fpub"] = self.create_publisher(Image, f"/{c}/depth_no_target", img_qos) if a.mode == "filter" else None
            self.create_subscription(Image, din, lambda m, c=c: self.on_depth(c, m), img_qos)
        self.pt, self.pp = collections.deque(maxlen=4000), collections.deque(maxlen=4000)
        self.box_pub = self.create_publisher(Marker, "/target_object/clear_box", 10) if a.clear_box else None; self.n_box = 0
        self.create_subscription(PoseStamped, a.pose_topic, self.on_pose, 50)
        self.k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (2 * a.dilate + 1, 2 * a.dilate + 1)) if a.dilate > 0 else None
        self.create_timer(5.0, self.log_stats)
        self.get_logger().info(f"target mask for '{self.obj}' ({len(tri)} triangles), cameras {list(self.cams)}, dilate {a.dilate} px")

    def on_pose(self, m):
        t = stamp(m.header)
        if self.pt and t < self.pt[-1]: self.pt.clear(); self.pp.clear()          # sim restarted
        o = m.pose.orientation
        self.pt.append(t); self.pp.append((np.array([m.pose.position.x, m.pose.position.y, m.pose.position.z]), quat_to_R(o.x, o.y, o.z, o.w)))

    def on_depth(self, c, m):
        t0 = time.time(); d = self.cams[c]; d["n"] += 1
        # No mask, no frame: with a mask topic nvblox only integrates depth frames that have a mask, so while the pose of the target (or the
        # intrinsics) is not known we publish NOTHING (an empty mask there would let the target into the static map; found 10-15: the first
        # 21 frames per camera went out empty and the drill stayed in the static TSDF for good). An empty mask only means 'target not in view'.
        if d["K"] is None or not self.pt:
            d["skipped"] += 1; return
        ts = list(self.pt); i = bisect.bisect_left(ts, stamp(m.header)); cand = [k for k in (i - 1, i) if 0 <= k < len(ts)]
        k = min(cand, key=lambda k: abs(ts[k] - stamp(m.header)))
        if abs(ts[k] - stamp(m.header)) > a.slop:
            d["skipped"] += 1; return
        mask = np.zeros((m.height, m.width), np.uint8)
        p, R = self.pp[k]; V = (self.tri.reshape(-1, 3) - self.p_ref) @ (R @ self.R_ref.T).T + p             # live pose of every triangle corner
        Pc = (V - d["t"]) @ d["R"]                                                                           # USD camera frame: -Z forward, +Y up
        z = -Pc[:, 2]; front = z > 0.05; zz = np.where(front, z, 1.0)
        u = d["K"][0, 0] * Pc[:, 0] / zz + d["K"][0, 2]; v = d["K"][1, 2] - d["K"][1, 1] * Pc[:, 1] / zz
        uv = np.stack([u, v], 1).reshape(-1, 3, 2); ok = front.reshape(-1, 3).all(1)
        uv = np.clip(np.round(uv[ok]), -4000, 8000).astype(np.int32)
        if len(uv): cv2.fillPoly(mask, list(uv), 255)
        if a.occlusion_m >= 0 and m.encoding == "32FC1" and len(uv):   # visible surface only, like a SAM2 mask: per-pixel depth test against the mesh
            dep_m = np.frombuffer(m.data, np.float32).reshape(m.height, m.width)
            ys, xs = np.nonzero(mask); y0, y1, x0, x1 = ys.min(), ys.max() + 1, xs.min(), xs.max() + 1
            zb = np.full((y1 - y0, x1 - x0), np.inf, np.float32); zt = z.reshape(-1, 3)[ok].mean(1)
            for j in np.argsort(-zt):                                   # painter's order: far triangles first, near ones overwrite them
                cv2.fillPoly(zb, [uv[j] - np.array([x0, y0], np.int32)], float(zt[j]))
            sub, dsub = mask[y0:y1, x0:x1], dep_m[y0:y1, x0:x1]
            occl = (sub > 0) & np.isfinite(dsub) & (dsub > 0) & (np.abs(dsub - zb) > a.occlusion_m); sub[occl] = 0; self.n_occl += int(occl.sum())
        raw = mask
        grow = lambda n: cv2.dilate(raw, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (2 * n + 1, 2 * n + 1))) if n > 0 else raw
        mask = grow(a.dilate)
        for n, pb in d["xpub"].items():
            mx = grow(n); pb.publish(Image(header=m.header, height=m.height, width=m.width, encoding="mono8", is_bigendian=0, step=m.width, data=mx.tobytes()))
        d["frac"].append(float((mask > 0).mean())); d["empty"] += int(not mask.any())
        if a.mode == "filter":
            if m.encoding != "32FC1": d["skipped"] += 1; return
            dep = np.frombuffer(m.data, np.float32).reshape(m.height, m.width).copy(); dep[mask > 0] = 0.0          # 0 = no depth: nvblox integrates nothing there
            d["fpub"].publish(Image(header=m.header, height=m.height, width=m.width, encoding="32FC1", is_bigendian=0, step=m.width * 4, data=dep.tobytes()))
        else:
            d["pub"].publish(Image(header=m.header, height=m.height, width=m.width, encoding="mono8", is_bigendian=0, step=m.width, data=mask.tobytes()))
            if self.box_pub is not None and m.encoding == "32FC1": self.publish_box(d, mask, np.frombuffer(m.data, np.float32).reshape(m.height, m.width), m.header)
        ts = stamp(m.header)
        if mask.any() and (not d["dbg"] or (a.snap_every > 0 and ts - d["last_snap"] >= a.snap_every and d["snaps"] < 12)):
            d["dbg"] = d["rgb"] is not None; d["last_snap"] = ts; d["snaps"] += 1; tag = f"{c}_t{ts:07.2f}"
            dep = np.frombuffer(m.data, np.float32).reshape(m.height, m.width) if m.encoding == "32FC1" else None
            if dep is not None:
                g = np.clip(np.nan_to_num(dep, nan=0.0, posinf=0.0, neginf=0.0) / 3.0 * 255, 0, 255).astype(np.uint8); im = cv2.cvtColor(g, cv2.COLOR_GRAY2BGR)
                im[mask > 0] = (0.5 * im[mask > 0] + 0.5 * np.array([0, 0, 255])).astype(np.uint8)
                cv2.imwrite(os.path.join(a.run, "mask_inspect", f"{tag}_depth_overlay.jpg"), im)
            r = d["rgb"]
            if r is not None and r.height == m.height and r.width == m.width:
                ch = {"bgra8": 4, "rgba8": 4, "bgr8": 3, "rgb8": 3}.get(r.encoding)
                if ch:
                    im = np.frombuffer(r.data, np.uint8).reshape(r.height, r.width, ch)
                    im = np.ascontiguousarray(im[..., :3] if r.encoding in ("bgra8", "bgr8") else im[..., 2::-1][..., :3])
                    cv2.imwrite(os.path.join(a.run, "mask_inspect", f"{tag}_rgb.jpg"), im)
                    ov = im.copy(); cols = [(0, 0, 255), (0, 200, 255), (255, 120, 0), (0, 200, 0)]
                    for kk, n in enumerate([a.dilate] + self.extra):
                        cn, _ = cv2.findContours((raw if n == 0 else grow(n)), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE); cv2.drawContours(ov, cn, -1, cols[kk % 4], 2)
                    cv2.imwrite(os.path.join(a.run, "mask_inspect", f"{tag}_rgb_contours.jpg"), ov)
            cv2.imwrite(os.path.join(a.run, "mask_inspect", f"{tag}_mask.png"), mask)
        d["ms"].append((time.time() - t0) * 1000)

    def publish_box(self, d, mask, depth, header):
        """sam2_tracker_node._measure_and_publish_clear_box, same rules: optical-frame box from the mask's pixel extent at the median masked depth
        and the 10-90th percentile depth spread (floored 4 cm, capped 0.5 m), re-enclosed as an AABB in the base frame (h' = |R| h)."""
        ys, xs = np.nonzero(mask > 0)
        if xs.size < 10: return
        dv = depth[mask > 0]; dv = dv[np.isfinite(dv) & (dv > 0.0)]
        if dv.size < 5: return
        z_lo, z_mid, z_hi = (float(v) for v in np.percentile(dv, [10.0, 50.0, 90.0]))
        fx, fy, cx, cy = d["K"][0, 0], d["K"][1, 1], d["K"][0, 2], d["K"][1, 2]
        u_lo, u_hi, v_lo, v_hi = float(xs.min()), float(xs.max()), float(ys.min()), float(ys.max())
        half = np.array([0.5 * (u_hi - u_lo + 1) * z_mid / fx, 0.5 * (v_hi - v_lo + 1) * z_mid / fy, 0.5 * float(np.clip(z_hi - z_lo, 0.04, 0.5))])
        ctr = np.array([(0.5 * (u_lo + u_hi) - cx) * z_mid / fx, (0.5 * (v_lo + v_hi) - cy) * z_mid / fy, z_mid])
        R = d["R"] @ np.diag([1.0, -1.0, -1.0])                # optical (x right, y down, z forward) -> USD camera -> base
        cw, hw = R @ ctr + d["t"], np.abs(R) @ half
        mk = Marker(); mk.header.frame_id = "fr3_link0"; mk.header.stamp = header.stamp; mk.ns, mk.id, mk.type, mk.action = "target_clear_box", 0, Marker.CUBE, Marker.ADD
        mk.pose.position.x, mk.pose.position.y, mk.pose.position.z = (float(v) for v in cw); mk.pose.orientation.w = 1.0
        mk.scale.x, mk.scale.y, mk.scale.z = (float(max(2 * v, 1e-3)) for v in hw); self.box_pub.publish(mk); self.n_box += 1
        self.last_box = (cw.round(3).tolist(), (2 * hw).round(3).tolist())

    def log_stats(self):
        self.get_logger().info("stats " + json.dumps({c: {"frames": d["n"], "skipped_no_pose": d["skipped"], "empty": d["empty"], "have_K": d["K"] is not None,
                               "mask_frac_median": round(float(np.median(d["frac"])), 5) if d["frac"] else None,
                               "ms_median": round(float(np.median(d["ms"])), 1) if d["ms"] else None} for c, d in self.cams.items()}) + f" poses {len(self.pt)} occluded px dropped {self.n_occl}" + (f" | clear boxes {self.n_box}, last centre/size {getattr(self, 'last_box', None)}" if self.box_pub is not None else ""))
        for d in self.cams.values(): d["frac"], d["ms"] = [], []


rclpy.init(); node = TargetMask()
try: rclpy.spin(node)
except KeyboardInterrupt: pass
