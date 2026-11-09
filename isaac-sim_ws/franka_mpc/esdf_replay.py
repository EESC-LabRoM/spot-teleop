"""Serve nvblox's /nvblox_node/get_esdf_and_gradient from a SAVED map (esdf_capture.py's esdf_full.npz): the MPC node gets the same kind of answer as from nvblox,
without nvblox, ZED or Isaac. The request's AABB is cropped on nvblox's own voxel lattice (origin = floor(min / voxel) * voxel, n = floor(max / voxel) - floor(min / voxel) + 1: every voxel the box touches),
so the node's shape check (e.g. 81 x 81 x 61 at 2 cm for 1.6 x 1.6 x 1.2 m) passes. Outside the saved box: -1000 (unobserved) and a warning.
   (Isaac image python + Jazzy rclpy + nvblox_msgs_jazzy)  python3 esdf_replay.py --npz <esdf_full.npz> [--service /nvblox_node/get_esdf_and_gradient] --ros-args -p use_sim_time:=true"""
import argparse, sys
import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import MultiArrayDimension
from geometry_msgs.msg import Point
from nvblox_msgs.srv import EsdfAndGradients

ap = argparse.ArgumentParser(); ap.add_argument("--npz", required=True); ap.add_argument("--service", default="/nvblox_node/get_esdf_and_gradient")
ap.add_argument("--gate-file", default="", help="answer success=False until this file exists (the Isaac runs warmed the MPC up BEFORE the first map arrived; with the map present the node's warm-up steps take 7-12 s each)")
a, ros_args = ap.parse_known_args(rclpy.utilities.remove_ros_args(sys.argv)[1:])
Z = np.load(a.npz); E, O, V = Z["esdf"], Z["origin"].astype(np.float64), float(Z["voxel"])

class Replay(Node):
    def __init__(self):
        super().__init__("esdf_replay"); self.n = 0
        self.create_service(EsdfAndGradients, a.service, self.cb)
        self.get_logger().info(f"serving {a.service} from {a.npz}: shape {E.shape} voxel {V} origin {O.round(3).tolist()}")
    def cb(self, req, res):
        import os
        if a.gate_file and not os.path.exists(a.gate_file): res.success = False; return res
        mn = np.array([req.aabb_min_m.x, req.aabb_min_m.y, req.aabb_min_m.z]); sz = np.array([req.aabb_size_m.x, req.aabb_size_m.y, req.aabb_size_m.z])
        if not req.use_aabb: mn, sz = O, np.array(E.shape) * V
        i_lo = np.floor(mn / V + 1e-3).astype(int); i_hi = np.floor((mn + sz) / V + 1e-3).astype(int); org = i_lo * V; n = i_hi - i_lo + 1   # every voxel the box touches, boundaries included
        i0 = np.round((org - O) / V).astype(int); out = np.full(tuple(n), -1000.0, np.float32)
        lo, hi = np.maximum(i0, 0), np.minimum(i0 + n, E.shape)
        if np.all(hi > lo): out[tuple(slice(l - i, h - i) for l, h, i in zip(lo, hi, i0))] = E[tuple(slice(l, h) for l, h in zip(lo, hi))]
        else: self.get_logger().warn("request entirely outside the saved box")
        if np.any(i0 < 0) or np.any(i0 + n > E.shape): self.get_logger().warn(f"request partly outside the saved box (-1000 there): min {mn.round(3).tolist()} size {sz.round(3).tolist()}", throttle_duration_sec=10.0)
        res.origin_m = Point(x=float(org[0]), y=float(org[1]), z=float(org[2])); res.voxel_size_m = V; res.success = True
        m = res.esdf_and_gradients
        m.layout.dim = [MultiArrayDimension(label=l, size=int(s_), stride=int(st)) for l, s_, st in zip("xyz", n, (n[0] * n[1] * n[2], n[1] * n[2], n[2]))]
        m.data = out.reshape(-1).tolist(); self.n += 1
        if self.n <= 3 or self.n % 50 == 0: self.get_logger().info(f"request {self.n}: shape {tuple(n)} origin {org.round(3).tolist()}")
        return res

rclpy.init(args=ros_args); rclpy.spin(Replay())
