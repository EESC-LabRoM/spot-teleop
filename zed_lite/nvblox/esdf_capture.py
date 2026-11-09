"""Capture the nvblox ESDF once, exactly as the MPC node asks for it (service /nvblox_node/get_esdf_and_gradient), on a LARGER box than the node's grid,
so that esdf_replay.py can later serve any of the node's requests without nvblox / Isaac / ZED (the 'map once, grasps in Newton' workflow).
   (in the nvblox-lite container, ROS 2 Humble)  python3 esdf_capture.py --out /ws/nvblox/out/<run>/esdf_full.npz [--center 0.45 0 0.45] [--size 2.4 2.4 1.8]
Saved: esdf (nx, ny, nz) float32 in nvblox's order (x slowest), origin (3) = min corner of voxel [0,0,0], voxel, frame, and the request. -1000 = unobserved, <= 0 occupied."""
import argparse, sys, time
import numpy as np
import rclpy
from geometry_msgs.msg import Point, Vector3
from nvblox_msgs.srv import EsdfAndGradients

ap = argparse.ArgumentParser()
ap.add_argument("--out", required=True)
ap.add_argument("--service", default="/nvblox_node/get_esdf_and_gradient")
ap.add_argument("--frame", default="fr3_link0")
ap.add_argument("--center", type=float, nargs=3, default=[0.45, 0.0, 0.45])
ap.add_argument("--size", type=float, nargs=3, default=[2.4, 2.4, 1.8])
a = ap.parse_args()
rclpy.init(); node = rclpy.create_node("esdf_capture")
cli = node.create_client(EsdfAndGradients, a.service)
if not cli.wait_for_service(timeout_sec=30.0): print("no service", a.service); sys.exit(2)
req = EsdfAndGradients.Request(); req.update_esdf = True; req.visualize_esdf = False; req.use_aabb = True; req.frame_id = a.frame
c, s = np.array(a.center), np.array(a.size); mn = c - s / 2
req.aabb_min_m = Point(x=float(mn[0]), y=float(mn[1]), z=float(mn[2])); req.aabb_size_m = Vector3(x=float(s[0]), y=float(s[1]), z=float(s[2]))
fut = cli.call_async(req); rclpy.spin_until_future_complete(node, fut, timeout_sec=60.0)
r = fut.result()
if r is None or not r.success: print("ESDF request failed"); sys.exit(3)
dims = [d.size for d in r.esdf_and_gradients.layout.dim]; E = np.array(r.esdf_and_gradients.data, np.float32).reshape(dims)
np.savez(a.out, esdf=E, origin=np.array([r.origin_m.x, r.origin_m.y, r.origin_m.z]), voxel=float(r.voxel_size_m), frame=a.frame,
         req_min=mn, req_size=s, captured=time.time())
print(f"ESDF captured: shape {E.shape} voxel {r.voxel_size_m:.4f} origin {r.origin_m.x:.3f} {r.origin_m.y:.3f} {r.origin_m.z:.3f} | "
      f"occupied {int(((E > -999) & (E <= 0)).sum())} unobserved {int((E <= -999).sum())} free {int((E > 0).sum())} -> {a.out}")
