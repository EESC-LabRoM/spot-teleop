"""Check the nvblox map of the Franka workcell against the scene we built (franka_scene.py scene.json).
Asks /nvblox_node/get_esdf_and_gradient (the service cuRobo uses) for the table region in the robot base frame, then:
the table top must come out as a surface at z ~ 0 (the base stands on it) and the objects as raised blobs where
scene.json puts them. Also checks the TF chain base -> left optical frame against Isaac's camera pose.
Writes <out>/esdf.npz, <out>/map_check.json, <out>/map_check.png (plotting needs matplotlib on the host: map_plot.py).
    python3 map_check.py --workcell /workcell --out /ws/nvblox/out
"""
import argparse, json, os, sys, time
import numpy as np
import rclpy
from geometry_msgs.msg import Point, Vector3
from nvblox_msgs.srv import EsdfAndGradients
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener


def quat2mat(q):                                                # (w, x, y, z) -> 3x3 rotation
    w, x, y, z = np.asarray(q, float) / np.linalg.norm(q)
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def mat2quat(R):                                                # 3x3 rotation -> (w, x, y, z)
    R = np.asarray(R, float); t = np.trace(R)
    if t > 0:
        s = 2 * np.sqrt(t + 1); return np.array([s / 4, (R[2, 1] - R[1, 2]) / s, (R[0, 2] - R[2, 0]) / s, (R[1, 0] - R[0, 1]) / s])
    i = int(np.argmax(np.diag(R))); j, k = (i + 1) % 3, (i + 2) % 3
    s = 2 * np.sqrt(1 + R[i, i] - R[j, j] - R[k, k]); q = np.zeros(4); q[0] = (R[k, j] - R[j, k]) / s
    q[1 + i] = s / 4; q[1 + j] = (R[j, i] + R[i, j]) / s; q[1 + k] = (R[k, i] + R[i, k]) / s; return q


ap = argparse.ArgumentParser()
ap.add_argument("--workcell", required=True)
ap.add_argument("--out", required=True)
ap.add_argument("--frame", default="fr3_link0")
ap.add_argument("--service", default="/nvblox_node/get_esdf_and_gradient")
ap.add_argument("--aabb-min", type=float, nargs=3, default=[-0.15, -0.90, -0.10])
ap.add_argument("--aabb-size", type=float, nargs=3, default=[0.95, 1.80, 0.70])
a = ap.parse_args(rclpy.utilities.remove_ros_args(sys.argv)[1:]); os.makedirs(a.out, exist_ok=True)
scene = json.load(open(os.path.join(a.workcell, "scene.json"))); cam = json.load(open(os.path.join(a.workcell, "camera.json")))

rclpy.init(); node = Node("nvblox_map_check"); buf = Buffer(); TransformListener(buf, node)
res = {"frame": a.frame, "aabb_min": a.aabb_min, "aabb_size": a.aabb_size}

# ---- TF chain check: base -> left optical must equal Isaac's CameraLeft pose (relative to the Franka base) -------------
T_w_opt = np.array(cam["T_world_camera_usd"]) @ np.diag([1.0, -1.0, -1.0, 1.0]); T_w_b = np.eye(4); T_w_b[:3, 3] = scene["franka_base"]
want = np.linalg.inv(T_w_b) @ T_w_opt; got = None; t0 = time.time()
while time.time() - t0 < 3:                                   # let the ZED node's dynamic lens TF (/tf, ~3 Hz) replace
    rclpy.spin_once(node, timeout_sec=0.1)                    # the URDF's static one, as it does for nvblox
while got is None and time.time() - t0 < 20:
    rclpy.spin_once(node, timeout_sec=0.2)
    try:
        tr = buf.lookup_transform(a.frame, "zed_left_camera_frame_optical", rclpy.time.Time()).transform
        got = np.eye(4); q = tr.rotation; got[:3, :3] = quat2mat([q.w, q.x, q.y, q.z]); got[:3, 3] = [tr.translation.x, tr.translation.y, tr.translation.z]
    except Exception:
        pass
if got is not None:
    dR = got[:3, :3].T @ want[:3, :3]
    res["tf_check"] = {"position_err_mm": round(float(np.linalg.norm(got[:3, 3] - want[:3, 3]) * 1000), 3),
                       "rotation_err_deg": round(float(np.degrees(np.arccos(np.clip((np.trace(dR) - 1) / 2, -1, 1)))), 4),
                       "camera_in_base_m": got[:3, 3].round(3).tolist()}
else:
    res["tf_check"] = f"no TF {a.frame} -> zed_left_camera_frame_optical"

# ---- ESDF grid from the service ---------------------------------------------------------------------------------------
cli = node.create_client(EsdfAndGradients, a.service)
if not cli.wait_for_service(timeout_sec=20):
    res["esdf"] = "service /nvblox_node/get_esdf_and_gradient not available"; print(json.dumps(res)); sys.exit(1)
req = EsdfAndGradients.Request(); req.update_esdf = True; req.visualize_esdf = True; req.use_aabb = True; req.frame_id = a.frame
req.aabb_min_m = Point(x=a.aabb_min[0], y=a.aabb_min[1], z=a.aabb_min[2]); req.aabb_size_m = Vector3(x=a.aabb_size[0], y=a.aabb_size[1], z=a.aabb_size[2])
t1 = time.time(); fut = cli.call_async(req); rclpy.spin_until_future_complete(node, fut, timeout_sec=30); rep = fut.result()
ms = (time.time() - t1) * 1000
if rep is None or not rep.success:
    res["esdf"] = f"service call failed (reply {rep})"; print(json.dumps(res)); sys.exit(1)
dims = [(d.label, d.size) for d in rep.esdf_and_gradients.layout.dim]
arr = np.array(rep.esdf_and_gradients.data, dtype=np.float32).reshape([s for _, s in dims])
esdf = arr[..., 0] if arr.ndim == 4 else arr                  # last axis: distance + 3 gradient components
vs = float(rep.voxel_size_m); org = np.array([rep.origin_m.x, rep.origin_m.y, rep.origin_m.z])
np.savez(os.path.join(a.out, "esdf.npz"), esdf=esdf, origin=org, voxel=vs)
res["esdf"] = {"call_ms": round(ms, 1), "layout": dims, "voxel_m": vs, "origin_m": org.round(3).tolist(),
               "finite_frac": round(float(np.isfinite(esdf).mean()), 3), "min_m": round(float(np.nanmin(esdf)), 3),
               "max_m": round(float(np.nanmax(esdf)), 3)}

# ---- geometry checks: surfaces are the voxels with |d| < 1 voxel ------------------------------------------------------
ix = np.argwhere(np.isfinite(esdf) & (np.abs(esdf) <= vs))
P = org + (ix + 0.5) * vs                                     # voxel centres, base frame
res["surface_voxels"] = int(len(P))
on_table = (P[:, 0] > 0.05) & (P[:, 0] < 0.65) & (np.abs(P[:, 1]) < 0.75)
z_tab = P[on_table & (P[:, 2] < 0.06), 2]
if len(z_tab):
    hist, edges = np.histogram(z_tab, bins=np.arange(-0.06, 0.07, vs))
    res["table_top_z_m"] = {"peak": round(float((edges[np.argmax(hist)] + edges[np.argmax(hist) + 1]) / 2), 3),
                            "expected": 0.0, "voxels": int(len(z_tab))}
objs = {}
for name, o in scene["objects"].items():
    fx, fl = o["forward_left_of_base_m"]
    near = (np.hypot(P[:, 0] - fx, P[:, 1] - fl) < 0.06) & (P[:, 2] > 0.03)
    objs[name] = {"expected_xy": [fx, fl], "voxels_above_table": int(near.sum()),
                  "top_z_m": round(float(P[near, 2].max()), 3) if near.any() else None,
                  "expected_top_z_m": round(o["centre_world"][2] * 2 - o["bottom_z"] - scene["table_top_z"], 3)}
res["objects"] = objs
json.dump(res, open(os.path.join(a.out, "map_check.json"), "w"), indent=1); print(json.dumps(res))
os.system(f"chown -R 1000:1000 {a.out}")
