"""What does the map the cuRobo asks for contain? Queries the ESDF service of one or more nvblox nodes over cuRobo's grid box
(fr3_link0, centre [0.35 0 0.55], size [1.4 1.6 1.2], the box of fr3_curobo_mpc_node.py) with the robot static and counts occupied voxels
  (a) inside each arm link's convex hull (fr3_link_hulls.npz posed by the URDF chain from /joint_states) grown by 0 / 3 cm,
  (b) inside the target's /target_object/clear_box above the table (z > 2 cm), plain / with the 3 cm padded clear shape cuRobo sends,
  (d) ESDF service latency.
  python3 curobo_map_compare.py --nodes raw=fr3_nvblox_raw,masked=nvblox_node [--out DIR] [--tag t]"""
import argparse, importlib.util, json, os, sys, time
import numpy as np, rclpy
from geometry_msgs.msg import Point, Vector3
from nvblox_msgs.srv import EsdfAndGradients
from rclpy.node import Node
from scipy.spatial import ConvexHull
from sensor_msgs.msg import JointState
from visualization_msgs.msg import Marker

HERE = os.path.dirname(os.path.abspath(__file__))
spec = importlib.util.spec_from_file_location("armmask", os.path.join(HERE, "fr3_arm_mask_node.py")); am = importlib.util.module_from_spec(spec); spec.loader.exec_module(am)
GRID_CENTER, GRID_SIZE = np.array([0.35, 0.0, 0.55]), np.array([1.4, 1.6, 1.2])
ap = argparse.ArgumentParser(); ap.add_argument("--nodes", default="masked=nvblox_node"); ap.add_argument("--out", default="/tmp/cmp"); ap.add_argument("--tag", default="cmp")
ap.add_argument("--pad", type=float, default=0.03); ap.add_argument("--repeat", type=int, default=10)
a, ros = ap.parse_known_args(); os.makedirs(a.out, exist_ok=True)
rclpy.init(args=ros); n = Node("curobo_map_compare"); got = {}
n.create_subscription(JointState, "/joint_states", lambda m: got.__setitem__("js", m), 10)
n.create_subscription(Marker, "/target_object/clear_box", lambda m: got.__setitem__("box", m), 10)
t0 = time.monotonic()
while ("js" not in got or "box" not in got) and time.monotonic() - t0 < 15: rclpy.spin_once(n, timeout_sec=0.1)
q = dict(zip(got["js"].name, got["js"].position)); h = np.load(os.path.join(HERE, "fr3_link_hulls.npz"))
links = {str(k): h[str(k)].astype(float) for k in h["names"]}; poses = am.Chain(os.path.join(HERE, "fr3.urdf")).poses(q, links)
hulls = {}
for k, V in links.items():
    T = poses[k]; P = V @ T[:3, :3].T + T[:3, 3]; hulls[k] = (ConvexHull(P), P)
bm = got.get("box"); c = np.array([bm.pose.position.x, bm.pose.position.y, bm.pose.position.z]); s = np.array([bm.scale.x, bm.scale.y, bm.scale.z]) if bm else None

def query(node, clear):
    cli = n.create_client(EsdfAndGradients, f"/{node}/get_esdf_and_gradient"); cli.wait_for_service(timeout_sec=10.0)
    r = EsdfAndGradients.Request(); r.update_esdf, r.use_aabb, r.frame_id = True, True, "fr3_link0"
    lo = GRID_CENTER - GRID_SIZE / 2
    r.aabb_min_m = Point(x=float(lo[0]), y=float(lo[1]), z=float(lo[2])); r.aabb_size_m = Vector3(x=float(GRID_SIZE[0]), y=float(GRID_SIZE[1]), z=float(GRID_SIZE[2]))
    if clear and bm is not None:                                                   # what cuRobo's _build_target_clear sends: box + target_clear_padding_m
        hh = s / 2 + a.pad
        r.aabbs_to_clear_min_m = [Point(x=float(c[0] - hh[0]), y=float(c[1] - hh[1]), z=float(c[2] - hh[2]))]
        r.aabbs_to_clear_size_m = [Vector3(x=float(2 * hh[0]), y=float(2 * hh[1]), z=float(2 * hh[2]))]
    t = time.monotonic(); f = cli.call_async(r); rclpy.spin_until_future_complete(n, f, timeout_sec=30.0); dt = 1000 * (time.monotonic() - t); res = f.result()
    d = np.asarray(res.esdf_and_gradients.data, np.float32).reshape([x.size for x in res.esdf_and_gradients.layout.dim])
    o = np.array([res.origin_m.x, res.origin_m.y, res.origin_m.z]); vs = res.voxel_size_m
    ii = np.stack(np.meshgrid(*[np.arange(k) for k in d.shape], indexing="ij"), -1); P = o + (ii + 0.5) * vs
    return d, P, vs, dt

def inside(P, hull, margin): return np.all(P @ hull.equations[:, :3].T + hull.equations[:, 3] <= margin, axis=-1)
out = {"tag": a.tag, "joint_state_stamp": got["js"].header.stamp.sec, "clear_box": None if bm is None else dict(centre=c.round(3).tolist(), size=s.round(3).tolist(), pad=a.pad)}
import matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt
specs = [x.split("=") for x in a.nodes.split(",")]; fig, ax = plt.subplots(len(specs), 2, figsize=(12, 5 * len(specs)), squeeze=False)
for row, (label, node) in enumerate(specs):
    d, P, vs, dt = query(node, False); occ = (d > -999) & (d <= 0); lat = [dt]
    for _ in range(a.repeat - 1): lat.append(query(node, False)[3])
    r = dict(voxel_m=round(float(vs), 4), grid_shape=list(d.shape), occupied_total=int(occ.sum()), unknown=int((d < -999).sum()), esdf_ms_mean=round(float(np.mean(lat)), 1),
             esdf_ms_max=round(float(np.max(lat)), 1), esdf_ms_n=len(lat))
    per, arm_any = {}, np.zeros(occ.shape, bool); Po = P[occ]
    for k, (hl, _) in hulls.items():
        i0 = inside(Po, hl, 0.0); i3 = inside(Po, hl, 0.03); per[k] = dict(in_hull=int(i0.sum()), within_3cm=int(i3.sum())); arm_any[occ] |= i3
    r["occupied_per_arm_link"] = per; r["arm_voxels_in_hull_total"] = sum(v["in_hull"] for v in per.values()); r["arm_voxels_within_3cm_total"] = int(arm_any.sum())
    if bm is not None:
        above = P[..., 2] > 0.02; box = np.all(np.abs(P - c) <= s / 2 + vs, axis=-1) & above; box3 = np.all(np.abs(P - c) <= s / 2 + a.pad + vs, axis=-1) & above
        dc, Pc, _, _ = query(node, True); occc = (dc > -999) & (dc <= 0)
        r["drill_box_occupied"] = dict(plain=int((occ & box).sum()), with_3cm_clear_box=int((occc & box).sum()), plain_padded_region=int((occ & box3).sum()),
                                      with_3cm_clear_box_padded_region=int((occc & box3).sum()), unknown_in_box=int(((d < -999) & box).sum()))
    out[label] = dict(node=node, **r)
    sel = occ & (P[..., 2] > 0.02)
    ax[row, 0].scatter(P[sel][:, 0], P[sel][:, 1], s=3, c=P[sel][:, 2], cmap="viridis"); ax[row, 0].set_title(f"{label} (/{node}): occupied voxels z>2 cm, top view (colour = z)")
    ax[row, 1].scatter(P[occ][:, 0], P[occ][:, 2], s=3, c="tab:blue")
    for k, (hl, HP) in hulls.items():
        ax[row, 0].scatter(HP[:, 0], HP[:, 1], s=0.5, c="red", alpha=0.25); ax[row, 1].scatter(HP[:, 0], HP[:, 2], s=0.5, c="red", alpha=0.25)
    if bm is not None:
        from matplotlib.patches import Rectangle
        ax[row, 0].add_patch(Rectangle((c[0] - s[0] / 2, c[1] - s[1] / 2), s[0], s[1], fill=False, ec="orange", lw=2))
        ax[row, 1].add_patch(Rectangle((c[0] - s[0] / 2, c[2] - s[2] / 2), s[0], s[2], fill=False, ec="orange", lw=2))
    ax[row, 1].set_title(f"{label}: side view x-z (red = arm hull points, orange = target box); arm voxels in hull {r['arm_voxels_in_hull_total']}, drill box {r.get('drill_box_occupied',{}).get('plain')}")
    for x in ax[row]: x.set_aspect("equal"); x.set_xlabel("x (m)")
fig.tight_layout(); fig.savefig(os.path.join(a.out, f"curobo_map_{a.tag}.png"), dpi=90)
json.dump(out, open(os.path.join(a.out, f"curobo_map_{a.tag}.json"), "w"), indent=1); print(json.dumps(out, indent=1))
