"""Is the object to manipulate out of the obstacle map? Reads /target_object/clear_box (sam2_tracker_node) and asks the ESDF service of
--node for the region around it, once plain and once with the clear box (what cuRobo's ESDF request sends, as on the Spot).
Counts occupied voxels inside the box (+margin) and draws a top view (x-y) + side view (x-z) of that region.
  python3 target_map_check.py [--node fr3_nvblox] [--tag name] [--out /tmp/track]"""
import argparse, json, os, sys, time, numpy as np, rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, Vector3
from nvblox_msgs.srv import EsdfAndGradients
from visualization_msgs.msg import Marker
ap = argparse.ArgumentParser(); ap.add_argument("--node", default="fr3_nvblox"); ap.add_argument("--tag", default="check"); ap.add_argument("--out", default="/tmp/track")
ap.add_argument("--margin", type=float, default=0.15)
a, ros = ap.parse_known_args()
rclpy.init(args=ros); n = Node("target_map_check"); box = {}
n.create_subscription(Marker, "/target_object/clear_box", lambda m: box.__setitem__("m", m), 10)
t0 = time.monotonic()
while "m" not in box and time.monotonic() - t0 < 10: rclpy.spin_once(n, timeout_sec=0.1)
if "m" not in box: print("no /target_object/clear_box"); sys.exit(1)
m = box["m"]; c = np.array([m.pose.position.x, m.pose.position.y, m.pose.position.z]); s = np.array([m.scale.x, m.scale.y, m.scale.z])
print(f"clear box in {m.header.frame_id}: centre {np.round(c, 3).tolist()} size {np.round(s, 3).tolist()} (orientation w={m.pose.orientation.w:.3f})")
cli = n.create_client(EsdfAndGradients, f"/{a.node}/get_esdf_and_gradient"); cli.wait_for_service(timeout_sec=10.0)
lo = c - s / 2 - a.margin; size = s + 2 * a.margin; lo[2] = max(lo[2], -0.04)
def query(clear):
    r = EsdfAndGradients.Request(); r.update_esdf, r.use_aabb, r.frame_id = True, True, m.header.frame_id or "fr3_link0"
    r.aabb_min_m = Point(x=float(lo[0]), y=float(lo[1]), z=float(lo[2])); r.aabb_size_m = Vector3(x=float(size[0]), y=float(size[1]), z=float(size[2]))
    if clear:
        r.aabbs_to_clear_min_m = [Point(x=float(c[0] - s[0] / 2), y=float(c[1] - s[1] / 2), z=float(c[2] - s[2] / 2))]
        r.aabbs_to_clear_size_m = [Vector3(x=float(s[0]), y=float(s[1]), z=float(s[2]))]
    f = cli.call_async(r); rclpy.spin_until_future_complete(n, f, timeout_sec=20.0); res = f.result()
    d = np.asarray(res.esdf_and_gradients.data, np.float32).reshape([x.size for x in res.esdf_and_gradients.layout.dim])
    o = np.array([res.origin_m.x, res.origin_m.y, res.origin_m.z]); vs = res.voxel_size_m
    ii = np.stack(np.meshgrid(*[np.arange(k) for k in d.shape], indexing="ij"), -1); P = o + (ii + 0.5) * vs
    occ = (d > -999) & (d <= 0)
    inbox = np.all(np.abs(P - c) <= s / 2 + vs, axis=-1) & (P[..., 2] > 0.02)                # the object's box, above the table top
    return P, occ, inbox, d
out = {}
import matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt; from matplotlib.patches import Rectangle
fig, ax = plt.subplots(2, 2, figsize=(11, 9))
for row, clear in enumerate((False, True)):
    P, occ, inbox, d = query(clear)
    k = f"{'with' if clear else 'without'}_clear_box"; out[k] = dict(occupied_in_object_box=int((occ & inbox).sum()), occupied_region=int(occ.sum()))
    for col, (i, j, lab) in enumerate([(0, 1, "top (x-y), above table"), (0, 2, "side (x-z)")]):
        sel = occ & (P[..., 2] > 0.02) if col == 0 else occ
        ax[row, col].scatter(P[sel & ~inbox][:, i], P[sel & ~inbox][:, j], s=6, c="tab:blue", label="occupied")
        ax[row, col].scatter(P[sel & inbox][:, i], P[sel & inbox][:, j], s=10, c="red", label="occupied inside the object box")
        ax[row, col].add_patch(Rectangle((c[i] - s[i] / 2, c[j] - s[j] / 2), s[i], s[j], fill=False, ec="green", lw=2, label="clear box"))
        ax[row, col].set_aspect("equal"); ax[row, col].set_title(f"{k.replace('_', ' ')}: {lab}"); ax[row, col].set_xlabel("x (m)"); ax[row, col].set_ylabel("yz"[col] + " (m)")
ax[0, 0].legend(fontsize=7, loc="lower left")
fig.suptitle(f"/{a.node}: object (clear box) region, {a.tag} -> occupied voxels inside the object box: "
             f"{out['without_clear_box']['occupied_in_object_box']} plain, {out['with_clear_box']['occupied_in_object_box']} with clear box")
fig.tight_layout(); os.makedirs(a.out, exist_ok=True); fig.savefig(os.path.join(a.out, f"target_map_{a.tag}.png"), dpi=100)
out.update(tag=a.tag, node=a.node, box_centre=c.round(3).tolist(), box_size=s.round(3).tolist())
json.dump(out, open(os.path.join(a.out, f"target_map_{a.tag}.json"), "w"), indent=1); print(json.dumps(out))
