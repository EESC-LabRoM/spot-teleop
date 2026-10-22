#!/usr/bin/env python3
"""Score the nvblox map of the simulated bench against the exact scene geometry (fr3_sim_zed.scene_boxes()):
one ESDF service call over the cuRobo grid box, then per voxel the ground-truth signed distance to the boxes.
  - spurious occupied voxels: nvblox says occupied (ESDF < 0) but the true surface is > --tol away (the arm, if it was mapped)
  - obstacles: per obstacle, occupied voxels on it and the mapped top height vs the true one (what cuRobo would avoid)
  - ESDF error: |nvblox ESDF - true distance| over observed free voxels 5-30 cm from the obstacles
  - a figure: top view (x-y) and side view (x-z) of the occupied voxels, true boxes outlined, spurious in red.
  ROS_DOMAIN_ID=79 python3 nvblox_check.py --tag <name> [--out dir]
"""
import argparse
import json
import os
import sys
import time

import numpy as np
import rclpy
from geometry_msgs.msg import Point, Vector3
from nvblox_msgs.srv import EsdfAndGradients
from rclpy.node import Node

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from fr3_sim_zed import scene_boxes  # noqa: E402

GRID_CENTER, GRID_SIZE = np.array([0.35, 0.0, 0.55]), np.array([1.4, 1.6, 1.2])


def box_sdf(P, c, h):
    q = np.abs(P - np.asarray(c)) - np.asarray(h)
    return np.linalg.norm(np.maximum(q, 0), axis=-1) + np.minimum(q.max(-1), 0)


def main():
    ap = argparse.ArgumentParser(); ap.add_argument("--tag", default="run"); ap.add_argument("--out", default="/tmp/sim")
    ap.add_argument("--tol", type=float, default=0.04)
    a, ros = ap.parse_known_args()
    rclpy.init(args=ros); n = Node("nvblox_check"); c = n.create_client(EsdfAndGradients, "/nvblox_node/get_esdf_and_gradient")
    if not c.wait_for_service(timeout_sec=10.0): print("no ESDF service"); return 1
    r = EsdfAndGradients.Request(); r.update_esdf, r.use_aabb, r.frame_id = True, True, "fr3_link0"
    lo = GRID_CENTER - GRID_SIZE / 2
    r.aabb_min_m = Point(x=float(lo[0]), y=float(lo[1]), z=float(lo[2])); r.aabb_size_m = Vector3(x=float(GRID_SIZE[0]), y=float(GRID_SIZE[1]), z=float(GRID_SIZE[2]))
    f = c.call_async(r); rclpy.spin_until_future_complete(n, f, timeout_sec=30.0); res = f.result()
    if res is None or not res.success: print("ESDF call failed"); return 1
    vs = res.voxel_size_m; shape = [d.size for d in res.esdf_and_gradients.layout.dim]
    E = np.asarray(res.esdf_and_gradients.data, np.float32).reshape(shape)
    o = np.array([res.origin_m.x, res.origin_m.y, res.origin_m.z])
    ii, jj, kk = np.meshgrid(*[np.arange(s) for s in shape], indexing="ij")
    P = o + (np.stack([ii, jj, kk], -1) + 0.5) * vs                                          # voxel centres
    boxes = scene_boxes()
    gt = np.min(np.stack([box_sdf(P, b[1], b[2]) for b in boxes]), 0)
    known = E > -999.0; occ = known & (E <= 0)                      # nvblox: surface voxels 0, inside < 0
    spurious = occ & (gt > a.tol)
    obs = []
    for name, cc, hh, _ in boxes[1:]:
        near = occ & (box_sdf(P, cc, hh) <= a.tol)
        top_true = cc[2] + hh[2]; top_map = float(P[near][:, 2].max() + vs / 2) if near.any() else float("nan")
        obs.append(dict(name=name, occupied_near=int(near.sum()), top_true_cm=round(top_true * 100, 1), top_mapped_cm=round(top_map * 100, 1)))
    band = known & ~occ & (gt > 0.05) & (gt < 0.30)
    err = np.abs(E[band] - gt[band])
    out = dict(tag=a.tag, voxel=vs, known=int(known.sum()), occupied=int(occ.sum()), spurious=int(spurious.sum()),
               spurious_above_table=int((spurious & (P[..., 2] > 0.05)).sum()),
               esdf_err_median_cm=round(float(np.median(err)) * 100, 2) if err.size else None,
               esdf_err_p90_cm=round(float(np.percentile(err, 90)) * 100, 2) if err.size else None, obstacles=obs)
    os.makedirs(a.out, exist_ok=True)
    json.dump(out, open(os.path.join(a.out, f"nvblox_check_{a.tag}.json"), "w"), indent=1)
    print(json.dumps(out, indent=1))

    import matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt
    from matplotlib.patches import Rectangle
    fig, ax = plt.subplots(1, 2, figsize=(14, 6))
    above = occ & (P[..., 2] > 0.01)
    for k, (i0, i1, lab) in enumerate([(0, 1, "top view (x-y), voxels above the table"), (0, 2, "side view (x-z)")]):
        good, bad = (above & ~spurious) if k == 0 else (occ & ~spurious), spurious
        ax[k].scatter(P[good][:, i0], P[good][:, i1], s=3, c="tab:blue", label="occupied, true surface")
        ax[k].scatter(P[bad][:, i0], P[bad][:, i1], s=6, c="red", label=f"occupied, no true surface ({int(bad.sum())})")
        for name, cc, hh, _ in boxes:
            ax[k].add_patch(Rectangle((cc[i0] - hh[i0], cc[i1] - hh[i1]), 2 * hh[i0], 2 * hh[i1], fill=False, ec="k", lw=1.2))
        ax[k].plot(0, 0, "k^", ms=10); ax[k].set_aspect("equal"); ax[k].set_title(lab); ax[k].set_xlabel("x (m)"); ax[k].set_ylabel("yz"[k] + " (m)")
        ax[k].set_xlim(-0.35, 1.05); ax[k].set_ylim((-0.8, 0.8) if k == 0 else (-0.1, 1.0))
    ax[0].legend(loc="lower left", fontsize=8)
    fig.suptitle(f"nvblox vs ground truth, {a.tag}: {out['occupied']} occupied, {out['spurious']} with no true surface; "
                 f"ESDF err median {out['esdf_err_median_cm']} cm")
    fig.tight_layout(); fig.savefig(os.path.join(a.out, f"nvblox_check_{a.tag}.png"), dpi=110)
    return 0


if __name__ == "__main__":
    sys.exit(main())
