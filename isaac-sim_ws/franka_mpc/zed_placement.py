"""Where to put the ZED stands: an offline search for the camera pair that sees the most of the bench and its objects.

Ray casting on the true geometry (no Isaac): the warehouse around the table (export_static_scene.py), the tabletop
objects as they stood at a run's start (objects_start.npz) and the Franka at retract (cuRobo collision spheres) as
occluders. A surface point counts as seen by a ZED X when BOTH stereo eyes (12 cm baseline) see it unoccluded, inside
the rendered field of view (fx 736 px at 1920x1080: 105 x 73 deg), 0.3 to --range m away, at an incidence under
--incidence deg. Candidates: floor stands on a grid around the table at several heights, aimed at the objects' centre,
the stand clear of the furniture and outside the arm's reach. Score = 0.75 x the mean over objects of the share of each
object's surface seen + 0.25 x the share of the reachable table top seen. Pairs are scored exactly (union of two
visibility sets, one matrix product on the GPU).
   curobo-isaac6/.venv/bin/python -I zed_placement.py [--run runs/graze_2zed_plus1cm] [--out runs/zed_placement]
"""
import argparse, json, os, time

import numpy as np
import open3d as o3d
import torch

HERE = os.path.dirname(os.path.abspath(__file__))
ap = argparse.ArgumentParser()
ap.add_argument("--run", default=os.path.join(HERE, "runs", "graze_2zed_plus1cm"))
ap.add_argument("--static", default=os.path.join(HERE, "runs", "static_scene", "static_scene.npz"))
ap.add_argument("--out", default=os.path.join(HERE, "runs", "zed_placement"))
ap.add_argument("--range", type=float, default=2.0, help="max camera-to-surface distance (m) that still maps well")
ap.add_argument("--incidence", type=float, default=75.0, help="max angle between the view ray and the surface normal (deg)")
ap.add_argument("--heights", type=float, nargs="+", default=[0.8, 1.1, 1.4, 1.7, 2.0])
ap.add_argument("--step", type=float, default=0.15, help="stand grid spacing (m)")
ap.add_argument("--near", type=float, default=0.02, help="a surface point is mapped when a seen point lies this close (m)")
ap.add_argument("--arm-poses", type=int, default=8, help="arm poses along the run's sweep to re-check the current and best pairs")
ap.add_argument("--reach", type=float, default=0.90, help="min distance from the Franka shoulder to the stand and camera (m)")
a = ap.parse_args(); os.makedirs(a.out, exist_ok=True); t_start = time.time()
FX, W, H, BASELINE = 736.0, 1920, 1080, 0.12
TANH, TANV = (W / 2) / FX, (H / 2) / FX
COS_INC = np.cos(np.radians(a.incidence))
rng = np.random.default_rng(0)

# ---- geometry ---------------------------------------------------------------------------------------------------------
S = np.load(a.static); static = np.concatenate([S[k] for k in S.files]).astype(np.float64)
O = dict(np.load(os.path.join(a.run, "objects_start.npz")))
root = np.array(json.load(open(os.path.join(a.run, "RESULT_isaac.json")))["root"])        # robot base (fr3_link0) in world
shoulder = root + [0, 0, 0.333]


def mesh(tris):
    t = np.asarray(tris, np.float64).reshape(-1, 3, 3); V = t.reshape(-1, 3); F = np.arange(len(V)).reshape(-1, 3)
    m = o3d.t.geometry.TriangleMesh(); m.vertex.positions = o3d.core.Tensor(V.astype(np.float32))
    m.triangle.indices = o3d.core.Tensor(F.astype(np.int32)); return m


from curobo.cuda_robot_model.cuda_robot_model import CudaRobotModel, CudaRobotModelConfig   # arm occluder: spheres at retract
from curobo.types.base import TensorDeviceType
from curobo.util_file import get_robot_configs_path, join_path, load_yaml
cfg = load_yaml(join_path(get_robot_configs_path(), "fr3.yml"))["robot_cfg"]; cfg["kinematics"]["collision_sphere_buffer"] = 0.0
kin = CudaRobotModel(CudaRobotModelConfig.from_data_dict(cfg["kinematics"], tensor_args=TensorDeviceType()))
Q = np.load(os.path.join(a.run, "trace.npz"))["q"]


def make_scene(q):                                               # static + objects + the arm's spheres at joints q
    sph = kin.get_state(torch.tensor(q[None], device="cuda", dtype=torch.float32)).link_spheres_tensor[0].cpu().numpy()
    arm = []
    for x, y, z, r in sph[sph[:, 3] > 0]:
        s_ = o3d.geometry.TriangleMesh.create_sphere(radius=float(r), resolution=8); s_.translate(root + [x, y, z])
        arm.append(np.asarray(s_.vertices)[np.asarray(s_.triangles)])
    sc_ = o3d.t.geometry.RaycastingScene()
    for t in (static, np.concatenate(arm), *O.values()): sc_.add_triangles(mesh(t))
    return sc_


scene = make_scene(Q[0])                                         # the search: arm at retract
dist_scene = o3d.t.geometry.RaycastingScene(); dist_scene.add_triangles(mesh(static))      # furniture only: stand clearance

# ---- targets: object surfaces (not the faces resting on the table) + the reachable table top --------------------------
pts, nrm, owner, names = [], [], [], list(O)
for k, n in enumerate(names):
    t = O[n].astype(np.float64); cr = np.cross(t[:, 1] - t[:, 0], t[:, 2] - t[:, 0]); ar = 0.5 * np.linalg.norm(cr, axis=1)
    keep = ar > 1e-12; t, cr, ar = t[keep], cr[keep], ar[keep]
    m = 4000; f = rng.choice(len(t), m, p=ar / ar.sum()); u, w = rng.random(m), rng.random(m); s = u + w > 1; u[s], w[s] = 1 - u[s], 1 - w[s]
    P = t[f, 0] + u[:, None] * (t[f, 1] - t[f, 0]) + w[:, None] * (t[f, 2] - t[f, 0]); N = cr[f] / np.linalg.norm(cr[f], axis=1, keepdims=True)
    ok = ~((P[:, 2] < t[..., 2].min() + 0.01) & (np.abs(N[:, 2]) > 0.9))          # drop the base resting on the table
    pts.append(P[ok]); nrm.append(N[ok]); owner.append(np.full(ok.sum(), k))
tz = 0.395; g = np.mgrid[-1.27:-0.47:0.02, -1.42:0.28:0.02].reshape(2, -1).T + 0.01
g = g[np.hypot(g[:, 0] - root[0], g[:, 1] - root[1]) < 0.855]
g = g[np.hypot(g[:, 0] - root[0], g[:, 1] - root[1]) > 0.12]                       # not under the robot's base
pts.append(np.c_[g, np.full(len(g), tz + 0.002)]); nrm.append(np.tile([0, 0, 1.0], (len(g), 1))); owner.append(np.full(len(g), -1))
P, N, OWN = np.concatenate(pts), np.concatenate(nrm), np.concatenate(owner)
wts = np.where(OWN >= 0, 0.75 / len(names) / np.bincount(OWN[OWN >= 0])[np.maximum(OWN, 0)], 0.25 / (OWN == -1).sum())
centre = np.mean([O[n].reshape(-1, 3).mean(0) for n in names], 0)                    # aim point: the objects' centre
print(f"targets: {len(P)} points ({(OWN >= 0).sum()} on {len(names)} objects, {(OWN == -1).sum()} on the table); aim {centre.round(2)}")


def frame(eye, at):
    f = at - eye; f /= np.linalg.norm(f); l = np.cross([0, 0, 1.0], f); l /= np.linalg.norm(l); return f, l, np.cross(f, l)


def visible(eye, at):
    """Bool per target point: both stereo eyes see it (FOV, range, incidence, unoccluded)."""
    f, l, u = frame(eye, at); ok = np.ones(len(P), bool)
    for e in (eye + l * BASELINE / 2, eye - l * BASELINE / 2):
        d = P - e; r = np.linalg.norm(d, axis=1); dn = d / r[:, None]
        zf = dn @ f; m = (zf > 0) & (np.abs(dn @ l) < TANH * zf) & (np.abs(dn @ u) < TANV * zf) & (r > 0.3) & (r < a.range)
        m &= np.abs(np.sum(dn * N, 1)) > COS_INC
        idx = np.nonzero(m & ok)[0]; hit = np.zeros(len(P), bool)
        if len(idx):
            rays = o3d.core.Tensor(np.c_[np.repeat(e[None], len(idx), 0), dn[idx]].astype(np.float32))
            th = scene.cast_rays(rays)["t_hit"].numpy(); hit[idx] = th > r[idx] - 0.01
        ok &= hit
    return ok


# ---- candidates -------------------------------------------------------------------------------------------------------
xs, ys = np.arange(-3.0, 1.0 + 1e-9, a.step), np.arange(-2.6, 1.6 + 1e-9, a.step)
cand, why = [], {"furniture": 0, "reach": 0, "too_close": 0}
for x in xs:
    for y in ys:
        for h in a.heights:
            pole = np.array([[x, y, z] for z in np.arange(0.15, h + 0.06, 0.1)] + [[x, y, h]])
            if np.min(dist_scene.compute_distance(o3d.core.Tensor(pole.astype(np.float32))).numpy()) < 0.12: why["furniture"] += 1; continue
            if np.min(np.linalg.norm(pole - shoulder, axis=1)) < a.reach: why["reach"] += 1; continue
            if np.linalg.norm(np.array([x, y, h]) - centre) < 0.6: why["too_close"] += 1; continue
            cand.append([x, y, h])
CUR = {"zed": json.load(open(os.path.join(a.run, "camera_zed.json")))["T_world_camera_usd"],
       "zed2": json.load(open(os.path.join(a.run, "camera_zed2.json")))["T_world_camera_usd"]}
# the json is the LEFT eye (USD camera: +X right, -Z forward): the rig centre is 6 cm to its right
cur_eye = {k: np.array(v)[:3, 3] + np.array(v)[:3, 0] * BASELINE / 2 for k, v in CUR.items()}
cur_at = {k: cur_eye[k] - np.array(v)[:3, 2] for k, v in CUR.items()}
C = np.array(cand); print(f"candidates: {len(C)} feasible stands ({why} rejected); casting rays ...", flush=True)
V = np.zeros((len(C), len(P)), bool); t0 = time.time()
for i, c in enumerate(C): V[i] = visible(c, centre)
Vcur = {k: visible(cur_eye[k], cur_at[k]) for k in cur_eye}
print(f"  {time.time() - t0:.0f} s")
# Scored as the map is checked (map_coverage.py: a true surface point is mapped if the map has a surface within 2 cm,
# which the 3 cm margin then covers): a point counts once some SEEN point lies within --near of it. A union of
# cameras dilates to the union of their dilations, so pairs stay exact.
from scipy.spatial import cKDTree
pr = cKDTree(P).query_pairs(a.near, output_type="ndarray"); ij = np.r_[pr, pr[:, ::-1], np.c_[np.arange(len(P)), np.arange(len(P))]]
A = torch.sparse_coo_tensor(torch.tensor(ij.T, device="cuda"), torch.ones(len(ij), device="cuda"), (len(P), len(P))).coalesce()
def near(vis):                                                   # (n, S) or (S,) bool -> within --near of a seen point
    v = torch.tensor(np.atleast_2d(vis), device="cuda", dtype=torch.float32)
    out = (torch.sparse.mm(A, v.T).T > 0).cpu().numpy(); return out if np.ndim(vis) == 2 else out[0]
seen_strict, seen_cur = V, Vcur
V = np.concatenate([near(V[k:k + 512]) for k in range(0, len(V), 512)]); Vcur = {k: near(v) for k, v in Vcur.items()}
seeable = V.any(0) | Vcur["zed"] | Vcur["zed2"]


def score(vis):
    vis = np.asarray(vis, bool); ob = [vis[OWN == k].mean() for k in range(len(names))]
    return {"score": round(float(wts @ vis), 4), "objects_mean": round(float(np.mean(ob)), 3), "table": round(float(vis[OWN == -1].mean()), 3),
            "per_object": {n: round(float(o), 3) for n, o in zip(names, ob)},
            "per_object_seeable": {n: round(float(vis[(OWN == k) & seeable].mean()), 3) for k, n in enumerate(names)}}


# ---- pairs (exact), the best partner for the current ZED 1, and a greedy third ------------------------------------------
Vt = torch.tensor(V, device="cuda", dtype=torch.float32); wt = torch.tensor(wts, device="cuda", dtype=torch.float32)
single = Vt @ wt; inter = (Vt * wt) @ Vt.T; pair = single[:, None] + single[None, :] - inter
sep = torch.cdist(torch.tensor(C[:, :2], device="cuda"), torch.tensor(C[:, :2], device="cuda")) < 0.4
pair[sep] = -1; i, j = np.unravel_index(int(torch.argmax(pair)), pair.shape)
v1 = torch.tensor(Vcur["zed"], device="cuda", dtype=torch.float32)
with1 = single + v1 @ wt - (Vt * wt) @ v1; k1 = int(torch.argmax(with1))
best2 = V[i] | V[j]; gain3 = ((torch.tensor(~best2, device="cuda", dtype=torch.float32) * wt) * Vt).sum(1); k3 = int(torch.argmax(gain3))
res = {"settings": {"range_m": a.range, "incidence_deg": a.incidence, "heights_m": a.heights, "grid_m": a.step, "reach_clear_m": a.reach,
                    "fov_deg": [round(float(np.degrees(2 * np.arctan(TANH))), 1), round(float(np.degrees(2 * np.arctan(TANV))), 1)],
                    "aim": centre.round(3).tolist(), "candidates": len(C), "rejected": why, "run": os.path.basename(a.run)},
       "current": {"zed": {"eye": cur_eye["zed"].round(3).tolist(), **score(Vcur["zed"])}, "zed2": {"eye": cur_eye["zed2"].round(3).tolist(), **score(Vcur["zed2"])},
                   "pair": score(Vcur["zed"] | Vcur["zed2"]),
                   "shoulder_dist_m": {k: round(float(min(np.linalg.norm(np.array([[*e[:2], z] for z in np.arange(0.15, e[2], 0.05)]) - shoulder, axis=1))), 2) for k, e in cur_eye.items()}},
       "best_single": {"eye": C[int(torch.argmax(single))].tolist(), **score(V[int(torch.argmax(single))])},
       "best_pair": {"a": C[i].tolist(), "b": C[j].tolist(), **score(best2)},
       "best_partner_for_current_zed": {"eye": C[k1].tolist(), **score(Vcur["zed"] | V[k1])},
       "best_third_after_best_pair": {"eye": C[k3].tolist(), **score(best2 | V[k3])},
       "seeable_by_any_candidate": score(seeable)}
top = torch.topk(pair.flatten(), 200); tp = []                                     # runner-ups: how flat is the optimum?
for v_, f_ in zip(top.values.tolist(), top.indices.tolist()):
    ii, jj = divmod(f_, len(C))
    if ii < jj: tp.append({"a": C[ii].tolist(), "b": C[jj].tolist(), "score": round(v_, 4)})
res["top_pairs"] = tp[:20]
sweep = {"current": [], "searched": []}                          # the arm moving: the run's sweep, --arm-poses poses
for q in Q[np.linspace(len(Q) * 0.2, len(Q) - 1, a.arm_poses).astype(int)]:
    scene = make_scene(q)
    sweep["current"].append(score(near(visible(cur_eye["zed"], cur_at["zed"]) | visible(cur_eye["zed2"], cur_at["zed2"])))["objects_mean"])
    sweep["searched"].append(score(near(visible(C[i], centre) | visible(C[j], centre)))["objects_mean"])
res["arm_along_sweep"] = {k: {"objects_mean_each": v, "mean": round(float(np.mean(v)), 3), "worst": round(float(np.min(v)), 3)} for k, v in sweep.items()}
print("arm along the sweep, objects covered:", {k: (v["mean"], v["worst"]) for k, v in res["arm_along_sweep"].items()})
val = []                                                         # does the model predict the measured maps? (map_coverage.py)
for cam, run in (("zed", "graze_1zed_plus1cm"), ("pair", "graze_2zed_plus1cm")):
    f = os.path.join(HERE, "runs", run, "map_error.json")
    if not os.path.exists(f): continue
    M_ = json.load(open(f))["objects"]; v = Vcur["zed"] if cam == "zed" else Vcur["zed"] | Vcur["zed2"]; B = P - root
    for k, n in enumerate(names):
        m = OWN == k; cx = 0.5 * (B[m, 0].min() + B[m, 0].max())
        for half, hm in (("front_half", B[:, 0] >= cx), ("robot_side_half", B[:, 0] < cx)):
            sel = m & hm & (B[:, 2] > 0.02); val.append((cam, n, half, float(v[sel].mean()), 1 - M_[n][half]["missed_over_2cm_pct"] / 100))
if val:
    pv, mv = np.array([x[3] for x in val]), np.array([x[4] for x in val])
    res["validation"] = {"cells": len(val), "bias": round(float((pv - mv).mean()), 3), "mae": round(float(np.abs(pv - mv).mean()), 3),
                         "corr": round(float(np.corrcoef(pv, mv)[0, 1]), 3), "rows": [[c, n, h, round(p_, 3), round(m_, 3)] for c, n, h, p_, m_ in val]}
    print(f"validation vs measured maps ({len(val)} object halves, ZED1 alone + current pair): predicted - measured "
          f"bias {res['validation']['bias']:+.3f}, MAE {res['validation']['mae']:.3f}, correlation {res['validation']['corr']:.2f}")
res["strict_seen"] = {"current_pair": score(seen_cur["zed"] | seen_cur["zed2"])["objects_mean"], "best_pair": score(seen_strict[i] | seen_strict[j])["objects_mean"]}
np.savez_compressed(os.path.join(a.out, "visibility.npz"), seen_cur_zed=seen_cur["zed"], seen_cur_zed2=seen_cur["zed2"], C=C, V=np.packbits(V, axis=1), n=len(P), P=P, OWN=OWN, names=names,
                    single=single.cpu().numpy(), with1=with1.cpu().numpy(), cur_zed=Vcur["zed"], cur_zed2=Vcur["zed2"], root=root, centre=centre)
json.dump(res, open(os.path.join(a.out, "placement.json"), "w"), indent=1)
for k in ("current", "best_single", "best_pair", "best_partner_for_current_zed", "best_third_after_best_pair", "seeable_by_any_candidate"):
    r = res[k]["pair"] if k == "current" else res[k]
    print(f"{k:30s} score {r['score']:.3f}  objects {r['objects_mean']:.3f}  table {r['table']:.3f}  "
          + " ".join(f"{n[:10]}={v:.2f}" for n, v in r["per_object"].items()) + "   " + str({kk: res[k][kk] for kk in ("a", "b", "eye") if kk in res[k]}))
print(f"done in {time.time() - t_start:.0f} s -> {a.out}/placement.json")
