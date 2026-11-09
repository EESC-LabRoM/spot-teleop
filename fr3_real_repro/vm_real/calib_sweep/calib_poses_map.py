#!/usr/bin/env python3
"""EasyHeC calibration poses, step 1: sample joint configurations, keep the ones that are (a) clear of the object boxes (inflated) and the table,
(b) in the safe reach region, (c) well visible in BOTH ZED cameras, then pick a spread-out subset (greedy max-min in joint+link space).
  calib_poses.py <objects.npz> <out.json> [--margin 0.12] [--n 24] [--seed 0]
Runs in the sim image (torch + curobo).  Geometry of the world = axis-aligned boxes around the real object clouds + the table slab."""
import sys, json, argparse, numpy as np, torch
from curobo.cuda_robot_model.cuda_robot_model import CudaRobotModel, CudaRobotModelConfig
from curobo.types.base import TensorDeviceType
from curobo.util_file import load_yaml
ap = argparse.ArgumentParser(); ap.add_argument("esdf"); ap.add_argument("cloud"); ap.add_argument("out")
ap.add_argument("--margin", type=float, default=0.08, help="m: distance kept from the MAP surface"); ap.add_argument("--cloud-margin", type=float, default=0.06, help="m: distance kept from the raw robot-free depth points (catches thin things the map misses)"); ap.add_argument("--n", type=int, default=24); ap.add_argument("--seed", type=int, default=0)
ap.add_argument("--table-z", type=float, default=0.046); ap.add_argument("--table-margin", type=float, default=0.10)
ap.add_argument("--v1", type=float, default=0.70); ap.add_argument("--v2", type=float, default=0.0); ap.add_argument("--area2", type=float, default=3500.0); ap.add_argument("--N", type=int, default=150000)
ap.add_argument("--only", default="", help="comma list of cloud keys to keep as obstacles (e.g. o4 = only the fixed wall panel of the 10-31 clouds)")
ap.add_argument("--cfg", default="/workspace/franka_mpc/node_config/fr3_vcap1.2_acc10_real.yml")
a = ap.parse_args(); A_V1, A_V2 = a.v1, a.v2; rng = np.random.default_rng(a.seed)
Q_HOME = np.array([-0.4027, -0.7167, 0.1832, -2.3529, 0.1943, 1.4888, 0.5356])
LO = np.array([-2.7437, -1.7837, -2.9007, -3.0421, -2.8065, 0.5445, -3.0159]); HI = np.array([2.7437, 1.7837, 2.9007, -0.1518, 2.8065, 4.5169, 3.0159])
# ---- world: the live 2 cm MAP (ESDF snapshot, unobserved = free like the planner does) + the raw robot-free depth points + the table
from scipy.ndimage import map_coordinates
from scipy.spatial import cKDTree
Zm = np.load(a.esdf); E = Zm["esdf"].astype(float); ORG = Zm["origin"]; VOX = float(Zm["voxel"]); Ef = np.where(E <= -999, 1.0, E)
S = np.load(a.cloud); CP = np.vstack([S[k] for k in S.files]); CP = CP[(CP[:, 2] > 0.09) & (CP[:, 0] > 0.10) & (CP[:, 0] < 1.10) & (np.abs(CP[:, 1]) < 0.80) & (CP[:, 2] < 1.0)]
CP = CP[np.unique(np.floor(CP / 0.015).astype(int), axis=0, return_index=True)[1]]; CT = cKDTree(CP); print(f"world: map {E.shape} voxel {VOX} ({int((E<=0).sum() - (E<=-999).sum())} occupied), {len(CP)} cloud points above the table")
TABLE_TOP = a.table_z + a.table_margin; boxes = []
def esdf_at(C):
    idx = (C - ORG) / VOX - 0.5; d = map_coordinates(Ef, idx.reshape(-1, 3).T, order=1, mode="nearest").reshape(C.shape[:-1])
    out = ~np.all((idx >= 0) & (idx <= np.array(E.shape) - 1), axis=-1); return np.where(out, 1.0, d) - 0.5 * VOX
def world_gap(C, r):
    """min over spheres of the free distance beyond the margins (>= 0 = allowed): map surface, raw depth points, table  (C (N,S,3), r (S,))"""
    g = (esdf_at(C) - r - a.margin).min(1)
    dc, _ = CT.query(C.reshape(-1, 3), distance_upper_bound=0.5); dc = dc.reshape(C.shape[:2]); g = np.minimum(g, (dc - r - a.cloud_margin).min(1))
    return np.minimum(g, (C[..., 2] - r - TABLE_TOP).min(1))
# ---- robot
kin = load_yaml(a.cfg)["robot_cfg"]["kinematics"]; kin["collision_sphere_buffer"] = 0.0
tdev = TensorDeviceType(); model = CudaRobotModel(CudaRobotModelConfig.from_data_dict(kin, tdev))
def spheres(Q):
    q = torch.tensor(Q, device=tdev.device, dtype=tdev.dtype); q = torch.cat([q, torch.full((len(q), 2), 0.04, device=tdev.device, dtype=tdev.dtype)], 1) if q.shape[1] == 7 else q
    st = model.get_state(q); S = st.link_spheres_tensor.cpu().numpy(); S = S[:, S[0, :, 3] > 0]; return S[..., :3], S[0, :, 3], st.ee_pose.position.cpu().numpy(), st.ee_pose.quaternion.cpu().numpy()
# ---- cameras (live extrinsics; zed2 = both the live one and the tag-PnP estimate; a pose must be visible under either)
K = {"zed1": np.array([[261.903, 0, 315.129], [0, 261.903, 181.272], [0, 0, 1]]), "zed2": np.array([[475.971, 0, 323.432], [0, 475.971, 177.632], [0, 0, 1]])}
EX = {"zed1": [np.array(json.load(open("/c/zed1_live.json"))["T_base_cam"])], "zed2": [np.array(json.load(open("/c/zed2_live.json"))["T_base_cam"]), np.array(json.load(open("/c/zed2_tagpnp.json"))["T_base_cam_new"])]}
def visible_area(C, r, cam, Tb, margin=18):
    Tc = np.linalg.inv(Tb); X = C @ Tc[:3, :3].T + Tc[:3, 3]; z = X[..., 2]; f = K[cam][0, 0]; u = f * X[..., 0] / np.maximum(z, 1e-6) + K[cam][0, 2]; v = f * X[..., 1] / np.maximum(z, 1e-6) + K[cam][1, 2]
    ok = (z > 0.30) & (u > margin) & (u < 640 - margin) & (v > margin) & (v < 360 - margin); return (ok * np.pi * (f * r / np.maximum(z, 1e-6)) ** 2).sum(1), ok.sum(1)
def visible_frac(C, cam, Tb, margin=18):
    Tc = np.linalg.inv(Tb); X = C @ Tc[:3, :3].T + Tc[:3, 3]; z = X[..., 2]; u = K[cam][0, 0] * X[..., 0] / np.maximum(z, 1e-6) + K[cam][0, 2]; v = K[cam][1, 1] * X[..., 1] / np.maximum(z, 1e-6) + K[cam][1, 2]
    ok = (z > 0.35) & (u > margin) & (u < 640 - margin) & (v > margin) & (v < 360 - margin); return ok.mean(1)
# ---- sample
N = a.N; sig = np.array([0.55, 0.45, 0.65, 0.40, 0.80, 0.55, 1.0]); Q = Q_HOME + rng.normal(size=(N, 7)) * sig
Q = Q[np.all((Q > LO + 0.30) & (Q < HI + -0.30), axis=1)]; print("sampled", N, "-> inside joint limits (0.30 rad margin):", len(Q))
C, r, EP, EQ = spheres(Q)
static = C.std(0).max(-1) < 0.005; print("static (base) spheres excluded from the world check and visibility:", int(static.sum()), "of", C.shape[1])
C, r = C[:, ~static], r[~static]
Ch, rh, _, _ = spheres(Q_HOME[None]); Ch = Ch[:, ~static]; print(f"HOME pose in this world: gap beyond the margins {100*world_gap(Ch, r)[0]:.1f} cm (>=0 ok; a negative number means phantom voxels at the arm or objects too close to home)")
keep = (EP[:, 0] > 0.28) & (EP[:, 0] < 0.72) & (np.abs(EP[:, 1]) < 0.45) & (EP[:, 2] > 0.22) & (EP[:, 2] < 0.65); print("hand in the safe region:", int(keep.sum()))
Q, C, EP, EQ = Q[keep], C[keep], EP[keep], EQ[keep]
g = world_gap(C, r); keep = g > 0.01; print("clear of the map / raw points / table with the margins (>1 cm beyond):", int(keep.sum()))
Q, C, EP, EQ = Q[keep], C[keep], EP[keep], EQ[keep]
arm = np.arange(C.shape[1]) >= 4          # skip the first moving spheres (shoulder area, little information)
distal = np.arange(C.shape[1]) >= C.shape[1] - 25      # forearm / wrist / hand spheres: the part zed2 (top camera, narrow view over the table) has to see
v1 = np.maximum.reduce([visible_frac(C[:, arm], "zed1", T) for T in EX["zed1"]])
A2 = np.minimum.reduce([visible_area(C[:, distal], r[distal], "zed2", T)[0] for T in EX["zed2"]])    # px^2 of projected distal spheres, under BOTH zed2 pose hypotheses
v2 = A2; good2 = A2 >= a.area2
keep = v1 > A_V1; print(f"zed1 sees >{A_V1:.0%} of the arm spheres: {int(keep.sum())} candidates; of them zed2 sees >= {a.area2:.0f} px^2 of forearm/wrist/hand (both pose hypotheses): {int((keep & good2).sum())} (zed2 p90 {np.percentile(A2[keep],90):.0f} px^2)")
Q, C, EP, EQ, v1, v2, good2 = Q[keep], C[keep], EP[keep], EQ[keep], v1[keep], v2[keep], good2[keep]
if len(Q) < a.n or good2.sum() < a.n // 3: print("too few candidates"); sys.exit(2)
# ---- greedy max-min selection on [joint config (scaled), hand position (m), arm sphere cloud centres]
feat = np.concatenate([Q / np.array([1, 1, 1, 1, 1, 1, 1.5]), 4 * EP, 2 * C[:, 4::4].reshape(len(Q), -1)], 1)
sel = [int(np.argmin(np.linalg.norm(Q - Q_HOME, axis=1) + 0.0))]
d = np.linalg.norm(feat - feat[sel[0]], axis=1); nq = a.n // 2
while len(sel) < nq: j = int(np.argmax(np.where(good2, d, -1))); sel.append(j); d = np.minimum(d, np.linalg.norm(feat - feat[j], axis=1))   # half of the poses: forearm/hand clearly in zed2's view
while len(sel) < a.n: j = int(np.argmax(d)); sel.append(j); d = np.minimum(d, np.linalg.norm(feat - feat[j], axis=1))
sel = sel[1:] if np.linalg.norm(Q[sel[0]] - Q_HOME) < 0.15 else sel   # the first pick is the closest to home: the run starts there anyway
# ---- order: nearest neighbour from home in joint space
order, cur, left = [], Q_HOME, list(sel)
while left: j = min(left, key=lambda k: np.linalg.norm(Q[k] - cur)); order.append(j); left.remove(j); cur = Q[j]
P = Q[order]; print(f"selected {len(P)} poses; hand z {EP[order][:,2].min():.2f}..{EP[order][:,2].max():.2f}, x {EP[order][:,0].min():.2f}..{EP[order][:,0].max():.2f}, y {EP[order][:,1].min():.2f}..{EP[order][:,1].max():.2f}")
print("per-joint range of the selected poses (rad):", np.round(P.min(0), 2).tolist(), np.round(P.max(0), 2).tolist())
print("min gap of any selected pose beyond the margins: %.1f cm; zed1 visibility min %.2f; poses with zed2 area >= %.0f px^2: %d of %d" % (100 * world_gap(C[order], r).min(), v1[order].min(), a.area2, int(good2[order].sum()), len(order)))
json.dump({"q_home": Q_HOME.tolist(), "poses": P.tolist(), "margin": a.margin, "table_top": TABLE_TOP, "boxes": []}, open(a.out, "w"), indent=1)
print("wrote", a.out)
