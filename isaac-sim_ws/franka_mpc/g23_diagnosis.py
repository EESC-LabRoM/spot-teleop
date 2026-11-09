"""Why g02/g03 missed in the HiveBoard valve MPC run (valve_mpc5), one figure. Needs only runs_text/valve_mpc5 + valve_ggx/grasps_top6 (git) and the lever mesh.
   ../graspgenx/.venv/bin/python -I g23_diagnosis.py <runs_text dir> <out.png>"""
import json, os, sys
import numpy as np, trimesh
import matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation as R
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__))); import valve_geometry as vg
rt, out = sys.argv[1], sys.argv[2]; gdir = f"{rt}/valve_ggx/grasps_top6"; mp = f"{rt}/valve_mpc5"
Vl, Fl = vg.link_mesh("Mesh_1.obj", vg.LEVER_VIS); P0 = trimesh.Trimesh(vertices=vg.to_base(Vl), faces=Fl, process=False).voxelized(0.001).fill().points
hinge_c, _ = vg.rot_about_hinge(0.0)
def coverage(P, Rm, t):                                     # share of the finger pad (21 x 45 mm) behind which lever material lies between the open fingers
    pl = (P - t) @ Rm; sel = (np.abs(pl[:, 0]) <= 0.0105) & (pl[:, 2] >= 0.0584) & (pl[:, 2] <= 0.1034) & (np.abs(pl[:, 1]) < 0.039)
    return len(np.unique(np.round(pl[sel][:, [0, 2]] * 1000).astype(int), axis=0)) / 945 * 100
col = {4: "#1f77b4", 10: "#d62728", 9: "#2ca02c"}; held = {0: True, 1: True, 2: False, 3: False, 4: True, 5: True}
fig, ax = plt.subplots(1, 3, figsize=(15, 4.4))
shorts = np.arange(0, 21, 1.0)
for g in (0, 2, 4):
    d = json.load(open(f"{gdir}/g{g:02d}.json")); p = np.array(d["grasp"]); Rm = R.from_quat(p[3:]).as_matrix()
    ax[0].plot(shorts, [coverage(P0, Rm, p[:3] - s * 1e-3 * Rm[:, 2]) for s in shorts], color=col[d["index"]], lw=2, label=f"GraspGen #{d['index']} (g{g:02d}/g{g+1:02d}), conf {d['confidence']:.2f}")
ax[0].set_xlabel("hand SHORT of the grasp pose along the approach axis (mm)"); ax[0].set_ylabel("pad area with lever between fingers (%)")
ax[0].set_title("exact hand pose: every grasp is a shallow pinch", fontsize=10); ax[0].legend(fontsize=8); ax[0].grid(alpha=.3)
short, cov, fing = [], [], []
for ep in range(6):
    z = np.load(f"{mp}/ep{ep:02d}/trace.npz"); t = z["t"] - z["t"][0]; d = json.load(open(f"{gdir}/g{ep:02d}.json")); p = np.array(d["grasp"]); Rg = R.from_quat(p[3:]).as_matrix()
    i7 = int(np.argmax(z["q_fingers"].mean(1) < 0.035)); short.append(-(Rg.T @ (z["hand"][i7] - p[:3]))[2] * 1e3)   # the step the fingers start to close
    i = np.searchsorted(t, 7.5); th = np.radians(z["lever_deg"][i]); _, Rl = vg.rot_about_hinge(th); P = (P0 - hinge_c) @ Rl.T + hinge_c
    cov.append(coverage(P, R.from_quat(z["hand_quat"][i]).as_matrix(), z["hand"][i])); fing.append(z["q_fingers"][-1].mean() * 1e3)
for ep in range(6):
    d = json.load(open(f"{gdir}/g{ep:02d}.json")); ok = held[ep]
    ax[1].scatter(short[ep], fing[ep], s=110, color=col[d["index"]], marker="o" if ok else "X", edgecolor="k", zorder=3); ax[1].annotate(f"g{ep:02d}", (short[ep], fing[ep]), textcoords="offset points", xytext=(6, 5), fontsize=9)
ax[1].axhline(11.5, color="grey", ls=":"); ax[1].text(5.8, 9.4, "lever held (23 mm / 2 per side)", fontsize=8, color="grey")
ax[1].set_xlabel("hand shortfall when the fingers start to close (mm)"); ax[1].set_ylabel("finger opening per side, end of episode (mm)")
ax[1].set_title("what the MPC delivered (o held and turned, X missed)", fontsize=10); ax[1].grid(alpha=.3)
names = [f"g{e:02d}" for e in range(6)]; ax[2].bar(names, cov, color=[col[json.load(open(f'{gdir}/g{e:02d}.json'))['index']] for e in range(6)], edgecolor="k")
for e in range(6): ax[2].text(e, cov[e] + 0.15, f"{cov[e]:.0f}%\n" + ("held" if held[e] else "MISSED"), ha="center", fontsize=8)
ax[2].set_ylabel("pad area with lever between fingers, t = 7.5 s (%)"); ax[2].set_ylim(0, 8); ax[2].set_title("actual poses while the fingers close", fontsize=10)
fig.suptitle("HiveBoard lever, cuRobo MPC in Newton (valve_mpc5): g02/g03 stopped 11-13 mm short; all six grasps are shallow pinches", fontsize=11)
fig.tight_layout(); fig.savefig(out, dpi=130); print(out, [round(x, 1) for x in short], [round(x, 1) for x in cov])
