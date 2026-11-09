"""Attraction-field trials (grasp_goal.py op_err + curobo_mpc_node.py pf_mode): per run, how far the hand was from the TRUE grasp when the
gripper started closing (position mm, angle deg), how far the target moved before the close (knocked), the final lift and the attraction
weight the node logged. Optional plot of all runs grouped by condition (run-name prefix up to the last '_t').
   python3 pf_eval.py runs/pf_off_t* runs/pf_on_t* [--json runs/pf_eval.json] [--plot runs/pf_eval.png]
"""
import argparse, glob, json, os, re
import numpy as np

T_CLOSE_START = 4.0 + 0.5 + 2.5                       # grasp_goal.py without via points: transit, settle, approach
ap = argparse.ArgumentParser(); ap.add_argument("runs", nargs="+"); ap.add_argument("--json", default=""); ap.add_argument("--plot", default="")
a = ap.parse_args()


def qmat(x, y, z, w):
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


rows = []
for r in [x for x in a.runs if os.path.isdir(x)]:
    try:
        d = np.load(os.path.join(r, "trace.npz")); g = json.load(open(glob.glob(os.path.join(r, "grasps", "*_selected.json"))[0]))
        gl = open(os.path.join(r, "goal.log")).read(); t0 = float(re.search(r"start at sim t=([0-9.]+)", gl).group(1))
    except Exception as e:
        print(f"{r}: skipped ({e})"); continue
    err = re.search(r"operator error \[([^\]]*)\]", gl); op = [float(v) for v in err.group(1).split(",")] if err else [0.0] * 6
    t = d["t"]; i = int(np.argmin(np.abs(t - (t0 + T_CLOSE_START))))
    pg, qg = np.array(g["grasp"][:3]), np.array(g["grasp"][3:])
    dp = float(np.linalg.norm(d["hand"][i] - pg)) * 1e3
    Rh, Rg = qmat(*d["hand_quat"][i]), qmat(*qg)
    ang = float(np.degrees(np.arccos(np.clip((np.trace(Rg.T @ Rh) - 1) / 2, -1, 1))))
    obj = d["obj"]; knocked = float(np.linalg.norm(obj[i, :2] - obj[0, :2])) * 1e3; lift = float(obj[-1, 2] - obj[0, 2]) * 1e3
    w = [float(x) for x in re.findall(r"Grasp attraction w=([0-9.]+)", open(os.path.join(r, "node.log")).read())] if os.path.exists(os.path.join(r, "node.log")) else []
    name = os.path.basename(r.rstrip("/")); cond = name.rsplit("_t", 1)[0]
    row = {"run": name, "condition": cond, "op_err_mm": round(float(np.linalg.norm(op[:3])) * 1e3, 1), "op_err_deg": round(float(np.degrees(np.linalg.norm(op[3:6]))), 1),
           "hand_err_mm": round(dp, 1), "hand_err_deg": round(ang, 1), "target_moved_before_close_mm": round(knocked, 1), "lift_mm": round(lift, 1),
           "success": bool(lift > 100.0), "max_weight": round(max(w), 2) if w else 0.0}
    rows.append(row); print(row)
if a.json: json.dump(rows, open(a.json, "w"), indent=1)
if a.plot and rows:
    import matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt
    conds = sorted({x["condition"] for x in rows}); col = ["#9aa5b1", "#2f6fb3", "#d08c2f", "#5a9e5a"]
    fig, ax = plt.subplots(1, 3, figsize=(11, 3.4))
    for j, c in enumerate(conds):
        R = [x for x in rows if x["condition"] == c]
        for k, key in enumerate(("hand_err_mm", "hand_err_deg", "lift_mm")):
            v = [x[key] for x in R]; ax[k].scatter([j] * len(v), v, color=col[j % 4], s=28, zorder=3); ax[k].plot([j - 0.2, j + 0.2], [np.median(v)] * 2, color="k", lw=1.5)
    for k, lab in enumerate(("hand-to-grasp error at close (mm)", "hand-to-grasp angle at close (deg)", "drill lift (mm)")):
        ax[k].set_xticks(range(len(conds)), conds); ax[k].set_title(lab, fontsize=10); ax[k].set_xlim(-0.6, len(conds) - 0.4)
        for s_ in ("top", "right"): ax[k].spines[s_].set_visible(False)
    ax[2].axhline(100, color="#999", lw=0.8, ls="--")
    fig.suptitle("Scripted operator with aim error, attraction field off vs on (Isaac, FR3, drill grasp #14)", fontsize=10); fig.tight_layout(); fig.savefig(a.plot, dpi=130)
