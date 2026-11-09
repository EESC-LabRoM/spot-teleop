"""One line per run of a batch (run_batch.sh): did the grasp lift the target, what else moved, how long it took.
   python3 batch_summary.py runs/<prefix>_* [--csv runs/<prefix>.csv] [--plot runs/<prefix>.png]
Lift = target centre height at the end minus at the start of the recording (trace.npz obj, base frame); success = lift > 10 cm.
Wall = the run dir's isaac.log lifetime (Isaac start -> RESULT), the sim part = RESULT_isaac.json record_wall_s.
"""
import argparse, csv, json, os
import numpy as np

ap = argparse.ArgumentParser(); ap.add_argument("runs", nargs="+"); ap.add_argument("--csv", default=""); ap.add_argument("--plot", default="")
ap.add_argument("--target", default="drill"); a = ap.parse_args()
rows = []
for r in a.runs:
    f = os.path.join(r, "RESULT_isaac.json")
    if not os.path.exists(f): rows.append({"run": os.path.basename(r), "status": "no RESULT"}); continue
    R = json.load(open(f)); T = np.load(os.path.join(r, "trace.npz")); obj = T["obj"]
    lift = float(obj[-1, 2] - obj[0, 2]) if len(obj) else float("nan")
    moved = {k: v for k, v in R["object_centre_moved_mm"].items() if k != a.target}
    log = os.path.join(r, "isaac.log"); wall = float("nan")           # session-mode episodes (run_session.sh) have no isaac.log of their own
    if os.path.exists(log):
        wall = os.stat(log).st_mtime - (os.stat(log).st_birthtime if hasattr(os.stat(log), "st_birthtime") else os.stat(log).st_ctime)
        try: wall = os.stat(log).st_mtime - float(os.popen(f"stat -c %W {log}").read())
        except Exception: pass
    rows.append({"run": os.path.basename(r), "status": "ok", "lift_mm": round(lift * 1000, 1), "success": lift > 0.10,
                 "others_max_moved_mm": max(moved.values()) if moved else 0.0, "others_moved": ",".join(f"{k}:{v}" for k, v in moved.items() if v > 5),
                 "sim_s": R["record_sim_s"], "sim_wall_s": R["record_wall_s"], "rtf": R["real_time_factor"], "run_wall_s": round(wall, 1),
                 "render_every": R.get("render_every", 1), "paced": R.get("paced", True), "video_frames": R["video_frames"], "zeds": R["zeds"] or "-"})
keys = ["run", "status", "success", "lift_mm", "others_max_moved_mm", "others_moved", "sim_s", "sim_wall_s", "rtf", "run_wall_s", "render_every", "paced", "video_frames", "zeds"]
print(" | ".join(keys))
for x in rows: print(" | ".join(str(x.get(k, "")) for k in keys))
ok = [x for x in rows if x["status"] == "ok"]
if ok: print(f"success {sum(x['success'] for x in ok)}/{len(ok)}, lift median {np.median([x['lift_mm'] for x in ok]):.1f} mm, run wall median {np.median([x['run_wall_s'] for x in ok]):.0f} s, rtf median {np.median([x['rtf'] for x in ok]):.3f}")
if a.csv:
    with open(a.csv, "w", newline="") as fh: w = csv.DictWriter(fh, keys); w.writeheader(); [w.writerow({k: x.get(k, "") for k in keys}) for x in rows]
if a.plot and ok:
    import matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt
    fig, ax = plt.subplots(1, 2, figsize=(11, 3.8)); n = [x["run"] for x in ok]
    ax[0].bar(n, [x["lift_mm"] for x in ok], color=["#2a9d5c" if x["success"] else "#c0392b" for x in ok]); ax[0].axhline(100, ls="--", c="#8a919a")
    ax[0].set_title("target lift (mm), dashed = success threshold", loc="left"); ax[0].tick_params(axis="x", rotation=60, labelsize=7)
    ax[1].bar(n, [x["run_wall_s"] for x in ok], color="#3a6ea5"); ax[1].set_title("wall time per run (s)", loc="left"); ax[1].tick_params(axis="x", rotation=60, labelsize=7)
    fig.tight_layout(); fig.savefig(a.plot, dpi=110); print(a.plot)
