"""Checkpoint-1 results for one run_free_space.sh run: tracking of the moving /wrist_pose reference by the sim Franka under
curobo_mpc_node.py, the node's solve time, and command timing in sim time.
   /usr/bin/python3 analyze_ros_run.py runs/ros_free_rt      -> runs/ros_free_rt/RESULT.json, plots.png"""
import csv, json, os, re, sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

run = os.path.abspath(sys.argv[1] if len(sys.argv) > 1 else "runs/ros_free_rt")
T = np.load(os.path.join(run, "trace.npz")); RI = json.load(open(os.path.join(run, "RESULT_isaac.json")))
T_APPROACH, T_SETTLE, T_RAMP, T_LOOP = 3.0, 0.5, 1.0, 13.0          # as in wrist_path.py
m = re.search(r"start at sim t=([0-9.]+)", open(os.path.join(run, "goal.log")).read())
t0 = float(m.group(1)) if m else float(T["target_stamp"].min())
tp = T["t"] - t0                                                   # path time (sim s)
full = (tp >= T_APPROACH + T_SETTLE + T_RAMP) & (tp <= T_APPROACH + T_SETTLE + T_LOOP - T_RAMP)
d = np.linalg.norm(T["hand"] - T["target"], axis=1)
v_hand = np.linalg.norm(np.gradient(T["hand"], T["t"], axis=0), axis=1)
v_ref = np.linalg.norm(np.gradient(T["target"], T["t"], axis=0), axis=1)
q_err = np.degrees(np.abs(T["q_cmd"] - T["q"]))
delay_ms = (T["cmd_applied_t"] - T["cmd_stamp"]) * 1000.0
new_cmds = np.diff(T["cmd_n"])                                     # commands per 20 ms logging step

rows = [r for r in csv.DictReader(open(os.path.join(run, "mpc_node.csv"))) if r["event"] == "control_step" and r["mpc_step_ms"]]
step_ms = np.array([float(r["mpc_step_ms"]) for r in rows]); gap_ms = np.array([float(r["gap_ms"]) for r in rows])
q = lambda x, p: round(float(np.percentile(x, p)), 2)

demo = json.load(open(os.path.join(os.path.dirname(run), "path_mpc", "RESULT.json")))
res = {"run": os.path.basename(run), "real_time_factor": RI["real_time_factor"], "record_sim_s": RI["record_sim_s"],
       "tracking_err_cm_at_full_speed": {"mean": round(float(d[full].mean()) * 100, 2), "p95": round(q(d[full], 95) * 100, 2),
                                         "max": round(float(d[full].max()) * 100, 2)},
       "final_err_cm": round(float(d[-1]) * 100, 2),
       "hand_speed_mps_at_full_speed": {"mean": round(float(v_hand[full].mean()), 3), "std": round(float(v_hand[full].std()), 3)},
       "reference_speed_mps_at_full_speed": round(float(v_ref[full].mean()), 3),
       "joint_cmd_vs_measured_deg": {"p95": q(q_err[full], 95), "max": round(float(q_err.max()), 2)},
       "commands_per_sim_s": round(RI["commands_received"] / RI["record_sim_s"], 1),
       "control_steps_without_new_command_pct": round(float((new_cmds == 0).mean()) * 100, 1),
       "command_delay_sim_ms": {"median": q(delay_ms, 50), "p95": q(delay_ms, 95), "max": round(float(delay_ms.max()), 1)},
       "node_mpc_step_ms_wall": {"median": q(step_ms, 50), "p95": q(step_ms, 95), "max": round(float(step_ms.max()), 2), "n": int(len(step_ms))},
       "node_loop_period_ms_wall": {"median": q(gap_ms, 50), "p95": q(gap_ms, 95)},
       "lockstep": RI.get("lockstep", False), "lockstep_timeouts": RI.get("lockstep_timeouts"),
       "in_process_demo_path_mpc": {"tracking_err_cm_mean": demo["path"]["tracking_err_cm_at_full_speed"]["mean"],
                                    "hand_speed_mps": demo["path"]["hand_speed_mps_at_full_speed"]["mean"],
                                    "mpc_solve_ms_median": demo["mpc_solve_ms"]["median"]}}
if len(T.get("lockstep_wait_ms", [])):                            # wall ms from the end of each update to the node's command
    w = T["lockstep_wait_ms"]; res["lockstep_wait_ms_wall"] = {"median": q(w, 50), "p95": q(w, 95), "max": round(float(w.max()), 1)}
json.dump(res, open(os.path.join(run, "RESULT.json"), "w"), indent=1)
print(json.dumps(res, indent=1))

SURF, INK, INK2, MUTED, GRID, AXIS = "#fcfcfb", "#0b0b0b", "#52514e", "#898781", "#e1e0d9", "#c3c2b7"
S1, S2 = "#2a78d6", "#eb6834"
plt.rcParams.update({"figure.facecolor": SURF, "axes.facecolor": SURF, "axes.edgecolor": AXIS, "axes.labelcolor": INK2,
                     "xtick.color": MUTED, "ytick.color": MUTED, "grid.color": GRID, "grid.linewidth": 0.6,
                     "font.size": 10, "axes.titlesize": 11, "axes.titleweight": "bold", "axes.titlecolor": INK,
                     "axes.spines.top": False, "axes.spines.right": False, "axes.grid": True, "lines.linewidth": 2.0,
                     "legend.frameon": False, "legend.labelcolor": INK2})
fig, ax = plt.subplots(2, 2, figsize=(13, 8.6))
fig.suptitle("Franka in sim, driven by curobo_mpc_node.py over ROS 2 (free space, no map)", x=0.01, ha="left",
             fontsize=14, fontweight="bold", color=INK)
fig.text(0.01, 0.935, f"{res['run']} ({'lockstep' if res['lockstep'] else 'free-running'}): real-time factor {res['real_time_factor']}, {res['commands_per_sim_s']} commands per "
         f"sim second, path from wrist_path.py (circle r 0.15 m + 4 cm wave, 6 s per lap, 2 laps)", ha="left", color=INK2)
def shade(a):
    a.axvspan(T_APPROACH + T_SETTLE + T_RAMP, T_APPROACH + T_SETTLE + T_LOOP - T_RAMP, color=GRID, alpha=0.45, lw=0, zorder=0)

a = ax[0, 0]
a.plot(T["target"][:, 1], T["target"][:, 0], color=S1, lw=1.6, label="reference (/wrist_pose)")
a.plot(T["hand"][:, 1], T["hand"][:, 0], color=S2, lw=1.6, label="hand (sim)")
a.set_aspect("equal"); a.invert_xaxis(); a.set_xlabel("y, left (m)"); a.set_ylabel("x, forward (m)")
a.set_title("Hand path seen from above (fr3_link0)", loc="left"); a.legend(loc="lower left")

a = ax[0, 1]; shade(a)
a.plot(tp, d * 100, color=S1, label="this run")
a.axhline(res["in_process_demo_path_mpc"]["tracking_err_cm_mean"], color=MUTED, lw=1, ls="--",
          label="in-process demo (franka_mpc_isaac.py), mean at full speed")
a.legend(loc="upper right", fontsize=9)
a.set_title("Distance from the hand to the reference", loc="left"); a.set_xlabel("path time (sim s)"); a.set_ylabel("cm")
a.set_ylim(0, a.get_ylim()[1] * 1.25)
a.text(T_APPROACH + T_SETTLE + T_LOOP / 2, a.get_ylim()[1] * 0.70, "full speed", color=INK2, fontsize=9, ha="center", va="top")

a = ax[1, 0]
a.plot(np.arange(len(step_ms)) * 10 * 0.02, step_ms, color=S1, lw=1.2, marker="o", ms=3)
a.axhline(20, color=MUTED, lw=1, ls="--")
a.text(len(step_ms) * 10 * 0.02, 20, "20 ms control period ", color=INK2, fontsize=9, va="bottom", ha="right")
a.set_ylim(0, max(25, float(step_ms.max()) * 1.15))
a.set_title("Node: mpc.step() time, every 10th step (wall clock)", loc="left"); a.set_xlabel("control time (sim s, since start)")
a.set_ylabel("ms")

a = ax[1, 1]; shade(a)
a.plot(tp, delay_ms, color=S1, lw=1.2)
a.set_title("Command delay: applied in Isaac minus node stamp (sim time)", loc="left"); a.set_xlabel("path time (sim s)")
a.set_ylabel("ms"); a.set_ylim(0, max(45, float(delay_ms.max()) * 1.15))
fig.tight_layout(rect=(0, 0, 1, 0.92))
fig.savefig(os.path.join(run, "plots.png"), dpi=110)
print("plots:", os.path.join(run, "plots.png"))
