# Does Isaac 6.0's FrankaFR3/fr3.usd (in franka_workcell.usda) agree with our cuRobo FR3 config (node_config/fr3.yml)?
# Prints the articulation's joint and link names, joint limits and drive gains, then teleports the arm to random joint
# angles inside the FR3 limits and compares PhysX's fr3_hand pose with cuRobo's forward kinematics (base frame).
#   ./run_isaac.sh -s check_fr3_isaac.py fr3_check
import argparse, json, os, time
HERE = os.path.dirname(os.path.abspath(__file__))
ap = argparse.ArgumentParser(); ap.add_argument("--out", required=True); ap.add_argument("--n", type=int, default=24)
ap.add_argument("--stage", default=os.path.join(HERE, "franka_workcell.usda"))
a = ap.parse_args(); os.makedirs(a.out, exist_ok=True)
from isaacsim import SimulationApp
app = SimulationApp({"headless": True, "width": 320, "height": 240, "multi_gpu": False})
import numpy as np, omni.usd, omni.timeline, warp as wp, torch
from pxr import Usd, UsdPhysics
from isaacsim.core.utils.stage import is_stage_loading
from isaacsim.core.simulation_manager import SimulationManager, SimulationEvent
ctx = omni.usd.get_context(); ctx.open_stage(a.stage); stage = ctx.get_stage()
for i in range(3000):
    app.update()
    if i > 60 and not is_stage_loading(): break
fr = stage.GetPrimAtPath("/World/franka")
art_root = next(str(p.GetPath()) for p in Usd.PrimRange(fr) if p.HasAPI(UsdPhysics.ArticulationRootAPI))
print("[fr3] reference:", [str(r.assetPath) for spec in fr.GetPrimStack() for r in spec.referenceList.prependedItems], "| root", art_root, flush=True)

from curobo.cuda_robot_model.cuda_robot_model import CudaRobotModel, CudaRobotModelConfig
from curobo.types.base import TensorDeviceType
from curobo.util_file import load_yaml
cfg = load_yaml(os.path.join(HERE, "node_config", "fr3.yml"))["robot_cfg"]["kinematics"]
kin = CudaRobotModel(CudaRobotModelConfig.from_data_dict(cfg, tensor_args=TensorDeviceType()))
lo, hi = [x.cpu().numpy() for x in kin.get_joint_limits().position]

def Rq(x, y, z, w):
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)], [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])
rng = np.random.default_rng(0); Q = lo + rng.random((a.n, 7)) * (hi - lo)
S = {"art": None, "k": -1, "wait": 0, "res": []}
def on_pre_step(dt, context):
    if S["art"] is None:
        view = SimulationManager.get_physics_simulation_view()
        if view is None: return
        art = view.create_articulation_view(art_root); S["art"] = art
        S["names"], S["links"] = list(art.shared_metatype.dof_names), list(art.shared_metatype.link_names)
        S["dev"] = art.get_dof_positions().device; S["idx"] = wp.array([0], dtype=wp.uint32, device=S["dev"])
        S["a2d"] = np.array([S["names"].index(f"fr3_joint{j + 1}") for j in range(7)]); S["hand"] = S["links"].index("fr3_hand")
        lim = art.get_dof_limits().numpy()[0]
        S["info"] = {"dof_names": S["names"], "link_names": S["links"], "dof_limits": lim.round(4).tolist(),
                     "stiffness": art.get_dof_stiffnesses().numpy()[0].round(1).tolist(), "damping": art.get_dof_dampings().numpy()[0].round(1).tolist(),
                     "max_velocity": art.get_dof_max_velocities().numpy()[0].round(3).tolist(),
                     "limits_vs_curobo_max_diff_rad": float(np.abs(lim[S["a2d"]] - np.stack([lo, hi], 1)).max())}
        print("[fr3]", json.dumps(S["info"]), flush=True); return
    art = S["art"]
    if S["wait"] > 0: S["wait"] -= 1; return
    if S["k"] >= 0:                                  # read the pose set S["k"] two steps ago
        q_isaac = art.get_dof_positions().numpy()[0][S["a2d"]]
        lt = art.get_link_transforms().numpy()[0]; rt = art.get_root_transforms().numpy()[0]
        R0 = Rq(*rt[3:7]); p = R0.T @ (lt[S["hand"], :3] - rt[:3]); Rh = R0.T @ Rq(*lt[S["hand"], 3:7])
        st = kin.get_state(torch.tensor(q_isaac[None], device="cuda", dtype=torch.float32))
        pc = st.ee_position[0].cpu().numpy(); w, x, y, z = st.ee_quaternion[0].cpu().numpy(); Rc = Rq(x, y, z, w)
        ang = np.degrees(np.arccos(np.clip((np.trace(Rh.T @ Rc) - 1) / 2, -1, 1)))
        S["res"].append({"q": q_isaac.round(4).tolist(), "pos_err_mm": float(np.linalg.norm(p - pc) * 1e3), "rot_err_deg": float(ang),
                         "q_drift_rad": float(np.abs(q_isaac - Q[S["k"]]).max())})
    S["k"] += 1
    if S["k"] >= a.n: return
    q = art.get_dof_positions().numpy()[0].astype(np.float32); q[S["a2d"]] = Q[S["k"]]
    z = np.zeros_like(q); arr = lambda v: wp.array(v[None], dtype=wp.float32, device=S["dev"])
    art.set_dof_positions(arr(q), S["idx"]); art.set_dof_velocities(arr(z), S["idx"]); art.set_dof_position_targets(arr(q), S["idx"])
    S["wait"] = 1
SimulationManager.setup_simulation(dt=1.0 / 200.0, device="cpu")
cb = SimulationManager.register_callback(on_pre_step, event=SimulationEvent.PHYSICS_PRE_STEP)
tl = omni.timeline.get_timeline_interface(); tl.play()
t0 = time.time()
while len(S["res"]) < a.n and time.time() - t0 < 300: app.update()
pe = [r["pos_err_mm"] for r in S["res"]]; re_ = [r["rot_err_deg"] for r in S["res"]]
out = {**S.get("info", {}), "fk_samples": len(pe), "fk_pos_err_mm": {"median": float(np.median(pe)), "max": float(np.max(pe))},
       "fk_rot_err_deg": {"median": float(np.median(re_)), "max": float(np.max(re_))},
       "q_drift_rad_max": float(max(r["q_drift_rad"] for r in S["res"])), "samples": S["res"]}
json.dump(out, open(os.path.join(a.out, "fr3_check.json"), "w"), indent=1)
print("[fr3] FK PhysX vs cuRobo fr3_hand:", out["fk_pos_err_mm"], "mm,", out["fk_rot_err_deg"], "deg; q drift", out["q_drift_rad_max"], flush=True)
tl.stop(); app.close()
