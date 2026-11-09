# Franka + cuRobo MPC in PhysX (Isaac Sim 6.0): the simple-MPC step of the RA-L Franka port, no perception yet.
# Structure follows ~/spot-isaac-sim/spot_isaac6/walk.py: SimulationManager physics view, control in a PHYSICS_PRE_STEP
# callback, joints mapped by name, hard exit at the end (Kit can hang in app.close()).
# The MPC is set up like curobo_mpc_node.py: MPPI, step_dt 0.02 (50 Hz), the node's mpc_override.yml smoothing, a goal
# updated every control step, and after the first step each solve starts from its own previous command.
#   --mode path (default): the hand follows a smooth closed path at constant speed, the reference moving every step
#                          (like the operator's /wrist_pose stream), with eased start and stop.
#   --mode targets:        four fixed targets held --hold s each (the first, point-to-point test).
#   run through run_isaac.sh (Isaac Sim 6.0 container). Writes <out>/trace.npz, <out>/RESULT.json, <out>/frames/*.jpg.
import argparse, json, math, os, time
HERE = os.path.dirname(os.path.abspath(__file__))
ap = argparse.ArgumentParser()
ap.add_argument("--out", default=os.path.join(HERE, "runs", "path_mpc"))
ap.add_argument("--mode", choices=["path", "targets"], default="path")
ap.add_argument("--hold", type=float, default=3.0, help="targets mode: seconds each target is held")
ap.add_argument("--radius", type=float, default=0.15, help="path mode: circle radius (m)")
ap.add_argument("--wave", type=float, default=0.04, help="path mode: vertical wave amplitude (m), two periods per lap")
ap.add_argument("--period", type=float, default=6.0, help="path mode: seconds per lap at full speed")
ap.add_argument("--laps", type=float, default=2.0)
ap.add_argument("--override", default=os.path.join(HERE, "mpc_override.yml"), help="MPPI overrides ('' = cuRobo defaults)")
ap.add_argument("--fps", type=float, default=25.0, help="video frames per simulated second")
ap.add_argument("--no-video", action="store_true")
ap.add_argument("--clock", action="store_true", help="diagnostic: a green ball sweeping at a known speed in open space, to "
                "check that each video frame shows the state of its own sim time")
a = ap.parse_args(); a.out = os.path.abspath(a.out)

from isaacsim import SimulationApp
app = SimulationApp({"headless": True, "renderer": "RayTracedLighting", "width": 960, "height": 540, "multi_gpu": False})
import numpy as np, omni.usd, omni.timeline, warp as wp, torch
from pxr import Gf, Usd, UsdGeom, UsdLux, UsdPhysics, Vt
from isaacsim.core.simulation_manager import SimulationManager, SimulationEvent
from isaacsim.storage.native import get_assets_root_path

from curobo.geom.sdf.world import CollisionCheckerType
from curobo.geom.types import Cuboid, WorldConfig
from curobo.rollout.rollout_base import Goal
from curobo.types.base import TensorDeviceType
from curobo.types.math import Pose
from curobo.types.state import JointState
from curobo.util_file import get_robot_configs_path, join_path, load_yaml
from curobo.wrap.reacher.mpc import MpcSolver, MpcSolverConfig

DT, CTRL_EVERY = 1.0 / 200.0, 4                      # PhysX at 200 Hz, MPC every 4 physics steps = 50 Hz = step_dt
DOWN = [0.0, 1.0, 0.0, 0.0]                          # hand pointing down (w, x, y, z)
TARGETS = [[0.45, 0.20, 0.35], [0.45, -0.20, 0.35], [0.35, 0.00, 0.55], [0.55, 0.00, 0.20]]
TABLE = dict(pose=[0.5, 0.0, -0.05], dims=[1.2, 1.2, 0.1])   # same box in PhysX and in cuRobo; top at z = 0
CENTER = np.array([0.45, 0.0, 0.35])
TCP = 0.1034                                         # fingertip centre along the hand z axis (franka_ros panda_hand_tcp)
TCP_W = np.array([0.0, 0.0, -TCP])                   # ... in world with the hand pointing DOWN: the marker and the drawn path
                                                     # sit here, because cuRobo's ee frame (panda_hand) is the flange, inside
                                                     # the wrist, so a marker there is hidden whenever the hand catches up
T_APPROACH, T_SETTLE, T_RAMP = 2.0, 0.5, 1.0         # path mode: ease onto the path, settle, speed ramp at both ends
os.makedirs(a.out, exist_ok=True)

def path_point(th):
    return CENTER + np.array([a.radius * math.cos(th), a.radius * math.sin(th), a.wave * math.sin(2 * th)])
W_FULL = 2 * math.pi / a.period
T_LOOP = a.laps * a.period + T_RAMP                  # with half-speed ramps the laps still total a.laps
T_END = (T_APPROACH + T_SETTLE + T_LOOP + 1.0) if a.mode == "path" else a.hold * len(TARGETS)
def min_jerk(s):
    s = min(max(s, 0.0), 1.0); return 10 * s**3 - 15 * s**4 + 6 * s**5
def omega(tl):                                       # angular speed on the path: smooth ramp up, constant, ramp down
    if tl < 0 or tl > T_LOOP: return 0.0
    if tl < T_RAMP: return W_FULL * 0.5 * (1 - math.cos(math.pi * tl / T_RAMP))
    if tl > T_LOOP - T_RAMP: return W_FULL * 0.5 * (1 - math.cos(math.pi * (T_LOOP - tl) / T_RAMP))
    return W_FULL

# ---- stage ---------------------------------------------------------------------------------------------------------
ctx = omni.usd.get_context(); ctx.new_stage(); stage = ctx.get_stage()
UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z); UsdGeom.SetStageMetersPerUnit(stage, 1.0)
import omni.kit.app                                  # one app update = one MPC step = 4 physics steps (Kit's default 60 Hz
omni.kit.app.get_app().get_extension_manager().set_extension_enabled_immediate("isaacsim.core.rendering_manager", True)
from isaacsim.core.rendering_manager import RenderingManager   # gives 3 or 4 steps per update, so video frames landed at
RenderingManager.set_dt(CTRL_EVERY * DT)             # uneven sim times and the motion stuttered); before the render product
UsdGeom.Xform.Define(stage, "/World")
sc = UsdPhysics.Scene.Define(stage, "/World/physicsScene")
sc.CreateGravityDirectionAttr(Gf.Vec3f(0, 0, -1)); sc.CreateGravityMagnitudeAttr(9.81)
tb = UsdGeom.Cube.Define(stage, "/World/table"); tb.CreateSizeAttr(1.0)
tx = UsdGeom.Xformable(tb); tx.AddTranslateOp().Set(Gf.Vec3d(*TABLE["pose"])); tx.AddScaleOp().Set(Gf.Vec3f(*TABLE["dims"]))
UsdPhysics.CollisionAPI.Apply(tb.GetPrim()); tb.CreateDisplayColorAttr([Gf.Vec3f(0.55, 0.47, 0.38)])
UsdLux.DomeLight.Define(stage, "/World/dome").CreateIntensityAttr(900)
sun = UsdLux.DistantLight.Define(stage, "/World/sun"); sun.CreateIntensityAttr(2500)
UsdGeom.Xformable(sun).AddRotateXYZOp().Set(Gf.Vec3f(-45, 0, 35))
franka_usd = get_assets_root_path() + "/Isaac/Robots/FrankaRobotics/FrankaFR3/fr3.usd"   # the lab arm (FR3)
fr = stage.DefinePrim("/World/franka", "Xform"); fr.GetReferences().AddReference(franka_usd)
mk = UsdGeom.Sphere.Define(stage, "/World/target"); mk.CreateRadiusAttr(0.018)   # visual only: no collision API
mk.CreateDisplayColorAttr([Gf.Vec3f(0.85, 0.1, 0.1)]); mk_op = UsdGeom.Xformable(mk).AddTranslateOp()
mk_op.Set(Gf.Vec3d(*((np.array(TARGETS[0]) if a.mode == "targets" else path_point(0.0)) + TCP_W)))
if a.mode == "path":                                 # draw the path as a thin grey curve (visual only)
    pts = [Gf.Vec3f(*(path_point(th) + TCP_W)) for th in np.linspace(0, 2 * math.pi, 181)]
    cv = UsdGeom.BasisCurves.Define(stage, "/World/path"); cv.CreateTypeAttr("linear"); cv.CreateWrapAttr("nonperiodic")
    cv.CreatePointsAttr(Vt.Vec3fArray(pts)); cv.CreateCurveVertexCountsAttr([len(pts)]); cv.CreateWidthsAttr([0.004] * len(pts))
    cv.CreateDisplayColorAttr([Gf.Vec3f(0.35, 0.35, 0.35)])
CLOCK0, CLOCK1, CLOCK_T = np.array([0.95, -0.35, 0.06]), np.array([0.95, 0.45, 0.06]), 2.0   # sweep, repeating every 2 s
def clock_pos(t): return CLOCK0 + (CLOCK1 - CLOCK0) * ((t % CLOCK_T) / CLOCK_T)
if a.clock:
    ck = UsdGeom.Sphere.Define(stage, "/World/clock"); ck.CreateRadiusAttr(0.015)
    ck.CreateDisplayColorAttr([Gf.Vec3f(0.1, 0.8, 0.1)]); ck_op = UsdGeom.Xformable(ck).AddTranslateOp(); ck_op.Set(Gf.Vec3d(*clock_pos(0.0)))
art_root = None
for i_ in range(1200):                                            # the asset streams from the asset server: wait for it
    app.update()
    art_root = next((str(p.GetPath()) for p in Usd.PrimRange(fr) if p.HasAPI(UsdPhysics.ArticulationRootAPI)), None)
    if art_root and i_ >= 30: break
print("[franka] asset", franka_usd, "| articulation root", art_root, flush=True)
if art_root is None:
    print("[franka] ERROR: no articulation root under /World/franka (asset did not load?)", flush=True); os._exit(2)

cam = UsdGeom.Camera.Define(stage, "/World/cam"); cam.CreateFocalLengthAttr(18.0); cam.CreateHorizontalApertureAttr(20.955)
cam.CreateVerticalApertureAttr(11.787); cam.CreateClippingRangeAttr(Gf.Vec2f(0.02, 50.0))
eye, look = np.array([1.30, 1.00, 0.85]), np.array([0.40, 0.0, 0.32])
f_ = look - eye; f_ /= np.linalg.norm(f_); r_ = np.cross(f_, [0, 0, 1.0]); r_ /= np.linalg.norm(r_); u_ = np.cross(r_, f_)
M = np.eye(4); M[:3, 0], M[:3, 1], M[:3, 2], M[:3, 3] = r_, u_, -f_, eye
cam.AddTransformOp().Set(Gf.Matrix4d(*M.T.flatten().tolist()))
if not a.no_video:
    import omni.replicator.core as rep, carb
    from PIL import Image
    carb.settings.get_settings().set("/omni/replicator/captureOnPlay", True)
    rp = rep.create.render_product("/World/cam", (960, 540))
    annot = rep.AnnotatorRegistry.get_annotator("rgb"); annot.attach([rp])
    fdir = os.path.join(a.out, "frames"); os.makedirs(fdir, exist_ok=True)
    for f in os.listdir(fdir): os.remove(os.path.join(fdir, f))

# ---- cuRobo MPC --------------------------------------------------------------------------------------------------------
tdev = TensorDeviceType()
cfg = load_yaml(join_path(get_robot_configs_path(), "fr3.yml"))["robot_cfg"]
world = WorldConfig(cuboid=[Cuboid(name="table", pose=TABLE["pose"] + [1, 0, 0, 0], dims=TABLE["dims"])])
mpc = MpcSolver(MpcSolverConfig.load_from_robot_config(
    cfg, world, use_cuda_graph=True, use_mppi=True, use_lbfgs=False, use_es=False,
    collision_checker_type=CollisionCheckerType.PRIMITIVE, store_rollouts=False, step_dt=CTRL_EVERY * DT,
    self_collision_check=True, override_particle_file=a.override or None))
J = list(mpc.rollout_fn.joint_names)                              # 7 arm joints (fingers are locked in fr3.yml)
retract = mpc.rollout_fn.dynamics_model.retract_config.clone().view(1, -1)
cur = JointState.from_position(retract.clone(), joint_names=J)
def fk(q):
    return mpc.rollout_fn.compute_kinematics(JointState.from_position(tdev.to_device(q).view(1, -1), joint_names=J)).ee_pos_seq.view(-1).cpu().numpy()
P_HOME = fk(retract.view(-1).cpu().numpy())
first = TARGETS[0] if a.mode == "targets" else P_HOME.tolist()
goal = mpc.setup_solve_single(Goal(current_state=cur, goal_state=cur.clone(),
                                   goal_pose=Pose(position=tdev.to_device([first]), quaternion=tdev.to_device([DOWN]))), 1)
mpc.update_goal(goal); mpc.step(cur, max_attempts=2)              # warm-up (CUDA graph capture)
print("[franka] mode", a.mode, "| override", a.override or "(cuRobo defaults)", "| cuRobo joints", J, flush=True)

def reference(t):
    """Return (phase index, reference position) at time t. Phases: 0 approach, 1 settle, 2 path, 3 final hold."""
    if a.mode == "targets":
        ti = min(int(t // a.hold), len(TARGETS) - 1); return ti, np.array(TARGETS[ti])
    if t < T_APPROACH: return 0, P_HOME + (path_point(0.0) - P_HOME) * min_jerk(t / T_APPROACH)
    if t < T_APPROACH + T_SETTLE: return 1, path_point(0.0)
    S["theta"] += omega(t - T_APPROACH - T_SETTLE) * CTRL_EVERY * DT
    return (2 if t < T_APPROACH + T_SETTLE + T_LOOP else 3), path_point(S["theta"])

# ---- controller: runs before every physics step -----------------------------------------------------------------------
S = {"art": None, "n": 0, "ti": -1, "cmd": None, "done": False, "theta": 0.0, "ref": None, "log": {k: [] for k in
     ("t", "ti", "target", "q", "qd", "q_cmd", "qd_cmd", "ee_fk", "ee_sim", "tcp_sim", "fingers_sim", "mpc_ms", "feasible")}}
def on_pre_step(dt, context):
    if S["done"]: return
    if S["art"] is None:
        view = SimulationManager.get_physics_simulation_view()
        if view is None: return
        art = view.create_articulation_view(art_root); meta = art.shared_metatype
        names, links = list(meta.dof_names), list(meta.link_names)
        dev = art.get_dof_positions().device; idx = wp.array([0], dtype=wp.uint32, device=dev)
        a2d = np.array([names.index(j) for j in J]); fing = [i for i, n in enumerate(names) if "finger" in n]
        q0 = art.get_dof_positions().numpy()[0].astype(np.float32).copy()
        q0[a2d] = retract.view(-1).cpu().numpy(); q0[fing] = 0.04
        z = np.zeros_like(q0)
        art.set_dof_positions(wp.array(q0[None], dtype=wp.float32, device=dev), idx)
        art.set_dof_velocities(wp.array(z[None], dtype=wp.float32, device=dev), idx)
        art.set_dof_position_targets(wp.array(q0[None], dtype=wp.float32, device=dev), idx)
        S.update(art=art, dev=dev, idx=idx, a2d=a2d, tgt=q0.copy(), vtgt=z.copy(), hand=links.index("fr3_hand"),
                 fingers=[links.index("fr3_leftfinger"), links.index("fr3_rightfinger")],
                 gains={"stiffness": art.get_dof_stiffnesses().numpy()[0][a2d].tolist(),
                        "damping": art.get_dof_dampings().numpy()[0][a2d].tolist()},
                 dofs=names, root0=art.get_root_transforms().numpy()[0][:3].copy())
        print("[franka] sim dofs", names, "| device", dev, flush=True)
        return
    art = S["art"]; t = S["n"] * DT
    if S["n"] % CTRL_EVERY == 0:
        if t >= T_END: S["done"] = True; return
        ti, ref = reference(t); S["ti"], S["ref"] = ti, ref
        mk_op.Set(Gf.Vec3d(*(ref + TCP_W).tolist()))               # marker = the fingertip reference of this step
        goal.goal_pose.copy_(Pose(position=tdev.to_device([ref.tolist()]), quaternion=tdev.to_device([DOWN])))
        mpc.update_goal(goal)                                       # every step, as the node does with /wrist_pose
        q = art.get_dof_positions().numpy()[0].copy(); qd = art.get_dof_velocities().numpy()[0].copy()
        qa, qda = q[S["a2d"]], qd[S["a2d"]]
        if S["cmd"] is None:                                       # first solve starts from the measured state
            cur.copy_(JointState(position=tdev.to_device(qa).view(1, -1), velocity=tdev.to_device(qda).view(1, -1),
                                 acceleration=tdev.to_device(qda * 0).view(1, -1), jerk=tdev.to_device(qda * 0).view(1, -1),
                                 joint_names=J))
        else:                                                      # then from its own command, as curobo_mpc_node.py does
            cur.copy_(S["cmd"])
        t0 = time.perf_counter(); res = mpc.step(cur, max_attempts=2); torch.cuda.synchronize()
        ms = (time.perf_counter() - t0) * 1000
        S["cmd"] = res.js_action.get_ordered_joint_state(J)
        qc = S["cmd"].position.view(-1).cpu().numpy(); qdc = S["cmd"].velocity.view(-1).cpu().numpy()
        S["tgt"][S["a2d"]] = qc; S["vtgt"][S["a2d"]] = qdc
        art.set_dof_position_targets(wp.array(S["tgt"][None], dtype=wp.float32, device=S["dev"]), S["idx"])
        art.set_dof_velocity_targets(wp.array(S["vtgt"][None], dtype=wp.float32, device=S["dev"]), S["idx"])
        lt = art.get_link_transforms().numpy()[0]; hand_w = lt[S["hand"]][:3] - S["root0"]
        qx, qy, qz, qw = lt[S["hand"]][3:7]                         # PhysX tensor API: xyzw; hand z axis = R[:, 2]
        z_ax = np.array([2 * (qx * qz + qw * qy), 2 * (qy * qz - qw * qx), 1 - 2 * (qx * qx + qy * qy)])
        tcp_w = hand_w + TCP * z_ax; fing_w = lt[S["fingers"]][:, :3].mean(0) - S["root0"]   # fingers: check of the axis
        L = S["log"]
        for k_, v_ in (("t", t), ("ti", ti), ("target", ref), ("q", qa), ("qd", qda), ("q_cmd", qc), ("qd_cmd", qdc),
                       ("ee_fk", fk(qa)), ("ee_sim", hand_w), ("tcp_sim", tcp_w), ("fingers_sim", fing_w), ("mpc_ms", ms), ("feasible", bool(res.metrics.feasible.item()))):
            L[k_].append(np.array(v_, dtype=np.float64))
    S["n"] += 1
    if a.clock: ck_op.Set(Gf.Vec3d(*clock_pos(S["n"] * DT).tolist()))

SimulationManager.setup_simulation(dt=DT, device="cpu")
cb = SimulationManager.register_callback(on_pre_step, event=SimulationEvent.PHYSICS_PRE_STEP)
tl = omni.timeline.get_timeline_interface(); tl.play()
wall0, updates, frames, next_frame, steps_per_update, frame_n = time.time(), 0, 0, 0.0, {}, []
while not S["done"] and updates < 40000:
    n_before = S["n"]
    shoot = not a.no_video and S["art"] is not None and (n_before + CTRL_EVERY) * DT >= next_frame - 1e-9
    if shoot:   # Replicator's synchronous step: advance one update, then wait until that sim time is rendered (a free
        rep.orchestrator.step(rt_subframes=1, pause_timeline=False)   # loop hands back frames 2-3 updates old, varying)
    else:
        app.update()
    updates += 1
    if S["art"] is not None and n_before > 0: steps_per_update[S["n"] - n_before] = steps_per_update.get(S["n"] - n_before, 0) + 1
    if shoot:
        img = annot.get_data()
        if isinstance(img, dict): img = img.get("data")
        if img is not None and getattr(img, "size", 0):
            Image.fromarray(np.asarray(img)[..., :3]).save(os.path.join(fdir, f"{frames:05d}.jpg"), quality=88); frames += 1
            frame_n.append(S["n"])
        next_frame += 1.0 / a.fps
tl.pause()

# ---- results ------------------------------------------------------------------------------------------------------------
L = {k: np.stack(v) for k, v in S["log"].items()}
np.savez(os.path.join(a.out, "trace.npz"), joint_names=np.array(J), frame_n=np.array(frame_n), **L)
d = np.linalg.norm(L["ee_fk"] - L["target"], axis=1)
res = {"mode": a.mode, "override": os.path.basename(a.override) if a.override else None}
if a.mode == "targets":
    per = []
    for i, tg in enumerate(TARGETS):
        m = L["ti"] == i; ts = L["t"][m] - L["t"][m][0]; di = d[m]
        below = np.flatnonzero(di > 0.01)
        per.append({"target_m": tg, "start_distance_cm": round(float(di[0]) * 100, 1), "final_distance_cm": round(float(di[-1]) * 100, 2),
                    "time_to_1cm_s": None if di[-1] > 0.01 else round(float(ts[below[-1] + 1]) if len(below) else 0.0, 3)})
    res["targets"] = per
else:
    on = (L["ti"] == 2) & (L["t"] >= T_APPROACH + T_SETTLE + T_RAMP) & (L["t"] <= T_APPROACH + T_SETTLE + T_LOOP - T_RAMP)
    v_hand = np.linalg.norm(np.gradient(L["ee_fk"], L["t"], axis=0), axis=1)
    res["path"] = {"radius_m": a.radius, "wave_m": a.wave, "period_s": a.period, "laps": a.laps,
                   "reference_speed_mps": {"min": round(a.radius * W_FULL, 3),          # the wave adds speed at its slopes
                                           "max": round(float(np.hypot(a.radius * W_FULL, 2 * a.wave * W_FULL)), 3)},
                   "tracking_err_cm_at_full_speed": {"mean": round(float(d[on].mean()) * 100, 2), "p95": round(float(np.percentile(d[on], 95)) * 100, 2),
                                                     "max": round(float(d[on].max()) * 100, 2)},
                   "hand_speed_mps_at_full_speed": {"mean": round(float(v_hand[on].mean()), 3), "std": round(float(v_hand[on].std()), 3)},
                   "approach_err_cm_at_path_start": round(float(d[L["ti"] == 1][-1]) * 100, 2)}
    dt_ = np.linalg.norm(L["tcp_sim"] - (L["target"] + TCP_W), axis=1)
    res["path"]["fingertip_to_marker_cm_at_full_speed"] = {"mean": round(float(dt_[on].mean()) * 100, 2), "max": round(float(dt_[on].max()) * 100, 2)}
    res["path"]["finger_links_vs_hand_axis_mm"] = round(float(np.abs(np.linalg.norm(L["fingers_sim"] - L["ee_sim"], axis=1) - 0.0584).max() * 1000), 2)
ms_ = L["mpc_ms"][5:]
res.update({"mpc_solve_ms": {"median": round(float(np.median(ms_)), 2), "p95": round(float(np.percentile(ms_, 95)), 2),
            "max": round(float(ms_.max()), 2)}, "all_steps_feasible": bool(L["feasible"].all()),
            "max_joint_tracking_err_deg": round(float(np.degrees(np.abs(L["q_cmd"] - L["q"]).max())), 2),
            "fk_vs_sim_hand_max_mm": round(float(np.linalg.norm(L["ee_fk"] - L["ee_sim"], axis=1).max() * 1000), 4),
            "base_moved_mm": round(float(np.linalg.norm(S["art"].get_root_transforms().numpy()[0][:3] - S["root0"]) * 1000), 2),
            "drive_gains": S.get("gains"), "sim_seconds": round(float(S["n"] * DT), 2), "physics_steps": S["n"],
            "control_steps": int(len(L["t"])), "render_updates": updates, "video_frames": frames,
            "physics_steps_per_update": {str(k): v for k, v in sorted(steps_per_update.items())},
            "physics_steps_between_frames": {str(k): int(v) for k, v in zip(*np.unique(np.diff(frame_n), return_counts=True))} if len(frame_n) > 1 else {},
            "wall_s": round(time.time() - wall0, 1), "asset": franka_usd})
json.dump(res, open(os.path.join(a.out, "RESULT.json"), "w"), indent=1)
print("[franka] RESULT " + json.dumps(res), flush=True)
os.system(f"chown -R 1000:1000 {a.out}")                       # the container runs as root; hand the outputs back
SimulationManager.deregister_callback(cb) if hasattr(SimulationManager, "deregister_callback") else None
tl.stop(); import sys; sys.stdout.flush(); os._exit(0)
