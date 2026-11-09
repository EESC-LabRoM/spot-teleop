"""Newton drop-in for franka_ros_arm.py: the FR3 + Franka Hand, the table and the drill in Newton (MuJoCo-Warp, hydroelastic fingers, Isaac-matched finger/mass/friction),
speaking the SAME ROS 2 interface as the Isaac script, so the cuRobo MPC node (curobo_mpc_node.py, step_on_clock) and grasp_goal.py run UNCHANGED:
  publishes /clock (once per 20 ms control step), /joint_states_isaac (100 Hz), /target_object/pose
  subscribes /joint_command_curobo (pos + vel targets by joint name), /wrist_pose (starts the recording), /gripper_width_cmd (finger target, m)
Lockstep as in Isaac: after each control step wait for the command answering that step's /clock (<= 1 s). Episode loop (--episodes N) with the same file handshake as
franka_ros_arm_session.py: done_<k> -> (orchestrator stops node + goal) -> reset_<k> -> reset -> ready_<k+1>. Per episode <out>/ep<kk>/{RESULT_isaac.json, trace.npz} with the
keys batch_summary.py and the plot scripts read. No rendering, no ZED, no nvblox: the map is served by esdf_replay.py from a saved capture.
Runs in the Isaac image (python 3.12 + Jazzy rclpy) with the host's Newton 1.5 / warp 1.16 site-packages in front on PYTHONPATH (see run_newton_session.sh).
   --selftest: no ROS, drives the arm with a sine and prints the tracking error (gain tuning)."""
import argparse, json, os, sys, time, threading
from dataclasses import replace
import numpy as np
import warp as wp
import newton
from newton.geometry import HydroelasticSDF

HERE = os.path.dirname(os.path.abspath(__file__))
ap = argparse.ArgumentParser()
ap.add_argument("--out", required=True); ap.add_argument("--seconds", type=float, default=14.0, help="sim seconds to record after the first /wrist_pose")
ap.add_argument("--episodes", type=int, default=1); ap.add_argument("--ctl-dir", default=""); ap.add_argument("--wait", type=float, default=300.0)
ap.add_argument("--objects", default=os.path.join(HERE, "runs", "newton_scene", "objects_start.npz")); ap.add_argument("--target", default="drill")
ap.add_argument("--mass", type=float, default=0.18); ap.add_argument("--mu", type=float, default=0.8, help="Newton friction; 0.8 reproduces Isaac(PhysX default 0.5) pass/fail on 11/12 replayed streams (0.65: 10/12, 1.0: 7/12)")
ap.add_argument("--finger-ke", type=float, default=60000.0); ap.add_argument("--finger-force", type=float, default=100.0)
ap.add_argument("--hulls", type=int, default=32)
ap.add_argument("--arm-ke", type=float, nargs=7, default=[30000.0, 30000.0, 24000.0, 24000.0, 12000.0, 12000.0, 12000.0])
ap.add_argument("--arm-kd", type=float, nargs=7, default=[1500.0, 1500.0, 1200.0, 1200.0, 600.0, 600.0, 600.0])
ap.add_argument("--substeps", type=int, default=12, help="physics substeps per 20 ms control step (dt = 20 ms / n)")
ap.add_argument("--free-run", action="store_true", help="do not wait for the node's command each step")
ap.add_argument("--selftest", action="store_true"); ap.add_argument("--replay", default="", help="no ROS: replay q_cmd/qd_cmd of a trace.npz (+ the grasp_goal gripper schedule) through the physics only, print lift/finger width (contact tuning)")
ap.add_argument("--prelude-s", type=float, default=0.0, help="replay: idle at the retract pose this many sim seconds before the trace (the closed loop idles 100-600 s before each episode)"); ap.add_argument("--close-t", type=float, default=7.0, help="replay: gripper close ramp start (s after the first row), 1 s ramp, as grasp_goal.py");  ap.add_argument("--no-graph", action="store_true")
ap.add_argument("--jitters", default="", help="drill perturbation per episode 'dx,dy,dyaw_deg;dx,dy,dyaw_deg;...' (m, m, deg), cycled")
a = ap.parse_args(); a.out = os.path.abspath(a.out); os.makedirs(a.out, exist_ok=True)

RETRACT = [float(x) for x in os.environ["Q0"].split(",")] if os.environ.get("Q0") else [0.0, -0.7853981634, 0.0, -2.3561944902, 0.0, 1.5707963268, 0.7853981634]; TCP = 0.1034; CTRL_DT = 0.02
root = np.array(json.load(open(os.path.join(HERE, "runs", "workcell", "scene.json")))["franka_base"])
tri = np.load(a.objects)[a.target].astype(np.float64).reshape(-1, 3) - root            # base frame (fr3_link0 = world here)
ref = json.load(open(os.path.join(os.path.dirname(os.path.abspath(a.objects)), "object_ref.json")))   # Isaac's rigid-body origin of the target at the start (base frame)

# ---- the model (as newton_grasp_batch.py, one world) -------------------------------------------------------------------------------------------------------
newton.use_coord_layout_targets = True
sdf_res, band = 64, (-0.01, 0.01)
cfg = newton.ModelBuilder.ShapeConfig(kh=1e11, gap=0.01, mu=a.mu, mu_torsional=0.0, mu_rolling=0.0)
b = newton.ModelBuilder(); b.default_shape_cfg = cfg
newton.solvers.SolverMuJoCo.register_custom_attributes(b)
import glob, pathlib
_ca = sorted(glob.glob(os.path.expanduser("~/.cache/newton/newton-assets_franka_emika_panda_*/franka_emika_panda")))      # the cached asset (no git in the container)
PANDA = pathlib.Path(_ca[-1]) if _ca else newton.utils.download_asset("franka_emika_panda")
b.add_urdf(PANDA / "urdf/fr3_franka_hand.urdf", xform=wp.transform((0.0, 0.0, 0.0), wp.quat_identity()),
           enable_self_collisions=False, parse_visuals_as_colliders=True)
find = lambda n: next(i for i, l in enumerate(b.body_label) if l.endswith(f"/{n}"))
hand_b, fl, fr = find("fr3_hand"), find("fr3_leftfinger"), find("fr3_rightfinger")
others = []
for si, bi in enumerate(b.shape_body):
    if bi in (hand_b, fl, fr) and b.shape_type[si] == newton.GeoType.MESH:
        m = b.shape_source[si]
        if m is not None and m.sdf is None:
            sc = np.asarray(b.shape_scale[si], np.float32)
            if not np.allclose(sc, 1.0): m = m.copy(vertices=m.vertices * sc, recompute_inertia=True); b.shape_source[si] = m; b.shape_scale[si] = (1.0, 1.0, 1.0)
            m.build_sdf(max_resolution=sdf_res, narrow_band_range=band, margin=cfg.gap)
        b.shape_flags[si] |= newton.ShapeFlags.HYDROELASTIC
    elif bi not in (hand_b, fl, fr): others.append(si)
b.approximate_meshes(method="convex_hull", shape_indices=others, keep_visual_shapes=True)
b.joint_q[:9] = [*RETRACT, 0.04, 0.04]; b.joint_target_q[:9] = [*RETRACT, 0.04, 0.04]
b.joint_target_ke[:9] = [*a.arm_ke, a.finger_ke, a.finger_ke]; b.joint_target_kd[:9] = [*a.arm_kd, 100.0, 100.0]
for j in range(7): b.joint_target_mode[j] = int(newton.JointTargetMode.POSITION_VELOCITY)           # kp on the position target + kd on the VELOCITY target (feed-forward): no lag from kd * v
gc = b.custom_attributes["mujoco:jnt_actgravcomp"]; gc.values = gc.values or {}
for j in range(7): gc.values[j] = True
gb = b.custom_attributes["mujoco:gravcomp"]; gb.values = gb.values or {}
b.joint_effort_limit[:7] = [87.0] * 4 + [12.0] * 3; b.joint_effort_limit[7:9] = [a.finger_force] * 2
b.joint_armature[:7] = [0.1] * 7; b.joint_armature[7:9] = [0.5] * 2
for bi in range(b.body_count): gb.values[bi] = 1.0
b.add_shape_box(body=-1, hx=0.8, hy=0.8, hz=0.05, xform=wp.transform(wp.vec3(0.3, 0.0, -0.05), wp.quat_identity()),
                cfg=replace(cfg, sdf_max_resolution=sdf_res, sdf_narrow_band_range=band, is_hydroelastic=True))
ctr = tri.mean(0)
Vw, I = np.unique(np.round(tri - ctr, 6), axis=0, return_inverse=True)
Vw, I = Vw.astype(np.float32), I.astype(np.int32).reshape(-1)
db = b.add_body(xform=wp.transform(wp.vec3(*ctr), wp.quat_identity()), label="drill")
dcfg = replace(cfg, density=100.0)
cache = os.path.join(os.path.dirname(os.path.abspath(a.objects)), f"{a.target}_coacd{a.hulls}.npz")
if os.path.exists(cache):
    C = np.load(cache); hulls = [C[f"h{i}"] for i in range(int(C["n"]))]
else:
    import coacd
    parts = coacd.run_coacd(coacd.Mesh(Vw.astype(np.float64), I.reshape(-1, 3)), max_convex_hull=a.hulls, decimate=True, max_ch_vertex=64, seed=0)
    hulls = [np.asarray(v, np.float32) for v, _f in parts]; np.savez(cache, n=len(hulls), **{f"h{i}": h for i, h in enumerate(hulls)})
from scipy.spatial import ConvexHull
def hull_faces(Vh):
    H = ConvexHull(Vh); F = H.simplices.copy(); c = Vh.mean(0)
    nrm = np.cross(Vh[F[:, 1]] - Vh[F[:, 0]], Vh[F[:, 2]] - Vh[F[:, 0]]); flip = np.einsum("ij,ij->i", nrm, Vh[F[:, 0]] - c) < 0
    F[flip] = F[flip][:, [0, 2, 1]]; return F.astype(np.int32).reshape(-1)
for Vh in hulls:
    try: vol = ConvexHull(Vh).volume
    except Exception: vol = 0.0
    if vol < 1e-6:
        c = Vh.mean(0); ext = np.maximum(np.ptp(Vh, 0), 1e-3); Vh = (c + (Vh - c) * np.where(ext < 0.01, 0.01 / ext, 1.0)).astype(np.float32)
    b.add_shape_convex_hull(body=db, mesh=newton.Mesh(Vh, hull_faces(Vh), compute_inertia=True), cfg=dcfg)
b.add_ground_plane(cfg=cfg)
model = b.finalize()
bm = model.body_mass.numpy(); inv = model.body_inertia.numpy(); sc_ = a.mass / bm[db]; inv[db] *= sc_; bm[db] *= sc_
model.body_mass.assign(bm); model.body_inertia.assign(inv); model.body_inv_mass.assign(np.where(bm > 0, 1.0 / np.maximum(bm, 1e-12), 0.0))
model.body_inv_inertia.assign(np.array([np.linalg.inv(x) if np.abs(np.linalg.det(x)) > 1e-18 else np.zeros((3, 3)) for x in inv], np.float32))
s0, s1 = model.state(), model.state(); newton.eval_fk(model, model.joint_q, model.joint_qd, s0)
pipe = newton.CollisionPipeline(model, reduce_contacts=True, broad_phase="explicit", sdf_hydroelastic_config=HydroelasticSDF.Config())
contacts = pipe.contacts()
import mujoco
_add_mesh = mujoco.MjSpec.add_mesh
def _add_mesh_shell(self, *args, **kw):
    m = _add_mesh(self, *args, **kw); m.inertia = mujoco.mjtMeshInertia.mjMESH_INERTIA_SHELL; return m
mujoco.MjSpec.add_mesh = _add_mesh_shell
solver = newton.solvers.SolverMuJoCo(model, use_mujoco_contacts=False, disable_sensors=True, solver="newton", integrator="implicitfast", cone="elliptic",
                                     njmax=1500, nconmax=1500, iterations=15, ls_iterations=100, impratio=1000.0)
control = model.control()
NQ = len(model.joint_q.numpy()); NQD = len(model.joint_qd.numpy())
jq0 = model.joint_q.numpy().copy()                                                         # the episode start state (arm at retract, drill on the table)
DT_SUB = CTRL_DT / a.substeps
JN = ["fr3_joint1", "fr3_joint2", "fr3_joint3", "fr3_joint4", "fr3_joint5", "fr3_joint6", "fr3_joint7", "fr3_finger_joint1", "fr3_finger_joint2"]
print(f"[newton] model: {model.body_count} bodies, {NQ} q, dt {DT_SUB * 1e3:.2f} ms x {a.substeps} per 20 ms; hand body {hand_b}, drill body {db}", flush=True)

def base_state():
    bq = s0.body_q.numpy(); return bq
def drill_origin_base():                                                                    # Isaac's rigid-body origin of the target, in the base frame (the same quantity)
    bq = s0.body_q.numpy()[db]; Tc = wp.transform(*bq); Tc0 = wp.transform(wp.vec3(*ctr), wp.quat_identity())
    To0 = wp.transform(wp.vec3(*ref["p_base"]), wp.quat(*ref["quat_xyzw"]))
    To = wp.transform_multiply(wp.transform_multiply(Tc, wp.transform_inverse(Tc0)), To0)
    return np.array(wp.transform_get_translation(To), np.float64), np.array(wp.transform_get_rotation(To), np.float64)
def quat_xyzw_to_R(x, y, z, w):
    n = x * x + y * y + z * z + w * w; s = 2.0 / n
    return np.array([[1 - s * (y * y + z * z), s * (x * y - z * w), s * (x * z + y * w)], [s * (x * y + z * w), 1 - s * (x * x + z * z), s * (y * z - x * w)],
                     [s * (x * z - y * w), s * (y * z + x * w), 1 - s * (x * x + y * y)]])

T = {"tgt": np.array([*RETRACT, 0.04, 0.04]), "vtgt": np.zeros(9), "n": 0}
def set_targets():
    tq = control.joint_target_q.numpy().copy(); tq[:9] = T["tgt"]; control.joint_target_q.assign(tq)
    tv = control.joint_target_qd.numpy().copy(); tv[:9] = T["vtgt"]; control.joint_target_qd.assign(tv)
HALF = a.substeps // 2; assert HALF % 2 == 0, "--substeps must be a multiple of 4 (CUDA-graphed halves with an even number of state swaps)"
def _block(n):
    global s0, s1
    for i in range(n):
        if i % 2 == 0: pipe.collide(s0, contacts)
        s0.clear_forces(); solver.step(s0, s1, control, contacts, DT_SUB); s0, s1 = s1, s0
GRAPH = None
if wp.get_device().is_cuda and not a.no_graph:
    try:                                                                                      # as Newton's hydroelastic examples: the whole substep block (collide + solve) in one CUDA graph
        _block(HALF)                                                                          # warm-up (kernels compiled, buffers allocated) before the capture
        with wp.ScopedCapture() as _cap: _block(HALF)
        GRAPH = _cap.graph; print("[newton] physics block captured in a CUDA graph", flush=True)
    except Exception as e_: print("[newton] CUDA graph capture failed, running eagerly:", repr(e_)[:200], flush=True); GRAPH = None
def physics(nsub):                                                                            # nsub = a multiple of HALF
    for _ in range(max(1, nsub // HALF)):
        if GRAPH is not None: wp.capture_launch(GRAPH)
        else: _block(HALF)
    T["n"] += nsub
def reset_state(jitter=(0.0, 0.0, 0.0)):
    global s0, s1
    q = jq0.copy(); q[NQ - 7] += jitter[0]; q[NQ - 6] += jitter[1]                           # drill free joint: px py pz qx qy qz qw (last 7 coords)
    yaw = np.radians(jitter[2]); q[NQ - 4:NQ] = [0.0, 0.0, np.sin(yaw / 2), np.cos(yaw / 2)]
    for st in (s0, s1):
        st.joint_q.assign(q); st.joint_qd.assign(np.zeros(NQD, np.float32)); newton.eval_fk(model, st.joint_q, st.joint_qd, st)
    T["tgt"] = np.array([*RETRACT, 0.04, 0.04]); T["vtgt"] = np.zeros(9); set_targets()
    physics(HALF * 10)                                                                       # settle 100 ms

if a.selftest:                                                                               # tracking of a sine on all arm joints, no ROS
    reset_state(); t = 0.0; errs = []; perj = []; t0 = time.time()
    for k in range(250):
        qt = np.array(RETRACT) + 0.25 * np.sin(2 * np.pi * 0.5 * t) * np.array([1, 0.5, 1, 0.5, 1, 0.5, 1]); vt = 0.25 * 2 * np.pi * 0.5 * np.cos(2 * np.pi * 0.5 * t) * np.array([1, 0.5, 1, 0.5, 1, 0.5, 1])
        T["tgt"][:7] = qt; T["vtgt"][:7] = vt; set_targets(); physics(a.substeps); t += CTRL_DT
        tn = t                                                                                # t is already advanced: the target the arm should have reached by the end of this step
        qn = np.array(RETRACT) + 0.25 * np.sin(2 * np.pi * 0.5 * tn) * np.array([1, 0.5, 1, 0.5, 1, 0.5, 1])
        q = s0.joint_q.numpy()[:7]; errs.append(np.abs(q - qn).max()); perj.append(np.abs(q - qn))
    print(f"[selftest] 5 s of a 0.5 Hz sine (0.25 rad): max tracking error {max(errs) * 1000:.1f} mrad, mean {np.mean(errs) * 1000:.1f} mrad, wall {time.time() - t0:.1f} s "
          f"({250 * a.substeps / (time.time() - t0):.0f} substeps/s), q finite {bool(np.isfinite(s0.joint_q.numpy()).all())}", flush=True)
    pj = np.array(perj); print("[selftest] per joint mean |err| mrad:", np.round(pj.mean(0) * 1000, 1).tolist(), "| max:", np.round(pj.max(0) * 1000, 1).tolist(), "| first 5 steps max:", np.round(pj[:5].max(1) * 1000, 1).tolist(), flush=True)
    sys.exit(0)

if a.replay:                                                                                 # open-loop replay of recorded command streams (comma-separated list): seconds each, no MPC node
    for path_ in a.replay.split(","):
        tr = np.load(path_); reset_state(); n = len(tr["t"])
        for _ in range(int(a.prelude_s / CTRL_DT)): set_targets(); physics(a.substeps)
        o0 = drill_origin_base()[0].copy(); t0 = time.time(); zs = []; fw = []; qs = []; hs = []; os_ = []
        close_t = a.close_t; gl = os.path.join(os.path.dirname(path_), "goal.log")                # grasp_goal's clock starts a few rows before the first trace row: align the gripper ramp to it
        if os.path.exists(gl):
            import re
            m_ = [re.search(r"start at sim t=([\d.]+)", l_) for l_ in open(gl)]; m_ = [x for x in m_ if x]
            if m_: close_t = a.close_t - (float(tr["t"][0]) - float(m_[0].group(1)))
        for k in range(n):
            tt = k * CTRL_DT; T["tgt"][:7] = tr["q_cmd"][k]; T["vtgt"][:7] = tr["qd_cmd"][k]
            w = 0.04 * float(np.clip(1.0 - (tt - close_t), 0.0, 1.0)); T["tgt"][7] = T["tgt"][8] = w; set_targets(); physics(a.substeps)
            zs.append(drill_origin_base()[0][2] - o0[2]); fw.append(float(s0.joint_q.numpy()[7:9].mean())); qs.append(s0.joint_q.numpy()[:7].copy()); hs.append(np.array(s0.body_q.numpy()[hand_b][:7], np.float64)); os_.append(drill_origin_base()[0].copy())
        o1 = drill_origin_base()[0]; zs = np.array(zs); fw = np.array(fw)
        np.savez(os.path.join(a.out, f"replay_{os.path.basename(os.path.dirname(path_))}.npz"), lift=zs, fingers=fw, q=np.array(qs), hand=np.array(hs), obj=np.array(os_))
        print(f"[replay] {os.path.basename(os.path.dirname(path_))}: close@{close_t:.2f} lift {zs[-1] * 1000:.1f} mm (max {zs.max() * 1000:.1f}), xy {np.linalg.norm(o1[:2] - o0[:2]) * 1000:.1f} mm, finger at 8/10/12 s "
              f"{[round(float(fw[min(int(x / CTRL_DT), n - 1)]), 4) for x in (8, 10, 12)]}, wall {time.time() - t0:.1f} s | mu {a.mu} ke {a.finger_ke} F {a.finger_force} mass {a.mass}", flush=True)
    sys.exit(0)

# ---- ROS 2 -------------------------------------------------------------------------------------------------------------------------------------------------
import rclpy
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import JointState as JointStateMsg
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float32
rclpy.init(); node = rclpy.create_node("newton_franka")
clock_pub = node.create_publisher(Clock, "/clock", 10); js_pub = node.create_publisher(JointStateMsg, "/joint_states_isaac", 10)
obj_pub = node.create_publisher(PoseStamped, "/target_object/pose", 10)
lock = threading.Lock()
R = {"cmd": None, "cmd_n": 0, "cmd_new": False, "pose": None, "pose_n": 0, "grip": None, "grip_new": False}
def on_cmd(m):
    with lock: R["cmd"], R["cmd_n"], R["cmd_new"] = m, R["cmd_n"] + 1, True
def on_pose(m):
    with lock: R["pose"], R["pose_n"] = m, R["pose_n"] + 1
def on_grip(m):
    with lock: R["grip"], R["grip_new"] = float(min(max(m.data, 0.0), 0.04)), True
node.create_subscription(JointStateMsg, "/joint_command_curobo", on_cmd, 10); node.create_subscription(PoseStamped, "/wrist_pose", on_pose, 10)
node.create_subscription(Float32, "/gripper_width_cmd", on_grip, 10)
threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()
def stamp_s(h): return h.stamp.sec + h.stamp.nanosec * 1e-9
def tstamp(t): return int(t), int(round((t - int(t)) * 1e9)) % 1000000000

S = {"k": 0, "rec": False, "pose_first": None, "cmd_q": np.array(RETRACT), "cmd_qd": np.zeros(7), "cmd_stamp": float("nan"), "cmd_applied_t": float("nan"), "cmd_n": 0}
LOGK = ("t", "q", "qd", "q_cmd", "qd_cmd", "cmd_stamp", "cmd_n", "cmd_applied_t", "target", "target_quat", "target_stamp", "hand", "hand_quat", "tcp", "q_fingers", "obj")
LOG = {k: [] for k in LOGK}
def publish_clock(t):
    c = Clock(); c.clock.sec, c.clock.nanosec = tstamp(t); clock_pub.publish(c)
def publish_js(t):
    q = s0.joint_q.numpy(); qd = s0.joint_qd.numpy()
    js = JointStateMsg(); js.header.stamp.sec, js.header.stamp.nanosec = tstamp(t); js.name = JN; js.position = [float(v) for v in q[:9]]; js.velocity = [float(v) for v in qd[:9]]; js_pub.publish(js)
def apply_commands(t):
    with lock:
        m, new, cn = R["cmd"], R["cmd_new"], R["cmd_n"]; R["cmd_new"] = False
        grip, gnew = R["grip"], R["grip_new"]; R["grip_new"] = False
    if gnew: T["tgt"][7] = T["tgt"][8] = grip
    if new and m is not None:
        for nm, p_, v_ in zip(m.name, m.position, m.velocity if len(m.velocity) else [0.0] * len(m.position)):
            if nm in JN: i_ = JN.index(nm); T["tgt"][i_], T["vtgt"][i_] = p_, v_
        S["cmd_q"], S["cmd_qd"] = T["tgt"][:7].copy(), T["vtgt"][:7].copy(); S["cmd_stamp"], S["cmd_applied_t"], S["cmd_n"] = stamp_s(m.header), t, cn
    if new or gnew: set_targets()
def log_row(t, pose):
    q = s0.joint_q.numpy(); qd = s0.joint_qd.numpy(); bq = s0.body_q.numpy()[hand_b]; hp = np.array(bq[:3], np.float64); hq = np.array(bq[3:7], np.float64)
    o = pose.pose.orientation; p = np.array([pose.pose.position.x, pose.pose.position.y, pose.pose.position.z]); Rh = quat_xyzw_to_R(*hq)
    for k_, v_ in (("t", t), ("q", q[:7]), ("qd", qd[:7]), ("q_cmd", S["cmd_q"]), ("qd_cmd", S["cmd_qd"]), ("cmd_stamp", S["cmd_stamp"]), ("cmd_n", S["cmd_n"]),
                   ("cmd_applied_t", S["cmd_applied_t"]), ("target", p), ("target_quat", [o.x, o.y, o.z, o.w]), ("target_stamp", stamp_s(pose.header)),
                   ("hand", hp), ("hand_quat", hq), ("tcp", hp + Rh @ np.array([0.0, 0.0, TCP])), ("q_fingers", q[7:9]), ("obj", drill_origin_base()[0])):
        LOG[k_].append(np.array(v_, dtype=np.float64))
def lockstep(nb, W):
    if a.free_run: return
    if R["cmd_n"] == 0 and S["pose_first"] is None: return                                 # no node yet: idle, the sim runs ahead of the wall clock
    first = R["cmd_n"] == 0                                                                # a goal is flowing but the node has not answered yet: HOLD the sim (Isaac's rendering slowed it enough to hide this)
    t_ = time.time(); lim = 120.0 if first else 1.0
    while R["cmd_n"] <= nb and time.time() - t_ < lim: time.sleep(0.0002)
    if R["cmd_n"] <= nb: W["timeouts"] += 1
    if S["rec"]: W["wait_ms"].append((time.time() - t_) * 1000.0)
def control_step(W):                                                                         # one 20 ms control step, the Isaac order: /clock, state, apply the last command, physics, wait
    t = S["k"] * CTRL_DT; nb = R["cmd_n"]
    publish_clock(t); publish_js(t)
    apply_commands(t)
    with lock: pose = R["pose"]
    if pose is not None and S["pose_first"] is None: S["pose_first"] = t
    if S["rec"] and pose is not None: log_row(t, pose)
    physics(HALF); publish_js(t + CTRL_DT / 2); physics(HALF)
    p, qo = drill_origin_base(); om = PoseStamped(); om.header.frame_id = "fr3_link0"; om.header.stamp.sec, om.header.stamp.nanosec = tstamp(t)
    om.pose.position.x, om.pose.position.y, om.pose.position.z = (float(v) for v in p); om.pose.orientation.x, om.pose.orientation.y, om.pose.orientation.z, om.pose.orientation.w = (float(v) for v in qo)
    obj_pub.publish(om)
    S["k"] += 1; lockstep(nb, W)
def idle_until(pred, what, timeout):
    t0_ = time.time(); W = {"wait_ms": [], "timeouts": 0}
    while not pred():
        control_step(W); time.sleep(CTRL_DT * 0.5)                                           # paced a bit: the node starts/stops meanwhile
        if time.time() - t0_ > timeout: print(f"[newton] ERROR: timeout waiting for {what}", flush=True); os._exit(4)
def ctl_touch(name): open(os.path.join(a.ctl_dir, name), "w").write(f"{S['k'] * CTRL_DT:.3f}\n")

SESSION = a.episodes > 1
if SESSION: assert a.ctl_dir, "session mode needs --ctl-dir"; os.makedirs(a.ctl_dir, exist_ok=True)
jit = [tuple(float(x) for x in s.split(",")) for s in a.jitters.split(";") if s] or [(0.0, 0.0, 0.0)]
reset_state(jit[0]); P0 = s0.body_q.numpy().copy(); SUMMARY = []
print("[newton] ready", flush=True)
for ep in range(a.episodes):
    odir = os.path.join(a.out, f"ep{ep:02d}") if SESSION else a.out; os.makedirs(odir, exist_ok=True)
    W = {"wait_ms": [], "timeouts": 0}; S["rec"] = False; S["pose_first"] = None
    for k_ in LOG: LOG[k_] = []
    drill_start = s0.body_q.numpy()[db][:3].copy()
    print(f"[newton] episode {ep}/{a.episodes}: holding retract, waiting for /wrist_pose (jitter {jit[ep % len(jit)]})", flush=True)
    wall0 = time.time()
    while S["pose_first"] is None:
        control_step(W); time.sleep(CTRL_DT * 0.25)
        if time.time() - wall0 > a.wait: print("[newton] ERROR: no /wrist_pose", flush=True); os._exit(3)
    t_rec0, wall_rec0, cmd_n0 = S["pose_first"], time.time(), R["cmd_n"]; S["rec"] = True
    print(f"[newton] first /wrist_pose at sim {t_rec0:.2f} s: recording {a.seconds} s", flush=True)
    while S["k"] * CTRL_DT < t_rec0 + a.seconds: control_step(W)
    wall_rec = time.time() - wall_rec0; S["rec"] = False
    Lg = {k: np.stack(v) for k, v in LOG.items()}; drill_end = s0.body_q.numpy()[db][:3]
    moved = {a.target: round(float(np.linalg.norm(drill_end - drill_start)) * 1000, 1)}
    np.savez(os.path.join(odir, "trace.npz"), frame_t=np.array([]), lockstep_wait_ms=np.array(W["wait_ms"]), **Lg)
    res = {"stage": "newton", "record_sim_s": round(float(Lg["t"][-1] - Lg["t"][0]), 2), "record_wall_s": round(wall_rec, 1), "real_time_factor": round(float(Lg["t"][-1] - Lg["t"][0]) / wall_rec, 3),
           "commands_received": int(R["cmd_n"] - cmd_n0), "wrist_poses_received": int(R["pose_n"]), "zeds": "", "object_centre_moved_mm": moved, "lockstep": not a.free_run,
           "lockstep_timeouts": W["timeouts"], "render_every": 0, "paced": False, "video_frames": 0, "jitter": jit[ep % len(jit)], "episode": ep, "physics": "newton 1.5 mujoco_warp"}
    json.dump(res, open(os.path.join(odir, "RESULT_isaac.json"), "w"), indent=1); print("[newton] RESULT " + json.dumps(res), flush=True)
    SUMMARY.append({"episode": ep, "moved_mm": moved})
    if ep == a.episodes - 1: break
    ctl_touch(f"done_{ep}")
    idle_until(lambda: os.path.exists(os.path.join(a.ctl_dir, f"reset_{ep}")), f"reset_{ep}", 600)
    with lock: R["cmd"], R["cmd_n"], R["cmd_new"], R["pose"], R["pose_n"], R["grip"], R["grip_new"] = None, 0, False, None, 0, None, False
    S["cmd_n"] = 0; S["cmd_stamp"] = S["cmd_applied_t"] = float("nan"); S["cmd_q"], S["cmd_qd"] = np.array(RETRACT), np.zeros(7)
    reset_state(jit[(ep + 1) % len(jit)]); Pn = s0.body_q.numpy()
    err = float(np.abs(Pn[:, :3] - P0[:, :3]).max()) * 1000 if jit[(ep + 1) % len(jit)] == (0.0, 0.0, 0.0) else float("nan")
    print(f"[newton] reset {ep + 1}: max body offset from the start state {err:.2f} mm", flush=True); SUMMARY[-1]["next_reset_err_mm"] = round(err, 3)
    ctl_touch(f"ready_{ep + 1}")
if SESSION: json.dump(SUMMARY, open(os.path.join(a.out, "SESSION.json"), "w"), indent=1)
os.system(f"chown -R 1000:1000 {a.out}"); sys.stdout.flush(); os._exit(0)
