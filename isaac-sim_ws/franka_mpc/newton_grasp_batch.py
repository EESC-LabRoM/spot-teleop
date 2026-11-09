"""Many drill grasps at once in Newton (no Isaac, no ROS, headless): a screen for grasp_variants.py's grasps before the Isaac stack.
One world per (grasp json x jitter): FR3 + Franka Hand (Newton's fr3_franka_hand.urdf, hydroelastic fingers + pads like Newton's
panda_hydro example), the table (top at z = 0 of fr3_link0, as in the workcell), the drill (exact Isaac mesh from objects_start.npz,
CoACD convex decomposition for its collider). Each world follows grasp_goal.py's sequence with batched IK on the hand flange
(fr3_hand): retract -> pregrasp (4 s) -> settle 0.5 -> approach 2.5 -> close 1.0 -> grip 0.8 -> lift 15 cm 3.0 -> hold 1.0.
The jitter moves the DRILL (xy, yaw) while the grasp stays where perception put it: robustness to a perception error.
No MPC and no obstacle avoidance (free-space IK): it checks the grasp physics only. Screen here, confirm in Isaac.
   PY=~/snap/code/current/.local/share/uv/python/cpython-3.12.13-linux-x86_64-gnu/bin/python3.12
   PYTHONPATH=<crawl-lab>/IsaacLab/.venv/lib/python3.12/site-packages $PY newton_grasp_batch.py runs/b1_grasps \
       --objects runs/grasp_full1/objects_start.npz --reps 4 --jitter 0.01 --out runs/nb1
Writes <out>/RESULT.json (per world: grasp, jitter, lift) and prints the table.
"""
import argparse, glob, json, os, time
from dataclasses import replace

import numpy as np
import warp as wp

import newton
import newton.ik as ik
from newton.geometry import HydroelasticSDF

HERE = os.path.dirname(os.path.abspath(__file__))
ap = argparse.ArgumentParser()
ap.add_argument("grasps", help="dir of grasp jsons (grasp_variants.py)")
ap.add_argument("--objects", default=os.path.join(HERE, "runs", "grasp_full1", "objects_start.npz"))
ap.add_argument("--target", default="drill")
ap.add_argument("--reps", type=int, default=4, help="worlds per grasp (rep 0 without jitter)")
ap.add_argument("--jitter", type=float, default=0.01, help="drill xy jitter (m, uniform +-); yaw +-3 deg per 1 cm")
ap.add_argument("--mass", type=float, default=0.18, help="drill mass (kg); Isaac: density 100 on the mesh -> ~0.16-0.21 kg (a real drill ~1.2)")
ap.add_argument("--finger-ke", type=float, default=60000.0, help="finger position gain (N/m); Isaac fr3.usd: 60000, i.e. always at its force cap")
ap.add_argument("--finger-force", type=float, default=100.0, help="finger effort limit (N); Isaac fr3.usd: 100")
ap.add_argument("--mu", type=float, default=0.5, help="friction (Isaac: no physics material bound -> PhysX default 0.5)")
ap.add_argument("--out", required=True)
ap.add_argument("--seed", type=int, default=0)
ap.add_argument("--hulls", type=int, default=32, help="CoACD max convex hulls of the drill (Isaac: PhysX convexDecomposition, 32)")
ap.add_argument("--video", default="", help="mp4: Newton's headless GL view of the jitter-free world of each grasp (tiles), 1x at 15 fps")
ap.add_argument("--video-all", action="store_true", help="film every world (one row per grasp, smaller tiles)")
a = ap.parse_args()
os.makedirs(a.out, exist_ok=True)

RETRACT = [0.0, -1.3, 0.0, -2.5, 0.0, 1.0, 0.0]
T_PRE, T_SETTLE, T_IN, T_CLOSE, T_GRIP, T_LIFT, T_HOLD, LIFT = 4.0, 0.5, 2.5, 1.0, 0.8, 3.0, 1.0, 0.15
FPS, SUB = 60, 10
GJ = sorted(glob.glob(os.path.join(a.grasps, "g*.json"))); G = [json.load(open(f)) for f in GJ]
root = np.array(json.load(open(os.path.join(HERE, "runs", "workcell", "scene.json")))["franka_base"])
tri = np.load(a.objects)[a.target].astype(np.float64).reshape(-1, 3) - root           # base frame (fr3_link0 = world here)
rng = np.random.default_rng(a.seed)
worlds = []                                                                           # (grasp k, dx, dy, dyaw)
for k in range(len(G)):
    for r in range(a.reps):
        if r == 0: worlds.append((k, 0.0, 0.0, 0.0))
        else: worlds.append((k, *rng.uniform(-a.jitter, a.jitter, 2), np.radians(rng.uniform(-3, 3) * a.jitter / 0.01)))
W = len(worlds)

newton.use_coord_layout_targets = True
sdf_res, band = 64, (-0.01, 0.01)
cfg = newton.ModelBuilder.ShapeConfig(kh=1e11, gap=0.01, mu=a.mu, mu_torsional=0.0, mu_rolling=0.0)
cfg_mesh = replace(cfg, is_hydroelastic=True)
b = newton.ModelBuilder(); b.default_shape_cfg = cfg
newton.solvers.SolverMuJoCo.register_custom_attributes(b)
b.add_urdf(newton.utils.download_asset("franka_emika_panda") / "urdf/fr3_franka_hand.urdf", xform=wp.transform((0.0, 0.0, 0.0), wp.quat_identity()),
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
b.joint_target_ke[:9] = [4500, 4500, 3500, 3500, 2000, 2000, 2000, a.finger_ke, a.finger_ke]       # arm: Newton's ik_cube_stacking gains
b.joint_target_kd[:9] = [450, 450, 350, 350, 200, 200, 200, 100, 100]
gc = b.custom_attributes["mujoco:jnt_actgravcomp"]; gc.values = gc.values or {}
for j in range(7): gc.values[j] = True                                               # gravity compensation on the arm joints
gb = b.custom_attributes["mujoco:gravcomp"]; gb.values = gb.values or {}
b.joint_effort_limit[:7] = [87.0] * 4 + [12.0] * 3; b.joint_effort_limit[7:9] = [a.finger_force] * 2
b.joint_armature[:7] = [0.1] * 7; b.joint_armature[7:9] = [0.5] * 2
for bi in range(b.body_count): gb.values[bi] = 1.0                                     # robot links (the drill is added after)
# table: top at z = 0, under the robot base too (the workcell's robot stands on it)
b.add_shape_box(body=-1, hx=0.8, hy=0.8, hz=0.05, xform=wp.transform(wp.vec3(0.3, 0.0, -0.05), wp.quat_identity()), cfg=replace(cfg, sdf_max_resolution=sdf_res, sdf_narrow_band_range=band, is_hydroelastic=True))
ctr = tri.mean(0)
Vw, I = np.unique(np.round(tri - ctr, 6), axis=0, return_inverse=True)                # weld: a triangle soup is one CoACD component per triangle
Vw, I = Vw.astype(np.float32), I.astype(np.int32).reshape(-1)
db = b.add_body(xform=wp.transform(wp.vec3(*ctr), wp.quat_identity()), label="drill")
dcfg = replace(cfg, density=100.0)
cache = os.path.join(os.path.dirname(os.path.abspath(a.objects)), f"{a.target}_coacd{a.hulls}.npz")   # same hulls every run, no 70 s CoACD
t0 = time.time()
if os.path.exists(cache):
    C = np.load(cache); hulls = [C[f"h{i}"] for i in range(int(C["n"]))]
else:                                                                                # CoACD directly: Newton's wrapper ignored max_convex_hull (~1900 hulls)
    import coacd
    parts = coacd.run_coacd(coacd.Mesh(Vw.astype(np.float64), I.reshape(-1, 3)), max_convex_hull=a.hulls, decimate=True, max_ch_vertex=64, seed=0)
    hulls = [np.asarray(v, np.float32) for v, _f in parts]                          # PhysX: hullVertexLimit 64
    np.savez(cache, n=len(hulls), **{f"h{i}": h for i, h in enumerate(hulls)})
t_dec = time.time() - t0
from scipy.spatial import ConvexHull
def hull_faces(Vh):                                                                  # outward-wound triangles (positive volume -> sane inertia)
    H = ConvexHull(Vh); F = H.simplices.copy(); c = Vh.mean(0)
    nrm = np.cross(Vh[F[:, 1]] - Vh[F[:, 0]], Vh[F[:, 2]] - Vh[F[:, 0]]); flip = np.einsum("ij,ij->i", nrm, Vh[F[:, 0]] - c) < 0
    F[flip] = F[flip][:, [0, 2, 1]]; return F.astype(np.int32).reshape(-1)
n_grown = 0
for Vh in hulls:
    try: vol = ConvexHull(Vh).volume
    except Exception: vol = 0.0
    if vol < 1e-6:                                                                   # MuJoCo rejects slivers: thicken flat axes to 1 cm
        c = Vh.mean(0); ext = np.maximum(np.ptp(Vh, 0), 1e-3); Vh = (c + (Vh - c) * np.where(ext < 0.01, 0.01 / ext, 1.0)).astype(np.float32); n_grown += 1
    b.add_shape_convex_hull(body=db, mesh=newton.Mesh(Vh, hull_faces(Vh), compute_inertia=True), cfg=dcfg)
print(f"[newton] drill: {sum(1 for si in range(b.shape_count) if b.shape_body[si] == db)} collision hulls, {n_grown} thickened, {t_dec:.1f} s, {cache}", flush=True)
per_world = b.body_count
model_single = __import__("copy").deepcopy(b).finalize()
scene = newton.ModelBuilder()
for w, (k, dx, dy, dyaw) in enumerate(worlds):
    scene.begin_world(); scene.add_builder(b); scene.end_world()
scene.add_ground_plane(cfg=cfg)
model = scene.finalize()
# drill jitter per world: rotate about the drill's centre (yaw), shift xy; set its mass
q = model.joint_q.numpy(); nq = len(q) // W
bm = model.body_mass.numpy(); dbi = np.array([w * per_world + db for w in range(W)])
for w, (k, dx, dy, dyaw) in enumerate(worlds):
    o = w * nq + 9                                                                   # free joint of the drill: px py pz qx qy qz qw
    q[o + 0] += dx; q[o + 1] += dy; q[o + 3:o + 7] = [0.0, 0.0, np.sin(dyaw / 2), np.cos(dyaw / 2)]
scale = a.mass / bm[dbi[0]]
inv = model.body_inertia.numpy(); inv[dbi] *= scale; bm[dbi] *= scale
model.joint_q.assign(q); model.body_mass.assign(bm); model.body_inertia.assign(inv)
model.body_inv_mass.assign(np.where(bm > 0, 1.0 / np.maximum(bm, 1e-12), 0.0))
model.body_inv_inertia.assign(np.array([np.linalg.inv(x) if np.abs(np.linalg.det(x)) > 1e-18 else np.zeros((3, 3)) for x in inv], np.float32))

s0, s1 = model.state(), model.state(); newton.eval_fk(model, model.joint_q, model.joint_qd, s0)
pipe = newton.CollisionPipeline(model, reduce_contacts=True, broad_phase="explicit", sdf_hydroelastic_config=HydroelasticSDF.Config())
contacts = pipe.contacts()
import mujoco                                                                         # sliver hulls (CoACD) fail MuJoCo's mesh-volume check: shell inertia
_add_mesh = mujoco.MjSpec.add_mesh                                                   # (Newton sets the body inertias itself)
def _add_mesh_shell(self, *args, **kw):
    m = _add_mesh(self, *args, **kw); m.inertia = mujoco.mjtMeshInertia.mjMESH_INERTIA_SHELL; return m
mujoco.MjSpec.add_mesh = _add_mesh_shell
solver = newton.solvers.SolverMuJoCo(model, use_mujoco_contacts=False, disable_sensors=True, solver="newton", integrator="implicitfast",
                                     cone="elliptic", njmax=1500, nconmax=1500, iterations=15, ls_iterations=100, impratio=1000.0)
control = model.control()
# IK: one problem per world on the single-world model (robot dofs only matter)
st = model_single.state(); newton.eval_fk(model_single, model_single.joint_q, model_single.joint_qd, st)
P0 = wp.transform(*st.body_q.numpy()[hand_b])                                        # hand at retract
p_home, q_home = np.array(wp.transform_get_translation(P0)), np.array(wp.transform_get_rotation(P0))   # xyzw
pos_obj = ik.IKObjectivePosition(link_index=hand_b, link_offset=wp.vec3(0.0, 0.0, 0.0), target_positions=wp.array([p_home] * W, dtype=wp.vec3))
rot_obj = ik.IKObjectiveRotation(link_index=hand_b, link_offset_rotation=wp.quat_identity(), target_rotations=wp.array([q_home] * W, dtype=wp.vec4))
lim = ik.IKObjectiveJointLimit(joint_limit_lower=model_single.joint_limit_lower, joint_limit_upper=model_single.joint_limit_upper)
qik = wp.array(np.tile(model_single.joint_q.numpy(), (W, 1)), dtype=wp.float32)
iks = ik.IKSolver(model=model_single, n_problems=W, objectives=[pos_obj, rot_obj, lim], lambda_initial=0.1, jacobian_mode=ik.IKJacobianType.ANALYTIC)

def slerp(q0, q1, s):
    q0, q1 = q0 / np.linalg.norm(q0), q1 / np.linalg.norm(q1); d = float(q0 @ q1)
    if d < 0: q1, d = -q1, -d
    if d > 0.9995: r = q0 + s * (q1 - q0)
    else: th = np.arccos(d); r = (np.sin((1 - s) * th) * q0 + np.sin(s * th) * q1) / np.sin(th)
    return r / np.linalg.norm(r)
mj = lambda s: (lambda s: 10 * s**3 - 15 * s**4 + 6 * s**5)(min(max(s, 0.0), 1.0))
marks = np.cumsum([T_PRE, T_SETTLE, T_IN, T_CLOSE, T_GRIP, T_LIFT, T_HOLD])
def target(g, t):                                                                    # grasp_goal.py's pose_at (no via points)
    pp, qp, pg, qg = np.array(g["pregrasp"][:3]), np.array(g["pregrasp"][3:]), np.array(g["grasp"][:3]), np.array(g["grasp"][3:])
    if t < marks[0]: s = mj(t / T_PRE); return p_home + (pp - p_home) * s, slerp(q_home, qp, s), 0.04
    if t < marks[1]: return pp, qp, 0.04
    if t < marks[2]: s = mj((t - marks[1]) / T_IN); return pp + (pg - pp) * s, qg, 0.04
    if t < marks[3]: return pg, qg, 0.04 * (1 - (t - marks[2]) / T_CLOSE)
    if t < marks[4]: return pg, qg, 0.0
    if t < marks[5]: s = mj((t - marks[4]) / T_LIFT); return pg + np.array([0, 0, LIFT]) * s, qg, 0.0
    return pg + np.array([0, 0, LIFT]), qg, 0.0

VEVERY = 4                                                                            # video: one frame per 4 control frames (15 fps, 1x)
vworlds = list(range(W)) if a.video_all else [w for w, x in enumerate(worlds) if x[1:] == (0.0, 0.0, 0.0)]   # default: the jitter-free world of each grasp
vlog = []                                                                            # (t, body_q of those worlds)
def video_frame(t, st):
    vlog.append((t, st.body_q.numpy().reshape(W, per_world, 7)[vworlds].copy()))
def drill_in_hand(st):                                                               # (W, 7): drill pose in the hand frame, per world
    bq = st.body_q.numpy().reshape(W, per_world, 7); out = []
    for w in range(W):
        h, d = wp.transform(*bq[w, hand_b]), wp.transform(*bq[w, db]); r = wp.transform_multiply(wp.transform_inverse(h), d)
        out.append([*wp.transform_get_translation(r), *wp.transform_get_rotation(r)])
    return np.array(out, np.float64)
rel_grip = None
dt = 1.0 / FPS / SUB; n_frames = int(marks[-1] * FPS); z0 = s0.body_q.numpy()[dbi, 2].copy(); t_run = time.time()
for f in range(n_frames):
    t = f / FPS; T = [target(G[k], t) for (k, *_rest) in worlds]
    pos_obj.set_target_positions(wp.array(np.array([x[0] for x in T], np.float32), dtype=wp.vec3))
    rot_obj.set_target_rotations(wp.array(np.array([x[1] for x in T], np.float32), dtype=wp.vec4))
    iks.step(qik, qik, iterations=24)
    if a.video and f % VEVERY == 0: video_frame(t, s0)
    if f == int(marks[4] * FPS): rel_grip = drill_in_hand(s0)                     # drill pose in the hand when the grip is done
    tq = control.joint_target_q.numpy().reshape(W, -1); qs = qik.numpy()
    tq[:, :7] = qs[:, :7]; tq[:, 7] = tq[:, 8] = [x[2] for x in T]
    control.joint_target_q.assign(tq.reshape(-1))
    for i in range(SUB):
        if i % 2 == 0: pipe.collide(s0, contacts)
        s0.clear_forces(); solver.step(s0, s1, control, contacts, dt); s0, s1 = s1, s0
wp.synchronize(); t_run = time.time() - t_run
if a.video:                                                                           # render after the run, on the one-world model (fast)
    video_frame(marks[-1], s0)
    import pyglet; pyglet.options["headless"] = True
    import imageio.v2 as imageio
    from PIL import Image, ImageDraw
    VW, VH = (320, 240) if a.video_all else (480, 360)
    viewer = newton.viewer.ViewerGL(width=VW, height=VH, headless=True); viewer.set_model(model_single)
    dc = ctr + np.array([0.0, 0.0, 0.05])
    viewer.camera.fov = 45.0; viewer.set_camera(pos=wp.vec3(*(dc + np.array([0.9, -0.9, 0.55]))), pitch=-25.0, yaw=0.0); viewer.camera.look_at(wp.vec3(*dc))
    ncol = a.reps if a.video_all else min(3, len(vworlds)); nrow = -(-len(vworlds) // ncol)
    vw = imageio.get_writer(a.video, fps=FPS / VEVERY, codec="libx264", quality=None, pixelformat="yuv420p", macro_block_size=2, output_params=["-crf", "28"])
    phases = ["pregrasp", "settle", "approach", "close", "grip", "lift", "hold"]
    vs = model_single.state()
    for t, bq in vlog:
        ph = phases[min(int(np.searchsorted(marks, t, side="right")), len(phases) - 1)]; tiles = []
        for i, w in enumerate(vworlds):
            vs.body_q.assign(bq[i]); viewer.begin_frame(t); viewer.log_state(vs); viewer.end_frame()
            im = Image.fromarray(viewer.get_frame().numpy()[:, :, :3]); d = ImageDraw.Draw(im)
            d.rectangle([0, 0, VW, 22], fill=(0, 0, 0))
            d.text((6, 4), f"{os.path.basename(GJ[worlds[w][0]])} w{w}  t {t:4.1f} s  {ph}  drill dz {1e3 * (bq[i][db, 2] - z0[w]):+5.0f} mm", fill=(255, 255, 255))
            tiles.append(np.asarray(im))
        while len(tiles) < nrow * ncol: tiles.append(np.zeros_like(tiles[0]))
        vw.append_data(np.vstack([np.hstack(tiles[r * ncol:(r + 1) * ncol]) for r in range(nrow)]))
    vw.close(); print(f"[newton] video {a.video}: {len(vlog)} frames x {len(vworlds)} worlds", flush=True)
z1 = s0.body_q.numpy()[dbi, 2]; lift = z1 - z0
rel_end = drill_in_hand(s0)
slip_mm = np.linalg.norm(rel_end[:, :3] - rel_grip[:, :3], axis=1) * 1e3              # drill moved in the hand during lift + hold
slip_deg = np.degrees(2 * np.arccos(np.clip(np.abs(np.einsum("ij,ij->i", rel_end[:, 3:], rel_grip[:, 3:])), 0, 1)))
rows = [{"world": w, "grasp": os.path.basename(GJ[k]), "rank": G[k].get("rank"), "dx_mm": round(dx * 1e3, 1), "dy_mm": round(dy * 1e3, 1),
         "dyaw_deg": round(float(np.degrees(dyaw)), 2), "lift_mm": round(float(lift[w]) * 1e3, 1),
         "slip_mm": round(float(slip_mm[w]), 1), "slip_deg": round(float(slip_deg[w]), 1), "success": bool(lift[w] > 0.10)} for w, (k, dx, dy, dyaw) in enumerate(worlds)]
res = {"worlds": W, "sim_s": float(marks[-1]), "wall_s": round(t_run, 1), "robot_s_per_wall_s": round(W * float(marks[-1]) / t_run, 1),
       "coacd_s": round(t_dec, 1), "drill_mass_kg": a.mass, "rows": rows}
rank = []
for k, f in enumerate(GJ):
    r = [x for x in rows if x["grasp"] == os.path.basename(f)]; ok = [x for x in r if x["success"]]
    rank.append({"grasp": os.path.basename(f), "graspgen_index": G[k].get("index"), "flipped": G[k].get("flipped"), "confidence": G[k].get("confidence"),
                 "success_rate": len(ok) / len(r), "median_slip_mm": float(np.median([x["slip_mm"] for x in ok])) if ok else None,
                 "median_slip_deg": float(np.median([x["slip_deg"] for x in ok])) if ok else None,
                 "median_lift_mm": float(np.median([x["lift_mm"] for x in ok])) if ok else None})
rank.sort(key=lambda x: (-x["success_rate"], x["median_slip_mm"] if x["median_slip_mm"] is not None else 1e9))   # held most often, then steadiest
res["ranking"] = rank
json.dump(res, open(os.path.join(a.out, "RESULT.json"), "w"), indent=1)
import csv
with open(os.path.join(a.out, "RANKING.csv"), "w", newline="") as fh:
    wr = csv.DictWriter(fh, fieldnames=list(rank[0])); wr.writeheader(); wr.writerows(rank)
print("rank grasp  GraspGen-X  conf   held   slip mm  slip deg  lift mm")
for i, x in enumerate(rank):
    f_ = lambda v, n=0: "-" if v is None else f"{v:.{n}f}"
    print(f"{i + 1:4d} {x['grasp'][:3]}  #{x['graspgen_index']}{'f' if x['flipped'] else ' '}  {x['confidence']:.3f}  {x['success_rate']:.2f}  {f_(x['median_slip_mm'], 1):>7}  {f_(x['median_slip_deg'], 1):>8}  {f_(x['median_lift_mm']):>7}")
print(f"{W} worlds x {marks[-1]:.1f} s in {t_run:.1f} s wall ({res['robot_s_per_wall_s']} robot-s per s), coacd {t_dec:.1f} s")
