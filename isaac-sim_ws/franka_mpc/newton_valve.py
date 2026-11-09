"""The Franka grasps and turns the HiveBoard ball-valve lever in Newton (no Isaac, no ROS, headless), grasps from GraspGen-X.
One world per grasp json (valve_clouds.py -> grasp_generate.py -> grasp_filter.py --no-ik-cuboids -> grasp_variants.py): FR3 + Franka Hand
(hydroelastic fingers, as newton_grasp_batch.py), the table (top at z = 0), the valve from assets/hiveboard (Ball_Valve.urdf: fixed body + lever on a
revolute joint, limits -90..0 deg, lever gravity-compensated and 0.02 N.m joint friction as in the isaaclab-hiveboard task) and the honeycomb panel as
a visual only (no collision, as in HiveBoard's own Isaac Lab scene). Sequence (batched IK on the hand flange, free space, no obstacle avoidance):
retract -> pregrasp (4 s) -> settle 0.5 -> approach 2.5 -> close 1.0 -> grip 0.8 -> TURN the hand about the lever hinge (3.5 s) -> hold 1.0.
Success = the lever turned at least --success-deg while held (read from the lever/body poses, not from the command).
   PY=~/snap/code/current/.local/share/uv/python/cpython-3.12.13-linux-x86_64-gnu/bin/python3.12
   PYTHONPATH=<crawl-lab>/IsaacLab/.venv/lib/python3.12/site-packages $PY newton_valve.py runs/valve_ggx/grasps_top6 --out runs/valve_n1 --video runs/valve_n1/valve.mp4
"""
import argparse, glob, json, os, time
from dataclasses import replace

import numpy as np
import warp as wp
from scipy.spatial.transform import Rotation as Rot

import newton
import newton.ik as ik
from newton.geometry import HydroelasticSDF
import valve_geometry as vg

HERE = os.path.dirname(os.path.abspath(__file__))
ap = argparse.ArgumentParser()
ap.add_argument("grasps", help="dir of grasp jsons (grasp_variants.py)")
ap.add_argument("--turn-deg", type=float, default=-80.0, help="commanded lever angle at the end of the turn (limits -90..0)")
ap.add_argument("--success-deg", type=float, default=60.0, help="success = lever turned at least this much (deg) at the end")
ap.add_argument("--finger-ke", type=float, default=60000.0); ap.add_argument("--finger-force", type=float, default=100.0)
ap.add_argument("--mu", type=float, default=0.8, help="friction on every shape (the value fitted to Isaac's drill pass/fail; not measured for PLA)")
ap.add_argument("--lever-friction", type=float, default=0.02, help="lever joint friction (N.m), isaaclab-hiveboard task value")
ap.add_argument("--reps", type=int, default=1, help="worlds per grasp (rep 0 without jitter; jitter moves nothing here: grasp and valve are exact)")
ap.add_argument("--out", required=True)
ap.add_argument("--video", default="", help="mp4 of the best world (side + close-up tiles)")
ap.add_argument("--video-all", default="", help="mp4 tiling every world (small)")
a = ap.parse_args()
os.makedirs(a.out, exist_ok=True)

RETRACT = [0.0, -1.3, 0.0, -2.5, 0.0, 1.0, 0.0]
T_PRE, T_SETTLE, T_IN, T_CLOSE, T_GRIP, T_TURN, T_HOLD = 4.0, 0.5, 2.5, 1.0, 0.8, 3.5, 1.0
FPS, SUB = 60, 10
GJ = sorted(glob.glob(os.path.join(a.grasps, "g*.json"))); G = [json.load(open(f)) for f in GJ]
worlds = [k for k in range(len(G)) for _ in range(a.reps)]; W = len(worlds)

newton.use_coord_layout_targets = True
sdf_res, band = 64, (-0.01, 0.01)
cfg = newton.ModelBuilder.ShapeConfig(kh=1e11, gap=0.01, mu=a.mu, mu_torsional=0.0, mu_rolling=0.0)
b = newton.ModelBuilder(); b.default_shape_cfg = cfg
newton.solvers.SolverMuJoCo.register_custom_attributes(b)
b.add_urdf(newton.utils.download_asset("franka_emika_panda") / "urdf/fr3_franka_hand.urdf", xform=wp.transform((0.0, 0.0, 0.0), wp.quat_identity()),
           enable_self_collisions=False, parse_visuals_as_colliders=True)
n_robot_dofs = len(b.joint_q)                                                          # 9: 7 arm + 2 fingers
qv = Rot.from_euler("z", vg.YAW, degrees=True).as_quat()                               # valve frame -> base frame (xyzw)
b.add_urdf(os.path.join(vg.VALVE_DIR, "Ball_Valve.urdf"), xform=wp.transform(tuple(vg.POS), tuple(qv)), enable_self_collisions=False)
find = lambda n: next(i for i, l in enumerate(b.body_label) if l.endswith(f"/{n}"))
hand_b, fl, fr = find("fr3_hand"), find("fr3_leftfinger"), find("fr3_rightfinger")
lever_b, vbody_b = find("alavanca_pivot"), find("valvula_esfera")
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
b.approximate_meshes(method="convex_hull", shape_indices=others, keep_visual_shapes=True)   # incl. the valve body and lever (convex hulls)
# the honeycomb panel: visual only (no collision), added after the hull approximation so its hexagon rings stay rings
panel_xf = wp.transform(tuple(vg.to_base(vg.PANEL_IN_VALVE)), tuple(qv))
b.add_urdf(os.path.join(vg.HB, "Honeycomb", "Honeycomb_Panel.urdf"), xform=panel_xf, enable_self_collisions=False)
for si in range(b.shape_count):
    if b.shape_label[si].startswith("Honeycomb") or "Honeycomb" in str(b.shape_label[si]): b.shape_flags[si] &= ~newton.ShapeFlags.COLLIDE_SHAPES
b.joint_q[:9] = [*RETRACT, 0.04, 0.04]; b.joint_target_q[:9] = [*RETRACT, 0.04, 0.04]
b.joint_target_ke[:9] = [4500, 4500, 3500, 3500, 2000, 2000, 2000, a.finger_ke, a.finger_ke]
b.joint_target_kd[:9] = [450, 450, 350, 350, 200, 200, 200, 100, 100]
gc = b.custom_attributes["mujoco:jnt_actgravcomp"]; gc.values = gc.values or {}
for j in range(7): gc.values[j] = True
gb = b.custom_attributes["mujoco:gravcomp"]; gb.values = gb.values or {}
b.joint_effort_limit[:7] = [87.0] * 4 + [12.0] * 3; b.joint_effort_limit[7:9] = [a.finger_force] * 2
b.joint_armature[:7] = [0.1] * 7; b.joint_armature[7:9] = [0.5] * 2
lever_dof = n_robot_dofs                                                              # the valve's revolute joint follows the arm and fingers
for bi in range(b.body_count): gb.values[bi] = 1.0                                     # robot links and the lever (HiveBoard disables the valve's gravity)
b.joint_target_ke[lever_dof] = 0.0; b.joint_target_kd[lever_dof] = 0.0; b.joint_friction[lever_dof] = a.lever_friction
print(f"[newton] valve joint dof {lever_dof}: limits {np.degrees(b.joint_limit_lower[lever_dof]):.0f}..{np.degrees(b.joint_limit_upper[lever_dof]):.0f} deg, friction {a.lever_friction}", flush=True)
b.add_shape_box(body=-1, hx=0.8, hy=0.8, hz=0.05, xform=wp.transform(wp.vec3(0.3, 0.0, -0.05), wp.quat_identity()), cfg=replace(cfg, sdf_max_resolution=sdf_res, sdf_narrow_band_range=band, is_hydroelastic=True))
per_world = b.body_count
model_single = __import__("copy").deepcopy(b).finalize()
scene = newton.ModelBuilder()
for w in range(W): scene.begin_world(); scene.add_builder(b); scene.end_world()
scene.add_ground_plane(cfg=cfg)
model = scene.finalize()
s0, s1 = model.state(), model.state(); newton.eval_fk(model, model.joint_q, model.joint_qd, s0)
pipe = newton.CollisionPipeline(model, reduce_contacts=True, broad_phase="explicit", sdf_hydroelastic_config=HydroelasticSDF.Config())
contacts = pipe.contacts()
solver = newton.solvers.SolverMuJoCo(model, use_mujoco_contacts=False, disable_sensors=True, solver="newton", integrator="implicitfast",
                                     cone="elliptic", njmax=1500, nconmax=1500, iterations=15, ls_iterations=100, impratio=1000.0)
control = model.control()
st = model_single.state(); newton.eval_fk(model_single, model_single.joint_q, model_single.joint_qd, st)
P0 = wp.transform(*st.body_q.numpy()[hand_b])
p_home, q_home = np.array(wp.transform_get_translation(P0)), np.array(wp.transform_get_rotation(P0))
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
marks = np.cumsum([T_PRE, T_SETTLE, T_IN, T_CLOSE, T_GRIP, T_TURN, T_HOLD])
theta_end = np.radians(a.turn_deg)
def turned(pg, qg, th):                                                               # hand pose carried by the lever from 0 to th about the hinge
    c, Rm = vg.rot_about_hinge(th); return c + Rm @ (pg - c), (Rot.from_matrix(Rm) * Rot.from_quat(qg)).as_quat()
def target(g, t):
    pp, qp, pg, qg = np.array(g["pregrasp"][:3]), np.array(g["pregrasp"][3:]), np.array(g["grasp"][:3]), np.array(g["grasp"][3:])
    if t < marks[0]: s = mj(t / T_PRE); return p_home + (pp - p_home) * s, slerp(q_home, qp, s), 0.04
    if t < marks[1]: return pp, qp, 0.04
    if t < marks[2]: s = mj((t - marks[1]) / T_IN); return pp + (pg - pp) * s, qg, 0.04
    if t < marks[3]: return pg, qg, 0.04 * (1 - (t - marks[2]) / T_CLOSE)
    if t < marks[4]: return pg, qg, 0.0
    th = theta_end * mj((t - marks[4]) / T_TURN); p, q = turned(pg, qg, th); return p, q, 0.0

def lever_angle(bq):                                                                  # (W,) lever angle about the hinge axis (valve +x), deg, from the body poses
    out = []
    for w in range(bq.shape[0]):
        Rb = Rot.from_quat(bq[w, vbody_b, 3:7]).as_matrix(); Rl = Rot.from_quat(bq[w, lever_b, 3:7]).as_matrix(); Rr = Rb.T @ Rl
        out.append(np.degrees(np.arctan2(Rr[2, 1], Rr[1, 1])))
    return np.array(out)
def lever_in_hand(bq):
    out = []
    for w in range(bq.shape[0]):
        h, d = wp.transform(*bq[w, hand_b]), wp.transform(*bq[w, lever_b]); r = wp.transform_multiply(wp.transform_inverse(h), d)
        out.append([*wp.transform_get_translation(r), *wp.transform_get_rotation(r)])
    return np.array(out, np.float64)
VEVERY = 4
vlog = []                                                                             # (t, body_q of every world)
rel_grip = None; dt = 1.0 / FPS / SUB; n_frames = int(marks[-1] * FPS); t_run = time.time(); angles = []; fing = []
for f in range(n_frames):
    t = f / FPS; T = [target(G[k], t) for k in worlds]
    pos_obj.set_target_positions(wp.array(np.array([x[0] for x in T], np.float32), dtype=wp.vec3))
    rot_obj.set_target_rotations(wp.array(np.array([x[1] for x in T], np.float32), dtype=wp.vec4))
    iks.step(qik, qik, iterations=24)
    bq = s0.body_q.numpy().reshape(W, per_world, 7)
    if f % VEVERY == 0: vlog.append((t, bq.copy()))
    if f == int(marks[4] * FPS): rel_grip = lever_in_hand(bq)
    if f % 6 == 0: angles.append((t, lever_angle(bq))); fing.append((t, s0.joint_q.numpy().reshape(W, -1)[:, 7:9].mean(1).copy()))
    tq = control.joint_target_q.numpy().reshape(W, -1); qs = qik.numpy()
    tq[:, :7] = qs[:, :7]; tq[:, 7] = tq[:, 8] = [x[2] for x in T]
    control.joint_target_q.assign(tq.reshape(-1))
    for i in range(SUB):
        if i % 2 == 0: pipe.collide(s0, contacts)
        s0.clear_forces(); solver.step(s0, s1, control, contacts, dt); s0, s1 = s1, s0
wp.synchronize(); t_run = time.time() - t_run
bq = s0.body_q.numpy().reshape(W, per_world, 7); vlog.append((marks[-1], bq.copy()))
ang_end = lever_angle(bq); rel_end = lever_in_hand(bq)
slip_mm = np.linalg.norm(rel_end[:, :3] - rel_grip[:, :3], axis=1) * 1e3
slip_deg = np.degrees(2 * np.arccos(np.clip(np.abs(np.einsum("ij,ij->i", rel_end[:, 3:], rel_grip[:, 3:])), 0, 1)))
tt = np.array([x[0] for x in angles]); AA = np.array([x[1] for x in angles])                     # lever angle over time per world
rows = []
for w, k in enumerate(worlds):
    rows.append({"world": w, "grasp": os.path.basename(GJ[k]), "graspgen_index": G[k].get("index"), "flipped": G[k].get("flipped"), "confidence": round(G[k].get("confidence", 0), 3),
                 "lever_deg_end": round(float(ang_end[w]), 1), "finger_mm_end_grip": round(1000 * float(next(v[w] for tt_, v in fing if tt_ >= marks[4] - 0.05)), 1), "finger_mm_end": round(1000 * float(fing[-1][1][w]), 1), "lever_deg_min": round(float(AA[:, w].min()), 1), "slip_mm": round(float(slip_mm[w]), 1), "slip_deg": round(float(slip_deg[w]), 1),
                 "success": bool(ang_end[w] <= -a.success_deg)})
res = {"worlds": W, "sim_s": float(marks[-1]), "wall_s": round(t_run, 1), "mu": a.mu, "turn_deg_commanded": a.turn_deg, "success_deg": a.success_deg, "rows": rows,
       "lever_angle_t": tt.tolist(), "lever_angle_deg": AA.T.round(2).tolist()}
json.dump(res, open(os.path.join(a.out, "RESULT.json"), "w"), indent=1)
print("grasp  GraspGen-X  conf   lever end deg  min deg  slip mm  slip deg  finger mm (end of grip / end)  turned")
for r in rows: print(f"{r['grasp'][:3]}   #{r['graspgen_index']}{'f' if r['flipped'] else ' '}  {r['confidence']:.3f}  {r['lever_deg_end']:9.1f}  {r['lever_deg_min']:9.1f}  {r['slip_mm']:7.1f}  {r['slip_deg']:8.1f}  {r['finger_mm_end_grip']:6.1f} / {r['finger_mm_end']:6.1f}  {r['success']}")
print(f"{W} worlds x {marks[-1]:.1f} s in {t_run:.1f} s wall")

if a.video or a.video_all:
    import pyglet; pyglet.options["headless"] = True
    import imageio.v2 as imageio
    from PIL import Image, ImageDraw
    phases = ["pregrasp", "settle", "approach", "close", "grip", "turn", "hold"]
    vs = model_single.state()
    ctrp = vg.POS + np.array([-0.05, -0.02, 0.0])
    def make_viewer(w_, h_):
        v = newton.viewer.ViewerGL(width=w_, height=h_, headless=True); v.set_model(model_single); return v
    def view(v, eye, at, fov=40.0):
        v.camera.fov = fov; v.set_camera(pos=wp.vec3(*eye), pitch=-20.0, yaw=0.0); v.camera.look_at(wp.vec3(*at))
    def caption(im, text):
        d = ImageDraw.Draw(im); d.rectangle([0, 0, im.width, 22], fill=(0, 0, 0)); d.text((6, 4), text, fill=(255, 255, 255)); return im
    def ph(t): return phases[min(int(np.searchsorted(marks, t, side="right")), len(phases) - 1)]
    if a.video:
        ok = [w for w in range(W) if rows[w]['success']] or list(range(W)); best = min(ok, key=lambda w: slip_mm[w] + 0.01 * abs(ang_end[w] - a.turn_deg))   # turned, and held with the least slip
        VW, VH = 640, 480; vw_ = make_viewer(VW, VH); vc_ = make_viewer(VW, VH)
        view(vw_, vg.POS + np.array([-0.30, -0.62, 0.12]), vg.POS + np.array([-0.12, -0.02, -0.01]), 40.0)      # side view: wrist + valve from the right
        view(vc_, vg.POS + np.array([-0.20, 0.42, 0.30]), vg.POS + np.array([-0.04, -0.02, -0.03]), 36.0)        # from the left and above, down onto the lever
        wr = imageio.get_writer(a.video, fps=FPS / VEVERY, codec="libx264", quality=None, pixelformat="yuv420p", macro_block_size=2, output_params=["-crf", "26"])
        angs = {round(float(x), 3): AA[i, best] for i, x in enumerate(tt)}
        for t, bqs in vlog:
            vs.body_q.assign(bqs[best]); tiles = []
            ang = float(lever_angle(bqs[best:best + 1])[0])
            for v in (vw_, vc_):
                v.begin_frame(t); v.log_state(vs); v.end_frame(); tiles.append(np.asarray(caption(Image.fromarray(v.get_frame().numpy()[:, :, :3]), f"{os.path.basename(GJ[worlds[best]])[:3]} GraspGen-X #{G[worlds[best]].get('index')}  t {t:4.1f} s  {ph(t)}  lever {ang:6.1f} deg")))
            wr.append_data(np.hstack(tiles))
        wr.close(); print(f"[newton] video {a.video}: world {best} ({os.path.basename(GJ[worlds[best]])}), {len(vlog)} frames", flush=True)
    if a.video_all:
        VW, VH = 320, 240; v = make_viewer(VW, VH); view(v, vg.POS + np.array([-0.35, -0.95, 0.30]), vg.POS + np.array([-0.15, 0.0, -0.02]), 42.0)
        ncol = min(3, W); nrow = -(-W // ncol)
        wr = imageio.get_writer(a.video_all, fps=FPS / VEVERY, codec="libx264", quality=None, pixelformat="yuv420p", macro_block_size=2, output_params=["-crf", "28"])
        for t, bqs in vlog:
            tiles = []
            for w in range(W):
                vs.body_q.assign(bqs[w]); v.begin_frame(t); v.log_state(vs); v.end_frame()
                tiles.append(np.asarray(caption(Image.fromarray(v.get_frame().numpy()[:, :, :3]), f"{os.path.basename(GJ[worlds[w]])[:3]} #{G[worlds[w]].get('index')}  t {t:4.1f}  {ph(t)}  lever {float(lever_angle(bqs[w:w + 1])[0]):6.1f}")))
            while len(tiles) < nrow * ncol: tiles.append(np.zeros_like(tiles[0]))
            wr.append_data(np.vstack([np.hstack(tiles[r * ncol:(r + 1) * ncol]) for r in range(nrow)]))
        wr.close(); print(f"[newton] video {a.video_all}", flush=True)
