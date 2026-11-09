"""Render a run's trace.npz (q, q_fingers, obj per 20 ms control step) as an mp4 with Newton's headless GL viewer: the FR3 + hand + table + ALL workcell objects
(static meshes from runs/newton_scene/objects_start.npz, the drill follows trace 'obj'), overlay = sim time, node loop_ms / Hz from mpc_node.csv, arm-to-object notes none.
   run_render_trace.sh runs/<run> [--every 2] [--out video.mp4]   (inside the Isaac image with the host Newton, as run_newton_selftest.sh)"""
import argparse, csv, json, os, pathlib, glob
import numpy as np, warp as wp
import newton

ap = argparse.ArgumentParser(); ap.add_argument("run"); ap.add_argument("--every", type=int, default=2, help="render every Nth control step (20 ms): 2 = 25 fps = real time")
ap.add_argument("--out", default=""); ap.add_argument("--w", type=int, default=1280); ap.add_argument("--h", type=int, default=720); ap.add_argument("--title", default="")
ap.add_argument("--max-frames", type=int, default=0); ap.add_argument("--rtx", action="store_true", help="ViewerRTX (OVRTX ray tracing) instead of the GL viewer"); ap.add_argument("--out-suffix", default="")
a = ap.parse_args(); HERE = os.path.dirname(os.path.abspath(__file__)); run = os.path.abspath(a.run); out = a.out or os.path.join(run, "video_rtx.mp4" if a.rtx else "video.mp4")
T = np.load(os.path.join(run, "trace.npz")); q, qf, obj, t = T["q"], T["q_fingers"], T["obj"], T["t"]; n = len(q)
root = np.array(json.load(open(os.path.join(HERE, "runs", "workcell", "scene.json")))["franka_base"])
objs = np.load(os.path.join(HERE, "runs", "newton_scene", "objects_start.npz"))
loop = []                                                                            # (step, loop_ms) from the node CSV (1 row per 10 control steps)
try:
    for r in csv.DictReader(open(os.path.join(run, "mpc_node.csv"))):
        if r["loop_ms"] and r["step"]: loop.append((int(float(r["step"])), float(r["loop_ms"])))
except Exception: pass

b = newton.ModelBuilder()
_ca = sorted(glob.glob(os.path.expanduser("~/.cache/newton/newton-assets_franka_emika_panda_*/franka_emika_panda")))
PANDA = pathlib.Path(_ca[-1]) if _ca else newton.utils.download_asset("franka_emika_panda")
b.add_urdf(PANDA / "urdf/fr3_franka_hand.urdf", xform=wp.transform((0.0, 0.0, 0.0), wp.quat_identity()), enable_self_collisions=False, parse_visuals_as_colliders=False)
b.add_shape_box(body=-1, hx=0.8, hy=0.8, hz=0.05, xform=wp.transform(wp.vec3(0.3, 0.0, -0.05), wp.quat_identity()))
drill_body = None
for name in objs.files:
    tri = objs[name].astype(np.float64).reshape(-1, 3) - root
    V, I = np.unique(np.round(tri, 5), axis=0, return_inverse=True); V = V.astype(np.float32); I = I.astype(np.int32).reshape(-1)
    if name == "drill":
        c = tri.mean(0); drill_body = b.add_body(xform=wp.transform(wp.vec3(*c), wp.quat_identity()), label="drill"); drill_c = c.copy()
        b.add_shape_mesh(body=drill_body, mesh=newton.Mesh(V - c.astype(np.float32), I, compute_inertia=False))
    else:
        b.add_shape_mesh(body=-1, mesh=newton.Mesh(V, I, compute_inertia=False))
if not a.rtx: b.add_ground_plane()                                              # RTX: the table top is at z=0 too -> z-fighting speckle; RTX has its own background
model = b.finalize(); st = model.state()
nq = len(model.joint_q.numpy())
import pyglet; pyglet.options["headless"] = True
import imageio.v2 as imageio
from PIL import Image, ImageDraw
if a.rtx:
    from newton.viewer import ViewerRTX
    viewer = ViewerRTX(width=a.w, height=a.h, headless=True, async_rendering=False); viewer.set_model(model)
    def rtx_pixels():                                                          # ovrtx 0.6 names the var '/Render/Vars/LdrColor', newton 1.5 looks for 'LdrColor'
        from ovrtx import Device
        for pr in (viewer._render_products or {}).values():
            for f in pr.frames:
                for k_, rv in f.render_vars.items():
                    if k_.endswith("LdrColor"):
                        with rv.map(device=Device.CPU) as m: return np.array(np.from_dlpack(m), copy=True)
        return None
else:
    viewer = newton.viewer.ViewerGL(width=a.w, height=a.h, headless=True); viewer.set_model(model)
look = wp.vec3(0.38, 0.0, 0.22); viewer.camera.fov = 45.0
viewer.set_camera(pos=wp.vec3(1.25, -1.05, 0.8), pitch=-22.0, yaw=0.0); viewer.camera.look_at(look)
wr = imageio.get_writer(out, fps=50.0 / a.every, codec="libx264", quality=None, pixelformat="yuv420p", macro_block_size=2, output_params=["-crf", "22"])
d0 = obj[0].copy(); frames = range(0, n, a.every)
for k, i in enumerate(frames):
    if a.max_frames and k >= a.max_frames: break
    jq = model.joint_q.numpy().copy(); jq[:7] = q[i]; jq[7:9] = qf[i]; model.joint_q.assign(jq)
    newton.eval_fk(model, model.joint_q, model.joint_qd, st)
    if drill_body is not None:
        bq = st.body_q.numpy(); bq[drill_body] = [*(drill_c + (obj[i] - d0)), 0, 0, 0, 1]; st.body_q.assign(bq)
    viewer.begin_frame(float(t[i] - t[0])); viewer.log_state(st); viewer.end_frame()
    fr = rtx_pixels() if a.rtx else viewer.get_frame().numpy()
    if fr is None: continue                                                    # RTX: the first frames can be empty
    im = Image.fromarray(fr[:, :, :3]); d = ImageDraw.Draw(im); d.rectangle([0, 0, a.w, 26], fill=(0, 0, 0))
    lm = min(loop, key=lambda x: abs(x[0] - i), default=None) if loop else None
    s = f"{a.title or os.path.basename(run)}   sim t {t[i] - t[0]:5.2f} s"
    if lm: s += f"   node loop {lm[1]:5.1f} ms ({1000.0 / lm[1]:4.0f} Hz)"
    d.text((8, 7), s, fill=(255, 255, 255)); wr.append_data(np.asarray(im))
wr.close(); print(f"[render] {out}: {k + 1} frames", flush=True)
