"""Probe (10-29): how real does Newton look through ViewerRTX (OVRTX) vs the GL viewer, and how fast? Same scene as newton_render_trace.py (FR3 + hand, table, the 7 workcell objects as
meshes from objects_start.npz), arm at one frame of a run's trace. Saves rtx_*.png / gl_*.png (overview 1280x720 + a 640x360 'camera' view) and prints per-frame ms.
   run_rtx_probe.sh runs/<run> [--frame 460]"""
import argparse, json, os, pathlib, glob, time
import numpy as np, warp as wp
import newton

ap = argparse.ArgumentParser(); ap.add_argument("run"); ap.add_argument("--frame", type=int, default=460); ap.add_argument("--n", type=int, default=20)
ap.add_argument("--out", default=""); ap.add_argument("--env", default="default", help="RTX lighting preset: default | studio | none")
a = ap.parse_args(); HERE = os.path.dirname(os.path.abspath(__file__)); run = os.path.abspath(a.run); out = a.out or os.path.join(run, "rtx_probe"); os.makedirs(out, exist_ok=True)
T = np.load(os.path.join(run, "trace.npz")); q, qf, obj = T["q"], T["q_fingers"], T["obj"]; i = min(a.frame, len(q) - 1)
root = np.array(json.load(open(os.path.join(HERE, "runs", "workcell", "scene.json")))["franka_base"]); objs = np.load(os.path.join(HERE, "runs", "newton_scene", "objects_start.npz"))
b = newton.ModelBuilder()
_ca = sorted(glob.glob(os.path.expanduser("~/.cache/newton/newton-assets_franka_emika_panda_*/franka_emika_panda")))
PANDA = pathlib.Path(_ca[-1]) if _ca else newton.utils.download_asset("franka_emika_panda")
b.add_urdf(PANDA / "urdf/fr3_franka_hand.urdf", xform=wp.transform((0.0, 0.0, 0.0), wp.quat_identity()), enable_self_collisions=False, parse_visuals_as_colliders=False)
b.add_shape_box(body=-1, hx=0.8, hy=0.8, hz=0.05, xform=wp.transform(wp.vec3(0.3, 0.0, -0.05), wp.quat_identity()))
for name in objs.files:
    tri = objs[name].astype(np.float64).reshape(-1, 3) - root
    V, I = np.unique(np.round(tri, 5), axis=0, return_inverse=True)
    b.add_shape_mesh(body=-1, mesh=newton.Mesh(V.astype(np.float32), I.astype(np.int32).reshape(-1), compute_inertia=False))
model = b.finalize(); st = model.state()
jq = model.joint_q.numpy().copy(); jq[:7] = q[i]; jq[7:9] = qf[i]; model.joint_q.assign(jq); newton.eval_fk(model, model.joint_q, model.joint_qd, st)

CAMS = {"overview": (1280, 720, (1.25, -1.05, 0.8), (0.38, 0.0, 0.22), 45.0),
        "camview": (640, 360, (1.243, 0.206, 0.608), (0.35, 0.0, 0.15), 70.0)}          # camview ~ the 'zed' camera position in the base frame (map_check tf), looking at the table
def shoot(viewer, tag):
    for nm, (w, h, pos, look, fov) in CAMS.items():
        viewer.camera.fov = fov; viewer.set_camera(pos=wp.vec3(*pos), pitch=-20.0, yaw=0.0); viewer.camera.look_at(wp.vec3(*look))
        ts = []
        for k in range(a.n):
            t0 = time.perf_counter(); viewer.begin_frame(0.0); viewer.log_state(st); viewer.end_frame()
            if hasattr(viewer, "get_frame"): fr = viewer.get_frame().numpy()
            ts.append((time.perf_counter() - t0) * 1000)
        if not hasattr(viewer, "get_frame"):                                                       # ViewerRTX: ovrtx 0.6 names the var '/Render/Vars/LdrColor', newton 1.5 looks for 'LdrColor' -> read it here
            from ovrtx import Device
            fr = None
            for pr in (viewer._render_products or {}).values():
                for f in pr.frames:
                    for k_, rv in f.render_vars.items():
                        if k_.endswith("LdrColor"):
                            with rv.map(device=Device.CPU) as m: fr = np.array(np.from_dlpack(m), copy=True)
            if fr is None: print(f"[{tag}] {nm}: no LdrColor in the render products", flush=True)
        if fr is not None:
            from PIL import Image; Image.fromarray(fr[:, :, :3]).save(os.path.join(out, f"{tag}_{nm}.png"))
        print(f"[{tag}] {nm}: first {ts[0]:.0f} ms, median of rest {np.median(ts[1:]):.1f} ms, min {min(ts[1:]):.1f}", flush=True)
import pyglet; pyglet.options["headless"] = True
if os.environ.get("OVRTX_CACHE"):                                                          # persistent OVRTX cache (shaders/materials): ViewerRTX builds RendererConfig() itself, so patch the default
    import ovrtx; _RC = ovrtx.RendererConfig
    def _RC2(*a_, **k_):
        c = _RC(*a_, **k_); c.cache_root_path = os.environ["OVRTX_CACHE"]; return c
    ovrtx.RendererConfig = _RC2
which = os.environ.get("PROBE", "rtx,gl").split(",")
if "rtx" in which:
    from newton.viewer import ViewerRTX
    t0 = time.perf_counter(); v = ViewerRTX(width=1280, height=720, headless=True, environment=a.env, async_rendering=False); v.set_model(model); print(f"[rtx] init {time.perf_counter() - t0:.1f} s", flush=True)
    shoot(v, "rtx")
if "gl" in which:
    v2 = newton.viewer.ViewerGL(width=1280, height=720, headless=True); v2.set_model(model); shoot(v2, "gl")
