"""Newton render of a closed-loop valve episode (newton_ros_valve.py trace.npz): arm q, fingers and lever angle -> FR3 + valve + panel, two views, captions.
   PYTHONPATH=<crawl-lab>/IsaacLab/.venv/lib/python3.12/site-packages $PY valve_video_from_trace.py runs/valve_mpc1/ep02/trace.npz runs/valve_mpc1/ep02.mp4 [--label g02]"""
import argparse, os, sys
import numpy as np, warp as wp
import newton
from scipy.spatial.transform import Rotation as Rot
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__))); import valve_geometry as vg
ap = argparse.ArgumentParser(); ap.add_argument("trace"); ap.add_argument("out"); ap.add_argument("--label", default=""); ap.add_argument("--fps", type=float, default=15.0); a = ap.parse_args()
T = np.load(a.trace); t = T["t"] - T["t"][0]
cfg = newton.ModelBuilder.ShapeConfig(mu=0.8)
b = newton.ModelBuilder(); b.default_shape_cfg = cfg
import glob, pathlib
_ca = sorted(glob.glob(os.path.expanduser("~/.cache/newton/newton-assets_franka_emika_panda_*/franka_emika_panda")))
PANDA = pathlib.Path(_ca[-1]) if _ca else newton.utils.download_asset("franka_emika_panda")
b.add_urdf(PANDA / "urdf/fr3_franka_hand.urdf", xform=wp.transform((0.0, 0.0, 0.0), wp.quat_identity()), enable_self_collisions=False, parse_visuals_as_colliders=True)
qv = Rot.from_euler("z", vg.YAW, degrees=True).as_quat()
b.add_urdf(os.path.join(vg.VALVE_DIR, "Ball_Valve.urdf"), xform=wp.transform(tuple(vg.POS), tuple(qv)), enable_self_collisions=False)
b.add_urdf(os.path.join(vg.HB, "Honeycomb", "Honeycomb_Panel.urdf"), xform=wp.transform(tuple(vg.to_base(vg.PANEL_IN_VALVE)), tuple(qv)), enable_self_collisions=False)
b.add_shape_box(body=-1, hx=0.8, hy=0.8, hz=0.05, xform=wp.transform(wp.vec3(0.3, 0.0, -0.05), wp.quat_identity()))
model = b.finalize(); st = model.state(); q0 = model.joint_q.numpy().copy()
import pyglet; pyglet.options["headless"] = True
import imageio.v2 as imageio
from PIL import Image, ImageDraw
VW, VH = 640, 480
def mk(eye, at, fov):
    v = newton.viewer.ViewerGL(width=VW, height=VH, headless=True); v.set_model(model); v.camera.fov = fov
    v.set_camera(pos=wp.vec3(*eye), pitch=-20.0, yaw=0.0); v.camera.look_at(wp.vec3(*at)); return v
v1 = mk(vg.POS + np.array([-0.30, -0.62, 0.12]), vg.POS + np.array([-0.12, -0.02, -0.01]), 40.0)
v2 = mk(vg.POS + np.array([-0.20, 0.42, 0.30]), vg.POS + np.array([-0.04, -0.02, -0.03]), 36.0)
wr = imageio.get_writer(a.out, fps=a.fps, codec="libx264", quality=None, pixelformat="yuv420p", macro_block_size=2, output_params=["-crf", "26"])
step = max(1, int(round(1.0 / a.fps / 0.02)))
for i in range(0, len(t), step):
    q = q0.copy(); q[:7] = T["q"][i]; q[7:9] = T["q_fingers"][i]; q[9] = np.radians(T["lever_deg"][i])
    st.joint_q.assign(q); newton.eval_fk(model, st.joint_q, model.joint_qd, st); tiles = []
    err = np.linalg.norm(T["hand"][i] - T["target"][i]) * 1000
    for v in (v1, v2):
        v.begin_frame(float(t[i])); v.log_state(st); v.end_frame(); im = Image.fromarray(v.get_frame().numpy()[:, :, :3]); d = ImageDraw.Draw(im)
        d.rectangle([0, 0, VW, 22], fill=(0, 0, 0)); d.text((6, 4), f"{a.label} cuRobo MPC  t {t[i]:5.1f} s  lever {T['lever_deg'][i]:6.1f} deg  hand-to-goal {err:5.1f} mm  fingers {1000 * T['q_fingers'][i].mean():4.1f} mm", fill=(255, 255, 255))
        tiles.append(np.asarray(im))
    wr.append_data(np.hstack(tiles))
wr.close(); print("video", a.out, len(t) // step, "frames")
