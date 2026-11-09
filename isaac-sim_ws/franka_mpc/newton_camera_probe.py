"""Feasibility probe: Newton's SensorTiledCamera (warp raytracer) on the valve scene at the REAL calibrated ZED poses (real_cams.json): RGB, z-depth, per-shape segmentation,
timing, and a geometric check of the depth against the analytic z of a known 3D point. Replays one frame of a valve_mpc trace so the arm is where it was.
   PYTHONPATH=<crawl-lab>/IsaacLab/.venv/lib/python3.12/site-packages $PY newton_camera_probe.py runs/valve_mpc5/ep04/trace.npz 9.6 runs/newton_cam_probe"""
import json, os, sys, time
import numpy as np, warp as wp
import newton
from scipy.spatial.transform import Rotation as Rot
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__))); import valve_geometry as vg
trace, tsel, out = sys.argv[1], float(sys.argv[2]), sys.argv[3]; os.makedirs(out, exist_ok=True)
HERE = os.path.dirname(os.path.abspath(__file__)); W, H = 1280, 720
cam0 = json.load(open(os.path.join(HERE, "runs", "workcell", "camera.json"))); VFOV = np.radians(cam0["vfov_deg"]); fx = cam0["fx"]; fy = cam0["fy"]
real = json.load(open(os.path.join(HERE, "real_cams.json")))
cfg = newton.ModelBuilder.ShapeConfig(mu=0.8)
b = newton.ModelBuilder(); b.default_shape_cfg = cfg
import glob, pathlib
_ca = sorted(glob.glob(os.path.expanduser("~/.cache/newton/newton-assets_franka_emika_panda_*/franka_emika_panda")))
PANDA = pathlib.Path(_ca[-1]) if _ca else newton.utils.download_asset("franka_emika_panda")
b.add_urdf(PANDA / "urdf/fr3_franka_hand.urdf", xform=wp.transform((0.0, 0.0, 0.0), wp.quat_identity()), enable_self_collisions=False, parse_visuals_as_colliders=False)
qv = Rot.from_euler("z", vg.YAW, degrees=True).as_quat()
b.add_urdf(os.path.join(vg.VALVE_DIR, "Ball_Valve.urdf"), xform=wp.transform(tuple(vg.POS), tuple(qv)), enable_self_collisions=False)
b.add_urdf(os.path.join(vg.HB, "Honeycomb", "Honeycomb_Panel.urdf"), xform=wp.transform(tuple(vg.to_base(vg.PANEL_IN_VALVE)), tuple(qv)), enable_self_collisions=False)
b.add_shape_box(body=-1, hx=0.8, hy=0.8, hz=0.05, xform=wp.transform(wp.vec3(0.3, 0.0, -0.05), wp.quat_identity()), cfg=cfg)
model = b.finalize(); st = model.state()
T = np.load(trace); i = int(np.argmin(abs((T["t"] - T["t"][0]) - tsel)))
q = model.joint_q.numpy().copy(); q[:7] = T["q"][i]; q[7:9] = T["q_fingers"][i]; q[9] = np.radians(T["lever_deg"][i])
st.joint_q.assign(q); newton.eval_fk(model, st.joint_q, model.joint_qd, st)
sensor = newton.sensors.SensorTiledCamera(model)
fov_v = 2 * np.arctan(H / 2 / fy)
rays = sensor.utils.compute_camera_rays_pinhole(W, H, camera_fovs=float(fov_v))
color = sensor.utils.create_color_image_output(W, H); depth = sensor.utils.create_depth_image_output(W, H); shape = sensor.utils.create_shape_index_image_output(W, H)
res = {}
for name in ("zed", "zed2"):
    Tbc = np.array(real[name]["T_base_cam"], np.float64)                      # OpenCV optical frame (x right, y down, z forward) in the base frame
    for conv in ("opencv_as_is", "opengl_flip"):
        Tc = Tbc.copy()
        if conv == "opengl_flip": Tc[:3, :3] = Tbc[:3, :3] @ np.diag([1.0, -1.0, -1.0])      # OpenGL camera: x right, y up, looks along -z
        qx = Rot.from_matrix(Tc[:3, :3]).as_quat()
        tf = wp.array([[wp.transformf(wp.vec3(*Tc[:3, 3]), wp.quatf(*qx))]], dtype=wp.transformf)
        model.bvh_refit_shapes(st)
        sensor.update(st, tf, rays, color_image=color, depth_image=depth, shape_index_image=shape); wp.synchronize()
        t0 = time.time()
        for _ in range(10): sensor.update(st, tf, rays, color_image=color, depth_image=depth, shape_index_image=shape)
        wp.synchronize(); ms = (time.time() - t0) / 10 * 1000
        rgb = sensor.utils.flatten_color_image_to_rgba(color).numpy()[..., :3] if hasattr(sensor.utils, "flatten_color_image_to_rgba") else color.numpy()
        d = depth.numpy()[0, 0]; sh = shape.numpy()[0, 0]
        res[f"{name}_{conv}"] = {"ms_per_frame": round(ms, 1), "valid_frac": round(float((d > 0).mean()), 3), "depth_min_med_max": [round(float(d[d > 0].min()), 3), round(float(np.median(d[d > 0])), 3), round(float(d[d > 0].max()), 3)] if (d > 0).any() else None, "n_shape_ids": int(len(np.unique(sh)))}
        from PIL import Image
        Image.fromarray(np.asarray(rgb).reshape(H, W, -1)[..., :3].astype(np.uint8)).save(os.path.join(out, f"{name}_{conv}_rgb.jpg"), quality=90)
        np.save(os.path.join(out, f"{name}_{conv}_depth.npy"), d)
        print(name, conv, res[f"{name}_{conv}"], flush=True)
json.dump(res, open(os.path.join(out, "probe.json"), "w"), indent=1)
