# Stream the workcell's virtual ZED X to the ZED SDK (Stereolabs extension v5.1.0 for Isaac Sim 6.0), paced to real time
# for --seconds, and publish the Franka's /joint_states over ROS 2 Humble (Isaac's bundled rclpy; ROS=1 in run_isaac.sh).
# --arm hold: the Franka stays at retract; --arm sweep: a slow swing around retract (>= 0.17 m from every object, checked
# with cuRobo spheres) to test the robot self-filter. A ZED SDK client (zed_lite: zed_wrapper sim_mode) receives the stream.
#   ROS=1 IMAGE=spot-isaac-sim:6.0.0-zed ./run_isaac.sh -s zed_stream.py zed_stream --seconds 90 [--arm sweep]
import argparse, json, os, time
HERE = os.path.dirname(os.path.abspath(__file__))
ap = argparse.ArgumentParser()
ap.add_argument("--out", required=True)
ap.add_argument("--stage", default=os.path.join(HERE, "franka_workcell.usda"))
ap.add_argument("--ext-dir", default="/workspace/zed-isaac-sim-5.1.0/IsaacSimZED@110.1.1+isaac-sim-600.0.13b03659.local.manylinux_2_35_x86_64.release/exts")
ap.add_argument("--zed", default="/World/zed")
ap.add_argument("--resolution", default="SVGA")
ap.add_argument("--fps", type=int, default=30)
ap.add_argument("--port", type=int, default=30000)
ap.add_argument("--transport", default="BOTH", choices=["IPC", "NETWORK", "BOTH"])
ap.add_argument("--seconds", type=float, default=90.0)
ap.add_argument("--arm", choices=["hold", "sweep"], default="hold")
ap.add_argument("--js-every", type=int, default=2, help="publish /joint_states every N physics steps (200 Hz / N)")
ap.add_argument("--no-joint-states", action="store_true")
a = ap.parse_args(); os.makedirs(a.out, exist_ok=True)

from isaacsim import SimulationApp
app = SimulationApp({"headless": True, "renderer": "RayTracedLighting", "multi_gpu": False})
import numpy as np, omni.usd, omni.timeline, omni.kit.app, warp as wp
from pxr import Sdf, Usd, UsdPhysics
from isaacsim.core.utils.stage import is_stage_loading
from isaacsim.core.simulation_manager import SimulationManager, SimulationEvent

em = omni.kit.app.get_app().get_extension_manager(); em.add_path(a.ext_dir)
assert em.set_extension_enabled_immediate("sl.sensor.camera", True), "ZED extension failed to enable"
from sl.sensor.camera.annotators import ZEDAnnotator

ctx = omni.usd.get_context(); ctx.open_stage(a.stage); stage = ctx.get_stage()
for i in range(3000):
    app.update()
    if i > 60 and not is_stage_loading(): break
fr = stage.GetPrimAtPath("/World/franka")
art_root = next((str(p.GetPath()) for p in Usd.PrimRange(fr) if p.HasAPI(UsdPhysics.ArticulationRootAPI)), None)
print("[zed] stage", a.stage, "| franka articulation", art_root, flush=True)

RETRACT = [0.0, -1.3, 0.0, -2.5, 0.0, 1.0, 0.0]
AMP, PER = np.array([0.8, 0.25, 0.0, 0.35, 0.0, 0.0, 0.0]), np.array([14.0, 9.0, 1.0, 11.0, 1.0, 1.0, 1.0])   # sweep (s)
S = {"art": None, "n": 0, "t_sweep0": None}
pub = None
if not a.no_joint_states:
    import rclpy                                     # Isaac 6.0's bundled ROS 2 Humble rclpy (Python 3.12)
    from sensor_msgs.msg import JointState as JointStateMsg
    rclpy.init(); ros_node = rclpy.create_node("isaac_franka"); pub = ros_node.create_publisher(JointStateMsg, "/joint_states", 10)
def on_pre_step(dt, context):                        # retract, optionally swept; publish joint states
    S["n"] += 1
    if S["art"] is not None:
        art, t = S["art"], SimulationManager.get_simulation_time()
        if a.arm == "sweep" and S["t_sweep0"] is not None:
            ts = t - S["t_sweep0"]; ramp = min(1.0, ts / 3.0)   # 3 s fade-in
            S["tgt"][S["a2d"]] = np.array(RETRACT) + ramp * AMP * np.sin(2 * np.pi * ts / PER)
            art.set_dof_position_targets(wp.array(S["tgt"][None], dtype=wp.float32, device=S["dev"]), S["idx"])
        if pub is not None and S["n"] % a.js_every == 0:
            q = art.get_dof_positions().numpy()[0]; qd = art.get_dof_velocities().numpy()[0]
            m = JointStateMsg(); sec = int(t); m.header.stamp.sec, m.header.stamp.nanosec = sec, int((t - sec) * 1e9)
            m.name = S["names"]; m.position = [float(v) for v in q]; m.velocity = [float(v) for v in qd]; pub.publish(m)
        return
    if art_root is None: return
    view = SimulationManager.get_physics_simulation_view()
    if view is None: return
    art = view.create_articulation_view(art_root); names = list(art.shared_metatype.dof_names)
    dev = art.get_dof_positions().device; idx = wp.array([0], dtype=wp.uint32, device=dev)
    q = art.get_dof_positions().numpy()[0].astype(np.float32).copy()
    pre = next(n for n in names if n.endswith("_joint1"))[:-len("joint1")]   # "fr3_" (lab arm) or "panda_"
    for j, v in enumerate(RETRACT): q[names.index(f"{pre}joint{j + 1}")] = v
    for n in names:
        if "finger" in n: q[names.index(n)] = 0.04
    art.set_dof_positions(wp.array(q[None], dtype=wp.float32, device=dev), idx)
    art.set_dof_velocities(wp.array(np.zeros_like(q)[None], dtype=wp.float32, device=dev), idx)
    art.set_dof_position_targets(wp.array(q[None], dtype=wp.float32, device=dev), idx)
    S.update(art=art, dev=dev, idx=idx, tgt=q.copy(), names=names, a2d=np.array([names.index(f"{pre}joint{j + 1}") for j in range(7)]))
SimulationManager.setup_simulation(dt=1.0 / 200.0, device="cpu")
cb = SimulationManager.register_callback(on_pre_step, event=SimulationEvent.PHYSICS_PRE_STEP)

import glob                                          # a receiver that died while connected leaves a held lock in the
for f in glob.glob(f"/dev/shm/sl_local_*_{a.port}"):  # IPC segments (--ipc host): the next receiver then waits forever
    os.remove(f); print("[zed] removed stale", f, flush=True)
tl = omni.timeline.get_timeline_interface(); tl.play()
for _ in range(10): app.update()
zed = ZEDAnnotator([Sdf.Path(a.zed)], camera_model="ZED_X", streaming_port=a.port, resolution=a.resolution, fps=a.fps,
                   transport_layer_mode=a.transport)
print(f"[zed] streaming {a.zed} as ZED_X {a.resolution} @ {a.fps} fps, port {a.port}, transport {a.transport}", flush=True)
S["t_sweep0"] = SimulationManager.get_simulation_time()
print(f"[zed] arm {a.arm} | /joint_states {'off' if pub is None else f'at {200 / a.js_every:g} Hz, stamped with sim time'}", flush=True)

t0, n0, updates = time.time(), S["n"], 0
while time.time() - t0 < a.seconds:
    app.update(); updates += 1
    lead = (S["n"] - n0) / 200.0 - (time.time() - t0)  # keep sim time at wall time
    if lead > 0: time.sleep(lead)
    if updates % 250 == 0:                           # clock log: lets us check the ZED images use the same time base
        print(f"[zed] clock sim {SimulationManager.get_simulation_time():.3f} wall {time.time():.3f}", flush=True)
wall = time.time() - t0
res = {"stage": a.stage, "zed": a.zed, "resolution": a.resolution, "fps": a.fps, "port": a.port, "transport": a.transport,
       "wall_s": round(wall, 1), "sim_s": round((S["n"] - n0) / 200.0, 1), "app_updates_per_s": round(updates / wall, 1)}
json.dump(res, open(os.path.join(a.out, "RESULT.json"), "w"), indent=1)
print("[zed] RESULT " + json.dumps(res), flush=True)
try: zed.destroy()
except Exception as e: print("[zed] destroy:", e, flush=True)
os.system(f"chown -R 1000:1000 {a.out}")
tl.stop(); import sys; sys.stdout.flush(); os._exit(0)
