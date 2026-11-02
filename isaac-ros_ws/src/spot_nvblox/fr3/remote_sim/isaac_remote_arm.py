# Adapted from Murilo franka_ros_arm.py for the equivalent remote workcell.
# All control/state topics use /fr3_sim in DDS 188; no hardware driver.
# GPU PhysX advances between rendered snapshots; latest JPEG only, no trace/video accumulation.
import argparse, json, math, os, threading, time
HERE = os.path.dirname(os.path.abspath(__file__))
ap = argparse.ArgumentParser()
ap.add_argument("--out", required=True)
ap.add_argument("--physics-hz",type=int,default=60,choices=[60,120,200],help="GPU physics step frequency; 60 Hz avoids wall-time lag for one arm")
ap.add_argument("--arm-drive-damping-scale",type=float,default=1.,help="Simulation-only arm servo damping multiplier; never applies to hardware")
ap.add_argument("--physics-device",default="cuda:0",choices=["cuda:0","cpu"])
ap.add_argument("--stage", default=os.path.join(HERE, "remote_workcell.usda"))
ap.add_argument("--seconds", type=float, default=18.0, help="sim seconds to record after the first /fr3_sim/goal")
ap.add_argument("--wait", type=float, default=900.0, help="wall seconds to wait for the first /fr3_sim/goal")
ap.add_argument("--fps", type=float, default=25.0, help="video frames per simulated second")
ap.add_argument("--view", type=float, nargs=7, default=[0.9, -2.2, 1.8, -0.8, -0.35, 0.8, 14.0],
                help="video camera: eye xyz, look-at xyz (world), focal mm; default frames both ZED stands, robot, table")
ap.add_argument("--video-res", type=int, nargs=2, default=[1280, 720])
ap.add_argument("--no-video", action="store_true")
ap.add_argument("--js-every", type=int, default=2, help="publish /fr3_sim/joint_states every N physics steps (200 Hz / N)")
ap.add_argument("--render-every", type=int, default=1, help="render (app.update) every N 20 ms control steps, physics-only steps in between "
                "(SimulationManager.step); /clock, joint states and object pose come from the physics callback, so they stay at 200/100/50 Hz. "
                "The ZED streams and the video only get a frame on render steps: with N > 1 use --no-video or --fps <= 50/N")
ap.add_argument("--clock-every", type=int, default=1, help="publish /clock every N physics steps (200 Hz / N); 4 = once per 20 ms control "
                "step: a 200 Hz /clock keeps the node's rclpy executor busy (light runs)")
ap.add_argument("--no-pace", action="store_true", help="do not hold the sim to wall time (no ZED streams: the lockstep with the node is the only clock)")
ap.add_argument("--free-run", action="store_true", help="do not wait for the node's command before each 20 ms step")
ap.add_argument("--zeds", default="", help="ZED X cameras to stream, name:port[,name:port]: zed = the workcell's stand in front,"
                " zed2 = a second stand behind the robot (added here), e.g. zed:30000,zed2:30010")
ap.add_argument("--zed-pose", type=float, nargs=6, help="move the workcell's ZED 1 stand: eye xyz, look-at xyz (world)")
ap.add_argument("--zed2-pose", type=float, nargs=6, default=[-1.60, 0.20, 1.60, -0.72, -0.58, 0.42],
                help="ZED 2 stand: eye xyz, look-at xyz (world); default behind-left of the robot, high")
ap.add_argument("--zed-res", default="HD1080")
ap.add_argument("--zed-preview", action="store_true", help="save each ZED's left RGB (zed_<name>_left.jpg) at start")
ap.add_argument("--preview-only", action="store_true", help="exit after the previews")
ap.add_argument("--zed-fps", type=int, default=30)
ap.add_argument("--capture-cams", default="", help="grasp perception: cameras (zed,zed2) whose instance segmentation + depth give each"
                " object's point cloud in the robot base frame -> <out>/clouds/<object>.npz (grasp_capture below); the ZED stands are"
                " placed even without --zeds")
ap.add_argument("--track-object", default="", help="log this object's pose (trace.npz obj) to show a grasp lifting it")
ap.add_argument("--capture-only", action="store_true", help="exit after the capture (no ROS wait, no ZED streams)")
ap.add_argument("--capture-res", type=int, nargs=2, default=[1280, 720])
ap.add_argument("--ext-dir", default="/workspace/zed-isaac-sim-5.1.0/IsaacSimZED@110.1.1+isaac-sim-600.0.13b03659.local.manylinux_2_35_x86_64.release/exts")
a = ap.parse_args();
if not .1 <= a.arm_drive_damping_scale <= 1.:ap.error("arm drive damping scale must be in [0.1,1]")
a.out = os.path.abspath(a.out); os.makedirs(a.out, exist_ok=True)

from isaacsim import SimulationApp
app = SimulationApp({"headless": True, "limit_cpu_threads": 4, "renderer": "RayTracedLighting", "width": a.video_res[0], "height": a.video_res[1], "disable_viewport_updates": True, "anti_aliasing": 2, "multi_gpu": False, "active_gpu": 0, "physics_gpu": 0})
import numpy as np, omni.usd, omni.timeline, omni.kit.app, warp as wp
from pxr import Gf, Usd, UsdGeom, UsdPhysics, Vt
from isaacsim.core.utils.stage import is_stage_loading
from isaacsim.core.simulation_manager import SimulationManager, SimulationEvent

DT, CTRL_EVERY = 1.0 / a.physics_hz, max(1,round(a.physics_hz/50))                      # PhysX 200 Hz; one app update = 4 physics steps = the node's 20 ms
TCP = 0.1034
RETRACT = [0.0, -1.3, 0.0, -2.5, 0.0, 1.0, 0.0]      # franka.yml retract_config (arm joints)
omni.kit.app.get_app().get_extension_manager().set_extension_enabled_immediate("isaacsim.core.rendering_manager", True)
from isaacsim.core.rendering_manager import RenderingManager
RenderingManager.set_dt(CTRL_EVERY * DT)             # before the render product (see franka_mpc_isaac.py)

ZEDS = [(z.split(":")[0], int(z.split(":")[1])) for z in a.zeds.split(",") if z]
if ZEDS:
    em = omni.kit.app.get_app().get_extension_manager(); em.add_path(a.ext_dir)
    assert em.set_extension_enabled_immediate("sl.sensor.camera", True), "ZED extension failed to enable"
CAPCAMS = [c for c in a.capture_cams.split(",") if c]
HAS_Z2 = any(n == "zed2" for n, _ in ZEDS) or "zed2" in CAPCAMS
ctx = omni.usd.get_context(); ctx.open_stage(a.stage); stage = ctx.get_stage()
ZED2_EYE, ZED2_AT = np.array(a.zed2_pose[:3]), np.array(a.zed2_pose[3:])
if a.zed_pose:                                      # re-place ZED 1 (zed_placement.py): rig frame +X forward, +Z up, and its stand
    e1, t1 = np.array(a.zed_pose[:3]), np.array(a.zed_pose[3:])
    f1 = t1 - e1; f1 /= np.linalg.norm(f1); l1 = np.cross([0, 0, 1.0], f1); l1 /= np.linalg.norm(l1)
    M1 = np.eye(4); M1[:3, 0], M1[:3, 1], M1[:3, 2], M1[:3, 3] = f1, l1, np.cross(f1, l1), e1
    z1 = UsdGeom.Xformable(stage.GetPrimAtPath("/World/zed")); z1.ClearXformOpOrder(); z1.AddTransformOp().Set(Gf.Matrix4d(*M1.T.flatten().tolist()))
    pole, foot = UsdGeom.Cylinder(stage.GetPrimAtPath("/World/zed_stand/pole")), stage.GetPrimAtPath("/World/zed_stand/foot")
    pole.GetHeightAttr().Set(float(e1[2] - 0.03))
    UsdGeom.Xformable(pole).GetOrderedXformOps()[0].Set(Gf.Vec3d(float(e1[0]), float(e1[1]), float(e1[2] - 0.03) / 2))
    UsdGeom.Xformable(foot).GetOrderedXformOps()[0].Set(Gf.Vec3d(float(e1[0]), float(e1[1]), 0.01))
    print("[arm] moved /World/zed to", e1.tolist(), "looking at", t1.tolist(), flush=True)
if HAS_Z2:               # a second ZED X on a floor stand behind the Franka (table edge x -1.27),
    zref = stage.GetPrimAtPath("/World/zed").GetPrimStack()   # looking at the faces of the objects turned to the robot
    zusd = next(str(r.assetPath) for spec in zref for r in spec.referenceList.prependedItems)
    zusd = os.path.normpath(os.path.join(os.path.dirname(a.stage), zusd)) if not os.path.isabs(zusd) else zusd
    f2 = ZED2_AT - ZED2_EYE; f2 /= np.linalg.norm(f2); l2 = np.cross([0, 0, 1.0], f2); l2 /= np.linalg.norm(l2); u2 = np.cross(f2, l2)
    M2 = np.eye(4); M2[:3, 0], M2[:3, 1], M2[:3, 2], M2[:3, 3] = f2, l2, u2, ZED2_EYE   # +X forward, +Z up (franka_scene.py)
    z2 = stage.DefinePrim("/World/zed2", "Xform"); z2.GetReferences().AddReference(zusd)
    UsdGeom.Xformable(z2).ClearXformOpOrder(); UsdGeom.Xformable(z2).AddTransformOp().Set(Gf.Matrix4d(*M2.T.flatten().tolist()))
    if z2.HasAPI(UsdPhysics.RigidBodyAPI): UsdPhysics.RigidBodyAPI(z2).CreateKinematicEnabledAttr(True)   # pin it
    for nm, rad, h, z in (("pole", 0.015, float(ZED2_EYE[2] - 0.03), float(ZED2_EYE[2] - 0.03) / 2), ("foot", 0.18, 0.02, 0.01)):
        c = UsdGeom.Cylinder.Define(stage, f"/World/zed2_stand/{nm}"); c.CreateRadiusAttr(rad); c.CreateHeightAttr(h)
        UsdGeom.Xformable(c).AddTranslateOp().Set(Gf.Vec3d(float(ZED2_EYE[0]), float(ZED2_EYE[1]), z))
        c.CreateDisplayColorAttr([Gf.Vec3f(0.08, 0.08, 0.08)])
    print("[arm] added /World/zed2 at", ZED2_EYE.tolist(), "looking at", ZED2_AT.tolist(), "|", zusd, flush=True)
for i in range(3000):
    app.update()
    if i > 10 and not is_stage_loading(): break
if HAS_Z2:
    z2 = stage.GetPrimAtPath("/World/zed2")         # the referenced root is the rigid body: pin it once composed
    if z2.HasAPI(UsdPhysics.RigidBodyAPI): UsdPhysics.RigidBodyAPI(z2).CreateKinematicEnabledAttr(True)
OBJ = ["drill", "bottle", "mug", "box"]
def obj_boxes():
    bc = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_, UsdGeom.Tokens.render]); out = {}
    for n in OBJ:
        r = bc.ComputeWorldBound(stage.GetPrimAtPath(f"/World/{n}")).ComputeAlignedRange()
        out[n] = {"min": list(r.GetMin()), "max": list(r.GetMax())}
    return out
def obj_meshes():                                    # world-frame triangles of every mesh under each object
    out = {}
    for n in OBJ:
        tris = []
        for p in Usd.PrimRange(stage.GetPrimAtPath(f"/World/{n}")):
            if not p.IsA(UsdGeom.Mesh) or UsdGeom.Imageable(p).ComputeVisibility() == UsdGeom.Tokens.invisible: continue
            m = UsdGeom.Mesh(p); P = np.array(m.GetPointsAttr().Get() or [], float)
            cnt = np.array(m.GetFaceVertexCountsAttr().Get() or [], int); idx = np.array(m.GetFaceVertexIndicesAttr().Get() or [], int)
            if not len(P) or not len(cnt): continue
            W = np.array(UsdGeom.Xformable(p).ComputeLocalToWorldTransform(Usd.TimeCode.Default())).T
            Pw = (W[:3, :3] @ P.T).T + W[:3, 3]; o = 0
            for c in cnt:                            # fan-triangulate polygons
                for k in range(1, c - 1): tris.append(Pw[[idx[o], idx[o + k], idx[o + k + 1]]])
                o += c
        out[n] = np.array(tris, np.float32)
    return out
fr = stage.GetPrimAtPath("/World/franka")
art_root = next((str(p.GetPath()) for p in Usd.PrimRange(fr) if p.HasAPI(UsdPhysics.ArticulationRootAPI)), None)
print("[arm] stage", a.stage, "| franka articulation", art_root, flush=True)
if art_root is None: os._exit(2)

mk = UsdGeom.Sphere.Define(stage, "/World/mpc_ref_marker"); mk.CreateRadiusAttr(0.015)   # visual only: no collision API
mk.CreateDisplayColorAttr([Gf.Vec3f(0.85, 0.1, 0.1)]); mk_op = UsdGeom.Xformable(mk).AddTranslateOp()
mk_op.Set(Gf.Vec3d(0, 0, -5))                       # out of sight under the floor until the first /fr3_sim/goal
trail = UsdGeom.BasisCurves.Define(stage, "/World/mpc_ref_trail"); trail.CreateTypeAttr("linear")
trail.CreateWrapAttr("nonperiodic"); trail.CreateDisplayColorAttr([Gf.Vec3f(0.30, 0.30, 0.30)])
trail.CreatePointsAttr(Vt.Vec3fArray([Gf.Vec3f(0, 0, -5), Gf.Vec3f(0, 0, -5.001)])); trail.CreateCurveVertexCountsAttr([2])
trail.CreateWidthsAttr([0.006, 0.006]); trail.SetWidthsInterpolation(UsdGeom.Tokens.vertex)   # authored now: attributes
                                                     # created after RTX's first sync keep defaults (a 1 m thick tube)
# Visual ruler uses the measured articulation pose, never the requested goal.
tool_rulers=[]
for name,color,length,axis in [('forward',(1.,.35,.02),.14,2),('side',(.05,.35,1.),.075,0)]:
    line=UsdGeom.BasisCurves.Define(stage,'/World/review_tool_'+name)
    line.CreateTypeAttr('linear');line.CreateWrapAttr('nonperiodic');line.CreateDisplayColorAttr([Gf.Vec3f(*color)])
    line.CreatePointsAttr(Vt.Vec3fArray([Gf.Vec3f(0,0,-5),Gf.Vec3f(0,0,-5.001)]));line.CreateCurveVertexCountsAttr([2])
    line.CreateWidthsAttr([.008,.008]);line.SetWidthsInterpolation(UsdGeom.Tokens.vertex)
    tool_rulers.append((line,length,axis))
TRAIL_N = int(3.0 / (CTRL_EVERY * DT))
cam = UsdGeom.Camera.Define(stage, "/World/mpc_view_cam"); cam.CreateFocalLengthAttr(float(a.view[6]))
cam.CreateHorizontalApertureAttr(20.955); cam.CreateVerticalApertureAttr(20.955 * a.video_res[1] / a.video_res[0])
cam.CreateClippingRangeAttr(Gf.Vec2f(0.02, 80.0))
eye, ctr = np.array(a.view[:3]), np.array(a.view[3:6]); f_ = ctr - eye; f_ /= np.linalg.norm(f_)
r_ = np.cross(f_, [0, 0, 1.0]); r_ /= np.linalg.norm(r_); u_ = np.cross(r_, f_)
M = np.eye(4); M[:3, 0], M[:3, 1], M[:3, 2], M[:3, 3] = r_, u_, -f_, eye
cam.AddTransformOp().Set(Gf.Matrix4d(*M.T.flatten().tolist()))
def make_view_video():                               # after the ZED streams: creating the ZED annotators with this render
    global rep, Image, annot, fdir                   # product already there hung RTX ("Out of resource descriptors")
    import omni.replicator.core as rep, carb
    from PIL import Image
    carb.settings.get_settings().set("/omni/replicator/captureOnPlay", True)
    rp = rep.create.render_product("/World/mpc_view_cam", tuple(a.video_res))
    annot = rep.AnnotatorRegistry.get_annotator("rgb"); annot.attach([rp])
    fdir = os.path.join(a.out, "frames"); os.makedirs(fdir, exist_ok=True)
    for f in os.listdir(fdir): os.remove(os.path.join(fdir, f))

# ---- ROS 2 (Isaac's bundled Jazzy rclpy; spun in a thread, callbacks only store the latest message) --------------------
import rclpy
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import JointState as JointStateMsg
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float32,Empty,String
from home_pose import HOME_Q
from simulation_recording import FrameRecorder
recorder=FrameRecorder()
rclpy.init(); node = rclpy.create_node("isaac_franka")
tcp_pub = node.create_publisher(PoseStamped, "/fr3_sim/tcp", 10)
clock_pub = node.create_publisher(Clock, "/clock", 10)
js_pub = node.create_publisher(JointStateMsg, "/fr3_sim/joint_states", 10)
obj_pub = node.create_publisher(PoseStamped, "/target_object/pose", 10)       # --track-object: its rigid body in fr3_link0, sim stamps
lock = threading.Lock()
R = {"cmd": None, "cmd_n": 0, "cmd_new": False, "pose": None, "pose_n": 0, "grip": None, "grip_new": False,"reset":False,"records":[]}
def on_cmd(m):
    with lock: R["cmd"], R["cmd_n"], R["cmd_new"] = m, R["cmd_n"] + 1, True
def on_pose(m):
    with lock: R["pose"], R["pose_n"] = m, R["pose_n"] + 1
node.create_subscription(JointStateMsg, "/fr3_sim/joint_command", on_cmd, 10)
node.create_subscription(PoseStamped, "/fr3_sim/goal", on_pose, 10)
def on_grip(m):                                      # /fr3_sim/gripper_width: target of EACH finger joint, m (0.04 open, 0 closed)
    with lock: R["grip"], R["grip_new"] = float(min(max(m.data, 0.0), 0.04)), True
node.create_subscription(Float32, "/fr3_sim/gripper_width", on_grip, 10)
def on_reset(_):
    with lock:R["reset"]=True
node.create_subscription(Empty,"/fr3_sim/reset",on_reset,10)
def on_record(m):
    with lock:R["records"].append(m.data)
node.create_subscription(String,"/fr3_sim/record",on_record,10)
threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()
def stamp_s(h): return h.stamp.sec + h.stamp.nanosec * 1e-9
def quat_xyzw_to_R(x, y, z, w):
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])

# ---- physics callback ---------------------------------------------------------------------------------------------------
def obj_pose():                                      # --track-object's rigid body in the robot base frame (physics tensors; nan when not tracked)
    if not a.track_object or "rb" not in S: return np.full(3, np.nan)
    return S["root_R"].T @ (S["rb"].get_transforms().numpy()[0][:3] - S["root_p"])
S = {"art": None, "n": 0, "rec": False, "pose_first_sim": None, "trail": [], "log": {k: [] for k in (
    "t", "q", "qd", "q_cmd", "qd_cmd", "cmd_stamp", "cmd_n", "cmd_applied_t", "target", "target_quat", "target_stamp",
    "hand", "hand_quat", "tcp", "q_fingers", "obj")}}
def on_pre_step(dt, context):
    S["n"] += 1; t = S["n"] * DT
    if (S["n"] - 1) % a.clock_every == 0:
        c = Clock(); c.clock.sec = int(t); c.clock.nanosec = int(round((t - int(t)) * 1e9)) % 1000000000; clock_pub.publish(c)
    if S["art"] is None:
        view = SimulationManager.get_physics_simulation_view()
        if view is None: return
        art = view.create_articulation_view(art_root); names = list(art.shared_metatype.dof_names)
        links = list(art.shared_metatype.link_names)
        dev = art.get_dof_positions().device; idx = wp.array([0], dtype=wp.uint32, device=dev)
        q = art.get_dof_positions().numpy()[0].astype(np.float32).copy()
        pre = next(n for n in names if n.endswith("_joint1"))[:-len("joint1")]   # "fr3_" (lab arm) or "panda_"
        a2d = np.array([names.index(f"{pre}joint{j + 1}") for j in range(7)])
        q[a2d] = HOME_Q
        for i_, n_ in enumerate(names):
            if "finger" in n_: q[i_] = 0.04
        z = np.zeros_like(q)
        art.set_dof_positions(wp.array(q[None], dtype=wp.float32, device=dev), idx)
        art.set_dof_velocities(wp.array(z[None], dtype=wp.float32, device=dev), idx)
        art.set_dof_position_targets(wp.array(q[None], dtype=wp.float32, device=dev), idx)
        damping=art.get_dof_dampings().numpy().copy()
        damping[0,a2d]*=a.arm_drive_damping_scale
        # Drive properties are host metadata even when dynamics and state tensors use CUDA.
        art.set_dof_dampings(wp.array(damping,dtype=wp.float32,device="cpu"),wp.array([0],dtype=wp.uint32,device="cpu"))
        rt = art.get_root_transforms().numpy()[0]
        S.update(art=art, dev=dev, idx=idx, names=names, a2d=a2d, tgt=q.copy(), vtgt=z.copy(), hand=links.index(f"{pre}hand"),
                 root_p=rt[:3].copy(), root_R=quat_xyzw_to_R(*rt[3:7]), cmd_q=q[a2d].copy(), cmd_qd=z[a2d].copy(),
                 cmd_stamp=float("nan"), cmd_applied_t=float("nan"), cmd_n=0,
                 gains={"stiffness": art.get_dof_stiffnesses().numpy()[0][a2d].tolist(),
                        "damping": art.get_dof_dampings().numpy()[0][a2d].tolist()})
        try:
            fi = [i_ for i_, n_ in enumerate(names) if "finger" in n_]
            print("[arm] fingers", [names[i_] for i_ in fi], "stiffness", art.get_dof_stiffnesses().numpy()[0][fi].tolist(),
                  "damping", art.get_dof_dampings().numpy()[0][fi].tolist(), "max force", art.get_dof_max_forces().numpy()[0][fi].tolist(), flush=True)
        except Exception as e_: print("[arm] finger gains unavailable:", e_, flush=True)
        S["fi"] = [i_ for i_, n_ in enumerate(names) if "finger" in n_]
        if a.track_object:
            rb = next(str(p_.GetPath()) for p_ in Usd.PrimRange(stage.GetPrimAtPath(f"/World/{a.track_object}")) if p_.HasAPI(UsdPhysics.RigidBodyAPI))
            S["rb"] = view.create_rigid_body_view(rb); print("[arm] tracking rigid body", rb, flush=True)
        print("[arm] physics tensor device:", str(dev),flush=True)
        if a.physics_device.startswith("cuda") and not dev.is_cuda:raise RuntimeError("GPU physics required; refusing CPU tensor fallback")
        print("[arm] dofs", names, "| root", rt.tolist(), "| gains", S["gains"], flush=True)
        return
    art = S["art"]
    with lock:
        m, new, cmd_n = R["cmd"], R["cmd_new"], R["cmd_n"]; R["cmd_new"] = False
        pose, pose_n = R["pose"], R["pose_n"]
        grip, grip_new = R["grip"], R["grip_new"]; R["grip_new"] = False
        reset=R["reset"];R["reset"]=False
        commands=R["records"][:];R["records"].clear()
        if reset:R["cmd"]=None;R["cmd_new"]=False;R["grip_new"]=False;R["pose"]=None
    for command in commands:
        try:recorder.handle_command(command,time.monotonic())
        except Exception as error:print("[remote] record error",str(error),flush=True)
    if reset:
        q=S["tgt"].copy();q[S["a2d"]]=HOME_Q;q[S["fi"]]=.04;zero=np.zeros_like(q)
        art.set_dof_positions(wp.array(q[None],dtype=wp.float32,device=S["dev"]),S["idx"])
        art.set_dof_velocities(wp.array(zero[None],dtype=wp.float32,device=S["dev"]),S["idx"])
        art.set_dof_position_targets(wp.array(q[None],dtype=wp.float32,device=S["dev"]),S["idx"])
        art.set_dof_velocity_targets(wp.array(zero[None],dtype=wp.float32,device=S["dev"]),S["idx"])
        S["tgt"],S["vtgt"]=q,zero;S["trail"]=[];m=None;new=False;grip_new=False;pose=None

    if grip_new:
        for i_ in S["fi"]: S["tgt"][i_] = grip
    if (new and m is not None) or grip_new:                        # latest command -> drive targets (by joint name)
        for nm, p_, v_ in (zip(m.name, m.position, m.velocity if len(m.velocity) else [0.0] * len(m.position)) if new and m is not None else []):
            if nm in S["names"]:
                i_ = S["names"].index(nm); S["tgt"][i_], S["vtgt"][i_] = p_, v_
        art.set_dof_position_targets(wp.array(S["tgt"][None], dtype=wp.float32, device=S["dev"]), S["idx"])
        art.set_dof_velocity_targets(wp.array(S["vtgt"][None], dtype=wp.float32, device=S["dev"]), S["idx"])
        S["cmd_q"], S["cmd_qd"] = S["tgt"][S["a2d"]].copy(), S["vtgt"][S["a2d"]].copy()
        if new and m is not None:S["cmd_stamp"], S["cmd_applied_t"], S["cmd_n"] = stamp_s(m.header), t, cmd_n
    if a.track_object and "rb" in S and S["n"] % CTRL_EVERY == 0:                # live pose of the object: target_mask.py rasterizes its mask
        w_ = S["rb"].get_transforms().numpy()[0]; pb = S["root_R"].T @ (w_[:3] - S["root_p"])
        om = PoseStamped(); om.header.frame_id = "fr3_link0"; om.header.stamp.sec, om.header.stamp.nanosec = int(t), int(round((t - int(t)) * 1e9)) % 1000000000
        om.pose.position.x, om.pose.position.y, om.pose.position.z = (float(v) for v in pb)
        om.pose.orientation.x, om.pose.orientation.y, om.pose.orientation.z, om.pose.orientation.w = (float(v) for v in w_[3:7])   # base has no rotation
        obj_pub.publish(om)
    if S["n"] % a.js_every == 0:
        q = art.get_dof_positions().numpy()[0]; qd = art.get_dof_velocities().numpy()[0]
        js = JointStateMsg(); js.header.stamp.sec, js.header.stamp.nanosec = int(t), int(round((t - int(t)) * 1e9)) % 1000000000
        js.name = S["names"]; js.position = [float(v) for v in q]; js.velocity = [float(v) for v in qd]; js_pub.publish(js)
        lt = art.get_link_transforms().numpy()[0][S["hand"]]
        Rh = S["root_R"].T @ quat_xyzw_to_R(*lt[3:7])
        tcp = S["root_R"].T @ (lt[:3]-S["root_p"]) + Rh @ np.array([0.,0.,TCP])
        tm = PoseStamped(); tm.header=js.header;tm.header.frame_id="fr3_link0"
        tm.pose.position.x,tm.pose.position.y,tm.pose.position.z=map(float,tcp)
        tm.pose.orientation.x,tm.pose.orientation.y,tm.pose.orientation.z,tm.pose.orientation.w=map(float,lt[3:7])
        tcp_pub.publish(tm)
    if pose is not None and S["n"] % CTRL_EVERY == 0:
        if S["pose_first_sim"] is None: S["pose_first_sim"] = t
        p = np.array([pose.pose.position.x, pose.pose.position.y, pose.pose.position.z])
        o = pose.pose.orientation; Rt = quat_xyzw_to_R(o.x, o.y, o.z, o.w)
        tip_w = S["root_p"] + S["root_R"] @ (p)
        S["trail"] = (S["trail"] + [Gf.Vec3f(*tip_w.tolist())])[-TRAIL_N:]
        if S["rec"]:
            q = art.get_dof_positions().numpy()[0]; qd = art.get_dof_velocities().numpy()[0]
            lt = art.get_link_transforms().numpy()[0][S["hand"]]
            Rw = S["root_R"].T                       # world -> base (fr3_link0)
            hand = Rw @ (lt[:3] - S["root_p"]); Rh = Rw @ quat_xyzw_to_R(*lt[3:7])
            L = S["log"]
            for k_, v_ in (("t", t), ("q", q[S["a2d"]]), ("qd", qd[S["a2d"]]), ("q_cmd", S["cmd_q"]), ("qd_cmd", S["cmd_qd"]),
                           ("cmd_stamp", S["cmd_stamp"]), ("cmd_n", S["cmd_n"]), ("cmd_applied_t", S["cmd_applied_t"]),
                           ("target", p), ("target_quat", [o.x, o.y, o.z, o.w]), ("target_stamp", stamp_s(pose.header)),
                           ("hand", hand), ("hand_quat", lt[3:7]), ("tcp", hand + Rh @ np.array([0.0, 0.0, TCP])), ("q_fingers", q[S["fi"]]),
                           ("obj", obj_pose())):
                L[k_].append(np.array(v_, dtype=np.float64))
# ---- grasp perception: per-object point clouds from instance segmentation + depth (stands in for SAM2 + ZED depth) ----------
def grasp_capture():
    """Each capture camera (the ZED stands' left lens) renders instance_id_segmentation + distance_to_image_plane; every pixel of
    an object's instance becomes a point in the robot base frame (fr3_link0 = world minus the base position, Z up). Writes
    clouds/<object>.npz (pc = fused, pc_<cam>) and clouds/summary.json (pixels and points per camera and object)."""
    import omni.replicator.core as rep
    cdir = os.path.join(a.out, "clouds"); os.makedirs(cdir, exist_ok=True); Wd, Hd = a.capture_res
    root = np.array(json.load(open(os.path.join(HERE, "runs", "workcell", "scene.json")))["franka_base"])
    per, summ = {n: {} for n in OBJ}, {}
    for cn in CAPCAMS:
        prim = stage.GetPrimAtPath(f"/World/{cn}/base_link/ZED_X/CameraLeft"); cm = UsdGeom.Camera(prim)
        fl, ap_h = float(cm.GetFocalLengthAttr().Get()), float(cm.GetHorizontalApertureAttr().Get())
        fx = fl * Wd / ap_h; K = np.array([[fx, 0, Wd / 2], [0, fx, Hd / 2], [0, 0, 1.0]])   # square pixels
        Tw = np.array(UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default())).T
        rp = rep.create.render_product(str(prim.GetPath()), (Wd, Hd))
        ia = rep.AnnotatorRegistry.get_annotator("instance_id_segmentation", init_params={"colorize": False}); ia.attach([rp])
        da = rep.AnnotatorRegistry.get_annotator("distance_to_image_plane"); da.attach([rp])
        for _ in range(6): rep.orchestrator.step(rt_subframes=4, pause_timeline=False)
        seg, depth = ia.get_data(), np.asarray(da.get_data(), np.float64)
        info = seg["info"]["idToLabels"]; ids = np.asarray(seg["data"]); ids = ids[..., 0] if ids.ndim == 3 else ids
        v, u = np.mgrid[0:Hd, 0:Wd]; ok = np.isfinite(depth) & (depth > 0.1) & (depth < 6.0)
        summ[cn] = {"focal_mm": fl, "h_aperture_mm": ap_h, "fx_px": round(fx, 1), "pixels": {}, "points": {}}
        for n in OBJ:
            m = np.zeros_like(ok)
            for i_, path in info.items():
                p_ = path if isinstance(path, str) else str(path)
                if p_ == f"/World/{n}" or p_.startswith(f"/World/{n}/"): m |= (ids == int(i_))
            m &= ok; z = depth[m]; uu, vv = u[m], v[m]
            Pc = np.stack([(uu - K[0, 2]) * z / K[0, 0], -(vv - K[1, 2]) * z / K[1, 1], -z], 1)   # USD camera: -Z forward, +Y up
            P = (Tw[:3, :3] @ Pc.T).T + Tw[:3, 3] - root
            per[n][cn] = P.astype(np.float32); summ[cn]["pixels"][n] = int(m.sum())
            summ[cn]["points"][n] = int(len(P))
        ia.detach([rp]); da.detach([rp]); rp.destroy()
    for n in OBJ:
        np.savez_compressed(os.path.join(cdir, f"{n}.npz"), pc=np.concatenate([per[n][c] for c in CAPCAMS]),
                            **{f"pc_{c}": per[n][c] for c in CAPCAMS})
    json.dump(summ, open(os.path.join(cdir, "summary.json"), "w"), indent=1)
    print("[arm] grasp capture:", json.dumps({c: summ[c]["pixels"] for c in CAPCAMS}), flush=True)

SimulationManager.setup_simulation(dt=DT, device=a.physics_device)
gpu_scenes=[dict(gpu_dynamics=scene.get_enabled_gpu_dynamics(),broadphase=scene.get_broadphase_type()) for scene in SimulationManager.get_physics_scenes()]
print("[remote] physics backend:",str(SimulationManager.get_device()),json.dumps(gpu_scenes),flush=True)
if a.physics_device.startswith("cuda") and not all(scene['gpu_dynamics'] and scene['broadphase']=='GPU' for scene in gpu_scenes):
    raise RuntimeError("GPU PhysX required; CPU fallback refused")
SimulationManager.enable_fabric(True)
cb = SimulationManager.register_callback(on_pre_step, event=SimulationEvent.PHYSICS_PRE_STEP)
tl = omni.timeline.get_timeline_interface(); tl.play()

# ---- grasp perception capture: BEFORE the ZED annotators exist (render products made and destroyed on a camera that is already
# streaming made the ZED wrapper time out in its handshake, 10-15)
if CAPCAMS:
    for _ in range(150): app.update()                # the arm is at retract and the objects have settled (3 s)
    grasp_capture()
    if a.capture_only:
        os.system(f"chown -R 1000:1000 {a.out}"); import sys; sys.stdout.flush(); os._exit(0)

# ---- ZED streams, camera poses, ground-truth objects ---------------------------------------------------------------------
for _ in range(10): app.update()
zeds = []
if ZEDS:
    import glob
    from pxr import Sdf
    from sl.sensor.camera.annotators import ZEDAnnotator
    for n, port in ZEDS:
        for f in glob.glob(f"/dev/shm/sl_local_*_{port}"):   # a receiver that died leaves a held IPC lock (zed_stream.py)
            os.remove(f); print("[arm] removed stale", f, flush=True)
        zeds.append(ZEDAnnotator([Sdf.Path(f"/World/{n}")], camera_model="ZED_X", streaming_port=port, resolution=a.zed_res,
                                 fps=a.zed_fps, transport_layer_mode="BOTH"))
        left = stage.GetPrimAtPath(f"/World/{n}/base_link/ZED_X/CameraLeft")
        Tl = np.array(UsdGeom.Xformable(left).ComputeLocalToWorldTransform(Usd.TimeCode.Default())).T
        json.dump({"prim": str(left.GetPath()), "T_world_camera_usd": Tl.round(6).tolist()}, open(os.path.join(a.out, f"camera_{n}.json"), "w"), indent=1)
        print(f"[arm] streaming /World/{n} as ZED_X {a.zed_res} @ {a.zed_fps} fps on port {port}; left camera at {Tl[:3, 3].round(3).tolist()}", flush=True)
if not a.no_video: make_view_video()
if a.zed_preview or a.preview_only:
    import omni.replicator.core as rep
    from PIL import Image
    for n, _ in (ZEDS if a.zed_preview else []):      # --preview-only alone: just the video camera's frame
        rp_ = rep.create.render_product(f"/World/{n}/base_link/ZED_X/CameraLeft", (960, 540))
        an_ = rep.AnnotatorRegistry.get_annotator("rgb"); an_.attach([rp_])
        for _ in range(3): rep.orchestrator.step(rt_subframes=4, pause_timeline=False)
        im = an_.get_data(); im = im.get("data") if isinstance(im, dict) else im
        Image.fromarray(np.asarray(im)[..., :3]).save(os.path.join(a.out, f"zed_{n}_left.jpg"), quality=90)
        an_.detach([rp_]); rp_.destroy()
    if not a.no_video:                               # and one frame of the video camera
        rep.orchestrator.step(rt_subframes=4, pause_timeline=False); rep.orchestrator.step(rt_subframes=4, pause_timeline=False)
        im = annot.get_data(); im = im.get("data") if isinstance(im, dict) else im
        Image.fromarray(np.asarray(im)[..., :3]).save(os.path.join(a.out, "view_preview.jpg"), quality=90)
    print("[arm] ZED previews saved", flush=True)
    if a.preview_only:
        os.system(f"chown -R 1000:1000 {a.out}"); import sys; sys.stdout.flush(); os._exit(0)
M0 = obj_meshes(); B0 = obj_boxes()
np.savez_compressed(os.path.join(a.out, "objects_start.npz"), **M0)
if a.track_object:                                   # the pose the exported triangles belong to (target_mask.py moves them with the live pose)
    w_ = S["rb"].get_transforms().numpy()[0]
    json.dump({"object": a.track_object, "p_base": (S["root_R"].T @ (w_[:3] - S["root_p"])).tolist(), "quat_xyzw": [float(v) for v in w_[3:7]]},
              open(os.path.join(a.out, "object_ref.json"), "w"))
print("[arm] objects:", {k: len(v) for k, v in M0.items()}, "triangles", flush=True)

# Own the physics clock explicitly. Direct SimulationManager.step does not dispatch
# the timeline's PHYSICS_PRE_STEP event; run the ROS/control callback ourselves.
SimulationManager.deregister_callback(cb)
tl.set_auto_update(False);tl.commit_silently()
def advance_physics(count):
    budget_start=time.monotonic()
    for _ in range(count):
        on_pre_step(DT,None)
        SimulationManager.step(steps=1)
        # Never hide the renderer behind a multi-second catch-up batch when
        # physics cannot keep up with wall time on this host.
        if time.monotonic()-budget_start>=.02:break

# Live loop: bounded latest JPEG only; no unbounded frame directory or trace buffers.
wall0,n0,updates,last_shot = time.monotonic(),S["n"],0,0.
performance_at,performance_n,performance_frames,last_report=time.monotonic(),S["n"],0,0.
physics_wall,render_wall=0.,0.
import omni.physxfabric
fabric_interface=omni.physxfabric.get_physx_fabric_interface()
print("[remote] ready: /fr3_sim/* only; latest view.jpg",flush=True)
while app.is_running() and time.monotonic()-wall0 < a.wait:
    # Rendering must not set the rate of physics. Catch up wall time with physics-only steps.
    now=time.monotonic()
    deficit=(now-wall0)-(S["n"]-n0)*DT
    if deficit>DT:
        before=time.monotonic();advance_physics(min(100,max(1,int(deficit/DT))));physics_wall+=time.monotonic()-before
    if not a.no_video and (now-last_shot >= 1./a.fps or recorder.should_capture(now)):
        before=time.monotonic()
        # Manual PhysX steps default to update_fabric=False. The renderer must
        # explicitly receive the new transforms when timeline auto-update is off.
        fabric_interface.update(SimulationManager.get_simulation_time(),DT)
        if S['art'] is not None:
            transform=S['art'].get_link_transforms().numpy()[0][S['hand']]
            rotation=quat_xyzw_to_R(*transform[3:7]);tip=transform[:3]+rotation@np.array([0.,0.,TCP])
            for ruler,length,axis in tool_rulers:
                ruler.GetPointsAttr().Set(Vt.Vec3fArray([Gf.Vec3f(*tip.tolist()),Gf.Vec3f(*(tip+length*rotation[:,axis]).tolist())]))
        if S["trail"]:
            mk_op.Set(Gf.Vec3d(*S["trail"][-1]))
        if len(S["trail"])>=2:
            trail.GetPointsAttr().Set(Vt.Vec3fArray(S["trail"]));trail.GetCurveVertexCountsAttr().Set([len(S["trail"])])
            trail.GetWidthsAttr().Set([.006]*len(S["trail"]))
        app.update()  # Continuous captureOnPlay avoids Replicator waiting on a manually advanced physics clock.
        im=annot.get_data();im=im.get("data") if isinstance(im,dict) else im
        if im is not None and getattr(im,"size",0):
            tmp=os.path.join(a.out,"view.tmp.jpg")
            Image.fromarray(np.asarray(im)[...,:3]).save(tmp,quality=80)
            os.replace(tmp,os.path.join(a.out,"view.jpg"))
            if recorder.should_capture(time.monotonic()):recorder.record_frame(np.asarray(im)[...,:3],S["n"]*DT,now_mono=time.monotonic())
        performance_frames+=1
        render_wall+=time.monotonic()-before
        # Start the frame interval AFTER rendering; otherwise a slow render makes
        # every iteration render again and starves the physics-only iterations.
        last_shot=time.monotonic()
    elif deficit<=DT:
        time.sleep(.001)
    updates+=1
    lead=(S["n"]-n0)*DT-(time.monotonic()-wall0)
    if lead>0:time.sleep(min(lead,.02))
    report_now=time.monotonic()
    if report_now-last_report>=2.:
        elapsed=report_now-performance_at
        stats=dict(backend="isaac_physx",physics_device=str(SimulationManager.get_device()),tensor_device=str(S.get("dev")),physical_gpu=os.environ.get("FR3_PHYSICAL_GPU"),gpu_scenes=gpu_scenes,arm_drive_damping_scale=a.arm_drive_damping_scale,configured_physics_hz=a.physics_hz,physics_hz=(S["n"]-performance_n)/max(elapsed,1e-6),
                   realtime_factor=(S["n"]-performance_n)*DT/max(elapsed,1e-6),
                   render_fps=performance_frames/max(elapsed,1e-6),physics_wall_s=physics_wall,render_wall_s=render_wall,at=time.time())
        path=os.path.join(a.out,"performance.tmp.json")
        with open(path,"w") as f:json.dump(stats,f)
        os.replace(path,os.path.join(a.out,"performance.json"))
        performance_at,performance_n,performance_frames,last_report=report_now,S["n"],0,report_now
        physics_wall,render_wall=0.,0.
        print('[remote] '+json.dumps(stats),flush=True)
recorder.stop();tl.stop();app.close()
