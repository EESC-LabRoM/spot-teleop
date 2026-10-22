#!/usr/bin/env python3
"""cuRobo MotionGen PLAN-ONLY on the real FR3's nvblox map. Sends nothing to the robot: reads /joint_states (relayed, read-only) as the
start, asks the ESDF service(s) for the cuRobo grid box (as fr3_curobo_mpc_node.py / the lab's curobo_mpc_node.py: fr3_link0, 2 cm), loads it as the
voxel world and plans to goals relative to the tracked object's /target_object/clear_box. Prints success, planning time, path length and the
minimum robot-sphere-vs-map clearance along the planned path. The clear shape (box + --pad, as cuRobo's _build_target_clear) is sent with the request.
  python3 fr3_curobo_plan_only.py --maps masked=nvblox_node,raw=nvblox_raw_ref [--pad 0.03] [--out DIR]"""
import argparse, json, os, sys, time
import numpy as np, rclpy, torch
from geometry_msgs.msg import Point, Vector3
from nvblox_msgs.srv import EsdfAndGradients
from rclpy.node import Node
from sensor_msgs.msg import JointState
from visualization_msgs.msg import Marker
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import fr3_curobo_mpc_node as base
from curobo.geom.sdf.world import CollisionCheckerType
from curobo.geom.types import WorldConfig
from curobo.types.base import TensorDeviceType
from curobo.types.math import Pose
from curobo.types.state import JointState as CuJointState
from curobo.wrap.reacher.motion_gen import MotionGen, MotionGenConfig, MotionGenPlanConfig

ap = argparse.ArgumentParser(); ap.add_argument("--maps", default="masked=nvblox_node"); ap.add_argument("--pad", type=float, default=0.03)
ap.add_argument("--out", default="/tmp/plan"); ap.add_argument("--urdf", default=os.path.join(base.HERE, "fr3.urdf"))
a, ros = ap.parse_known_args(); os.makedirs(a.out, exist_ok=True)
rclpy.init(args=ros); n = Node("fr3_curobo_plan_only"); got = {}
n.create_subscription(JointState, "/joint_states", lambda m: got.__setitem__("js", m), 10)
n.create_subscription(Marker, "/target_object/clear_box", lambda m: got.__setitem__("box", m), 10)
t0 = time.monotonic()
while ("js" not in got or "box" not in got) and time.monotonic() - t0 < 15: rclpy.spin_once(n, timeout_sec=0.1)
q0 = np.array([dict(zip(got["js"].name, got["js"].position))[j] for j in base.ARM], np.float32)
bm = got["box"]; c = np.array([bm.pose.position.x, bm.pose.position.y, bm.pose.position.z]); s = np.array([bm.scale.x, bm.scale.y, bm.scale.z])
print("start q", q0.round(3).tolist(), "| target box", c.round(3).tolist(), s.round(3).tolist(), flush=True)

def esdf(node, clear):
    cli = n.create_client(EsdfAndGradients, f"/{node}/get_esdf_and_gradient"); cli.wait_for_service(timeout_sec=10.0)
    r = EsdfAndGradients.Request(); r.update_esdf, r.visualize_esdf, r.use_aabb, r.frame_id = True, False, True, base.BASE
    lo = np.array(base.GRID_CENTER) - np.array(base.GRID_SIZE) / 2
    r.aabb_min_m = Point(x=float(lo[0]), y=float(lo[1]), z=float(lo[2])); r.aabb_size_m = Vector3(x=float(base.GRID_SIZE[0]), y=float(base.GRID_SIZE[1]), z=float(base.GRID_SIZE[2]))
    if clear:
        h = s / 2 + a.pad
        r.aabbs_to_clear_min_m = [Point(x=float(c[0] - h[0]), y=float(c[1] - h[1]), z=float(c[2] - h[2]))]
        r.aabbs_to_clear_size_m = [Vector3(x=float(2 * h[0]), y=float(2 * h[1]), z=float(2 * h[2]))]
    t = time.monotonic(); f = cli.call_async(r); rclpy.spin_until_future_complete(n, f, timeout_sec=30.0); res = f.result()
    d = np.asarray(res.esdf_and_gradients.data, np.float32).reshape([x.size for x in res.esdf_and_gradients.layout.dim])
    return d, np.array([res.origin_m.x, res.origin_m.y, res.origin_m.z]), 1000 * (time.monotonic() - t)

tensor_args = TensorDeviceType(); robot_cfg = base.fr3_robot_cfg(a.urdf)
world_cfg = WorldConfig.from_dict({"voxel": {"world_voxel": {"dims": base.GRID_SIZE, "pose": base.GRID_CENTER + [1, 0, 0, 0], "voxel_size": base.VOXEL, "feature_dtype": torch.float32}}})
t = time.monotonic()
mg = MotionGen(MotionGenConfig.load_from_robot_config(robot_cfg, world_cfg, tensor_args, collision_checker_type=CollisionCheckerType.VOXEL, use_cuda_graph=True,
                                                      num_trajopt_seeds=4, num_graph_seeds=4, interpolation_dt=0.02, collision_activation_distance=0.02))
mg.warmup(enable_graph=True, warmup_js_trajopt=False)
print(f"MotionGen loaded+warmed up in {time.monotonic() - t:.1f} s, device {tensor_args.device}", flush=True)
kin = mg.kinematics
top = c[2] + s[2] / 2
goals = {"pregrasp_10cm_above_object_top": [c[0], c[1], top + 0.10],
         "grasp_at_object_centre": [c[0], c[1], c[2]]}                     # inside the object's volume: only feasible if the map does not contain it
Q_DOWN = [0.0, 1.0, 0.0, 0.0]                                              # w x y z: tool z pointing down
res_all = {"start_q": q0.round(4).tolist(), "box": dict(centre=c.round(3).tolist(), size=s.round(3).tolist()), "pad": a.pad, "plans": []}
for label, node in [x.split("=") for x in a.maps.split(",")]:
    for clear in (False, True):
        d, origin, ems = esdf(node, clear)
        with torch.no_grad():
            mg.world_coll_checker.update_voxel_data(base.esdf_to_voxelgrid(d, origin, tensor_args.device))
        for gname, gp in goals.items():
            start = CuJointState.from_position(tensor_args.to_device(q0[None, :]), joint_names=mg.kinematics.joint_names)
            goal = Pose(position=tensor_args.to_device(np.array([gp], np.float32)), quaternion=tensor_args.to_device(np.array([Q_DOWN], np.float32)))
            t = time.monotonic()
            r = mg.plan_single(start, goal, MotionGenPlanConfig(max_attempts=4, enable_graph=True, enable_graph_attempt=2, timeout=20.0))
            wall = time.monotonic() - t; ok = bool(r.success.item())
            row = dict(map=label, node=node, clear_box_pad=bool(clear), goal=gname, goal_xyz=np.round(gp, 3).tolist(), success=ok, plan_time_s=round(float(r.solve_time), 3), wall_s=round(wall, 3),
                       status=str(r.status), esdf_ms=round(ems, 1), occupied_voxels=int(((d > -999) & (d <= 0)).sum()))
            if ok:
                traj = r.get_interpolated_plan(); row.update(traj_points=int(traj.position.shape[0]), traj_s=round(float(r.motion_time), 2),
                                                           path_len_rad=round(float(np.linalg.norm(np.diff(traj.position.cpu().numpy()[:, :7], axis=0), axis=1).sum()), 3))
            print(json.dumps(row), flush=True); res_all["plans"].append(row)
json.dump(res_all, open(os.path.join(a.out, "plan_only.json"), "w"), indent=1)
