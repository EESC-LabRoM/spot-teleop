#!/usr/bin/env python3
"""cuRobo MPC on the FR3 against the arm-filtered nvblox ESDF, PASSIVE: it runs the same MPPI MPC as the Spot node
(spot_operation_ros2/curobo_mpc_node.py) on the measured joint state and publishes what it WOULD command, but nothing reaches the robot
(this runs on the perception domain 8; the robot is on domain 77 and only the read-only relay connects them).

Robot model: cuRobo's franka.yml (spheres, self-collision, MPC weights) with panda_* renamed to fr3_*, kinematics and joint limits from the
real FR3 URDF (fr3.urdf, written by fr3_state_relay.py; meshes stripped), end effector fr3_hand_tcp (the teleop's control frame).
World: nvblox ESDF via /nvblox_node/get_esdf_and_gradient over a fixed box around the arm, in fr3_link0 (= nvblox global frame here).

Inputs (domain 8):  /joint_states (relayed), /fr3/curobo/goal_pose (PoseStamped, fr3_link0, TCP; default = hold the current TCP)
Outputs:            /fr3/curobo/command_preview (JointState: the MPC's next joint target, NOT sent to the robot)
                    /fr3/curobo/spheres (MarkerArray: robot collision spheres, red = inside the map's obstacles)
                    log every 2 s: ESDF occupancy, per-link clearance of the CURRENT arm to the map, MPC step time and pose error.
The clearance check is the self-filter test: with fr3_arm_mask_node.py working, the arm's own spheres must not be inside obstacles.
  python3 fr3_curobo_mpc_node.py [--urdf fr3.urdf] [--rate 20]
"""
import argparse
import os
import re
import tempfile
import threading
import time

import numpy as np
import rclpy
import torch
import yaml
from geometry_msgs.msg import Point, PoseStamped, Vector3
from nvblox_msgs.srv import EsdfAndGradients
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray

from curobo.geom.sdf.world import CollisionCheckerType
from curobo.geom.types import VoxelGrid as CuVoxelGrid, WorldConfig
from curobo.rollout.rollout_base import Goal
from curobo.types.math import Pose
from curobo.types.state import JointState as CuJointState
from curobo.util_file import get_robot_configs_path, join_path, load_yaml
from curobo.wrap.reacher.mpc import MpcSolver, MpcSolverConfig

BASE, EE = "fr3_link0", "fr3_hand_tcp"
ARM = [f"fr3_joint{i}" for i in range(1, 8)]
HOME = [0.0, -0.7853981634, 0.0, -2.3561944902, 0.0, 1.5707963268, 0.7853981634]
GRID_CENTER, GRID_SIZE, VOXEL = [0.35, 0.0, 0.55], [1.4, 1.6, 1.2], 0.02   # m, fr3_link0: table to above the arm's reach
HERE = os.path.dirname(os.path.abspath(__file__))
MPC_OVERRIDE = os.path.join(HERE, "..", "..", "..", "..", "spot-ros2_ws", "src", "spot_operation_ros2", "config", "mpc_override.yml")


def rename(obj, old="panda_", new="fr3_"):
    if isinstance(obj, dict): return {rename(k, old, new): rename(v, old, new) for k, v in obj.items()}
    if isinstance(obj, list): return [rename(v, old, new) for v in obj]
    if isinstance(obj, str): return obj.replace(old, new)
    return obj


def fr3_robot_cfg(urdf_path):
    """cuRobo franka.yml -> FR3: names, URDF (real limits, no meshes), TCP end effector, home as retract."""
    cfg = rename(load_yaml(join_path(get_robot_configs_path(), "franka.yml")))["robot_cfg"]
    k = cfg["kinematics"]
    urdf = open(urdf_path).read()
    urdf = re.sub(r"<visual>.*?</visual>|<collision>.*?</collision>", "", urdf, flags=re.S)    # meshes are package:// paths, unused
    urdf = re.sub(r"<mimic[^>]*/>", "", urdf)       # fingers are locked at 0.04 anyway; cuRobo's lock_joints trips on the mimic joint
    tu = tempfile.NamedTemporaryFile("w", suffix="_fr3_curobo.urdf", delete=False); tu.write(urdf); tu.close()
    sph = rename(load_yaml(join_path(get_robot_configs_path(), k["collision_spheres"])))
    ts = tempfile.NamedTemporaryFile("w", suffix="_fr3_spheres.yml", delete=False); yaml.safe_dump(sph, ts); ts.close()
    k.update(urdf_path=tu.name, asset_root_path="", ee_link=EE, collision_spheres=ts.name, use_usd_kinematics=False)
    k["mesh_link_names"] = []
    k["cspace"]["retract_config"] = HOME + [0.04, 0.04]
    return cfg


def esdf_to_voxelgrid(d, origin, device):
    """nvblox ESDF block (x, y, z array: + outside, - inside, -1000 unknown; origin = min corner, fr3_link0) -> cuRobo VoxelGrid."""
    feat = torch.as_tensor(np.ascontiguousarray(d, np.float32)).reshape(-1, 1)
    feat[feat < -999.9] = 1000.0; feat = -feat; feat += 0.5 * VOXEL                         # -> cuRobo convention (as the Spot node)
    center = np.asarray(origin) + np.array(GRID_SIZE) / 2
    return CuVoxelGrid(name="world_voxel", dims=GRID_SIZE, pose=[*center.tolist(), 1, 0, 0, 0], voxel_size=VOXEL,
                       feature_tensor=feat.to(device=device, dtype=torch.float32))


def sphere_clearance(sph, d, origin):
    """per sphere: ESDF distance at its centre (nearest voxel) minus its radius; NaN outside the grid or in unknown space."""
    ijk = np.floor((sph[:, :3] - origin) / VOXEL).astype(int)
    inside = np.all((ijk >= 0) & (ijk < np.array(d.shape)), axis=1)
    dist = np.full(len(sph), np.nan)
    dist[inside] = d[ijk[inside, 0], ijk[inside, 1], ijk[inside, 2]]
    dist[dist < -999.9] = np.nan
    return dist - sph[:, 3]


class Fr3CuroboMpc(Node):
    def __init__(self, a):
        super().__init__("fr3_curobo_mpc")
        self.a = a
        self.robot_cfg = fr3_robot_cfg(a.urdf)
        world_cfg = WorldConfig.from_dict({"voxel": {"world_voxel": {
            "dims": GRID_SIZE, "pose": GRID_CENTER + [1, 0, 0, 0], "voxel_size": VOXEL, "feature_dtype": torch.float32}}})
        self.get_logger().info("loading the MPC (CUDA graphs, first step is slow)...")
        mpc_cfg = MpcSolverConfig.load_from_robot_config(
            self.robot_cfg, world_cfg, use_cuda_graph=True, use_cuda_graph_metrics=False, use_cuda_graph_full_step=False,
            self_collision_check=True, collision_checker_type=CollisionCheckerType.VOXEL,
            override_particle_file=MPC_OVERRIDE if os.path.exists(MPC_OVERRIDE) else None,
            use_mppi=True, use_lbfgs=False, use_es=False, store_rollouts=False, step_dt=1.0 / a.rate)
        self.mpc = MpcSolver(mpc_cfg)
        self.world = self.mpc.world_coll_checker
        self.grid_shape = self.world.get_voxel_grid("world_voxel").get_grid_shape()[0]
        self.robot = self.mpc.rollout_fn.dynamics_model.robot_model
        self.jn = self.mpc.rollout_fn.joint_names
        self.tensor_args = self.mpc.tensor_args
        kc = self.robot.kinematics_config
        idx_to_name = {v: k for k, v in kc.link_name_to_idx_map.items()}
        self.sphere_link = [idx_to_name.get(int(i), "?") for i in kc.link_sphere_idx_map.cpu().numpy()]

        self.lock = threading.Lock()
        self.gpu = threading.Lock()   # one GPU user at a time: the MPC step captures CUDA graphs, concurrent kernels invalidate them
        self.q_meas, self.goal_pose, self.goal_dirty, self.esdf = None, None, False, None
        self.stats = dict(steps=0, ms=0.0, err=float("nan"), esdf_ms=0.0, occ=0, esdf_n=0)
        self.cmd_pub = self.create_publisher(JointState, "/fr3/curobo/command_preview", 10)
        self.sph_pub = self.create_publisher(MarkerArray, "/fr3/curobo/spheres", 1)
        self.create_subscription(JointState, "/joint_states", self.on_js, 20)
        self.create_subscription(PoseStamped, "/fr3/curobo/goal_pose", self.on_goal, 10)
        self.esdf_cli = self.create_client(EsdfAndGradients, "/nvblox_node/get_esdf_and_gradient",
                                           callback_group=MutuallyExclusiveCallbackGroup())
        self.esdf_pending = False
        self.create_timer(1.0 / a.esdf_rate, self.request_esdf, callback_group=MutuallyExclusiveCallbackGroup())
        self.create_timer(1.0 / a.rate, self.step, callback_group=MutuallyExclusiveCallbackGroup())
        self.create_timer(2.0, self.report, callback_group=MutuallyExclusiveCallbackGroup())
        self.goal_buffer = None
        self.get_logger().info(f"FR3 cuRobo MPC ready (PASSIVE, preview only): {len(self.sphere_link)} spheres, grid {self.grid_shape} "
                               f"at {VOXEL * 100:.0f} cm, joints {self.jn}")

    # ---- inputs -------------------------------------------------------------------------------------------------------
    def on_js(self, m):
        d = dict(zip(m.name, m.position))
        if all(j in d for j in ARM):
            with self.lock: self.q_meas = np.array([d[j] for j in ARM], np.float32)   # fingers are locked joints in the cuRobo model

    def on_goal(self, m):
        if m.header.frame_id not in ("", BASE):
            self.get_logger().warn(f"goal in {m.header.frame_id} ignored: give it in {BASE}", throttle_duration_sec=5.0); return
        p, o = m.pose.position, m.pose.orientation
        with self.lock: self.goal_pose, self.goal_dirty = ([p.x, p.y, p.z], [o.w, o.x, o.y, o.z]), True

    # ---- world --------------------------------------------------------------------------------------------------------
    def request_esdf(self):
        if self.esdf_pending or not self.esdf_cli.service_is_ready(): return
        r = EsdfAndGradients.Request()
        r.update_esdf, r.visualize_esdf, r.use_aabb, r.frame_id = True, False, True, BASE
        lo = np.array(GRID_CENTER) - np.array(GRID_SIZE) / 2
        r.aabb_min_m = Point(x=float(lo[0]), y=float(lo[1]), z=float(lo[2]))
        r.aabb_size_m = Vector3(x=float(GRID_SIZE[0]), y=float(GRID_SIZE[1]), z=float(GRID_SIZE[2]))
        self.esdf_pending, t0 = True, time.monotonic()
        self.esdf_cli.call_async(r).add_done_callback(lambda f: self.on_esdf(f, t0))

    def on_esdf(self, fut, t0):
        self.esdf_pending = False
        try: r = fut.result()
        except Exception as e: self.get_logger().error(f"ESDF call failed: {e}"); return
        if r is None or not r.success: self.get_logger().warn("ESDF service returned failure", throttle_duration_sec=5.0); return
        if abs(r.voxel_size_m - VOXEL) > 1e-4: self.get_logger().error(f"ESDF voxel {r.voxel_size_m} != {VOXEL}"); return
        shape = [r.esdf_and_gradients.layout.dim[i].size for i in range(3)]
        if shape != list(self.grid_shape): self.get_logger().error(f"ESDF shape {shape} != cuRobo grid {self.grid_shape}"); return
        d = np.asarray(r.esdf_and_gradients.data, np.float32).reshape(shape)            # nvblox: + outside, - inside, -1000 unknown
        origin = np.array([r.origin_m.x, r.origin_m.y, r.origin_m.z])
        grid = esdf_to_voxelgrid(d, origin, self.tensor_args.device)
        with self.gpu: self.world.update_voxel_data(grid)
        with self.lock: self.esdf = (d, origin)
        self.stats.update(esdf_ms=1000 * (time.monotonic() - t0), occ=int((d < 0).sum() - (d < -999.9).sum()), esdf_n=self.stats["esdf_n"] + 1)

    def clearance(self, q):
        """per-link min (ESDF distance at sphere centre - radius) for joint config q, from the last ESDF (CPU, nearest voxel)."""
        with self.lock: e = self.esdf
        if e is None: return None, None
        d, origin = e
        with self.gpu: sph = self.robot.get_state(self.tensor_args.to_device(q[None])).link_spheres_tensor[0].cpu().numpy()   # (n, 4): x y z r, fr3_link0
        clear = sphere_clearance(sph, d, origin)
        per = {}
        for name, c in zip(self.sphere_link, clear):
            if np.isfinite(c): per[name] = min(per.get(name, np.inf), c)
        return per, (sph, clear)

    # ---- MPC (preview only) -------------------------------------------------------------------------------------------
    def step(self):
        with self.lock:
            q, gp, dirty = (None if self.q_meas is None else self.q_meas.copy()), self.goal_pose, self.goal_dirty
            self.goal_dirty = False
        if q is None or self.esdf is None: return
        with self.gpu: self._step(q, gp, dirty)

    def _step(self, q, gp, dirty):
        state = CuJointState.from_position(self.tensor_args.to_device(q[None]), joint_names=self.jn)
        if self.goal_buffer is None or dirty:
            if gp is None:                                                                   # default: hold the current TCP
                ks = self.mpc.rollout_fn.compute_kinematics(state)
                goal_pose = Pose(ks.ee_pos_seq, quaternion=ks.ee_quat_seq)
            else:
                goal_pose = Pose(self.tensor_args.to_device([gp[0]]), quaternion=self.tensor_args.to_device([gp[1]]))
            goal = Goal(current_state=state, goal_state=CuJointState.from_position(state.position.clone(), joint_names=self.jn),
                        goal_pose=goal_pose)
            if self.goal_buffer is None: self.goal_buffer = self.mpc.setup_solve_single(goal, 1)
            else: self.goal_buffer.goal_pose.copy_(goal_pose)
            self.mpc.update_goal(self.goal_buffer)
        t0 = time.monotonic()
        res = self.mpc.step(state, max_attempts=2)
        self.stats["ms"] = 1000 * (time.monotonic() - t0); self.stats["steps"] += 1
        try: self.stats["err"] = float(res.metrics.pose_error.max())
        except Exception: pass
        cmd = res.js_action.position[0].cpu().numpy() if res.js_action.position.dim() > 1 else res.js_action.position.cpu().numpy()
        m = JointState(); m.header.stamp = self.get_clock().now().to_msg(); m.name = list(self.jn); m.position = [float(v) for v in cmd]
        self.cmd_pub.publish(m)

    def report(self):
        with self.lock: q = None if self.q_meas is None else self.q_meas.copy()
        if q is None: self.get_logger().info("waiting for /joint_states (relay running?)"); return
        if self.esdf is None: self.get_logger().info(f"waiting for the ESDF (service ready: {self.esdf_cli.service_is_ready()})"); return
        per, (sph, clear) = self.clearance(q)
        worst = sorted(per.items(), key=lambda kv: kv[1])[:3]
        n_in = int(np.nansum(clear < 0))
        self.get_logger().info(
            f"ESDF {self.stats['occ']} occupied voxels ({self.stats['esdf_ms']:.0f} ms) | arm vs map: {n_in}/{len(clear)} spheres inside obstacles, "
            f"tightest " + ", ".join(f"{k} {v * 100:+.1f} cm" for k, v in worst) +
            f" | MPC {self.stats['ms']:.0f} ms/step, pose err {self.stats['err'] * 100:.1f} cm (preview only)")
        ma = MarkerArray(); now = self.get_clock().now().to_msg()
        for i, (s, c) in enumerate(zip(sph, clear)):
            mk = Marker(); mk.header.frame_id = BASE; mk.header.stamp = now; mk.ns = "fr3_spheres"; mk.id = i; mk.type = Marker.SPHERE
            mk.pose.position.x, mk.pose.position.y, mk.pose.position.z = (float(v) for v in s[:3]); mk.pose.orientation.w = 1.0
            mk.scale.x = mk.scale.y = mk.scale.z = float(2 * s[3])
            mk.color = ColorRGBA(r=1.0, g=0.1, b=0.1, a=0.8) if (np.isfinite(c) and c < 0) else ColorRGBA(r=0.1, g=0.8, b=0.2, a=0.35)
            ma.markers.append(mk)
        self.sph_pub.publish(ma)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--urdf", default=os.path.join(HERE, "fr3.urdf"))
    ap.add_argument("--rate", type=float, default=20.0)
    ap.add_argument("--esdf-rate", type=float, default=1.0)
    a, ros = ap.parse_known_args()
    rclpy.init(args=ros); n = Fr3CuroboMpc(a)
    ex = MultiThreadedExecutor(num_threads=4); ex.add_node(n)
    try: ex.spin()
    except KeyboardInterrupt: pass


if __name__ == "__main__":
    main()
