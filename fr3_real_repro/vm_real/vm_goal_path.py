"""VM-side goal path sender for the real FR3 via VM 103 (ROS domain 143).

Port of lab_trial/goal_path.py: same waypoint timeline (offsets from the
current hand pose P0, minimum-jerk legs), but publishes /wrist_pose (best
effort, depth 1, the QoS the node subscribes with) and watches the VM
preview /curobo/joint_trajectory. P0 comes from live /joint_states via
cuRobo's own FK, so no hand-measured origin is needed.

Runs INSIDE the vm-mpc container (has GPU + cuRobo + rclpy):
  docker exec -e ROS_DOMAIN_ID=143 ... vm-mpc /isaac-sim/python.sh \\
    /node/../vm_goal_path.py --out /out/traj_ID --wps "..."   # see vm_real.sh go_traj

If the preview stream is silent for >0.5 s after the hold, the goal freezes.
"""
import argparse, csv, math, time
import torch
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory
from curobo.cuda_robot_model.cuda_robot_model import CudaRobotModel, CudaRobotModelConfig
from curobo.types.base import TensorDeviceType
from curobo.util_file import load_yaml

ap = argparse.ArgumentParser()
ap.add_argument("--out", required=True)
ap.add_argument("--cfg", default="/workspace/franka_mpc/node_config/fr3_vcap1.2_acc10.yml")
ap.add_argument("--wps", required=True, help="dx,dy,dz,move_s,dwell_s;...")
ap.add_argument("--hold", type=float, default=10)
ap.add_argument("--rate", type=float, default=30)
ap.add_argument("--tail", type=float, default=3)
a, _ = ap.parse_known_args()

def mj(u):
    u = max(0.0, min(1.0, u)); return 10 * u**3 - 15 * u**4 + 6 * u**5

WP = [tuple(float(x) for x in w.split(",")) for w in a.wps.split(";") if w.strip()]
SEG, t, cur = [], a.hold, (0.0, 0.0, 0.0)
for dx, dy, dz, mv, dw in WP:
    SEG.append((t, t + mv, cur, (dx, dy, dz), dw)); t += mv + dw; cur = (dx, dy, dz)
T_END = t + a.tail

def offset(t):
    pos = (0.0, 0.0, 0.0)
    for s, e, f, to, dw in SEG:
        if t < s:
            return pos
        if t < e:
            u = mj((t - s) / (e - s))
            return tuple(f[i] + (to[i] - f[i]) * u for i in range(3))
        pos = to
    return pos

QOS = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                 durability=DurabilityPolicy.VOLATILE, depth=1)

class G(Node):
    def __init__(self, p0, q0):
        super().__init__("vm_goal_path")
        self.p0, self.q0 = p0, q0
        self.pub = self.create_publisher(PoseStamped, "/wrist_pose", QOS)
        self.create_subscription(JointTrajectory, "/curobo/joint_trajectory", self.on_prev, 10)
        self.t0 = time.time(); self.last_prev = time.time(); self.frozen = None; self.np = 0
        self.gf = open(a.out + "/goal.csv", "w"); self.gw = csv.writer(self.gf)
        self.gw.writerow(["t", "x", "y", "z", "frozen"])
        self.pf = open(a.out + "/preview.csv", "w"); self.pw = csv.writer(self.pf)
        self.pw.writerow(["t"] + [f"q{i}" for i in range(7)] + [f"v{i}" for i in range(7)])
        self.create_timer(1.0 / a.rate, self.tick)

    def on_prev(self, m):
        p = m.points[0]; self.last_prev = time.time(); self.np += 1
        self.pw.writerow([f"{self.last_prev:.4f}"] + [f"{x:.6f}" for x in list(p.positions) + list(p.velocities)])
        if self.np % 200 == 0:
            self.pf.flush()

    def tick(self):
        t = time.time() - self.t0
        if t > T_END:
            self.gf.flush(); self.pf.flush()
            self.get_logger().info(f"done: {self.np} preview msgs, frozen={self.frozen is not None}")
            raise SystemExit
        o = offset(t)
        if self.frozen is None and self.np > 0 and t > a.hold and time.time() - self.last_prev > 0.5:
            self.frozen = o; self.get_logger().error("preview stream silent: goal frozen")
        if self.frozen is not None:
            o = self.frozen
        m = PoseStamped(); m.header.frame_id = "fr3_link0"; m.header.stamp = self.get_clock().now().to_msg()
        m.pose.position.x, m.pose.position.y, m.pose.position.z = self.p0[0] + o[0], self.p0[1] + o[1], self.p0[2] + o[2]
        m.pose.orientation.x, m.pose.orientation.y, m.pose.orientation.z, m.pose.orientation.w = self.q0
        self.pub.publish(m)
        self.gw.writerow([f"{time.time():.4f}", f"{m.pose.position.x:.5f}",
                          f"{m.pose.position.y:.5f}", f"{m.pose.position.z:.5f}", int(self.frozen is not None)])

def get_p0():
    rclpy.init()
    n = rclpy.create_node("vm_p0")
    box = {}
    def cb(m):
        box["m"] = m
    n.create_subscription(JointState, "/joint_states", cb, 10)
    t0 = time.time()
    while "m" not in box and time.time() - t0 < 15:
        rclpy.spin_once(n, timeout_sec=0.5)
    n.destroy_node()
    if "m" not in box:
        raise RuntimeError("no /joint_states in 15 s")
    m = box["m"]
    tdev = TensorDeviceType()
    kin = load_yaml(a.cfg)["robot_cfg"]["kinematics"]
    model = CudaRobotModel(CudaRobotModelConfig.from_data_dict(kin, tdev))
    qd = dict(zip(m.name, m.position))
    q = [qd.get(j, 0.04 if "finger" in j else 0.0) for j in model.joint_names]
    qt = torch.tensor([q], device=tdev.device, dtype=tdev.dtype)
    s = model.get_state(qt[:, :len(model.joint_names)])
    p = s.ee_position[0].tolist(); w = s.ee_quaternion[0].tolist()  # cuRobo quat order w,x,y,z
    return p, (w[1], w[2], w[3], w[0])

if __name__ == "__main__":
    p0, q0 = get_p0()
    print(f"P0 {p0[0]:.5f} {p0[1]:.5f} {p0[2]:.5f} Q0 {' '.join(f'{x:.5f}' for x in q0)}", flush=True)
    n = G(p0, q0)
    try:
        rclpy.spin(n)
    except (SystemExit, KeyboardInterrupt):
        pass
