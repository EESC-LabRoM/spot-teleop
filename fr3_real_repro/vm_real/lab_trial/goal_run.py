"""Goal publisher + preview logger for the lab trial. Runs in a container in the MPC node's domain (142).
Publishes PoseStamped on /fr3_joystick_mpc/wrist_goal at 30 Hz: hold at P0 for --hold s, minimum-jerk move to P0+dx over --ramp s, dwell, back, tail.
Logs every /fr3_joystick_mpc/joint_preview message (arrival time, q, v). If the preview stream is silent for >0.5 s during the motion, the goal is frozen.
python3 goal_run.py --p0 x y z --q0 qx qy qz qw --out DIR [--dx 0.05 --hold 60 --ramp 3 --dwell 6 --tail 10]"""
import argparse, csv, time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from trajectory_msgs.msg import JointTrajectory
ap = argparse.ArgumentParser()
ap.add_argument("--p0", type=float, nargs=3, required=True); ap.add_argument("--q0", type=float, nargs=4, required=True); ap.add_argument("--out", required=True)
ap.add_argument("--dx", type=float, default=0.05); ap.add_argument("--hold", type=float, default=60); ap.add_argument("--ramp", type=float, default=3)
ap.add_argument("--dwell", type=float, default=6); ap.add_argument("--tail", type=float, default=10); ap.add_argument("--rate", type=float, default=30)
a, _ = ap.parse_known_args()

def mj(u): u = max(0.0, min(1.0, u)); return 10 * u**3 - 15 * u**4 + 6 * u**5
def offset(t):
    """x offset (m) of the goal at t seconds after start"""
    t -= a.hold
    if t < 0: return 0.0
    if t < a.ramp: return a.dx * mj(t / a.ramp)
    t -= a.ramp
    if t < a.dwell: return a.dx
    t -= a.dwell
    if t < a.ramp: return a.dx * (1 - mj(t / a.ramp))
    return 0.0
T_END = a.hold + 2 * a.ramp + a.dwell + a.tail

class G(Node):
    def __init__(self):
        super().__init__("claude_goal_run")
        self.pub = self.create_publisher(PoseStamped, "/fr3_joystick_mpc/wrist_goal", 1)
        self.create_subscription(JointTrajectory, "/fr3_joystick_mpc/joint_preview", self.on_prev, 10)
        self.t0 = time.time(); self.last_prev = 0.0; self.frozen = None; self.np = 0
        self.gf = open(a.out + "/goal.csv", "w"); self.gw = csv.writer(self.gf); self.gw.writerow(["t", "x", "y", "z", "frozen"])
        self.pf = open(a.out + "/preview.csv", "w"); self.pw = csv.writer(self.pf); self.pw.writerow(["t"] + [f"q{i}" for i in range(7)] + [f"v{i}" for i in range(7)])
        self.create_timer(1.0 / a.rate, self.tick)
    def on_prev(self, m):
        p = m.points[0]; self.last_prev = time.time(); self.np += 1
        self.pw.writerow([f"{self.last_prev:.4f}"] + [f"{x:.6f}" for x in list(p.positions) + list(p.velocities)])
        if self.np % 200 == 0: self.pf.flush()
    def tick(self):
        t = time.time() - self.t0
        if t > T_END: self.gf.flush(); self.pf.flush(); self.get_logger().info(f"done: {self.np} preview msgs"); raise SystemExit
        x = offset(t)
        if self.frozen is None and t > a.hold and time.time() - self.last_prev > 0.5: self.frozen = x; self.get_logger().error("preview stream silent: goal frozen")
        if self.frozen is not None: x = self.frozen
        m = PoseStamped(); m.header.frame_id = "fr3_link0"; m.header.stamp = self.get_clock().now().to_msg()
        m.pose.position.x, m.pose.position.y, m.pose.position.z = a.p0[0] + x, a.p0[1], a.p0[2]
        m.pose.orientation.x, m.pose.orientation.y, m.pose.orientation.z, m.pose.orientation.w = a.q0
        self.pub.publish(m); self.gw.writerow([f"{time.time():.4f}", f"{m.pose.position.x:.5f}", f"{m.pose.position.y:.5f}", f"{m.pose.position.z:.5f}", int(self.frozen is not None)])
rclpy.init(); n = G()
try: rclpy.spin(n)
except (SystemExit, KeyboardInterrupt): pass
