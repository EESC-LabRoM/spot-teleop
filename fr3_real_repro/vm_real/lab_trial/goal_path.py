"""Multi-waypoint goal publisher + preview logger for the lab trial (node domain 142). Like goal_run.py but a whole path.
Waypoints are offsets (dx dy dz, metres) from P0, each with a move time and a dwell: --wps "dx,dy,dz,move_s,dwell_s;..." . Orientation fixed.
Minimum-jerk between waypoints. If the preview stream is silent for >0.5 s the goal freezes where it is.
python3 goal_path.py --p0 x y z --q0 qx qy qz qw --out DIR --hold 10 --wps "0,0.1,0,3,2;..." """
import argparse, csv, time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from trajectory_msgs.msg import JointTrajectory
ap = argparse.ArgumentParser()
ap.add_argument("--p0", type=float, nargs=3, required=True); ap.add_argument("--q0", type=float, nargs=4, required=True); ap.add_argument("--out", required=True)
ap.add_argument("--hold", type=float, default=10); ap.add_argument("--wps", required=True); ap.add_argument("--rate", type=float, default=30); ap.add_argument("--tail", type=float, default=3)
a, _ = ap.parse_known_args()
def mj(u): u = max(0.0, min(1.0, u)); return 10 * u**3 - 15 * u**4 + 6 * u**5
WP = [tuple(float(x) for x in w.split(",")) for w in a.wps.split(";") if w.strip()]
# timeline: (t_start_move, t_end_move, from, to)
SEG, t, cur = [], a.hold, (0.0, 0.0, 0.0)
for dx, dy, dz, mv, dw in WP:
    SEG.append((t, t + mv, cur, (dx, dy, dz), dw)); t += mv + dw; cur = (dx, dy, dz)
T_END = t + a.tail
def offset(t):
    pos = (0.0, 0.0, 0.0)
    for s, e, f, to, dw in SEG:
        if t < s: return f if False else pos
        if t < e: u = mj((t - s) / (e - s)); return tuple(f[i] + (to[i] - f[i]) * u for i in range(3))
        pos = to
    return pos
if __name__ == "__main__" and not hasattr(rclpy, "_skip"):
    class G(Node):
        def __init__(self):
            super().__init__("claude_goal_path")
            self.pub = self.create_publisher(PoseStamped, "/fr3_joystick_mpc/wrist_goal", 1)
            self.create_subscription(JointTrajectory, "/fr3_joystick_mpc/joint_preview", self.on_prev, 10)
            self.t0 = time.time(); self.last_prev = time.time(); self.frozen = None; self.np = 0
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
            o = offset(t)
            if self.frozen is None and self.np > 0 and t > a.hold and time.time() - self.last_prev > 0.5: self.frozen = o; self.get_logger().error("preview stream silent: goal frozen")
            if self.frozen is not None: o = self.frozen
            m = PoseStamped(); m.header.frame_id = "fr3_link0"; m.header.stamp = self.get_clock().now().to_msg()
            m.pose.position.x, m.pose.position.y, m.pose.position.z = a.p0[0] + o[0], a.p0[1] + o[1], a.p0[2] + o[2]
            m.pose.orientation.x, m.pose.orientation.y, m.pose.orientation.z, m.pose.orientation.w = a.q0
            self.pub.publish(m); self.gw.writerow([f"{time.time():.4f}", f"{m.pose.position.x:.5f}", f"{m.pose.position.y:.5f}", f"{m.pose.position.z:.5f}", int(self.frozen is not None)])
    rclpy.init(); n = G()
    try: rclpy.spin(n)
    except (SystemExit, KeyboardInterrupt): pass
