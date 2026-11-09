"""Goal source for the AVOIDANCE sim with the real objects: the SAME goal logic as the real run (fr3_real_repro/vm_real/vm_goal_path.py): /wrist_pose (PoseStamped, fr3_link0, 30 Hz),
hand = P0 + minimum-jerk offsets through WPS = "dx,dy,dz,move_s,dwell_s;...", initial HOLD at P0, orientation held at the start orientation.
  env P0="x,y,z" QUAT="qx,qy,qz,qw" WPS=... HOLD=10 TAIL=3   python goal_obstacle.py --ros-args -p use_sim_time:=true"""
import os, rclpy, numpy as np
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
P0 = np.array([float(x) for x in os.environ["P0"].split(",")]); Q = [float(x) for x in os.environ["QUAT"].split(",")]
HOLD = float(os.environ.get("HOLD", "10")); TAIL = float(os.environ.get("TAIL", "3"))
WP = [tuple(float(x) for x in w.split(",")) for w in os.environ["WPS"].split(";") if w.strip()]
SEG, t, cur = [], HOLD, (0.0, 0.0, 0.0)
for dx, dy, dz, mv, dw in WP: SEG.append((t, t + mv, cur, (dx, dy, dz), dw)); t += mv + dw; cur = (dx, dy, dz)
T_END = t + TAIL
def mj(u): u = max(0.0, min(1.0, u)); return 10 * u**3 - 15 * u**4 + 6 * u**5
def offset(t):
    pos = (0.0, 0.0, 0.0)
    for s, e, f, to, dw in SEG:
        if t < s: return pos
        if t < e: u = mj((t - s) / (e - s)); return tuple(f[i] + (to[i] - f[i]) * u for i in range(3))
        pos = to
    return pos
class G(Node):
    def __init__(self):
        super().__init__("goal_obstacle"); self.pub = self.create_publisher(PoseStamped, "/wrist_pose", 10); self.t0 = None
        self.create_timer(1 / 30, self.tick)
    def tick(self):
        now = self.get_clock().now().nanoseconds * 1e-9
        if self.t0 is None: self.t0 = now
        t = now - self.t0; p = P0 + np.array(offset(t))
        m = PoseStamped(); m.header.frame_id = "fr3_link0"; m.header.stamp = self.get_clock().now().to_msg()
        m.pose.position.x, m.pose.position.y, m.pose.position.z = (float(v) for v in p)
        m.pose.orientation.x, m.pose.orientation.y, m.pose.orientation.z, m.pose.orientation.w = Q
        self.pub.publish(m)
        if t > T_END + 1: raise SystemExit
print(f"goal_obstacle: P0 {P0.tolist()} legs {len(SEG)} T_END {T_END:.1f} s", flush=True)
rclpy.init(); n = G()
try: rclpy.spin(n)
except SystemExit: pass
