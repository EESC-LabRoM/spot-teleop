"""Goal source for the deployment-path sim: /wrist_pose (PoseStamped, fr3_link0, 30 Hz, sim time) starting at the ready-pose hand, then
moves like an operator: +5 cm x, back, a fast +12 cm y, back; with holds in between (min-jerk legs).
  python goal_step.py --ros-args -p use_sim_time:=true"""
import math, rclpy, numpy as np
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
P0 = np.array([0.30689, 0.0, 0.59028]); Q0 = (-1.0, 0.0, 0.0, 0.0)          # cuRobo FK of fr3_hand at the ready pose (fk_ready.py)
# (t_start, duration, offset to reach) - held until the next leg
LEGS = [(6.0, 3.0, (0.05, 0, 0)), (15.0, 3.0, (0, 0, 0)), (24.0, 1.0, (0, 0.12, 0)), (31.0, 2.0, (0, 0, 0))]
T_END = 40.0
def mj(s): s = min(max(s, 0.0), 1.0); return 10 * s**3 - 15 * s**4 + 6 * s**5
def offset(t):
    cur = np.zeros(3)
    for t0, d, tgt in LEGS:
        if t < t0: break
        prev = cur.copy(); cur = prev + (np.array(tgt) - prev) * mj((t - t0) / d) if t < t0 + d else np.array(tgt, float)
        if t >= t0 + d: continue
        break
    return cur
class G(Node):
    def __init__(self):
        super().__init__("goal_step"); self.pub = self.create_publisher(PoseStamped, "/wrist_pose", 10); self.t0 = None
        self.create_timer(1 / 30, self.tick)
    def tick(self):
        now = self.get_clock().now().nanoseconds * 1e-9
        if self.t0 is None: self.t0 = now
        t = now - self.t0; p = P0 + offset(t)
        m = PoseStamped(); m.header.frame_id = "fr3_link0"; m.header.stamp = self.get_clock().now().to_msg()
        m.pose.position.x, m.pose.position.y, m.pose.position.z = (float(v) for v in p)
        m.pose.orientation.x, m.pose.orientation.y, m.pose.orientation.z, m.pose.orientation.w = Q0
        self.pub.publish(m)
        if t > T_END + 1: raise SystemExit
rclpy.init(); n = G()
try: rclpy.spin(n)
except SystemExit: pass
