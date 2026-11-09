"""Goal source for the AVOIDANCE sim with the real objects of 10-31 (drill, bottle, box in the live map): /wrist_pose (PoseStamped, fr3_link0, 30 Hz, sim time),
starting at the ready-pose hand, then: approach over the table, a low sweep along y through the drill's position (the goal itself is inside the drill),
a second low pass near the bottle, and back to the ready pose. min-jerk legs, holds in between.
  python goal_obstacle.py --ros-args -p use_sim_time:=true"""
import rclpy, numpy as np
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
P0 = np.array([0.30689, 0.0, 0.59028]); Q0 = (-1.0, 0.0, 0.0, 0.0)          # fr3_hand at the ready pose (fk_ready.py); tool z points down, fingertips 0.1034 m below
# (t_start, duration, absolute hand-flange target in fr3_link0); held until the next leg. Table top ~0.04, objects up to ~0.25 (bottle, drill, box).
LEGS = [(3.0, 4.0, (0.48, 0.30, 0.32)),     # over the table, above everything (fingertips 0.22)
        (9.0, 7.0, (0.48, -0.30, 0.22)),    # low sweep along y, goal inside the drill (fingertips 0.12; drill top 0.24 at x 0.43-0.57, y -0.33..-0.21)
        (18.0, 5.0, (0.42, 0.14, 0.22)),    # low goal at the bottle (bottle at 0.38, 0.14, top 0.25)
        (26.0, 5.0, tuple(P0))]             # back to the ready pose
T_END = 36.0
def mj(s): s = min(max(s, 0.0), 1.0); return 10 * s**3 - 15 * s**4 + 6 * s**5
def target(t):
    cur = P0.copy()
    for t0, d, tgt in LEGS:
        if t < t0: break
        tgt = np.array(tgt, float)
        if t < t0 + d: return cur + (tgt - cur) * mj((t - t0) / d)
        cur = tgt
    return cur
class G(Node):
    def __init__(self):
        super().__init__("goal_obstacle"); self.pub = self.create_publisher(PoseStamped, "/wrist_pose", 10); self.t0 = None
        self.create_timer(1 / 30, self.tick)
    def tick(self):
        now = self.get_clock().now().nanoseconds * 1e-9
        if self.t0 is None: self.t0 = now
        t = now - self.t0; p = target(t)
        m = PoseStamped(); m.header.frame_id = "fr3_link0"; m.header.stamp = self.get_clock().now().to_msg()
        m.pose.position.x, m.pose.position.y, m.pose.position.z = (float(v) for v in p)
        m.pose.orientation.x, m.pose.orientation.y, m.pose.orientation.z, m.pose.orientation.w = Q0
        self.pub.publish(m)
        if t > T_END + 1: raise SystemExit
rclpy.init(); n = G()
try: rclpy.spin(n)
except SystemExit: pass
