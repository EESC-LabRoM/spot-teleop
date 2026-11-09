"""Goal source for the GRASP dry run on the live 10-31 map: /wrist_pose (PoseStamped, fr3_link0, 30 Hz, sim time).

Starts at the hand pose passed via P0 (default: live pose at capture), holds, flies to above the GraspGen
drill grasp, descends 5 cm onto it, holds (the grasp point), then lifts empty (touch-test, no pick).
min-jerk legs. P0/QUAT from env (comma floats), tool-down orientation throughout.
  P0="0.30722,-0.00169,0.58725" python goal_grasp.py --ros-args -p use_sim_time:=true"""
import os, rclpy, numpy as np
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
P0 = np.array([float(v) for v in os.environ.get("P0", "0.30722,-0.00169,0.58725").split(",")])
Q0 = tuple(float(v) for v in os.environ.get("QUAT", "-1.0,0.0,0.0,0.0").split(","))
GRASP = np.array([0.516, -0.286, 0.345])   # GraspGen pick, conf 0.83, vertical
ABOVE = GRASP + np.array([0.0, 0.0, 0.05])
LIFT = GRASP + np.array([0.0, 0.0, 0.105])
# (t_start, duration, absolute hand-flange target)
LEGS = [(8.0, 8.0, tuple(ABOVE)),    # fly to above the grasp
        (19.0, 5.0, tuple(GRASP)),   # 5 cm straight down onto the grasp
        (29.0, 5.0, tuple(LIFT))]    # empty lift (touch-test ending)
T_END = 37.0
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
        super().__init__("goal_grasp"); self.pub = self.create_publisher(PoseStamped, "/wrist_pose", 10); self.t0 = None
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
