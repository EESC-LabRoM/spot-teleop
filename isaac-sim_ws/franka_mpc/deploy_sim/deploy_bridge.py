"""Sits between the MPC node and the sim, standing in for  /curobo/joint_command -> curobo_to_jtc.py -> controller  on the robot.
Applies the REAL slew() from the deployed curobo_to_jtc.py (extracted by ast, not re-typed) and can inject faults in SIM time:
  --spike T      one bad packet at sim time T s after the first goal (every arm joint +0.3 rad)
  --stall T N    relay stall: for N control steps from T the last forwarded command is repeated (the controller holds its last target)
  --delay K      every command reaches the sim K control steps late (latency)
node publishes /cmd_raw (remapped), this republishes /joint_command_curobo. CSV: sim_t, raw q, forwarded q, clipped."""
import argparse, ast, collections, csv, math
import rclpy
from rclpy.node import Node
from rosgraph_msgs.msg import Clock
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory
ap = argparse.ArgumentParser()
ap.add_argument("--lab", default="/vr/lab/curobo_to_jtc.py"); ap.add_argument("--max-step", type=float, default=0.03); ap.add_argument("--period", type=float, default=0.02)
ap.add_argument("--spike", type=float, default=-1.0); ap.add_argument("--stall", type=float, nargs=2, default=None); ap.add_argument("--delay", type=int, default=0)
ap.add_argument("--max-accel", type=float, default=0.0, help="0 = old behaviour (position clip only)")
ap.add_argument("--traj", action="store_true", help="the node publishes trajectory_msgs/JointTrajectory (command_type:=joint_trajectory) on /cmd_raw")
ap.add_argument("--cmd-topic", default="/cmd_raw", help="where the node (or the lab-side forwarder, in the E2E wiring test) publishes")
ap.add_argument("--noslew", action="store_true", help="forward raw (old behaviour with a huge max-step)"); ap.add_argument("--out", default="/out/bridge.csv")
a, _ = ap.parse_known_args()
src = open(a.lab).read(); ns = {"math": math}
for n_ in ast.parse(src).body:
    if isinstance(n_, ast.FunctionDef) and n_.name in ("slew", "limit"): exec(ast.get_source_segment(src, n_), ns)   # the deployed functions, not re-typed
slew, limit = ns["slew"], ns["limit"]
ARM = [f"fr3_joint{i}" for i in range(1, 8)]
class B(Node):
    def __init__(self):
        super().__init__("deploy_bridge"); self.t = 0.0; self.t0 = None; self.last = None; self.hold = None; self.spiked = False; self.fifo = collections.deque(); self.n = 0; self.nclip = 0; self.nlim = 0; self.was_stalled = False
        self.f = open(a.out, "w"); self.w = csv.writer(self.f); self.w.writerow(["t", "clipped", "stalled", "limited"] + [f"raw{i}" for i in range(7)] + [f"fwd{i}" for i in range(7)])
        self.pub = self.create_publisher(JointState, "/joint_command_curobo", 10)
        self.create_subscription(Clock, "/clock", lambda m: setattr(self, "t", m.clock.sec + m.clock.nanosec * 1e-9), 50)
        self.create_subscription(PoseStamped, "/wrist_pose", self.on_goal, 10)
        if a.traj: self.create_subscription(JointTrajectory, a.cmd_topic, self.on_traj, 10)
        else: self.create_subscription(JointState, a.cmd_topic, self.on_raw, 10)
    def on_traj(self, m):
        p = m.points[0]; js = JointState(); js.header = m.header; js.name = list(m.joint_names); js.position = list(p.positions); js.velocity = list(p.velocities)
        self.on_raw(js)
    def on_goal(self, m):
        if self.t0 is None: self.t0 = self.t          # fault times are relative to the first goal (the sim clock runs ahead while idle)
    @property
    def rel(self): return -1.0 if self.t0 is None else self.t - self.t0
    def on_raw(self, m):
        idx = [m.name.index(j) for j in ARM]; q = [float(m.position[i]) for i in idx]; v = [float(m.velocity[i]) if len(m.velocity) else 0.0 for i in idx]; raw = list(q)
        if a.spike >= 0 and not self.spiked and self.rel >= a.spike: self.spiked = True; q = [x + 0.3 for x in q]
        stalled = a.stall is not None and a.stall[0] <= self.rel < a.stall[0] + a.stall[1] * a.period
        clipped = False; limf = False
        if stalled and self.last is not None: q, v = list(self.last[0]), list(self.last[1])           # relay stalled: the controller keeps its last target
        elif not a.noslew and self.last is not None:
            if self.was_stalled: self.last = (self.last[0], [0.0] * 7)   # same rule as emit(): after a gap the limiter restarts from rest
            if a.max_accel > 0:
                q, v, lim = limit(q, v, self.last[0], self.last[1], a.period, a.max_accel); self.nlim += lim; limf = lim
            q, clipped = slew(q, self.last[0], a.max_step)
            if clipped: vmax = a.max_step / a.period; v = [max(-vmax, min(vmax, x)) for x in v]
        self.last = (q, v); self.nclip += clipped; self.was_stalled = stalled
        out = JointState(); out.header = m.header; out.name = list(m.name); pos = list(m.position); vel = list(m.velocity) if len(m.velocity) else [0.0] * len(pos)
        for k, i in enumerate(idx): pos[i] = q[k]; vel[i] = v[k]
        out.position, out.velocity = pos, vel
        self.fifo.append(out)
        if len(self.fifo) > a.delay: self.pub.publish(self.fifo.popleft())
        self.n += 1; self.w.writerow([f"{self.rel:.3f}", int(clipped), int(stalled), int(limf)] + [f"{x:.5f}" for x in raw] + [f"{x:.5f}" for x in q])
        if self.n % 250 == 0: self.f.flush(); self.get_logger().info(f"forwarded {self.n}, clipped {self.nclip}, accel-limited {self.nlim}")
rclpy.init(); rclpy.spin(B())
