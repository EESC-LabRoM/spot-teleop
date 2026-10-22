#!/usr/bin/env python3
"""Return the FR3 to its home pose WITHOUT a controller swap: the equilibrium target of cartesian_impedance_target_controller is moved
from the measured hand pose (franka_robot_state_broadcaster/current_pose = O_T_EE = fr3_hand_tcp) to the hand pose of Paiva's home joint
configuration (fr3_kin FK), along a straight line with a minimum-jerk profile, orientation by slerp. Same pattern as real_lift.py.

Why: the previous version (kept as fr3_home_jtc.py) swapped to a position JointTrajectoryController, and on the real arm libfranka aborted
it twice on 10-15 with joint_motion_generator_acceleration_discontinuity, leaving the hardware deactivated (real-recover needed).
Here nothing is switched; the impedance controller stays active the whole time.

Limits: the hand pose is reached, the elbow is left to the controller's nullspace (it may differ slightly from the joint home).
Stops and re-targets the MEASURED pose (releases the spring) if: robot_mode leaves 2, the state goes stale, the hand lags the target by
more than --max-lag for --lag-time, or SIGINT/SIGTERM (Back on the pad). Publishes only equilibrium_pose; the teleop must be disengaged.
  python3 /lab/fr3_home.py [--dry-run] [--speed 0.08] [--pose a,b,c,d,e,f,g]"""
import argparse
import os
import signal
import sys
import time

import numpy as np
import rclpy
from controller_manager_msgs.srv import ListControllers
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

try:
    from franka_msgs.msg import FrankaRobotState
except ImportError:
    FrankaRobotState = None

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.environ.get("FR3_LAB_DIR", "/lab"))
import fr3_kin as K  # noqa: E402

HOME = [0.0, -0.7853981634, 0.0, -2.3561944902, 0.0, 1.5707963268, 0.7853981634]        # motion.yaml home_position.poses.home (Paiva)
Z_MIN = 0.05                                                                              # m, hand target never below this (base frame)


def mat2quat(R):                                                                          # -> [x, y, z, w]
    t = np.trace(R)
    if t > 0:
        s = 2 * np.sqrt(t + 1); w = s / 4; x = (R[2, 1] - R[1, 2]) / s; y = (R[0, 2] - R[2, 0]) / s; z = (R[1, 0] - R[0, 1]) / s
    else:
        i = int(np.argmax(np.diag(R))); j, k = (i + 1) % 3, (i + 2) % 3
        s = 2 * np.sqrt(1 + R[i, i] - R[j, j] - R[k, k]); v = np.zeros(3); v[i] = s / 4
        v[j] = (R[j, i] + R[i, j]) / s; v[k] = (R[k, i] + R[i, k]) / s; w = (R[k, j] - R[j, k]) / s; x, y, z = v
    q = np.array([x, y, z, w]); return q / np.linalg.norm(q)


def qangle(a, b): return 2 * np.arccos(min(1.0, abs(float(np.dot(a, b)))))


def qslerp(a, b, t):
    d = float(np.dot(a, b))
    if d < 0: b, d = -b, -d
    if d > 0.9995: q = a + t * (b - a); return q / np.linalg.norm(q)
    th = np.arccos(d); return (np.sin((1 - t) * th) * a + np.sin(t * th) * b) / np.sin(th)


class Home(Node):
    def __init__(self, a):
        super().__init__("fr3_home")
        self.a, self.abort, self.mode, self.pose, self.t_pose = a, False, None, None, 0.0
        self.pub = self.create_publisher(PoseStamped, f"/{a.controller}/equilibrium_pose", 1)
        self.create_subscription(PoseStamped, "/franka_robot_state_broadcaster/current_pose", self._pose, qos_profile_sensor_data)
        if FrankaRobotState is not None:
            self.create_subscription(FrankaRobotState, "/franka_robot_state_broadcaster/robot_state",
                                     lambda m: setattr(self, "mode", int(m.robot_mode)), qos_profile_sensor_data)
        self.list_cli = self.create_client(ListControllers, "/controller_manager/list_controllers")

    def _pose(self, m):
        o = m.pose.orientation
        self.pose = (np.array([m.pose.position.x, m.pose.position.y, m.pose.position.z]), np.array([o.x, o.y, o.z, o.w]))
        self.t_pose = time.monotonic()

    def log(self, s): self.get_logger().info(s)

    def spin_for(self, s):
        end = time.monotonic() + s
        while time.monotonic() < end and rclpy.ok(): rclpy.spin_once(self, timeout_sec=0.005)

    def send(self, p, q):
        m = PoseStamped(); m.header.stamp = self.get_clock().now().to_msg(); m.header.frame_id = "fr3_link0"
        m.pose.position.x, m.pose.position.y, m.pose.position.z = (float(c) for c in p)
        m.pose.orientation.x, m.pose.orientation.y, m.pose.orientation.z, m.pose.orientation.w = (float(c) for c in q)
        self.pub.publish(m)

    def hold_measured(self):                                                              # release the spring where the arm is
        if self.pose is not None:
            for _ in range(5): self.send(*self.pose); self.spin_for(0.02)

    def controller_active(self):
        if not self.list_cli.wait_for_service(timeout_sec=5.0): return None
        f = self.list_cli.call_async(ListControllers.Request()); rclpy.spin_until_future_complete(self, f, timeout_sec=5.0)
        if not f.done() or f.result() is None: return None
        return {c.name: c.state for c in f.result().controller}.get(self.a.controller) == "active"

    def run(self):
        a = self.a
        for _ in range(100):
            if self.pose is not None: break
            self.spin_for(0.05)
        if self.pose is None: return False, "no current_pose from franka_robot_state_broadcaster"
        if FrankaRobotState is not None:
            for _ in range(40):
                if self.mode is not None: break
                self.spin_for(0.05)
            if self.mode != 2: return False, f"robot_mode {self.mode} (need 2 = move, impedance controller running): recover first"
        act = self.controller_active()
        if act is not True: return False, f"{a.controller} not active ({act}): nothing is commanded"
        p0, q0 = self.pose[0].copy(), self.pose[1].copy()
        T = K.fk_tcp(np.array(a.pose)); p1, q1 = T[:3, 3].copy(), mat2quat(T[:3, :3])
        if np.dot(q0, q1) < 0: q1 = -q1
        dist, ang = float(np.linalg.norm(p1 - p0)), qangle(q0, q1)
        dur = max(a.min_duration, 1.875 * dist / a.speed, 1.875 * ang / np.radians(a.rot_speed_deg))  # min-jerk peak = 1.875 x mean
        self.log(f"hand {np.round(p0, 3).tolist()} -> home {np.round(p1, 3).tolist()}: {dist * 100:.1f} cm, {np.degrees(ang):.0f} deg, {dur:.1f} s")
        if min(p0[2], p1[2]) < Z_MIN: return False, f"start or home hand height below {Z_MIN} m: move the hand up first"
        if a.dry_run: return True, "dry run: plan ok, nothing published"
        t0 = time.monotonic(); lag_since = None; period = 1.0 / a.rate
        while rclpy.ok():
            now = time.monotonic(); t = now - t0
            if self.abort: self.hold_measured(); return False, "aborted (Back / signal): holding where the arm is"
            if now - self.t_pose > 0.5: self.hold_measured(); return False, "robot state stale: holding"
            if self.mode is not None and self.mode != 2: return False, f"robot_mode {self.mode}: stopped"
            s = min(1.0, t / dur); e = 10 * s ** 3 - 15 * s ** 4 + 6 * s ** 5
            p, q = p0 + (p1 - p0) * e, qslerp(q0, q1, e)
            self.send(p, q)
            lag = float(np.linalg.norm(self.pose[0] - p))
            rotation_lag = qangle(self.pose[1], q)
            if lag > a.max_lag or rotation_lag > np.radians(10):
                lag_since = lag_since or now
                if now - lag_since > a.lag_time: self.hold_measured(); return False, f"hand not following ({lag * 100:.1f} cm, {np.degrees(rotation_lag):.1f} deg behind): holding"
            else:
                lag_since = None
            if t > dur + a.settle: break
            self.spin_for(max(0.0, period - (time.monotonic() - now)))
        err = float(np.linalg.norm(self.pose[0] - p1)) * 100
        rot_err = np.degrees(qangle(self.pose[1], q1))
        if err > 1.5 or rot_err > 5:
            self.hold_measured(); return False, f"home not reached ({err:.1f} cm, {rot_err:.1f} deg): holding"
        return True, f"home reached (hand {err:.1f} cm from target); teleop stays disengaged, Start engages"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--pose", type=lambda s: [float(x) for x in s.split(",")], default=HOME)
    ap.add_argument("--speed", type=float, default=0.03, help="m/s peak hand speed (hard limit 0.15)")
    ap.add_argument("--rot-speed-deg", type=float, default=6.0, help="deg/s peak rotation speed (hard limit 45)")
    ap.add_argument("--min-duration", type=float, default=3.0); ap.add_argument("--settle", type=float, default=1.0)
    ap.add_argument("--max-lag", type=float, default=0.03); ap.add_argument("--lag-time", type=float, default=0.6)
    ap.add_argument("--rate", type=float, default=100.0)
    ap.add_argument("--controller", default="cartesian_impedance_target_controller")
    ap.add_argument("--dry-run", action="store_true")
    a, ros = ap.parse_known_args()
    if len(a.pose) != 7: print("--pose needs 7 joint angles"); return 2
    a.speed = min(abs(a.speed), 0.15); a.rot_speed_deg = min(abs(a.rot_speed_deg), 45.0)
    rclpy.init(args=ros); n = Home(a)
    def stop(*_): n.abort = True
    signal.signal(signal.SIGINT, stop); signal.signal(signal.SIGTERM, stop)
    try: ok, msg = n.run()
    except Exception as e: n.hold_measured(); ok, msg = False, f"error: {e}"
    (n.log if ok else n.get_logger().error)(("HOME OK: " if ok else "HOME FAILED: ") + msg)
    n.destroy_node(); rclpy.shutdown()
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
