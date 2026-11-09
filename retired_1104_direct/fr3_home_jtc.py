#!/usr/bin/env python3
"""Return the FR3 to its home pose with a joint-space move, as Paiva's /teleop/home does (oopsie_dataset fr3_teleop._run_home):
disengage, plan a two-point trajectory to the pose in motion.yaml (default_pose: home) at 20 % of each joint's velocity limit
(at least 3 s, 1 s settle, targets 0.05 rad inside the limits), swap the controllers (the impedance controller releases the effort
interface, a JointTrajectoryController takes the position interface), run the move, and ALWAYS swap back. franka_hardware restarts
the libfranka loop on each swap, so the arm must be standing still (checked here). Back on the pad aborts the move (SIGINT: cancel, hold, restore).

  python3 /lab/fr3_home.py [--dry-run] [--no-check] [--pose a,b,c,d,e,f,g] [--speed-fraction 0.2]   (or ./fr3_ros2.sh real-home)
Checks before it touches the controllers: robot mode idle/move, arm standing still, joint limits, and a coarse model of the path
(fr3_kin.py: arm segments above the table, no self-collision). The model is approximate and does NOT see the valve, the hand's cargo or people.
If the impedance controller is stranded inactive (e.g. after a failed swap), it is re-activated first (soft-switch, no motion);
a missing controller, or a failed re-activation still refuse. If the JTC is
already active while the impedance controller is not holding (the direct/cuRobo
stack, where the JTC stays live), home commands the move through it with no
swap and no restore instead of refusing; both active at once still refuses."""
import argparse
import os
import signal
import subprocess
import sys
import time

import numpy as np
import rclpy
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from controller_manager_msgs.srv import ListControllers, SwitchController
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint

try:
    from franka_msgs.msg import FrankaRobotState
except ImportError:
    FrankaRobotState = None

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
try:
    import fr3_kin as K
except ImportError:
    K = None

def select_home_mode(states, pos, imp):
    """Home controller strategy (pure). Returns (mode, reason).

    swap: classic path, pos inactive -> swap imp->pos, move, swap back.
    direct: pos already live while imp is not holding (the direct/cuRobo
      stack, where the JTC stays active): command the move through it,
      no swap and no restore.
    refuse: pos already active while imp also holds -> a genuine second
      mover; keep the historical refusal message.
    """
    if states.get(pos) == "active":
        if states.get(imp) == "active":
            return "refuse", f"{pos} is already active (another move running?)"
        return "direct", (
            f"{pos} already active, {imp} not holding: home through the live "
            "controller, no swap (do not press Start during home)")
    return "swap", f"swap {imp} -> {pos}"


NAMES = [f"fr3_joint{i}" for i in range(1, 8)]
HOME = [0.0, -0.7853981634, 0.0, -2.3561944902, 0.0, 1.5707963268, 0.7853981634]        # motion.yaml home_position.poses.home (Paiva)
# franka_description/robots/fr3/joint_limits.yaml (lower, upper, velocity) per joint, as Paiva reads them from the URDF
LOWER = np.array([-2.9007, -1.8361, -2.9007, -3.0770, -2.8763, 0.4398, -3.0508])
UPPER = np.array([2.9007, 1.8361, 2.9007, -0.1169, 2.8763, 4.6216, 3.0508])
VEL = np.array([2.62, 2.62, 2.62, 2.62, 5.26, 4.18, 5.26])


class Home(Node):
    def __init__(self, a):
        super().__init__("fr3_home")
        self.a, self.q, self.mode, self.abort = a, None, None, False
        self.create_subscription(JointState, "/joint_states", self._js, 10)
        if FrankaRobotState is not None:
            from rclpy.qos import qos_profile_sensor_data
            self.create_subscription(FrankaRobotState, "/franka_robot_state_broadcaster/robot_state", self._rs, qos_profile_sensor_data)
        self.list_cli = self.create_client(ListControllers, "/controller_manager/list_controllers")
        self.sw_cli = self.create_client(SwitchController, "/controller_manager/switch_controller")
        self.ac = ActionClient(self, FollowJointTrajectory, f"/{a.position_controller}/follow_joint_trajectory")

    def _js(self, m):
        d = dict(zip(m.name, m.position))
        if all(n in d for n in NAMES): self.q = np.array([d[n] for n in NAMES])

    def _rs(self, m): self.mode = int(m.robot_mode)

    def log(self, s): self.get_logger().info(s)

    def spin_for(self, s):
        end = time.monotonic() + s
        while time.monotonic() < end and rclpy.ok(): rclpy.spin_once(self, timeout_sec=0.02)

    def call(self, cli, req, timeout=10.0):
        if not cli.wait_for_service(timeout_sec=5.0): raise RuntimeError(f"service {cli.srv_name} not available")
        f = cli.call_async(req); rclpy.spin_until_future_complete(self, f, timeout_sec=timeout)
        if not f.done(): raise RuntimeError(f"{cli.srv_name} timed out")
        return f.result()

    def states(self):
        # Discovery races drop service calls on a loaded domain (the teleop
        # node retries for the same reason); a single timeout must not fail home.
        last = None
        for _ in range(3):
            try:
                return {c.name: c.state for c in self.call(self.list_cli, ListControllers.Request()).controller}
            except RuntimeError as e:
                last = e
                time.sleep(2.0)
        raise last

    def switch(self, activate, deactivate):
        r = SwitchController.Request(); r.activate_controllers, r.deactivate_controllers = list(activate), list(deactivate)
        r.strictness = SwitchController.Request.STRICT; r.activate_asap = False; r.timeout = Duration(sec=5)
        return bool(self.call(self.sw_cli, r, 15.0).ok)

    def standing_still(self):
        self.q = None; self.spin_for(0.3); q0 = self.q
        self.spin_for(0.5); q1 = self.q
        if q0 is None or q1 is None: return None
        return float(np.abs(q1 - q0).max())

    def plan(self, start, target):
        target = np.clip(target, LOWER + self.a.margin, UPPER - self.a.margin)
        T = max(self.a.min_duration, float(np.max(np.abs(target - start) / (self.a.speed_fraction * VEL))))
        return target, T

    def path_check(self, start, target, T):
        """coarse model check along the quintic path: (worst floor margin m, worst self-collision margin m) or None without fr3_kin."""
        if K is None: return None
        fl = sc = 1e9
        for s in np.linspace(0.0, 1.0, 61):
            e = 10 * s ** 3 - 15 * s ** 4 + 6 * s ** 5; q = start + (target - start) * e
            fl, sc = min(fl, K.floor_margin(q)), min(sc, K.self_collision_margin(q))
        return fl, sc

    def run(self):
        a = self.a
        for _ in range(100):
            if self.q is not None: break
            self.spin_for(0.1)
        if self.q is None: return False, "no /joint_states"
        if self.mode is not None and self.mode not in (1, 2): return False, f"robot_mode {self.mode} (need 1 idle or 2 move): clear the error first (real-recover)"
        d = self.standing_still()
        if d is None or d > 0.003: return False, f"the arm is not standing still (moved {d} rad in 0.5 s): disengage and wait"
        start = self.q.copy(); target, T = self.plan(start, np.array(a.pose))
        self.log(f"start {np.round(start, 3).tolist()}")
        self.log(f"home  {np.round(target, 3).tolist()}  max joint change {np.degrees(np.abs(target - start).max()):.0f} deg, {T:.1f} s (+{a.settle:.0f} s settle)")
        if np.any(start < LOWER) or np.any(start > UPPER): return False, "start outside the joint limits"
        pc = None if a.no_check else self.path_check(start, target, T)
        if pc is not None:
            self.log(f"path model: lowest arm point {pc[0] * 100:+.1f} cm above the table plane, tightest self-collision gap {pc[1] * 100:+.1f} cm")
            if pc[0] < 0.0 or pc[1] < 0.0: return False, "the straight joint-space path fails the coarse model check (table or self-collision); move the arm by hand first, or --no-check"
        elif not a.no_check: self.log("fr3_kin.py not found: path model check skipped")
        if a.dry_run: return True, "dry run: plan ok, nothing switched"
        st = self.states()
        imp, pos = a.controller, a.position_controller
        mode, why = select_home_mode(st, pos, imp)
        if mode == "refuse": return False, why
        direct = (mode == "direct")
        if direct:
            self.log("HOME: " + why)
        else:
            if st.get(imp) != "active":
                if imp not in st: return False, f"{imp} not loaded: run real-recover"
                self.log(f"{imp} is {st.get(imp)}: soft-switching it back to hold first (no motion)")
                if not self.switch([imp], []): return False, f"{imp} did not re-activate (nothing switched): run real-recover"
                st = self.states()
                if st.get(imp) != "active": return False, f"{imp} did not re-activate (nothing switched): run real-recover"
                self.log(f"{imp} active again, continuing with the swap")
            if pos not in st:
                self.log(f"loading {pos}")
                r = subprocess.run(["ros2", "run", "controller_manager", "spawner", pos, "-p", a.params, "--inactive"], capture_output=True, text=True, timeout=40)
                if r.returncode != 0: return False, f"could not load {pos}: {(r.stdout + r.stderr).strip()[-200:]}"
                st = self.states()
            if st.get(pos) == "active": return False, f"{pos} is already active (another move running?)"
        stopped = False; ok, msg = False, "home failed"
        try:
            if not direct:
                self.log(f"HOME: {imp} -> {pos}; keep the workspace clear, e-stop in hand")
                if not self.switch([pos], [imp]): return False, "controller swap refused (nothing changed)"
            stopped = not direct  # direct mode: nothing swapped, nothing to restore
            if not self.ac.wait_for_server(timeout_sec=10.0): raise RuntimeError("trajectory action server did not appear")
            g = FollowJointTrajectory.Goal(); g.trajectory.joint_names = NAMES
            for t, q in ((a.settle, start), (a.settle + T, target)):
                p = JointTrajectoryPoint(); p.positions = [float(x) for x in q]; p.velocities = [0.0] * 7; p.accelerations = [0.0] * 7
                p.time_from_start = Duration(sec=int(t), nanosec=int((t % 1) * 1e9)); g.trajectory.points.append(p)
            fut = self.ac.send_goal_async(g); rclpy.spin_until_future_complete(self, fut, timeout_sec=5.0)
            gh = fut.result()
            if gh is None or not gh.accepted: raise RuntimeError("trajectory goal rejected")
            rf = gh.get_result_async(); end = time.monotonic() + a.settle + T + 5.0; cancelled = False
            while not rf.done() and rclpy.ok():
                rclpy.spin_once(self, timeout_sec=0.05)
                if self.abort and not cancelled: gh.cancel_goal_async(); cancelled = True; self.log("abort requested: cancelling, the arm holds where it is")
                if time.monotonic() > end and not cancelled: gh.cancel_goal_async(); cancelled = True; self.log("timeout: cancelling")
                if time.monotonic() > end + 5.0: break
            res = rf.result() if rf.done() else None
            if cancelled: msg = "home aborted; the arm holds"
            elif res is None: raise RuntimeError("no trajectory result")
            elif res.result.error_code != 0: raise RuntimeError(f"trajectory failed: {res.result.error_code} {res.result.error_string}")
            else: ok, msg = True, "home complete; teleop stays disengaged (press Start to engage)"
        except RuntimeError as e:
            msg = str(e)
        finally:
            if stopped:                          # ALWAYS back to the impedance controller (it re-anchors at the pose the arm is in)
                self.abort = False; self.spin_for(0.5)
                try:
                    if not self.switch([imp], [pos]): ok, msg = False, f"CRITICAL: impedance controller did not restart ({msg}); run real-recover"
                    else: self.log(f"{imp} active again")
                except RuntimeError as e:
                    ok, msg = False, f"CRITICAL: swap back failed ({e}); {msg}; run real-recover"
        return ok, msg


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--pose", type=lambda s: [float(x) for x in s.split(",")], default=HOME)
    ap.add_argument("--speed-fraction", type=float, default=0.2); ap.add_argument("--min-duration", type=float, default=3.0)
    ap.add_argument("--settle", type=float, default=1.0); ap.add_argument("--margin", type=float, default=0.05)
    ap.add_argument("--controller", default="cartesian_impedance_target_controller"); ap.add_argument("--position-controller", default="fr3_position_controller")
    ap.add_argument("--params", default="/lab/fr3_position_jtc.yaml")
    ap.add_argument("--dry-run", action="store_true"); ap.add_argument("--no-check", action="store_true")
    a, ros = ap.parse_known_args()
    if len(a.pose) != 7: print("--pose needs 7 joint angles"); return 2
    if not 0.0 < a.speed_fraction <= 0.5: print("--speed-fraction must be within (0, 0.5]"); return 2
    rclpy.init(args=ros); n = Home(a)
    def stop(*_): n.abort = True
    signal.signal(signal.SIGINT, stop); signal.signal(signal.SIGTERM, stop)
    try: ok, msg = n.run()
    except Exception as e: ok, msg = False, f"error: {e}"
    (n.log if ok else n.get_logger().error)(("HOME OK: " if ok else "HOME FAILED: ") + msg)
    n.destroy_node(); rclpy.shutdown()
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
