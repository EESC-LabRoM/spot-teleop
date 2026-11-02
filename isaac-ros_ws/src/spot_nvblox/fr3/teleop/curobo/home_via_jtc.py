#!/usr/bin/env python3
"""Joint home while the cuRobo joystick mode owns the arm (fr3_arm_controller = effort JTC active, Cartesian impedance inactive).
fr3_home_jtc.py cannot be used there: it swaps impedance -> JTC and refuses when impedance is already inactive.
Checks before moving: fr3_arm_controller active and the impedance controller inactive, nobody else publishing to the JTC
(disarm the cuRobo bridge first), robot_mode 2, the arm standing still. Then one quintic two-point trajectory to HOME at
--speed-fraction of the FR3 joint velocity limits (at least --min-duration s) through the controller's action. Ctrl+C cancels (holds).
Run in fr3-ros2, domain 77:  python3 home_via_jtc.py [--dry-run]
"""
import argparse, signal, sys, time
import numpy as np, rclpy
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from controller_manager_msgs.srv import ListControllers
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint

NAMES = [f"fr3_joint{i}" for i in range(1, 8)]
HOME = np.array([0.0, -0.7853981634, 0.0, -2.3561944902, 0.0, 1.5707963268, 0.7853981634])
VEL = np.array([2.62, 2.62, 2.62, 2.62, 5.26, 4.18, 5.26])
ap = argparse.ArgumentParser(); ap.add_argument("--speed-fraction", type=float, default=0.2); ap.add_argument("--min-duration", type=float, default=5.0)
ap.add_argument("--dry-run", action="store_true"); a = ap.parse_args()
rclpy.init(domain_id=77); n = rclpy.create_node("fr3_home_via_jtc"); st = dict(q=None, t=0.0)
n.create_subscription(JointState, "/joint_states", lambda m: st.update(q=np.array([dict(zip(m.name, m.position))[j] for j in NAMES]), t=time.monotonic()), 20)
def spin(s):
    e = time.monotonic() + s
    while time.monotonic() < e: rclpy.spin_once(n, timeout_sec=0.02)
def fail(msg): print("HOME REFUSED:", msg); sys.exit(1)
spin(1.0)
cl = n.create_client(ListControllers, "/controller_manager/list_controllers")
if not cl.wait_for_service(timeout_sec=10): fail("no controller_manager")
res = None
for _ in range(3):                                  # the controller_manager answers slowly when the lab PC is loaded
    f = cl.call_async(ListControllers.Request()); rclpy.spin_until_future_complete(n, f, timeout_sec=30); res = f.result()
    if res is not None: break
if res is None: fail("controller_manager did not answer list_controllers")
states = {c.name: c.state for c in res.controller}
if states.get("fr3_arm_controller") != "active": fail(f"fr3_arm_controller is {states.get('fr3_arm_controller')}")
if states.get("cartesian_impedance_target_controller") == "active": fail("Cartesian impedance controller is active")
pubs = [p.node_name for p in n.get_publishers_info_by_topic("/fr3_arm_controller/joint_trajectory")]
others = [x for x in pubs if x != "fr3_curobo_hardware"]
if others: fail(f"publisher(s) {others} on /fr3_arm_controller/joint_trajectory")
if pubs:                                            # the cuRobo bridge keeps its publisher while disarmed: require disarmed
    import json, urllib.request
    try: bridge = json.load(urllib.request.urlopen("http://127.0.0.1:8784/status", timeout=2))
    except Exception as e: fail(f"cuRobo bridge publisher present and its status is unreadable ({e})")
    if bridge.get("armed"): fail("the cuRobo bridge is ARMED: disarm it first (/fr3_curobo/enable false)")
if st["q"] is None or time.monotonic() - st["t"] > 0.5: fail("no fresh /joint_states")
q0 = st["q"].copy(); spin(0.5)
if np.abs(st["q"] - q0).max() > 0.003: fail("the arm is moving")
start = st["q"].copy(); T = max(a.min_duration, float(np.max(np.abs(HOME - start) / (a.speed_fraction * VEL))))
print(f"start {np.round(start, 3).tolist()}\nhome  {np.round(HOME, 3).tolist()}  max change {np.degrees(np.abs(HOME - start).max()):.0f} deg in {T:.1f} s")
if a.dry_run: print("dry run: nothing sent"); sys.exit(0)
ac = ActionClient(n, FollowJointTrajectory, "/fr3_arm_controller/follow_joint_trajectory")
if not ac.wait_for_server(timeout_sec=10): fail("no follow_joint_trajectory action")
g = FollowJointTrajectory.Goal(); g.trajectory.joint_names = NAMES
for t, q in ((0.5, start), (0.5 + T, HOME)):
    p = JointTrajectoryPoint(); p.positions = [float(x) for x in q]; p.velocities = [0.0] * 7; p.accelerations = [0.0] * 7
    p.time_from_start = Duration(sec=int(t), nanosec=int((t % 1) * 1e9)); g.trajectory.points.append(p)
fu = ac.send_goal_async(g); rclpy.spin_until_future_complete(n, fu, timeout_sec=5); gh = fu.result()
if gh is None or not gh.accepted: fail("trajectory rejected")
abort = dict(on=False); signal.signal(signal.SIGINT, lambda *_: abort.update(on=True))
rf = gh.get_result_async(); end = time.monotonic() + T + 6
while not rf.done() and time.monotonic() < end:
    rclpy.spin_once(n, timeout_sec=0.05)
    if abort["on"]: gh.cancel_goal_async(); spin(0.5); print("cancelled: holding"); sys.exit(2)
r = rf.result(); spin(0.3)
print("HOME OK" if r and r.result.error_code == 0 else f"HOME FAILED: {r.result.error_code if r else 'timeout'} {r.result.error_string if r else ''}",
      "| error to home (deg):", np.round(np.degrees(np.abs(st['q'] - HOME)).max(), 2))
