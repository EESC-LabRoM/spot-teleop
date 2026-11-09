import time, json, math, rclpy, numpy as np
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from std_msgs.msg import String
from std_srvs.srv import SetBool
from geometry_msgs.msg import PoseStamped
from trajectory_msgs.msg import JointTrajectory
from sensor_msgs.msg import JointState
HAND = json.loads('[0.31353947872199445, -0.010929719145073721, 0.5884965305936811, 0.9985947501388144, 0.017488659162843336, -0.02363438112621941, -0.0440918113107798]')
rclpy.init(); n = Node("claude_mpc_test"); be = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)
st = []; pv = []; js = []
n.create_subscription(String, "/fr3_joystick_mpc/state", lambda m: st.append((time.time(), m.data)), QoSProfile(depth=5, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL))
n.create_subscription(JointTrajectory, "/fr3_joystick_mpc/joint_preview", lambda m: pv.append((time.time(), m)), be)
n.create_subscription(JointState, "/joint_states", lambda m: js.append((time.time(), m)), be)
pub = n.create_publisher(PoseStamped, "/fr3_joystick_mpc/wrist_goal", be); cli = n.create_client(SetBool, "/fr3_joystick_mpc/set_active")
def spin(sec, goal=None):
    t0 = time.time(); last = 0
    while time.time() - t0 < sec:
        rclpy.spin_once(n, timeout_sec=0.01)
        if goal is not None and time.time() - last > 0.1:
            m = PoseStamped(); m.header.frame_id = "fr3_link0"; m.header.stamp = n.get_clock().now().to_msg()
            m.pose.position.x, m.pose.position.y, m.pose.position.z = goal[:3]; m.pose.orientation.x, m.pose.orientation.y, m.pose.orientation.z, m.pose.orientation.w = HAND[3:7]; pub.publish(m); last = time.time()
def call(v):
    f = cli.call_async(SetBool.Request(data=v)); t0 = time.time()
    while not f.done() and time.time() - t0 < 10: rclpy.spin_once(n, timeout_sec=0.05)
    return f.result().message if f.done() else "no response"
def last_state(): return st[-1][1] if st else "?"
def seg_stats(t0, t1, label):
    P = [(t, m) for t, m in pv if t0 <= t <= t1]; Jm = [m for t, m in js if t0 <= t <= t1]
    if not P: print(f"  [{label}] no preview messages"); return
    rate = len(P) / (t1 - t0); q0 = np.array(P[0][1].points[0].positions) if P[0][1].points else None
    names = P[-1][1].joint_names; meas = Jm[-1]; idx = {nm: i for i, nm in enumerate(meas.name)}; qm = np.array([meas.position[idx[nm]] for nm in names])
    qp = np.array(P[-1][1].points[0].positions); d = np.abs(qp - qm)
    print(f"  [{label}] previews {len(P)} in {t1-t0:.1f} s = {rate:.1f} Hz; last preview vs measured joints: max |dq| {d.max()*1000:.1f} mrad (mean {d.mean()*1000:.1f}); preview joint range moved from first to last: {np.abs(np.array(P[-1][1].points[0].positions)-np.array(P[0][1].points[0].positions)).max()*1000:.1f} mrad")
try:
    print("state at start:", last_state() if (spin(2) or True) else "")
    print("set_active(True):", call(True)); t_act = time.time()
    t0 = time.time()
    while time.time() - t0 < 150 and last_state() != "active":
        spin(0.5, goal=HAND[:3]);
        if last_state() == "fault": break
    print(f"state after {time.time()-t_act:.0f} s: {last_state()}  (history: {[s for _, s in st]})")
    if last_state() == "active":
        tA = time.time(); spin(8, goal=HAND[:3]); seg_stats(tA, time.time(), "HOLD at the current hand pose")
        up = [HAND[0], HAND[1], HAND[2] + 0.05]; tB = time.time(); spin(10, goal=up); seg_stats(tB, time.time(), "GOAL +5 cm in z")
        tC = time.time(); spin(10, goal=HAND[:3]); seg_stats(tC, time.time(), "GOAL back to the hold pose")
        print("state at the end of the test:", last_state())
finally:
    print("set_active(False):", call(False)); spin(2); print("final state:", last_state())
