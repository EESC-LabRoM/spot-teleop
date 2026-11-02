#!/usr/bin/env python3
"""Robot-side helper for the ZED movement protocol (runs inside fr3-ros2, domain 77). Read-only except `home`.
  cycle [--timeout s]  waits for the operator's Start (/teleop/engaged true) and then Stop (false); prints JSON events
  home  [--timeout s]  calls /teleop/home and waits for the teleop's home result (zed_status.home_result)
  serve                for zed_protocol_server.py: status JSON lines at 5 Hz on stdout; reads commands from stdin, one per line:
                       zed_start | stop | home  (-> /teleop/zed_start, /teleop/stop, /teleop/home)
"""
import argparse, json, sys, threading, time
import rclpy
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger

ap = argparse.ArgumentParser(); ap.add_argument("mode", choices=["cycle", "home", "serve"]); ap.add_argument("--timeout", type=float, default=600.0)
a = ap.parse_args()
rclpy.init(domain_id=77); n = rclpy.create_node("zed_protocol_robot_side")
st = dict(engaged=None, status=None)
n.create_subscription(Bool, "/teleop/engaged", lambda m: st.update(engaged=m.data), 10)
n.create_subscription(String, "/teleop/zed_status", lambda m: st.update(status=json.loads(m.data)), 10)
def emit(**k): print(json.dumps(dict(t=time.time(), **k)), flush=True)
def spin_until(cond, timeout):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        rclpy.spin_once(n, timeout_sec=0.05)
        if cond(): return True
    return False

if not spin_until(lambda: st["engaged"] is not None and st["status"] is not None, 10): emit(event="error", msg="teleop not publishing"); sys.exit(2)
if a.mode == "serve":
    tcp = {}
    n.create_subscription(PoseStamped, "/franka_robot_state_broadcaster/current_pose",
                          lambda m: tcp.update(p=[round(m.pose.position.x, 4), round(m.pose.position.y, 4), round(m.pose.position.z, 4)]), qos_profile_sensor_data)
    srv = {k: n.create_client(Trigger, "/teleop/" + k) for k in ("zed_start", "stop", "home")}
    cmds = []
    threading.Thread(target=lambda: [cmds.append(l.strip()) for l in sys.stdin], daemon=True).start()
    pending = []; last = 0.0
    while rclpy.ok():
        rclpy.spin_once(n, timeout_sec=0.02)
        while cmds:
            c = cmds.pop(0)
            if c in srv: pending.append((c, srv[c].call_async(Trigger.Request()))); emit(event="sent", cmd=c)
        for c, f in list(pending):
            if f.done():
                pending.remove((c, f)); r = f.result(); emit(event="reply", cmd=c, ok=bool(r and r.success), msg=getattr(r, "message", None))
        if time.monotonic() - last > 0.2:
            last = time.monotonic(); z = st["status"] or {}
            emit(event="status", engaged=st["engaged"], phase=z.get("phase"), note=z.get("note"), tracking=z.get("tracking"),
                 orientation_valid=z.get("orientation_valid"), home_running=z.get("home_running"), home_result=z.get("home_result"),
                 gesture=z.get("gesture"), tcp=tcp.get("p"), input=(z.get("diagnostics") or {}).get("reason"))
elif a.mode == "cycle":
    if st["engaged"]: emit(event="error", msg="already engaged: click Stop first"); sys.exit(2)
    emit(event="waiting_start")
    if not spin_until(lambda: st["engaged"], a.timeout): emit(event="timeout"); sys.exit(3)
    emit(event="engaged", note=st["status"].get("note"))
    last_note = None
    def done():
        global last_note
        note = st["status"].get("note") if st["status"] else None
        if note != last_note: last_note = note; emit(event="note", note=note, phase=st["status"].get("phase"))
        return not st["engaged"]
    spin_until(done, a.timeout); emit(event="disengaged", note=st["status"].get("note"))
else:
    if st["engaged"]: emit(event="error", msg="engaged: home refused"); sys.exit(2)
    c = n.create_client(Trigger, "/teleop/home")
    if not c.wait_for_service(timeout_sec=5): emit(event="error", msg="no /teleop/home"); sys.exit(2)
    f = c.call_async(Trigger.Request()); rclpy.spin_until_future_complete(n, f, timeout_sec=10)
    r = f.result()
    if r is None or not r.success: emit(event="home_refused", msg=getattr(r, "message", "no answer")); sys.exit(4)
    emit(event="home_started", msg=r.message)
    spin_until(lambda: st["status"].get("home_running"), 5)
    ok = spin_until(lambda: not st["status"].get("home_running"), a.timeout)
    spin_until(lambda: False, 0.5)
    res = st["status"].get("home_result")
    emit(event="home_done" if ok else "home_timeout", result=res); sys.exit(0 if ok and res and "falhou" not in res.lower() else 5)
