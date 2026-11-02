#!/usr/bin/env python3
"""Web dashboard (read-only except the teleop speed scale) for the FR3 sim: a ROS 2 node + a tiny HTTP server (stdlib only).

Mirrors the robot in a browser (three.js, rotate/zoom like RViz) from TF and the URDF on /robot_description; the visual
meshes are served from the franka_description share directory. It also shows the teleop's commanded tool pose, the measured
TCP, the teleop engage state and the gamepad (joydev read-only, so it can run next to fr3_joy_teleop.py).
This is a kinematic mirror of Gazebo's physics state (headless Gazebo has no renderer here), not Gazebo's own view.
It sends nothing to the robot; its two writes are POST /speed (the teleop's speed_scale, 0.05..1 of its start-up speeds) and POST /padtest (makes the teleop ignore the pad during the pad check). Serve:  python3 fr3_web_dashboard.py --ros-args -p use_sim_time:=true [-p port:=8088]
Endpoints: /  page,  /config  link/mesh list,  /mesh/<pkg>/<path>  mesh files,  /stream  server-sent events (20 Hz).
Remote gamepad (opt-in, -p remote_pad:=true): /ws accepts ONE browser pad over a WebSocket and the node publishes it as
sensor_msgs/Joy on remote_topic (/joy_remote) for fr3_joy_teleop.py; interlocks and parameters in remote_pad.py / REMOTE_PAD.md.
"""
import json
import math
import os
import struct
import threading
import time
import xml.etree.ElementTree as ET
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import unquote, urlparse
from urllib.request import urlopen
from urllib.error import URLError

import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool, Float32MultiArray, String
from std_srvs.srv import Trigger, SetBool
from controller_manager_msgs.srv import SwitchController
from curobo_start import prepare as prepare_curobo
from tf2_ros import Buffer, TransformListener

import remote_pad

HERE = os.path.dirname(os.path.abspath(__file__))


def find_pad():                                       # as fr3_joy_teleop.py: the 8BitDo's node and layout, by sysfs name
    try:
        for j in sorted(os.listdir("/sys/class/input")):
            if j.startswith("js"):
                name = open(f"/sys/class/input/{j}/device/name").read()
                if "X-Box" in name: return f"/dev/input/{j}", "xinput"
                if "8BitDo" in name: return f"/dev/input/{j}", "dinput"
    except OSError:
        pass
    return None, None
LATCHED = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)


def pose_list(t):  # TransformStamped -> [x, y, z, qx, qy, qz, qw]
    a, b = t.transform.translation, t.transform.rotation
    return [a.x, a.y, a.z, b.x, b.y, b.z, b.w]


class Dash(Node):
    def __init__(self):
        super().__init__("fr3_web_dashboard")
        P = lambda n, d: self.declare_parameter(n, d).value
        self.port, self.base = int(P("port", 8088)), P("base_frame", "fr3_link0")
        self.bind = [a.strip() for a in str(P("bind", "0.0.0.0")).split(",") if a.strip()]   # addresses to listen on (host networking: never 0.0.0.0)
        self.tool, self.joy = P("tool_frame", "fr3_hand_tcp"), P("joy_device", "auto")        # "auto": find the 8BitDo by name each time it (re)appears
        self.source = P("source", "sim")              # label for the page: sim (Gazebo) | real (franka_ros2 on the robot) | mock
        self.layout = P("layout", "xinput")           # pad label set for the page: xinput | dinput
        self.rp = None
        if P("remote_pad", False):                    # opt-in: this is the only part of the dashboard that can move the robot
            csv = lambda v: [x.strip() for x in str(v).split(",") if x.strip()]
            self.rp = remote_pad.RemotePad(timeout=float(P("remote_timeout", 0.2)), max_rtt=float(P("remote_max_rtt", 0.4)),
                                           users=csv(P("remote_users", "")), trusted_peers=csv(P("remote_trusted_peers", "127.0.0.1,::1")),
                                           log=lambda m: self.get_logger().info(m))
            self.joy_pub = self.create_publisher(Joy, str(P("remote_topic", "/joy_remote")), 10)
            self.create_timer(0.02, self._remote_tick)
            self.get_logger().warn("remote gamepad ENABLED on /ws" + (f", users {self.rp.users}" if self.rp.users else ", NO user allow-list (remote_users)"))
        self.tf = Buffer(); TransformListener(self.tf, self)
        self.lock = threading.Lock()
        self.visuals, self.urdf = [], None            # [{link, mesh, xyz, rpy, scale}]
        self.cmd, self.engaged = None, False
        self.input_source = "unknown"
        self.create_subscription(String, "/teleop/input_source", self._on_source, LATCHED)
        self.zed_status = None; self.zed_status_at = 0.
        self.create_subscription(String, "/teleop/zed_status", self._on_zed_status, 1)
        self.pad = {"axes": {}, "buttons": {}, "ok": False, "layout": self.layout}
        self.msg, self.seq = "{}", 0
        self.create_subscription(String, "/robot_description", self._on_urdf, LATCHED)
        self.create_subscription(PoseStamped, "/teleop/command_tool", self._on_cmd, 1)   # internal command, tool frame (compares with the TCP)
        self.create_subscription(Bool, "/teleop/engaged", self._on_eng, 1)   # the teleop publishes it volatile, on change
        self.speed = None                                 # [scale, max horizontal m/s, max vertical m/s] from the teleop
        self.create_subscription(Float32MultiArray, "/teleop/speed_scale", self._on_speed, LATCHED)
        self.padmap = None                                # the teleop's actual bindings (for the dashboard's pad check)
        self.create_subscription(String, "/teleop/pad_map", self._on_padmap, LATCHED)
        self.zed_command_lock = threading.Lock()
        self.zed_cancel = threading.Event()
        self.zed_start_cli = self.create_client(Trigger, "/teleop/zed_start")
        self.zed_home_cli = self.create_client(Trigger, "/teleop/home")
        self.zed_engage_cli = self.create_client(Trigger, "/teleop/zed_engage")
        self.zed_stop_cli = self.create_client(Trigger, "/teleop/stop")
        self.speed_cli = self.create_client(SetParameters, "/fr3_joy_teleop/set_parameters")
        self.curobo_enable_cli=self.create_client(SetBool,'/fr3_curobo/enable')
        self.curobo_switch_cli=self.create_client(SwitchController,'/controller_manager/switch_controller')
        self.create_timer(0.05, self._tick)
        if self.joy: threading.Thread(target=self._joydev, daemon=True).start()
        self.get_logger().info(f"dashboard on :{self.port}, waiting for /robot_description")

    def _on_zed_status(self, m):
        try:
            with self.lock: self.zed_status = json.loads(m.data); self.zed_status_at = time.monotonic()
        except ValueError: pass

    def _on_source(self, m):
        with self.lock: self.input_source = m.data

    def _on_urdf(self, m):
        if self.visuals: return
        vis = []
        for link in ET.fromstring(m.data).iter("link"):
            for v in link.findall("visual"):
                mesh = v.find("geometry/mesh")
                if mesh is None or not mesh.get("filename", "").startswith("package://"): continue
                o = v.find("origin")
                f = lambda s: [float(c) for c in (s or "0 0 0").split()]
                vis.append(dict(link=link.get("name"), mesh=mesh.get("filename")[len("package://"):],
                                xyz=f(o.get("xyz") if o is not None else None), rpy=f(o.get("rpy") if o is not None else None),
                                scale=f(mesh.get("scale") or "1 1 1")))
        with self.lock: self.visuals = vis
        self.get_logger().info(f"URDF: {len(vis)} visual meshes")

    def _on_cmd(self, m):
        p, q = m.pose.position, m.pose.orientation
        with self.lock: self.cmd = [p.x, p.y, p.z, q.x, q.y, q.z, q.w]

    def _on_speed(self, m):
        with self.lock: self.speed = [float(x) for x in m.data]

    def set_teleop_param(self, name, value):
        """Set one of the two teleop parameters the dashboard may touch: speed_scale (float, the teleop rejects > 1) or pad_test (bool,
        which only makes the teleop ignore the pad). Returns (ok, message)."""
        if name not in ("speed_scale", "rot_scale", "pad_test", "grasp_force", "input_source", "zed_review_exercise"): return False, "parameter not allowed"
        if not self.speed_cli.wait_for_service(timeout_sec=1.0): return False, "teleop is not running"
        if name == "input_source" and value not in ("joystick", "zed"): return False, "invalid source"
        pv = (ParameterValue(type=ParameterType.PARAMETER_STRING, string_value=value) if name in ("input_source","zed_review_exercise") else ParameterValue(type=ParameterType.PARAMETER_BOOL, bool_value=bool(value)) if name == "pad_test"
              else ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=float(value)))
        req = SetParameters.Request(); req.parameters = [Parameter(name=name, value=pv)]
        done = threading.Event(); res = []
        fut = self.speed_cli.call_async(req); fut.add_done_callback(lambda f: (res.append(f.result()), done.set()))
        if not done.wait(2.0) or not res or not res[0]: return False, "no answer from the teleop"
        r = res[0].results[0]; return bool(r.successful), r.reason or "ok"

    def control_zed(self, action):
        if action=='disengage':
            self.zed_cancel.set()
            return self._control_zed(action)
        with self.zed_command_lock:
            if action in ('start','engage','recalibrate'):self.zed_cancel.clear()
            return self._control_zed(action)

    def _curobo_rpc(self,client,request,timeout=3.):
        if not client.wait_for_service(timeout_sec=1.):return None
        done=threading.Event();box=[]
        def finished(future):
            try:box.append(future.result())
            except Exception:box.append(None)
            finally:done.set()
        client.call_async(request).add_done_callback(finished)
        if not done.wait(timeout) or not box:return None
        return box[0]

    def _curobo_enable(self,enabled):
        request=SetBool.Request();request.data=bool(enabled)
        result=self._curobo_rpc(self.curobo_enable_cli,request)
        return (bool(result.success),result.message) if result else (False,'No answer from cuRobo output bridge')

    def _prepare_curobo(self):
        with self.lock:
            if self.engaged:return False,'Stop before preparing cuRobo'
        def status():
            with urlopen('http://127.0.0.1:8784/status',timeout=2.) as response:return json.load(response)
        def switch(activate,deactivate):
            request=SwitchController.Request();request.activate_controllers=activate;request.deactivate_controllers=deactivate;request.strictness=2;request.activate_asap=True;request.timeout.sec=2
            result=self._curobo_rpc(self.curobo_switch_cli,request)
            return bool(result and result.ok)
        try:return prepare_curobo(status,switch,lambda:self._curobo_enable(True),lambda:self._curobo_enable(False),cancelled=self.zed_cancel.is_set)
        except Exception as error:return False,'cuRobo preparation failed: '+str(error)

    def _control_zed(self, action):
        if action=="recalibrate":
            ok,message=self._control_zed("disengage")
            if not ok:return ok,message
            import socket
            with socket.socket(socket.AF_INET,socket.SOCK_DGRAM) as control:
                control.sendto(b'{"action":"reset_person","version":1}',('127.0.0.1',47815))
            return self._control_zed("start")
        if action not in ("start", "engage", "disengage", "home"): return False, "invalid action"
        if action=='start' and (not self.zed_status or time.monotonic()-self.zed_status_at>2.):return False,'Teleop status absent/stale'
        if action == "engage":
            with self.lock:
                if self.input_source != "zed": return False, "Select ZED3 first"
                if self.engaged: return False, "Already active"
                if not self.zed_status or self.zed_status.get("reason"):
                    return False, (self.zed_status or {}).get("reason") or "Tracking unavailable"
        if action=='start' and (self.zed_status or {}).get('control_backend')=='curobo':
            if self.zed_status.get('start_reason'):return False,self.zed_status['start_reason']
            ok,message=self._prepare_curobo()
            if not ok:return False,message
        if action=='start' and self.zed_cancel.is_set():return False,'Start cancelled'
        client = getattr(self, {"start": "zed_start_cli", "engage": "zed_engage_cli", "disengage": "zed_stop_cli", "home": "zed_home_cli"}[action])
        if not client.wait_for_service(timeout_sec=1): return False, "Teleop unavailable"
        done = threading.Event(); result = []
        future = client.call_async(Trigger.Request())
        def finished(f):
            try: result.append(f.result())
            finally: done.set()
        future.add_done_callback(finished)
        if not done.wait(2) or not result or result[0] is None:
            return False, "No response; check the active status before retrying"
        if action=='start' and self.zed_cancel.is_set():return self._control_zed('disengage')
        if action in ('disengage','home') and (self.zed_status or {}).get('control_backend')=='curobo':self._curobo_enable(False)
        return bool(result[0].success), result[0].message

    def set_speed(self, scale, name="speed_scale"): return self.set_teleop_param(name, scale)

    def _on_padmap(self, m):
        try:
            with self.lock: self.padmap = json.loads(m.data)
        except ValueError: pass

    def _on_eng(self, m):
        with self.lock: self.engaged = bool(m.data)

    def _joydev(self):                                # read-only joydev, the same struct as the teleop node
        while True:
            dev, layout = (find_pad() if self.joy == "auto" else (self.joy, self.layout))
            if dev is None:
                with self.lock: self.pad.update(ok=False, axes={}, buttons={})
                time.sleep(1.0); continue
            try:
                with open(dev, "rb") as f:
                    with self.lock: self.pad.update(ok=True, layout=layout, axes={}, buttons={})
                    while True:
                        _, value, typ, num = struct.unpack("IhBB", f.read(8))
                        with self.lock:
                            (self.pad["axes"] if typ & 2 else self.pad["buttons"])[str(num)] = value / 32767.0 if typ & 2 else int(value)
            except (OSError, struct.error):
                with self.lock: self.pad.update(ok=False, axes={}, buttons={})
                time.sleep(1.0)

    def _remote_tick(self):                           # 50 Hz: publish the accepted browser pad; one neutral Joy when it stops
        act = self.rp.tick()
        if act is None: return
        m = Joy(); m.header.stamp = self.get_clock().now().to_msg()
        m.axes, m.buttons = (act[1], act[2]) if act[0] == "pub" else remote_pad.NEUTRAL_JOY
        self.joy_pub.publish(m)

    def _tick(self):
        with self.lock: links = sorted({v["link"] for v in self.visuals})
        poses = {}
        for l in links:
            try: poses[l] = pose_list(self.tf.lookup_transform(self.base, l, Time()))
            except Exception: pass
        try: tcp = pose_list(self.tf.lookup_transform(self.base, self.tool, Time()))
        except Exception: tcp = None
        with self.lock:
            self.seq += 1
            self.msg = json.dumps(dict(seq=self.seq, t=self.get_clock().now().nanoseconds * 1e-9, links=poses, tcp=tcp,
                                       cmd=self.cmd, engaged=self.engaged, input_source=self.input_source, zed=self.zed_status if time.monotonic()-self.zed_status_at<2. else None, pad=self.pad, remote=self.rp.status() if self.rp else None, speed=self.speed, padmap=self.padmap))


def make_handler(node):
    class H(BaseHTTPRequestHandler):
        def log_message(self, *a): pass

        def _send(self, code, body, ctype):
            self.send_response(code); self.send_header("Content-Type", ctype); self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-cache"); self.end_headers(); self.wfile.write(body)

        def do_POST(self):
            # The only write path: the speed scale. Guards: JSON + a custom header (a cross-site form cannot send either without a CORS
            # preflight, which this server never answers), same-origin check, and a hard clamp; the teleop itself rejects > 1.
            path = urlparse(self.path).path
            host = (self.headers.get("Host") or "").split(":")[0]; origin = self.headers.get("Origin")
            same = origin is None or (urlparse(origin).hostname or "") == host
            if path not in ("/speed", "/padtest", "/grip", "/input-source", "/zed-control", "/zed-profile") or not same or self.headers.get("X-Dashboard") != "1" or "json" not in (self.headers.get("Content-Type") or ""):
                return self._send(403, b"forbidden", "text/plain")
            if path == "/zed-control":
                # Start is an explicit motion-enable action; preserve local/proxy trust.
                peer = self.client_address[0]
                if peer not in ("127.0.0.1", "::1", self.server.server_address[0]):
                    return self._send(403, b"Use the local dashboard or authenticated HTTPS dashboard", "text/plain")
                user = self.headers.get("Tailscale-User-Login", "")
                if node.rp is not None and node.rp.users:
                    local_host = host in ("127.0.0.1", "localhost", "100.88.134.127")
                    if (user and user not in node.rp.users) or (not user and not local_host):
                        return self._send(403, b"User not allowed", "text/plain")
                try:
                    action = json.loads(self.rfile.read(min(int(self.headers.get("Content-Length") or 0), 200)))["action"]
                    if action not in ("start", "engage", "disengage", "home", "recalibrate"): raise ValueError("invalid action")
                except Exception: return self._send(400, b"bad request", "text/plain")
                try: ok, msg = node.control_zed(action)
                except Exception: ok, msg = False, "Teleop request failed"
                return self._send(200 if ok else 409, json.dumps(dict(ok=ok, message=msg)).encode(), "application/json")
            if path == "/zed-profile":
                try:
                    profile=json.loads(self.rfile.read(min(int(self.headers.get("Content-Length") or 0),200)))["profile"]
                    if profile not in ('full_6d', 'translation', 'horizontal', 'height', 'reach', 'roll', 'pitch', 'yaw', 'gripper', 'cross', 'neutral'):raise ValueError('Unknown exercise')
                except Exception:return self._send(400,b"bad request","text/plain")
                ok,msg=node.set_teleop_param("zed_review_exercise",profile)
                return self._send(200 if ok else 409,json.dumps(dict(ok=ok,message=msg)).encode(),"application/json")
            if path == "/input-source":
                try:
                    source = json.loads(self.rfile.read(min(int(self.headers.get("Content-Length") or 0), 200)))["source"]
                    if source not in ("joystick", "zed"): raise ValueError("invalid source")
                except Exception: return self._send(400, b"bad request", "text/plain")
                try: ok, msg = node.set_teleop_param("input_source", source)
                except Exception: ok, msg = False, "teleop unavailable"
                return self._send(200 if ok else 502, json.dumps(dict(ok=ok, message=msg)).encode(), "application/json")
            if path == "/grip":                       # {"force": N}: the teleop's grasp_force for the next grasp, clamped 1..70 N
                try:
                    f = float(json.loads(self.rfile.read(min(int(self.headers.get("Content-Length") or 0), 200)))["force"]); assert math.isfinite(f)
                except Exception: return self._send(400, b"bad request", "text/plain")
                f = min(70.0, max(1.0, f))
                try: ok, msg = node.set_teleop_param("grasp_force", f)
                except Exception as e: ok, msg = False, f"ROS side of the dashboard is down ({type(e).__name__}); restart it"
                node.get_logger().info(f"dashboard grasp_force request {f:.0f} N from {self.client_address[0]}: {msg}")
                return self._send(200 if ok else 502, json.dumps(dict(ok=ok, message=msg, force=f)).encode(), "application/json")
            if path == "/padtest":                    # pad check on/off: the teleop then ignores the pad (self-expires after 5 s)
                try: on = json.loads(self.rfile.read(min(int(self.headers.get("Content-Length") or 0), 200))).get("on")
                except Exception: return self._send(400, b"bad request", "text/plain")
                if not isinstance(on, bool): return self._send(400, b"bad request", "text/plain")
                try: ok, msg = node.set_teleop_param("pad_test", on)
                except Exception as e: ok, msg = False, f"ROS side of the dashboard is down ({type(e).__name__}); restart it"
                return self._send(200 if ok else 502, json.dumps(dict(ok=ok, message=msg, on=on)).encode(), "application/json")
            try:                                      # {"scale": x} (translation) and/or {"rot_scale": y} (rotation), each a fraction of the teleop's ceiling
                body = json.loads(self.rfile.read(min(int(self.headers.get("Content-Length") or 0), 200)))
                req = {k: float(body[k]) for k in ("scale", "rot_scale") if k in body}
                assert req and all(math.isfinite(v) for v in req.values())
            except Exception: return self._send(400, b"bad request", "text/plain")
            out, ok_all, msgs = {}, True, []
            for k, v in req.items():
                v = min(1.0, max(0.05, v)); out[k] = v
                try: ok, msg = node.set_speed(v, "speed_scale" if k == "scale" else "rot_scale")
                except Exception as e: ok, msg = False, f"ROS side of the dashboard is down ({type(e).__name__}); restart it"
                ok_all &= ok; msgs.append(msg)
                node.get_logger().info(f"dashboard {k} request {v:.2f} from {self.client_address[0]}: {msg}")
            return self._send(200 if ok_all else 502, json.dumps(dict(ok=ok_all, message="; ".join(msgs), **out)).encode(), "application/json")

        def do_GET(self):
            path = unquote(urlparse(self.path).path)
            if path in ("/", "/index.html"):
                return self._send(200, open(os.path.join(HERE, "fr3_dashboard.html"), "rb").read(), "text/html; charset=utf-8")
            if path == "/zed3/wrist":
                # Fixed local upstream: same-origin video also works through the remote dashboard tunnel.
                # No user-supplied URL or topic. No redirect forwarding.
                try:
                    upstream = urlopen("http://127.0.0.1:8787/stream?topic=/fr3_operator/wrist_overlay&type=mjpeg&quality=75", timeout=3)
                except (OSError, URLError): return self._send(503, b"ZED3 wrist stream unavailable", "text/plain")
                try:
                    self.send_response(200)
                    self.send_header("Content-Type", upstream.headers.get("Content-Type", "multipart/x-mixed-replace;boundary=boundarydonotcross"))
                    self.send_header("Cache-Control", "no-store")
                    self.end_headers()
                    while True:
                        chunk = upstream.read1(65536)
                        if not chunk: break
                        self.wfile.write(chunk); self.wfile.flush()
                except (BrokenPipeError, ConnectionResetError, OSError): pass
                finally: upstream.close()
                return
            if path == "/config":
                with node.lock: body = json.dumps(dict(visuals=node.visuals, base=node.base, layout=node.layout, remote=node.rp is not None, source=node.source)).encode()
                return self._send(200, body, "application/json")
            if path.startswith("/mesh/"):
                pkg, _, rel = path[len("/mesh/"):].partition("/")
                try: share = os.path.normpath(get_package_share_directory(pkg))
                except Exception: return self._send(404, b"no such package", "text/plain")
                full = os.path.normpath(os.path.join(share, rel))      # normpath, not realpath: a symlink install points outside share/
                if not full.startswith(share + os.sep) or not full.lower().endswith((".dae", ".stl", ".png", ".jpg")) or not os.path.isfile(full):
                    return self._send(404, b"not found", "text/plain")
                return self._send(200, open(full, "rb").read(), "model/vnd.collada+xml" if full.endswith(".dae") else "application/octet-stream")
            if path == "/ws":
                if node.rp is None: return self._send(404, b"remote pad disabled", "text/plain")
                return remote_pad.serve_socket(self, node.rp)
            if path == "/stream":
                self.send_response(200); self.send_header("Content-Type", "text/event-stream"); self.send_header("Cache-Control", "no-cache")
                self.end_headers(); last = -1
                try:
                    while True:
                        with node.lock: seq, msg = node.seq, node.msg
                        if seq != last: self.wfile.write(f"data: {msg}\n\n".encode()); self.wfile.flush(); last = seq
                        time.sleep(0.02)
                except (BrokenPipeError, ConnectionResetError, OSError): return
            self._send(404, b"not found", "text/plain")
    return H


def main():
    rclpy.init(); n = Dash()
    threading.Thread(target=rclpy.spin, args=(n,), daemon=True).start()
    def _watchdog():                                  # rclpy turns SIGTERM/SIGINT into a context shutdown and the HTTP threads would keep
        while rclpy.ok(): time.sleep(0.5)             # answering with a dead ROS side ("failed to fetch" on POST): exit instead
        print("ROS context is gone: exiting so the dashboard is not left half-dead", flush=True)
        os._exit(1)
    threading.Thread(target=_watchdog, daemon=True).start()
    srvs = [ThreadingHTTPServer((a, n.port), make_handler(n)) for a in n.bind]
    for x in srvs: x.daemon_threads = True
    for x in srvs[1:]: threading.Thread(target=x.serve_forever, daemon=True).start()
    n.get_logger().info("listening on " + ", ".join(f"{a}:{n.port}" for a in n.bind))
    try: srvs[0].serve_forever()
    except KeyboardInterrupt: pass
    finally:
        for x in srvs: x.server_close()
        n.destroy_node()
        if rclpy.ok(): rclpy.shutdown()


if __name__ == "__main__":
    main()
