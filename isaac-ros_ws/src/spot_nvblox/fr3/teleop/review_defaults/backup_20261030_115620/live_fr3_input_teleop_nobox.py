#!/usr/bin/env python3
"""Gamepad teleop for the FR3 on franka_ros2 (ROS 2 Jazzy): a port of Paiva's ROS 1 fr3_teleop (joystick_teleop.py +
fr3_teleop.py, oopsie_dataset). Publishes ~/equilibrium_pose (PoseStamped) for a Cartesian impedance controller that
FOLLOWS it, as their fr3_control does. NOT franka_ros2 v3.6.0's cartesian_impedance_example_controller: that demo
overwrites the position target every cycle with a built-in 10 cm / 10 s arc and only uses the topic's orientation
(updateMotionTarget; measured 10-14 in Gazebo: a 5.7 x 8.1 cm swing with nothing published).

Input: the pad read straight from its joydev device (as theirs, so their 8BitDo Ultimate 2C button/axis numbers carry
over), or sensor_msgs/Joy on /joy when joy_device is empty (tests). Sticks integrate a target pose in the base frame
(translation) and the tool frame (roll/pitch/yaw); a follower moves the commanded pose toward it with their limits
(first-order time constant, max step per update, max slew per update) and publishes it at `rate`.
Safety kept from theirs: nothing moves until Start (engage), Back disengages (the controller holds the last pose),
stale robot state -> hold. Added because franka_ros2's controller has no error clip (theirs caps force = stiffness x
clip): the PUBLISHED pose stays within max_error / max_rot_error_deg of the measured one (the command itself is not
dragged; it stops advancing away from the arm), and a workspace box.
Frames: the pad moves tool_frame (the fingertip TCP); the controller's equilibrium pose refers to control_frame, its
O_T_EE: fr3_hand_tcp on the real FR3 with the hand set as end effector in Desk, fr3_link8 (flange) in franka_gazebo,
which models no tool. The fixed control-in-tool transform is read from TF at engage.
Home: hold X (2 s, local pad only) or call /teleop/home -> fr3_home_joints_effort.py (effort-JTC joint home, no cuRobo; FK fr3_home_safe.py is the deprecated fallback, Back aborts).
Not ported: scale toggle, recording buttons, base-yaw orbit mode, joint-limit warnings.
Remote pad: with remote_joy_topic set (the dashboard's remote_pad publishes /joy_remote, raw XInput numbering whatever the local
pad is) a second input slot exists. While it is fresh (remote_timeout, default 0.25 s) it drives the arm and the local pad is
ignored EXCEPT its Back button, which always disengages; when it goes stale while it was driving, the teleop DISENGAGES (a
remote link glitch needs a deliberate Start again) and control falls back to the local pad.
   python3 fr3_joy_teleop.py --ros-args --params-file fr3_joy_teleop.yaml [-p joy_device:=/dev/input/js1]
"""
from pathlib import Path
import math
import os
import signal
import struct
import socket
import subprocess
import threading
import time

# Port of /home/murilo/franka_ws/lab/fr3_joy_teleop.py, 2026-10-20.
# One writer for joystick and ZED; source changes always disengage.
import numpy as np
from controller_manager_msgs.srv import ListControllers
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rcl_interfaces.msg import SetParametersResult
from rclpy.time import Time
from sensor_msgs.msg import Joy
import json
from std_msgs.msg import Bool, Float32MultiArray, String
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformListener

try:
    from franka_msgs.action import Grasp, Move
except ImportError:                                   # gripper optional (tests without franka_msgs)
    Grasp = Move = None


def qmul(a, b):                                       # quaternions as [x, y, z, w]
    ax, ay, az, aw = a; bx, by, bz, bw = b
    return np.array([aw * bx + ax * bw + ay * bz - az * by, aw * by - ax * bz + ay * bw + az * bx,
                     aw * bz + ax * by - ay * bx + az * bw, aw * bw - ax * bx - ay * by - az * bz])


def qexp(w):                                          # rotation vector -> quaternion
    a = float(np.linalg.norm(w))
    if a < 1e-12: return np.array([0.0, 0.0, 0.0, 1.0])
    return np.r_[np.sin(a / 2) * np.asarray(w) / a, np.cos(a / 2)]


def qrot(q, v):                                       # rotate vector v by quaternion q
    return qmul(qmul(q, np.r_[v, 0.0]), q * np.array([-1, -1, -1, 1]))[:3]


def qangle(a, b):
    return 2.0 * math.acos(min(1.0, abs(float(np.dot(a, b)))))


def qslerp(a, b, t):
    d = float(np.dot(a, b))
    if d < 0: b, d = -b, -d
    if d > 0.9995: q = a + t * (b - a); return q / np.linalg.norm(q)
    th = math.acos(d); return (math.sin((1 - t) * th) * a + math.sin(t * th) * b) / math.sin(th)


def toward(a, b, max_angle):                          # rotate a toward b by at most max_angle
    ang = qangle(a, b)
    return b / np.linalg.norm(b) if ang <= max_angle else qslerp(a, b, max_angle / ang)


LAYOUTS = {   # raw joydev numbering of the two 8BitDo Ultimate 2C modes (axis strings: "-1" = axis 1 negated)
    "dinput": dict(ax=dict(x="-1", y="-0", z="-5", roll="2", pitch="-3", yaw="-4"),   # Bluetooth, Paiva's map
                   btn=dict(z_up=6, yaw_ccw=7, close=0, open=1, engage=11, disengage=10, home=3)),          # home = X (hold)
    "xinput": dict(ax=dict(x="-1", y="-0", z="2", roll="3", pitch="-4", yaw="5"),     # USB / 2.4 GHz dongle ("Generic X-Box pad")
                   btn=dict(z_up=4, yaw_ccw=5, close=0, open=1, engage=7, disengage=6, home=2)),           # home = X (hold)
}


def find_pad():
    """(device node, layout) of the first 8BitDo joydev in sysfs, else (None, None); the node number changes when it re-plugs."""
    try:
        for j in sorted(os.listdir("/sys/class/input")):
            if not j.startswith("js"): continue
            name = open(f"/sys/class/input/{j}/device/name").read()
            if "X-Box" in name: return f"/dev/input/{j}", "xinput"
            if "8BitDo" in name: return f"/dev/input/{j}", "dinput"
    except OSError:
        pass
    return None, None


class Fr3JoyTeleop(Node):
    def __init__(self):
        super().__init__("fr3_joy_teleop")
        P = lambda n, d: self.declare_parameter(n, d).value
        self.base, self.ee = P("base_frame", "fr3_link0"), P("tool_frame", "fr3_hand_tcp")
        self.ctrl_frame = P("control_frame", "fr3_hand_tcp")
        self.t_tc, self.q_tc = np.zeros(3), np.array([0.0, 0.0, 0.0, 1.0])   # control frame in the tool frame
        self.input_source = str(P("input_source", "joystick"))
        if self.input_source not in ("joystick", "zed"): raise ValueError("input_source must be joystick or zed")
        self.zed_orientation_mode = str(P("zed_orientation_mode", "full_6d"))
        if self.zed_orientation_mode not in ("roll", "full_6d"): raise ValueError("invalid ZED orientation mode")
        self.zed_timeout = float(P("zed_timeout", 0.35))
        self.zed_recovery_timeout=float(P("zed_recovery_timeout",1.5))
        self.zed_calibration_port=int(P("zed_calibration_port",0))
        self.zed_diagnostics=None;self.zed_diag_received=0.
        if not 0.35<=self.zed_recovery_timeout<=30 or not 0<=self.zed_calibration_port<=65535:raise ValueError("invalid ZED recovery/calibration settings")
        self.zed_scale = float(P("zed_scale", 1.0))  # wrist detector already scales human -> FR3
        self.zed_forward_sign = float(P("zed_forward_sign", 1.0))   # +1: hand forward = robot toward the table (user, 10-22)
        if self.zed_forward_sign not in (-1., 1.): raise ValueError("zed_forward_sign must be -1 or +1")
        if not (0 < self.zed_timeout <= 1 and 0 < self.zed_scale <= 2):
            raise ValueError("invalid ZED timeout/speed/scale")
        self.zed_sample, self.zed_received, self.zed_stamp = None, 0.0, -1
        self.zed_anchor = None
        self.zed_gesture, self.zed_gesture_stamp = None, -1
        self.grip_candidate, self.grip_since, self.grip_frames = None, 0., 0
        self.grip_last, self.grip_sent, self.grip_time = None, -1, 0.
        self.zed_phase, self.zed_note = "idle", "Clique Start teleop ZED"
        self.zed_deadline, self.zed_loss_since = 0., None
        self.zed_stable, self.zed_stable_stamp = [], -1
        self.zed_loss_sample=None
        self.roll_received = 0.0
        self.zed_position_only_protocol=False;self.zed_orientation_valid=True;self.zed_rotation_pending=False;self.zed_recovery_pair=None
        self.zed_require_roll = bool(P("zed_require_roll", True))
        self.home_script = str(P("home_script", str(Path(__file__).with_name("fr3_home_joints_effort.py"))))
        self.ctrl_active, self.ctrl_checked = False, 0.0
        self.rate = float(P("rate", 50.0))
        self.joy_device = P("joy_device", "")
        # mapping: "-1" = axis 1 negated (their teleop.yaml strings), buttons are indices
        self.ax = {k: str(P(f"axis_{k}", d)) for k, d in (("x", "-1"), ("y", "-0"), ("z", "-5"), ("roll", "2"),
                                                          ("pitch", "-3"), ("yaw", "-4"))}
        self.btn = {k: int(P(f"{k}_button", d)) for k, d in (("z_up", 6), ("yaw_ccw", 7), ("close", 0), ("open", 1),
                                                             ("engage", 11), ("disengage", 10), ("home", 3))}
        self.home_hold = float(P("home_hold_seconds", 2.0))      # hold X this long to send the arm home (Paiva's home_hold_seconds)
        self.home_proc, self.home_since, self.home_sent = None, None, False
        self.home_result = None; self.home_output = None
        self.trigger_axes = {abs(int(self.ax["z"])), abs(int(self.ax["yaw"]))}
        self.trigger_rest, self.trigger_dz = float(P("trigger_rest", -1.0)), float(P("trigger_deadzone", 0.06))
        self.deadzone, self.curve = float(P("deadzone", 0.08)), float(P("curve", 1.2))
        self.v_lin, self.v_z = float(P("linear_speed", 0.15)), float(P("vertical_speed", 0.08))
        self.w_ang = math.radians(float(P("angular_speed_deg", 40.0)))
        # live sensitivity: a fraction of the speeds above, 0..1. It can only slow the arm down from the start-up speeds, never exceed them.
        self.speed_scale = min(1.0, max(0.0, float(P("speed_scale", 1.0))))      # translation (horizontal and vertical)
        self.rot_scale = min(1.0, max(0.0, float(P("rot_scale", 1.0))))          # rotation, its own slider
        self.input_timeout = float(P("input_timeout", 0.0))
        self.remote_topic, self.remote_timeout = str(P("remote_joy_topic", "")), float(P("remote_timeout", 0.25))
        self.max_step, self.max_slew = float(P("max_step", 0.005)), math.radians(float(P("max_slew_deg", 2.0)))
        self.tau, self.state_timeout = float(P("follower_time_constant", 0.04)), float(P("timeout", 0.3))
        self.max_lead = float(P("max_lead", 0.05))
        self.max_error, self.max_rot_err = float(P("max_error", 0.04)), math.radians(float(P("max_rot_error_deg", 15.0)))
        self.use_box = bool(P("use_workspace_box", False))   # OFF: no Cartesian clamp, joint limits live in the controller/cuRobo
        self.box_lo, self.box_hi = np.array(P("workspace_min", [0.20, -0.55, 0.03])), np.array(P("workspace_max", [0.80, 0.55, 0.80]))
        self.grasp = dict(width=float(P("grasp_width", 0.0)), force=float(P("grasp_force", 20.0)),
                          eps=float(P("grasp_epsilon", 0.08)), speed=float(P("gripper_speed", 0.1)),
                          max_width=float(P("max_width", 0.08)))
        ctrl = P("controller", "cartesian_impedance_target_controller").rstrip("/")
        self.ctrl = ctrl
        self.pub = self.create_publisher(PoseStamped, f"/{ctrl}/equilibrium_pose", 1)
        self.mpc_routed=self.pub.topic_name.startswith('/fr3_curobo/')
        self.curobo_status=None;self.curobo_status_at=0.
        self.create_subscription(String,'/fr3_curobo/status',self._on_curobo_status,1)
        self.engaged_pub = self.create_publisher(Bool, "/teleop/engaged", 1)
        self.scale_pub = self.create_publisher(Float32MultiArray, "/teleop/speed_scale",
                                               QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE))
        self.map_pub = self.create_publisher(String, "/teleop/pad_map",    # the bindings actually in use, for the dashboard's pad check
                                             QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE))
        self.pad_test_until = 0.0                          # pad check running (renewed by the dashboard every 2 s): input is ignored until this time
        self.declare_parameter("pad_test", False)
        self._pub_scale()
        self.debug_pub = self.create_publisher(PoseStamped, "/teleop/command_tool", 1)   # internal command, tool frame
        self.create_service(Trigger, "/teleop/zed_start", lambda rq, rs: self._srv(rs, self.start_zed()))
        self.create_service(Trigger, "/teleop/stop", lambda rq, rs: self._srv(rs, self.stop_teleop()))
        self.create_service(Trigger, "/teleop/zed_engage", lambda rq, rs: self._srv(rs, self.engage_zed()))
        self.create_service(Trigger, "/teleop/engage", lambda rq, rs: self._srv(rs, self.engage(via_service=True)))
        self.create_service(Trigger, "/teleop/disengage", lambda rq, rs: self._srv(rs, self.disengage()))
        self.create_service(Trigger, "/teleop/home", lambda rq, rs: self._srv(rs, self.start_home()))
        gns = P("gripper_ns", "/franka_gripper").rstrip("/")
        self.grasp_ac = ActionClient(self, Grasp, f"{gns}/grasp") if Grasp else None
        self.move_ac = ActionClient(self, Move, f"{gns}/move") if Move else None
        self.tf = Buffer(); TransformListener(self.tf, self)
        self.axes, self.buttons, self.prev_buttons, self.last_input = {}, {}, {}, 0.0
        self.lock = threading.Lock(); self.engaged = False; self.cmd_p = self.cmd_q = self.tgt_p = self.tgt_q = None
        self.recenter_required = False   # service/auto re-engage (recovery): pad sticks must read neutral once before motion resumes
        self.n_pub = self.n_hold = 0
        self.pad_layout = ""
        self.r_axes, self.r_buttons, self.r_last, self.src, self.prev_local = {}, {}, 0.0, "local", {}
        self.rx = LAYOUTS["xinput"]                   # the remote slot is always raw XInput numbering
        if self.remote_topic: self.create_subscription(Joy, self.remote_topic, self._on_remote, 10)
        self._pub_map()
        self.source_pub = self.create_publisher(String, "/teleop/input_source",
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE))
        self.source_pub.publish(String(data=self.input_source))
        self.zed_status_pub = self.create_publisher(String, "/teleop/zed_status", 1)
        self.create_timer(0.2, self._zed_status)
        self.cm_client = self.create_client(ListControllers, "/controller_manager/list_controllers")
        self.cm_future = None
        self.create_timer(0.5, self._check_controller)
        self.create_timer(0.5, lambda: self.engaged_pub.publish(Bool(data=self.engaged)))
        self.zed_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.zed_socket.bind(("127.0.0.1", int(P("zed_port", 47813))))
        self.zed_socket.setblocking(False)
        self.create_timer(0.01, self._receive_zed)
        self.add_on_set_parameters_callback(self._on_params)
        if self.joy_device: threading.Thread(target=self._joydev, daemon=True).start()
        else: self.create_subscription(Joy, "/joy", self._on_joy, 10)
        self.create_timer(1.0 / self.rate, self.tick); self.create_timer(2.0, self._status); self.t_last = time.monotonic()
        self.get_logger().info(f"fr3_joy_teleop: {'joydev ' + self.joy_device if self.joy_device else '/joy'} -> "
                               f"/{ctrl}/equilibrium_pose @ {self.rate:.0f} Hz, pad moves {self.ee}, controller frame "
                               f"{self.ctrl_frame}; press Start to engage")

    def _pub_scale(self):                             # [scale, max horizontal m/s, max vertical m/s] for the dashboard
        # Both inputs share the joystick's ceilings and dashboard multipliers.
        linear, vertical, angular = self.v_lin, self.v_z, self.w_ang
        self.scale_pub.publish(Float32MultiArray(data=[float(self.speed_scale), float(linear), float(vertical),
                                                       float(self.rot_scale), float(math.degrees(angular)),
                                                       float(self.grasp["force"])]))   # [scale, max m/s, max vertical m/s, rot scale, max deg/s, grasp force N]

    def _pub_map(self):
        self.map_pub.publish(String(data=json.dumps(dict(layout=self.pad_layout or "yaml", ax=self.ax, btn=self.btn, deadzone=self.deadzone,
                                                         trigger_axes=sorted(self.trigger_axes), trigger_rest=self.trigger_rest))))

    def _on_params(self, params):
        for p in params:
            if p.name == "input_source" and p.value not in ("joystick", "zed"):
                return SetParametersResult(successful=False, reason="input_source must be joystick or zed")
            if p.name.startswith("zed_") or p.name in ("controller", "home_script"):
                return SetParametersResult(successful=False, reason="startup-only parameter")
        for p in params:
            if p.name == "input_source" and p.value != self.input_source:
                self.disengage()
                self.input_source = str(p.value)
                self.zed_anchor = None
                self.source_pub.publish(String(data=self.input_source))
                self._pub_scale()
            if p.name == "pad_test":
                if p.type_ != p.Type.BOOL: return SetParametersResult(successful=False, reason="pad_test must be a bool")
                was = time.monotonic() < self.pad_test_until
                self.pad_test_until = time.monotonic() + 5.0 if p.value else 0.0      # self-expires: a closed tab cannot leave it on
                if p.value and self.engaged: self.disengage()                         # the arm keeps holding its last target
                if bool(p.value) != was: self.get_logger().info("pad check: pad input ignored" if p.value else "pad check over: pad input active again")
            if p.name == "rot_scale":
                if p.type_ not in (p.Type.DOUBLE, p.Type.INTEGER) or not (0.0 <= float(p.value) <= 1.0):
                    return SetParametersResult(successful=False, reason="rot_scale must be within 0..1 (a fraction of angular_speed_deg)")
                self.rot_scale = float(p.value); self._pub_scale()
                self.get_logger().info(f"rot_scale -> {self.rot_scale:.2f} ({self.rot_scale * math.degrees(self.w_ang):.0f} deg/s)")
            if p.name == "grasp_force":                    # live: used by the NEXT grasp goal (press close again to re-grasp a held object)
                if p.type_ not in (p.Type.DOUBLE, p.Type.INTEGER) or not (1.0 <= float(p.value) <= 70.0):
                    return SetParametersResult(successful=False, reason="grasp_force must be within 1..70 N (Franka Hand continuous force)")
                self.grasp["force"] = float(p.value); self._pub_scale()
                self.get_logger().info(f"grasp_force -> {self.grasp['force']:.0f} N (next grasp)")
            if p.name == "speed_scale":
                if p.type_ not in (p.Type.DOUBLE, p.Type.INTEGER) or not (0.0 <= float(p.value) <= 1.0):
                    return SetParametersResult(successful=False, reason="speed_scale must be within 0..1 (a fraction of the start-up speeds)")
                self.speed_scale = float(p.value); self._pub_scale()
                self.get_logger().info(f"speed_scale -> {self.speed_scale:.2f} ({self.speed_scale * self.v_lin * 100:.0f} cm/s horizontal)")
        return SetParametersResult(successful=True)

    # ---- input -------------------------------------------------------------------------------------------------------
    def _zed_status(self):
        self.zed_status_pub.publish(String(data=json.dumps(dict(
            tracking=bool(self._zed_live()), reason=self._zed_ready(), controller_reason=self._zed_controller_ready(), orientation_mode=self.zed_orientation_mode,
            diagnostics=self.zed_diagnostics if time.monotonic()-self.zed_diag_received<1. else None,
            orientation_valid=bool(self._zed_live() and self.zed_orientation_valid),
            phase=self.zed_phase, note=self.zed_note, home_running=self._home_running(), home_result=self.home_result,
            gesture=self.zed_gesture if self._zed_live() else None, gripper_command=self.grip_last,
            stability=min(1., (self.zed_stable[-1][0]-self.zed_stable[0][0])/.65) if len(self.zed_stable)>1 else 0.,
            age_ms=round((time.monotonic() - self.zed_received)*1000) if self.zed_received else None))))

    def _check_controller(self):
        if self.cm_future is not None and not self.cm_future.done(): return
        if not self.cm_client.service_is_ready():
            self.ctrl_active = False
            return
        self.cm_future = self.cm_client.call_async(ListControllers.Request())
        def done(f):
            try:
                self.ctrl_active = any(c.name == self.ctrl and c.state == "active" for c in f.result().controller)
                self.ctrl_checked = time.monotonic()
            except Exception: self.ctrl_active = False
        self.cm_future.add_done_callback(done)

    def _receive_zed(self):
        for _ in range(16):
            try: raw, peer = self.zed_socket.recvfrom(4096)
            except BlockingIOError: return
            try:
                d = json.loads(raw)
                if not isinstance(d, dict): continue
                if peer[0] != "127.0.0.1": continue
                if d.get("kind")=="tracking":
                    age=(self.get_clock().now().nanoseconds-int(d["stamp_ns"]))*1e-9
                    if d.get("version")==1 and 0<=age<self.zed_timeout:
                        self.zed_diagnostics=d.get("diagnostics",{})
                        self.zed_diag_received=time.monotonic()
                    continue  # Diagnostic messages never renew the motion watchdog.
                position_protocol=d.get('version')==3 and self.zed_orientation_mode=='full_6d'
                orientation_valid=True
                if self.zed_orientation_mode == "full_6d":
                    if d.get("version") not in (2,3) or d.get("orientation_mode") != "full_6d": continue
                    if position_protocol:
                        if d.get('position_valid') is not True:raise ValueError('invalid wrist')
                        if type(d.get('orientation_valid')) is not bool:raise ValueError('orientation validity missing')
                        orientation_valid=d['orientation_valid']
                    if orientation_valid and int(d["orientation_stamp_ns"]) != int(d["stamp_ns"]): raise ValueError("unpaired orientation")
                elif d.get("version") != 1: continue
                # The placeholder below is internal only: validity remains false and no TF is fabricated.
                quat=d['q'] if orientation_valid else (self.zed_sample[1].tolist() if self.zed_sample is not None else [0.,0.,0.,1.])
                if len(d["p"]) != 3 or len(quat) != 4: raise ValueError("pose shape")
                stamp = int(d["stamp_ns"])
                m = PoseStamped(); m.header.frame_id = str(d["frame"])
                m.header.stamp.sec, m.header.stamp.nanosec = divmod(stamp, 10**9)
                m.pose.position.x, m.pose.position.y, m.pose.position.z = map(float, d["p"])
                m.pose.orientation.x, m.pose.orientation.y, m.pose.orientation.z, m.pose.orientation.w = map(float, quat)
                roll_age = (self.get_clock().now().nanoseconds - int(d["orientation_stamp_ns"] if self.zed_orientation_mode == "full_6d" else d["roll_stamp_ns"])) * 1e-9 if orientation_valid else None
                previous_stamp = self.zed_stamp
                self._on_zed(m)
                if self.zed_sample is not None and self.zed_stamp == stamp and stamp > previous_stamp:
                    self.zed_position_only_protocol=position_protocol
                    self.zed_orientation_valid=orientation_valid
                    if orientation_valid and 0<=roll_age<self.zed_timeout:self.roll_received=time.monotonic()-roll_age
                    gesture = d.get("gesture") if orientation_valid else None
                    score = float(d.get("gesture_score",0.))
                    self.zed_gesture = gesture if gesture in ("Open_Palm","Closed_Fist") and math.isfinite(score) and score >= .75 else None
                    self.zed_gesture_stamp = stamp
            except (ValueError, KeyError, TypeError, OverflowError, AssertionError):
                self.zed_sample = None

    def _on_zed(self, m):
        p, q = m.pose.position, m.pose.orientation
        xyz, quat = np.array([p.x, p.y, p.z]), np.array([q.x, q.y, q.z, q.w])
        stamp = m.header.stamp.sec * 1000000000 + m.header.stamp.nanosec
        age = (self.get_clock().now().nanoseconds - stamp) * 1e-9
        with self.lock:
            if (m.header.frame_id != self.base or not np.all(np.isfinite(np.r_[xyz, quat])) or
                    not .9 < np.linalg.norm(quat) < 1.1 or not 0 <= age < self.zed_timeout):
                self.zed_sample = None
                return
            if stamp <= self.zed_stamp: return  # replayed samples cannot renew the watchdog
            self.zed_sample = (xyz, quat / np.linalg.norm(quat))
            self.zed_stamp, self.zed_received = stamp, time.monotonic() - age

    def _zed_live(self):
        now = time.monotonic()
        return (self.zed_sample is not None and now - self.zed_received < self.zed_timeout and
                (self.zed_position_only_protocol or not self.zed_require_roll or now - self.roll_received < self.zed_timeout))

    def _zed_ready(self):
        if not self._zed_live(): return "ZED wrist/hand tracking absent or stale"
        return self._zed_controller_ready()

    def _on_curobo_status(self,message):
        try:self.curobo_status=json.loads(message.data);self.curobo_status_at=time.monotonic()
        except (ValueError,TypeError):self.curobo_status=None

    def _zed_controller_ready(self):
        if self.mpc_routed:
            if self.curobo_status is None or time.monotonic()-self.curobo_status_at>.6:return 'cuRobo bridge absent/stale'
            return None if self.curobo_status.get('mode_ready') else 'cuRobo controller path not ready' 
        if not self.ctrl_active or time.monotonic() - self.ctrl_checked > 1.5:
            return "Cartesian impedance controller not confirmed active"
        if self.pub.get_subscription_count() < 1: return "controller has no pose subscription"
        if len(self.get_publishers_info_by_topic(f"/{self.ctrl}/equilibrium_pose")) != 1:
            return "another publisher is commanding the controller"
        return None

    def _clear_input(self):                           # a vanished pad must not leave its last stick values behind
        with self.lock: self.axes, self.buttons = {}, {}

    def _use_layout(self, layout):
        L = LAYOUTS[layout]; self.ax, self.btn = dict(L["ax"]), dict(L["btn"]); self.pad_layout = layout
        self.trigger_axes = {abs(int(self.ax["z"])), abs(int(self.ax["yaw"]))}
        self._pub_map()

    def _joydev(self):                                # struct js_event {u32 time; s16 value; u8 type; u8 number;}
        while rclpy.ok():
            dev = self.joy_device
            if dev == "auto":                         # follow the pad across re-plugs and mode changes (js0 <-> js1, BT <-> USB)
                dev, layout = find_pad()
                if dev is None:
                    time.sleep(1.0); self.get_logger().warn("no 8BitDo pad found; waiting", throttle_duration_sec=10.0); continue
                self._use_layout(layout)
            try:
                with open(dev, "rb") as f:
                    self.get_logger().info(f"opened {dev}" + (f" ({self.pad_layout} map)" if self.joy_device == "auto" else ""))
                    while rclpy.ok():
                        _, value, typ, num = struct.unpack("IhBB", f.read(8))
                        with self.lock:
                            if typ & 0x02: self.axes[num] = value / 32767.0
                            elif typ & 0x01: self.buttons[num] = int(value)
                            self.last_input = time.monotonic()
            except (OSError, struct.error) as e:
                self._clear_input()
                self.get_logger().warn(f"{dev}: {e}; input zeroed, retrying in 1 s", throttle_duration_sec=10.0); time.sleep(1.0)

    def _on_joy(self, m):
        with self.lock:
            self.axes = dict(enumerate(m.axes)); self.buttons = dict(enumerate(m.buttons)); self.last_input = time.monotonic()

    def _on_remote(self, m):
        with self.lock:
            self.r_axes = dict(enumerate(m.axes)); self.r_buttons = dict(enumerate(m.buttons)); self.r_last = time.monotonic()

    def _axis(self, spec, axes, trig=None):
        trig = self.trigger_axes if trig is None else trig
        i = abs(int(spec)); s = -1.0 if spec.startswith("-") else 1.0
        v = axes.get(i, self.trigger_rest if i in trig else 0.0)   # a missing trigger reads released, not half-pressed
        if i in trig:                                 # trigger: rest -> 0, full -> 1, sign ignored
            v = (v - self.trigger_rest) / (1.0 - self.trigger_rest)
            return 0.0 if v < self.trigger_dz else min(1.0, (v - self.trigger_dz) / (1.0 - self.trigger_dz))
        return s * v

    def _shape(self, x, y):                           # radial deadzone + response curve on a stick pair
        r = math.hypot(x, y)
        if r < self.deadzone: return 0.0, 0.0
        g = ((min(r, 1.0) - self.deadzone) / (1.0 - self.deadzone)) ** self.curve / r
        return x * g, y * g

    # ---- engage / gripper ---------------------------------------------------------------------------------------------
    def _measured(self, parent=None, child=None):
        try:
            t = self.tf.lookup_transform(parent or self.base, child or self.ee, Time())
        except Exception:
            return None
        age = (self.get_clock().now() - Time.from_msg(t.header.stamp)).nanoseconds * 1e-9
        tr, ro = t.transform.translation, t.transform.rotation
        return np.array([tr.x, tr.y, tr.z]), np.array([ro.x, ro.y, ro.z, ro.w]), age

    # ---- home move (hold X; Paiva's /teleop/home): the home script runs as a child process, the teleop stays disengaged until Start --------
    def _home_running(self):
        if self.home_proc is not None and self.home_proc.poll() is not None:
            code = self.home_proc.returncode
            try:
                lines = Path(self.home_output.name).read_text().splitlines()
                summary = next((line.split("HOME FAILED:",1)[1].strip() for line in reversed(lines) if "HOME FAILED:" in line), None)
            except (OSError, AttributeError): summary = None
            self.home_result = "Home concluído · controle parado" if code == 0 else f"Home falhou: {summary or ('exit ' + str(code))}"
            self.zed_note = self.home_result
            if self.home_output is not None: self.home_output.close(); self.home_output = None
            self.get_logger().info(self.home_result)
            self.home_proc = None
        return self.home_proc is not None

    def start_home(self):
        if self._home_running(): return False, "home is already running"
        self.disengage()
        script = self.home_script
        try:
            self.home_output = open("/runs/fr3-zed-home.log", "w")
            self.home_proc = subprocess.Popen(["python3", script], stdout=self.home_output, stderr=subprocess.STDOUT)
            self.home_result = None
        except OSError as e:
            if self.home_output is not None: self.home_output.close(); self.home_output = None
            return False, f"could not start the home script: {e}"
        self.get_logger().warn("HOME started: effort-JTC joint move with path-model and following guards; Back aborts")
        return True, "home started"

    def _abort_home(self):
        if self._home_running():
            self.get_logger().warn("Back during home: aborting (the arm holds where it is, the impedance controller is restored)")
            self.home_proc.send_signal(signal.SIGINT)

    def _home_hold(self, buttons, btn, remote, now):          # local pad only: a browser pad never starts a home move
        b = btn.get("home")
        if remote or b is None or not buttons.get(b, 0):
            self.home_since, self.home_sent = None, False; return
        if self.home_since is None:
            self.home_since = now
            self.get_logger().warn(f"home button held: keep holding for {self.home_hold:.1f} s to send the arm home (joint-space move)")
            return
        if not self.home_sent and now - self.home_since >= self.home_hold:
            self.home_sent = True
            ok, msg = self.start_home(); self.get_logger().info(f"home: {msg}")

    def stop_teleop(self):
        self.disengage()
        self._abort_home()
        return True, "Controle parado"

    def start_zed(self):
        if self._home_running(): return False, "Home em andamento"
        if self.input_source=="zed" and (self.engaged or self.zed_phase=="calibrating"):return True,self.zed_note
        # Source selection and calibration belong to one ROS callback, not two browser requests.
        from rclpy.parameter import Parameter
        self.set_parameters([Parameter("input_source", value="zed")])
        self.disengage()
        if self.zed_calibration_port:
            with socket.socket(socket.AF_INET,socket.SOCK_DGRAM) as control:
                control.sendto(b'{"action":"calibrate_arm","version":1}',('127.0.0.1',self.zed_calibration_port))
            self.zed_sample=None;self.zed_received=0.;self.roll_received=0.
        self.home_result = None
        self.zed_phase, self.zed_note = "calibrating", "Calibrando: mostre ombro, cotovelo e punho direitos; depois mantenha a mão parada"
        self.zed_deadline = time.monotonic() + 60.
        self.zed_stable_stamp = self.zed_stamp  # only frames captured AFTER this click may arm
        self.zed_stable = []
        return True, self.zed_note

    def _stable_zed(self, now):
        if not self._zed_live():
            if self.zed_stable and now-self.zed_stable[-1][0]>.8:self.zed_stable=[]
            return False
        if self.zed_stamp <= self.zed_stable_stamp: return False
        self.zed_stable_stamp = self.zed_stamp
        p, q = self.zed_sample
        if self.zed_stable and (now-self.zed_stable[-1][0] > .8 or
                any(np.linalg.norm(p-a[1]) > .04 or (not self.zed_position_only_protocol and self.zed_orientation_valid and qangle(q,a[2]) > math.radians(12))
                    for a in self.zed_stable)):
            self.zed_stable = []
        self.zed_stable.append((now, p.copy(), q.copy()))
        return len(self.zed_stable) >= 4 and now-self.zed_stable[0][0] >= .65

    def _recover_moving_zed(self):
        """Two fresh, consistent wrists can re-anchor without requiring a stationary hand."""
        if not self._zed_live():return False
        stamp=self.zed_stamp;p=self.zed_sample[0]
        previous=self.zed_recovery_pair
        if previous is not None and stamp<=previous[0]:return False
        self.zed_recovery_pair=(stamp,p.copy())
        if previous is None:return False
        dt=(stamp-previous[0])*1e-9
        return 0<dt<=.5 and np.linalg.norm(p-previous[1])<=.03+2.*dt

    def _advance_zed_start(self, now):
        if self.zed_phase != "calibrating": return
        if now > self.zed_deadline:
            self.disengage()
            self.zed_note = "Calibração expirou. Mostre ombros e mão direita; clique Start novamente."
            return
        why=self._zed_ready()
        if why:
            if why != "ZED wrist/hand tracking absent or stale":self.zed_stable=[]
            return
        if self._stable_zed(now):
            ok, why = self.engage_zed()
            if not ok:
                self.disengage()
                self.zed_note = why

    def engage_zed(self):
        if self.input_source != "zed": return False, "Select ZED3 first"
        return self.engage()

    def engage(self, via_service=False):
        if self.mpc_routed:
            if self.input_source not in ('joystick','zed'):return False,'cuRobo path accepts the joystick or the ZED only'   # ZED allowed through cuRobo (10-23); the bridge gates ZED goals on zed_status phase=active
            why=self._zed_controller_ready()
            if why:return False,why
            if not self.curobo_status.get('command_ready'):return False,'cuRobo hold not ready: '+str(self.curobo_status.get('packet_reason'))
        # Service engages (recovery/resume path) arm a re-center guard for the
        # pad: deflected sticks must return to neutral once before motion
        # resumes, so the arm never moves without a fresh deliberate input
        # after a fault. The Start-button path (via_service=False) drives at
        # once; pressing Start IS the deliberate action. ZED needs no guard:
        # engage anchors to the live hand, so there is no jump to suppress.
        if self.input_source == "zed":
            why = self._zed_ready()
            if why: return False, why
        if self._home_running(): return False, "home move running: wait for it (Back aborts)"
        m = self._measured()
        if m is None: return False, f"no TF {self.base}->{self.ee}"
        p, q, age = m
        if age < 0 or age > self.state_timeout or not np.all(np.isfinite(np.r_[p, q])):
            return False, "robot state absent/stale/invalid"
        if self.use_box and self.input_source == "zed" and (np.any(p < self.box_lo) or np.any(p > self.box_hi)):
            return False, "TCP outside workspace; reposition with joystick before ZED engage"
        tc = self._measured(self.ee, self.ctrl_frame)
        if tc is None: return False, f"no TF {self.ee}->{self.ctrl_frame}"
        self.t_tc, self.q_tc = tc[0], tc[1]
        with self.lock:
            self.cmd_p, self.cmd_q, self.tgt_p, self.tgt_q, self.engaged = p.copy(), q.copy(), p.copy(), q.copy(), True
            self.meas_p, self.meas_q = p.copy(), q.copy()
            guard = via_service and self.input_source != "zed"
            self.recenter_required = guard
            if self.input_source == "zed":
                if not self._zed_live():
                    self.engaged = False
                    return False, "ZED tracking changed during engage"
                zp, zq = self.zed_sample
                self.zed_anchor = (p.copy(), q.copy(), zp.copy(), zq.copy())
                self.zed_rotation_pending=not self.zed_orientation_valid
        if self.input_source == "zed":
            self.zed_phase, self.zed_note = "active", "ZED ativa · translação e rotação 6D"
            self.zed_loss_since = None
            self.zed_recovery_pair=None
            self.zed_loss_sample=None
        self._publish(); self.engaged_pub.publish(Bool(data=True))
        self.get_logger().info(f"engaged at {np.round(p, 3).tolist()}" + (" (sticks must re-center once before motion)" if guard else ""))
        return True, ("engaged, re-center the sticks once to drive" if guard else "engaged")

    def disengage(self):
        self.zed_phase, self.zed_note = "idle", "Controle parado · clique Start teleop ZED"
        self.zed_loss_since, self.zed_stable = None, []
        self.zed_loss_sample=None
        self.grip_candidate, self.grip_since, self.grip_frames = None, 0., 0
        self.grip_last, self.grip_sent = None, self.zed_stamp
        with self.lock: self.engaged = False; self.recenter_required = False
        self.engaged_pub.publish(Bool(data=False)); self.get_logger().info("disengaged: the controller holds the last pose")
        return True, "disengaged"

    @staticmethod
    def _srv(rs, r):
        rs.success, rs.message = r; return rs

    def _zed_gripper(self, now):
        if not self.engaged or self.zed_phase != "active" or not self._zed_live():
            self.grip_candidate, self.grip_frames = None, 0
            return
        if self.zed_gesture_stamp <= self.grip_sent: return
        self.grip_sent = self.zed_gesture_stamp
        gesture = self.zed_gesture
        if gesture is None:
            self.grip_candidate, self.grip_frames = None, 0
            return
        if gesture != self.grip_candidate or now-self.grip_since > 2.:
            self.grip_candidate, self.grip_since, self.grip_frames = gesture, now, 1
        else: self.grip_frames += 1
        if self.grip_frames >= 3 and now-self.grip_since >= .35 and gesture != self.grip_last and now-self.grip_time >= 1.:
            self.grip_last, self.grip_time = gesture, now
            self._gripper(gesture == "Closed_Fist")

    def _gripper(self, close):
        ac = self.grasp_ac if close else self.move_ac
        if ac is None or not ac.server_is_ready():
            self.get_logger().warn(f"gripper {'grasp' if close else 'move'} action not available", throttle_duration_sec=5.0); return
        if close:
            g = Grasp.Goal(); g.width, g.speed, g.force = self.grasp["width"], self.grasp["speed"], self.grasp["force"]
            g.epsilon.inner = g.epsilon.outer = self.grasp["eps"]
        else:
            g = Move.Goal(); g.width, g.speed = self.grasp["max_width"], self.grasp["speed"]
        ac.send_goal_async(g); self.get_logger().info("gripper close" if close else "gripper open")

    # ---- loop ----------------------------------------------------------------------------------------------------------
    def tick(self):
        now = time.monotonic(); dt = min(0.1, now - self.t_last); self.t_last = now
        with self.lock:
            axes, buttons, prev, last_in = dict(self.axes), dict(self.buttons), dict(self.prev_buttons), self.last_input
            r_axes, r_buttons, r_last = dict(self.r_axes), dict(self.r_buttons), self.r_last
        ax, btn, trig = self.ax, self.btn, self.trigger_axes
        remote = bool(self.remote_topic) and now - r_last < self.remote_timeout
        local_back = buttons.get(self.btn["disengage"], 0) and not self.prev_local.get(self.btn["disengage"], 0)
        self.prev_local = dict(buttons)
        if self.src == "remote" and not remote and self.engaged:      # the remote driver went away: a deliberate Start is needed again
            self.get_logger().warn("remote pad lost while engaged: disengaging"); self.disengage()
        if local_back:                    # local emergency stop beats the remote driver
            self.get_logger().warn("local Back during remote control: disengaging"); self.disengage()
        if remote:
            ax, btn, trig = self.rx["ax"], self.rx["btn"], {abs(int(self.rx["ax"]["z"])), abs(int(self.rx["ax"]["yaw"]))}
            axes, buttons, last_in = r_axes, r_buttons, r_last
        if (self.src == "remote") != remote:                          # source switch: no button edge may carry over between the two pads
            self.get_logger().info(f"input source: {'remote' if remote else 'local'}"); self.src = "remote" if remote else "local"
            prev = dict(buttons)
        self.prev_buttons = dict(buttons)
        if now < self.pad_test_until: return              # pad check: nothing is engaged, no gripper, no motion
        pressed = lambda k: buttons.get(btn[k], 0) and not prev.get(btn[k], 0)
        if self.input_source == "joystick" and pressed("engage") and not self.engaged: self.engage()
        if pressed("disengage"): self.disengage()
        if pressed("disengage"): self._abort_home()
        self._home_hold(buttons if self.input_source == "joystick" else {}, btn, remote, now)
        if pressed("close"): self._gripper(True)
        if pressed("open"): self._gripper(False)
        self._advance_zed_start(now)
        if not self.engaged: return
        m = self._measured()
        if m is None or m[2] < 0 or m[2] > self.state_timeout:
            if self.input_source == "zed": self.disengage()
            self.n_hold += 1; self.get_logger().warn("robot state stale: holding", throttle_duration_sec=2.0); return
        mp, mq, _ = m
        live = self.input_timeout <= 0 or now - last_in < self.input_timeout
        x, y = self._shape(self._axis(ax["x"], axes, trig), self._axis(ax["y"], axes, trig)) if live else (0.0, 0.0)
        r, p = self._shape(self._axis(ax["roll"], axes, trig), self._axis(ax["pitch"], axes, trig)) if live else (0.0, 0.0)
        z = (float(buttons.get(btn["z_up"], 0)) - self._axis(ax["z"], axes, trig)) if live else 0.0
        yaw = (float(buttons.get(btn["yaw_ccw"], 0)) - self._axis(ax["yaw"], axes, trig)) if live else 0.0
        k = self.speed_scale; v = np.array([x * self.v_lin, y * self.v_lin, z * self.v_z]) * k; w = np.array([r, p, yaw]) * self.w_ang * self.rot_scale
        if self.input_source != "zed" and self.recenter_required:   # service/auto re-engage: hold until the sticks read neutral once
            if x == 0.0 and y == 0.0 and r == 0.0 and p == 0.0 and z == 0.0 and yaw == 0.0:
                self.recenter_required = False
                self.get_logger().info("sticks re-centered after auto-engage: driving enabled")
            else: v, w = np.zeros(3), np.zeros(3)
        if self.input_source == "zed":
            why = self._zed_controller_ready()
            if why:
                self.disengage(); self.zed_note = why
                return
            # A brief dropout freezes commands immediately; never extrapolate a stale hand.
            if not self._zed_live() and self.zed_loss_since is None:
                self.zed_loss_since = min(self.zed_received, self.roll_received) if self.zed_require_roll and not self.zed_position_only_protocol else self.zed_received
                self.zed_loss_sample=(self.zed_sample[0].copy(),self.zed_sample[1].copy()) if self.zed_sample is not None else None
                self.zed_stable, self.zed_stable_stamp = [], self.zed_stamp
                self.zed_recovery_pair=None
                self.zed_phase, self.zed_note = "paused", ("Pausado · aguardando punho atual para retomar" if self.zed_position_only_protocol else "Pausado · mostre a mão parada para retomar")
                self.grip_candidate, self.grip_frames = None, 0
            if self.zed_loss_since is not None:
                if now-self.zed_loss_since > self.zed_recovery_timeout:
                    self.disengage()
                    self.zed_note = f"Rastreamento perdido por {self.zed_recovery_timeout:g} s · clique Start para recalibrar"
                    return
                if self._zed_live() and now-self.zed_loss_since<.8 and self.zed_loss_sample is not None:
                    p0,q0=self.zed_loss_sample;p1,q1=self.zed_sample
                    if np.linalg.norm(p1-p0)<.04 and (not self.zed_orientation_valid or qangle(q1,q0)<math.radians(12)):
                        self.zed_loss_since=None;self.zed_loss_sample=None
                        self.zed_phase,self.zed_note="active","ZED ativa · perda breve recuperada"
                        return  # Fresh nearby sample; preserve the original hand/robot reference.
                if self.zed_position_only_protocol:
                    if not self._recover_moving_zed():return
                elif not self._stable_zed(now):return
                # Re-anchor to actual TCP and current hand; discard movement made while invisible.
                ok, why = self.engage_zed()
                if not ok:
                    if why.startswith("TCP outside workspace"):
                        # Keep holding while the measured arm settles back inside.
                        # The existing recovery timeout still ends a persistent violation.
                        self.zed_note="Pausado · aguardando TCP dentro do limite de alcance"
                    else:
                        self.disengage(); self.zed_note = why
                return
        with self.lock:
            if self.input_source == "zed":
                ap, aq, zp0, zq0 = self.zed_anchor
                zp, zq = self.zed_sample
                # Relative operator motion; the FR3 forward/backward convention is applied below.
                delta = zp-zp0
                # FR3 operator faces the table: reaching forward reverses the Spot
                # translation convention. This is not a reflection of the SO(3) frame.
                delta = delta*np.array([self.zed_forward_sign, 1., 1.])
                self.tgt_p = ap + self.zed_scale * delta
                if self.zed_position_only_protocol and not self.zed_orientation_valid:
                    self.tgt_q=self.cmd_q.copy();self.zed_rotation_pending=True
                    self.zed_note="ZED ativa · seguindo punho, giro mantido"
                elif self.zed_position_only_protocol and self.zed_rotation_pending:
                    self.zed_anchor=(ap, self.cmd_q.copy(), zp0, zq.copy())
                    self.tgt_q=self.cmd_q.copy();self.zed_rotation_pending=False
                    self.zed_note="ZED ativa · orientação recuperada sem salto"
                elif self.zed_orientation_mode == "full_6d":
                    self.zed_note="ZED ativa · translação e rotação 6D"
                    # Both measured hand orientations are in base axes. Apply their
                    # spatial rotation on the LEFT of the robot's anchored orientation.
                    self.tgt_q = qmul(qmul(zq, zq0 * np.array([-1, -1, -1, 1])), aq)
                else:
                    self.tgt_q = qmul(aq, qmul(zq0 * np.array([-1, -1, -1, 1]), zq))
                self.tgt_q /= np.linalg.norm(self.tgt_q)
                v, w = np.zeros(3), np.zeros(3)
            raw = self.tgt_p + v * dt
            if self.use_box:
                self.tgt_p = np.clip(raw, self.box_lo, self.box_hi)
                if self.input_source=="zed" and self.zed_position_only_protocol:
                    # Remove unreachable hand travel, so reversing the hand responds immediately.
                    ap,aq,zp0,zq0=self.zed_anchor
                    self.zed_anchor=(ap+self.tgt_p-raw,aq,zp0,zq0)
            else: self.tgt_p = raw
            hit = "".join(a for a, c in zip("xyz", raw != self.tgt_p) if c)
            if hit: self.get_logger().info(f"workspace box clamps {hit} at {np.round(self.tgt_p, 3).tolist()} "
                                           f"(box {self.box_lo.tolist()} .. {self.box_hi.tolist()})", throttle_duration_sec=2.0)
            self.tgt_q = qmul(self.tgt_q, qexp(w * dt)); self.tgt_q /= np.linalg.norm(self.tgt_q)
            lead = self.tgt_p - self.cmd_p; n = np.linalg.norm(lead)           # target never runs far ahead
            if n > self.max_lead: self.tgt_p = self.cmd_p + lead * self.max_lead / n
            if qangle(self.tgt_q, self.cmd_q) > 3 * self.max_slew * self.rate * 0.1:
                self.tgt_q = toward(self.cmd_q, self.tgt_q, 3 * self.max_slew * self.rate * 0.1)
            a = -math.expm1(-dt / self.tau)                                    # their first-order follower
            step = a * (self.tgt_p - self.cmd_p); n = np.linalg.norm(step)
            if self.input_source == "zed":
                horizontal = np.linalg.norm(step[:2])
                horizontal_limit = self.v_lin * self.speed_scale * dt
                if horizontal > horizontal_limit:
                    step[:2] *= horizontal_limit / horizontal
                step[2] = np.clip(step[2], -self.v_z*self.speed_scale*dt, self.v_z*self.speed_scale*dt)
                n = np.linalg.norm(step)
            if n > self.max_step: step *= self.max_step / n
            new_p = self.cmd_p + step                                          # no windup: never step further away
            if np.linalg.norm(new_p - mp) > self.max_error and np.linalg.norm(new_p - mp) > np.linalg.norm(self.cmd_p - mp):
                self.tgt_p = self.cmd_p.copy()                                 # from an arm that cannot follow
                self.get_logger().info(f"arm not following: target reset (error {1000 * np.linalg.norm(new_p - mp):.0f} mm > "
                                       f"{1000 * self.max_error:.0f} mm leash) at {np.round(mp, 3).tolist()}", throttle_duration_sec=2.0)
            else: self.cmd_p = new_p
            slew_limit = min(self.max_slew, self.w_ang * self.rot_scale * dt) if self.input_source == "zed" else self.max_slew
            new_q = toward(self.cmd_q, qslerp(self.cmd_q, self.tgt_q, a), slew_limit)
            if qangle(new_q, mq) > self.max_rot_err and qangle(new_q, mq) > qangle(self.cmd_q, mq): self.tgt_q = self.cmd_q.copy()
            else: self.cmd_q = new_q
            self.meas_p, self.meas_q = mp, mq
        self._publish()
        if self.input_source == "zed": self._zed_gripper(now)

    def _publish(self):
        with self.lock:
            p, q, mp, mq = self.cmd_p.copy(), self.cmd_q.copy(), self.meas_p.copy(), self.meas_q.copy()
        e = p - mp; n = np.linalg.norm(e)                                      # force ceiling, as theirs (clip, not drag)
        if n > self.max_error: p = mp + e * self.max_error / n
        if qangle(q, mq) > self.max_rot_err: q = toward(mq, q, self.max_rot_err)
        d = PoseStamped(); d.header.stamp = self.get_clock().now().to_msg(); d.header.frame_id = self.base
        d.pose.position.x, d.pose.position.y, d.pose.position.z = (float(c) for c in self.cmd_p)
        d.pose.orientation.x, d.pose.orientation.y, d.pose.orientation.z, d.pose.orientation.w = (float(c) for c in self.cmd_q)
        self.debug_pub.publish(d)
        p, q = p + qrot(q, self.t_tc), qmul(q, self.q_tc)                     # tool pose -> control-frame pose
        m = PoseStamped(); m.header.stamp = self.get_clock().now().to_msg(); m.header.frame_id = self.base
        m.pose.position.x, m.pose.position.y, m.pose.position.z = (float(c) for c in p)
        m.pose.orientation.x, m.pose.orientation.y, m.pose.orientation.z, m.pose.orientation.w = (float(c) for c in q)
        self.pub.publish(m); self.n_pub += 1

    def destroy_node(self):
        self.zed_socket.close()
        return super().destroy_node()

    def _status(self):
        self.get_logger().info(f"engaged={self.engaged} published={self.n_pub} holds={self.n_hold}", throttle_duration_sec=10.0)


def main():
    rclpy.init(); n = Fr3JoyTeleop()
    try: rclpy.spin(n)
    except KeyboardInterrupt: pass
    finally:
        n.destroy_node()
        if rclpy.ok(): rclpy.shutdown()


if __name__ == "__main__":
    main()
