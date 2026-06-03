#!/usr/bin/env python3
"""Publishes a hardcoded wrist target for testing curobo MPC without the camera.

Mimics arm_pose_estimator/wrist_detector: publishes PoseStamped on /wrist_pose
and broadcasts TF body -> wrist_target.

Usage:
    ros2 run spot_operation_ros2 fake_wrist_target
    # static pose:
    ros2 run spot_operation_ros2 fake_wrist_target --ros-args \
        -p x:=0.7 -p y:=0.0 -p z:=0.4
    # continuous sine sweep:
    ros2 run spot_operation_ros2 fake_wrist_target --ros-args \
        -p animate:=True
    # step-hold cycle (left 10s → center 2s → arc right → back left → repeat):
    ros2 run spot_operation_ros2 fake_wrist_target --ros-args \
        -p step_hold:=True -p amp_y:=0.45 -p amp_z:=0.15 \
        -p hold_secs:=10.0 -p center_hold_secs:=2.0 \
        -p ramp_secs:=2.0 -p arc_secs:=5.0
    # keyboard control (W/S=x, A/D=y, Q/E=z, arrows=pitch/yaw, R/F=roll,
    #                   O/C=gripper open/close):
    ros2 run spot_operation_ros2 fake_wrist_target --ros-args \
        -p keyboard:=True
"""

import math
import select
import sys
import termios
import threading
import tty

import rclpy
from geometry_msgs.msg import PoseStamped, TransformStamped
from rclpy.node import Node
from std_msgs.msg import Float64
from tf2_ros import TransformBroadcaster


def _coslerp(a: float, b: float, t: float, duration: float) -> float:
    """Cosine interpolation from a to b. Zero velocity at both ends."""
    alpha = min(t / duration, 1.0)
    alpha = 0.5 * (1.0 - math.cos(math.pi * alpha))
    return a + (b - a) * alpha


def _euler_to_quat(roll: float, pitch: float, yaw: float):
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    qw = cr * cp * cy + sr * sp * sy
    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy
    return qx, qy, qz, qw


def _quat_to_euler(qx: float, qy: float, qz: float, qw: float):
    sinr_cosp = 2 * (qw * qx + qy * qz)
    cosr_cosp = 1 - 2 * (qx * qx + qy * qy)
    roll = math.atan2(sinr_cosp, cosr_cosp)
    sinp = max(-1.0, min(1.0, 2 * (qw * qy - qz * qx)))
    pitch = math.asin(sinp)
    siny_cosp = 2 * (qw * qz + qx * qy)
    cosy_cosp = 1 - 2 * (qy * qy + qz * qz)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return roll, pitch, yaw


_KEY_HELP = """\
Keyboard controls (wrist target):
  W / S       →  X  forward / back      (+/- {pos:.3f} m)
  A / D       →  Y  left / right        (+/- {pos:.3f} m)
  Q / E       →  Z  up / down           (+/- {pos:.3f} m)
  ↑ / ↓       →  Pitch +/-              (+/- {deg:.1f}°)
  ← / →       →  Yaw   +/-             (+/- {deg:.1f}°)
  R / F       →  Roll  +/-              (+/- {deg:.1f}°)
  O / C       →  Gripper open / close  (→ /gripper/goal)
  Ctrl+C      →  quit
"""


class _KeyboardReader(threading.Thread):
    """Non-blocking raw-terminal reader that calls callback(key_str) on each keystroke."""

    def __init__(self, callback):
        super().__init__(daemon=True)
        self._cb = callback
        self._stop = threading.Event()

    def stop(self):
        self._stop.set()

    def run(self):
        fd = sys.stdin.fileno()
        old = termios.tcgetattr(fd)
        try:
            tty.setraw(fd)
            while not self._stop.is_set():
                if not select.select([sys.stdin], [], [], 0.1)[0]:
                    continue
                ch = sys.stdin.read(1)
                if ch == '\x03':  # Ctrl+C
                    self._cb('\x03')
                    break
                if ch == '\x1b':
                    # Escape sequence — read up to 2 more bytes with short timeout
                    seq = ch
                    for _ in range(2):
                        if select.select([sys.stdin], [], [], 0.05)[0]:
                            seq += sys.stdin.read(1)
                        else:
                            break
                    self._cb(seq)
                else:
                    self._cb(ch)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old)


class FakeWristTarget(Node):
    def __init__(self):
        super().__init__('fake_wrist_target')

        self.declare_parameter('frame_id', 'body')
        self.declare_parameter('child_frame_id', 'wrist_target')
        self.declare_parameter('rate_hz', 30.0)
        self.declare_parameter('x', 0.75)
        self.declare_parameter('y', 0.0)
        self.declare_parameter('z', 0.35)
        self.declare_parameter('qx', 0.0)
        self.declare_parameter('qy', 0.0)
        self.declare_parameter('qz', 0.0)
        self.declare_parameter('qw', 1.0)
        self.declare_parameter('animate', False)
        self.declare_parameter('amp_y', 0.45)
        self.declare_parameter('amp_z', 0.15)
        self.declare_parameter('freq', 0.3)
        self.declare_parameter('step_hold', False)
        self.declare_parameter('ramp_secs', 2.0)
        self.declare_parameter('arc_secs', 5.0)
        self.declare_parameter('hold_secs', 10.0)
        self.declare_parameter('center_hold_secs', 2.0)
        self.declare_parameter('keyboard', False)
        self.declare_parameter('pos_step', 0.02)   # metres per keypress
        self.declare_parameter('rot_step_deg', 5.0) # degrees per keypress
        # Gripper: published to /gripper/goal (std_msgs/Float64), consumed by
        # gripper_controller in the cuRobo MPC launch. cuRobo filters arm_f1x out
        # of its arm command, so the gripper is driven entirely through this path.
        self.declare_parameter('gripper_topic', '/gripper/goal')
        self.declare_parameter('gripper_open_rad', -1.57)  # fully open
        self.declare_parameter('gripper_close_rad', 0.0)   # fully closed

        self.frame_id = self.get_parameter('frame_id').value
        self.child_frame_id = self.get_parameter('child_frame_id').value
        rate = float(self.get_parameter('rate_hz').value)

        self.x = float(self.get_parameter('x').value)
        self.y = float(self.get_parameter('y').value)
        self.z = float(self.get_parameter('z').value)
        self.qx = float(self.get_parameter('qx').value)
        self.qy = float(self.get_parameter('qy').value)
        self.qz = float(self.get_parameter('qz').value)
        self.qw = float(self.get_parameter('qw').value)
        self.animate = bool(self.get_parameter('animate').value)
        self.amp_y = float(self.get_parameter('amp_y').value)
        self.amp_z = float(self.get_parameter('amp_z').value)
        self.freq = float(self.get_parameter('freq').value)
        self.step_hold = bool(self.get_parameter('step_hold').value)
        self.ramp_secs = float(self.get_parameter('ramp_secs').value)
        self.arc_secs = float(self.get_parameter('arc_secs').value)
        self.hold_secs = float(self.get_parameter('hold_secs').value)
        self.center_hold_secs = float(self.get_parameter('center_hold_secs').value)
        self.keyboard = bool(self.get_parameter('keyboard').value)
        self._pos_step = float(self.get_parameter('pos_step').value)
        self._rot_step = math.radians(float(self.get_parameter('rot_step_deg').value))
        self._gripper_topic = self.get_parameter('gripper_topic').value
        self._gripper_open = float(self.get_parameter('gripper_open_rad').value)
        self._gripper_close = float(self.get_parameter('gripper_close_rad').value)

        self._t = 0.0
        self._dt = 1.0 / rate

        # step_hold state
        self._phase = 'hold_left'
        self._phase_t = 0.0

        # keyboard state (position + euler angles, protected by a lock)
        self._kb_lock = threading.Lock()
        self._kb_x = self.x
        self._kb_y = self.y
        self._kb_z = self.z
        self._kb_roll, self._kb_pitch, self._kb_yaw = _quat_to_euler(
            self.qx, self.qy, self.qz, self.qw)
        self._kb_reader = None

        self.pose_pub = self.create_publisher(PoseStamped, '/wrist_pose', 10)
        self.gripper_pub = self.create_publisher(Float64, self._gripper_topic, 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.timer = self.create_timer(1.0 / rate, self.tick)

        if self.keyboard:
            mode = 'keyboard'
        elif self.step_hold:
            mode = 'step_hold'
        elif self.animate:
            mode = 'animate'
        else:
            mode = 'static'

        self.get_logger().info(
            f'fake_wrist_target | mode={mode} | {rate:.1f} Hz | frame="{self.frame_id}" | '
            f'base=({self.x:.3f},{self.y:.3f},{self.z:.3f})'
        )
        if self.step_hold:
            self.get_logger().info(
                f'  amp_y=±{self.amp_y:.2f}  amp_z=±{self.amp_z:.2f} | '
                f'hold={self.hold_secs:.1f}s  center_hold={self.center_hold_secs:.1f}s | '
                f'ramp={self.ramp_secs:.1f}s  arc={self.arc_secs:.1f}s'
            )
        if self.keyboard:
            print(_KEY_HELP.format(pos=self._pos_step,
                                   deg=math.degrees(self._rot_step)), flush=True)
            self._kb_reader = _KeyboardReader(self._on_key)
            self._kb_reader.start()

    # ------------------------------------------------------------------
    # Keyboard handler
    # ------------------------------------------------------------------

    def _on_key(self, key: str):
        if key == '\x03':
            rclpy.shutdown()
            return

        ps = self._pos_step
        rs = self._rot_step

        # Gripper keys publish a one-shot goal; no pose state to mutate, so handle
        # them outside the kb_lock (the gripper_controller smooths the motion).
        k = key.lower()
        if k == 'o':
            self._send_gripper(self._gripper_open, 'OPEN')
            return
        if k == 'c':
            self._send_gripper(self._gripper_close, 'CLOSE')
            return

        with self._kb_lock:
            if k == 'w':
                self._kb_x += ps
            elif k == 's':
                self._kb_x -= ps
            elif k == 'a':
                self._kb_y += ps
            elif k == 'd':
                self._kb_y -= ps
            elif k == 'q':
                self._kb_z += ps
            elif k == 'e':
                self._kb_z -= ps
            elif key == '\x1b[A':   # arrow up → pitch up
                self._kb_pitch += rs
            elif key == '\x1b[B':   # arrow down → pitch down
                self._kb_pitch -= rs
            elif key == '\x1b[D':   # arrow left → yaw left
                self._kb_yaw += rs
            elif key == '\x1b[C':   # arrow right → yaw right
                self._kb_yaw -= rs
            elif k == 'r':
                self._kb_roll += rs
            elif k == 'f':
                self._kb_roll -= rs
            else:
                return  # unknown key — no log spam

            self.get_logger().info(
                f'pos=({self._kb_x:.3f},{self._kb_y:.3f},{self._kb_z:.3f})  '
                f'rpy=({math.degrees(self._kb_roll):.1f}°,'
                f'{math.degrees(self._kb_pitch):.1f}°,'
                f'{math.degrees(self._kb_yaw):.1f}°)'
            )

    def _send_gripper(self, angle: float, label: str):
        """Publish a one-shot gripper goal (rad) to /gripper/goal."""
        self.gripper_pub.publish(Float64(data=float(angle)))
        self.get_logger().info(f'gripper {label} → {angle:.3f} rad')

    # ------------------------------------------------------------------
    # Step-hold animation
    # ------------------------------------------------------------------

    def _step_hold_yz(self):
        """Returns (y_offset, z_offset) for current phase.

        Cycle:
          hold_left      y=+amp_y, z=0          (hold_secs)
          ramp_to_center y: +amp_y→0, z=0       (ramp_secs)
          hold_center    y=0, z=0               (center_hold_secs)
          arc_right_out  y: 0→-amp_y            (arc_secs)  z arcs up+down
          arc_right_in   y: -amp_y→+amp_y       (arc_secs)  z arcs up+down
          → back to hold_left
        """
        self._phase_t += self._dt

        if self._phase == 'hold_left':
            if self._phase_t >= self.hold_secs:
                self._next_phase('ramp_to_center')
            return self.amp_y, 0.0

        elif self._phase == 'ramp_to_center':
            y = _coslerp(self.amp_y, 0.0, self._phase_t, self.ramp_secs)
            if self._phase_t >= self.ramp_secs:
                self._next_phase('hold_center')
            return y, 0.0

        elif self._phase == 'hold_center':
            if self._phase_t >= self.center_hold_secs:
                self._next_phase('arc_right_out')
            return 0.0, 0.0

        elif self._phase == 'arc_right_out':
            y = _coslerp(0.0, -self.amp_y, self._phase_t, self.arc_secs)
            progress = min(self._phase_t / self.arc_secs, 1.0)
            z = self.amp_z * math.sin(math.pi * progress)
            if self._phase_t >= self.arc_secs:
                self._next_phase('arc_right_in')
            return y, z

        else:  # arc_right_in
            y = _coslerp(-self.amp_y, self.amp_y, self._phase_t, self.arc_secs)
            progress = min(self._phase_t / self.arc_secs, 1.0)
            z = self.amp_z * math.sin(math.pi * progress)
            if self._phase_t >= self.arc_secs:
                self._next_phase('hold_left')
            return y, z

    def _next_phase(self, phase: str):
        self._phase = phase
        self._phase_t = 0.0
        self.get_logger().info(f'→ {phase}')

    # ------------------------------------------------------------------
    # Main publish tick
    # ------------------------------------------------------------------

    def tick(self):
        stamp = self.get_clock().now().to_msg()

        if self.keyboard:
            with self._kb_lock:
                x = self._kb_x
                y = self._kb_y
                z = self._kb_z
                qx, qy, qz, qw = _euler_to_quat(
                    self._kb_roll, self._kb_pitch, self._kb_yaw)
        elif self.step_hold:
            dy, dz = self._step_hold_yz()
            x = self.x
            y = self.y + dy
            z = self.z + dz
            qx, qy, qz, qw = self.qx, self.qy, self.qz, self.qw
        elif self.animate:
            self._t += self._dt
            x = self.x
            y = self.y + self.amp_y * math.sin(2 * math.pi * self.freq * self._t)
            z = self.z + self.amp_z * math.sin(2 * math.pi * self.freq * self._t * 0.7)
            qx, qy, qz, qw = self.qx, self.qy, self.qz, self.qw
        else:
            x, y, z = self.x, self.y, self.z
            qx, qy, qz, qw = self.qx, self.qy, self.qz, self.qw

        pose = PoseStamped()
        pose.header.stamp = stamp
        pose.header.frame_id = self.frame_id
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z
        pose.pose.orientation.x = qx
        pose.pose.orientation.y = qy
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw
        self.pose_pub.publish(pose)

        tf = TransformStamped()
        tf.header.stamp = stamp
        tf.header.frame_id = self.frame_id
        tf.child_frame_id = self.child_frame_id
        tf.transform.translation.x = x
        tf.transform.translation.y = y
        tf.transform.translation.z = z
        tf.transform.rotation.x = qx
        tf.transform.rotation.y = qy
        tf.transform.rotation.z = qz
        tf.transform.rotation.w = qw
        self.tf_broadcaster.sendTransform(tf)

    def destroy_node(self):
        if self._kb_reader is not None:
            self._kb_reader.stop()
        super().destroy_node()


def main():
    rclpy.init()
    node = FakeWristTarget()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
