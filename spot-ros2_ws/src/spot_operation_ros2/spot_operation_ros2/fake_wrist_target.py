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
"""

import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, TransformStamped
from tf2_ros import TransformBroadcaster


def _coslerp(a: float, b: float, t: float, duration: float) -> float:
    """Cosine interpolation from a to b. Zero velocity at both ends."""
    alpha = min(t / duration, 1.0)
    alpha = 0.5 * (1.0 - math.cos(math.pi * alpha))
    return a + (b - a) * alpha


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
        self.declare_parameter('ramp_secs', 2.0)        # left↔center transitions
        self.declare_parameter('arc_secs', 5.0)         # each arc leg (center→right, right→left)
        self.declare_parameter('hold_secs', 10.0)       # hold at left
        self.declare_parameter('center_hold_secs', 2.0) # hold at center

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

        self._t = 0.0
        self._dt = 1.0 / rate

        # Cycle: hold_left → ramp_to_center → hold_center → arc_right_out → arc_right_in → repeat
        self._phase = 'hold_left'
        self._phase_t = 0.0

        self.pose_pub = self.create_publisher(PoseStamped, '/wrist_pose', 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.timer = self.create_timer(1.0 / rate, self.tick)

        mode = 'step_hold' if self.step_hold else ('animate' if self.animate else 'static')
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
            # y: center → right, z: sine bump peaking at midpoint
            y = _coslerp(0.0, -self.amp_y, self._phase_t, self.arc_secs)
            progress = min(self._phase_t / self.arc_secs, 1.0)
            z = self.amp_z * math.sin(math.pi * progress)
            if self._phase_t >= self.arc_secs:
                self._next_phase('arc_right_in')
            return y, z

        else:  # arc_right_in: right → left, z: sine bump peaking at midpoint
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

    def tick(self):
        stamp = self.get_clock().now().to_msg()

        if self.step_hold:
            dy, dz = self._step_hold_yz()
            x = self.x
            y = self.y + dy
            z = self.z + dz
        elif self.animate:
            self._t += self._dt
            x = self.x
            y = self.y + self.amp_y * math.sin(2 * math.pi * self.freq * self._t)
            z = self.z + self.amp_z * math.sin(2 * math.pi * self.freq * self._t * 0.7)
        else:
            x, y, z = self.x, self.y, self.z

        pose = PoseStamped()
        pose.header.stamp = stamp
        pose.header.frame_id = self.frame_id
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z
        pose.pose.orientation.x = self.qx
        pose.pose.orientation.y = self.qy
        pose.pose.orientation.z = self.qz
        pose.pose.orientation.w = self.qw
        self.pose_pub.publish(pose)

        tf = TransformStamped()
        tf.header.stamp = stamp
        tf.header.frame_id = self.frame_id
        tf.child_frame_id = self.child_frame_id
        tf.transform.translation.x = x
        tf.transform.translation.y = y
        tf.transform.translation.z = z
        tf.transform.rotation.x = self.qx
        tf.transform.rotation.y = self.qy
        tf.transform.rotation.z = self.qz
        tf.transform.rotation.w = self.qw
        self.tf_broadcaster.sendTransform(tf)


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
