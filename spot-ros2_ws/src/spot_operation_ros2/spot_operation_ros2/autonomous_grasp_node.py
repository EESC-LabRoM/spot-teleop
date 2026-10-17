#!/usr/bin/env python3

"""
Autonomous Grasp Node — SDK-DIRECT (no driver manipulation action).

The spot_ros2 driver's manipulation action churns/contends the lease and fails
with LeaseUseError. Talking to the Bosdyn SDK directly (own lease) works — this is
the old `continuous_direct_grasp.py` approach. So this node holds its own lease and
sends Spot's native PickObject grasp via the SDK.

Architecture this fits:
  * spot_ros2 driver runs for CAMERAS / state / TF only (read-only; with
    auto_claim:=false + start_estop:=false it never grabs the lease).
  * perception_minimal publishes the `target_object` TF (hand camera VLM/SAM2).
  * THIS node holds the lease and grasps. (Arm-following, if used, must also be
    SDK-direct in the same lease owner — do not route arm_pose_commands through the
    driver, or it will grab the lease and conflict.)

Trigger: /gripper/goal (close=0.0 -> grasp, open=-1.57 -> release+re-arm) or the
/autonomous_grasp service. hand_pose_estimator already publishes /gripper/goal on each
fist/palm transition, so no separate gesture subscription is needed.

The object 3D point is read from the `target_object` TF (in `grasp_frame`, default
`vision`) and sent as PickObject(frame_name=grasp_frame, object_rt_frame=xyz). Spot
plans+executes the grasp onboard (approach + gripper).
"""

import math
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from std_srvs.srv import Trigger
from std_msgs.msg import Float64
from geometry_msgs.msg import PoseStamped
import tf2_ros

import bosdyn.client
import bosdyn.client.util
from bosdyn.client.lease import LeaseClient, LeaseKeepAlive
from bosdyn.client.manipulation_api_client import ManipulationApiClient
from bosdyn.client.robot_command import RobotCommandClient, RobotCommandBuilder, blocking_stand
from bosdyn.client.robot_state import RobotStateClient
from bosdyn.client.frame_helpers import get_a_tform_b
from bosdyn.api import geometry_pb2, manipulation_api_pb2


class AutonomousGraspNode(Node):
    """SDK-direct native PickObject grasp at the tracked target_object point."""

    def __init__(self):
        super().__init__('autonomous_grasp_node')
        self.cb_group = ReentrantCallbackGroup()

        # --- Spot connection params (default to the workspace spot_config.yaml) ---
        self.declare_parameter('hostname', '10.0.0.3')
        self.declare_parameter('username', 'admin')
        self.declare_parameter('password', 'spotadmin2017')
        # Power on + stand on startup so the robot is grasp-ready.
        self.declare_parameter('power_on', True)
        self.declare_parameter('stand', True)

        # --- Grasp params ---
        self.declare_parameter('target_frame', 'target_object')
        self.declare_parameter('grasp_frame', 'vision')   # parent/world frame of target_object
        self.declare_parameter('average_samples', 5)
        self.declare_parameter('sample_period_sec', 0.05)
        self.declare_parameter('tf_timeout_sec', 2.0)
        self.declare_parameter('grasp_timeout_sec', 30.0)
        # Constrain the grasp approach instead of letting Spot pick (which tends top-down):
        #   'frontal'  -> horizontal grasp (gripper Y aligned with world +Z)
        #   'top_down' -> top-down grasp   (gripper X aligned with world -Z)
        #   'auto'     -> no constraint (Spot decides)
        self.declare_parameter('grasp_orientation', 'frontal')
        self.declare_parameter('grasp_tolerance_deg', 15.0)

        # --- Wrist-follow arm control (SDK arm_pose_command), shares this node's lease ---
        # The EE follows /wrist_pose before AND after the grasp; it is paused while a grasp
        # runs (the manipulation preempts it). This replaces the driver-based
        # wrist_arm_follower, which would grab the driver's lease and conflict.
        self.declare_parameter('follow', True)
        self.declare_parameter('wrist_topic', '/wrist_pose')
        self.declare_parameter('follow_rate_hz', 10.0)
        self.declare_parameter('arm_cmd_seconds', 0.5)   # trajectory duration per pose
        self.declare_parameter('wrist_stale_timeout_sec', 0.5)

        # --- Rotation-only lock on grasp (port of cuRobo's lock_xyz_on_grasp) ---
        # When true, a successful grasp FREEZES the EE position at the grasp point and only
        # the wrist's ORIENTATION is followed -> the operator can rotate the gripper in place
        # (valve turning) without translating it. The lock holds until RELEASE (open palm),
        # then full position control is restored.
        self.declare_parameter('lock_xyz_on_grasp', False)

        # --- /gripper/goal is the SINGLE input (from hand_pose_estimator or the
        # fake_wrist_target keyboard). Convention: angle -1.57 = open, 0.0 = closed.
        #   close (fist)  -> trigger the grasp (PickObject closes the gripper itself)
        #   open  (palm)  -> open the gripper (release the object) + re-arm the trigger
        # The hand_gesture topic is intentionally NOT used: /gripper/goal already carries
        # both transitions, so a separate gesture subscription would be redundant.
        self.declare_parameter('gripper_topic', '/gripper/goal')
        self.declare_parameter('gripper_open_angle', -1.57)

        self.hostname = str(self.get_parameter('hostname').value)
        self.username = str(self.get_parameter('username').value)
        self.password = str(self.get_parameter('password').value)
        self.do_power_on = bool(self.get_parameter('power_on').value)
        self.do_stand = bool(self.get_parameter('stand').value)
        self.target_frame = str(self.get_parameter('target_frame').value)
        self.grasp_frame = str(self.get_parameter('grasp_frame').value)
        self.average_samples = int(self.get_parameter('average_samples').value)
        self.sample_period = float(self.get_parameter('sample_period_sec').value)
        self.tf_timeout = float(self.get_parameter('tf_timeout_sec').value)
        self.grasp_timeout = float(self.get_parameter('grasp_timeout_sec').value)
        self.grasp_orientation = str(self.get_parameter('grasp_orientation').value).lower()
        self.grasp_tolerance_deg = float(self.get_parameter('grasp_tolerance_deg').value)
        self.follow = bool(self.get_parameter('follow').value)
        self.wrist_topic = str(self.get_parameter('wrist_topic').value)
        self.follow_rate = float(self.get_parameter('follow_rate_hz').value)
        self.arm_cmd_seconds = float(self.get_parameter('arm_cmd_seconds').value)
        self.wrist_stale_timeout = float(self.get_parameter('wrist_stale_timeout_sec').value)
        self.lock_xyz_on_grasp = bool(self.get_parameter('lock_xyz_on_grasp').value)
        self.gripper_topic = str(self.get_parameter('gripper_topic').value)
        self.gripper_open_angle = float(self.get_parameter('gripper_open_angle').value)

        # --- TF (read target_object) ---
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # --- Bosdyn SDK: own lease (proven to work where the driver action did not) ---
        self._busy = False
        self._armed = True  # re-armed on open; a close fires the grasp once
        self._connect_spot()

        # --- Wrist-follow arm control + gripper/trigger ---
        self._latest_wrist = None
        self._latest_wrist_time = 0.0
        self._gripper_fraction = 1.0  # 1=open, 0=closed; open/ready before a grasp
        self._locked = False          # rotation-only lock (grasp -> release)
        self._lock_pos = None         # frozen EE position while locked
        if self.follow:
            self.create_subscription(PoseStamped, self.wrist_topic, self._wrist_cb, 10,
                                     callback_group=self.cb_group)
            self.create_timer(1.0 / self.follow_rate, self._follow_tick,
                              callback_group=self.cb_group)
        # /gripper/goal: close -> grasp, open -> release + re-arm (single input).
        self.create_subscription(Float64, self.gripper_topic, self._gripper_cb, 10,
                                 callback_group=self.cb_group)

        # Manual fallback trigger.
        self.create_service(Trigger, 'autonomous_grasp', self.grasp_callback,
                            callback_group=self.cb_group)

        self.get_logger().info('=' * 60)
        self.get_logger().info('Autonomous Grasp Node (SDK-direct) ready.')
        self.get_logger().info(f'  Spot: {self.username}@{self.hostname}')
        self.get_logger().info(f'  Object frame: {self.target_frame} in {self.grasp_frame}')
        self.get_logger().info(
            f'  Grasp orientation: {self.grasp_orientation} (tol {self.grasp_tolerance_deg:.0f} deg)')
        self.get_logger().info(
            f'  Trigger: {self.gripper_topic} close -> grasp, open -> release+re-arm '
            '(or /autonomous_grasp service)')
        self.get_logger().info(
            f"  Wrist-follow: {self.follow}"
            + (f" ({self.wrist_topic} -> arm @ {self.follow_rate:.0f} Hz)" if self.follow else ""))
        self.get_logger().info(
            f"  Rotation-only lock on grasp: {self.lock_xyz_on_grasp} (locks grasp->release)")
        self.get_logger().info('=' * 60)

    # ------------------------------------------------------------------ SDK setup
    def _connect_spot(self):
        """Connect, take the lease (held for the node lifetime), power on + stand."""
        sdk = bosdyn.client.create_standard_sdk('AutonomousGraspNode')
        self.robot = sdk.create_robot(self.hostname)
        self.robot.authenticate(self.username, self.password)
        self.robot.time_sync.wait_for_sync()
        if not self.robot.has_arm():
            raise RuntimeError('Spot has no arm')

        self.lease_client = self.robot.ensure_client(LeaseClient.default_service_name)
        self.command_client = self.robot.ensure_client(RobotCommandClient.default_service_name)
        self.manip_client = self.robot.ensure_client(ManipulationApiClient.default_service_name)
        self.state_client = self.robot.ensure_client(RobotStateClient.default_service_name)

        self.get_logger().info('Taking lease (force take) and starting keepalive...')
        self.lease_client.take()
        # Held for the whole node lifetime. return_at_exit=False: do NOT return the lease
        # when the node stops, so Spot is not left ownerless after a grasp (which would
        # power it off and drop the object). The robot stays powered/standing/holding "as
        # is"; the next run force-takes the lease again.
        self._lease_keepalive = LeaseKeepAlive(self.lease_client, must_acquire=True,
                                               return_at_exit=False)
        if self.do_power_on:
            self.get_logger().info('Powering on...')
            self.robot.power_on(timeout_sec=20)
        if self.do_stand:
            self.get_logger().info('Standing...')
            blocking_stand(self.command_client, timeout_sec=10)

    # ------------------------------------------------------------------ wrist follow
    def _wrist_cb(self, msg: PoseStamped):
        self._latest_wrist = msg
        self._latest_wrist_time = self.get_clock().now().nanoseconds * 1e-9

    def _gripper_cb(self, msg: Float64):
        """Single input. /gripper/goal angle: -1.57 = open, 0.0 = closed.

          close (fist) -> TRIGGER the grasp on the rising edge (PickObject closes the
                          gripper itself; we don't command the gripper closed here).
          open  (palm) -> open the gripper (release) + re-arm the trigger.

        The open fraction is carried on the follow command so it doesn't preempt the arm
        (sent standalone if not following). Ignored entirely while a grasp runs.
        """
        if self._busy:
            return
        denom = abs(self.gripper_open_angle) or 1.0
        frac = max(0.0, min(1.0, -float(msg.data) / denom))
        if frac < 0.5:
            # CLOSE -> grasp trigger (once, until re-armed by an open).
            if self._armed:
                self._armed = False
                self.get_logger().info('gripper close -> triggering grasp')
                threading.Thread(target=self._run_grasp_logged, args=('gripper_close',),
                                 daemon=True).start()
            return
        # OPEN -> release + re-arm + unlock (restore full position control).
        self._armed = True
        self._gripper_fraction = frac
        if self._locked:
            self._locked = False
            self._lock_pos = None
            self.get_logger().info('XYZ UNLOCKED (released) - full position control restored')
        if not self.follow:
            try:
                self.command_client.robot_command(
                    RobotCommandBuilder.claw_gripper_open_fraction_command(self._gripper_fraction))
            except Exception as e:  # noqa: BLE001
                self.get_logger().warn(f'gripper command failed: {e}', throttle_duration_sec=2.0)

    def _follow_tick(self):
        """Stream the wrist target to the arm via SDK arm_pose_command.

        Runs before and after the grasp. Skipped while a grasp is in progress
        (_busy) so the manipulation cleanly owns the arm; resumes automatically
        afterward (now carrying the grasped object).
        """
        if self._busy or self._latest_wrist is None:
            return
        age = self.get_clock().now().nanoseconds * 1e-9 - self._latest_wrist_time
        if age > self.wrist_stale_timeout:
            return  # hold last pose; don't chase a stale target
        w = self._latest_wrist
        p, o = w.pose.position, w.pose.orientation
        frame = w.header.frame_id or 'body'

        # Rotation-only lock: while locked (grasp -> release), freeze the EE position at
        # the grasp point and follow only the wrist's orientation. Position is captured on
        # the first locked tick (EE is at the grasp point then).
        if self._locked:
            if self._lock_pos is None:
                self._lock_pos = self._current_ee_pos(frame)
                if self._lock_pos is not None:
                    self.get_logger().info(
                        f'XYZ LOCKED at ({self._lock_pos[0]:.3f},{self._lock_pos[1]:.3f},'
                        f'{self._lock_pos[2]:.3f}) - rotation only until release')
            px, py, pz = self._lock_pos if self._lock_pos is not None else (p.x, p.y, p.z)
        else:
            px, py, pz = p.x, p.y, p.z

        try:
            cmd = RobotCommandBuilder.arm_pose_command(
                px, py, pz, o.w, o.x, o.y, o.z, frame, self.arm_cmd_seconds)
            # Combine gripper into the same synchro command so neither preempts the other.
            cmd = RobotCommandBuilder.claw_gripper_open_fraction_command(
                self._gripper_fraction, build_on_command=cmd)
            self.command_client.robot_command(cmd)
        except Exception as e:  # noqa: BLE001
            self.get_logger().warn(f'arm-follow command failed: {e}',
                                   throttle_duration_sec=2.0)

    def _current_ee_pos(self, frame):
        """EE (hand) position in `frame` from the robot state, or None."""
        try:
            snap = self.state_client.get_robot_state().kinematic_state.transforms_snapshot
            t = get_a_tform_b(snap, frame, 'hand')
            if t is None:
                return None
            return (t.x, t.y, t.z)
        except Exception as e:  # noqa: BLE001
            self.get_logger().warn(f'EE pose lookup failed: {e}', throttle_duration_sec=2.0)
            return None

    # ------------------------------------------------------------------ helpers
    def get_target_point(self):
        """Average grasp_frame -> target_object translation. Returns (x,y,z) or None."""
        samples = []
        for _ in range(max(1, self.average_samples)):
            try:
                tf = self.tf_buffer.lookup_transform(
                    self.grasp_frame, self.target_frame, rclpy.time.Time(),
                    timeout=rclpy.duration.Duration(seconds=self.tf_timeout))
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                    tf2_ros.ExtrapolationException, tf2_ros.TransformException) as e:
                self.get_logger().warn(
                    f'TF {self.grasp_frame} -> {self.target_frame} failed: {e}')
                if not samples:
                    return None
                break
            t = tf.transform.translation
            samples.append((t.x, t.y, t.z))
            time.sleep(self.sample_period)
        n = len(samples)
        return (sum(s[0] for s in samples) / n,
                sum(s[1] for s in samples) / n,
                sum(s[2] for s in samples) / n)

    def _apply_grasp_constraint(self, pick):
        """Constrain the grasp approach (default frontal) instead of Spot's top-down pick.

        Aligns a gripper axis with a world axis (expressed in grasp_frame, which is
        gravity-aligned so +Z is up), via a VectorAlignmentWithTolerance constraint.
        """
        if self.grasp_orientation == 'auto':
            return
        if self.grasp_orientation == 'top_down':
            gripper_axis = geometry_pb2.Vec3(x=1.0, y=0.0, z=0.0)   # gripper approach (X)
            world_axis = geometry_pb2.Vec3(x=0.0, y=0.0, z=-1.0)    # -> down
        else:  # 'frontal' / horizontal
            gripper_axis = geometry_pb2.Vec3(x=0.0, y=1.0, z=0.0)   # gripper Y
            world_axis = geometry_pb2.Vec3(x=0.0, y=0.0, z=1.0)     # -> world up (horizon-facing)
        pick.grasp_params.grasp_params_frame_name = self.grasp_frame
        c = pick.grasp_params.allowable_orientation.add()
        c.vector_alignment_with_tolerance.axis_on_gripper_ewrt_gripper.CopyFrom(gripper_axis)
        c.vector_alignment_with_tolerance.axis_to_align_with_ewrt_frame.CopyFrom(world_axis)
        c.vector_alignment_with_tolerance.threshold_radians = math.radians(self.grasp_tolerance_deg)

    def _do_grasp(self):
        """One grasp attempt via the SDK. Returns (success, message)."""
        if self._busy:
            return False, 'Grasp already in progress.'
        self._busy = True
        try:
            point = self.get_target_point()
            if point is None:
                return False, (f"No '{self.target_frame}' TF in '{self.grasp_frame}'. "
                               "Is perception detecting the object?")
            self.get_logger().info(
                f'Grasp target in {self.grasp_frame}: '
                f'[{point[0]:.3f}, {point[1]:.3f}, {point[2]:.3f}]')

            pick = manipulation_api_pb2.PickObject(
                frame_name=self.grasp_frame,
                object_rt_frame=geometry_pb2.Vec3(x=float(point[0]),
                                                  y=float(point[1]),
                                                  z=float(point[2])))
            self._apply_grasp_constraint(pick)
            req = manipulation_api_pb2.ManipulationApiRequest(pick_object=pick)
            self.get_logger().info('Sending PickObject grasp to Spot (SDK)...')
            resp = self.manip_client.manipulation_api_command(manipulation_api_request=req)
            cmd_id = resp.manipulation_cmd_id

            # Poll feedback (Spot does approach + grip onboard).
            start = time.time()
            state = 'UNKNOWN'
            while time.time() - start < self.grasp_timeout:
                fb = self.manip_client.manipulation_api_feedback_command(
                    manipulation_api_feedback_request=manipulation_api_pb2.ManipulationApiFeedbackRequest(
                        manipulation_cmd_id=cmd_id))
                state = manipulation_api_pb2.ManipulationFeedbackState.Name(fb.current_state)
                if fb.current_state == manipulation_api_pb2.MANIP_STATE_GRASP_SUCCEEDED:
                    # Latch the gripper closed so the resumed follow keeps holding the object.
                    self._gripper_fraction = 0.0
                    # Rotation-only lock until release (next locked follow tick captures pos).
                    if self.lock_xyz_on_grasp:
                        self._locked = True
                        self._lock_pos = None
                    return True, 'Grasp succeeded'
                if fb.current_state in (manipulation_api_pb2.MANIP_STATE_GRASP_FAILED,
                                        manipulation_api_pb2.MANIP_STATE_GRASP_PLANNING_NO_SOLUTION):
                    return False, f'Grasp failed ({state})'
                time.sleep(0.25)
            return False, f'Grasp timed out (last state {state})'
        except Exception as e:  # noqa: BLE001 - surface any SDK error to the caller
            self.get_logger().error(f'Grasp exception: {e}')
            return False, f'Exception: {e}'
        finally:
            self._busy = False

    def _run_grasp_logged(self, source):
        ok, msg = self._do_grasp()
        (self.get_logger().info if ok else self.get_logger().error)(
            f'Grasp ({source}) result: success={ok} ({msg})')

    # ------------------------------------------------------------------ triggers
    def grasp_callback(self, request, response):
        self.get_logger().info('AUTONOMOUS GRASP TRIGGERED')
        ok, msg = self._do_grasp()
        response.success = ok
        response.message = msg
        (self.get_logger().info if ok else self.get_logger().error)(f'Grasp result: {msg}')
        return response

    def destroy_node(self):
        try:
            self._lease_keepalive.shutdown()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = AutonomousGraspNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
