#!/usr/bin/env python3
import math
import re
import time

import rclpy
from geometry_msgs.msg import PointStamped, PoseStamped, TransformStamped
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo
from std_msgs.msg import Float64, String
from tf2_ros import Buffer, TransformBroadcaster, TransformListener


class TFProjectionNode(Node):
    def __init__(self):
        super().__init__("tf_projection_node")
        self.declare_parameter("tracking_3d_topic", "/tracking_3d_point")
        self.declare_parameter("tracking_state_topic", "/tracking_state")
        self.declare_parameter("target_pose_topic", "/target_pose")
        self.declare_parameter("target_frame_name", "target_object")
        self.declare_parameter("target_parent_frame", "vision")
        self.declare_parameter("hold_last_target_on_lost", True)
        self.declare_parameter("target_publish_hz", 30.0)
        self.declare_parameter("target_ema_alpha", 0.1)
        # Treat target_object as world-fixed: reject hand-tracking 3D readings that jump
        # more than this from the latched position (they come from partial/edge masks or
        # the depth fallback under camera motion — the object itself is static). A
        # SUSTAINED run of far readings (target_jump_relatch_count) is accepted as a real
        # move and re-latches. 0 disables the gate (pure EMA, old behaviour).
        self.declare_parameter("target_jump_reject_m", 0.15)
        self.declare_parameter("target_jump_relatch_count", 8)
        self.declare_parameter("max_target_abs_m", 50.0)
        self.declare_parameter("tf_future_tolerance_sec", 0.35)
        self.declare_parameter("tf_past_tolerance_sec", 0.2)
        self.declare_parameter("tf_lookup_timeout_sec", 1.0)
        self.declare_parameter("tf_buffer_cache_time_sec", 120.0)
        self.declare_parameter("secondary_cameras", "")
        self.declare_parameter("reloc_reference_tolerance_m", 0.4)  # max dist from first detection to accept re-seed
        self.declare_parameter("camera_info_topic", "/hand/camera_info")
        self.declare_parameter("secondary_camera_info_topic_pattern", "/{cam}/camera_info")
        self.declare_parameter("hand_camera_frame", "hand_color_image_sensor")
        self.declare_parameter("camera_speed_reference_frame", "vision")
        self.declare_parameter("camera_speed_topic", "/hand/camera_speed")
        self.declare_parameter("camera_speed_hz", 15.0)
        self.declare_parameter("secondary_reinit_speed_gate_m_s", 0.1)
        self.declare_parameter("seed_pixel_publish_hz", 10.0)
        self.declare_parameter("seed_pixel_active_ttl_sec", 10.0)

        tracking_3d_topic = str(self.get_parameter("tracking_3d_topic").value)
        tracking_state_topic = str(self.get_parameter("tracking_state_topic").value)
        tracking_point_topic = tracking_3d_topic if tracking_3d_topic else "/tracking_3d_point"
        target_pose_topic = str(self.get_parameter("target_pose_topic").value)
        self.target_frame_name = str(self.get_parameter("target_frame_name").value)
        self.target_parent_frame = str(self.get_parameter("target_parent_frame").value)
        self.hold_last_target_on_lost = bool(self.get_parameter("hold_last_target_on_lost").value)
        self.target_publish_hz = float(max(0.5, self.get_parameter("target_publish_hz").value))
        self.target_ema_alpha = float(self.get_parameter("target_ema_alpha").value)
        self.target_jump_reject_m = float(self.get_parameter("target_jump_reject_m").value)
        self.target_jump_relatch_count = int(self.get_parameter("target_jump_relatch_count").value)
        self._jump_count = 0  # consecutive far readings (for re-latch on a real move)
        self.max_target_abs_m = float(max(1.0, self.get_parameter("max_target_abs_m").value))
        self.tf_future_tolerance_sec = float(
            max(0.0, self.get_parameter("tf_future_tolerance_sec").value)
        )
        self.tf_past_tolerance_sec = float(
            max(0.0, self.get_parameter("tf_past_tolerance_sec").value)
        )
        self.tf_lookup_timeout_sec = float(
            max(0.01, self.get_parameter("tf_lookup_timeout_sec").value)
        )
        tf_cache = float(max(1.0, self.get_parameter("tf_buffer_cache_time_sec").value))
        camera_info_topic = str(self.get_parameter("camera_info_topic").value)
        secondary_camera_info_topic_pattern = str(
            self.get_parameter("secondary_camera_info_topic_pattern").value
        )

        self.tf_buffer = Buffer(cache_time=Duration(seconds=tf_cache))
        self.tf_listener = TransformListener(self.tf_buffer, self, spin_thread=False)
        self.tf_broadcaster = TransformBroadcaster(self)

        self._tracking_state = "UNKNOWN"
        self._last_valid_target = None  # (x, y, z) in target_parent_frame (vision/odom)
        self._ema_target = None
        self._last_vision_pt = None     # (x, y, z) in target_parent_frame — set on seed, used as depth fallback
        self._reference_vision_pt = None  # vision-frame position from FIRST successful detection — immutable reference
        self._latest_cam_speed = 0.0
        self._pending_force_reinit_pt = None  # (odom_x, odom_y, odom_z) deferred until camera stable

        self._pose_pub = self.create_publisher(PoseStamped, target_pose_topic, 10)
        # Geometry depth fallback: continuously republish the known vision-frame object position
        # reprojected into the hand camera frame.  The tracker uses this as depth fallback when
        # the depth sensor doesn't cover the edge of the RGB image.
        self._geometry_cam_pub = self.create_publisher(
            PointStamped, "/tracking/geometry_3d_in_cam", 10
        )
        self._geometry_cam_timer = self.create_timer(0.1, self._publish_geometry_cam_pt)
        self._track_sub = self.create_subscription(
            PointStamped, tracking_point_topic, self._tracking_3d_cb, 10
        )
        self._state_sub = self.create_subscription(
            String, tracking_state_topic, self._tracking_state_cb, 10
        )
        self._target_publish_timer = self.create_timer(1.0 / self.target_publish_hz, self._target_publish_cb)

        # Camera speed publisher (for VLM stability gate)
        self._hand_camera_frame = str(self.get_parameter("hand_camera_frame").value)
        self._cam_speed_ref_frame = str(self.get_parameter("camera_speed_reference_frame").value)
        self._cam_speed_pub = self.create_publisher(
            Float64, str(self.get_parameter("camera_speed_topic").value), 10
        )
        self._prev_cam_pos = None
        self._prev_cam_pos_time = None
        cam_speed_hz = float(max(1.0, self.get_parameter("camera_speed_hz").value))
        self._cam_speed_timer = self.create_timer(1.0 / cam_speed_hz, self._publish_cam_speed)
        self._secondary_reinit_speed_gate_m_s = float(self.get_parameter("secondary_reinit_speed_gate_m_s").value)

        # Seed reprojection: 3D@T0 → pixel@T_now
        self._seed_3d_sub = self.create_subscription(
            PointStamped, "/tracking/seed_3d", self._seed_3d_cb, 10
        )
        self._seed_pixel_pub = self.create_publisher(
            PointStamped, "/tracking/seed_pixel", 10
        )
        # Continuous pixel re-seed: while active, reproject _reference_vision_pt through
        # current TF and republish so the tracker always has a fresh pixel (the velocity
        # gate can hold init for several seconds while the camera moves, so a one-shot
        # pixel captured at _seed_3d_cb time ends up paralax-stale).
        self._seed_pixel_active_until = 0.0  # monotonic clock seconds
        self._seed_pixel_ttl_sec = float(
            max(0.5, self.get_parameter("seed_pixel_active_ttl_sec").value)
        )
        seed_pixel_hz = float(max(1.0, self.get_parameter("seed_pixel_publish_hz").value))
        self._seed_pixel_hand_frame = None  # hand camera frame_id (set on first _seed_3d_cb)
        self._seed_pixel_timer = self.create_timer(
            1.0 / seed_pixel_hz, self._publish_seed_pixel_continuous
        )
        self._cam_info_sub = self.create_subscription(
            CameraInfo, camera_info_topic, self._camera_info_cb, 10
        )
        self.camera_intrinsics = None
        self._hand_image_size = None  # (w, h) from CameraInfo

        # Secondary cameras: reprojection of tracking point for FOV-gated seeding
        secondary_cameras_str = str(self.get_parameter("secondary_cameras").value)
        self._secondary_cameras = [c.strip() for c in secondary_cameras_str.split(',') if c.strip()]
        self._secondary_intrinsics = {}   # cam → (fx, fy, cx, cy)
        self._secondary_image_size = {}   # cam → (w, h)
        self._secondary_seed_pixel_pubs = {}
        for cam in self._secondary_cameras:
            secondary_camera_info_topic = secondary_camera_info_topic_pattern.replace("{cam}", cam)
            self._secondary_intrinsics[cam] = None
            self._secondary_image_size[cam] = None
            self._secondary_seed_pixel_pubs[cam] = self.create_publisher(
                PointStamped, f"/{cam}/tracking/seed_pixel", 10
            )
            self.create_subscription(
                CameraInfo, secondary_camera_info_topic,
                lambda msg, c=cam: self._secondary_camera_info_cb(msg, c), 10
            )
        if self._secondary_cameras:
            # Continuous secondary reprojection at 10Hz from the stable vision-frame
            # position (_last_vision_pt).  This decouples secondary FOV detection from
            # the hand tracking cycle rate (~1.3s) and centroid jitter.
            self._secondary_reproject_timer = self.create_timer(
                0.1, self._continuous_secondary_reproject
            )
            self.get_logger().info(
                f"Secondary cameras for reprojection: {self._secondary_cameras}, camera_info_pattern={secondary_camera_info_topic_pattern}"
            )

        self.get_logger().info(
            f"TF projection ready. tracking_3d={tracking_point_topic}, "
            f"tracking_state={tracking_state_topic}, "
            f"target={self.target_parent_frame}->{self.target_frame_name}, "
            f"camera_info={camera_info_topic}"
        )

    def _tracking_state_cb(self, msg: String):
        self._tracking_state = str(msg.data).strip().upper()

    def _publish_target(self, x: float, y: float, z: float, stamp_msg):
        tf_msg = TransformStamped()
        tf_msg.header.stamp = stamp_msg
        tf_msg.header.frame_id = self.target_parent_frame
        tf_msg.child_frame_id = self.target_frame_name
        tf_msg.transform.translation.x = float(x)
        tf_msg.transform.translation.y = float(y)
        tf_msg.transform.translation.z = float(z)
        tf_msg.transform.rotation.w = 1.0
        self.tf_broadcaster.sendTransform(tf_msg)

        pose = PoseStamped()
        pose.header = tf_msg.header
        pose.pose.position.x = tf_msg.transform.translation.x
        pose.pose.position.y = tf_msg.transform.translation.y
        pose.pose.position.z = tf_msg.transform.translation.z
        pose.pose.orientation.w = 1.0
        self._pose_pub.publish(pose)

    def _target_publish_cb(self):
        if self._ema_target is None:
            return
        now_stamp = self.get_clock().now().to_msg()
        x, y, z = self._ema_target
        self._publish_target(x, y, z, now_stamp)
        # Note: secondary camera seeding is handled entirely by
        # _continuous_secondary_reproject at 10Hz using the stable _last_vision_pt.

    def _continuous_secondary_reproject(self):
        """Reproject the stable vision-frame object position to secondary cameras at 10Hz.

        Unlike _tracking_3d_cb (which uses the jittery hand-tracking centroid at ~1.3s rate),
        this uses _last_vision_pt — the original VLM detection, which is fixed in the world.
        Runs regardless of tracking state so secondary cameras detect FOV entry promptly.
        Also fires any deferred force_reinit once the camera has stabilised.
        """
        # Fire deferred force_reinit once camera has stabilised
        if self._pending_force_reinit_pt is not None:
            if self._latest_cam_speed <= self._secondary_reinit_speed_gate_m_s:
                x, y, z = self._pending_force_reinit_pt
                self._pending_force_reinit_pt = None
                self.get_logger().info(
                    f"[TF] Firing deferred secondary force_reinit (speed={self._latest_cam_speed:.3f} m/s)"
                )
                self._reproject_to_secondary(x, y, z, Duration(seconds=0.0), force_reinit=True)
            # else: still moving — keep pending

        # Normal 10Hz update-only seeds from stable vision-frame position
        if self._last_vision_pt is None:
            return
        x, y, z = self._last_vision_pt
        self._reproject_to_secondary(x, y, z, Duration(seconds=0.0), force_reinit=False)

    def _lookup_transform_at_stamp(self, source_frame: str, stamp_msg) -> TransformStamped:
        st = rclpy.time.Time.from_msg(stamp_msg)
        timeout = Duration(seconds=self.tf_lookup_timeout_sec)
        try:
            return self.tf_buffer.lookup_transform(self.target_parent_frame, source_frame, st, timeout=timeout)
        except Exception as exc:
            txt = str(exc)
            # Future extrapolation: requested > latest
            m_future = re.search(r"Requested time ([\d.]+) but the latest data is at time ([\d.]+)", txt)
            if m_future:
                req = float(m_future.group(1))
                latest = float(m_future.group(2))
                if 0.0 <= (req - latest) <= self.tf_future_tolerance_sec:
                    sec = int(latest)
                    nsec = int((latest - sec) * 1e9)
                    return self.tf_buffer.lookup_transform(
                        self.target_parent_frame,
                        source_frame,
                        rclpy.time.Time(seconds=sec, nanoseconds=nsec),
                        timeout=timeout,
                    )
            # Past extrapolation: requested < earliest — snapshot taken before TF buffer had data
            m_past = re.search(r"Requested time ([\d.]+) but the earliest data is at time ([\d.]+)", txt)
            if m_past:
                req = float(m_past.group(1))
                earliest = float(m_past.group(2))
                if 0.0 <= (earliest - req) <= self.tf_past_tolerance_sec:
                    sec = int(earliest)
                    nsec = int((earliest - sec) * 1e9)
                    self.get_logger().warn(
                        f"TF past tolerance applied: using earliest={earliest:.3f} instead of requested={req:.3f} (diff={(earliest-req)*1000:.1f}ms)"
                    )
                    return self.tf_buffer.lookup_transform(
                        self.target_parent_frame,
                        source_frame,
                        rclpy.time.Time(seconds=sec, nanoseconds=nsec),
                        timeout=timeout,
                    )
            raise

    def _rotate_point_by_quaternion(self, px, py, pz, qx, qy, qz, qw):
        t0 = 2.0 * (qy * pz - qz * py)
        t1 = 2.0 * (qz * px - qx * pz)
        t2 = 2.0 * (qx * py - qy * px)
        rx = px + qw * t0 + (qy * t2 - qz * t1)
        ry = py + qw * t1 + (qz * t0 - qx * t2)
        rz = pz + qw * t2 + (qx * t1 - qy * t0)
        return rx, ry, rz

    def _secondary_camera_info_cb(self, msg: CameraInfo, cam: str):
        if self._secondary_intrinsics.get(cam) is not None:
            return
        self._secondary_intrinsics[cam] = (
            float(msg.k[0]), float(msg.k[4]), float(msg.k[2]), float(msg.k[5])
        )
        # Use the TF frame_id from the CameraInfo header, not the topic prefix
        tf_frame = str(msg.header.frame_id) if msg.header.frame_id else cam
        self._secondary_image_size[cam] = (int(msg.width), int(msg.height), tf_frame)
        self.get_logger().info(
            f"Camera intrinsics set for {cam}: {self._secondary_intrinsics[cam]} tf_frame={tf_frame}"
        )

    def _reproject_to_secondary(self, odom_x: float, odom_y: float, odom_z: float, timeout, force_reinit: bool = False):
        """Project an odom-frame point into each secondary camera; publish seed_pixel if in FOV.
        force_reinit=True: sent from one-shot VLM re-seed → secondary SAM2 must reinitialize.
        force_reinit=False: sent from continuous tracking → only update hint, no reinit if already tracking.
        point.z encodes this flag (1.0 = reinit, 0.0 = update-only).
        """
        for cam in self._secondary_cameras:
            intrinsics = self._secondary_intrinsics.get(cam)
            size_info = self._secondary_image_size.get(cam)
            if intrinsics is None or size_info is None:
                continue
            w, h, tf_frame = size_info
            try:
                t = self.tf_buffer.lookup_transform(
                    tf_frame, self.target_parent_frame, rclpy.time.Time(), timeout=timeout
                )
            except Exception as exc:
                self.get_logger().warn(f"TF lookup for {cam} ({tf_frame}): {exc}", throttle_duration_sec=2.0)
                continue
            tx = t.transform.translation.x
            ty = t.transform.translation.y
            tz = t.transform.translation.z
            qx = t.transform.rotation.x
            qy = t.transform.rotation.y
            qz = t.transform.rotation.z
            qw = t.transform.rotation.w
            cx, cy, cz = self._rotate_point_by_quaternion(odom_x, odom_y, odom_z, qx, qy, qz, qw)
            cam_x, cam_y, cam_z = cx + tx, cy + ty, cz + tz
            if cam_z <= 0.0:
                continue  # behind the camera
            fx, fy, cx_i, cy_i = intrinsics
            u = fx * (cam_x / cam_z) + cx_i
            v = fy * (cam_y / cam_z) + cy_i
            # Basic bounds check only (no margin) — prevent sending negative or
            # out-of-image coords that would crash SAM2. Borderline/edge cases
            # are left to the video predictor's IoU/area validation.
            if not (0 <= u < w and 0 <= v < h):
                continue
            pixel_msg = PointStamped()
            pixel_msg.header.stamp = t.header.stamp
            pixel_msg.header.frame_id = tf_frame
            pixel_msg.point.x = float(u)
            pixel_msg.point.y = float(v)
            # point.z carries the object's expected depth (m) in the camera optical
            # frame (= reprojected ray range, cam_z > 0 here). Its SIGN is the reinit
            # flag: negative = force SAM2 reinit (VLM re-seed), positive = update-only.
            # The tracker uses |z| for its COLD-init depth-consistency gate.
            pixel_msg.point.z = -float(cam_z) if force_reinit else float(cam_z)
            self._secondary_seed_pixel_pubs[cam].publish(pixel_msg)

    def _camera_info_cb(self, msg: CameraInfo):
        if self.camera_intrinsics is not None:
            return
        self.camera_intrinsics = (float(msg.k[0]), float(msg.k[4]), float(msg.k[2]), float(msg.k[5]))
        self._hand_image_size = (int(msg.width), int(msg.height))
        self.get_logger().info(f"Camera intrinsics set: {self.camera_intrinsics} size={self._hand_image_size}")

    def _seed_3d_cb(self, msg: PointStamped):
        """Receive 3D point@T0, reproject to pixel in current camera frame, publish."""
        if self.camera_intrinsics is None:
            self.get_logger().warn("No camera intrinsics yet, cannot reproject seed")
            return

        source_frame = str(msg.header.frame_id)
        x, y, z = float(msg.point.x), float(msg.point.y), float(msg.point.z)
        t0_stamp_sec = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.get_logger().info(
            f"[TF] seed_3d received: cam_pt=({x:.3f},{y:.3f},{z:.3f}) stamp={t0_stamp_sec:.3f} frame={source_frame}"
        )

        # 1) hand_cam→target_parent @T0
        try:
            t0 = self._lookup_transform_at_stamp(source_frame, msg.header.stamp)
        except Exception as exc:
            self.get_logger().warn(f"TF lookup @T0 failed: {exc}")
            return

        tx, ty, tz = t0.transform.translation.x, t0.transform.translation.y, t0.transform.translation.z
        qx, qy, qz, qw = t0.transform.rotation.x, t0.transform.rotation.y, t0.transform.rotation.z, t0.transform.rotation.w
        px, py, pz = self._rotate_point_by_quaternion(x, y, z, qx, qy, qz, qw)
        odom_x, odom_y, odom_z = px + tx, py + ty, pz + tz
        self.get_logger().info(
            f"[TF] step1 cam→{self.target_parent_frame}: pt=({odom_x:.3f},{odom_y:.3f},{odom_z:.3f}) tf_t=({tx:.3f},{ty:.3f},{tz:.3f})"
        )
        # Validate against first-detection reference (object is stationary — large deviations = bad TF or wrong object)
        tol = float(self.get_parameter("reloc_reference_tolerance_m").value)
        if self._reference_vision_pt is None:
            self._reference_vision_pt = (odom_x, odom_y, odom_z)
            self.get_logger().info(
                f"[TF] Reference vision-frame point stored: ({odom_x:.3f},{odom_y:.3f},{odom_z:.3f})"
            )
            allow_secondary_reinit = True
        else:
            rx, ry, rz = self._reference_vision_pt
            dist = ((odom_x - rx) ** 2 + (odom_y - ry) ** 2 + (odom_z - rz) ** 2) ** 0.5
            if dist > tol:
                self.get_logger().warn(
                    f"[TF] Re-detection too far from reference ({dist:.2f}m > {tol:.2f}m) — "
                    f"likely bad TF or wrong object. Discarding seed entirely. "
                    f"new=({odom_x:.3f},{odom_y:.3f},{odom_z:.3f}) ref=({rx:.3f},{ry:.3f},{rz:.3f})"
                )
                return  # Don't update _last_vision_pt or publish seed pixel
            else:
                allow_secondary_reinit = True

        # Store vision-frame position for geometry depth fallback (object is stationary in world)
        self._last_vision_pt = (odom_x, odom_y, odom_z)
        # Store hand frame for continuous republisher (TTL activated only after
        # successful in-bounds reproject below — prevents the republisher from
        # producing edge-of-FOV garbage when the object has left the camera view).
        self._seed_pixel_hand_frame = source_frame

        # 2) target_parent→hand_cam @T_now (latest available)
        timeout = Duration(seconds=self.tf_lookup_timeout_sec)
        try:
            t_now = self.tf_buffer.lookup_transform(
                source_frame,
                self.target_parent_frame,
                rclpy.time.Time(),
                timeout=timeout,
            )
        except Exception as exc:
            self.get_logger().warn(f"TF lookup @T_now failed: {exc}")
            return

        t_now_stamp_sec = t_now.header.stamp.sec + t_now.header.stamp.nanosec * 1e-9
        dt = t_now_stamp_sec - t0_stamp_sec
        if abs(dt) > 8.0:
            self.get_logger().warn(
                f"[TF] T0→T_now interval too large ({dt:.1f}s, |dt| > 8.0s), discarding stale reprojection"
            )
            return
        tx2, ty2, tz2 = t_now.transform.translation.x, t_now.transform.translation.y, t_now.transform.translation.z
        qx2, qy2, qz2, qw2 = t_now.transform.rotation.x, t_now.transform.rotation.y, t_now.transform.rotation.z, t_now.transform.rotation.w
        cx, cy, cz = self._rotate_point_by_quaternion(odom_x, odom_y, odom_z, qx2, qy2, qz2, qw2)
        cam_x, cam_y, cam_z = cx + tx2, cy + ty2, cz + tz2
        self.get_logger().info(
            f"[TF] step2 {self.target_parent_frame}→cam: cam_pt=({cam_x:.3f},{cam_y:.3f},{cam_z:.3f}) tf_t=({tx2:.3f},{ty2:.3f},{tz2:.3f}) t_now={t_now_stamp_sec:.3f}"
        )

        if cam_z <= 0.0:
            self.get_logger().warn(f"Reprojected point behind camera (cam_z={cam_z:.3f})")
            return

        # 3) Project to pixel with intrinsics
        fx, fy, cx_i, cy_i = self.camera_intrinsics
        u = fx * (cam_x / cam_z) + cx_i
        v = fy * (cam_y / cam_z) + cy_i
        self.get_logger().info(
            f"[TF] step3 projection: pixel=({u:.1f},{v:.1f}) depth={cam_z:.3f}"
        )

        # 4) Publish reprojected pixel (only if within image bounds)
        margin = 20
        img_w, img_h = self._hand_image_size if self._hand_image_size else (640, 480)
        if not (margin <= u < img_w - margin and margin <= v < img_h - margin):
            self.get_logger().warn(
                f"[TF] Reprojected seed pixel ({u:.0f},{v:.0f}) out of bounds ({img_w}x{img_h}) — object left FOV"
            )
            return
        pixel_msg = PointStamped()
        pixel_msg.header.stamp = t_now.header.stamp
        pixel_msg.header.frame_id = source_frame
        pixel_msg.point.x = float(u)
        pixel_msg.point.y = float(v)
        pixel_msg.point.z = 0.0
        self._seed_pixel_pub.publish(pixel_msg)
        # Activate continuous republisher TTL only after a successful in-bounds publish.
        # If the initial reproject was out of bounds, the continuous republisher stays
        # deactivated — it would produce marginally-in-bounds but wrong pixels.
        self._seed_pixel_active_until = time.monotonic() + self._seed_pixel_ttl_sec
        self.get_logger().info(f"[TF] Reprojected seed pixel: ({u:.0f}, {v:.0f})")

        # Defer secondary force_reinit until camera is stable — robot may have moved during VLM processing
        if self._secondary_cameras and allow_secondary_reinit:
            self._pending_force_reinit_pt = (odom_x, odom_y, odom_z)
            self.get_logger().info(
                f"[TF] Secondary force_reinit pending until camera stable "
                f"(current speed={self._latest_cam_speed:.3f} m/s)"
            )
        # If allow_secondary_reinit=False: reference check failed; skip reinit.
        # _continuous_secondary_reproject handles 10Hz update-only seeds via _last_vision_pt.

    def _tracking_3d_cb(self, msg: PointStamped):
        if not msg.header.frame_id:
            self.get_logger().warn("tracking_3d point without frame_id", throttle_duration_sec=1.0)
            return

        source_frame = str(msg.header.frame_id)
        x = float(msg.point.x)
        y = float(msg.point.y)
        z = float(msg.point.z)
        if not all(math.isfinite(v) for v in (x, y, z)):
            self.get_logger().warn("tracking_3d point has non-finite values", throttle_duration_sec=1.0)
            return

        try:
            t = self._lookup_transform_at_stamp(source_frame, msg.header.stamp)
        except Exception as exc:
            self.get_logger().warn(f"tf_miss: {exc}", throttle_duration_sec=1.0)
            return

        tx = t.transform.translation.x
        ty = t.transform.translation.y
        tz = t.transform.translation.z
        qx = t.transform.rotation.x
        qy = t.transform.rotation.y
        qz = t.transform.rotation.z
        qw = t.transform.rotation.w
        px, py, pz = self._rotate_point_by_quaternion(x, y, z, qx, qy, qz, qw)
        target_x = px + tx
        target_y = py + ty
        target_z = pz + tz
        if not all(math.isfinite(v) for v in (target_x, target_y, target_z)):
            self.get_logger().warn("target point became non-finite after transform", throttle_duration_sec=1.0)
            return
        if any(abs(v) > self.max_target_abs_m for v in (target_x, target_y, target_z)):
            self.get_logger().warn("target point out of bounds; skipping publish", throttle_duration_sec=1.0)
            return

        self._last_valid_target = (float(target_x), float(target_y), float(target_z))

        if self._ema_target is None:
            self._ema_target = (float(target_x), float(target_y), float(target_z))
            self._jump_count = 0
        else:
            ex, ey, ez = self._ema_target
            # World-fixed gate: reject readings that jump far from the latched position
            # (partial/edge mask or depth-fallback noise under camera motion), unless a
            # sustained run of far readings indicates the object actually moved.
            if self.target_jump_reject_m > 0.0:
                dist = math.sqrt((target_x - ex) ** 2 + (target_y - ey) ** 2 + (target_z - ez) ** 2)
                if dist > self.target_jump_reject_m:
                    self._jump_count += 1
                    if self._jump_count < self.target_jump_relatch_count:
                        self.get_logger().info(
                            f"target jump {dist:.2f}m > {self.target_jump_reject_m:.2f}m rejected "
                            f"(holding world-fixed, {self._jump_count}/{self.target_jump_relatch_count})",
                            throttle_duration_sec=1.0,
                        )
                        return  # hold the latched position
                    # sustained move → re-latch to the new location
                    self.get_logger().info(
                        f"target moved {dist:.2f}m for {self._jump_count} readings — re-latching")
                    self._ema_target = (float(target_x), float(target_y), float(target_z))
                    self._jump_count = 0
                    return
                self._jump_count = 0
            alpha = self.target_ema_alpha
            self._ema_target = (
                ex + alpha * (float(target_x) - ex),
                ey + alpha * (float(target_y) - ey),
                ez + alpha * (float(target_z) - ez)
            )

        # Note: _publish_target is now handled continuously by _target_publish_cb.

        # Note: Secondary camera seeding is handled by _continuous_secondary_reproject
        # at 10Hz using the stable _last_vision_pt, not the jittery hand-tracking centroid.

    def _publish_geometry_cam_pt(self):
        """Reproject last known vision-frame object position into hand camera frame and publish.

        Used by the tracker as a depth fallback when the depth sensor doesn't cover the
        edge of the RGB image (depth resolution mismatch near FOV boundary).
        """
        if self._last_vision_pt is None:
            return
        ox, oy, oz = self._last_vision_pt
        try:
            t_now = self.tf_buffer.lookup_transform(
                self._hand_camera_frame,
                self.target_parent_frame,
                rclpy.time.Time(),
                timeout=Duration(seconds=0.0),
            )
        except Exception:
            return
        tx2 = t_now.transform.translation.x
        ty2 = t_now.transform.translation.y
        tz2 = t_now.transform.translation.z
        qx2 = t_now.transform.rotation.x
        qy2 = t_now.transform.rotation.y
        qz2 = t_now.transform.rotation.z
        qw2 = t_now.transform.rotation.w
        cx, cy, cz = self._rotate_point_by_quaternion(ox, oy, oz, qx2, qy2, qz2, qw2)
        cam_x, cam_y, cam_z = cx + tx2, cy + ty2, cz + tz2
        if cam_z <= 0.0:
            return
        msg = PointStamped()
        msg.header.stamp = t_now.header.stamp
        msg.header.frame_id = self._hand_camera_frame
        msg.point.x = float(cam_x)
        msg.point.y = float(cam_y)
        msg.point.z = float(cam_z)
        self._geometry_cam_pub.publish(msg)

    def _publish_seed_pixel_continuous(self):
        """Re-reproject the stable reference position to a pixel with current TF and republish.

        Active for TTL seconds after each successful _seed_3d_cb. This way when the
        tracker's velocity gate finally releases (which can take several seconds while
        the camera is moving), the pending seed pixel it uses was computed with the
        latest TF — not the one captured back in _seed_3d_cb.
        """
        if self._reference_vision_pt is None or self._seed_pixel_hand_frame is None:
            return
        if self.camera_intrinsics is None:
            return
        if time.monotonic() > self._seed_pixel_active_until:
            return
        ox, oy, oz = self._reference_vision_pt
        try:
            t_now = self.tf_buffer.lookup_transform(
                self._seed_pixel_hand_frame,
                self.target_parent_frame,
                rclpy.time.Time(),
                timeout=Duration(seconds=0.0),
            )
        except Exception:
            return
        tx = t_now.transform.translation.x
        ty = t_now.transform.translation.y
        tz = t_now.transform.translation.z
        qx = t_now.transform.rotation.x
        qy = t_now.transform.rotation.y
        qz = t_now.transform.rotation.z
        qw = t_now.transform.rotation.w
        cx, cy, cz = self._rotate_point_by_quaternion(ox, oy, oz, qx, qy, qz, qw)
        cam_x, cam_y, cam_z = cx + tx, cy + ty, cz + tz
        if cam_z <= 0.0:
            return
        fx, fy, cx_i, cy_i = self.camera_intrinsics
        u = fx * (cam_x / cam_z) + cx_i
        v = fy * (cam_y / cam_z) + cy_i
        if not (math.isfinite(u) and math.isfinite(v)):
            return
        # Reject out-of-bounds — object left the camera FOV
        img_w, img_h = self._hand_image_size if self._hand_image_size else (640, 480)
        margin = 20
        if not (margin <= u < img_w - margin and margin <= v < img_h - margin):
            return
        msg = PointStamped()
        msg.header.stamp = t_now.header.stamp
        msg.header.frame_id = self._seed_pixel_hand_frame
        msg.point.x = float(u)
        msg.point.y = float(v)
        msg.point.z = 0.0
        self._seed_pixel_pub.publish(msg)

    def _publish_cam_speed(self):
        """Compute linear speed of hand camera in 'vision' frame and publish to /hand/camera_speed."""
        try:
            t = self.tf_buffer.lookup_transform(
                self._cam_speed_ref_frame,
                self._hand_camera_frame,
                rclpy.time.Time(),
                timeout=Duration(seconds=0.0),
            )
        except Exception:
            return
        now = time.monotonic()
        x = t.transform.translation.x
        y = t.transform.translation.y
        z = t.transform.translation.z
        speed = 0.0
        if self._prev_cam_pos is not None and self._prev_cam_pos_time is not None:
            dt = now - self._prev_cam_pos_time
            if dt > 1e-4:
                dx = x - self._prev_cam_pos[0]
                dy = y - self._prev_cam_pos[1]
                dz = z - self._prev_cam_pos[2]
                speed = math.sqrt(dx * dx + dy * dy + dz * dz) / dt
        self._prev_cam_pos = (x, y, z)
        self._prev_cam_pos_time = now
        self._latest_cam_speed = speed
        msg = Float64()
        msg.data = float(speed)
        self._cam_speed_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = TFProjectionNode()
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
