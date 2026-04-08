#!/usr/bin/env python3
import math
import re

import rclpy
from geometry_msgs.msg import PointStamped, PoseStamped, TransformStamped
from rclpy.duration import Duration
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo
from std_msgs.msg import String
from tf2_ros import Buffer, TransformBroadcaster, TransformListener


class TFProjectionNode(Node):
    def __init__(self):
        super().__init__("tf_projection_node")
        self.declare_parameter("tracking_3d_topic", "/tracking_3d_point")
        self.declare_parameter("tracking_state_topic", "/tracking_state")
        self.declare_parameter("target_pose_topic", "/target_pose")
        self.declare_parameter("target_frame_name", "target_object")
        self.declare_parameter("target_parent_frame", "odom")
        self.declare_parameter("hold_last_target_on_lost", True)
        self.declare_parameter("lost_hold_publish_hz", 10.0)
        self.declare_parameter("max_target_abs_m", 50.0)
        self.declare_parameter("tf_future_tolerance_sec", 0.35)
        self.declare_parameter("tf_past_tolerance_sec", 0.2)
        self.declare_parameter("tf_lookup_timeout_sec", 1.0)
        self.declare_parameter("tf_buffer_cache_time_sec", 120.0)
        self.declare_parameter("secondary_cameras", "")

        tracking_3d_topic = str(self.get_parameter("tracking_3d_topic").value)
        tracking_state_topic = str(self.get_parameter("tracking_state_topic").value)
        tracking_point_topic = tracking_3d_topic if tracking_3d_topic else "/tracking_3d_point"
        target_pose_topic = str(self.get_parameter("target_pose_topic").value)
        self.target_frame_name = str(self.get_parameter("target_frame_name").value)
        self.target_parent_frame = str(self.get_parameter("target_parent_frame").value)
        self.hold_last_target_on_lost = bool(self.get_parameter("hold_last_target_on_lost").value)
        hold_hz = float(max(0.5, self.get_parameter("lost_hold_publish_hz").value))
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

        self.tf_buffer = Buffer(cache_time=Duration(seconds=tf_cache))
        self.tf_listener = TransformListener(self.tf_buffer, self, spin_thread=True)
        self.tf_broadcaster = TransformBroadcaster(self)

        self._tracking_state = "UNKNOWN"
        self._last_valid_target = None  # (x, y, z)

        self._pose_pub = self.create_publisher(PoseStamped, target_pose_topic, 10)
        self._track_sub = self.create_subscription(
            PointStamped, tracking_point_topic, self._tracking_3d_cb, 10
        )
        self._state_sub = self.create_subscription(
            String, tracking_state_topic, self._tracking_state_cb, 10
        )
        self._hold_timer = self.create_timer(1.0 / hold_hz, self._hold_last_target_cb)

        # Seed reprojection: 3D@T0 → pixel@T_now
        self._seed_3d_sub = self.create_subscription(
            PointStamped, "/tracking/seed_3d", self._seed_3d_cb, 10
        )
        self._seed_pixel_pub = self.create_publisher(
            PointStamped, "/tracking/seed_pixel", 10
        )
        self._cam_info_sub = self.create_subscription(
            CameraInfo, "/hand/camera_info", self._camera_info_cb, 10
        )
        self.camera_intrinsics = None

        # Secondary cameras: reprojection of tracking point for FOV-gated seeding
        secondary_cameras_str = str(self.get_parameter("secondary_cameras").value)
        self._secondary_cameras = [c.strip() for c in secondary_cameras_str.split(',') if c.strip()]
        self._secondary_intrinsics = {}   # cam → (fx, fy, cx, cy)
        self._secondary_image_size = {}   # cam → (w, h)
        self._secondary_seed_pixel_pubs = {}
        for cam in self._secondary_cameras:
            self._secondary_intrinsics[cam] = None
            self._secondary_image_size[cam] = None
            self._secondary_seed_pixel_pubs[cam] = self.create_publisher(
                PointStamped, f"/{cam}/tracking/seed_pixel", 10
            )
            self.create_subscription(
                CameraInfo, f"/{cam}/camera_info",
                lambda msg, c=cam: self._secondary_camera_info_cb(msg, c), 10
            )
        if self._secondary_cameras:
            self.get_logger().info(f"Secondary cameras for reprojection: {self._secondary_cameras}")

        self.get_logger().info(
            f"TF projection ready. tracking_3d={tracking_point_topic}, "
            f"tracking_state={tracking_state_topic}, "
            f"target={self.target_parent_frame}->{self.target_frame_name}"
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

    def _hold_last_target_cb(self):
        if not self.hold_last_target_on_lost:
            return
        if self._tracking_state != "LOST":
            return
        if self._last_valid_target is None:
            return
        now_stamp = self.get_clock().now().to_msg()
        x, y, z = self._last_valid_target
        self._publish_target(x, y, z, now_stamp)
        # Fallback: seed secondary cameras from last known odom position while hand is LOST.
        # force_reinit=False — keep existing SAM2 state, just refresh the FOV hint.
        # Reset on new VLM detection is handled by _seed_3d_cb (force_reinit=True).
        if self._secondary_cameras:
            timeout = Duration(seconds=self.tf_lookup_timeout_sec)
            self._reproject_to_secondary(x, y, z, timeout, force_reinit=False)
            self.get_logger().debug(
                f"[HOLD] Seeding secondary cameras from last known odom ({x:.3f},{y:.3f},{z:.3f})",
            )

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
            margin = 10
            if not (margin <= u < w - margin and margin <= v < h - margin):
                continue  # outside FOV (with margin) — do not publish; SAM2 stops tracking
            pixel_msg = PointStamped()
            pixel_msg.header.stamp = self.get_clock().now().to_msg()
            pixel_msg.header.frame_id = tf_frame
            pixel_msg.point.x = float(u)
            pixel_msg.point.y = float(v)
            pixel_msg.point.z = 1.0 if force_reinit else 0.0  # 1.0 = force SAM2 reinit
            self._secondary_seed_pixel_pubs[cam].publish(pixel_msg)

    def _camera_info_cb(self, msg: CameraInfo):
        if self.camera_intrinsics is not None:
            return
        self.camera_intrinsics = (float(msg.k[0]), float(msg.k[4]), float(msg.k[2]), float(msg.k[5]))
        self.get_logger().info(f"Camera intrinsics set: {self.camera_intrinsics}")

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

        # 1) hand_cam→odom @T0
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
            f"[TF] step1 cam→odom: odom_pt=({odom_x:.3f},{odom_y:.3f},{odom_z:.3f}) tf_t=({tx:.3f},{ty:.3f},{tz:.3f})"
        )

        # 2) odom→hand_cam @T_now (latest available)
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
        tx2, ty2, tz2 = t_now.transform.translation.x, t_now.transform.translation.y, t_now.transform.translation.z
        qx2, qy2, qz2, qw2 = t_now.transform.rotation.x, t_now.transform.rotation.y, t_now.transform.rotation.z, t_now.transform.rotation.w
        cx, cy, cz = self._rotate_point_by_quaternion(odom_x, odom_y, odom_z, qx2, qy2, qz2, qw2)
        cam_x, cam_y, cam_z = cx + tx2, cy + ty2, cz + tz2
        self.get_logger().info(
            f"[TF] step2 odom→cam: cam_pt=({cam_x:.3f},{cam_y:.3f},{cam_z:.3f}) tf_t=({tx2:.3f},{ty2:.3f},{tz2:.3f}) t_now={t_now_stamp_sec:.3f}"
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

        # 4) Publish reprojected pixel
        pixel_msg = PointStamped()
        pixel_msg.header.stamp = self.get_clock().now().to_msg()
        pixel_msg.header.frame_id = source_frame
        pixel_msg.point.x = float(u)
        pixel_msg.point.y = float(v)
        pixel_msg.point.z = 0.0
        self._seed_pixel_pub.publish(pixel_msg)
        self.get_logger().info(f"[TF] Reprojected seed pixel: ({u:.0f}, {v:.0f})")

        # Also reproject the odom-frame seed to secondary cameras (1-shot on re-seed, force reinit)
        if self._secondary_cameras:
            self._reproject_to_secondary(odom_x, odom_y, odom_z, timeout, force_reinit=True)

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
        self._publish_target(float(target_x), float(target_y), float(target_z), msg.header.stamp)

        # Continuously reproject to secondary cameras (FOV-gated; no publish = SAM2 stops)
        if self._secondary_cameras:
            timeout = Duration(seconds=self.tf_lookup_timeout_sec)
            self._reproject_to_secondary(float(target_x), float(target_y), float(target_z), timeout)


def main(args=None):
    rclpy.init(args=args)
    node = TFProjectionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
