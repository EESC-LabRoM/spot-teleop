#!/usr/bin/env python3
import json
import math
import re
from collections import deque
from typing import Optional

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped, TransformStamped
from rclpy.duration import Duration
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image as RosImage
from std_msgs.msg import String
from tf2_ros import Buffer, TransformBroadcaster, TransformListener

from .tracking_contracts import is_valid_tracking_2d_payload


class TFProjectionNode(Node):
    def __init__(self):
        super().__init__("tf_projection_node")
        self.declare_parameter("tracking_2d_topic", "/tracking_2d_result")
        self.declare_parameter("depth_topic", "/hand/depth")
        self.declare_parameter("depth_info_topic", "/hand/camera_info")
        self.declare_parameter("target_pose_topic", "/target_pose")
        self.declare_parameter("target_frame_name", "target_object")
        self.declare_parameter("target_parent_frame", "odom")
        self.declare_parameter("camera_frame_fallback", "hand_cam")
        self.declare_parameter("depth_cache_size", 180)
        self.declare_parameter("depth_match_tolerance_sec", 0.08)
        self.declare_parameter("tf_future_tolerance_sec", 0.35)
        self.declare_parameter("tf_lookup_timeout_sec", 1.0)
        self.declare_parameter("tf_buffer_cache_time_sec", 120.0)

        tracking_2d_topic = str(self.get_parameter("tracking_2d_topic").value)
        depth_topic = str(self.get_parameter("depth_topic").value)
        depth_info_topic = str(self.get_parameter("depth_info_topic").value)
        target_pose_topic = str(self.get_parameter("target_pose_topic").value)
        self.target_frame_name = str(self.get_parameter("target_frame_name").value)
        self.target_parent_frame = str(self.get_parameter("target_parent_frame").value)
        self.camera_frame_fallback = str(self.get_parameter("camera_frame_fallback").value)
        self.depth_cache_size = int(max(20, self.get_parameter("depth_cache_size").value))
        self.depth_match_tolerance_sec = float(
            max(0.0, self.get_parameter("depth_match_tolerance_sec").value)
        )
        self.tf_future_tolerance_sec = float(
            max(0.0, self.get_parameter("tf_future_tolerance_sec").value)
        )
        self.tf_lookup_timeout_sec = float(
            max(0.01, self.get_parameter("tf_lookup_timeout_sec").value)
        )
        tf_cache = float(max(1.0, self.get_parameter("tf_buffer_cache_time_sec").value))

        self.bridge = CvBridge()
        self.camera_intrinsics = None
        self.camera_frame_id = None
        self._depth_cache = deque(maxlen=self.depth_cache_size)

        self.tf_buffer = Buffer(cache_time=Duration(seconds=tf_cache))
        self.tf_listener = TransformListener(self.tf_buffer, self, spin_thread=True)
        self.tf_broadcaster = TransformBroadcaster(self)

        self._pose_pub = self.create_publisher(PoseStamped, target_pose_topic, 10)
        self._depth_sub = self.create_subscription(RosImage, depth_topic, self._depth_cb, 10)
        self._track_sub = self.create_subscription(String, tracking_2d_topic, self._tracking_2d_cb, 10)
        self._cam_info_sub = self.create_subscription(CameraInfo, depth_info_topic, self._camera_info_cb, 10)

        self.get_logger().info(
            f"TF projection ready. tracking_2d={tracking_2d_topic}, depth={depth_topic}, "
            f"target={self.target_parent_frame}->{self.target_frame_name}"
        )

    def _camera_info_cb(self, msg: CameraInfo):
        if self.camera_intrinsics is not None:
            return
        fx = float(msg.k[0])
        fy = float(msg.k[4])
        cx = float(msg.k[2])
        cy = float(msg.k[5])
        self.camera_intrinsics = (fx, fy, cx, cy)
        self.camera_frame_id = msg.header.frame_id
        self.get_logger().info(
            f"Camera intrinsics set fx={fx:.1f} fy={fy:.1f} cx={cx:.1f} cy={cy:.1f} frame={self.camera_frame_id}"
        )

    def _depth_cb(self, msg: RosImage):
        try:
            depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding="32FC1")
            stamp_sec = float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) * 1e-9
            self._depth_cache.append({"stamp": stamp_sec, "depth": depth, "header": msg.header})
        except Exception as exc:
            self.get_logger().warn(f"Depth decode failed: {exc}", throttle_duration_sec=2.0)

    def _find_depth_by_stamp(self, stamp_sec: float) -> Optional[dict]:
        if not self._depth_cache:
            return None
        best = None
        best_dt = 1e9
        for item in self._depth_cache:
            dt = abs(float(item["stamp"]) - stamp_sec)
            if dt < best_dt:
                best = item
                best_dt = dt
        if best is None:
            return None
        if best_dt > self.depth_match_tolerance_sec:
            return None
        return best

    def _get_valid_depth(self, depth_map, u, v, radius=3):
        h, w = depth_map.shape[:2]
        v_c = min(max(0, v), h - 1)
        u_c = min(max(0, u), w - 1)
        v_min, v_max = max(0, v_c - radius), min(h, v_c + radius + 1)
        u_min, u_max = max(0, u_c - radius), min(w, u_c + radius + 1)
        patch = depth_map[v_min:v_max, u_min:u_max]
        finite = patch[np.isfinite(patch) & (patch > 0)]
        if len(finite) == 0:
            return 0.0, False
        med = float(np.median(finite))
        if med > 20.0:
            med = med / 1000.0
        if med <= 0.05 or med >= 10.0:
            return 0.0, False
        return med, True

    def _lookup_transform_at_stamp(self, source_frame: str, header) -> TransformStamped:
        st = rclpy.time.Time.from_msg(header.stamp)
        timeout = Duration(seconds=self.tf_lookup_timeout_sec)
        try:
            return self.tf_buffer.lookup_transform(self.target_parent_frame, source_frame, st, timeout=timeout)
        except Exception as exc:
            txt = str(exc)
            m = re.search(r"Requested time ([\d.]+) but the latest data is at time ([\d.]+)", txt)
            if m:
                req = float(m.group(1))
                latest = float(m.group(2))
                if 0.0 <= (req - latest) <= self.tf_future_tolerance_sec:
                    sec = int(latest)
                    nsec = int((latest - sec) * 1e9)
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

    def _tracking_2d_cb(self, msg: String):
        try:
            payload = json.loads(msg.data)
        except Exception:
            return
        if not is_valid_tracking_2d_payload(payload):
            return
        if self.camera_intrinsics is None:
            return

        stamp_sec = float(payload["stamp_sec"]) + float(payload["stamp_nanosec"]) * 1e-9
        depth_item = self._find_depth_by_stamp(stamp_sec)
        if depth_item is None:
            self.get_logger().warn("depth_cache_miss for tracking stamp", throttle_duration_sec=1.0)
            return
        depth_map = depth_item["depth"]
        header = depth_item["header"]
        u = int(payload["u"])
        v = int(payload["v"])
        z, ok = self._get_valid_depth(depth_map, u, v)
        if not ok:
            self.get_logger().warn("No valid depth for tracking point", throttle_duration_sec=1.0)
            return

        fx, fy, cx, cy = self.camera_intrinsics
        x = (u - cx) * z / fx
        y = (v - cy) * z / fy
        source_frame = str(payload.get("frame_id", "")) or self.camera_frame_id or self.camera_frame_fallback

        try:
            t = self._lookup_transform_at_stamp(source_frame, header)
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

        tf_msg = TransformStamped()
        tf_msg.header.stamp = self.get_clock().now().to_msg()
        tf_msg.header.frame_id = self.target_parent_frame
        tf_msg.child_frame_id = self.target_frame_name
        tf_msg.transform.translation.x = float(target_x)
        tf_msg.transform.translation.y = float(target_y)
        tf_msg.transform.translation.z = float(target_z)
        tf_msg.transform.rotation.w = 1.0
        self.tf_broadcaster.sendTransform(tf_msg)

        pose = PoseStamped()
        pose.header = tf_msg.header
        pose.pose.position.x = tf_msg.transform.translation.x
        pose.pose.position.y = tf_msg.transform.translation.y
        pose.pose.position.z = tf_msg.transform.translation.z
        pose.pose.orientation.w = 1.0
        self._pose_pub.publish(pose)


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
