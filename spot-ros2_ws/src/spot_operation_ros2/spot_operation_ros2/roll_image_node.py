#!/usr/bin/env python3
import json
import math
import time
import uuid

import cv2
import numpy as np
import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from cv_bridge import CvBridge
from PIL import Image
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import Image as RosImage
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener


def rotate_image_upright(img_pil, angle_deg):
    """
    Rotate a PIL image by angle_deg (CCW positive, OpenCV convention).
    Expands canvas so no content is clipped. Fills border with neutral gray.
    Returns (rotated_pil, M_forward, (rot_w, rot_h)).
    """
    img_np = np.array(img_pil.convert("RGB"))
    h, w = img_np.shape[:2]
    center = (w / 2.0, h / 2.0)
    M = cv2.getRotationMatrix2D(center, angle_deg, 1.0)
    cos_a = abs(M[0, 0])
    sin_a = abs(M[0, 1])
    rot_w = int(h * sin_a + w * cos_a)
    rot_h = int(h * cos_a + w * sin_a)
    M[0, 2] += (rot_w - w) / 2.0
    M[1, 2] += (rot_h - h) / 2.0
    rotated_np = cv2.warpAffine(img_np, M, (rot_w, rot_h), borderValue=(127, 127, 127))
    rotated_pil = Image.fromarray(rotated_np)
    return rotated_pil, M, (rot_w, rot_h)


class RollImageNode(Node):
    def __init__(self):
        super().__init__("roll_image_node")
        self.declare_parameter("rgb_topic", "/hand/rgb")
        self.declare_parameter("camera_info_topic", "/hand/camera_info")
        self.declare_parameter("rotated_rgb_topic", "/hand/rgb_upright")
        self.declare_parameter("roll_metadata_topic", "/hand/roll_metadata")
        self.declare_parameter("target_frame", "body")
        self.declare_parameter("tf_lookup_timeout_sec", 1.0)
        self.declare_parameter("min_abs_rotation_deg", 5.0)

        rgb_topic = str(self.get_parameter("rgb_topic").value)
        camera_info_topic = str(self.get_parameter("camera_info_topic").value)
        rotated_rgb_topic = str(self.get_parameter("rotated_rgb_topic").value)
        roll_metadata_topic = str(self.get_parameter("roll_metadata_topic").value)
        self.target_frame = str(self.get_parameter("target_frame").value)
        self.tf_lookup_timeout_sec = float(max(0.01, self.get_parameter("tf_lookup_timeout_sec").value))
        self.min_abs_rotation_deg = float(max(0.0, self.get_parameter("min_abs_rotation_deg").value))

        self.bridge = CvBridge()
        self._cb_group = ReentrantCallbackGroup()
        self._latest_rgb = None
        self._latest_header = None
        self._camera_frame_id = None
        self._last_correction_angle = 0.0

        self.tf_buffer = Buffer(cache_time=Duration(seconds=120.0))
        self.tf_listener = TransformListener(self.tf_buffer, self, spin_thread=False)

        self._rgb_sub = self.create_subscription(
            RosImage, rgb_topic, self._rgb_cb, 10, callback_group=self._cb_group
        )
        self._camera_info_sub = self.create_subscription(
            CameraInfo, camera_info_topic, self._camera_info_cb, 10, callback_group=self._cb_group
        )
        self._rotated_pub = self.create_publisher(RosImage, rotated_rgb_topic, 10)
        self._roll_meta_pub = self.create_publisher(String, roll_metadata_topic, 10)
        self.get_logger().info(
            f"Roll image node ready. rgb={rgb_topic} cam_info={camera_info_topic} "
            f"rotated={rotated_rgb_topic} metadata={roll_metadata_topic}"
        )

    def _dbg_log(self, hypothesis_id: str, location: str, message: str, data: dict):
        # #region agent log
        payload = {
            "sessionId": "eb5d37",
            "id": f"log_{int(time.time() * 1000)}_{uuid.uuid4().hex[:8]}",
            "timestamp": int(time.time() * 1000),
            "runId": "repro-2",
            "hypothesisId": hypothesis_id,
            "location": location,
            "message": message,
            "data": data,
        }
        try:
            with open("/home/spot-teleop/spot-ros2_ws/.cursor/debug-eb5d37.log", "a", encoding="utf-8") as f:
                f.write(json.dumps(payload, ensure_ascii=True) + "\n")
        except Exception:
            pass
        # #endregion

    def _camera_info_cb(self, msg: CameraInfo):
        frame = str(msg.header.frame_id).strip()
        if not frame:
            return
        self._camera_frame_id = frame

    def _resolve_source_frame(self, rgb_header) -> str:
        if self._camera_frame_id:
            return self._camera_frame_id
        header_frame = str(rgb_header.frame_id).strip()
        if header_frame:
            return header_frame
        raise RuntimeError("No source frame available from camera_info or RGB header")

    def _lookup_roll_deg(self, source_frame: str, header) -> float:
        """Non-blocking TF lookup; falls back to latest available transform."""
        st = rclpy.time.Time.from_msg(header.stamp)
        try:
            t = self.tf_buffer.lookup_transform(
                self.target_frame, source_frame, st,
                timeout=Duration(seconds=0.0),
            )
        except Exception:
            t = self.tf_buffer.lookup_transform(
                self.target_frame, source_frame, rclpy.time.Time(),
                timeout=Duration(seconds=0.0),
            )
        x = t.transform.rotation.x
        y = t.transform.rotation.y
        z = t.transform.rotation.z
        w = t.transform.rotation.w
        up_x = 2.0 * (x * z + w * y)
        up_y = 2.0 * (y * z - w * x)
        return math.degrees(math.atan2(up_y, up_x))

    def _rgb_cb(self, msg: RosImage):
        try:
            cv_rgb = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            cv_rgb = cv2.cvtColor(cv_rgb, cv2.COLOR_BGR2RGB)
            self._latest_rgb = cv_rgb
            self._latest_header = msg.header
        except Exception as exc:
            self.get_logger().warn(f"RGB decode failed: {exc}", throttle_duration_sec=2.0)
            return

        try:
            source_frame = self._resolve_source_frame(msg.header)
            correction_angle = self._lookup_roll_deg(source_frame, msg.header)
            self._last_correction_angle = correction_angle
        except Exception:
            correction_angle = self._last_correction_angle

        img_pil = Image.fromarray(cv_rgb)
        orig_w, orig_h = img_pil.size

        if abs(correction_angle) > self.min_abs_rotation_deg:
            img_rot, _, _ = rotate_image_upright(img_pil, correction_angle)
        else:
            img_rot = img_pil
            correction_angle = 0.0

        rot_np = np.array(img_rot.convert("RGB"))
        rot_bgr = cv2.cvtColor(rot_np, cv2.COLOR_RGB2BGR)
        rot_msg = self.bridge.cv2_to_imgmsg(rot_bgr, encoding="bgr8")
        rot_msg.header = msg.header
        self._rotated_pub.publish(rot_msg)

        meta_msg = String()
        meta_msg.data = json.dumps({"angle_deg": correction_angle, "orig_w": orig_w, "orig_h": orig_h})
        self._roll_meta_pub.publish(meta_msg)


def main(args=None):
    rclpy.init(args=args)
    node = RollImageNode()
    executor = MultiThreadedExecutor(num_threads=2)
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
