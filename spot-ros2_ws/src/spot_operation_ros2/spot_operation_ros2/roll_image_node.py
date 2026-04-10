#!/usr/bin/env python3
import json
import math

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from PIL import Image
from rclpy.duration import Duration
from rclpy.node import Node
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
        self.declare_parameter("rotated_rgb_topic", "/hand/rgb_upright")
        self.declare_parameter("roll_metadata_topic", "/hand/roll_metadata")
        self.declare_parameter("target_frame", "odom")
        self.declare_parameter("source_frame", "hand_cam")
        self.declare_parameter("tf_lookup_timeout_sec", 1.0)
        self.declare_parameter("min_abs_rotation_deg", 5.0)

        rgb_topic = str(self.get_parameter("rgb_topic").value)
        rotated_rgb_topic = str(self.get_parameter("rotated_rgb_topic").value)
        roll_metadata_topic = str(self.get_parameter("roll_metadata_topic").value)
        self.target_frame = str(self.get_parameter("target_frame").value)
        self.source_frame = str(self.get_parameter("source_frame").value)
        self.min_abs_rotation_deg = float(max(0.0, self.get_parameter("min_abs_rotation_deg").value))

        self.bridge = CvBridge()
        self._last_roll_deg = 0.0

        self.tf_buffer = Buffer(cache_time=Duration(seconds=120.0))
        self.tf_listener = TransformListener(self.tf_buffer, self, spin_thread=True)

        self._rgb_sub = self.create_subscription(RosImage, rgb_topic, self._rgb_cb, 10)
        self._rotated_pub = self.create_publisher(RosImage, rotated_rgb_topic, 10)
        self._roll_metadata_pub = self.create_publisher(String, roll_metadata_topic, 10)
        self.get_logger().info(
            f"Roll image node ready. rgb={rgb_topic} rotated={rotated_rgb_topic} "
            f"roll_metadata={roll_metadata_topic}"
        )

    def _lookup_roll_deg(self, header) -> float:
        st = rclpy.time.Time.from_msg(header.stamp)
        t = self.tf_buffer.lookup_transform(
            self.target_frame,
            self.source_frame,
            st,
            timeout=Duration(seconds=0),
        )
        x = t.transform.rotation.x
        y = t.transform.rotation.y
        z = t.transform.rotation.z
        w = t.transform.rotation.w
        sinr_cosp = 2.0 * (w * x + y * z)
        cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
        roll = math.atan2(sinr_cosp, cosr_cosp)
        return -math.degrees(roll)

    def _rgb_cb(self, msg: RosImage):
        try:
            cv_rgb = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            cv_rgb = cv2.cvtColor(cv_rgb, cv2.COLOR_BGR2RGB)
        except Exception as exc:
            self.get_logger().warn(f"RGB decode failed: {exc}", throttle_duration_sec=2.0)
            return

        orig_h, orig_w = cv_rgb.shape[:2]

        try:
            correction_angle = self._lookup_roll_deg(msg.header)
            self._last_roll_deg = correction_angle
        except Exception:
            correction_angle = self._last_roll_deg

        img_pil = Image.fromarray(cv_rgb)
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

        meta = String()
        meta.data = json.dumps({"angle_deg": float(correction_angle), "orig_w": int(orig_w), "orig_h": int(orig_h)})
        self._roll_metadata_pub.publish(meta)


def main(args=None):
    rclpy.init(args=args)
    node = RollImageNode()
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
