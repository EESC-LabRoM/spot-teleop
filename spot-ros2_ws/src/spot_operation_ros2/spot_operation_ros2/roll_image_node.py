#!/usr/bin/env python3
import json
import math
import threading
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
from sensor_msgs.msg import Image as RosImage
from std_srvs.srv import Trigger
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


def inverse_rotate_coords_1000(bbox_1000, grasps_1000, M_forward, rotated_size, original_size):
    """
    Map bbox and grasp points from the rotated image's [0-1000] space
    back to the original image's [0-1000] space.
    """
    rot_w, rot_h = rotated_size
    orig_w, orig_h = original_size
    M_inv = cv2.invertAffineTransform(M_forward)

    xmin = bbox_1000[0] / 1000.0 * rot_w
    ymin = bbox_1000[1] / 1000.0 * rot_h
    xmax = bbox_1000[2] / 1000.0 * rot_w
    ymax = bbox_1000[3] / 1000.0 * rot_h

    corners = np.array([
        [xmin, ymin], [xmax, ymin], [xmax, ymax], [xmin, ymax],
    ], dtype=np.float64)
    ones = np.ones((4, 1), dtype=np.float64)
    corners_h = np.hstack([corners, ones])
    orig_corners = (M_inv @ corners_h.T).T

    ox_min = max(0.0, np.min(orig_corners[:, 0]))
    oy_min = max(0.0, np.min(orig_corners[:, 1]))
    ox_max = min(float(orig_w), np.max(orig_corners[:, 0]))
    oy_max = min(float(orig_h), np.max(orig_corners[:, 1]))

    corrected_bbox = [
        int(ox_min / orig_w * 1000),
        int(oy_min / orig_h * 1000),
        int(ox_max / orig_w * 1000),
        int(oy_max / orig_h * 1000),
    ]

    corrected_grasps = []
    for g in (grasps_1000 or []):
        gx_px = g[0] / 1000.0 * rot_w
        gy_px = g[1] / 1000.0 * rot_h
        pt_h = np.array([gx_px, gy_px, 1.0])
        orig_pt = M_inv @ pt_h
        ox = max(0.0, min(float(orig_w), orig_pt[0]))
        oy = max(0.0, min(float(orig_h), orig_pt[1]))
        corrected_grasps.append([int(ox / orig_w * 1000), int(oy / orig_h * 1000)])

    return corrected_bbox, corrected_grasps


class RollImageNode(Node):
    def __init__(self):
        super().__init__("roll_image_node")
        self.declare_parameter("rgb_topic", "/hand/rgb")
        self.declare_parameter("rotated_rgb_topic", "/hand/rgb_upright")
        self.declare_parameter("target_frame", "odom")
        self.declare_parameter("source_frame", "hand_cam")
        self.declare_parameter("vlm_service_name", "/vlm/trigger_relocalize")
        self.declare_parameter("service_name", "/roll/trigger_relocalize")
        self.declare_parameter("tf_lookup_timeout_sec", 1.0)
        self.declare_parameter("min_abs_rotation_deg", 5.0)

        rgb_topic = str(self.get_parameter("rgb_topic").value)
        rotated_rgb_topic = str(self.get_parameter("rotated_rgb_topic").value)
        self.target_frame = str(self.get_parameter("target_frame").value)
        self.source_frame = str(self.get_parameter("source_frame").value)
        vlm_service_name = str(self.get_parameter("vlm_service_name").value)
        service_name = str(self.get_parameter("service_name").value)
        self.tf_lookup_timeout_sec = float(max(0.01, self.get_parameter("tf_lookup_timeout_sec").value))
        self.min_abs_rotation_deg = float(max(0.0, self.get_parameter("min_abs_rotation_deg").value))

        self.bridge = CvBridge()
        self._cb_group = ReentrantCallbackGroup()
        self._latest_rgb = None
        self._latest_header = None
        self._last_rotate_matrix = None
        self._last_rotated_size = None
        self._last_original_size = None
        self._roll_req_seq = 0

        self.tf_buffer = Buffer(cache_time=Duration(seconds=120.0))
        self.tf_listener = TransformListener(self.tf_buffer, self, spin_thread=True)

        self._rgb_sub = self.create_subscription(
            RosImage, rgb_topic, self._rgb_cb, 10, callback_group=self._cb_group
        )
        self._rotated_pub = self.create_publisher(RosImage, rotated_rgb_topic, 10)
        self._vlm_client = self.create_client(
            Trigger, vlm_service_name, callback_group=self._cb_group
        )
        self._srv = self.create_service(
            Trigger, service_name, self._handle_roll_relocalize, callback_group=self._cb_group
        )
        self.get_logger().info(
            f"Roll image node ready. rgb={rgb_topic} rotated={rotated_rgb_topic} roll_srv={service_name} vlm_srv={vlm_service_name}"
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

    def _rgb_cb(self, msg: RosImage):
        try:
            cv_rgb = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            cv_rgb = cv2.cvtColor(cv_rgb, cv2.COLOR_BGR2RGB)
            self._latest_rgb = cv_rgb
            self._latest_header = msg.header
        except Exception as exc:
            self.get_logger().warn(f"RGB decode failed: {exc}", throttle_duration_sec=2.0)

    def _lookup_roll_deg(self, header) -> float:
        st = rclpy.time.Time.from_msg(header.stamp)
        t = self.tf_buffer.lookup_transform(
            self.target_frame,
            self.source_frame,
            st,
            timeout=Duration(seconds=self.tf_lookup_timeout_sec),
        )
        x = t.transform.rotation.x
        y = t.transform.rotation.y
        z = t.transform.rotation.z
        w = t.transform.rotation.w
        sinr_cosp = 2.0 * (w * x + y * z)
        cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
        roll = math.atan2(sinr_cosp, cosr_cosp)
        return -math.degrees(roll)

    def _handle_roll_relocalize(self, _req, resp):
        if self._latest_rgb is None or self._latest_header is None:
            resp.success = False
            resp.message = "No RGB frame available yet"
            return resp
        if not self._vlm_client.wait_for_service(timeout_sec=0.0):
            resp.success = False
            resp.message = "VLM service unavailable"
            return resp
        try:
            rgb_np = self._latest_rgb.copy()
            header = self._latest_header
            self._roll_req_seq += 1
            req_id = int(self._roll_req_seq)
            # #region agent log
            self._dbg_log(
                "H13",
                "roll_image_node.py:_handle_roll_relocalize",
                "roll_request_start",
                {
                    "req_id": req_id,
                    "header_stamp_sec": int(header.stamp.sec),
                    "header_stamp_nanosec": int(header.stamp.nanosec),
                },
            )
            # #endregion
            img_pil = Image.fromarray(rgb_np)
            correction_angle = self._lookup_roll_deg(header)
            m_fwd = None
            if abs(correction_angle) > self.min_abs_rotation_deg:
                img_rot, m_fwd, rot_size = rotate_image_upright(img_pil, correction_angle)
                self._last_rotate_matrix = m_fwd
                self._last_rotated_size = rot_size
                self._last_original_size = img_pil.size
            else:
                img_rot = img_pil
                self._last_rotate_matrix = None
                self._last_rotated_size = img_pil.size
                self._last_original_size = img_pil.size

            rot_np = np.array(img_rot.convert("RGB"))
            rot_bgr = cv2.cvtColor(rot_np, cv2.COLOR_RGB2BGR)
            rot_msg = self.bridge.cv2_to_imgmsg(rot_bgr, encoding="bgr8")
            rot_msg.header = header  # keep original time T
            self._rotated_pub.publish(rot_msg)
            # #region agent log
            self._dbg_log(
                "H13",
                "roll_image_node.py:_handle_roll_relocalize",
                "rotated_frame_published",
                {
                    "req_id": req_id,
                    "published_stamp_sec": int(rot_msg.header.stamp.sec),
                    "published_stamp_nanosec": int(rot_msg.header.stamp.nanosec),
                    "correction_angle_deg": float(correction_angle),
                    "rotated_size": list(self._last_rotated_size) if self._last_rotated_size else None,
                },
            )
            # #endregion

            t0 = time.time()
            timeout_sec = 10.0
            future = self._vlm_client.call_async(Trigger.Request())
            done_evt = threading.Event()
            future.add_done_callback(lambda _f: done_evt.set())
            completed = done_evt.wait(timeout_sec)
            result = None
            if completed and future.done() and not future.cancelled():
                try:
                    result = future.result()
                except Exception:
                    result = None
            # #region agent log
            self._dbg_log(
                "H14",
                "roll_image_node.py:_handle_roll_relocalize",
                "vlm_call_completed",
                {
                    "req_id": req_id,
                    "elapsed_ms": int((time.time() - t0) * 1000),
                    "future_done": bool(future.done()),
                    "future_cancelled": bool(future.cancelled()),
                    "result_is_none": result is None,
                },
            )
            # #endregion
            # #region agent log
            self._dbg_log(
                "H14",
                "roll_image_node.py:_handle_roll_relocalize",
                "vlm_service_returned",
                {
                    "req_id": req_id,
                    "result_is_none": result is None,
                    "result_success": bool(result.success) if result is not None else False,
                    "result_message_preview": (result.message[:180] if result is not None else ""),
                },
            )
            # #endregion
            if result is None or not result.success:
                resp.success = False
                resp.message = result.message if result is not None else "VLM call failed"
                return resp

            seed = json.loads(result.message)
            if self._last_rotate_matrix is not None and "bbox_1000" in seed:
                bbox_orig, grasps_orig = inverse_rotate_coords_1000(
                    seed.get("bbox_1000", []),
                    seed.get("grasps_1000", []),
                    self._last_rotate_matrix,
                    self._last_rotated_size,
                    self._last_original_size,
                )
                seed["bbox_1000"] = bbox_orig
                seed["grasps_1000"] = grasps_orig
            resp.success = True
            resp.message = json.dumps(seed, ensure_ascii=True)
            return resp
        except Exception as exc:
            resp.success = False
            resp.message = str(exc)
            return resp


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
