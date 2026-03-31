#!/usr/bin/env python3
import json
import time
import uuid

import cv2
import numpy as np
import rclpy
from std_msgs.msg import String

from .detect_qwen import (
    DetectQwenNode,
    _best_mask_from_results,
    _sam_prompt_masks,
    create_crop_mosaic,
    draw_result,
    load_sam_model,
    select_best_mask_by_bbox_iou,
)


class Sam2TrackerNode(DetectQwenNode):
    """2D-only tracker node: seed -> SAM2 tracking -> filtered grasp uv."""

    def __init__(self):
        super().__init__()
        self.initial_detection_done = True
        self.declare_parameter("seed_command_topic", "/perception/seed_command")
        self.declare_parameter("tracking_state_topic", "/tracking_state")
        self.declare_parameter("tracking_2d_topic", "/tracking_2d_result")

        seed_topic = self.get_parameter("seed_command_topic").value
        tracking_state_topic = self.get_parameter("tracking_state_topic").value
        tracking_2d_topic = self.get_parameter("tracking_2d_topic").value

        self._pending_seed = None
        self._seed_cb_count = 0

        self._seed_sub = self.create_subscription(String, seed_topic, self._seed_command_cb, 10)
        self._tracking_state_pub = self.create_publisher(String, tracking_state_topic, 10)
        self._tracking_2d_pub = self.create_publisher(String, tracking_2d_topic, 10)
        self._hand_ui_calls = 0

        self.get_logger().info(
            f"SAM2 tracker 2D enabled. seed_topic={seed_topic}, tracking2d={tracking_2d_topic}"
        )
        self._dbg_timer_count = 0

    def _dbg_log(self, hypothesis_id: str, location: str, message: str, data: dict):
        # #region agent log
        payload = {
            "sessionId": "eb5d37",
            "id": f"log_{int(time.time() * 1000)}_{uuid.uuid4().hex[:8]}",
            "timestamp": int(time.time() * 1000),
            "runId": "repro-3",
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

    def _run_initial_detection(self):
        # Disable internal Qwen path; this node only starts from external seed.
        return

    def _synced_image_cb(self, rgb_msg, depth_msg):
        super()._synced_image_cb(rgb_msg, depth_msg)

    def _seed_command_cb(self, msg: String):
        try:
            self._seed_cb_count += 1
            self._pending_seed = json.loads(msg.data)
            # #region agent log
            self._dbg_log(
                "H6",
                "sam2_tracker_node.py:_seed_command_cb",
                "seed_received",
                {
                    "seed_count": self._seed_cb_count,
                    "has_bbox": isinstance(self._pending_seed.get("bbox_1000"), list),
                    "bbox_len": len(self._pending_seed.get("bbox_1000", []))
                    if isinstance(self._pending_seed.get("bbox_1000"), list)
                    else -1,
                },
            )
            # #endregion
            self.get_logger().info(
                f"[TRACKER] seed command received (count={self._seed_cb_count})",
                throttle_duration_sec=1.0,
            )
        except Exception as exc:
            self.get_logger().warn(f"Ignoring invalid seed command JSON: {exc}")

    def _publish_tracking_2d(self, u: int, v: int, score: float, mask_area: int, header):
        payload = {
            "stamp_sec": int(header.stamp.sec),
            "stamp_nanosec": int(header.stamp.nanosec),
            "frame_id": str(header.frame_id),
            "u": int(u),
            "v": int(v),
            "score": float(score),
            "mask_area": int(mask_area),
        }
        msg = String()
        msg.data = json.dumps(payload, ensure_ascii=True)
        self._tracking_2d_pub.publish(msg)

    def _apply_seed_command(self, seed: dict):
        bbox_1000 = seed.get("bbox_1000")
        grasps_1000 = seed.get("grasps_1000", [])
        if not isinstance(bbox_1000, list) or len(bbox_1000) < 4:
            raise RuntimeError("missing bbox_1000 in seed")
        if self.latest_rgb is None or not hasattr(self, "latest_depth_header") or self.latest_depth_header is None:
            raise RuntimeError("latest frame unavailable for seed apply")

        img_pil = self.latest_rgb.copy()
        header = self.latest_depth_header
        orig_w, orig_h = img_pil.size

        x1 = int((bbox_1000[0] / 1000.0) * orig_w)
        y1 = int((bbox_1000[1] / 1000.0) * orig_h)
        x2 = int((bbox_1000[2] / 1000.0) * orig_w)
        y2 = int((bbox_1000[3] / 1000.0) * orig_h)
        x1, y1 = max(0, x1), max(0, y1)
        x2, y2 = min(orig_w - 1, x2), min(orig_h - 1, y2)
        bbox = [x1, y1, x2, y2]

        grasp_points_uv = []
        for g in grasps_1000:
            if not isinstance(g, list) or len(g) < 2:
                continue
            gu = int((g[0] / 1000.0) * orig_w)
            gv = int((g[1] / 1000.0) * orig_h)
            grasp_points_uv.append([max(0, min(orig_w - 1, gu)), max(0, min(orig_h - 1, gv))])
        if not grasp_points_uv:
            grasp_points_uv = [[(x1 + x2) // 2, (y1 + y2) // 2]]

        # Keep parity with detect_qwen debug artifacts.
        seed_box = {
            "label": str(seed.get("label", self.object_prompt)),
            "bbox_1000": [float(v) for v in bbox_1000[:4]],
            "grasps_1000": [[float(p[0]), float(p[1])] for p in grasps_1000 if isinstance(p, list) and len(p) >= 2],
            "confidence": float(seed.get("confidence", 1.0)),
        }
        draw_result(img_pil, [seed_box], "camera_capture_detected.jpg")
        self.get_logger().info("Saved camera capture: camera_capture_detected.jpg")

        sam_img_predictor = load_sam_model()
        grasp_u, grasp_v = grasp_points_uv[0]
        best_mask = None
        if sam_img_predictor is not None:
            all_masks = []
            for pt in grasp_points_uv:
                masks, scores = _sam_prompt_masks(sam_img_predictor, img_pil, pt, multimask_output=True)
                if len(masks) == 0:
                    continue
                all_masks.append(masks[int(np.argmax(scores))])
            if all_masks:
                best_idx = 0
                if len(all_masks) > 1:
                    create_crop_mosaic(img_pil, all_masks, bbox, "debug_mosaic.jpg")
                    self.get_logger().info("Saved mosaic: debug_mosaic.jpg")
                    best_idx = int(select_best_mask_by_bbox_iou(all_masks, bbox, img_pil.size))
                best_mask = all_masks[best_idx]
                grasp_u, grasp_v = grasp_points_uv[min(best_idx, len(grasp_points_uv) - 1)]

        img_np = np.array(img_pil.convert("RGB"))
        init_results = self.video_predictor(img_np, points=[[grasp_u, grasp_v]], labels=[1])
        best_tracked_mask, score = _best_mask_from_results(init_results)
        if best_tracked_mask is None:
            raise RuntimeError("tracker init produced no mask")

        mask_np = best_tracked_mask.astype(np.uint8) if best_mask is None else best_mask.astype(np.uint8)
        self.tracking_active = True
        self.tracking_frame_count = 1
        self.last_tracking_score = score
        self.grasp_points_uv_stored = grasp_points_uv
        self._publish_segmentation_mask(mask_np)

        m = cv2.moments(mask_np)
        if m["m00"] != 0:
            mask_u = int(m["m10"] / m["m00"])
            mask_v = int(m["m01"] / m["m00"])
            self.grasp_offset = (grasp_u - mask_u, grasp_v - mask_v)
            u_out, v_out = grasp_u, grasp_v
        else:
            self.grasp_offset = (0, 0)
            u_out, v_out = grasp_u, grasp_v
        self._publish_tracking_2d(u_out, v_out, float(score), int(mask_np.sum()), header)
        if self.visualize:
            self._hand_ui_calls += 1
            self._dbg_log(
                "H17",
                "sam2_tracker_node.py:_apply_seed_command",
                "hand_imshow_attempt",
                {"call_idx": self._hand_ui_calls, "tracking_active": bool(self.tracking_active)},
            )
            self._update_display(img_pil, mask_np, float(score), centroid_uv=(u_out, v_out))

    def _run_tracking_step_2d(self, now: float):
        if self.latest_rgb is None or self.latest_depth is None:
            return
        if not self.new_frame_available:
            return
        if (now - self._last_track_time) < self.active_tracking_interval:
            return

        self.new_frame_available = False
        self._last_track_time = now
        img_pil = self.latest_rgb
        img_np = np.array(img_pil.convert("RGB"))
        tracked_results = self.video_predictor(img_np)
        best_tracked_mask, score = _best_mask_from_results(tracked_results)
        if best_tracked_mask is None:
            self._tracking_lost_streak += 1
            if self._tracking_lost_streak >= self.tracking_lost_confirm_frames:
                self.tracking_active = False
            return

        mask_np = best_tracked_mask.astype(np.uint8)
        mask_area = int(mask_np.sum())
        self.last_tracking_score = float(score)
        self.tracking_frame_count += 1
        self._tracking_lost_streak = 0
        self._publish_segmentation_mask(mask_np)

        m = cv2.moments(mask_np)
        if m["m00"] == 0:
            return
        u = int(m["m10"] / m["m00"])
        v = int(m["m01"] / m["m00"])
        if hasattr(self, "grasp_offset"):
            u += int(self.grasp_offset[0])
            v += int(self.grasp_offset[1])
        h = self.latest_depth_header if hasattr(self, "latest_depth_header") and self.latest_depth_header else None
        if h is None:
            return
        self._publish_tracking_2d(u, v, float(score), mask_area, h)
        if self.visualize:
            self._hand_ui_calls += 1
            if self._hand_ui_calls <= 12:
                self._dbg_log(
                    "H17",
                    "sam2_tracker_node.py:_run_tracking_step_2d",
                    "hand_imshow_attempt",
                    {
                        "call_idx": self._hand_ui_calls,
                        "tracking_active": bool(self.tracking_active),
                        "mask_area": int(mask_area),
                    },
                )
            self._update_display(img_pil, mask_np, float(score), centroid_uv=(u, v))

    def _tracking_timer_cb(self):
        self._dbg_timer_count += 1
        if self._dbg_timer_count <= 8:
            # #region agent log
            self._dbg_log(
                "H7",
                "sam2_tracker_node.py:_tracking_timer_cb",
                "tracking_timer_tick",
                {
                    "pending_seed": self._pending_seed is not None,
                    "detection_running": bool(self.detection_running),
                    "tracking_active": bool(self.tracking_active),
                },
            )
            # #endregion
        if self._pending_seed is not None and not self.detection_running:
            seed = self._pending_seed
            self._pending_seed = None
            try:
                self._apply_seed_command(seed)
                # #region agent log
                self._dbg_log(
                    "H11",
                    "sam2_tracker_node.py:_tracking_timer_cb",
                    "seed_apply_ok",
                    {"tracking_active": bool(self.tracking_active)},
                )
                # #endregion
                self.get_logger().info(
                    "[TRACKER] seed applied, tracking initialized",
                    throttle_duration_sec=1.0,
                )
            except Exception as exc:
                # #region agent log
                self._dbg_log(
                    "H11",
                    "sam2_tracker_node.py:_tracking_timer_cb",
                    "seed_apply_failed",
                    {"error": str(exc)[:240]},
                )
                # #endregion
                self.get_logger().error(f"Seed apply failed: {exc}")

        if self.tracking_active:
            self._run_tracking_step_2d(time.time())

        st = String()
        st.data = "TRACKING" if self.tracking_active else "LOST"
        self._tracking_state_pub.publish(st)


def main(args=None):
    rclpy.init(args=args)
    node = Sam2TrackerNode()
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
