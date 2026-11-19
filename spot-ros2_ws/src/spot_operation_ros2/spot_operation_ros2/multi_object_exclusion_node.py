#!/usr/bin/env python3
"""SAM2 per-frame object masks for the static ZED1 table scene.

Publishes source-stamped per-object PNG masks and a union PNG. If any required
mask is missing or fails geometry gates, no union is published; the existing
depth filter consequently pauses map integration. Fixed prompts must be
reseeded after an object is moved. This node never changes TF, the map, or a
robot controller.
"""
from __future__ import annotations

import json
import time

import cv2
import numpy as np
import rclpy
import torch
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import String
from ultralytics import SAM

from .object_exclusion_union import MaskRejected, ObjectMaskSpec, union_object_masks


DEFAULT_OBJECTS = [
    {"id": "drill", "box_norm": [0.46, 0.62, 0.56, 0.94], "min_fraction": 0.003, "max_fraction": 0.06},
    {"id": "tennis_ball", "box_norm": [0.412, 0.506, 0.45, 0.576], "min_fraction": 0.0004, "max_fraction": 0.015},
    {"id": "sprite", "box_norm": [0.469, 0.415, 0.525, 0.63], "min_fraction": 0.001, "max_fraction": 0.04},
    {"id": "screwdriver", "box_norm": [0.38, 0.59, 0.49, 0.71], "min_fraction": 0.001, "max_fraction": 0.035},
]


class MultiObjectExclusionNode(Node):
    def __init__(self):
        super().__init__("zed1_multi_object_exclusion")
        self.declare_parameter("rgb_topic", "/vm/cable/rgb")
        self.declare_parameter("model_path", "/tmp/trackpkg/sam2.1_t.pt")
        self.declare_parameter("device", "0")
        self.declare_parameter("imgsz", 1024)
        self.declare_parameter("max_input_age_s", 1.25)
        self.declare_parameter("max_output_age_s", 1.40)
        self.declare_parameter("minimum_inference_interval_s", 0.75)
        self.declare_parameter("objects_json", json.dumps(DEFAULT_OBJECTS))
        self.declare_parameter("union_topic", "/zed1/object_exclusion_mask/compressed")
        self.declare_parameter("diagnostics_topic", "/zed1/object_exclusion_status")
        self.declare_parameter("debug_prefix", "/zed1/object_masks")

        self.spec_data = json.loads(str(self.get_parameter("objects_json").value))
        self.specs = [ObjectMaskSpec(
            object_id=x["id"], box_norm=tuple(x["box_norm"]),
            min_fraction=float(x["min_fraction"]), max_fraction=float(x["max_fraction"]),
        ) for x in self.spec_data]
        self.model_path = str(self.get_parameter("model_path").value)
        self.device = str(self.get_parameter("device").value)
        self.imgsz = int(self.get_parameter("imgsz").value)
        self.max_age = float(self.get_parameter("max_input_age_s").value)
        self.max_output_age = float(self.get_parameter("max_output_age_s").value)
        self.min_interval = float(self.get_parameter("minimum_inference_interval_s").value)
        self.latest: Image | None = None
        self.last_infer_wall = 0.0
        self.busy = False
        self.model = None

        self.union_pub = self.create_publisher(
            CompressedImage, str(self.get_parameter("union_topic").value), 1)
        self.diag_pub = self.create_publisher(
            String, str(self.get_parameter("diagnostics_topic").value), 10)
        prefix = str(self.get_parameter("debug_prefix").value).rstrip("/")
        self.object_pubs = {
            spec.object_id: self.create_publisher(
                CompressedImage, f"{prefix}/{spec.object_id}/compressed", 1)
            for spec in self.specs
        }
        self.create_subscription(Image, str(self.get_parameter("rgb_topic").value),
                                 self._on_image, 1)
        self.create_timer(0.05, self._infer_latest)
        self.get_logger().info("ready; model loads on first fresh RGB frame")

    @staticmethod
    def _stamp(msg: Image) -> float:
        return msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

    def _on_image(self, msg: Image) -> None:
        self.latest = msg

    def _status(self, state: str, stamp: float, detail: str = "", stats=None) -> None:
        self.diag_pub.publish(String(data=json.dumps({
            "state": state, "source_stamp": stamp, "wall": time.time(),
            "detail": detail, "objects": stats or {},
        }, separators=(",", ":"))))

    def _compressed_png(self, mask: np.ndarray, source: Image) -> CompressedImage:
        ok, encoded = cv2.imencode(".png", mask)
        if not ok:
            raise RuntimeError("PNG encode failed")
        msg = CompressedImage()
        msg.header = source.header
        msg.format = "png; mono8"
        msg.data = encoded.tobytes()
        return msg

    def _infer_latest(self) -> None:
        if self.busy or self.latest is None:
            return
        now = time.time()
        if now - self.last_infer_wall < self.min_interval:
            return
        src = self.latest
        stamp = self._stamp(src)
        age = now - stamp
        if age < 0 or age > self.max_age:
            self._status("STALE_INPUT", stamp, f"age_s={age:.3f}")
            return
        self.busy = True
        self.latest = None
        self.last_infer_wall = now
        try:
            if src.encoding.lower() not in ("bgr8", "rgb8"):
                raise MaskRejected(f"unsupported RGB encoding {src.encoding}")
            dtype = np.uint8
            channels = 3
            row = np.frombuffer(src.data, dtype=dtype).reshape(src.height, src.step)[:, :src.width * channels]
            image = row.reshape(src.height, src.width, channels).copy()
            if src.encoding.lower() == "rgb8":
                image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)

            if self.model is None:
                torch.set_num_threads(2)
                self.model = SAM(self.model_path)
            h, w = image.shape[:2]
            boxes = [
                (np.asarray(spec.box_norm, float) * np.asarray([w, h, w, h])).tolist()
                for spec in self.specs
            ]
            results = self.model.predict(source=image, bboxes=boxes, device=self.device,
                                         imgsz=self.imgsz, save=False, verbose=False)
            if not results or results[0].masks is None:
                raise MaskRejected("SAM returned no masks")
            raw_masks = (results[0].masks.data > 0).to(torch.uint8).cpu().numpy()
            if len(raw_masks) != len(self.specs):
                raise MaskRejected(f"SAM returned {len(raw_masks)} masks for {len(self.specs)} prompts")
            masks = {}
            for spec, raw in zip(self.specs, raw_masks):
                m = raw
                if m.shape != (h, w):
                    m = cv2.resize(m, (w, h), interpolation=cv2.INTER_NEAREST)
                masks[spec.object_id] = m
            union, stats = union_object_masks(masks, self.specs, (h, w))

            # Inference can be slower than a single RGB interval. Never publish
            # a geometrically valid but already-stale exclusion mask.
            output_age = time.time() - stamp
            if output_age < 0 or output_age > self.max_output_age:
                raise MaskRejected(f"inference output stale age_s={output_age:.3f}")

            # Per-object topics preserve IDs for later cloud/grasp consumers.
            # Union is published only after every required mask passes.
            for object_id, mask in masks.items():
                self.object_pubs[object_id].publish(self._compressed_png(mask * 255, src))
            self.union_pub.publish(self._compressed_png(union, src))
            self._status("OK", stamp, f"age_s={output_age:.3f}", stats)
        except Exception as exc:
            self._status("REJECTED", stamp, f"{type(exc).__name__}: {exc}")
            self.get_logger().warn(f"withholding union mask for source frame {stamp:.3f}: {exc}",
                                   throttle_duration_sec=1.0)
        finally:
            self.busy = False


def main(args=None):
    rclpy.init(args=args)
    node = MultiObjectExclusionNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
