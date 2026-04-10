import base64
import io
import json
import os
import re
import time
import uuid
import shutil
from pathlib import Path

import cv2
import numpy as np
import rclpy
import requests
from PIL import Image
from rclpy.node import Node
from sensor_msgs.msg import Image as RosImage
from std_msgs.msg import Float64, String
from std_srvs.srv import Trigger


def _find_workspace_root() -> Path:
    for parent in Path(__file__).resolve().parents:
        if parent.name == "spot-ros2_ws":
            return parent
    return Path(__file__).resolve().parents[3]


_WORKSPACE_ROOT = _find_workspace_root()
_VLM_INPUT_DIR = _WORKSPACE_ROOT / "tmp" / "vlm_relocalize_requests"


def _prepare_vlm_input_dir() -> Path:
    input_dir = _VLM_INPUT_DIR
    input_dir.mkdir(parents=True, exist_ok=True)
    for child in input_dir.iterdir():
        try:
            if child.is_dir():
                shutil.rmtree(child)
            else:
                child.unlink()
        except Exception:
            pass
    return input_dir


def parse_qwen_response(response_text: str, image_size: tuple = None) -> list:
    """
    Extrai BBs no formato nativo do Qwen <ref>...<box> ou JSON format.
    """
    boxes = []
    w, h = image_size if image_size else (1000, 1000)

    clean_text = response_text.replace("```json", "").replace("```", "").strip()

    def _safe_confidence(raw_conf) -> float:
        if raw_conf is None:
            return 1.0
        if isinstance(raw_conf, (int, float)):
            return float(raw_conf)
        s = str(raw_conf).strip().replace("%", "")
        try:
            val = float(s)
            return (val / 100.0) if val > 1.0 else val
        except Exception:
            return 1.0

    try:
        data = json.loads(clean_text)
        if isinstance(data, list):
            print("DEBUG: Detectado formato JSON.")
            if len(data) == 0:
                print("DEBUG: Lista vazia retornada (objeto não encontrado pelo Qwen).")
                return []
            for item in data:
                if not isinstance(item, dict):
                    continue
                if 'bbox_2d' in item:
                    b = item['bbox_2d']
                    if not isinstance(b, (list, tuple)) or len(b) < 4:
                        print(f"DEBUG: bbox_2d inválido em item JSON: {item}")
                        continue
                    label = item.get('label', 'object')
                    confidence = _safe_confidence(item.get('confidence', 1.0))
                    grasp_points = item.get('grasp_point_2ds', None)
                    if not grasp_points:
                        gp = item.get('grasp_point_2d', None)
                        if gp:
                            grasp_points = [gp]
                    xmin, ymin, xmax, ymax = b[0], b[1], b[2], b[3]
                    grasps_1000 = []
                    if grasp_points:
                        for pt in grasp_points:
                            if not isinstance(pt, (list, tuple)) or len(pt) < 2:
                                continue
                            gx, gy = pt[0], pt[1]
                            grasps_1000.append([int(float(gx)), int(float(gy))])
                    boxes.append({
                        'label': label,
                        'bbox_1000': [int(float(xmin)), int(float(ymin)), int(float(xmax)), int(float(ymax))],
                        'grasps_1000': grasps_1000,
                        'confidence': confidence,
                    })
            if boxes:
                return boxes
    except Exception as e:
        print(f"DEBUG: Falha ao parsear JSON do Qwen: {e}. Trecho: {clean_text[:220]}")

    pattern_strict = r'<ref>(.*?)</ref>\s*<box>\[\[(\d+),\s*(\d+),\s*(\d+),\s*(\d+)\]\]</box>'
    matches = re.findall(pattern_strict, response_text)
    if not matches:
        pattern_simple = r'<ref>(.*?)</ref>\s*<box>\[(\d+),\s*(\d+),\s*(\d+),\s*(\d+)\]</box>'
        matches = re.findall(pattern_simple, response_text)

    matches_loose = []
    if not matches:
        pattern_loose_full = r'<ref>(.*?)</ref>.*?<box.*?[(\[]\s*(\d+)[,\s]+(\d+)[,\s]+(\d+)[,\s]+(\d+)'
        matches_loose = re.findall(pattern_loose_full, response_text, re.DOTALL)

    all_found = []
    if matches:
        for m in matches:
            all_found.append({'label': m[0], 'coords': [int(x) for x in m[1:]], 'source': 'strict'})
    elif matches_loose:
        for m in matches_loose:
            all_found.append({'label': m[0], 'coords': [int(x) for x in m[1:]], 'source': 'loose'})

    if not all_found:
        return []

    print(f"DEBUG: Encontrados {len(all_found)} boxes candidatos via {all_found[0]['source']}.")

    for item in all_found:
        label = item['label']
        c1, c2, c3, c4 = item['coords']
        source = item['source']
        explicit_pixels = any(c > 1000 for c in [c1, c2, c3, c4])
        use_pixels_logic = (source == 'loose') or explicit_pixels
        h_a = c3 - c1
        w_a = c4 - c2
        w_b = c3 - c1
        h_b = c4 - c2
        valid_a = h_a > 0 and w_a > 0
        valid_b = h_b > 0 and w_b > 0
        if valid_a and not valid_b:
            final_coords = [c2, c1, c4, c3]
        elif valid_b and not valid_a:
            final_coords = [c1, c2, c3, c4]
        elif valid_a and valid_b:
            if source == 'strict':
                final_coords = [c2, c1, c4, c3]
            else:
                final_coords = [c1, c2, c3, c4]
        else:
            final_coords = [c1, c2, c3, c4]
        xmin, ymin, xmax, ymax = final_coords
        if use_pixels_logic:
            xmin = (xmin / w) * 1000
            xmax = (xmax / w) * 1000
            ymin = (ymin / h) * 1000
            ymax = (ymax / h) * 1000
        boxes.append({
            'label': label.strip(),
            'bbox_1000': [int(xmin), int(ymin), int(xmax), int(ymax)],
        })

    return boxes


def _build_rotation_matrix(orig_w, orig_h, angle_deg):
    """Reconstruct the forward rotation matrix (same logic as rotate_image_upright)."""
    center = (orig_w / 2.0, orig_h / 2.0)
    M = cv2.getRotationMatrix2D(center, angle_deg, 1.0)
    cos_a = abs(M[0, 0])
    sin_a = abs(M[0, 1])
    rot_w = int(orig_h * sin_a + orig_w * cos_a)
    rot_h = int(orig_h * cos_a + orig_w * sin_a)
    M[0, 2] += (rot_w - orig_w) / 2.0
    M[1, 2] += (rot_h - orig_h) / 2.0
    return M, (rot_w, rot_h)


def _inverse_rotate_coords_1000(bbox_1000, grasps_1000, M_forward, rotated_size, original_size):
    """Map bbox and grasp points from rotated [0-1000] space back to original [0-1000] space."""
    rot_w, rot_h = rotated_size
    orig_w, orig_h = original_size
    M_inv = cv2.invertAffineTransform(M_forward)

    xmin = bbox_1000[0] / 1000.0 * rot_w
    ymin = bbox_1000[1] / 1000.0 * rot_h
    xmax = bbox_1000[2] / 1000.0 * rot_w
    ymax = bbox_1000[3] / 1000.0 * rot_h

    corners = np.array([[xmin, ymin], [xmax, ymin], [xmax, ymax], [xmin, ymax]], dtype=np.float64)
    corners_h = np.hstack([corners, np.ones((4, 1), dtype=np.float64)])
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


def _encode_image_to_base64(image_input) -> str:
    if isinstance(image_input, (str, Path)):
        with open(image_input, "rb") as f:
            return base64.b64encode(f.read()).decode("utf-8")
    if isinstance(image_input, Image.Image):
        buffered = io.BytesIO()
        img = image_input.convert("RGB") if image_input.mode != "RGB" else image_input
        img.save(buffered, format="JPEG")
        return base64.b64encode(buffered.getvalue()).decode("utf-8")
    raise ValueError("Unsupported image input type")


class VlmRelocalizeNode(Node):
    def __init__(self):
        super().__init__("vlm_relocalize_node")
        self.declare_parameter("rgb_topic", "/hand/rgb_upright")
        self.declare_parameter("roll_metadata_topic", "/hand/roll_metadata")
        self.declare_parameter("object_prompt", "wheel valve")
        self.declare_parameter("vlm_url", "http://100.111.174.61:8000")
        self.declare_parameter("request_timeout_sec", 5.0)
        self.declare_parameter("request_max_retries", 1)
        self.declare_parameter("service_name", "/vlm/trigger_relocalize")
        self.declare_parameter("camera_speed_topic", "/hand/camera_speed")
        self.declare_parameter("stability_speed_threshold", 0.1)
        self.declare_parameter("stability_check_enabled", True)
        rgb_topic = self.get_parameter("rgb_topic").value
        roll_metadata_topic = self.get_parameter("roll_metadata_topic").value
        self.object_prompt = self.get_parameter("object_prompt").value
        self.vlm_url = self.get_parameter("vlm_url").value
        self.request_timeout_sec = float(
            max(1.0, self.get_parameter("request_timeout_sec").value)
        )
        self.request_max_retries = int(
            max(0, self.get_parameter("request_max_retries").value)
        )
        service_name = self.get_parameter("service_name").value

        self._latest_rgb_pil = None
        self._latest_stamp_sec = None
        self._latest_stamp_nanosec = None
        self._latest_roll_angle_deg = 0.0
        self._latest_orig_size = None
        self._rgb_cb_count = 0
        self._srv_req_seq = 0
        self._camera_speed = None
        self._vlm_input_dir = _prepare_vlm_input_dir()
        self._rgb_sub = self.create_subscription(RosImage, rgb_topic, self._rgb_cb, 10)
        self._roll_meta_sub = self.create_subscription(String, roll_metadata_topic, self._roll_meta_cb, 10)
        self._cam_speed_sub = self.create_subscription(
            Float64,
            str(self.get_parameter("camera_speed_topic").value),
            self._cam_speed_cb,
            10,
        )
        self._srv = self.create_service(Trigger, service_name, self._handle_relocalize)
        self.get_logger().info(
            f"VLM relocalize service ready at {service_name}, rgb_topic={rgb_topic}, "
            f"roll_metadata={roll_metadata_topic}, input_dir={self._vlm_input_dir}"
        )

    def _dbg_log(self, hypothesis_id: str, location: str, message: str, data: dict):
        # #region agent log
        payload = {
            "sessionId": "eb5d37",
            "id": f"log_{int(time.time() * 1000)}_{uuid.uuid4().hex[:8]}",
            "timestamp": int(time.time() * 1000),
            "runId": "repro-1",
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

    def _roll_meta_cb(self, msg: String):
        try:
            meta = json.loads(msg.data)
            self._latest_roll_angle_deg = float(meta.get("angle_deg", 0.0))
            self._latest_orig_size = (int(meta["orig_w"]), int(meta["orig_h"]))
        except Exception:
            pass

    def _cam_speed_cb(self, msg: Float64):
        self._camera_speed = float(msg.data)

    def _rgb_cb(self, msg: RosImage):
        try:
            frame = np.frombuffer(msg.data, dtype=np.uint8)
            channels = 3
            if msg.encoding == "rgb8":
                frame = frame.reshape((msg.height, msg.width, channels))
                rgb = frame
            else:
                frame = frame.reshape((msg.height, msg.width, channels))
                rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            self._latest_rgb_pil = Image.fromarray(rgb)
            self._latest_stamp_sec = int(msg.header.stamp.sec)
            self._latest_stamp_nanosec = int(msg.header.stamp.nanosec)
            self._rgb_cb_count += 1
            if self._rgb_cb_count <= 5:
                self._dbg_log(
                    "H3",
                    "vlm_relocalize_node.py:_rgb_cb",
                    "rgb_frame_received",
                    {
                        "encoding": str(msg.encoding),
                        "width": int(msg.width),
                        "height": int(msg.height),
                        "step": int(msg.step),
                        "stamp_sec": int(msg.header.stamp.sec),
                        "stamp_nanosec": int(msg.header.stamp.nanosec),
                    },
                )
        except Exception as exc:
            self._latest_rgb_pil = None
            self._dbg_log(
                "H3",
                "vlm_relocalize_node.py:_rgb_cb",
                "rgb_frame_decode_failed",
                {
                    "has_data": bool(getattr(msg, "data", None)),
                    "encoding": str(msg.encoding),
                    "error": str(exc)[:220],
                },
            )

    def _run_vlm(self, image: Image.Image):
        base64_img = _encode_image_to_base64(image)
        # Keep prompt structure aligned with detect_qwen.detect_object().
        prompt = f"""Task: Detect '{self.object_prompt}'.
If the object is clearly visible, return its bounding box, a confidence score (0.0 to 1.0), and exactly 3 distinct 2D grasping points.
The 3 grasp points MUST be physically far apart from each other, representing different valid locations where a robot could grasp the object.
Output STRICTLY in JSON format as a list of dictionaries:
[ {{"label": "{self.object_prompt}", "bbox_2d": [xmin, ymin, xmax, ymax], "grasp_point_2ds": [[x1, y1], [x2, y2], [x3, y3]], "confidence": 0.95}} ]
If the object is NOT present or mostly occluded, return an empty list: []
Ensure bounding box and grasp point coordinates are normalized to [0-1000] scale."""
        payload = {
            "model": "Qwen/Qwen3-VL-8B-Instruct",
            "messages": [
                {
                    "role": "user",
                    "content": [
                        {
                            "type": "image_url",
                            "image_url": {"url": f"data:image/jpeg;base64,{base64_img}"},
                        },
                        {"type": "text", "text": prompt},
                    ],
                }
            ],
            "max_tokens": 512,
            "temperature": 0.01,
        }
        last_exc = None
        attempts = 1 + self.request_max_retries
        for attempt in range(attempts):
            try:
                start = time.time()
                self._dbg_log(
                    "H4",
                    "vlm_relocalize_node.py:_run_vlm",
                    "vlm_http_request_start",
                    {
                        "attempt": attempt + 1,
                        "attempts": attempts,
                        "timeout_sec": float(self.request_timeout_sec),
                        "vlm_url": self.vlm_url,
                        "payload_image_bytes_est": len(base64_img),
                    },
                )
                resp = requests.post(
                    f"{self.vlm_url}/v1/chat/completions",
                    json=payload,
                    timeout=self.request_timeout_sec,
                )
                latency_ms = int((time.time() - start) * 1000)
                resp.raise_for_status()
                content = resp.json()["choices"][0]["message"]["content"]
                self._dbg_log(
                    "H4",
                    "vlm_relocalize_node.py:_run_vlm",
                    "vlm_response_received",
                    {
                        "attempt": attempt + 1,
                        "latency_ms": latency_ms,
                        "content_preview": content[:120],
                        "content_len": len(content),
                    },
                )
                return content, latency_ms
            except Exception as exc:
                last_exc = exc
                self._dbg_log(
                    "H4",
                    "vlm_relocalize_node.py:_run_vlm",
                    "vlm_http_request_failed",
                    {
                        "attempt": attempt + 1,
                        "attempts": attempts,
                        "error": str(exc)[:240],
                    },
                )
                if attempt + 1 < attempts:
                    time.sleep(0.1)
        raise RuntimeError(f"VLM request failed: {last_exc}")

    def _save_vlm_input_image(self, image: Image.Image, req_id: int, stamp_sec: int, stamp_ns: int):
        """Persist the exact image sent to the VLM for debugging and replay."""
        try:
            filename = f"vlm_input_{req_id:04d}_{stamp_sec}_{stamp_ns:09d}.png"
            image_path = self._vlm_input_dir / filename
            image.save(image_path)
            return image_path
        except Exception as exc:
            self.get_logger().warn(f"[VLM] failed to save input image: {exc}")
            return None

    def _handle_relocalize(self, _request, response):
        self.get_logger().info("[VLM] request recebido em /vlm/trigger_relocalize")
        if self._latest_rgb_pil is None:
            response.success = False
            response.message = "No RGB frame available yet"
            self.get_logger().warn("[VLM] falha: sem frame RGB para processar")
            self._dbg_log(
                "H15",
                "vlm_relocalize_node.py:_handle_relocalize",
                "relocalize_exit",
                {"success": False, "reason": "no_rgb_frame"},
            )
            return response
        if self.get_parameter("stability_check_enabled").value:
            threshold = float(self.get_parameter("stability_speed_threshold").value)
            if self._camera_speed is not None and self._camera_speed > threshold:
                response.success = False
                response.message = f"camera_moving: speed={self._camera_speed:.3f} m/s > threshold={threshold}"
                self.get_logger().info(
                    f"[VLM] request rejeitado: câmera em movimento ({self._camera_speed:.3f} m/s) — coordinator vai retentar"
                )
                return response
        try:
            self._srv_req_seq += 1
            req_id = int(self._srv_req_seq)
            stamp_sec = int(self._latest_stamp_sec or 0)
            stamp_ns = int(self._latest_stamp_nanosec or 0)
            frame_size = list(self._latest_rgb_pil.size) if self._latest_rgb_pil is not None else None
            self.get_logger().info(
                f"[VLM] req_id={req_id} iniciando com frame stamp={stamp_sec}.{stamp_ns:09d}"
            )
            self._dbg_log(
                "H15",
                "vlm_relocalize_node.py:_handle_relocalize",
                "relocalize_start",
                {
                    "req_id": req_id,
                    "latest_stamp_sec": stamp_sec,
                    "latest_stamp_nanosec": stamp_ns,
                    "frame_size": frame_size,
                    "rgb_cb_count": int(self._rgb_cb_count),
                },
            )
            img = self._latest_rgb_pil.copy()
            w, h = img.size
            saved_path = self._save_vlm_input_image(img, req_id, stamp_sec, stamp_ns)
            if saved_path is not None:
                self.get_logger().info(f"[VLM] input image saved at {saved_path}")
            self._dbg_log(
                "H15",
                "vlm_relocalize_node.py:_handle_relocalize",
                "relocalize_image_ready",
                {
                    "req_id": req_id,
                    "width": int(w),
                    "height": int(h),
                    "saved_path": str(saved_path) if saved_path is not None else None,
                },
            )
            text, latency_ms = self._run_vlm(img)
            # #region agent log
            schema_hint = {}
            try:
                parsed_raw = json.loads(text.replace("```json", "").replace("```", "").strip())
                if isinstance(parsed_raw, list) and parsed_raw:
                    first = parsed_raw[0] if isinstance(parsed_raw[0], dict) else {}
                    bbox = first.get("bbox_2d")
                    grasps = first.get("grasp_point_2ds")
                    schema_hint = {
                        "raw_list_len": len(parsed_raw),
                        "first_is_dict": isinstance(first, dict),
                        "bbox_type": type(bbox).__name__ if bbox is not None else "none",
                        "bbox_len": (len(bbox) if isinstance(bbox, list) else -1),
                        "bbox_first_item_type": (type(bbox[0]).__name__ if isinstance(bbox, list) and bbox else "none"),
                        "grasps_type": type(grasps).__name__ if grasps is not None else "none",
                        "text_preview": text[:180],
                    }
                else:
                    schema_hint = {
                        "raw_type": type(parsed_raw).__name__,
                        "raw_list_len": (len(parsed_raw) if isinstance(parsed_raw, list) else -1),
                        "text_preview": text[:180],
                    }
            except Exception as exc_schema:
                schema_hint = {"json_parse_error": str(exc_schema)[:180], "text_preview": text[:180]}
            self._dbg_log(
                "H2",
                "vlm_relocalize_node.py:_handle_relocalize",
                "vlm_raw_schema_hint",
                schema_hint,
            )
            # #endregion
            self.get_logger().info(f"[VLM] raw response (req_id={req_id}): {text[:300]}")
            boxes = parse_qwen_response(text, image_size=(w, h))
            if boxes and self._latest_roll_angle_deg != 0.0 and self._latest_orig_size is not None:
                M_fwd, rot_size = _build_rotation_matrix(
                    self._latest_orig_size[0], self._latest_orig_size[1],
                    self._latest_roll_angle_deg,
                )
                for box in boxes:
                    if "bbox_1000" in box:
                        box["bbox_1000"], box["grasps_1000"] = _inverse_rotate_coords_1000(
                            box["bbox_1000"],
                            box.get("grasps_1000", []),
                            M_fwd,
                            rot_size,
                            self._latest_orig_size,
                        )
            self._dbg_log(
                "H15",
                "vlm_relocalize_node.py:_handle_relocalize",
                "relocalize_parse_result",
                {
                    "req_id": req_id,
                    "boxes_count": len(boxes),
                    "latency_ms": latency_ms,
                    "latest_stamp_sec": stamp_sec,
                    "latest_stamp_nanosec": stamp_ns,
                },
            )
            if not boxes:
                response.success = False
                response.message = "[]"
                self._dbg_log(
                    "H15",
                    "vlm_relocalize_node.py:_handle_relocalize",
                    "relocalize_no_boxes",
                    {"req_id": req_id, "latency_ms": latency_ms, "text_preview": text[:180]},
                )
                self._dbg_log(
                    "H15",
                    "vlm_relocalize_node.py:_handle_relocalize",
                    "relocalize_exit",
                    {"req_id": req_id, "success": False, "reason": "no_boxes"},
                )
                return response
            seed = boxes[0]
            seed["frame_stamp_sec"] = stamp_sec + stamp_ns * 1e-9
            seed["frame_stamp_nanosec"] = stamp_ns
            seed["vlm_latency_ms"] = latency_ms
            response.success = True
            response.message = json.dumps(seed, ensure_ascii=True)
            self._dbg_log(
                "H15",
                "vlm_relocalize_node.py:_handle_relocalize",
                "relocalize_exit",
                {
                    "req_id": req_id,
                    "success": True,
                    "message_len": len(response.message),
                    "latency_ms": latency_ms,
                },
            )
            return response
        except Exception as exc:
            response.success = False
            response.message = str(exc)
            self.get_logger().warn(f"[VLM] req falhou: {str(exc)[:220]}")
            self._dbg_log(
                "H15",
                "vlm_relocalize_node.py:_handle_relocalize",
                "relocalize_exit",
                {"success": False, "reason": "exception", "error": str(exc)[:220]},
            )
            return response


def main(args=None):
    rclpy.init(args=args)
    node = VlmRelocalizeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

