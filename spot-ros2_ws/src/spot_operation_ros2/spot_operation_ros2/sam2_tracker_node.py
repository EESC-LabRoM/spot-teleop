#!/usr/bin/env python3
import collections
import json
import os
import site
import shutil
import threading
import time
import uuid
from collections import OrderedDict
from pathlib import Path

import cv2
import message_filters
import numpy as np
import rclpy
from PIL import Image, ImageDraw, ImageFont
from cv_bridge import CvBridge
from geometry_msgs.msg import PointStamped
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image as RosImage
from std_msgs.msg import Float64, String

# Inject venv so torch/ultralytics are found via ros2 run
_VENV_SITE = (
    "/home/spot-teleop/spot-ros2_ws/src/spot_operation_ros2"
    "/venv_valve_detection/lib/python3.10/site-packages"
)
if os.path.isdir(_VENV_SITE):
    site.addsitedir(_VENV_SITE)

try:
    import torch
    from ultralytics import SAM
    from ultralytics.models.sam import SAM2VideoPredictor
    SAM2_AVAILABLE = True
except ImportError:
    SAM2_AVAILABLE = False
    SAM2VideoPredictor = None  # type: ignore

# ---------------------------------------------------------------------------
# SAM2 model loading (singletons)
# ---------------------------------------------------------------------------

SAM2_MODEL_NAME = "sam2.1_t.pt"
_sam_model = None       # singleton SAM image predictor
_sam_video_model = None  # singleton SAM video predictor


def _find_workspace_root() -> Path:
    for parent in Path(__file__).resolve().parents:
        if parent.name == "spot-ros2_ws":
            return parent
    return Path(__file__).resolve().parents[3]


_WORKSPACE_ROOT = _find_workspace_root()
_SNAPSHOT_TEMP_DIR = _WORKSPACE_ROOT / "tmp" / "sam2_tracker_snapshots"


def _prepare_snapshot_temp_dir() -> Path:
    snapshot_dir = _SNAPSHOT_TEMP_DIR
    snapshot_dir.mkdir(parents=True, exist_ok=True)
    for child in snapshot_dir.iterdir():
        try:
            if child.is_dir():
                shutil.rmtree(child)
            else:
                child.unlink()
        except Exception:
            pass
    return snapshot_dir

if SAM2_AVAILABLE:
    class SAM2ROSVideoPredictor(SAM2VideoPredictor):
        """SAM2VideoPredictor adaptado para fluxo ROS (numpy frame a frame)."""

        def __init__(self, cfg=None, overrides=None, _callbacks=None):
            from ultralytics.utils import DEFAULT_CFG
            if cfg is None:
                cfg = DEFAULT_CFG
            super().__init__(cfg, overrides, _callbacks)
            self.callbacks["on_predict_start"] = [
                self._init_state_ros if cb is SAM2VideoPredictor.init_state else cb
                for cb in self.callbacks["on_predict_start"]
            ]
            self._ros_frame_idx = 0

        @staticmethod
        def _init_state_ros(predictor):
            if len(predictor.inference_state) > 0:
                return
            ds = predictor.dataset
            num_frames = getattr(ds, "frames", 10**9)
            predictor.inference_state = {
                "num_frames": num_frames,
                "point_inputs_per_obj": {},
                "mask_inputs_per_obj": {},
                "constants": {},
                "obj_id_to_idx": OrderedDict(),
                "obj_idx_to_id": OrderedDict(),
                "obj_ids": [],
                "output_dict": {"cond_frame_outputs": {}, "non_cond_frame_outputs": {}},
                "output_dict_per_obj": {},
                "temp_output_dict_per_obj": {},
                "consolidated_frame_inds": {
                    "cond_frame_outputs": set(),
                    "non_cond_frame_outputs": set(),
                },
                "tracking_has_started": False,
                "frames_already_tracked": [],
            }

        def inference(self, im, bboxes=None, points=None, labels=None, masks=None):
            if self.dataset is not None:
                self.dataset.frame = self._ros_frame_idx
            try:
                return super().inference(im, bboxes=bboxes, points=points, labels=labels, masks=masks)
            finally:
                self._ros_frame_idx += 1


def _trim_sam2_memory(predictor, keep_frames: int = 6):
    """Remove old frames from SAM2 inference_state, keeping only the last `keep_frames`.

    SAM2 only attends to ~6 past frames, so older entries are pure GPU waste.
    This avoids a full reset+reseed cycle — the predictor keeps tracking seamlessly.
    """
    state = getattr(predictor, 'inference_state', None)
    if not state:
        return
    for bucket in ('cond_frame_outputs', 'non_cond_frame_outputs'):
        od = state.get('output_dict', {}).get(bucket)
        ci = state.get('consolidated_frame_inds', {}).get(bucket)
        if not od:
            continue
        sorted_keys = sorted(od.keys())
        to_remove = sorted_keys[:-keep_frames] if len(sorted_keys) > keep_frames else []
        for k in to_remove:
            del od[k]
            if ci is not None:
                ci.discard(k)
    if 'frames_already_tracked' in state:
        tracked = state['frames_already_tracked']
        if len(tracked) > keep_frames:
            state['frames_already_tracked'] = tracked[-keep_frames:]


def _load_sam_model():
    global _sam_model
    if not SAM2_AVAILABLE:
        return None
    if _sam_model is None:
        try:
            _sam_model = SAM(SAM2_MODEL_NAME)
        except Exception as e:
            print(f"Erro ao carregar SAM2: {e}")
            return None
    return _sam_model


def _load_sam_video_model():
    global _sam_video_model
    if not SAM2_AVAILABLE:
        return None
    if _sam_video_model is None:
        try:
            torch.backends.cuda.matmul.allow_tf32 = True
            torch.backends.cudnn.allow_tf32 = True
            _sam_video_model = SAM2ROSVideoPredictor(
                overrides={
                    "conf": 0.01,
                    "task": "segment",
                    "mode": "predict",
                    "imgsz": 1024,
                    "model": SAM2_MODEL_NAME,
                    "save": False,
                    "verbose": False,
                }
            )
        except Exception as e:
            print(f"Erro ao carregar SAM2 tracker: {e}")
            return None
    return _sam_video_model

# ---------------------------------------------------------------------------
# Mask utilities
# ---------------------------------------------------------------------------


def _best_mask_from_results(results):
    if not results:
        return None, 0.0
    r = results[0]
    if r.masks is None or r.masks.data is None or len(r.masks.data) == 0:
        return None, 0.0
    masks_np = (r.masks.data > 0).to(dtype=torch.uint8).cpu().numpy()
    if r.boxes is not None and r.boxes.conf is not None and len(r.boxes.conf) == len(masks_np):
        scores_np = r.boxes.conf.cpu().numpy()
    else:
        scores_np = np.ones(len(masks_np), dtype=np.float32)
    best_idx = int(np.argmax(scores_np))
    return masks_np[best_idx], float(scores_np[best_idx])


def _sam_prompt_masks(model, img_pil, point_xy, multimask_output=True):
    img_np = np.array(img_pil.convert("RGB"))
    results = model(img_np, points=[point_xy], labels=[1], conf=0.0, verbose=False)
    if not results:
        return np.empty((0, img_np.shape[0], img_np.shape[1]), dtype=np.uint8), np.array([])
    r = results[0]
    if r.masks is None or r.masks.data is None or len(r.masks.data) == 0:
        return np.empty((0, img_np.shape[0], img_np.shape[1]), dtype=np.uint8), np.array([])
    masks_np = (r.masks.data > 0).to(dtype=torch.uint8).cpu().numpy()
    if r.boxes is not None and r.boxes.conf is not None and len(r.boxes.conf) == len(masks_np):
        scores_np = r.boxes.conf.cpu().numpy()
    else:
        scores_np = np.ones(len(masks_np), dtype=np.float32)
    return masks_np, scores_np


def select_best_mask_by_bbox_iou(masks, bbox, image_size):
    x1, y1, x2, y2 = bbox
    w, h = image_size
    bbox_region = np.zeros((h, w), dtype=bool)
    bbox_region[y1:y2, x1:x2] = True
    bbox_area = int(bbox_region.sum())
    best_idx, best_iou = 0, -1.0
    for i in range(len(masks)):
        m = masks[i]
        if hasattr(m, 'cpu'):
            m = m.cpu().numpy()
        if m.ndim > 2:
            m = m.squeeze()
        if m.shape[0] != h or m.shape[1] != w:
            m = np.array(
                Image.fromarray((m * 255).astype(np.uint8)).resize((w, h), Image.NEAREST)
            ) > 127
        else:
            m = m > 0.5
        intersection = int((m & bbox_region).sum())
        union = int((m | bbox_region).sum())
        iou = intersection / union if union > 0 else 0.0
        if iou > best_iou:
            best_iou = iou
            best_idx = i
    return best_idx


def create_crop_mosaic(image_pil, masks, bbox, output_path="debug_mosaic.jpg"):
    w, h = image_pil.size
    x1, y1, x2, y2 = bbox
    bw, bh = x2 - x1, y2 - y1
    margin_w, margin_h = int(bw * 0.1), int(bh * 0.1)
    cx1, cy1 = max(0, x1 - margin_w), max(0, y1 - margin_h)
    cx2, cy2 = min(w, x2 + margin_w), min(h, y2 + margin_h)
    crop_w, crop_h = cx2 - cx1, cy2 - cy1
    base_crop = image_pil.crop((cx1, cy1, cx2, cy2))
    mosaic = Image.new("RGB", (crop_w * 3, crop_h))
    draw = ImageDraw.Draw(mosaic)
    try:
        font = ImageFont.truetype("DejaVuSans.ttf", 40)
    except Exception:
        font = ImageFont.load_default()
    for i in range(min(3, len(masks))):
        m = masks[i]
        if hasattr(m, 'cpu'):
            m = m.cpu().numpy()
        if m.ndim > 2:
            m = m.squeeze()
        mask_pil = Image.fromarray((m * 255).astype(np.uint8))
        if mask_pil.size != image_pil.size:
            mask_pil = mask_pil.resize(image_pil.size, Image.NEAREST)
        mask_crop = mask_pil.crop((cx1, cy1, cx2, cy2))
        green = Image.new("RGBA", base_crop.size, (0, 255, 0, 100))
        comp = base_crop.convert("RGBA").copy()
        comp.paste(green, (0, 0), mask_crop)
        mosaic.paste(comp.convert("RGB"), (i * crop_w, 0))
        draw.text((i * crop_w + 10, 10), str(i + 1), fill="white", font=font)
        draw.text((i * crop_w + 12, 12), str(i + 1), fill="black", font=font)
    mosaic.save(output_path)
    return output_path


def draw_result(image_input, boxes: list, output_path: str):
    if isinstance(image_input, (str, Path)):
        img = Image.open(image_input)
    elif isinstance(image_input, Image.Image):
        img = image_input.copy()
    else:
        raise ValueError("Invalid image input for drawing")
    draw = ImageDraw.Draw(img)
    w, h = img.size
    try:
        font = ImageFont.truetype("DejaVuSans.ttf", 24)
    except IOError:
        font = ImageFont.load_default()
    for box_data in boxes:
        label = box_data['label']
        b1000 = box_data['bbox_1000']
        x1 = max(0, int((b1000[0] / 1000.0) * w))
        y1 = max(0, int((b1000[1] / 1000.0) * h))
        x2 = min(w - 1, int((b1000[2] / 1000.0) * w))
        y2 = min(h - 1, int((b1000[3] / 1000.0) * h))
        color = '#00FF00'
        for i in range(4):
            draw.rectangle([x1 - i, y1 - i, x2 + i, y2 + i], outline=color)
        if hasattr(draw, 'textbbox'):
            tb = draw.textbbox((0, 0), label, font=font)
            text_w, text_h = tb[2] - tb[0], tb[3] - tb[1]
        else:
            text_w, text_h = draw.textsize(label, font=font)
        draw.rectangle([x1, y1 - text_h - 4, x1 + text_w + 4, y1], fill=color)
        draw.text((x1 + 2, y1 - text_h - 2), label, fill='black', font=font)
        for g1000 in box_data.get('grasps_1000', []):
            gx = max(0, min(w - 1, int((g1000[0] / 1000.0) * w)))
            gy = max(0, min(h - 1, int((g1000[1] / 1000.0) * h)))
            r = 5
            draw.ellipse([gx - r, gy - r, gx + r, gy + r], fill='red')
    if img.mode == 'RGBA':
        img = img.convert('RGB')
    img.save(output_path)


# ---------------------------------------------------------------------------
# Node
# ---------------------------------------------------------------------------

class Sam2TrackerNode(Node):
    """2D-only tracker node: seed -> SAM2 tracking -> filtered grasp uv."""

    def __init__(self):
        super().__init__('sam2_tracker_node')

        # Parameters
        self.declare_parameter('rgb_topic', '/hand/rgb')
        self.declare_parameter('depth_topic', '/hand/depth')
        self.declare_parameter('visualize', True)
        self.declare_parameter('tracking_window_name', 'SAM2 Live Tracking')
        self.declare_parameter('segmentation_mask_topic', '/hand/segmentation_mask')
        self.declare_parameter('publish_segmentation_mask', True)
        self.declare_parameter('active_tracking_interval', 0.2)
        self.declare_parameter('tracking_lost_confirm_frames', 5)
        self.declare_parameter('seed_command_topic', '/perception/seed_command')
        self.declare_parameter('tracking_state_topic', '/tracking_state')
        self.declare_parameter('tracking_3d_topic', '/tracking_3d_point')
        self.declare_parameter('depth_info_topic', '/hand/camera_info')
        self.declare_parameter('secondary_cameras', '')
        self.declare_parameter('secondary_rgb_topic_pattern', '/{cam}/rgb')
        self.declare_parameter('secondary_mask_topic_pattern', '/{cam}/segmentation_mask')
        self.declare_parameter('secondary_fov_timeout_sec', 3.0)
        self.declare_parameter('secondary_max_centroid_dist_px', 120.0)
        self.declare_parameter('secondary_fov_enter_count', 5)  # consecutive seeds to arm init
        self.declare_parameter('secondary_iou_radius_px', 80.0)   # expected object radius for IoU check
        self.declare_parameter('secondary_iou_min_overlap', 0.30)  # min fraction of mask inside circle
        self.declare_parameter('secondary_warm_promote_frames', 3)      # frames of valid tracking → COLD→WARM
        self.declare_parameter('secondary_warm_fail_max', 3)            # failed warm re-prompts → DEGRADED
        self.declare_parameter('secondary_iou_min_overlap_warm', 0.15)  # relaxed IoU for warm re-prompts
        self.declare_parameter('sam2_memory_reset_interval', 200)  # frames between memory resets
        self.declare_parameter('camera_speed_topic', '/hand/camera_speed')
        self.declare_parameter('hand_reinit_speed_gate_m_s', 0.08)
        self.declare_parameter('hand_reinit_speed_gate_timeout_s', 1.5)
        self.declare_parameter('hand_iou_radius_px', 100.0)
        self.declare_parameter('hand_iou_min_overlap', 0.25)
        self.declare_parameter('hand_max_centroid_dist_px', 150.0)

        self.visualize = self.get_parameter('visualize').value
        self.tracking_window_name = self.get_parameter('tracking_window_name').value
        self.publish_segmentation_mask = self.get_parameter('publish_segmentation_mask').value
        self.active_tracking_interval = self.get_parameter('active_tracking_interval').value
        self.tracking_lost_confirm_frames = int(
            max(1, self.get_parameter('tracking_lost_confirm_frames').value)
        )

        rgb_topic = self.get_parameter('rgb_topic').value
        depth_topic = self.get_parameter('depth_topic').value
        mask_topic = self.get_parameter('segmentation_mask_topic').value
        seed_topic = self.get_parameter('seed_command_topic').value
        tracking_state_topic = self.get_parameter('tracking_state_topic').value
        tracking_3d_topic = self.get_parameter('tracking_3d_topic').value
        tracking_point_topic = tracking_3d_topic if tracking_3d_topic else '/tracking_3d_point'
        depth_info_topic = self.get_parameter('depth_info_topic').value
        secondary_rgb_topic_pattern = str(self.get_parameter('secondary_rgb_topic_pattern').value)
        secondary_mask_topic_pattern = str(self.get_parameter('secondary_mask_topic_pattern').value)

        # State
        self.bridge = CvBridge()
        self.latest_rgb = None
        self.latest_depth = None
        self.latest_depth_header = None
        self._latest_rgb_header = None
        self._last_mask_np = None   # last known mask; None = publish zeros (all background)
        self._last_mask_shape = None  # (h, w) cached from first frame
        self.camera_frame_id = None
        self.camera_intrinsics = None
        self.new_frame_available = False
        self.detection_running = False
        self.initial_detection_done = True
        # Ring buffer of recent (stamp_sec, rgb_pil, depth_np, header) for seed frame lookup
        self._frame_buffer = collections.deque(maxlen=60)

        # Pending seed pixel from tf_projection: deferred until the frame buffer catches up
        # to T_now (which may lag due to single-threaded spin blocking during _apply_seed_command)
        self._pending_seed_pixel_uv = None    # (u, v) reprojected pixel
        self._pending_seed_pixel_tnow = None  # sim_time (float) when TF was computed
        self._pending_seed_pixel_clock_t = 0.0  # node clock time (sec) when seed_pixel was queued

        # Tracking state
        self.video_predictor = None
        self.tracking_active = False
        self.tracking_frame_count = 0
        self.last_tracking_score = 0.0
        self._tracking_lost_streak = 0
        self._last_track_time = 0.0

        # Sam2TrackerNode-specific state
        self._pending_seed = None
        self._seed_cb_count = 0
        self._hand_ui_calls = 0
        self._dbg_timer_count = 0

        # Hand display state (updated throughout pipeline, displayed every frame)
        self._hand_disp_state = 'IDLE'       # IDLE | RELOCALIZING | TRACKING | LOST
        self._hand_disp_mask = None
        self._hand_disp_score = 0.0
        self._hand_disp_centroid = None
        self._hand_disp_label = ''           # detection label from VLM
        self._hand_disp_conf = 0.0           # detection confidence
        self._hand_disp_bbox = None          # [x1,y1,x2,y2] in pixels

        # Snapshot state for relocalization
        self._snapshot_rgb = None
        self._snapshot_depth = None
        self._snapshot_header = None
        self._snapshot_taken = False
        self._snapshot_run_idx = 0
        self._snapshot_temp_dir = _prepare_snapshot_temp_dir()

        # GPU lock shared between hand and secondary cameras
        self._gpu_lock = threading.Lock()

        # Secondary cameras
        secondary_cameras_str = str(self.get_parameter('secondary_cameras').value)
        self._secondary_cameras = [c.strip() for c in secondary_cameras_str.split(',') if c.strip()]
        self._secondary_cam_state = {}
        self._secondary_mask_pubs = {}
        self._secondary_fov_timeout_sec = float(self.get_parameter('secondary_fov_timeout_sec').value)
        self._secondary_max_centroid_dist_px = float(self.get_parameter('secondary_max_centroid_dist_px').value)
        self._secondary_fov_enter_count = int(max(1, self.get_parameter('secondary_fov_enter_count').value))
        self._sam2_memory_reset_interval = int(max(1, self.get_parameter('sam2_memory_reset_interval').value))
        self._secondary_iou_radius_px = float(self.get_parameter('secondary_iou_radius_px').value)
        self._secondary_iou_min_overlap = float(self.get_parameter('secondary_iou_min_overlap').value)
        self._secondary_warm_promote_frames = int(max(1, self.get_parameter('secondary_warm_promote_frames').value))
        self._secondary_warm_fail_max = int(max(1, self.get_parameter('secondary_warm_fail_max').value))
        self._secondary_iou_min_overlap_warm = float(self.get_parameter('secondary_iou_min_overlap_warm').value)
        self._hand_reinit_speed_gate_m_s = float(self.get_parameter('hand_reinit_speed_gate_m_s').value)
        self._hand_reinit_speed_gate_timeout_s = float(self.get_parameter('hand_reinit_speed_gate_timeout_s').value)
        self._hand_iou_radius_px = float(self.get_parameter('hand_iou_radius_px').value)
        self._hand_iou_min_overlap = float(self.get_parameter('hand_iou_min_overlap').value)
        self._hand_max_centroid_dist_px = float(self.get_parameter('hand_max_centroid_dist_px').value)

        # Callback groups
        self.img_cb_group = ReentrantCallbackGroup()
        self.inference_cb_group = MutuallyExclusiveCallbackGroup()

        # Publishers
        self.mask_pub = self.create_publisher(RosImage, mask_topic, 10)
        self._tracking_state_pub = self.create_publisher(String, tracking_state_topic, 10)
        self._tracking_2d_pub = self.create_publisher(PointStamped, tracking_point_topic, 10)
        self._seed_3d_pub = self.create_publisher(PointStamped, "/tracking/seed_3d", 10)

        # RGB+depth sync via ApproximateTimeSynchronizer
        self.color_sub = message_filters.Subscriber(self, RosImage, rgb_topic)
        self.depth_sub = message_filters.Subscriber(self, RosImage, depth_topic)
        self.sync = message_filters.ApproximateTimeSynchronizer(
            [self.color_sub, self.depth_sub], queue_size=10, slop=0.1
        )
        self.sync.registerCallback(self._synced_image_cb)

        self._seed_sub = self.create_subscription(String, seed_topic, self._seed_command_cb, 10)
        self._cam_info_sub = self.create_subscription(CameraInfo, depth_info_topic, self._camera_info_cb, 10)
        self._hand_cam_speed = None
        self.create_subscription(
            Float64,
            str(self.get_parameter('camera_speed_topic').value),
            self._cam_speed_cb,
            10,
        )
        self._coord_state_sub = self.create_subscription(
            String, "/coordinator/state", self._coordinator_state_cb, 10
        )
        self._seed_pixel_sub = self.create_subscription(
            PointStamped, "/tracking/seed_pixel", self._seed_pixel_cb, 10
        )
        # Geometry depth fallback: vision-frame object position reprojected to hand cam by tf_projection
        self._geometry_cam_pt: tuple = None  # (x, y, z) in hand_cam frame, TF-derived (no depth sensor)
        self.create_subscription(
            PointStamped, "/tracking/geometry_3d_in_cam", self._geometry_cam_pt_cb, 10
        )

        # Register secondary cameras
        for cam in self._secondary_cameras:
            secondary_rgb_topic = secondary_rgb_topic_pattern.replace('{cam}', cam)
            self._secondary_cam_state[cam] = {
                'video_predictor': None,
                'tracking_initialized': False,
                'needs_reinit': False,
                'init_uv': None,
                'init_uv_stamp': None,  # sim-time (float) of the TF lookup that produced init_uv
                'last_seed_time': 0.0,
                'consecutive_seed_count': 0,  # hysteresis: counts consecutive seeds to arm init
                'cooldown_until': 0.0,  # wall-clock time until which continuous re-arming is blocked
                'latest_rgb': None,
                'latest_rgb_header': None,
                'new_frame_available': False,
                'tracking_frame_count': 0,
                'last_score': 0.0,
                'lost_streak': 0,
                # display cache — updated each tracking step so stale frames still show last state
                '_disp_state': 'OUT_OF_FOV',
                '_disp_mask': None,
                '_disp_score': 0.0,
                '_disp_centroid': None,
                '_last_mask_np': None,
                '_last_mask_shape': None,
                'lifecycle': 'COLD',      # 'COLD' | 'WARM' | 'DEGRADED'
                'warm_tracked_frames': 0, # successful frames since current init (for COLD→WARM promotion)
                'warm_fail_streak': 0,    # consecutive warm re-prompt failures (→ DEGRADED)
            }
            self.create_subscription(
                RosImage, secondary_rgb_topic,
                lambda msg, c=cam: self._secondary_rgb_cb(msg, c), 10
            )
            self.create_subscription(
                PointStamped, f'/{cam}/tracking/seed_pixel',
                lambda msg, c=cam: self._secondary_seed_pixel_cb(msg, c), 10
            )
            secondary_mask_topic = secondary_mask_topic_pattern.replace('{cam}', cam)
            self._secondary_mask_pubs[cam] = self.create_publisher(
                RosImage, secondary_mask_topic, 10
            )
        if self._secondary_cameras:
            self.get_logger().info(
                f"Secondary cameras registered: {self._secondary_cameras}, rgb_pattern={secondary_rgb_topic_pattern}"
            )

        self.tracking_timer = self.create_timer(
            0.1, self._tracking_timer_cb, callback_group=self.inference_cb_group
        )

        self.get_logger().info(
            f"Sam2TrackerNode ready. rgb={rgb_topic}, depth={depth_topic}, "
            f"seed={seed_topic}, tracking3d={tracking_point_topic}, snapshot_dir={self._snapshot_temp_dir}"
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

    def _cam_speed_cb(self, msg: Float64):
        self._hand_cam_speed = float(msg.data)

    def _coordinator_state_cb(self, msg: String):
        state = msg.data.strip().upper()
        if state == "RELOCALIZING" and not self._snapshot_taken:
            # A new VLM request is starting. Any pending seed pixel from a prior VLM
            # request is now obsolete. Clear it so we don't initialize on a stale seed.
            self._pending_seed_pixel_uv = None
            self._pending_seed_pixel_tnow = None

            if self.latest_rgb is not None and self.latest_depth is not None:
                self._snapshot_rgb = self.latest_rgb.copy()
                self._snapshot_depth = self.latest_depth.copy()
                self._snapshot_header = self.latest_depth_header
                self._snapshot_taken = True
                snap_stamp = self._snapshot_header.stamp
                snap_sec = snap_stamp.sec + snap_stamp.nanosec * 1e-9
                self._save_snapshot_to_temp_dir()
                self.get_logger().info(
                    f"[TRACKER] Snapshot taken for relocalization stamp={snap_sec:.3f} frame={self._snapshot_header.frame_id}"
                )
            # Pause secondary cameras that are NOT actively tracking.
            # Cameras already tracking are left alone — they'll be reseeded when VLM
            # succeeds and publishes force_reinit=True.  Resetting an active tracker
            # on every failed VLM attempt causes an infinite pause→reinit loop.
            for cam, st in self._secondary_cam_state.items():
                if st['tracking_initialized']:
                    continue  # already tracking — don't disrupt
                if st['needs_reinit']:
                    self.get_logger().info(f"[TRACKER] Pausing secondary cam {cam} during relocalization")
                st['needs_reinit'] = False
                st['consecutive_seed_count'] = 0
                st['last_seed_time'] = 0.0
                st['_disp_state'] = 'OUT_OF_FOV'
                st['_disp_mask'] = None
                st['_disp_centroid'] = None
            if not self.tracking_active:
                self._hand_disp_state = 'RELOCALIZING'
        elif state != "RELOCALIZING":
            self._snapshot_taken = False
            if not self.tracking_active and self._hand_disp_state == 'RELOCALIZING':
                self._hand_disp_state = 'IDLE'

    def _save_snapshot_to_temp_dir(self):
        """Persist the current relocalization snapshot to a fixed temp directory."""
        if self._snapshot_rgb is None or self._snapshot_depth is None or self._snapshot_header is None:
            return

        self._snapshot_run_idx += 1
        run_tag = f"snapshot_{self._snapshot_run_idx:04d}"
        rgb_path = self._snapshot_temp_dir / f"{run_tag}_rgb.png"
        depth_path = self._snapshot_temp_dir / f"{run_tag}_depth.npy"
        meta_path = self._snapshot_temp_dir / f"{run_tag}_meta.json"

        try:
            self._snapshot_rgb.save(rgb_path)
            np.save(depth_path, self._snapshot_depth)
            header_stamp = self._snapshot_header.stamp
            metadata = {
                "snapshot_index": int(self._snapshot_run_idx),
                "stamp_sec": float(header_stamp.sec + header_stamp.nanosec * 1e-9),
                "stamp_nanosec": int(header_stamp.nanosec),
                "frame_id": self._snapshot_header.frame_id,
                "rgb_size": [int(self._snapshot_rgb.width), int(self._snapshot_rgb.height)],
                "depth_shape": [int(v) for v in self._snapshot_depth.shape[:2]],
                "rgb_file": str(rgb_path),
                "depth_file": str(depth_path),
            }
            meta_path.write_text(json.dumps(metadata, indent=2, ensure_ascii=False), encoding="utf-8")
            self.get_logger().info(f"[TRACKER] Snapshot saved in temp dir: {self._snapshot_temp_dir}")
        except Exception as exc:
            self.get_logger().warn(f"[TRACKER] Failed to save snapshot to temp dir: {exc}")

    def _save_backproject_debug_image(self, img_pil, depth_map, pixel_uv, req_tag: str):
        """Save a visual debug artifact for the back-project step.

        The artifact contains the RGB frame on the left and a colorized depth
        visualization on the right, both annotated with the sampled pixel.
        """
        if img_pil is None or depth_map is None or pixel_uv is None:
            return None

        try:
            rgb_np = np.array(img_pil.convert("RGB"))
            h, w = rgb_np.shape[:2]

            depth = np.asarray(depth_map, dtype=np.float32)
            u, v = int(pixel_uv[0]), int(pixel_uv[1])
            u = max(0, min(w - 1, u))
            v = max(0, min(h - 1, v))

            # Sample depth value at selected pixel (in metres)
            depth_val_m = float(depth[v, u]) if np.isfinite(depth[v, u]) and depth[v, u] > 0 else None

            finite = depth[np.isfinite(depth) & (depth > 0)]
            if finite.size > 0:
                # Normalize around a tight window centred on the sampled pixel's
                # neighbourhood so near objects fill the full colour range.
                sample_val = depth_val_m if depth_val_m is not None else float(np.median(finite))
                spread = max(0.3, float(np.std(finite)))
                d_min = max(float(np.min(finite)), sample_val - 2.0 * spread)
                d_max = min(float(np.max(finite)), sample_val + 2.0 * spread)
                if d_max <= d_min:
                    d_min, d_max = float(np.percentile(finite, 2)), float(np.percentile(finite, 98))
                depth_clip = np.clip(depth, d_min, d_max)
                depth_norm = ((depth_clip - d_min) / (d_max - d_min) * 255.0).astype(np.uint8)
                depth_color = cv2.applyColorMap(depth_norm, cv2.COLORMAP_TURBO)
            else:
                depth_color = np.zeros((h, w, 3), dtype=np.uint8)

            rgb_bgr = cv2.cvtColor(rgb_np, cv2.COLOR_RGB2BGR)

            # Overlay panel: depth colormap blended over RGB
            overlay = cv2.addWeighted(rgb_bgr, 0.45, depth_color, 0.55, 0)

            # Crosshair marker on all three panels
            marker_color_rgb   = (0, 80, 255)   # red-ish on RGB
            marker_color_depth = (255, 255, 255) # white on depth
            marker_color_ov    = (0, 255, 255)   # yellow on overlay
            for panel, col in [(rgb_bgr, marker_color_rgb), (depth_color, marker_color_depth), (overlay, marker_color_ov)]:
                cv2.circle(panel, (u, v), 9, col, 2)
                cv2.line(panel, (u - 14, v), (u + 14, v), col, 1)
                cv2.line(panel, (u, v - 14), (u, v + 14), col, 1)

            # Depth value label near the marker
            if depth_val_m is not None:
                depth_label = f"{depth_val_m:.3f} m"
                lx, ly = u + 14, v - 8
                for panel in (depth_color, overlay):
                    cv2.putText(panel, depth_label, (lx, ly), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 0), 3)
                    cv2.putText(panel, depth_label, (lx, ly), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 1)

            for panel, caption in [(rgb_bgr, "RGB"), (depth_color, f"depth  [{d_min:.2f}-{d_max:.2f} m]"), (overlay, "overlay")]:
                cv2.putText(panel, caption, (10, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 0, 0), 3)
                cv2.putText(panel, caption, (10, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255, 255, 255), 1)

            composite = np.hstack([rgb_bgr, depth_color, overlay])
            out_path = self._snapshot_temp_dir / f"{req_tag}_backproject.png"
            cv2.imwrite(str(out_path), composite)

            meta_path = self._snapshot_temp_dir / f"{req_tag}_backproject.json"
            header_stamp = self._snapshot_header.stamp if self._snapshot_header is not None else None
            metadata = {
                "req_tag": req_tag,
                "pixel_uv": [u, v],
                "rgb_size": [int(w), int(h)],
                "depth_shape": [int(v) for v in depth.shape[:2]],
                "stamp_sec": float(header_stamp.sec + header_stamp.nanosec * 1e-9) if header_stamp else None,
                "frame_id": self._snapshot_header.frame_id if self._snapshot_header is not None else None,
                "file": str(out_path),
            }
            meta_path.write_text(json.dumps(metadata, indent=2, ensure_ascii=False), encoding="utf-8")
            return out_path
        except Exception as exc:
            self.get_logger().warn(f"[TRACKER] Failed to save backproject debug image: {exc}")
            return None

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

    def _geometry_cam_pt_cb(self, msg: PointStamped):
        """Store the latest TF-derived object position in hand camera frame (no depth sensor needed)."""
        self._geometry_cam_pt = (float(msg.point.x), float(msg.point.y), float(msg.point.z))

    def _synced_image_cb(self, rgb_msg: RosImage, depth_msg: RosImage):
        """Recebe par RGB+depth sincronizado e armazena para o tracking loop."""
        try:
            cv_rgb = self.bridge.imgmsg_to_cv2(rgb_msg, desired_encoding='bgr8')
            cv_rgb = cv2.cvtColor(cv_rgb, cv2.COLOR_BGR2RGB)
            self.latest_rgb = Image.fromarray(cv_rgb)
            self._last_mask_shape = (rgb_msg.height, rgb_msg.width)
            self.latest_depth = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='32FC1')
            self.latest_depth_header = depth_msg.header
            self._latest_rgb_header = rgb_msg.header
            self.new_frame_available = True
            stamp_sec = rgb_msg.header.stamp.sec + rgb_msg.header.stamp.nanosec * 1e-9
            self._frame_buffer.append((stamp_sec, self.latest_rgb, self.latest_depth, depth_msg.header))
        except Exception as e:
            self.get_logger().error(f"Image conversion error: {e}")
            return
        # Pending seed_pixel: tf_projection republishes the pixel at ~10 Hz with a fresh TF
        # lookup while seeding is active, so by the time the velocity gate passes the pixel
        # used for init is <100 ms old. Gate on (a) a frame captured at/after the last TF
        # stamp and (b) camera velocity below the reinit gate.
        if self._pending_seed_pixel_uv is not None:
            # If tracking is already active, consume and discard the pending seed pixel.
            # The continuous republisher sends at 10Hz — we must not re-init each time.
            if self.tracking_active:
                self._pending_seed_pixel_uv = None
                self._pending_seed_pixel_tnow = None
            elif stamp_sec >= self._pending_seed_pixel_tnow:
                clock_now = self.get_clock().now().nanoseconds * 1e-9
                age = clock_now - self._pending_seed_pixel_clock_t
                if age > self._hand_reinit_speed_gate_timeout_s:
                    self.get_logger().warn(
                        f"[TRACKER] seed_pixel expired after {age:.1f}s — discarding"
                    )
                    self._pending_seed_pixel_uv = None
                    self._pending_seed_pixel_tnow = None
                elif (self._hand_cam_speed is not None
                      and self._hand_cam_speed > self._hand_reinit_speed_gate_m_s):
                    self.get_logger().info(
                        f"[TRACKER] seed_pixel init deferred: cam_speed={self._hand_cam_speed:.3f} m/s "
                        f"> gate={self._hand_reinit_speed_gate_m_s:.3f} (age={age:.2f}s)",
                        throttle_duration_sec=0.5,
                    )
                else:
                    u_sp, v_sp = self._pending_seed_pixel_uv
                    self._pending_seed_pixel_uv = None
                    self._pending_seed_pixel_tnow = None
                    self.get_logger().info(
                        f"[TRACKER] stamp-gate passed (cam_speed={self._hand_cam_speed or 0.0:.3f}), "
                        f"initializing SAM2 at ({u_sp},{v_sp})"
                    )
                    self._do_video_predictor_init(u_sp, v_sp, self.latest_rgb, self.latest_depth, depth_msg.header)
        # Publish last known mask (or zeros) with this frame's exact timestamp so
        # nvblox ExactTimeSynchronizer (color+mask) always fires on every frame.
        self._publish_mask_for_header(rgb_msg.header)
        # Always refresh display so feed is visible even before detection
        if self.visualize and not self.tracking_active:
            self._update_display(self.latest_rgb)

    def _update_display(self, img_pil=None):
        """Refresh the hand camera debug window using current _hand_disp_* state."""
        if img_pil is None:
            img_pil = self.latest_rgb
        if img_pil is None:
            return
        try:
            img_bgr = cv2.cvtColor(np.array(img_pil), cv2.COLOR_RGB2BGR)
            state = self._hand_disp_state
            mask_np = self._hand_disp_mask
            score = self._hand_disp_score
            centroid_uv = self._hand_disp_centroid

            # Mask overlay
            if mask_np is not None and state == 'TRACKING':
                overlay = np.zeros_like(img_bgr)
                overlay[mask_np > 0] = [0, 255, 0]
                img_bgr = cv2.addWeighted(img_bgr, 1.0, overlay, 0.5, 0)
                contours, _ = cv2.findContours(mask_np, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                cv2.drawContours(img_bgr, contours, -1, (0, 255, 0), 2)

            # Centroid dot
            if centroid_uv is not None and state == 'TRACKING':
                u, v = int(centroid_uv[0]), int(centroid_uv[1])
                cv2.circle(img_bgr, (u, v), 6, (0, 0, 255), -1)

            # Detection bbox (shown while seed was received, until tracking stabilises)
            if self._hand_disp_bbox is not None and state in ('RELOCALIZING', 'TRACKING'):
                x1, y1, x2, y2 = [int(c) for c in self._hand_disp_bbox]
                cv2.rectangle(img_bgr, (x1, y1), (x2, y2), (0, 200, 255), 2)
                det_txt = f"{self._hand_disp_label}  {self._hand_disp_conf:.2f}"
                cv2.putText(img_bgr, det_txt, (x1, max(y1 - 6, 14)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 0), 3)
                cv2.putText(img_bgr, det_txt, (x1, max(y1 - 6, 14)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 200, 255), 2)

            # Header bar
            if state == 'TRACKING':
                label_color = (0, 220, 0)
                if centroid_uv is not None:
                    u, v = int(centroid_uv[0]), int(centroid_uv[1])
                    label = f"TRACKING  score={score:.3f}  centroid=({u},{v})"
                else:
                    label = f"TRACKING  score={score:.3f}"
            elif state == 'RELOCALIZING':
                label_color = (0, 220, 255)
                det = f"  [{self._hand_disp_label}  {self._hand_disp_conf:.2f}]" if self._hand_disp_label else ""
                label = f"RELOCALIZING{det}"
            elif state == 'LOST':
                label_color = (0, 165, 255)
                label = "LOST (no mask)"
            else:  # IDLE
                label_color = (160, 160, 160)
                label = "IDLE — waiting for detection"

            header = f"[hand]  {label}"
            cv2.putText(img_bgr, header, (10, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 0, 0), 3)
            cv2.putText(img_bgr, header, (10, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.65, label_color, 2)

            if self._hand_cam_speed is not None:
                speed_label = f"cam_speed: {self._hand_cam_speed:.3f} m/s"
                speed_color = (0, 220, 0) if self._hand_cam_speed <= 0.05 else (0, 100, 255)
                cv2.putText(img_bgr, speed_label, (10, 52), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 3)
                cv2.putText(img_bgr, speed_label, (10, 52), cv2.FONT_HERSHEY_SIMPLEX, 0.6, speed_color, 2)

            cv2.imshow(self.tracking_window_name, img_bgr)
            cv2.waitKey(1)
        except Exception as e:
            self.get_logger().warn(f"Visualization error: {e}")

    def _publish_mask_for_header(self, header):
        """Publish last known mask (or zeros) stamped with the given header.

        Called on every incoming frame so nvblox's synchronizers always get a
        mask at the exact timestamp of the color/depth image.  When not
        tracking, zeros are published — all pixels are treated as background
        (static TSDF) which is the correct behaviour.
        """
        if not self.publish_segmentation_mask:
            return
        try:
            if self._last_mask_np is not None:
                m = ((self._last_mask_np > 0).astype(np.uint8) * 255)
            else:
                h, w = self._last_mask_shape if self._last_mask_shape else (480, 640)
                m = np.zeros((h, w), dtype=np.uint8)
            msg = self.bridge.cv2_to_imgmsg(m, encoding='mono8')
            msg.header = header
            self.mask_pub.publish(msg)
        except Exception as e:
            self.get_logger().error(f"Error publishing segmentation mask: {e}")

    def _publish_segmentation_mask(self, mask_np):
        """Update the last known mask so the next per-frame publish uses it."""
        if not self.publish_segmentation_mask:
            return
        self._last_mask_np = mask_np

    def _get_valid_depth(self, depth_map, u, v, radius=8):
        """Return robust depth estimate near (u,v).

        Uses a larger patch (radius=8 → 17×17) and returns the median of the
        closest-third of valid values. This avoids being pulled to background
        depth when the grasp point falls inside a hole (e.g. valve center).
        """
        h, w = depth_map.shape[:2]
        v_c = min(max(0, int(v)), h - 1)
        u_c = min(max(0, int(u)), w - 1)
        v_min, v_max = max(0, v_c - radius), min(h, v_c + radius + 1)
        u_min, u_max = max(0, u_c - radius), min(w, u_c + radius + 1)
        patch = depth_map[v_min:v_max, u_min:u_max]
        finite = patch[np.isfinite(patch) & (patch > 0)]
        if len(finite) == 0:
            return 0.0, False
        finite_m = finite.copy()
        if finite_m.max() > 20.0:
            finite_m = finite_m / 1000.0
        finite_m = finite_m[(finite_m > 0.05) & (finite_m < 10.0)]
        if len(finite_m) == 0:
            return 0.0, False
        # Use median of the closest third to prefer foreground over background
        finite_m.sort()
        n = max(1, len(finite_m) // 3)
        z = float(np.median(finite_m[:n]))
        if z <= 0.05 or z >= 10.0:
            return 0.0, False
        return z, True

    def _get_depth_from_mask(self, depth_map, mask_np):
        """Return median valid depth sampled from the tracked mask pixels."""
        ys, xs = np.where(mask_np > 0)
        if len(ys) == 0:
            return 0.0, False
        vals = depth_map[ys, xs]
        finite = vals[np.isfinite(vals) & (vals > 0)]
        if len(finite) == 0:
            return 0.0, False
        med = float(np.median(finite))
        if med > 20.0:
            med = med / 1000.0
        if med <= 0.05 or med >= 10.0:
            return 0.0, False
        return med, True

    def _lookup_frame_at_stamp(self, target_stamp_sec: float, max_dt_sec: float = 0.5):
        """Find the frame in the ring buffer closest to target_stamp_sec.
        Returns (rgb_pil, depth_np, header) or (None, None, None) if buffer empty
        or closest frame is further than max_dt_sec."""
        if not self._frame_buffer:
            return None, None, None
        best = min(self._frame_buffer, key=lambda e: abs(e[0] - target_stamp_sec))
        dt = abs(best[0] - target_stamp_sec)
        self.get_logger().info(
            f"[TRACKER] frame lookup: target={target_stamp_sec:.3f} best={best[0]:.3f} dt={dt*1000:.1f}ms"
        )
        if dt > max_dt_sec:
            self.get_logger().info(
                f"[TRACKER] frame lookup: dt={dt*1000:.0f}ms > {max_dt_sec*1000:.0f}ms, waiting for buffer to catch up"
            )
            return None, None, None
        return best[1], best[2], best[3]

    def _pixel_to_camera_point(self, u: int, v: int, depth_map):
        if self.camera_intrinsics is None:
            return None
        z, ok = self._get_valid_depth(depth_map, u, v)
        if not ok:
            return None
        fx, fy, cx, cy = self.camera_intrinsics
        x = (float(u) - cx) * z / fx
        y = (float(v) - cy) * z / fy
        return (x, y, z)

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

    def _publish_tracking_3d(self, x: float, y: float, z: float, header):
        msg = PointStamped()
        msg.header = header
        msg.point.x = float(x)
        msg.point.y = float(y)
        msg.point.z = float(z)
        if not msg.header.frame_id:
            msg.header.frame_id = self.camera_frame_id or 'hand_cam'
        self._tracking_2d_pub.publish(msg)

    def _apply_seed_command(self, seed: dict):
        """Run SAM image predictor on snapshot, back-project grasp to 3D, publish seed_3d."""
        bbox_1000 = seed.get("bbox_1000")
        grasps_1000 = seed.get("grasps_1000", [])
        if not isinstance(bbox_1000, list) or len(bbox_1000) < 4:
            raise RuntimeError("missing bbox_1000 in seed")

        # Use the exact frame the VLM processed (by stamp) from the ring buffer
        vlm_stamp = seed.get("frame_stamp_sec")
        if vlm_stamp is not None:
            img_pil, depth_for_seed, header = self._lookup_frame_at_stamp(float(vlm_stamp))
        else:
            img_pil, depth_for_seed, header = None, None, None

        # Fallback: snapshot taken at RELOCALIZING, then latest
        if img_pil is None:
            img_pil = self._snapshot_rgb if self._snapshot_rgb is not None else self.latest_rgb
            depth_for_seed = self._snapshot_depth if self._snapshot_depth is not None else self.latest_depth
            header = self._snapshot_header if self._snapshot_header is not None else self.latest_depth_header
            self.get_logger().warn("[TRACKER] frame_stamp_sec not in seed or buffer miss — using snapshot/latest fallback")

        if img_pil is None or header is None:
            raise RuntimeError("frame unavailable for seed apply")

        img_pil = img_pil.copy()
        orig_w, orig_h = img_pil.size

        x1 = int((bbox_1000[0] / 1000.0) * orig_w)
        y1 = int((bbox_1000[1] / 1000.0) * orig_h)
        x2 = int((bbox_1000[2] / 1000.0) * orig_w)
        y2 = int((bbox_1000[3] / 1000.0) * orig_h)
        x1, y1 = max(0, x1), max(0, y1)
        x2, y2 = min(orig_w - 1, x2), min(orig_h - 1, y2)
        bbox = [x1, y1, x2, y2]

        # Store detection info for display
        self._hand_disp_label = str(seed.get('label', ''))
        self._hand_disp_conf = float(seed.get('confidence', 0.0))
        self._hand_disp_bbox = bbox
        self._hand_disp_state = 'RELOCALIZING'

        grasp_points_uv = []
        for g in grasps_1000:
            if not isinstance(g, list) or len(g) < 2:
                continue
            gu = int((g[0] / 1000.0) * orig_w)
            gv = int((g[1] / 1000.0) * orig_h)
            grasp_points_uv.append([max(0, min(orig_w - 1, gu)), max(0, min(orig_h - 1, gv))])
        if not grasp_points_uv:
            grasp_points_uv = [[(x1 + x2) // 2, (y1 + y2) // 2]]

        seed_box = {
            "label": str(seed.get("label", "target")),
            "bbox_1000": [float(v) for v in bbox_1000[:4]],
            "grasps_1000": [[float(p[0]), float(p[1])] for p in grasps_1000 if isinstance(p, list) and len(p) >= 2],
            "confidence": float(seed.get("confidence", 1.0)),
        }
        draw_result(img_pil, [seed_box], "camera_capture_detected.jpg")
        self.get_logger().info("Saved camera capture: camera_capture_detected.jpg")

        sam_img_predictor = _load_sam_model()
        grasp_u, grasp_v = grasp_points_uv[0]
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
                grasp_u, grasp_v = grasp_points_uv[min(best_idx, len(grasp_points_uv) - 1)]

        # Back-project grasp pixel to 3D using snapshot depth
        snap_stamp_sec = header.stamp.sec + header.stamp.nanosec * 1e-9
        backproject_tag = f"backproject_{self._snapshot_run_idx:04d}"
        debug_path = self._save_backproject_debug_image(img_pil, depth_for_seed, (grasp_u, grasp_v), backproject_tag)
        if debug_path is not None:
            self.get_logger().info(f"[TRACKER] Back-project debug saved: {debug_path}")
        self.get_logger().info(
            f"[TRACKER] Back-projecting pixel ({grasp_u},{grasp_v}) snapshot_stamp={snap_stamp_sec:.3f} frame={header.frame_id}"
        )
        point_cam = self._pixel_to_camera_point(grasp_u, grasp_v, depth_for_seed)
        if point_cam is None:
            raise RuntimeError("No valid depth at grasp point in snapshot")
        z = point_cam[2]
        self.get_logger().info(
            f"[TRACKER] depth at grasp ({grasp_u},{grasp_v}): z={z:.3f}m"
        )
        # Depth sanity: reject if depth differs wildly from first successful detection
        if not hasattr(self, '_reference_seed_depth') or self._reference_seed_depth is None:
            self._reference_seed_depth = z
        else:
            ratio = z / self._reference_seed_depth if self._reference_seed_depth > 0 else 999.0
            if ratio > 2.5 or ratio < 0.4:
                self.get_logger().warn(
                    f"[TRACKER] Seed depth rejected: z={z:.3f}m vs reference={self._reference_seed_depth:.3f}m "
                    f"(ratio={ratio:.2f}, allowed 0.4–2.5)"
                )
                raise RuntimeError(f"Seed depth {z:.2f}m too far from reference {self._reference_seed_depth:.2f}m")

        # Publish seed 3D for tf_projection to reproject to current frame
        msg_3d = PointStamped()
        msg_3d.header = header  # stamp=T0, frame=hand_cam
        msg_3d.point.x = float(point_cam[0])
        msg_3d.point.y = float(point_cam[1])
        msg_3d.point.z = float(point_cam[2])
        self._seed_3d_pub.publish(msg_3d)
        self.get_logger().info(
            f"[TRACKER] Seed 3D published ({point_cam[0]:.3f}, {point_cam[1]:.3f}, {point_cam[2]:.3f})"
        )

        # Clear snapshot
        self._snapshot_rgb = None
        self._snapshot_depth = None
        self._snapshot_header = None

    def _seed_pixel_cb(self, msg: PointStamped):
        """Receive reprojected pixel from tf_projection.

        tf_projection republishes the pixel at ~10 Hz with a fresh TF lookup while a
        seed is active. This callback just refreshes (u, v) and tnow on each message;
        clock_t is set once on the first pending message so the timeout measures the
        full wait, not just the time since the latest republish.
        """
        u = int(msg.point.x)
        v = int(msg.point.y)
        t_now = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        # Reject out-of-bounds pixels (TF reprojection can produce garbage when
        # the object is outside the camera FOV after the robot has moved)
        if self._last_mask_shape is not None:
            h, w = self._last_mask_shape
            margin = 20
            if not (margin <= u < w - margin and margin <= v < h - margin):
                self.get_logger().info(
                    f"[TRACKER] seed_pixel rejected: ({u},{v}) out of image bounds ({w}x{h})",
                    throttle_duration_sec=0.5,
                )
                return
        first_pending = self._pending_seed_pixel_uv is None
        self._pending_seed_pixel_uv = (u, v)
        self._pending_seed_pixel_tnow = t_now
        if first_pending:
            self._pending_seed_pixel_clock_t = self.get_clock().now().nanoseconds * 1e-9
            self._pending_seed_pixel_frame_count = 0
            self.get_logger().info(
                f"[TRACKER] seed_pixel queued: ({u},{v}) tnow={t_now:.3f} — will init on next fresh frame"
            )

    def _compute_hand_expected_uv(self):
        """Project _geometry_cam_pt (3D in hand cam frame) to pixel coordinates.

        Returns (u, v) or None if unavailable.
        """
        if self._geometry_cam_pt is None or self.camera_intrinsics is None:
            return None
        cam_x, cam_y, cam_z = self._geometry_cam_pt
        if cam_z <= 0.0:
            return None
        fx, fy, cx, cy = self.camera_intrinsics
        u = fx * (cam_x / cam_z) + cx
        v = fy * (cam_y / cam_z) + cy
        # Use actual image dimensions when available, otherwise generous fallback
        if self._last_mask_shape is not None:
            h, w = self._last_mask_shape
            if not (0 <= u < w and 0 <= v < h):
                return None
        elif not (0 <= u < 2000 and 0 <= v < 2000):
            return None
        return (u, v)

    def _do_video_predictor_init(self, u: int, v: int, img_pil, depth_np, header):
        """Initialize (or reinitialize) the SAM2 video predictor on the given frame."""
        img_np = np.array(img_pil.convert("RGB"))

        # Lazy-init video predictor
        if self.video_predictor is None:
            self.get_logger().info("Lazy-loading SAM2 VideoPredictor...")
            self.video_predictor = _load_sam_video_model()
            if self.video_predictor is None:
                self.get_logger().error("Failed to load SAM2 VideoPredictor")
                return

        # Reset state for clean init
        self.video_predictor.inference_state = {}
        self.video_predictor._ros_frame_idx = 0

        with self._gpu_lock:
            init_results = self.video_predictor(img_np, points=[[u, v]], labels=[1])
        best_tracked_mask, score = _best_mask_from_results(init_results)
        if best_tracked_mask is None:
            self.get_logger().error("Video predictor init produced no mask from reprojected pixel")
            return

        mask_np = best_tracked_mask.astype(np.uint8)
        # IoU validation: reject if mask is not concentrated around seed pixel
        h_img, w_img = mask_np.shape[:2]
        circle_mask = np.zeros((h_img, w_img), dtype=np.uint8)
        cv2.circle(circle_mask, (u, v), int(self._hand_iou_radius_px), 1, -1)
        mask_area = float(mask_np.sum())
        overlap = float((mask_np & circle_mask).sum()) / max(mask_area, 1.0)
        if overlap < self._hand_iou_min_overlap:
            self.get_logger().warn(
                f"[HAND] SAM2 init rejected by IoU: overlap={overlap:.2f} < {self._hand_iou_min_overlap:.2f}"
                f" seed=({u},{v}) mask_area={int(mask_area)}px²"
            )
            return
        self.tracking_active = True
        self.tracking_frame_count = 1
        self.last_tracking_score = score
        self._tracking_lost_streak = 0
        self._publish_segmentation_mask(mask_np)

        m = cv2.moments(mask_np)
        if m["m00"] != 0:
            mask_u = int(m["m10"] / m["m00"])
            mask_v = int(m["m01"] / m["m00"])
            self.grasp_offset = (u - mask_u, v - mask_v)
        else:
            mask_u, mask_v = u, v
            self.grasp_offset = (0, 0)

        point_cam = self._pixel_to_camera_point(u, v, depth_np)
        if point_cam is None:
            # Reprojected pixel may be in a depth hole; fall back to mask centroid or mask pixels
            mask_u_c = mask_u if m["m00"] != 0 else u
            mask_v_c = mask_v if m["m00"] != 0 else v
            z, ok = self._get_valid_depth(depth_np, mask_u_c, mask_v_c, radius=10)
            if not ok:
                z, ok = self._get_depth_from_mask(depth_np, mask_np)
            if ok and header is not None and self.camera_intrinsics is not None:
                fx, fy, cx, cy = self.camera_intrinsics
                x = (float(mask_u_c) - cx) * z / fx
                y = (float(mask_v_c) - cy) * z / fy
                point_cam = (x, y, z)
        if point_cam is not None and header is not None:
            self._publish_tracking_3d(point_cam[0], point_cam[1], point_cam[2], header)
        else:
            self.get_logger().warn("No valid depth for tracking point after reproject")

        self.get_logger().info(
            f"[TRACKER] Video predictor initialized from reprojected pixel ({u}, {v}), score={score:.3f}"
        )

        # ── Debug: save the exact frame SAM2 was initialized on, with mask + seed point ──
        try:
            dbg_img = img_pil.convert("RGB").copy()
            dbg_arr = np.array(dbg_img)
            # Overlay mask in semi-transparent green
            green_overlay = np.zeros_like(dbg_arr)
            green_overlay[mask_np > 0] = [0, 200, 0]
            dbg_arr = cv2.addWeighted(dbg_arr, 1.0, green_overlay, 0.45, 0)
            # Draw seed point (cross-hair)
            cv2.drawMarker(dbg_arr, (u, v), (255, 0, 0), cv2.MARKER_CROSS, 20, 2)
            # Draw mask centroid
            if m["m00"] != 0:
                cv2.circle(dbg_arr, (mask_u, mask_v), 5, (0, 0, 255), -1)
            debug_tag = f"sam2_init_{self._snapshot_run_idx:04d}"
            debug_path = self._snapshot_temp_dir / f"{debug_tag}.png"
            cv2.imwrite(str(debug_path), cv2.cvtColor(dbg_arr, cv2.COLOR_RGB2BGR))
            self.get_logger().info(f"[TRACKER] SAM2 init debug saved: {debug_path}")
        except Exception as _exc:
            self.get_logger().warn(f"[TRACKER] SAM2 init debug save failed: {_exc}")

        self._hand_disp_state = 'TRACKING'
        self._hand_disp_mask = mask_np
        self._hand_disp_score = float(score)
        self._hand_disp_centroid = (u, v)
        if self.visualize:
            self._hand_ui_calls += 1
            self._update_display(img_pil)

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
        with self._gpu_lock:
            tracked_results = self.video_predictor(img_np)
        best_tracked_mask, score = _best_mask_from_results(tracked_results)
        if best_tracked_mask is None:
            self._tracking_lost_streak += 1
            self.get_logger().warn(
                f"[HAND] No mask from SAM2 (lost_streak={self._tracking_lost_streak}/{self.tracking_lost_confirm_frames})",
                throttle_duration_sec=1.0,
            )
            if self._tracking_lost_streak >= self.tracking_lost_confirm_frames:
                self.tracking_active = False
                self._hand_disp_state = 'LOST'
                self._hand_disp_mask = None
                self._hand_disp_centroid = None
                self.get_logger().warn("[HAND] Tracking lost — no mask for too long")
            return

        mask_np = best_tracked_mask.astype(np.uint8)
        self.last_tracking_score = float(score)
        self.tracking_frame_count += 1

        m = cv2.moments(mask_np)
        if m["m00"] == 0:
            self._tracking_lost_streak += 1
            if self._tracking_lost_streak >= self.tracking_lost_confirm_frames:
                self.tracking_active = False
                self._hand_disp_state = 'LOST'
                self._hand_disp_mask = None
                self._hand_disp_centroid = None
                self.get_logger().warn("[HAND] Tracking lost — empty mask")
            return
        u = int(m["m10"] / m["m00"])
        v = int(m["m01"] / m["m00"])

        # ── Centroid consistency: reject mask if centroid drifted from expected position ──
        # Uses _geometry_cam_pt (TF-projected object position in hand cam frame at 10Hz)
        # to compute where the object should appear in the image.
        expected_uv = self._compute_hand_expected_uv()
        if expected_uv is not None:
            eu, ev = expected_uv
            dist = ((u - eu) ** 2 + (v - ev) ** 2) ** 0.5
            if dist > self._hand_max_centroid_dist_px:
                self._tracking_lost_streak += 1
                self.get_logger().warn(
                    f"[HAND] Centroid ({u},{v}) too far from "
                    f"expected ({eu:.0f},{ev:.0f}), dist={dist:.0f}px — background?",
                    throttle_duration_sec=1.0,
                )
                if self._tracking_lost_streak >= self.tracking_lost_confirm_frames:
                    self.tracking_active = False
                    self._hand_disp_state = 'LOST'
                    self._hand_disp_mask = None
                    self._hand_disp_centroid = None
                    self.get_logger().warn("[HAND] Tracking lost — centroid too far for too long")
                return

        self._tracking_lost_streak = 0
        self._publish_segmentation_mask(mask_np)

        # Trim old SAM2 memory every N frames — keeps only the last 6 frames in
        # inference_state so GPU usage stays bounded without any reseed or state reset.
        if self.tracking_frame_count % self._sam2_memory_reset_interval == 0:
            _trim_sam2_memory(self.video_predictor, keep_frames=6)
            self.get_logger().info(
                f"[TRACKER] SAM2 memory trimmed at frame {self.tracking_frame_count}"
            )
        mask_centroid_u, mask_centroid_v = u, v
        if hasattr(self, "grasp_offset"):
            u += int(self.grasp_offset[0])
            v += int(self.grasp_offset[1])
        h = self.latest_depth_header if self.latest_depth_header is not None else None
        if h is None:
            return
        point_cam = self._pixel_to_camera_point(u, v, self.latest_depth)
        if point_cam is None:
            # Centroid+offset landed in a depth hole; try the mask centroid with wider radius
            z, ok = self._get_valid_depth(self.latest_depth, mask_centroid_u, mask_centroid_v, radius=10)
            if not ok:
                # Last resort: sample depth from all mask pixels
                z, ok = self._get_depth_from_mask(self.latest_depth, mask_np)
            if ok and self.camera_intrinsics is not None:
                fx, fy, cx, cy = self.camera_intrinsics
                x = (float(mask_centroid_u) - cx) * z / fx
                y = (float(mask_centroid_v) - cy) * z / fy
                point_cam = (x, y, z)
        # Last-resort fallback: use TF-derived geometry depth (no depth sensor required).
        # The tf_projection_node reprojects the known vision-frame object position into the
        # hand cam frame at ~10 Hz.  Z is accurate; we pair it with the mask centroid pixel.
        if point_cam is None and self._geometry_cam_pt is not None:
            gz = self._geometry_cam_pt[2]
            if gz > 0.0 and self.camera_intrinsics is not None:
                fx, fy, cx, cy = self.camera_intrinsics
                x_g = (float(mask_centroid_u) - cx) * gz / fx
                y_g = (float(mask_centroid_v) - cy) * gz / fy
                point_cam = (x_g, y_g, gz)
                self.get_logger().info(
                    f"Using geometry depth fallback: z={gz:.3f}m centroid=({mask_centroid_u},{mask_centroid_v})",
                    throttle_duration_sec=1.0,
                )

        if point_cam is not None:
            self._publish_tracking_3d(point_cam[0], point_cam[1], point_cam[2], h)
        else:
            self.get_logger().warn(
                f"No valid depth for tracking point — "
                f"centroid=({mask_centroid_u},{mask_centroid_v}) "
                f"adjusted_pixel=({u},{v}) "
                f"geometry_cam_pt={self._geometry_cam_pt}",
                throttle_duration_sec=1.0,
            )
        self._hand_disp_state = 'TRACKING'
        self._hand_disp_mask = mask_np
        self._hand_disp_score = float(score)
        self._hand_disp_centroid = (u, v)
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
                        "mask_area": int(mask_np.sum()),
                    },
                )
            self._update_display(img_pil)

    def _secondary_rgb_cb(self, msg: RosImage, cam: str):
        """Store latest RGB frame for a secondary camera."""
        st = self._secondary_cam_state.get(cam)
        if st is None:
            return
        try:
            cv_rgb = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            cv_rgb = cv2.cvtColor(cv_rgb, cv2.COLOR_BGR2RGB)
            st['latest_rgb'] = Image.fromarray(cv_rgb)
            st['latest_rgb_header'] = msg.header
            st['new_frame_available'] = True
            st['_last_mask_shape'] = (msg.height, msg.width)
        except Exception as e:
            self.get_logger().error(f"[{cam}] RGB conversion error: {e}")
            return
        # Publish zeros stamped with this RGB header only when confirmed out-of-FOV
        # (static TSDF integrates the whole frame). In-FOV frames do NOT publish
        # cached masks here — mask publishing is driven by SAM2 inference rate so
        # nvblox does not integrate object pixels as static during inference gaps.
        if not st['tracking_initialized']:
            last_seed = st.get('last_seed_time', 0.0)
            out_of_fov = (
                last_seed == 0.0
                or (time.time() - last_seed) > self._secondary_fov_timeout_sec
            )
            if out_of_fov:
                st['_last_mask_np'] = None
                self._publish_secondary_zero_mask(cam, msg.header, msg.height, msg.width)

    def _secondary_seed_pixel_cb(self, msg: PointStamped, cam: str):
        """Receive seed pixel for a secondary camera.
        point.z == 1.0: VLM re-seed → force SAM2 reinit immediately.
        point.z == 0.0: continuous tracking update → only update init_uv;
                         if not tracking, increment consecutive_seed_count for hysteresis.
        """
        st = self._secondary_cam_state.get(cam)
        if st is None:
            return
        st['init_uv'] = (float(msg.point.x), float(msg.point.y))
        st['init_uv_stamp'] = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        st['last_seed_time'] = time.time()
        force_reinit = float(msg.point.z) > 0.5
        if force_reinit:
            st['needs_reinit'] = True
            st['consecutive_seed_count'] = self._secondary_fov_enter_count  # bypass hysteresis
            st['cooldown_until'] = 0.0  # VLM detection overrides cooldown
        elif not st['tracking_initialized']:
            # WARM predictor: skip ARMING hysteresis and cooldown — the predictor
            # already has appearance memory for the object, re-prompt directly on
            # the next frame.
            if st.get('lifecycle') == 'WARM':
                st['needs_reinit'] = True
                return
            # Respect cooldown: continuous seeds cannot re-arm during cooldown
            if time.time() < st.get('cooldown_until', 0.0):
                remaining = st['cooldown_until'] - time.time()
                self.get_logger().info(
                    f"[{cam}] In cooldown ({remaining:.1f}s remaining) — ignoring continuous seed",
                    throttle_duration_sec=1.0,
                )
                return
            st['consecutive_seed_count'] += 1
            if st['consecutive_seed_count'] >= self._secondary_fov_enter_count:
                st['needs_reinit'] = True

    def _publish_secondary_mask(self, cam: str, mask_np, header):
        """Publish a SAM2 mask stamped with the RGB header SAM2 processed.

        Called only when SAM2 produces a fresh mask, so the mask topic ticks at the
        SAM2 inference rate while tracking. nvblox's synchronizers pair the mask
        with the RGB/depth frame at the exact stamp; gaps between inferences mean
        no mask is published and nvblox simply skips those frames (no static-TSDF
        leakage of the object).
        """
        pub = self._secondary_mask_pubs.get(cam)
        if pub is None or header is None:
            return
        st = self._secondary_cam_state.get(cam)
        if st is not None:
            st['_last_mask_np'] = mask_np
            if mask_np is not None:
                st['_last_mask_shape'] = mask_np.shape[:2]
        try:
            m = ((mask_np > 0).astype(np.uint8) * 255)
            ros_mask = self.bridge.cv2_to_imgmsg(m, encoding='mono8')
            ros_mask.header = header
            pub.publish(ros_mask)
        except Exception as e:
            self.get_logger().error(f"[{cam}] Error publishing mask: {e}")

    def _publish_secondary_zero_mask(self, cam: str, header, h: int, w: int):
        """Publish an all-zero mask so nvblox integrates the whole frame as static.

        Used only when the object is confirmed out-of-FOV — publishing zeros while
        the object is actually visible would leak it into the static TSDF.
        """
        pub = self._secondary_mask_pubs.get(cam)
        if pub is None or header is None:
            return
        try:
            m = np.zeros((h, w), dtype=np.uint8)
            ros_mask = self.bridge.cv2_to_imgmsg(m, encoding='mono8')
            ros_mask.header = header
            pub.publish(ros_mask)
        except Exception as e:
            self.get_logger().error(f"[{cam}] Error publishing zero mask: {e}")

    def _update_secondary_display(
        self, cam: str, img_pil, state: str,
        mask_np=None, score: float = 0.0,
        centroid_uv=None, expected_uv=None,
        arming_count: int = 0, arming_total: int = 0,
    ):
        """Show debug window for a secondary camera with rich state info.

        state: one of 'OUT_OF_FOV', 'ARMING', 'TRACKING', 'BACKGROUND', 'LOST'
        expected_uv: TF-projected target pixel — always drawn as a crosshair
        arming_count/arming_total: shown during ARMING phase (e.g. 3/5)
        """
        try:
            img_bgr = cv2.cvtColor(np.array(img_pil), cv2.COLOR_RGB2BGR)

            # State-specific overlay and label
            if state == 'TRACKING' and mask_np is not None:
                overlay = np.zeros_like(img_bgr)
                overlay[mask_np > 0] = [0, 255, 0]
                img_bgr = cv2.addWeighted(img_bgr, 1.0, overlay, 0.5, 0)
                contours, _ = cv2.findContours(mask_np, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                cv2.drawContours(img_bgr, contours, -1, (0, 255, 0), 2)
                if centroid_uv:
                    cu, cv_ = int(centroid_uv[0]), int(centroid_uv[1])
                    cv2.circle(img_bgr, (cu, cv_), 6, (0, 0, 255), -1)
                    label = f"TRACKING  score={score:.3f}  centroid=({cu},{cv_})"
                else:
                    label = f"TRACKING  score={score:.3f}"
                label_color = (0, 220, 0)

            elif state == 'BACKGROUND' and mask_np is not None:
                overlay = np.zeros_like(img_bgr)
                overlay[mask_np > 0] = [0, 0, 200]
                img_bgr = cv2.addWeighted(img_bgr, 1.0, overlay, 0.4, 0)
                label = f"BACKGROUND REJECTED  score={score:.3f}"
                if centroid_uv and expected_uv:
                    dist = ((centroid_uv[0]-expected_uv[0])**2 + (centroid_uv[1]-expected_uv[1])**2)**0.5
                    label += f"  dist={dist:.0f}px"
                label_color = (0, 0, 255)

            elif state == 'ARMING':
                label_color = (0, 220, 255)  # yellow
                if arming_total > 0:
                    label = f"IN FOV — arming ({arming_count}/{arming_total})"
                else:
                    label = "IN FOV — arming"

            elif state == 'LOST':
                label_color = (0, 165, 255)  # orange
                label = "LOST (no mask)"

            else:  # OUT_OF_FOV
                label_color = (160, 160, 160)  # gray
                label = "OUT OF FOV"

            # Always draw TF-projected expected pixel as a crosshair
            if expected_uv is not None:
                ex, ey = int(expected_uv[0]), int(expected_uv[1])
                r = 10
                cv2.line(img_bgr, (ex - r, ey), (ex + r, ey), (255, 200, 0), 2)
                cv2.line(img_bgr, (ex, ey - r), (ex, ey + r), (255, 200, 0), 2)
                cv2.circle(img_bgr, (ex, ey), r, (255, 200, 0), 1)

            # Header bar with state
            header = f"[{cam}]  {label}"
            cv2.putText(img_bgr, header, (10, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 0, 0), 3)
            cv2.putText(img_bgr, header, (10, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.65, label_color, 2)

            cv2.imshow(f"SAM2 {cam}", img_bgr)
            cv2.waitKey(1)
        except Exception as e:
            self.get_logger().warn(f"[{cam}] Visualization error: {e}")

    def _run_secondary_tracking_step(self, cam: str):
        """Run one SAM2 tracking step for a secondary camera.

        Always updates the debug window (even without a new frame) so all 3 windows
        remain visible from the moment the node starts.
        """
        st = self._secondary_cam_state[cam]
        img_pil = st['latest_rgb']
        frame_hdr = st.get('latest_rgb_header')
        now = time.time()

        # ── FOV timeout: only applies while NOT actively tracking ────────────────
        # When tracking_initialized=True, SAM2 itself determines when the object
        # is lost (via lost_streak).  The seed timeout only gates the ARMING phase —
        # it prevents re-arming after the object has left the FOV.
        if not st['tracking_initialized'] and st['last_seed_time'] > 0.0:
            if (now - st['last_seed_time']) > self._secondary_fov_timeout_sec:
                st['consecutive_seed_count'] = 0  # must accumulate again to re-arm
                st['_disp_state'] = 'OUT_OF_FOV'
                st['_disp_mask'] = None
                st['_disp_score'] = 0.0
                st['_disp_centroid'] = None
                self.get_logger().info(
                    f"[{cam}] Out of FOV (no seed for >{self._secondary_fov_timeout_sec:.1f}s while not tracking)",
                    throttle_duration_sec=3.0,
                )

        # ── Always refresh display, even without a new SAM2 frame ─────────────
        if not st['new_frame_available']:
            if self.visualize and img_pil is not None:
                self._update_secondary_display(
                    cam, img_pil, st['_disp_state'],
                    mask_np=st['_disp_mask'],
                    score=st['_disp_score'],
                    centroid_uv=st['_disp_centroid'],
                    expected_uv=st.get('init_uv'),
                    arming_count=st['consecutive_seed_count'],
                    arming_total=self._secondary_fov_enter_count,
                )
            return

        st['new_frame_available'] = False
        img_np = np.array(img_pil.convert('RGB'))

        # ── Object not in FOV at all (no seed ever or timed out) ─────────────
        no_seed_ever = st['last_seed_time'] == 0.0
        fov_timed_out = (
            st['last_seed_time'] > 0.0
            and (now - st['last_seed_time']) > self._secondary_fov_timeout_sec
        )
        if no_seed_ever or fov_timed_out:
            st['_disp_state'] = 'OUT_OF_FOV'
            st['_disp_mask'] = None
            if self.visualize:
                self._update_secondary_display(
                    cam, img_pil, 'OUT_OF_FOV',
                    expected_uv=st.get('init_uv'),
                )
            return

        # ── ARMING: in FOV but not yet armed (hysteresis not met) ────────────
        if not st['tracking_initialized'] and not st['needs_reinit']:
            st['_disp_state'] = 'ARMING'
            st['_disp_mask'] = None
            if self.visualize:
                self._update_secondary_display(
                    cam, img_pil, 'ARMING',
                    expected_uv=st.get('init_uv'),
                    arming_count=st['consecutive_seed_count'],
                    arming_total=self._secondary_fov_enter_count,
                )
            return

        # ── Lazy-init per-camera predictor (separate instance, shared GPU lock) ──
        if st['video_predictor'] is None:
            if not SAM2_AVAILABLE:
                return
            try:
                st['video_predictor'] = SAM2ROSVideoPredictor(overrides={
                    'conf': 0.01,
                    'task': 'segment',
                    'mode': 'predict',
                    'imgsz': 1024,
                    'model': SAM2_MODEL_NAME,
                    'save': False,
                    'verbose': False,
                })
                self.get_logger().info(f"[{cam}] SAM2 predictor loaded")
            except Exception as exc:
                self.get_logger().error(f"[{cam}] Failed to init SAM2 predictor: {exc}")
                return

        init_needed = not st['tracking_initialized'] or st['needs_reinit']

        # ── SAM2 init / re-seed ───────────────────────────────────────────────
        if init_needed and st['init_uv'] is not None:
            u, v = int(st['init_uv'][0]), int(st['init_uv'][1])

            # ── Temporal alignment: reject if seed pixel TF stamp is too far
            #    from the RGB frame we are about to process.  The seed pixel was
            #    computed at whatever TF time was "latest" in tf_projection, but
            #    the frontright frame could be significantly older when the robot
            #    is moving — leading to a pixel that points to a wrong location.
            seed_stamp = st.get('init_uv_stamp')
            if seed_stamp is not None and frame_hdr is not None:
                frame_stamp = frame_hdr.stamp.sec + frame_hdr.stamp.nanosec * 1e-9
                dt_ms = abs(seed_stamp - frame_stamp) * 1000.0
                if dt_ms > 150.0:
                    self.get_logger().warn(
                        f"[{cam}] Seed pixel stale: dt={dt_ms:.0f}ms > 150ms — skipping init "
                        f"(seed_t={seed_stamp:.3f} frame_t={frame_stamp:.3f})"
                    )
                    st['needs_reinit'] = False
                    st['consecutive_seed_count'] = 0
                    if self.visualize:
                        self._update_secondary_display(
                            cam, img_pil, st['_disp_state'],
                            expected_uv=st.get('init_uv'),
                        )
                    return

            is_warm = st.get('lifecycle') == 'WARM'
            if is_warm:
                # Warm re-prompt: keep inference_state so SAM2 memory helps
                # recognize the object from prior appearances. Trim to bound growth.
                _trim_sam2_memory(st['video_predictor'], keep_frames=6)
                iou_min = self._secondary_iou_min_overlap_warm
                iou_radius = self._secondary_iou_radius_px * 1.5
                skip_area_check = True
            else:
                # Cold (or DEGRADED) init: full reset.
                st['video_predictor'].inference_state = {}
                st['video_predictor']._ros_frame_idx = 0
                iou_min = self._secondary_iou_min_overlap
                iou_radius = self._secondary_iou_radius_px
                skip_area_check = False

            with self._gpu_lock:
                results = st['video_predictor'](img_np, points=[[u, v]], labels=[1])
            mask, score = _best_mask_from_results(results)
            if mask is not None:
                mask_u8 = mask.astype(np.uint8)
                h_img, w_img = mask_u8.shape[:2]
                circle_mask = np.zeros((h_img, w_img), dtype=np.uint8)
                cv2.circle(circle_mask, (u, v), int(iou_radius), 1, -1)
                mask_area = float(mask_u8.sum())
                overlap = float((mask_u8 & circle_mask).sum()) / max(mask_area, 1.0)

                image_area = float(h_img * w_img)
                mask_frac = mask_area / max(image_area, 1.0)
                rejected = False
                if (not skip_area_check) and mask_frac > 0.20:
                    self.get_logger().warn(
                        f"[{cam}] SAM2 init rejected: mask too large "
                        f"({mask_frac*100:.1f}% > 20%) — likely background"
                    )
                    rejected = True
                elif overlap < iou_min:
                    self.get_logger().warn(
                        f"[{cam}] SAM2 {'warm re-prompt' if is_warm else 'init'} rejected by IoU: "
                        f"overlap={overlap:.2f} < {iou_min:.2f} seed=({u},{v}) mask_area={int(mask_area)}px²"
                    )
                    rejected = True

                if rejected:
                    st['needs_reinit'] = False
                    st['consecutive_seed_count'] = 0
                    if is_warm:
                        st['warm_fail_streak'] += 1
                        if st['warm_fail_streak'] >= self._secondary_warm_fail_max:
                            self.get_logger().warn(
                                f"[{cam}] warm re-prompt failed {st['warm_fail_streak']}× → DEGRADED, "
                                f"forcing cold rebuild on next cycle"
                            )
                            st['lifecycle'] = 'DEGRADED'
                            st['video_predictor'].inference_state = {}
                            st['video_predictor']._ros_frame_idx = 0
                            st['warm_fail_streak'] = 0
                    else:
                        st['cooldown_until'] = time.time() + 5.0
                else:
                    st['tracking_initialized'] = True
                    st['needs_reinit'] = False
                    st['tracking_frame_count'] = 1
                    st['last_score'] = score
                    st['lost_streak'] = 0
                    st['warm_fail_streak'] = 0
                    if is_warm:
                        st['warm_tracked_frames'] += 1
                        self.get_logger().info(
                            f"[{cam}] WARM re-entry at ({u},{v}), score={score:.3f} overlap={overlap:.2f}"
                        )
                    else:
                        # DEGRADED just rebuilt → back to COLD until promoted again.
                        if st.get('lifecycle') == 'DEGRADED':
                            st['lifecycle'] = 'COLD'
                        st['warm_tracked_frames'] = 1
                    self._publish_secondary_mask(cam, mask_u8, frame_hdr)
                    st['_disp_state'] = 'TRACKING'
                    st['_disp_mask'] = mask_u8
                    st['_disp_score'] = score
                    st['_disp_centroid'] = (u, v)
                if self.visualize:
                    self._update_secondary_display(
                        cam, img_pil, st['_disp_state'],
                        mask_np=st['_disp_mask'], score=score,
                        centroid_uv=(u, v), expected_uv=st['init_uv'],
                    )
                if not st['tracking_initialized']:
                    pass  # IoU/area rejected — skip log and debug image
                else:
                    self.get_logger().info(
                        f"[{cam}] SAM2 initialized at ({u},{v}), score={score:.3f} overlap={overlap:.2f}",
                    )
                # ── Debug: save init frame with mask + seed point (only on accepted init) ──
                if not st['tracking_initialized']:
                    return
                try:
                    dbg_arr = np.array(img_pil.convert("RGB"))
                    green_overlay = np.zeros_like(dbg_arr)
                    green_overlay[mask_u8 > 0] = [0, 200, 0]
                    dbg_arr = cv2.addWeighted(dbg_arr, 1.0, green_overlay, 0.45, 0)
                    cv2.drawMarker(dbg_arr, (u, v), (255, 0, 0), cv2.MARKER_CROSS, 20, 2)
                    m_dbg = cv2.moments(mask_u8)
                    if m_dbg["m00"] != 0:
                        mc_u = int(m_dbg["m10"] / m_dbg["m00"])
                        mc_v = int(m_dbg["m01"] / m_dbg["m00"])
                        cv2.circle(dbg_arr, (mc_u, mc_v), 5, (0, 0, 255), -1)
                    dbg_path = self._snapshot_temp_dir / f"sam2_secondary_init_{cam}_{self._snapshot_run_idx:04d}.png"
                    cv2.imwrite(str(dbg_path), cv2.cvtColor(dbg_arr, cv2.COLOR_RGB2BGR))
                    self.get_logger().info(f"[{cam}] SAM2 secondary init debug saved: {dbg_path}")
                except Exception as _exc:
                    self.get_logger().warn(f"[{cam}] SAM2 secondary init debug save failed: {_exc}")
            else:
                st['_disp_state'] = 'LOST'
                st['_disp_mask'] = None
                if self.visualize:
                    self._update_secondary_display(
                        cam, img_pil, 'LOST',
                        expected_uv=st.get('init_uv'),
                    )

        # ── Continuous SAM2 tracking ──────────────────────────────────────────
        elif st['tracking_initialized']:
            with self._gpu_lock:
                results = st['video_predictor'](img_np)
            mask, score = _best_mask_from_results(results)
            if mask is not None and score > 0.0:
                mask_u8 = mask.astype(np.uint8)
                m_cv = cv2.moments(mask_u8)
                centroid = None
                if m_cv["m00"] != 0:
                    centroid = (int(m_cv["m10"] / m_cv["m00"]), int(m_cv["m01"] / m_cv["m00"]))

                # 2D consistency: centroid must be near TF-projected target
                consistent = True
                if centroid is not None and st['init_uv'] is not None:
                    eu, ev = st['init_uv']
                    dist = ((centroid[0] - eu) ** 2 + (centroid[1] - ev) ** 2) ** 0.5
                    if dist > self._secondary_max_centroid_dist_px:
                        consistent = False
                        st['lost_streak'] += 1
                        self.get_logger().warn(
                            f"[{cam}] Centroid ({centroid[0]},{centroid[1]}) too far from "
                            f"expected ({eu:.0f},{ev:.0f}), dist={dist:.0f}px — background?",
                            throttle_duration_sec=1.0,
                        )
                        if st['lost_streak'] >= self.tracking_lost_confirm_frames:
                            st['tracking_initialized'] = False
                            st['consecutive_seed_count'] = 0  # re-arm from scratch
                            # WARM: preserve inference_state + skip cooldown so next
                            # seed triggers an instant re-prompt.
                            if st.get('lifecycle') != 'WARM':
                                st['cooldown_until'] = time.time() + 5.0
                                self.get_logger().info(
                                    f"[{cam}] Background cooldown activated (5s)"
                                )
                            else:
                                self.get_logger().info(
                                    f"[{cam}] Tracking lost (WARM) — awaiting re-entry seed"
                                )

                if consistent:
                    st['tracking_frame_count'] += 1
                    st['last_score'] = score
                    st['lost_streak'] = 0
                    if st.get('lifecycle') == 'COLD':
                        st['warm_tracked_frames'] += 1
                        if st['warm_tracked_frames'] >= self._secondary_warm_promote_frames:
                            st['lifecycle'] = 'WARM'
                            self.get_logger().info(
                                f"[{cam}] predictor promoted to WARM "
                                f"(after {st['warm_tracked_frames']} valid frames)"
                            )
                    self._publish_secondary_mask(cam, mask_u8, frame_hdr)
                    # Trim old SAM2 memory every N frames (same rationale as primary camera)
                    if st['tracking_frame_count'] % self._sam2_memory_reset_interval == 0:
                        _trim_sam2_memory(st['video_predictor'], keep_frames=6)
                        self.get_logger().info(
                            f"[{cam}] SAM2 memory trimmed at frame {st['tracking_frame_count']}"
                        )
                    st['_disp_state'] = 'TRACKING'
                    st['_disp_mask'] = mask_u8
                    st['_disp_score'] = score
                    st['_disp_centroid'] = centroid
                    if self.visualize:
                        self._update_secondary_display(
                            cam, img_pil, 'TRACKING',
                            mask_np=mask_u8, score=score,
                            centroid_uv=centroid, expected_uv=st.get('init_uv'),
                        )
                else:
                    st['_disp_state'] = 'BACKGROUND'
                    st['_disp_mask'] = mask_u8
                    st['_disp_score'] = score
                    st['_disp_centroid'] = centroid
                    if self.visualize:
                        self._update_secondary_display(
                            cam, img_pil, 'BACKGROUND',
                            mask_np=mask_u8, score=score,
                            centroid_uv=centroid, expected_uv=st.get('init_uv'),
                        )
            else:
                st['lost_streak'] += 1
                self.get_logger().warn(
                    f"[{cam}] No mask from SAM2 (lost_streak={st['lost_streak']}/{self.tracking_lost_confirm_frames})",
                    throttle_duration_sec=1.0,
                )
                if st['lost_streak'] >= self.tracking_lost_confirm_frames:
                    st['tracking_initialized'] = False
                    st['consecutive_seed_count'] = 0  # re-arm from scratch
                    suffix = " (WARM — next seed re-prompts)" if st.get('lifecycle') == 'WARM' else ""
                    self.get_logger().warn(f"[{cam}] Tracking lost — no mask for too long{suffix}")
                st['_disp_state'] = 'LOST'
                st['_disp_mask'] = None
                if self.visualize:
                    self._update_secondary_display(
                        cam, img_pil, 'LOST',
                        expected_uv=st.get('init_uv'),
                    )

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
        # seed_pixel init is now handled in _synced_image_cb when matching frame arrives

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

        for cam in self._secondary_cameras:
            self._run_secondary_tracking_step(cam)


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
