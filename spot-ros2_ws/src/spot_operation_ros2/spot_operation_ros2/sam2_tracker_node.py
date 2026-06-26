#!/usr/bin/env python3
import collections
import json
import math
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
from PIL import Image
from cv_bridge import CvBridge
from geometry_msgs.msg import PointStamped
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image as RosImage
from std_msgs.msg import Float64, String
from visualization_msgs.msg import Marker
from tf2_ros import Buffer, ConnectivityException, ExtrapolationException, LookupException, TransformListener

from .image_roll import (
    roll_deg_from_quaternion,
    rotate_image_upright,
    reroll_mask_to_original,
)

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

    Trims BOTH the shared `output_dict` AND the per-object dicts
    (`output_dict_per_obj` / `temp_output_dict_per_obj`). The per-object dicts store a
    `maskmem_features` GPU tensor per object per frame (set in `_add_output_per_object`),
    so pruning only `output_dict` leaves those references alive and VRAM grows unbounded.
    """
    state = getattr(predictor, 'inference_state', None)
    if not state:
        return

    def _trim_buckets(container, consolidated=None):
        for bucket in ('cond_frame_outputs', 'non_cond_frame_outputs'):
            od = container.get(bucket)
            if not od:
                continue
            sorted_keys = sorted(od.keys())
            to_remove = sorted_keys[:-keep_frames] if len(sorted_keys) > keep_frames else []
            ci = consolidated.get(bucket) if consolidated else None
            for k in to_remove:
                del od[k]
                if ci is not None:
                    ci.discard(k)

    # Shared output dict
    _trim_buckets(state.get('output_dict', {}), state.get('consolidated_frame_inds', {}))

    # Per-object dicts — these hold the bulk of the per-frame GPU maskmem tensors and
    # are the actual leak if left untrimmed.
    for per_obj in (state.get('output_dict_per_obj', {}), state.get('temp_output_dict_per_obj', {})):
        for obj_state in per_obj.values():
            if isinstance(obj_state, dict):
                _trim_buckets(obj_state)

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


# ---------------------------------------------------------------------------
# Node
# ---------------------------------------------------------------------------

class Sam2TrackerNode(Node):
    """2D-only tracker node: seed -> SAM2 tracking -> filtered grasp uv."""

    def __init__(self):
        super().__init__('sam2_tracker_node')

        # Parameters
        self.declare_parameter('executor_threads', 6)
        self.declare_parameter('rgb_topic', '/hand/rgb')
        self.declare_parameter('depth_topic', '/hand/depth')
        self.declare_parameter('visualize', True)
        self.declare_parameter('tracking_window_name', 'SAM2 Live Tracking')
        self.declare_parameter('segmentation_mask_topic', '/hand/segmentation_mask')
        self.declare_parameter('publish_segmentation_mask', True)
        # Dilate the mask published to nvblox by N pixels. SAM2 masks often undershoot
        # the object edge; that uncovered rim fuses into the STATIC TSDF and cuRobo
        # (which inflates obstacles by the collision-sphere buffer) avoids the target
        # it's reaching for. Dilating closes the rim so the whole object routes to the
        # dynamic mapper. Keep it tight (rim width, a few px) — over-dilating masks
        # real nearby obstacles as dynamic too. 0 = off. Applies to hand + secondaries.
        self.declare_parameter('mask_dilation_px', 4)
        # Hand-camera roll correction. The hand cam rolls about its optical axis as
        # the wrist rotates; SAM2's video predictor (which propagates appearance
        # frame-to-frame) loses lock when the object rotates between frames. We
        # de-roll every frame fed to the predictor to a gravity-upright orientation
        # (continuous, so the stateful memory stays in one consistent frame), then
        # re-roll the output mask back to native so it still aligns with the depth.
        # A small deadband skips the warp when the roll is negligible.
        self.declare_parameter('hand_deroll_enabled', True)
        self.declare_parameter('hand_deroll_min_deg', 3.0)   # deadband; below this = no warp
        self.declare_parameter('roll_reference_frame', 'body')  # gravity-up reference for the roll angle
        self.declare_parameter('active_tracking_interval', 0.2)
        self.declare_parameter('tracking_lost_confirm_frames', 5)
        self.declare_parameter('seed_command_topic', '/perception/seed_command')
        self.declare_parameter('tracking_state_topic', '/tracking_state')
        self.declare_parameter('tracking_3d_topic', '/tracking_3d_point')
        self.declare_parameter('depth_info_topic', '/hand/camera_info')
        self.declare_parameter('secondary_cameras', '')
        self.declare_parameter('secondary_rgb_topic_pattern', '/{cam}/rgb')
        self.declare_parameter('secondary_depth_topic_pattern', '/depth_registered/{cam}/image')
        self.declare_parameter('secondary_mask_topic_pattern', '/{cam}/segmentation_mask')
        # Tracker-side exact reproject: the seed (u,v) is computed from TF at each
        # frame's OWN stamp (time-consistent with the image SAM2 segments), instead
        # of trusting the async latest-TF seed_pixel from tf_projection.
        self.declare_parameter('secondary_camera_info_topic_pattern', '/{cam}/camera_info')
        self.declare_parameter('target_object_frame', 'target_object')
        self.declare_parameter('secondary_tf_timeout_sec', 0.05)
        self.declare_parameter('secondary_fov_timeout_sec', 3.0)
        self.declare_parameter('secondary_max_centroid_dist_px', 120.0)
        # When a secondary is CONFIRMED LOST it publishes zeros so the camera maps
        # the static scene. But on out->in FOV RE-ENTRY there is a 1-2 frame window
        # where the object is already visible and the SAM2 predictor has not yet
        # re-acquired it: those frames would fuse the object STATIC (the leak). While
        # lost we therefore protect a disc at the reprojected world-fixed target_object
        # location (it persists at the last world pose per tf_projection) so the
        # re-entering object is masked dynamic until the predictor takes over. 0 = off
        # (plain zeros, original behaviour). The disc only appears when target_object
        # actually reprojects INTO this camera, so a genuinely absent object still
        # yields all-zeros and the camera keeps mapping. Default 0 = OFF: the disc
        # relies on image-space reprojection (imperfect); the cuRobo-side ESDF clear
        # sphere (curobo_mpc_node target_clear_radius_m) is the preferred catch-all
        # because it is a pure 3D point transform of target_object, no intrinsics.
        self.declare_parameter('secondary_lost_guard_radius_px', 0)
        self.declare_parameter('secondary_fov_enter_count', 5)  # consecutive seeds to arm init
        self.declare_parameter('secondary_iou_radius_px', 80.0)   # radius (px) of the seed-neighbourhood disc
        # Min fraction of the seed-neighbourhood disc that the mask must cover
        # (size-invariant: does NOT penalise masks larger than the disc).
        self.declare_parameter('secondary_iou_min_overlap', 0.30)
        self.declare_parameter('secondary_max_mask_frac', 0.45)  # reject masks larger than this fraction of the image
        self.declare_parameter('secondary_min_mask_area_px', 100)  # reject masks smaller than this (noise specks)
        self.declare_parameter('secondary_seed_in_mask_tol_px', 6)  # seed must fall within this many px of the mask
        # Secondary COLD init box prompt: project the hand-tracked object's metric size
        # at the secondary's expected depth into a pixel box around the seed, so SAM2
        # segments the whole object instead of the sub-part under a single seed point.
        # 0 disables (falls back to point-only prompt). Scale pads the box a little so
        # the full object is enclosed despite viewpoint differences.
        self.declare_parameter('secondary_box_scale', 1.4)
        self.declare_parameter('secondary_box_metric_clamp_m', [0.03, 0.8])  # [min,max] object size sanity
        # Object clear-box: measure the target's 3D AABB (mask extent + depth) and
        # publish it as a Marker so cuRobo can clear a tight, object-sized box from its
        # ESDF collision world (cuMotion-style "GIGO" catch-all for residual leaks).
        self.declare_parameter('clear_box_enabled', True)
        self.declare_parameter('target_parent_frame', 'world')   # frame the box is published in
        self.declare_parameter('clear_box_topic', '/target_object/clear_box')
        self.declare_parameter('clear_box_depth_extent_min_m', 0.04)  # floor for the along-view (z) thickness
        self.declare_parameter('clear_box_depth_extent_max_m', 0.5)   # cap (reject depth outliers)
        self.declare_parameter('secondary_warm_promote_frames', 3)      # frames of valid tracking → COLD→WARM
        self.declare_parameter('secondary_warm_fail_max', 3)            # failed warm re-prompts → DEGRADED
        self.declare_parameter('secondary_iou_min_overlap_warm', 0.15)  # relaxed IoU for warm re-prompts
        self.declare_parameter('sam2_memory_reset_interval', 200)  # frames between memory resets
        self.declare_parameter('camera_speed_topic', '/hand/camera_speed')
        self.declare_parameter('hand_reinit_speed_gate_m_s', 0.08)
        self.declare_parameter('hand_reinit_speed_gate_timeout_s', 1.5)
        # How much a frame stamp may lag behind the TF lookup tnow and still be
        # accepted as "fresh enough" to init on. Image pipeline latency typically
        # puts the freshest frame ~200ms behind wall-clock when the TF was sampled.
        self.declare_parameter('hand_seed_pixel_stamp_tolerance_s', 0.5)
        # Max allowed |TF-lookup stamp - RGB frame stamp| for a secondary seed
        # pixel to be accepted as fresh. Image pipeline latency on the real robot
        # is consistently 200-350ms, so anything tighter than ~500ms blocks init.
        self.declare_parameter('secondary_seed_pixel_stale_ms', 500.0)
        # Depth-consistency gate for secondary COLD init: the secondaries are
        # monocular for seeding, so reproject only yields a *ray*. We sample an
        # NxN depth window around the projected pixel and reject the seed if the
        # observed foreground depth disagrees with the object's expected depth
        # (the reprojected ray range carried in the seed_pixel z magnitude) by
        # more than the tolerance — catches seeds that land on background.
        self.declare_parameter('secondary_seed_depth_tol_m', 0.30)
        self.declare_parameter('secondary_seed_depth_radius_px', 8)  # NxN window = (2r+1)²
        self.declare_parameter('hand_iou_radius_px', 100.0)
        self.declare_parameter('hand_iou_min_overlap', 0.25)
        self.declare_parameter('hand_max_centroid_dist_px', 150.0)

        self._executor_threads = int(self.get_parameter('executor_threads').value)
        self.visualize = self.get_parameter('visualize').value
        self.tracking_window_name = self.get_parameter('tracking_window_name').value
        self.publish_segmentation_mask = self.get_parameter('publish_segmentation_mask').value
        self._hand_deroll_enabled = bool(self.get_parameter('hand_deroll_enabled').value)
        self._hand_deroll_min_deg = float(self.get_parameter('hand_deroll_min_deg').value)
        self._roll_reference_frame = str(self.get_parameter('roll_reference_frame').value)
        self._mask_dilation_px = int(self.get_parameter('mask_dilation_px').value)
        self._mask_dilation_kernel = (
            cv2.getStructuringElement(
                cv2.MORPH_ELLIPSE,
                (2 * self._mask_dilation_px + 1, 2 * self._mask_dilation_px + 1),
            )
            if self._mask_dilation_px > 0 else None
        )
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
        secondary_depth_topic_pattern = str(self.get_parameter('secondary_depth_topic_pattern').value)
        secondary_mask_topic_pattern = str(self.get_parameter('secondary_mask_topic_pattern').value)
        secondary_camera_info_topic_pattern = str(self.get_parameter('secondary_camera_info_topic_pattern').value)
        self._target_object_frame = str(self.get_parameter('target_object_frame').value)
        self._secondary_tf_timeout_sec = float(max(0.0, self.get_parameter('secondary_tf_timeout_sec').value))
        # TF for exact per-frame reprojection of the world target into each camera.
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self, spin_thread=False)

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
        # Once an initial seed has produced a mask, the tracking step runs every tick
        # (predictor keeps inferring across LOST frames so it can recover from its own
        # memory). Only an explicit VLM retrigger clears it.
        self._hand_initialized_once = False
        # While a retrigger is pending (RELOCALIZING or new seed command), the per-frame
        # mask publisher emits NOTHING — neither stale nor zeros — so nvblox doesn't
        # integrate the old object as static during the swap.
        self._hand_retrigger_pending = False

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

        # Prompt change tracking
        self._hand_current_prompt_change_id = 0

        # Snapshot state for relocalization
        self._snapshot_rgb = None
        self._snapshot_depth = None
        self._snapshot_header = None
        self._snapshot_taken = False
        self._snapshot_run_idx = 0
        self._snapshot_temp_dir = _prepare_snapshot_temp_dir()

        # GPU lock shared between hand and secondary cameras
        self._gpu_lock = threading.Lock()
        # Performance timing: per-call (wait_ms, infer_ms) for each SAM2 inference,
        # plus a rolling window for periodic summary logs.
        self._infer_stats = {
            'hand': collections.deque(maxlen=60),
            'hand_init': collections.deque(maxlen=10),
        }
        # Secondary camera buffers are added below after self._secondary_cameras is parsed.
        self._last_infer_stats_log_t = 0.0
        self._infer_stats_log_interval_s = 2.0
        # Rate diagnostics: synced RGB+depth pair callbacks (the input ceiling) vs
        # actual tracking steps executed vs total step wall time. Splits "input/sync
        # starved" from "step too slow" when the mask rate is below expectations.
        self._synced_cb_count = 0
        self._tracking_step_count = 0
        self._tracking_step_ms_sum = 0.0
        self._last_rate_log_t = time.perf_counter()

        # Secondary cameras
        secondary_cameras_str = str(self.get_parameter('secondary_cameras').value)
        self._secondary_cameras = [c.strip() for c in secondary_cameras_str.split(',') if c.strip()]
        self._secondary_cam_state = {}
        self._secondary_mask_pubs = {}
        self._secondary_fov_timeout_sec = float(self.get_parameter('secondary_fov_timeout_sec').value)
        self._secondary_lost_guard_radius_px = int(self.get_parameter('secondary_lost_guard_radius_px').value)
        self._secondary_max_centroid_dist_px = float(self.get_parameter('secondary_max_centroid_dist_px').value)
        self._secondary_fov_enter_count = int(max(1, self.get_parameter('secondary_fov_enter_count').value))
        self._sam2_memory_reset_interval = int(max(1, self.get_parameter('sam2_memory_reset_interval').value))
        self._secondary_iou_radius_px = float(self.get_parameter('secondary_iou_radius_px').value)
        self._secondary_iou_min_overlap = float(self.get_parameter('secondary_iou_min_overlap').value)
        self._secondary_max_mask_frac = float(self.get_parameter('secondary_max_mask_frac').value)
        self._secondary_min_mask_area_px = int(max(1, self.get_parameter('secondary_min_mask_area_px').value))
        self._secondary_seed_in_mask_tol_px = int(max(0, self.get_parameter('secondary_seed_in_mask_tol_px').value))
        self._secondary_warm_promote_frames = int(max(1, self.get_parameter('secondary_warm_promote_frames').value))
        self._secondary_warm_fail_max = int(max(1, self.get_parameter('secondary_warm_fail_max').value))
        self._secondary_iou_min_overlap_warm = float(self.get_parameter('secondary_iou_min_overlap_warm').value)
        self._hand_reinit_speed_gate_m_s = float(self.get_parameter('hand_reinit_speed_gate_m_s').value)
        self._hand_reinit_speed_gate_timeout_s = float(self.get_parameter('hand_reinit_speed_gate_timeout_s').value)
        self._hand_seed_pixel_stamp_tolerance_s = float(self.get_parameter('hand_seed_pixel_stamp_tolerance_s').value)
        self._secondary_seed_pixel_stale_ms = float(self.get_parameter('secondary_seed_pixel_stale_ms').value)
        self._secondary_seed_depth_tol_m = float(self.get_parameter('secondary_seed_depth_tol_m').value)
        self._secondary_seed_depth_radius_px = int(max(1, self.get_parameter('secondary_seed_depth_radius_px').value))
        self._hand_iou_radius_px = float(self.get_parameter('hand_iou_radius_px').value)
        self._hand_iou_min_overlap = float(self.get_parameter('hand_iou_min_overlap').value)
        self._hand_max_centroid_dist_px = float(self.get_parameter('hand_max_centroid_dist_px').value)
        self._secondary_box_scale = float(self.get_parameter('secondary_box_scale').value)
        self._clear_box_enabled = bool(self.get_parameter('clear_box_enabled').value)
        self._target_parent_frame = str(self.get_parameter('target_parent_frame').value)
        self._clear_box_depth_min = float(self.get_parameter('clear_box_depth_extent_min_m').value)
        self._clear_box_depth_max = float(self.get_parameter('clear_box_depth_extent_max_m').value)
        _box_clamp = list(self.get_parameter('secondary_box_metric_clamp_m').value)
        self._secondary_box_metric_min = float(_box_clamp[0]) if len(_box_clamp) > 0 else 0.03
        self._secondary_box_metric_max = float(_box_clamp[1]) if len(_box_clamp) > 1 else 0.8
        # Object's characteristic metric size (m), measured from the hand mask+depth at
        # hand init; reused to size the secondaries' box prompt. None until hand inits.
        self._hand_object_metric_size = None

        # Callback groups
        # - io_cb_group: most subscriptions (Reentrant — they're fast, can run concurrently with each other and with inference)
        # - hand_sync_cb_group: hand RGB+depth message_filters subs (MutuallyExclusive — ApproximateTimeSynchronizer
        #   internal state is NOT thread-safe; both filter callbacks must serialize)
        # - inference_cb_group_hand: hand 10 Hz inference timer (MutuallyExclusive)
        # - _secondary_inference_groups[cam]: per-secondary 5 Hz inference timer, each its own MutuallyExclusive group
        # - gui_cb_group: the single thread that owns cv2.imshow/waitKey (Qt is not thread-safe across executor workers)
        self.io_cb_group = ReentrantCallbackGroup()
        self.hand_sync_cb_group = MutuallyExclusiveCallbackGroup()
        self.inference_cb_group_hand = MutuallyExclusiveCallbackGroup()
        self.gui_cb_group = MutuallyExclusiveCallbackGroup()
        self._secondary_inference_groups: dict = {}

        # Pending GUI frames: window_name -> BGR ndarray. _update_display / _update_secondary_display
        # only write here under _gui_lock; the gui_cb_group timer below is the only thread that
        # calls cv2.imshow/waitKey, which keeps Qt happy.
        self._gui_lock = threading.Lock()
        self._pending_display_frames: dict = {}

        # Publishers
        self.mask_pub = self.create_publisher(RosImage, mask_topic, 10)
        self._tracking_state_pub = self.create_publisher(String, tracking_state_topic, 10)
        self._tracking_2d_pub = self.create_publisher(PointStamped, tracking_point_topic, 10)
        self._seed_3d_pub = self.create_publisher(PointStamped, "/tracking/seed_3d", 10)
        self._clear_box_pub = (
            self.create_publisher(Marker, str(self.get_parameter('clear_box_topic').value), 10)
            if self._clear_box_enabled else None)

        # RGB+depth sync via ApproximateTimeSynchronizer
        # Both filter subs share a single MutuallyExclusiveCallbackGroup because the
        # ApproximateTimeSynchronizer's internal book-keeping is not thread-safe.
        self.color_sub = message_filters.Subscriber(
            self, RosImage, rgb_topic, callback_group=self.hand_sync_cb_group
        )
        self.depth_sub = message_filters.Subscriber(
            self, RosImage, depth_topic, callback_group=self.hand_sync_cb_group
        )
        self.sync = message_filters.ApproximateTimeSynchronizer(
            [self.color_sub, self.depth_sub], queue_size=10, slop=0.1
        )
        self.sync.registerCallback(self._synced_image_cb)

        self._seed_sub = self.create_subscription(
            String, seed_topic, self._seed_command_cb, 10, callback_group=self.io_cb_group
        )
        self._cam_info_sub = self.create_subscription(
            CameraInfo, depth_info_topic, self._camera_info_cb, 10, callback_group=self.io_cb_group
        )
        self._hand_cam_speed = None
        self.create_subscription(
            Float64,
            str(self.get_parameter('camera_speed_topic').value),
            self._cam_speed_cb,
            10,
            callback_group=self.io_cb_group,
        )
        self._coord_state_sub = self.create_subscription(
            String, "/coordinator/state", self._coordinator_state_cb, 10,
            callback_group=self.io_cb_group,
        )
        self._seed_pixel_sub = self.create_subscription(
            PointStamped, "/tracking/seed_pixel", self._seed_pixel_cb, 10,
            callback_group=self.io_cb_group,
        )
        self._prompt_change_sub = self.create_subscription(
            String, "/vlm/prompt_change_id", self._prompt_change_cb, 10,
            callback_group=self.io_cb_group,
        )
        # Geometry depth fallback: vision-frame object position reprojected to hand cam by tf_projection
        self._geometry_cam_pt: tuple = None  # (x, y, z) in hand_cam frame, TF-derived (no depth sensor)
        self.create_subscription(
            PointStamped, "/tracking/geometry_3d_in_cam", self._geometry_cam_pt_cb, 10,
            callback_group=self.io_cb_group,
        )

        # Register secondary cameras
        for cam in self._secondary_cameras:
            self._infer_stats[cam] = collections.deque(maxlen=60)
            self._infer_stats[f'{cam}_init'] = collections.deque(maxlen=10)
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
                'latest_depth_msg': None,      # raw depth Image msg; converted lazily at init only
                'latest_depth_header': None,
                'expected_depth': None,        # object range (m) from the per-frame reproject
                'cam_intrinsics': None,        # (fx, fy, cx, cy) from camera_info
                'cam_frame_id': None,          # camera optical TF frame for reprojection
                'cam_size': None,              # (w, h) for image-bounds check
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
                # Latches True the first time SAM2 init succeeds. After that, the
                # per-frame publisher emits _last_mask_np (or zeros if None) every
                # frame — the FOV/arming gate only governs the very first seed.
                '_initialized_once': False,
                # Suppress publishing during a VLM retrigger (after first init only)
                # so old-object pixels don't leak into the TSDF while waiting for
                # the new init to produce a mask.
                '_retrigger_pending': False,
                'lifecycle': 'COLD',      # 'COLD' | 'WARM' | 'DEGRADED'
                'warm_tracked_frames': 0, # successful frames since current init (for COLD→WARM promotion)
                'warm_fail_streak': 0,    # consecutive warm re-prompt failures (→ DEGRADED)
                # Guards latest_rgb/header/new_frame_available now that RGB callback (io_cb_group)
                # and the 5 Hz inference timer (per-cam MutuallyExclusive) run on separate threads.
                '_rgb_lock': threading.Lock(),
                '_depth_lock': threading.Lock(),
            }
            self.create_subscription(
                RosImage, secondary_rgb_topic,
                lambda msg, c=cam: self._secondary_rgb_cb(msg, c), 10,
                callback_group=self.io_cb_group,
            )
            secondary_depth_topic = secondary_depth_topic_pattern.replace('{cam}', cam)
            self.create_subscription(
                RosImage, secondary_depth_topic,
                lambda msg, c=cam: self._secondary_depth_cb(msg, c), 10,
                callback_group=self.io_cb_group,
            )
            secondary_caminfo_topic = secondary_camera_info_topic_pattern.replace('{cam}', cam)
            self.create_subscription(
                CameraInfo, secondary_caminfo_topic,
                lambda msg, c=cam: self._secondary_camera_info_cb(msg, c), 10,
                callback_group=self.io_cb_group,
            )
            self.create_subscription(
                PointStamped, f'/{cam}/tracking/seed_pixel',
                lambda msg, c=cam: self._secondary_seed_pixel_cb(msg, c), 10,
                callback_group=self.io_cb_group,
            )
            secondary_mask_topic = secondary_mask_topic_pattern.replace('{cam}', cam)
            self._secondary_mask_pubs[cam] = self.create_publisher(
                RosImage, secondary_mask_topic, 10
            )
            # Per-camera inference timer (5 Hz) on its own MutuallyExclusiveCallbackGroup so
            # secondaries do not block each other or the hand timer at the executor level.
            self._secondary_inference_groups[cam] = MutuallyExclusiveCallbackGroup()
            self.create_timer(
                0.2,
                lambda c=cam: self._run_secondary_tracking_step(c),
                callback_group=self._secondary_inference_groups[cam],
            )
        if self._secondary_cameras:
            self.get_logger().info(
                f"Secondary cameras registered: {self._secondary_cameras}, rgb_pattern={secondary_rgb_topic_pattern}"
            )

        self.tracking_timer = self.create_timer(
            0.1, self._tracking_timer_cb, callback_group=self.inference_cb_group_hand
        )

        # GUI is NOT driven from a ROS timer: MultiThreadedExecutor would still run
        # successive ticks on different worker threads, and Qt windows are bound to
        # the thread that created them. Instead, main() spins the executor in a daemon
        # thread and drains _pending_display_frames on the process main thread.

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

    def _prompt_change_cb(self, msg: String):
        """Handle prompt change notifications from VLM node."""
        new_id = int(msg.data)
        if new_id > self._hand_current_prompt_change_id:
            self._hand_current_prompt_change_id = new_id
            self.get_logger().info(
                f"[TRACKER] Prompt change detected (id={new_id}), "
                "will use new object on next VLM seed"
            )

    def _coordinator_state_cb(self, msg: String):
        state = msg.data.strip().upper()
        if state == "RELOCALIZING" and not self._snapshot_taken:
            # A new VLM request is starting. Any pending seed pixel from a prior VLM
            # request is now obsolete. Clear it so we don't initialize on a stale seed.
            self._pending_seed_pixel_uv = None
            self._pending_seed_pixel_tnow = None
            # Retrigger: pause mask publishing and drop the cached mask so the per-frame
            # publisher emits nothing until the new init produces a mask. Also flip
            # tracking_active=False so the next seed_pixel (republished by tf_projection
            # after the VLM seed_3d) is consumed by _synced_image_cb instead of being
            # discarded as a duplicate of an active track.
            if self._hand_initialized_once:
                self._hand_retrigger_pending = True
            self._last_mask_np = None
            self.tracking_active = False

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

    def _run_sam2_timed(self, label: str, predictor, *args, **kwargs):
        """Run a SAM2 inference call with timing instrumentation.

        Reports lock-wait, GPU inference (sync'd via cuda.synchronize), and
        appends (wait_ms, infer_ms) to self._infer_stats[label]. Periodically
        emits a summary so we can see which cameras are blocking the queue.
        """
        t_req = time.perf_counter()
        with self._gpu_lock:
            t_acq = time.perf_counter()
            try:
                if torch.cuda.is_available():
                    torch.cuda.synchronize()
                t_start = time.perf_counter()
                results = predictor(*args, **kwargs)
                if torch.cuda.is_available():
                    torch.cuda.synchronize()
                t_end = time.perf_counter()
            except Exception:
                raise
        wait_ms = (t_acq - t_req) * 1000.0
        infer_ms = (t_end - t_start) * 1000.0
        buf = self._infer_stats.get(label)
        if buf is not None:
            buf.append((wait_ms, infer_ms))
        # Periodic summary across all cameras
        now = time.perf_counter()
        if now - self._last_infer_stats_log_t >= self._infer_stats_log_interval_s:
            self._last_infer_stats_log_t = now
            parts = []
            for k, q in self._infer_stats.items():
                if not q or k.endswith('_init'):
                    continue
                waits = [w for w, _ in q]
                infers = [i for _, i in q]
                parts.append(
                    f"{k}: n={len(q)} wait={sum(waits)/len(waits):.0f}/"
                    f"{max(waits):.0f}ms infer={sum(infers)/len(infers):.0f}/"
                    f"{max(infers):.0f}ms (avg/max)"
                )
            if parts:
                self.get_logger().info("[PERF] " + " | ".join(parts))
        return results, wait_ms, infer_ms

    def _maybe_log_rates(self):
        """Periodically log the input ceiling vs actual step rate vs step wall time.

        synced_pairs = rate the RGB+depth ApproximateTimeSync fires (the hard input
        ceiling); tracking_steps = rate a step actually executed; step_total = avg
        wall ms per step (infer is a subset, reported in [PERF]).
        - synced_pairs ≈ tracking_steps and both low  → INPUT/sync bound (widen slop,
          fix Isaac stamp alignment).
        - synced_pairs high but tracking_steps low + step_total high → STEP bound.
        """
        now = time.perf_counter()
        dt = now - self._last_rate_log_t
        if dt < 2.0:
            return
        synced_hz = self._synced_cb_count / dt
        step_hz = self._tracking_step_count / dt
        step_ms = (self._tracking_step_ms_sum / self._tracking_step_count
                   if self._tracking_step_count else 0.0)
        self.get_logger().info(
            f"[RATE] synced_pairs={synced_hz:.1f}Hz tracking_steps={step_hz:.1f}Hz "
            f"step_total={step_ms:.0f}ms (avg)"
        )
        self._synced_cb_count = 0
        self._tracking_step_count = 0
        self._tracking_step_ms_sum = 0.0
        self._last_rate_log_t = now

    def _geometry_cam_pt_cb(self, msg: PointStamped):
        """Store the latest TF-derived object position in hand camera frame (no depth sensor needed)."""
        self._geometry_cam_pt = (float(msg.point.x), float(msg.point.y), float(msg.point.z))

    def _synced_image_cb(self, rgb_msg: RosImage, depth_msg: RosImage):
        """Recebe par RGB+depth sincronizado e armazena para o tracking loop."""
        self._synced_cb_count += 1
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
            else:
                clock_now = self.get_clock().now().nanoseconds * 1e-9
                age = clock_now - self._pending_seed_pixel_clock_t
                if age > self._hand_reinit_speed_gate_timeout_s:
                    self.get_logger().warn(
                        f"[TRACKER] seed_pixel expired after {age:.1f}s "
                        f"(stamp_sec={stamp_sec:.3f} tnow={self._pending_seed_pixel_tnow:.3f}) — discarding"
                    )
                    self._pending_seed_pixel_uv = None
                    self._pending_seed_pixel_tnow = None
                elif stamp_sec >= self._pending_seed_pixel_tnow - self._hand_seed_pixel_stamp_tolerance_s:
                    # Speed gate applies ONLY to the first cold init (no SAM2 memory yet),
                    # where a seed on a fast-moving camera risks anchoring the predictor on
                    # the wrong pixel. A reinit is a WARM re-prompt: SAM2's memory + IoU
                    # validation already reject a bad re-prompt, so gating it on camera speed
                    # only adds latency (object visible-but-unmasked → leaks into static TSDF).
                    if (not self._hand_initialized_once
                          and self._hand_cam_speed is not None
                          and self._hand_cam_speed > self._hand_reinit_speed_gate_m_s):
                        self.get_logger().info(
                            f"[TRACKER] first-init deferred: cam_speed={self._hand_cam_speed:.3f} m/s "
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

            with self._gui_lock:
                self._pending_display_frames[self.tracking_window_name] = img_bgr
        except Exception as e:
            self.get_logger().warn(f"Visualization error: {e}")

    def drain_pending_display_frames(self) -> list:
        """Return (window_name, bgr) pairs queued since last call. Thread-safe.
        Intended for the process main thread (see main()), which is the only
        thread allowed to call cv2.imshow under Qt.
        """
        with self._gui_lock:
            frames = list(self._pending_display_frames.items())
            self._pending_display_frames.clear()
        return frames

    def _hand_roll_deg(self) -> float:
        """Roll (deg) of the hand camera about its optical axis vs gravity-up.

        Returns 0.0 when correction is disabled, the camera frame is unknown, TF is
        unavailable, or the angle is within the deadband — in all those cases the
        frame is fed to SAM2 natively (no warp)."""
        if not self._hand_deroll_enabled or self.camera_frame_id is None:
            return 0.0
        try:
            t = self.tf_buffer.lookup_transform(
                self._roll_reference_frame, self.camera_frame_id,
                rclpy.time.Time(), timeout=Duration(seconds=0.0),
            )
        except Exception:
            return 0.0
        ang = roll_deg_from_quaternion(
            t.transform.rotation.x, t.transform.rotation.y,
            t.transform.rotation.z, t.transform.rotation.w,
        )
        return ang if abs(ang) > self._hand_deroll_min_deg else 0.0

    def _queue_debug_window(self, name: str, img_bgr, roll_angle=None) -> None:
        """Thread-safe push of a BGR frame to a named cv2 debug window (drained on
        the GUI thread). No-op when not visualizing. If roll_angle is given, the
        applied correction angle + reference frame are overlaid so the de-roll can
        be validated visually (objects should stay upright as the wrist rolls)."""
        if not self.visualize:
            return
        if roll_angle is not None:
            txt = f"roll={roll_angle:+.1f} deg  ref={self._roll_reference_frame}"
            cv2.putText(img_bgr, txt, (10, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 3)
            cv2.putText(img_bgr, txt, (10, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 1)
        with self._gui_lock:
            self._pending_display_frames[name] = img_bgr

    def _dilate_mask_for_nvblox(self, m):
        """Grow the mask by mask_dilation_px so the object's rim is fully covered.

        SAM2 masks tend to undershoot the object edge; the uncovered shell fuses into
        the static TSDF and cuRobo avoids it. Dilation closes that gap before the mask
        reaches nvblox. Only applied to non-empty masks (an all-zero 'object absent'
        mask must stay all-zero). No-op when mask_dilation_px <= 0.
        """
        if self._mask_dilation_kernel is None or m is None or not m.any():
            return m
        return cv2.dilate(m, self._mask_dilation_kernel, iterations=1)

    def _publish_mask_for_header(self, header):
        """Publish ZEROS for this frame when NOT actively tracking.

        Non-empty masks are published only by _publish_segmentation_mask, stamped
        with the SOURCE frame SAM2 actually segmented. Re-stamping a stale silhouette
        onto a newer frame is what leaked the moving object into the static TSDF, so
        we no longer republish the cached mask here. The non-empty masks arrive at the
        SAM2 inference rate (~5 Hz); nvblox's approximate depth+mask sync pairs each
        with its own depth frame and drops the in-between frames (they are not fused).

        When NOT tracking (_last_mask_np is None: never seeded / LOST), publishing
        zeros lets the object-free scene fuse as static — empty masks are timeless, so
        re-stamping them to the current frame is harmless. While a retrigger is pending
        we publish nothing, so the old object is not fused as static during the swap.
        """
        if not self.publish_segmentation_mask:
            return
        if self._hand_retrigger_pending:
            return
        if self._last_mask_np is not None:
            return  # tracking: fresh mask already published, source-stamped
        try:
            h, w = self._last_mask_shape if self._last_mask_shape else (480, 640)
            m = np.zeros((h, w), dtype=np.uint8)
            msg = self.bridge.cv2_to_imgmsg(m, encoding='mono8')
            msg.header = header
            self.mask_pub.publish(msg)
        except Exception as e:
            self.get_logger().error(f"Error publishing segmentation mask: {e}")

    def _publish_segmentation_mask(self, mask_np, src_header=None):
        """Cache the latest hand mask and publish it stamped with its SOURCE frame.

        The mask carries the timestamp of the frame SAM2 segmented, so nvblox's
        approximate depth+mask sync pairs it with that exact depth frame instead of a
        newer one — no spatial staleness, no leak under motion. Caching keeps
        _last_mask_np for state/recovery logic; the per-frame publisher uses it only
        to decide whether to emit zeros (object absent) vs. nothing (tracking).
        """
        if not self.publish_segmentation_mask:
            return
        self._last_mask_np = mask_np
        if src_header is None or mask_np is None:
            return
        try:
            m = ((mask_np > 0).astype(np.uint8) * 255)
            m = self._dilate_mask_for_nvblox(m)
            msg = self.bridge.cv2_to_imgmsg(m, encoding='mono8')
            msg.header = src_header
            self.mask_pub.publish(msg)
        except Exception as e:
            self.get_logger().error(f"Error publishing segmentation mask: {e}")

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

    def _snap_point_into_mask(self, u, v, mask_np):
        """Return (u, v) guaranteed to lie inside mask_np.

        If (u, v) is already a mask pixel it's returned unchanged; otherwise the
        nearest mask pixel is returned. This keeps the grasp point (mask centroid +
        the VLM grasp offset) ON the object so its depth sample is foreground, never
        the background behind it — a fixed pixel offset can otherwise walk off the
        object under motion/shape change → background depth → metres of world drift.
        Returns None if the mask is empty.
        """
        h, w = mask_np.shape[:2]
        uu = min(max(0, int(u)), w - 1)
        vv = min(max(0, int(v)), h - 1)
        if mask_np[vv, uu] > 0:
            return (uu, vv)
        ys, xs = np.where(mask_np > 0)
        if len(xs) == 0:
            return None
        i = int(np.argmin((xs - uu) ** 2 + (ys - vv) ** 2))
        return (int(xs[i]), int(ys[i]))

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
            # Retrigger: pause mask publishing until the new init produces a mask.
            # tracking_active=False so the upcoming seed_pixel (after seed_3d → tf_projection)
            # is consumed instead of discarded as a duplicate of an active track.
            # On the very first VLM call _hand_initialized_once is still False — keep
            # publishing zeros so nvblox keeps integrating instead of starving.
            if self._hand_initialized_once:
                self._hand_retrigger_pending = True
            self._last_mask_np = None
            self.tracking_active = False
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

    @staticmethod
    def _quat_to_matrix(qx, qy, qz, qw):
        """3x3 rotation matrix from quaternion (x,y,z,w)."""
        n = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
        if n == 0.0:
            return np.eye(3)
        qx, qy, qz, qw = qx / n, qy / n, qz / n, qw / n
        return np.array([
            [1 - 2 * (qy * qy + qz * qz), 2 * (qx * qy - qz * qw), 2 * (qx * qz + qy * qw)],
            [2 * (qx * qy + qz * qw), 1 - 2 * (qx * qx + qz * qz), 2 * (qy * qz - qx * qw)],
            [2 * (qx * qz - qy * qw), 2 * (qy * qz + qx * qw), 1 - 2 * (qx * qx + qy * qy)],
        ])

    def _measure_and_publish_clear_box(self, mask_np, depth_np, header):
        """Measure the tracked object's 3D AABB (mask bbox + depth → metres in the
        camera optical frame), transform it into target_parent_frame, and publish it
        as a CUBE Marker. cuRobo consumes this to clear an object-sized box from its
        ESDF collision world (so residual leaks near the target don't make it stand
        off). Pure 3D transform — no image-space reprojection. No-op if disabled or
        the inputs/intrinsics/TF are unavailable.

        AABB axes in the camera frame: x=mask width, y=mask height, z=along-view depth
        spread (10th–90th pct of masked depth, floored/capped). The box is re-enclosed
        to an axis-aligned box in target_parent_frame via the half-extent rule
        h'_i = sum_j |R_ij| h_j, with the centre transformed by R·c + t.
        """
        if self._clear_box_pub is None or header is None:
            return
        if depth_np is None or mask_np is None:
            return
        if self.camera_intrinsics is None or self.camera_frame_id is None:
            return
        try:
            ys, xs = np.where(mask_np > 0)
            if xs.size < 10:
                return
            fx, fy, cx, cy = self.camera_intrinsics
            # masked depth values (resample mask to depth resolution if needed)
            mh, mw = mask_np.shape[:2]
            dh, dw = depth_np.shape[:2]
            mask_d = (cv2.resize(mask_np, (dw, dh), interpolation=cv2.INTER_NEAREST)
                      if (dw, dh) != (mw, mh) else mask_np)
            dvals = depth_np[mask_d > 0]
            dvals = dvals[np.isfinite(dvals) & (dvals > 0.0)]
            if dvals.size < 5:
                return
            z_lo, z_mid, z_hi = np.percentile(dvals, [10.0, 50.0, 90.0])
            z_mid = float(z_mid)
            if z_mid <= 0.0:
                return
            u_lo, u_hi = float(xs.min()), float(xs.max())
            v_lo, v_hi = float(ys.min()), float(ys.max())
            u_c, v_c = 0.5 * (u_lo + u_hi), 0.5 * (v_lo + v_hi)
            # half-extents in the camera optical frame (metres)
            hx = 0.5 * (u_hi - u_lo + 1.0) * z_mid / fx
            hy = 0.5 * (v_hi - v_lo + 1.0) * z_mid / fy
            hz = 0.5 * float(np.clip(z_hi - z_lo, self._clear_box_depth_min,
                                     self._clear_box_depth_max))
            # box centre in the camera optical frame (bbox centre at median depth)
            cxm = (u_c - cx) * z_mid / fx
            cym = (v_c - cy) * z_mid / fy
            center_cam = np.array([cxm, cym, z_mid])
            half_cam = np.array([hx, hy, hz])
            # camera optical frame -> target_parent_frame
            try:
                tf = self.tf_buffer.lookup_transform(
                    self._target_parent_frame, self.camera_frame_id,
                    rclpy.time.Time.from_msg(header.stamp),
                    timeout=Duration(seconds=0.1))
            except (ExtrapolationException, ConnectivityException):
                tf = self.tf_buffer.lookup_transform(
                    self._target_parent_frame, self.camera_frame_id,
                    rclpy.time.Time(), timeout=Duration(seconds=0.1))
            t = tf.transform.translation
            q = tf.transform.rotation
            R = self._quat_to_matrix(q.x, q.y, q.z, q.w)
            center_w = R @ center_cam + np.array([t.x, t.y, t.z])
            half_w = np.abs(R) @ half_cam  # AABB re-enclosure
            mk = Marker()
            mk.header.frame_id = self._target_parent_frame
            mk.header.stamp = header.stamp
            mk.ns = 'target_clear_box'
            mk.id = 0
            mk.type = Marker.CUBE
            mk.action = Marker.ADD
            mk.pose.position.x = float(center_w[0])
            mk.pose.position.y = float(center_w[1])
            mk.pose.position.z = float(center_w[2])
            mk.pose.orientation.w = 1.0
            mk.scale.x = float(max(2.0 * half_w[0], 1e-3))
            mk.scale.y = float(max(2.0 * half_w[1], 1e-3))
            mk.scale.z = float(max(2.0 * half_w[2], 1e-3))
            mk.color.r, mk.color.g, mk.color.b, mk.color.a = 1.0, 0.5, 0.0, 0.35
            self._clear_box_pub.publish(mk)
        except Exception as exc:
            self.get_logger().warn(f"clear-box measure/publish failed: {exc}",
                                   throttle_duration_sec=2.0)

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
        """Initialize (or reinitialize) the SAM2 video predictor on the given frame.

        Three modes:
          - First init: lazy-load predictor, full reset of inference_state.
          - VLM retrigger (`_hand_retrigger_pending=True`): full reset — the user/VLM
            has redirected to a new object.
          - Warm re-prompt (`_hand_initialized_once=True`, not retriggering):
            PRESERVE inference_state so the first-mask memory keeps anchoring the
            predictor. Mirrors the secondaries' WARM lifecycle.
        """
        img_np = np.array(img_pil.convert("RGB"))

        # Lazy-init video predictor
        if self.video_predictor is None:
            self.get_logger().info("Lazy-loading SAM2 VideoPredictor...")
            self.video_predictor = _load_sam_video_model()
            if self.video_predictor is None:
                self.get_logger().error("Failed to load SAM2 VideoPredictor")
                return

        is_warm = self._hand_initialized_once and not self._hand_retrigger_pending
        if is_warm:
            # Warm re-prompt: keep inference_state (first-mask memory). Trim so memory
            # growth stays bounded; the new (u,v) anchors the next prediction.
            _trim_sam2_memory(self.video_predictor, keep_frames=6)
            self.get_logger().info(
                f"[HAND] WARM re-prompt at ({u},{v}) — preserving inference_state",
                throttle_duration_sec=1.0,
            )
        else:
            # Cold (first init or VLM retrigger): clean slate.
            self.video_predictor.inference_state = {}
            self.video_predictor._ros_frame_idx = 0

        # De-roll to upright for the predictor (so init + tracking share one memory
        # orientation); transform the prompt pixel into upright space, then re-roll
        # the produced mask back to native. (u, v) stays native for the IoU check,
        # publish, and depth below. angle==0 (disabled / deadband) → native path.
        orig_w, orig_h = img_pil.size
        roll_angle = self._hand_roll_deg()
        if roll_angle != 0.0:
            img_up, M_fwd, _ = rotate_image_upright(img_pil, roll_angle)
            init_img_np = np.array(img_up.convert("RGB"))
            pt = M_fwd @ np.array([float(u), float(v), 1.0])
            prompt_uv = [[float(pt[0]), float(pt[1])]]
            self._queue_debug_window(
                "Hand Upright (de-rolled)", cv2.cvtColor(init_img_np, cv2.COLOR_RGB2BGR),
                roll_angle=roll_angle)
        else:
            M_fwd = None
            init_img_np = img_np
            prompt_uv = [[u, v]]
        init_results, _wait, _infer = self._run_sam2_timed(
            'hand_init', self.video_predictor, init_img_np, points=prompt_uv, labels=[1]
        )
        best_tracked_mask, score = _best_mask_from_results(init_results)
        if best_tracked_mask is None:
            self.get_logger().error("Video predictor init produced no mask from reprojected pixel")
            return
        if M_fwd is not None:
            best_tracked_mask = reroll_mask_to_original(
                best_tracked_mask.astype(np.uint8), M_fwd, (orig_w, orig_h))

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
        self._hand_initialized_once = True
        self._hand_retrigger_pending = False
        self._publish_segmentation_mask(mask_np, header)

        # Measure the object's characteristic metric size from the hand mask + depth.
        # The secondaries reuse it to size a box prompt (see _run_secondary_tracking_step),
        # so they segment the whole object instead of a sub-part under a single seed point.
        if self._secondary_box_scale > 0.0:
            try:
                ys, xs = np.where(mask_np > 0)
                if xs.size > 0 and self.camera_intrinsics is not None:
                    z_obj, ok_z = self._median_depth_under_mask(depth_np, mask_np)
                    if ok_z and z_obj > 0.0:
                        fx_h, fy_h, _, _ = self.camera_intrinsics
                        w_px = float(xs.max() - xs.min() + 1)
                        h_px = float(ys.max() - ys.min() + 1)
                        size_m = max(w_px * z_obj / fx_h, h_px * z_obj / fy_h)
                        size_m = min(self._secondary_box_metric_max,
                                     max(self._secondary_box_metric_min, size_m))
                        self._hand_object_metric_size = size_m
                        self.get_logger().info(
                            f"[HAND] object metric size ≈ {size_m:.3f} m "
                            f"(bbox {int(w_px)}x{int(h_px)}px @ {z_obj:.2f}m) — "
                            f"secondaries will box-prompt",
                            throttle_duration_sec=2.0,
                        )
            except Exception as _exc:
                self.get_logger().warn(f"[HAND] metric-size measure failed: {_exc}")

        self._measure_and_publish_clear_box(mask_np, depth_np, header)

        m = cv2.moments(mask_np)
        if m["m00"] != 0:
            mask_u = int(m["m10"] / m["m00"])
            mask_v = int(m["m01"] / m["m00"])
            self.grasp_offset = (u - mask_u, v - mask_v)
        else:
            mask_u, mask_v = u, v
            self.grasp_offset = (0, 0)

        # Keep the seed inside the mask so its depth sample is foreground, not the
        # background behind the object (see _snap_point_into_mask / Q1 fix).
        snapped = self._snap_point_into_mask(u, v, mask_np)
        grasp_u0, grasp_v0 = snapped if snapped is not None else (mask_u, mask_v)

        point_cam = self._pixel_to_camera_point(grasp_u0, grasp_v0, depth_np)
        if point_cam is None:
            # On-mask grasp pixel may be in a depth hole; fall back to mask centroid or mask pixels
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
        self._hand_disp_centroid = (grasp_u0, grasp_v0)
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
        self._tracking_step_count += 1
        _step_t0 = time.perf_counter()
        try:
            self._run_tracking_step_2d_body(now)
        finally:
            self._tracking_step_ms_sum += (time.perf_counter() - _step_t0) * 1000.0
            self._maybe_log_rates()

    def _run_tracking_step_2d_body(self, now: float):
        img_pil = self.latest_rgb
        # Snapshot the source frame's header alongside the frame so the produced mask
        # is published with the timestamp of the frame SAM2 actually segmented (not a
        # newer one that arrived during inference) — this is what makes nvblox pair it
        # with the correct depth frame instead of a stale silhouette.
        src_header = self._latest_rgb_header
        # De-roll the frame to gravity-upright before SAM2 so the video predictor's
        # memory stays in one consistent orientation through wrist roll. The output
        # mask is re-rolled back to native below so all downstream pixel/depth logic
        # and nvblox are unchanged. angle==0 (disabled / deadband) → native path.
        orig_w, orig_h = img_pil.size
        roll_angle = self._hand_roll_deg()
        if roll_angle != 0.0:
            img_up, M_fwd, _ = rotate_image_upright(img_pil, roll_angle)
            img_np = np.array(img_up.convert("RGB"))
            self._queue_debug_window(
                "Hand Upright (de-rolled)", cv2.cvtColor(img_np, cv2.COLOR_RGB2BGR),
                roll_angle=roll_angle)
        else:
            M_fwd = None
            img_np = np.array(img_pil.convert("RGB"))
        tracked_results, _wait, _infer = self._run_sam2_timed(
            'hand', self.video_predictor, img_np
        )
        best_tracked_mask, score = _best_mask_from_results(tracked_results)
        if best_tracked_mask is not None and M_fwd is not None:
            # Map the upright mask back to the native camera orientation.
            best_tracked_mask = reroll_mask_to_original(
                best_tracked_mask.astype(np.uint8), M_fwd, (orig_w, orig_h))

        def _enter_lost(reason: str):
            """LOST does not stop the predictor — only suppresses the published mask.
            The per-frame publisher emits zeros (because _last_mask_np is None) and
            tracking_active flips to False so the next tf_projection seed_pixel is
            consumed by _synced_image_cb → warm re-prompt (preserves inference_state
            so the first-mask memory is reused). Mirrors the secondaries' WARM path.
            """
            self._tracking_lost_streak += 1
            if self._tracking_lost_streak >= self.tracking_lost_confirm_frames:
                if self._hand_disp_state != 'LOST':
                    self.get_logger().warn(f"[HAND] Tracking lost — {reason}")
                self._hand_disp_state = 'LOST'
                self._hand_disp_mask = None
                self._hand_disp_centroid = None
                self._last_mask_np = None
                self.tracking_active = False

        if best_tracked_mask is None:
            self.get_logger().warn(
                f"[HAND] No mask from SAM2 (lost_streak={self._tracking_lost_streak + 1}/{self.tracking_lost_confirm_frames})",
                throttle_duration_sec=1.0,
            )
            _enter_lost("no mask")
            return

        mask_np = best_tracked_mask.astype(np.uint8)
        self.last_tracking_score = float(score)

        m = cv2.moments(mask_np)
        if m["m00"] == 0:
            _enter_lost("empty mask")
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
                self.get_logger().warn(
                    f"[HAND] Centroid ({u},{v}) too far from "
                    f"expected ({eu:.0f},{ev:.0f}), dist={dist:.0f}px — background?",
                    throttle_duration_sec=1.0,
                )
                _enter_lost("centroid too far")
                return

        # Mask accepted — recovery from LOST (if any) is complete.
        if self._hand_disp_state == 'LOST':
            self.get_logger().info("[HAND] Tracking recovered")
        self._tracking_lost_streak = 0
        self._hand_disp_state = 'TRACKING'
        self.tracking_active = True
        self.tracking_frame_count += 1
        self._publish_segmentation_mask(mask_np, src_header)
        self._measure_and_publish_clear_box(mask_np, self.latest_depth, src_header)

        # Trim old SAM2 memory every N frames — keeps only the last 6 frames in
        # inference_state so GPU usage stays bounded without any reseed or state reset.
        if self.tracking_frame_count % self._sam2_memory_reset_interval == 0:
            _trim_sam2_memory(self.video_predictor, keep_frames=6)
            self.get_logger().info(
                f"[TRACKER] SAM2 memory trimmed at frame {self.tracking_frame_count}"
            )
        mask_centroid_u, mask_centroid_v = u, v
        # Grasp point = mask centroid + the VLM grasp offset, but CLAMPED to stay
        # inside the mask so its depth sample is always foreground (on the object),
        # never the background behind it. The offset only biases WHERE inside the
        # mask we sample (toward Qwen's graspable spot); it can no longer walk off
        # the object and drag target_object metres along the camera ray.
        grasp_u, grasp_v = mask_centroid_u, mask_centroid_v
        if hasattr(self, "grasp_offset"):
            snapped = self._snap_point_into_mask(
                mask_centroid_u + int(self.grasp_offset[0]),
                mask_centroid_v + int(self.grasp_offset[1]),
                mask_np,
            )
            if snapped is not None:
                grasp_u, grasp_v = snapped
        h = self.latest_depth_header if self.latest_depth_header is not None else None
        if h is None:
            return
        point_cam = self._pixel_to_camera_point(grasp_u, grasp_v, self.latest_depth)
        if point_cam is None:
            # On-mask grasp pixel landed in a depth hole; widen, then sample all mask pixels
            z, ok = self._get_valid_depth(self.latest_depth, grasp_u, grasp_v, radius=10)
            if not ok:
                z, ok = self._get_depth_from_mask(self.latest_depth, mask_np)
            if ok and self.camera_intrinsics is not None:
                fx, fy, cx, cy = self.camera_intrinsics
                x = (float(grasp_u) - cx) * z / fx
                y = (float(grasp_v) - cy) * z / fy
                point_cam = (x, y, z)
        # Last-resort fallback: use TF-derived geometry depth (no depth sensor required).
        # The tf_projection_node reprojects the known vision-frame object position into the
        # hand cam frame at ~10 Hz.  Z is accurate; we pair it with the on-mask grasp pixel.
        if point_cam is None and self._geometry_cam_pt is not None:
            gz = self._geometry_cam_pt[2]
            if gz > 0.0 and self.camera_intrinsics is not None:
                fx, fy, cx, cy = self.camera_intrinsics
                x_g = (float(grasp_u) - cx) * gz / fx
                y_g = (float(grasp_v) - cy) * gz / fy
                point_cam = (x_g, y_g, gz)
                self.get_logger().info(
                    f"Using geometry depth fallback: z={gz:.3f}m grasp=({grasp_u},{grasp_v})",
                    throttle_duration_sec=1.0,
                )

        if point_cam is not None:
            self._publish_tracking_3d(point_cam[0], point_cam[1], point_cam[2], h)
        else:
            self.get_logger().warn(
                f"No valid depth for tracking point — "
                f"centroid=({mask_centroid_u},{mask_centroid_v}) "
                f"grasp_pixel=({grasp_u},{grasp_v}) "
                f"geometry_cam_pt={self._geometry_cam_pt}",
                throttle_duration_sec=1.0,
            )
        self._hand_disp_state = 'TRACKING'
        self._hand_disp_mask = mask_np
        self._hand_disp_score = float(score)
        self._hand_disp_centroid = (grasp_u, grasp_v)
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
            new_rgb = Image.fromarray(cv_rgb)
            with st['_rgb_lock']:
                st['latest_rgb'] = new_rgb
                st['latest_rgb_header'] = msg.header
                st['new_frame_available'] = True
                st['_last_mask_shape'] = (msg.height, msg.width)
        except Exception as e:
            self.get_logger().error(f"[{cam}] RGB conversion error: {e}")
            return
        # Per-frame mask publishing (unified with hand camera logic):
        # - Before first ever init: only publish zeros when confirmed out-of-FOV.
        #   While ARMING (waiting for first seed), publish nothing so nvblox skips
        #   the frame and doesn't fuse object pixels into the static TSDF.
        # - After first init: always publish _last_mask_np (or zeros if None),
        #   stamped with this RGB header. SAM2 itself zeros the mask when the
        #   object leaves FOV / is lost, which is the desired behaviour.
        # - During a VLM retrigger: suppress publishing until the new init lands.
        self._publish_secondary_mask_for_header(cam, msg.header, msg.height, msg.width)

    def _secondary_depth_cb(self, msg: RosImage, cam: str):
        """Cache the latest raw depth Image msg for a secondary camera.

        Depth is only consumed by the COLD-init depth-consistency gate in
        _run_secondary_tracking_step (secondaries do not otherwise track in 3D),
        so we store the raw msg and convert lazily at init time — converting every
        frame here would burn CPU/GIL for nothing and slow the display + inference.
        """
        st = self._secondary_cam_state.get(cam)
        if st is None:
            return
        with st['_depth_lock']:
            st['latest_depth_msg'] = msg
            st['latest_depth_header'] = msg.header

    def _secondary_camera_info_cb(self, msg: CameraInfo, cam: str):
        """Latch intrinsics + optical frame + size for a secondary camera (once)."""
        st = self._secondary_cam_state.get(cam)
        if st is None or st.get('cam_intrinsics') is not None:
            return
        st['cam_intrinsics'] = (float(msg.k[0]), float(msg.k[4]), float(msg.k[2]), float(msg.k[5]))
        st['cam_frame_id'] = str(msg.header.frame_id) if msg.header.frame_id else cam
        st['cam_size'] = (int(msg.width), int(msg.height))
        self.get_logger().info(
            f"[{cam}] camera_info latched: intr={st['cam_intrinsics']} "
            f"frame={st['cam_frame_id']} size={st['cam_size']}"
        )

    def _reproject_target_to_secondary(self, cam: str, stamp_msg):
        """Project the world-fixed target_object into camera `cam` AT the image stamp
        (time-consistent with the frame SAM2 will segment). Returns (u, v, depth_m)
        or None if intrinsics/TF unavailable, behind the camera, or out of bounds.

        This is the fix for the seed/frame time mismatch: under robot motion the old
        latest-TF seed_pixel was computed for a different camera pose than the frame.
        """
        st = self._secondary_cam_state.get(cam)
        if st is None:
            return None
        intr = st.get('cam_intrinsics')
        cam_frame = st.get('cam_frame_id')
        size = st.get('cam_size')
        if intr is None or cam_frame is None or size is None:
            return None
        try:
            t = self.tf_buffer.lookup_transform(
                cam_frame, self._target_object_frame,
                rclpy.time.Time.from_msg(stamp_msg),
                timeout=Duration(seconds=self._secondary_tf_timeout_sec),
            )
        except LookupException:
            # target_object frame not published yet — expected before the first VLM
            # detection (no object to track). Stay silent so it doesn't spam / mask
            # real failures.
            return None
        except (ExtrapolationException, ConnectivityException):
            # The frame stamp is typically a few ms AHEAD of the latest target_object
            # TF — that frame is published off the slower hand track, so it lags the
            # camera frames ("extrapolation into the future"). target_object is
            # world-fixed, so falling back to the latest available transform costs
            # only the tiny camera-pose change over that ~15 ms gap. This is the
            # continuous seed (FOV liveness / WARM, which now trusts memory), so the
            # small offset is harmless; the VLM cold-init world point is reconstructed
            # frame-accurately elsewhere (tf_projection, frame-stamp lookup).
            try:
                t = self.tf_buffer.lookup_transform(
                    cam_frame, self._target_object_frame,
                    rclpy.time.Time(),  # latest available
                    timeout=Duration(seconds=self._secondary_tf_timeout_sec),
                )
            except Exception as exc2:
                self.get_logger().warn(
                    f"[{cam}] reproject TF lookup failed (frame-stamp + latest): {exc2}",
                    throttle_duration_sec=2.0,
                )
                return None
        except Exception as exc:
            self.get_logger().warn(
                f"[{cam}] reproject TF lookup @frame stamp failed: {exc}",
                throttle_duration_sec=2.0,
            )
            return None
        # translation = target_object origin expressed in the camera optical frame
        X = t.transform.translation.x
        Y = t.transform.translation.y
        Z = t.transform.translation.z
        if Z <= 0.0:
            return None  # behind the camera
        fx, fy, cx, cy = intr
        u = fx * (X / Z) + cx
        v = fy * (Y / Z) + cy
        w, h = size
        if not (0 <= u < w and 0 <= v < h):
            return None  # out of image
        return (float(u), float(v), float(Z))

    def _depth_msg_to_np(self, msg):
        """Convert a depth Image msg to a float32 (metres-ish) ndarray, or None."""
        try:
            return self.bridge.imgmsg_to_cv2(msg, desired_encoding='32FC1')
        except Exception as e:
            self.get_logger().warn(f"depth conversion error: {e}", throttle_duration_sec=2.0)
            return None

    def _median_depth_under_mask(self, depth_np, mask_u8):
        """Median valid depth (m) under mask_u8, rescaling the mask to the depth
        image resolution if they differ (depth_registered need not match RGB).
        Returns (median_m, ok)."""
        dh, dw = depth_np.shape[:2]
        mh, mw = mask_u8.shape[:2]
        if (dw, dh) != (mw, mh):
            mask_d = cv2.resize(mask_u8, (dw, dh), interpolation=cv2.INTER_NEAREST)
        else:
            mask_d = mask_u8
        return self._get_depth_from_mask(depth_np, mask_d)

    def _secondary_seed_pixel_cb(self, msg: PointStamped, cam: str):
        """Force-reinit signal for a secondary camera (VLM re-seed only).

        The seed *position* is no longer taken from here — it's recomputed from TF
        at each frame's own stamp in _run_secondary_tracking_step (see
        _reproject_target_to_secondary), which is time-consistent with the image.
        This callback only carries the reinit flag in point.z's SIGN:
          z < 0: VLM re-seed → force SAM2 to rebuild on the next frame.
          z >= 0: continuous update — ignored here (the per-frame reproject owns it).
        """
        st = self._secondary_cam_state.get(cam)
        if st is None:
            return
        if float(msg.point.z) >= 0.0:
            return  # not a force-reinit; per-frame reproject drives normal seeding
        st['needs_reinit'] = True
        st['consecutive_seed_count'] = self._secondary_fov_enter_count  # bypass hysteresis
        st['cooldown_until'] = 0.0  # VLM detection overrides cooldown
        # Suppress per-frame publishing only if we've already produced at least one
        # mask — otherwise the first VLM call would starve nvblox of zeros while
        # waiting for init.
        if st['_initialized_once']:
            st['_retrigger_pending'] = True
            st['_last_mask_np'] = None

    def _cache_secondary_mask(self, cam: str, mask_np, src_header=None):
        """Cache the secondary mask and publish it stamped with its SOURCE frame.

        Called from the SAM2 tracking step. The non-empty mask is published here,
        carrying the timestamp of the frame SAM2 segmented, so nvblox's approximate
        depth+mask sync pairs it with that exact depth frame (no stale silhouette).
        The per-frame publisher only emits zeros when the object is absent; it no
        longer republishes this cached mask (that re-dated it onto newer frames and
        leaked the moving object into the static TSDF).
        """
        st = self._secondary_cam_state.get(cam)
        if st is None:
            return
        st['_last_mask_np'] = mask_np
        if mask_np is not None:
            st['_last_mask_shape'] = mask_np.shape[:2]
        pub = self._secondary_mask_pubs.get(cam)
        if pub is None or src_header is None or mask_np is None:
            return
        try:
            m = ((mask_np > 0).astype(np.uint8) * 255)
            m = self._dilate_mask_for_nvblox(m)
            ros_mask = self.bridge.cv2_to_imgmsg(m, encoding='mono8')
            ros_mask.header = src_header
            pub.publish(ros_mask)
        except Exception as e:
            self.get_logger().error(f"[{cam}] Error publishing mask: {e}")

    def _publish_secondary_mask_for_header(self, cam: str, header, h: int, w: int):
        """Per-frame mask publisher for a secondary, mirroring the hand's logic
        (_publish_mask_for_header): the decision is PREDICTOR-based, not reprojection
        based.

          - retrigger pending          → silent (object swap in progress)
          - never tracked (ARMING)     → silent (don't fuse the not-yet-segmented
                                         object as static; we have no mask for it yet)
          - currently tracking         → silent here (the fresh mask is published
                                         source-stamped by _cache_secondary_mask;
                                         _last_mask_np is held between inferences)
          - CONFIRMED lost (mask None  → ZEROS (object-free) so nvblox fuses the scene
            after the lost streak)        static and contributes this camera to the map.

        Note _last_mask_np is cleared ONLY after the lost streak is confirmed (see the
        tracking step), so a transient miss stays silent — no premature zeros over a
        still-visible object. This makes a secondary independent of target_object /
        the hand: if the hand loses tracking, a WARM secondary keeps masking from its
        own memory and only zeros when ITS OWN predictor gives up, exactly like the
        hand. That removes the old reproject-timeout leak (hand lost → target_object
        gone → secondary timed out → zeroed over the still-visible object).
        """
        pub = self._secondary_mask_pubs.get(cam)
        if pub is None or header is None:
            return
        st = self._secondary_cam_state.get(cam)
        if st is None:
            return
        if st.get('_retrigger_pending', False):
            return
        if not st.get('_initialized_once', False):
            return  # ARMING — never tracked yet; stay silent (don't fuse the object)
        if st.get('_last_mask_np') is not None:
            return  # tracking (or transient miss): fresh mask already published
        # Confirmed lost → emit zeros so this camera maps the scene. BUT on out->in
        # FOV re-entry there is a window where the object is already visible and the
        # SAM2 predictor has not re-acquired it yet; those zero-frames would fuse the
        # object STATIC (the intermittent re-entry leak). While lost, protect a disc
        # at the world-fixed target_object reprojected into THIS camera (it persists
        # at the last world pose via tf_projection even while the hand is lost), so
        # the re-entering object is masked dynamic until the predictor re-locks. If
        # the object does not reproject into this camera (genuinely out of view), the
        # reproject returns None → plain zeros (camera keeps mapping, unchanged).
        m = np.zeros((h, w), dtype=np.uint8)
        r = self._secondary_lost_guard_radius_px
        if r > 0:
            proj = self._reproject_target_to_secondary(cam, header.stamp)
            if proj is not None:
                u, v, _ = proj
                size = st.get('cam_size')  # reproject is in cam_size (w,h) px
                if size is not None:
                    sw, sh = size
                    if (sw, sh) != (w, h) and sw > 0 and sh > 0:
                        u *= w / float(sw)
                        v *= h / float(sh)
                cv2.circle(m, (int(round(u)), int(round(v))), r, 255, -1)
        try:
            ros_mask = self.bridge.cv2_to_imgmsg(m, encoding='mono8')
            ros_mask.header = header
            pub.publish(ros_mask)
        except Exception as e:
            self.get_logger().error(f"[{cam}] Error publishing mask: {e}")

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

            with self._gui_lock:
                self._pending_display_frames[f"SAM2 {cam}"] = img_bgr
        except Exception as e:
            self.get_logger().warn(f"[{cam}] Visualization error: {e}")

    def _run_secondary_tracking_step(self, cam: str):
        """Run one SAM2 tracking step for a secondary camera.

        Always updates the debug window (even without a new frame) so all 3 windows
        remain visible from the moment the node starts.
        """
        st = self._secondary_cam_state[cam]
        # Snapshot the RGB cache under the per-cam lock — the RGB callback runs on
        # io_cb_group (different thread from this timer), so direct dict reads would
        # race against writes in _secondary_rgb_cb.
        with st['_rgb_lock']:
            img_pil = st['latest_rgb']
            frame_hdr = st.get('latest_rgb_header')
            new_frame = st['new_frame_available']
            if new_frame:
                st['new_frame_available'] = False
        now = time.time()

        # ── Exact per-frame seed ─────────────────────────────────────────────────
        # Reproject the world-fixed target into THIS camera at the CURRENT frame's
        # stamp — time-consistent with the image SAM2 will segment. This replaces
        # the async latest-TF seed_pixel, which under robot motion was computed for
        # a different camera pose than the frame (the seed/frame mismatch). Drives
        # FOV liveness, ARMING hysteresis, expected_depth, and the centroid check.
        if new_frame and frame_hdr is not None:
            frame_seed = self._reproject_target_to_secondary(cam, frame_hdr.stamp)
            if frame_seed is not None:
                u_s, v_s, z_s = frame_seed
                st['init_uv'] = (u_s, v_s)
                st['init_uv_stamp'] = frame_hdr.stamp.sec + frame_hdr.stamp.nanosec * 1e-9
                st['expected_depth'] = z_s
                st['last_seed_time'] = now
                if not st['tracking_initialized']:
                    if st.get('lifecycle') == 'WARM':
                        # WARM but lost → re-prompt at the fresh seed (memory helps).
                        st['needs_reinit'] = True
                    elif now >= st.get('cooldown_until', 0.0):
                        # COLD arming: require N consecutive in-FOV frames first.
                        st['consecutive_seed_count'] += 1
                        if st['consecutive_seed_count'] >= self._secondary_fov_enter_count:
                            st['needs_reinit'] = True
            # frame_seed is None → object not in this camera at frame time; leave
            # last_seed_time stale so the FOV-timeout below flips to OUT_OF_FOV.

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
        if not new_frame:
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

        img_np = np.array(img_pil.convert('RGB'))

        # ── Object not in FOV at all (no seed ever or timed out) ─────────────
        # WARM cameras with an initialized predictor bypass FOV gating — SAM2
        # decides per frame whether the object is visible. If it's not, the
        # mask comes back empty and zeros propagate to nvblox.
        is_warm_with_predictor = (
            st.get('lifecycle') == 'WARM'
            and st.get('_initialized_once', False)
            and st['video_predictor'] is not None
        )
        no_seed_ever = st['last_seed_time'] == 0.0
        fov_timed_out = (
            st['last_seed_time'] > 0.0
            and (now - st['last_seed_time']) > self._secondary_fov_timeout_sec
        )
        if (no_seed_ever or fov_timed_out) and not is_warm_with_predictor:
            st['_disp_state'] = 'OUT_OF_FOV'
            st['_disp_mask'] = None
            if self.visualize:
                self._update_secondary_display(
                    cam, img_pil, 'OUT_OF_FOV',
                    expected_uv=st.get('init_uv'),
                )
            return

        # ── ARMING: in FOV but not yet armed (hysteresis not met) ────────────
        # WARM cameras skip ARMING entirely: their predictor's memory is enough
        # to re-acquire the object on its own — fall through to continuous
        # tracking and let SAM2 produce masks (or zeros) every frame.
        is_warm_with_predictor = (
            st.get('lifecycle') == 'WARM'
            and st.get('_initialized_once', False)
            and st['video_predictor'] is not None
        )
        if not st['tracking_initialized'] and not st['needs_reinit'] and not is_warm_with_predictor:
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
            # WARM bypass: a WARM predictor already has appearance memory of the
            # object, so an imprecise (or stale) seed point is acceptable — SAM2's
            # memory will pull the mask onto the right object. Strict freshness
            # only matters for the very first COLD prompt where the seed *is* the
            # only signal identifying which object to track.
            seed_stamp = st.get('init_uv_stamp')
            is_warm_cam = st.get('lifecycle') == 'WARM'
            if not is_warm_cam and seed_stamp is not None and frame_hdr is not None:
                frame_stamp = frame_hdr.stamp.sec + frame_hdr.stamp.nanosec * 1e-9
                dt_ms = abs(seed_stamp - frame_stamp) * 1000.0
                if dt_ms > self._secondary_seed_pixel_stale_ms:
                    self.get_logger().warn(
                        f"[{cam}] Seed pixel stale: dt={dt_ms:.0f}ms > "
                        f"{self._secondary_seed_pixel_stale_ms:.0f}ms — skipping init "
                        f"(seed_t={seed_stamp:.3f} frame_t={frame_stamp:.3f})",
                        throttle_duration_sec=1.0,
                    )
                    # Keep needs_reinit and consecutive_seed_count as-is so the
                    # next fresh seed can retry without losing arming progress.
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
                skip_area_check = True
            else:
                # Cold (or DEGRADED) init: full reset.
                st['video_predictor'].inference_state = {}
                st['video_predictor']._ros_frame_idx = 0
                skip_area_check = False

            # COLD init: size a box prompt from the hand-tracked object's metric size,
            # projected at this secondary's expected depth, so SAM2 segments the whole
            # object rather than the connected sub-part under the single seed point.
            # WARM keeps the point-only prompt (its memory already knows the object).
            init_box = None
            if (not is_warm) and self._hand_object_metric_size is not None:
                z_sec = st.get('expected_depth')
                intr = st.get('cam_intrinsics')
                if z_sec is not None and z_sec > 0.0 and intr is not None:
                    fx_s, fy_s = float(intr[0]), float(intr[1])
                    ih, iw = img_np.shape[:2]
                    half_w = 0.5 * self._hand_object_metric_size * self._secondary_box_scale * fx_s / z_sec
                    half_h = 0.5 * self._hand_object_metric_size * self._secondary_box_scale * fy_s / z_sec
                    x1 = max(0.0, u - half_w)
                    y1 = max(0.0, v - half_h)
                    x2 = min(iw - 1.0, u + half_w)
                    y2 = min(ih - 1.0, v + half_h)
                    if (x2 - x1) >= 2.0 and (y2 - y1) >= 2.0:
                        init_box = [x1, y1, x2, y2]

            if init_box is not None:
                results, _wait, _infer = self._run_sam2_timed(
                    f'{cam}_init', st['video_predictor'], img_np,
                    bboxes=[init_box], points=[[u, v]], labels=[1],
                )
            else:
                results, _wait, _infer = self._run_sam2_timed(
                    f'{cam}_init', st['video_predictor'], img_np, points=[[u, v]], labels=[1]
                )
            mask, score = _best_mask_from_results(results)
            if mask is not None:
                mask_u8 = mask.astype(np.uint8)
                h_img, w_img = mask_u8.shape[:2]
                mask_area = float(mask_u8.sum())
                # Seed must land inside the mask (±tol px). SAM2 was prompted with
                # this point as positive, so a healthy mask contains it. WARM skips
                # it: an imprecise/stale seed is fine because SAM2 memory pulls the
                # mask onto the object.
                u_c = min(max(0, u), w_img - 1)
                v_c = min(max(0, v), h_img - 1)
                tol = self._secondary_seed_in_mask_tol_px
                seed_patch = mask_u8[max(0, v_c - tol):v_c + tol + 1,
                                     max(0, u_c - tol):u_c + tol + 1]
                seed_in_mask = bool(seed_patch.any())

                # Mask-depth consistency (COLD only) — the real background guard.
                # SAM2 can segment the background *around* a seed that sits on the
                # object (seed-in-mask still passes), so validate the MASK's own
                # median depth against the object's expected range (|seed_pixel.z|).
                # Fail-CLOSED when depth is available but the mask has no/inconsistent
                # depth: a genuine near object must register depth, so a masked region
                # with none is almost always background (sky/far wall / depth hole).
                # Fail-OPEN only when no depth msg exists at all (depth pipeline down).
                expected_depth = st.get('expected_depth')
                z_mask = None
                mask_depth_bad = False
                if (not is_warm) and expected_depth is not None:
                    with st['_depth_lock']:
                        depth_msg = st.get('latest_depth_msg')
                    if depth_msg is not None:
                        depth_np = self._depth_msg_to_np(depth_msg)
                        if depth_np is not None:
                            z_mask, ok_m = self._median_depth_under_mask(depth_np, mask_u8)
                            if (not ok_m) or abs(z_mask - expected_depth) > self._secondary_seed_depth_tol_m:
                                mask_depth_bad = True
                            # ── DIAGNOSTIC: is the reprojected SEED PIXEL itself on the
                            #    object?  Sample depth at the seed (u,v), scaled to the
                            #    depth image. z_seed≈expected → geometry is fine, the
                            #    problem is SAM2 grabbing an adjacent surface from an
                            #    edge point (→ box init fixes it). z_seed≠expected →
                            #    the pixel itself is mis-placed (→ geometry/timing).
                            dh, dw = depth_np.shape[:2]
                            du = int(u * dw / max(w_img, 1))
                            dv = int(v * dh / max(h_img, 1))
                            z_seed, ok_s = self._get_valid_depth(depth_np, du, dv, radius=6)
                            self.get_logger().warn(
                                f"[{cam}] SEED-DIAG seed=({u},{v}) "
                                f"z_seed={'n/a' if not ok_s else f'{z_seed:.2f}m'} "
                                f"z_mask={'n/a' if z_mask is None else f'{z_mask:.2f}m'} "
                                f"expected={expected_depth:.2f}m  "
                                f"(seedΔ={'n/a' if not ok_s else f'{abs(z_seed-expected_depth):.2f}m'} "
                                f"maskΔ={'n/a' if z_mask is None else f'{abs(z_mask-expected_depth):.2f}m'})"
                            )

                image_area = float(h_img * w_img)
                mask_frac = mask_area / max(image_area, 1.0)
                rejected = False
                if mask_area < self._secondary_min_mask_area_px:
                    self.get_logger().warn(
                        f"[{cam}] SAM2 init rejected: mask too small "
                        f"({int(mask_area)}px² < {self._secondary_min_mask_area_px}px²) — noise"
                    )
                    rejected = True
                elif (not skip_area_check) and mask_frac > self._secondary_max_mask_frac:
                    self.get_logger().warn(
                        f"[{cam}] SAM2 init rejected: mask too large "
                        f"({mask_frac*100:.1f}% > {self._secondary_max_mask_frac*100:.0f}%) — likely background"
                    )
                    rejected = True
                elif (not is_warm) and not seed_in_mask:
                    self.get_logger().warn(
                        f"[{cam}] SAM2 init rejected: seed ({u},{v}) not inside mask "
                        f"(±{tol}px) — mask off-target"
                    )
                    rejected = True
                elif mask_depth_bad:
                    self.get_logger().warn(
                        f"[{cam}] SAM2 init rejected by depth: mask depth "
                        f"{'n/a' if z_mask is None else f'{z_mask:.2f}m'} vs expected "
                        f"{expected_depth:.2f}m (tol {self._secondary_seed_depth_tol_m:.2f}m) "
                        f"— segmented region not at the object's range (likely background)"
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
                    st['_initialized_once'] = True
                    st['_retrigger_pending'] = False
                    st['tracking_frame_count'] = 1
                    st['last_score'] = score
                    st['lost_streak'] = 0
                    st['warm_fail_streak'] = 0
                    if is_warm:
                        st['warm_tracked_frames'] += 1
                        self.get_logger().info(
                            f"[{cam}] WARM re-entry at ({u},{v}), score={score:.3f} area={int(mask_area)}px²"
                        )
                    else:
                        # DEGRADED just rebuilt → back to COLD until promoted again.
                        if st.get('lifecycle') == 'DEGRADED':
                            st['lifecycle'] = 'COLD'
                        st['warm_tracked_frames'] = 1
                    self._cache_secondary_mask(cam, mask_u8, frame_hdr)
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
                        f"[{cam}] SAM2 initialized at ({u},{v}), score={score:.3f} area={int(mask_area)}px²"
                        + (f" depth={z_mask:.2f}m/exp{expected_depth:.2f}m"
                           if (z_mask is not None and expected_depth is not None) else ""),
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
        # Run while actively tracking, OR while WARM after a lost streak — the
        # video predictor's appearance memory can re-acquire the object on its
        # own without waiting for a new seed prompt.
        elif st['tracking_initialized'] or (
            st.get('lifecycle') == 'WARM'
            and st.get('_initialized_once', False)
            and st['video_predictor'] is not None
        ):
            results, _wait, _infer = self._run_sam2_timed(
                cam, st['video_predictor'], img_np
            )
            mask, score = _best_mask_from_results(results)
            if mask is not None and score > 0.0:
                mask_u8 = mask.astype(np.uint8)
                m_cv = cv2.moments(mask_u8)
                centroid = None
                if m_cv["m00"] != 0:
                    centroid = (int(m_cv["m10"] / m_cv["m00"]), int(m_cv["m01"] / m_cv["m00"]))

                # Validate the propagated mask. WARM → fully trust the video
                # predictor's appearance memory: do NOT veto on the cross-camera
                # reprojected centroid (init_uv). That reference is jittery (TF
                # extrapolation slop + reprojection drift) and was rejecting good
                # memory-propagated masks, causing the LOST/re-seed thrash. Only
                # reject a degenerate/runaway mask. COLD (initialized but not yet
                # promoted) keeps the centroid gate — the reprojection is still the
                # trust anchor until the memory has proven itself.
                is_warm = st.get('lifecycle') == 'WARM'
                consistent = True
                reject_reason = None
                if is_warm:
                    h_img, w_img = mask_u8.shape[:2]
                    mask_area = float(mask_u8.sum())
                    mask_frac = mask_area / max(float(h_img * w_img), 1.0)
                    if (mask_area < self._secondary_min_mask_area_px
                            or mask_frac > self._secondary_max_mask_frac):
                        consistent = False
                        reject_reason = (
                            f"WARM mask degenerate (area={int(mask_area)}px² "
                            f"frac={mask_frac*100:.1f}%)"
                        )
                elif centroid is not None and st['init_uv'] is not None:
                    eu, ev = st['init_uv']
                    dist = ((centroid[0] - eu) ** 2 + (centroid[1] - ev) ** 2) ** 0.5
                    if dist > self._secondary_max_centroid_dist_px:
                        consistent = False
                        reject_reason = (
                            f"Centroid ({centroid[0]},{centroid[1]}) too far from "
                            f"expected ({eu:.0f},{ev:.0f}), dist={dist:.0f}px — background?"
                        )

                if not consistent:
                    st['lost_streak'] += 1
                    self.get_logger().warn(f"[{cam}] {reject_reason}", throttle_duration_sec=1.0)
                    if st['lost_streak'] >= self.tracking_lost_confirm_frames:
                        st['tracking_initialized'] = False
                        st['consecutive_seed_count'] = 0  # re-arm from scratch
                        # Confirmed lost → clear the cached mask so the per-frame
                        # publisher emits zeros (object-free), mirroring the hand.
                        # Before confirm we keep the last mask so a transient miss
                        # stays SILENT (no premature zeros over a still-visible object).
                        st['_last_mask_np'] = None
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
                    # If we reached here via the WARM-after-LOST path, mark
                    # tracking as re-acquired so the regular display / state
                    # machine reflects reality.
                    st['tracking_initialized'] = True
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
                    self._cache_secondary_mask(cam, mask_u8, frame_hdr)
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
                    # Object centroid wandered to background — don't fuse this mask.
                    # (_last_mask_np is cleared only on CONFIRMED lost above, so a
                    # one-off reject stays silent rather than zeroing immediately.)
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
                # SAM2 returned nothing this frame. Mirror the hand: do NOT zero
                # immediately — keep the last cached mask so a transient miss stays
                # SILENT (nvblox drops the frame). Only on CONFIRMED lost do we clear
                # the mask so the per-frame publisher emits zeros (object-free).
                st['lost_streak'] += 1
                self.get_logger().warn(
                    f"[{cam}] No mask from SAM2 (lost_streak={st['lost_streak']}/{self.tracking_lost_confirm_frames})",
                    throttle_duration_sec=1.0,
                )
                if st['lost_streak'] >= self.tracking_lost_confirm_frames:
                    st['_last_mask_np'] = None
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

        # Once initialized, the predictor keeps inferring on every tick (even through
        # LOST) so it can recover from its own memory without an external re-seed.
        # Skip while a retrigger is pending (predictor state will be cleared by the
        # incoming init). Only an explicit VLM retrigger flips _hand_initialized_once
        # back to False; LOST keeps the predictor running on its first-mask memory.
        if self._hand_initialized_once and not self._hand_retrigger_pending:
            self._run_tracking_step_2d(time.time())

        st = String()
        is_tracking = self.tracking_active and self._hand_disp_state == 'TRACKING'
        st.data = "TRACKING" if is_tracking else "LOST"
        self._tracking_state_pub.publish(st)
        # Secondary inference is now driven by per-camera 5 Hz timers on their own
        # MutuallyExclusiveCallbackGroups — see secondary registration loop in __init__.


def main(args=None):
    rclpy.init(args=args)
    node = Sam2TrackerNode()
    executor = MultiThreadedExecutor(num_threads=node._executor_threads)
    executor.add_node(node)
    node.get_logger().info(
        f"MultiThreadedExecutor started with {node._executor_threads} threads"
    )

    if node.visualize:
        # cv2/Qt windows must live on the process main thread. Spin the executor in a
        # background daemon thread and drain queued display frames here.
        exec_thread = threading.Thread(target=executor.spin, name="sam2_executor", daemon=True)
        exec_thread.start()
        try:
            while rclpy.ok():
                for window_name, img_bgr in node.drain_pending_display_frames():
                    try:
                        cv2.imshow(window_name, img_bgr)
                    except Exception as e:
                        node.get_logger().warn(
                            f"cv2.imshow failed: {e}", throttle_duration_sec=2.0
                        )
                # waitKey drives the Qt event loop; 30 ms gives ~33 Hz refresh.
                cv2.waitKey(30)
        except KeyboardInterrupt:
            pass
        finally:
            executor.shutdown()
            exec_thread.join(timeout=2.0)
            try:
                cv2.destroyAllWindows()
            except Exception:
                pass
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
    else:
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
