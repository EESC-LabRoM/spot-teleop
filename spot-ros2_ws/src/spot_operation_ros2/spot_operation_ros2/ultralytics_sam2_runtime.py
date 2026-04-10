"""
Helpers for SAM2DynamicInteractivePredictor (Ultralytics) used by detect_qwen multi-camera mode.
"""
from __future__ import annotations

from collections import OrderedDict
from typing import TYPE_CHECKING, Any

import numpy as np

if TYPE_CHECKING:
    pass


def workspace_root() -> str:
    from pathlib import Path

    return str(Path(__file__).resolve().parents[3])


def default_ultralytics_site_packages() -> str:
    import sys

    ver = f"{sys.version_info.major}.{sys.version_info.minor}"
    return f"{workspace_root()}/.venv_ultralytics_sam2/lib/python{ver}/site-packages"


def inject_ultralytics_site_packages(site_packages: str) -> None:
    import sys

    if site_packages and site_packages not in sys.path:
        sys.path.insert(0, site_packages)


def inject_official_sam_paths() -> None:
    import sys

    venv_site = f"{workspace_root()}/src/segment-anything-2/.venv/lib/python3.10/site-packages"
    sam2_root = f"{workspace_root()}/src/segment-anything-2"
    if venv_site not in sys.path:
        sys.path.insert(0, venv_site)
    if sam2_root not in sys.path:
        sys.path.insert(1, sam2_root)


def build_dynamic_predictor(
    model: str,
    device: str | int,
    imgsz: int = 1024,
    max_obj_num: int = 1,
    half: bool = False,
) -> Any:
    from ultralytics.models.sam import SAM2DynamicInteractivePredictor
    from ultralytics.utils import DEFAULT_CFG
    from ultralytics.utils.checks import check_imgsz

    overrides = {"model": model, "device": str(device), "imgsz": imgsz, "half": half}
    p = SAM2DynamicInteractivePredictor(cfg=DEFAULT_CFG, overrides=overrides, max_obj_num=max_obj_num)
    p.setup_model()
    p.imgsz = check_imgsz(imgsz, stride=p.model.stride, min_dim=2)
    return p


def reset_dynamic_predictor(predictor: Any, max_obj_num: int) -> None:
    predictor.memory_bank = []
    predictor.obj_idx_set = set()
    predictor.obj_id_to_idx = predictor.obj_idx_to_id = OrderedDict(enumerate(range(max_obj_num)))
    predictor._max_obj_num = max_obj_num
    predictor.im = None


def preprocess_bgr(predictor: Any, img_bgr: np.ndarray) -> Any:
    import torch

    predictor.im = None
    predictor.batch = (None, [img_bgr], "")
    im = predictor.preprocess([img_bgr])
    if not isinstance(im, torch.Tensor):
        raise TypeError("preprocess must return torch.Tensor")
    return im


def ultra_masks_to_fullres(
    pred_masks: Any,
    orig_h: int,
    orig_w: int,
    pred_scores: Any | None = None,
    as_uint8: bool = True,
) -> tuple[np.ndarray, float]:
    """Mapeia máscaras low-res do canvas LetterBox (SAM Ultralytics) para a imagem original.

    Não usar cv2.resize directo: o preprocess usa LetterBox(center=False), com padding à
    direita/em baixo; `ultralytics.utils.ops.scale_masks(..., padding=False)` corta esse
    padding antes de interpolar para (orig_h, orig_w).
    """
    import torch
    from ultralytics.utils import ops

    if isinstance(pred_masks, torch.Tensor):
        m = pred_masks[0].detach()
    else:
        m = torch.as_tensor(np.asarray(pred_masks[0]), dtype=torch.float32)

    while m.dim() > 2:
        m = m.squeeze(0)
    if m.dim() != 2:
        raise ValueError(f"ultra_masks_to_fullres: esperado máscara HxW, shape={tuple(m.shape)}")

    m4 = m.float().unsqueeze(0).unsqueeze(0)
    scaled = ops.scale_masks(m4, (orig_h, orig_w), padding=False)
    m2 = scaled[0, 0]
    m8 = (m2 > 0.0).cpu().numpy().astype(np.uint8)

    if pred_scores is not None and len(pred_scores) > 0:
        score = float(pred_scores[0].item())
    else:
        score = float(m2.detach().float().cpu().mean().item()) if m2.numel() else 0.0
    if not as_uint8:
        return m8.astype(np.float32), score
    return m8, score


def condition_bbox(predictor: Any, im_tensor: Any, bbox_xyxy: list[float], obj_id: int = 0) -> tuple[Any, Any]:
    import torch

    with torch.inference_mode():
        masks, scores = predictor.inference(
            im_tensor,
            bboxes=[list(map(float, bbox_xyxy))],
            obj_ids=[obj_id],
            update_memory=True,
        )
    return masks, scores


def condition_bbox_center(
    predictor: Any,
    im_tensor: Any,
    cx: float,
    cy: float,
    half_size: float,
    im_w: int,
    im_h: int,
    obj_id: int = 0,
) -> tuple[Any, Any]:
    """Caixa quadrada centrada em (cx, cy) — útil para init a partir de ponto projetado."""
    hs = float(half_size)
    x1 = max(0.0, cx - hs)
    y1 = max(0.0, cy - hs)
    x2 = min(float(im_w - 1), cx + hs)
    y2 = min(float(im_h - 1), cy + hs)
    if x2 <= x1 + 2 or y2 <= y1 + 2:
        x1, y1 = max(0.0, cx - 4), max(0.0, cy - 4)
        x2, y2 = min(float(im_w - 1), cx + 4), min(float(im_h - 1), cy + 4)
    return condition_bbox(predictor, im_tensor, [x1, y1, x2, y2], obj_id=obj_id)


def track_frame(predictor: Any, im_tensor: Any) -> tuple[Any, Any]:
    import torch

    with torch.inference_mode():
        masks, scores = predictor.inference(im_tensor, update_memory=False)
    return masks, scores
