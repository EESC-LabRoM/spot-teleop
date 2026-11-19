"""Fail-closed union of independently prompted object masks.

The geometric checks are ROS/Ultralytics independent so they can be exercised
on saved images before the node is connected to the map filter.
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Mapping, Sequence

import numpy as np


@dataclass(frozen=True)
class ObjectMaskSpec:
    object_id: str
    box_norm: tuple[float, float, float, float]
    min_fraction: float
    max_fraction: float


class MaskRejected(ValueError):
    """A required object mask is missing, implausible, or ambiguous."""


def _binary(mask: np.ndarray, shape: tuple[int, int], object_id: str) -> np.ndarray:
    arr = np.asarray(mask)
    if arr.ndim != 2 or arr.shape != shape:
        raise MaskRejected(f"{object_id}: mask shape {arr.shape} != {shape}")
    return arr > 0


def union_object_masks(
    masks: Mapping[str, np.ndarray],
    specs: Sequence[ObjectMaskSpec],
    image_shape: tuple[int, int],
    *,
    max_pair_overlap: float = 0.35,
    box_padding: float = 0.15,
    max_union_fraction: float = 0.20,
) -> tuple[np.ndarray, dict[str, dict[str, float | int]]]:
    """Validate all required masks and return a mono8 0/255 exclusion union.

    Every required object must have a nonempty, size-plausible mask. Each
    centroid must remain near its prompted ROI and different IDs must not
    collapse onto the same SAM region. Any failure rejects the whole union so
    the caller can withhold the source-stamped map mask and pause integration.
    """
    h, w = image_shape
    if h <= 0 or w <= 0 or not specs:
        raise MaskRejected("empty image or object specification")
    ids = [s.object_id for s in specs]
    if len(ids) != len(set(ids)):
        raise MaskRejected("duplicate object IDs")
    if set(masks) != set(ids):
        missing = sorted(set(ids) - set(masks))
        extra = sorted(set(masks) - set(ids))
        raise MaskRejected(f"object set mismatch; missing={missing}, extra={extra}")

    binary: dict[str, np.ndarray] = {}
    stats: dict[str, dict[str, float | int]] = {}
    total = h * w
    for spec in specs:
        box = np.asarray(spec.box_norm, dtype=float)
        if (box.shape != (4,) or not np.isfinite(box).all() or
                np.any(box < 0) or np.any(box > 1) or
                box[0] >= box[2] or box[1] >= box[3] or
                not (0 <= spec.min_fraction < spec.max_fraction <= max_union_fraction)):
            raise MaskRejected(f"{spec.object_id}: invalid prompt/area limits")
        m = _binary(masks[spec.object_id], image_shape, spec.object_id)
        ys, xs = np.where(m)
        n = int(xs.size)
        frac = n / total
        if frac < spec.min_fraction or frac > spec.max_fraction:
            raise MaskRejected(
                f"{spec.object_id}: area fraction {frac:.6f} outside "
                f"[{spec.min_fraction:.6f},{spec.max_fraction:.6f}]"
            )
        cx, cy = float(xs.mean() / w), float(ys.mean() / h)
        px = (box[2] - box[0]) * box_padding
        py = (box[3] - box[1]) * box_padding
        if not (box[0] - px <= cx <= box[2] + px and
                box[1] - py <= cy <= box[3] + py):
            raise MaskRejected(f"{spec.object_id}: centroid escaped prompted ROI")
        binary[spec.object_id] = m
        stats[spec.object_id] = {
            "pixels": n,
            "fraction": frac,
            "centroid_u_norm": cx,
            "centroid_v_norm": cy,
        }

    for i, a_id in enumerate(ids):
        for b_id in ids[i + 1:]:
            a, b = binary[a_id], binary[b_id]
            smaller = min(int(a.sum()), int(b.sum()))
            overlap = int(np.count_nonzero(a & b)) / max(smaller, 1)
            if overlap > max_pair_overlap:
                raise MaskRejected(
                    f"{a_id}/{b_id}: ambiguous overlap {overlap:.3f}"
                )

    union = np.logical_or.reduce([binary[k] for k in ids])
    union_fraction = float(union.mean())
    if not 0 < union_fraction < max_union_fraction:
        raise MaskRejected(f"union area fraction {union_fraction:.6f} is unsafe")
    result = (union.astype(np.uint8) * 255)
    stats["_union"] = {"pixels": int(union.sum()), "fraction": union_fraction}
    return result, stats

