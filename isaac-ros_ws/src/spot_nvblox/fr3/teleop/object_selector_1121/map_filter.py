"""Fail-closed single-target depth routing for the three-object selector.

This module is transport-free. It does not subscribe, publish, seed a tracker,
clear nvblox, or call a robot. A ROS adapter must attach the current selection
identity to fresh tracker state and mask frames before calling route_depth().

Policy:
* explicit no-selection -> publish full depth (all objects remain obstacles);
* selected + current TRACKING + matching fresh mask -> carve only that target;
* selected + LOST/stale/missing/mismatched evidence -> pause depth integration.

Never infer "no selection" from a LOST tracker: the existing tracker only emits
TRACKING/LOST and cannot distinguish those states on its own.
"""
from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Optional

import cv2
import numpy as np

TARGETS = frozenset({"drill", "tennis_ball", "sprite"})
MASK_MAX_AGE_S = 1.5
SELECTION_MAX_AGE_S = 1.0       # selection-state heartbeat; stale state pauses mapping
DEPTH_MASK_SLOP_S = 0.025
FUTURE_TOL_S = 0.05
MIN_MASK_FRACTION = 0.0005
MAX_MASK_FRACTION = 0.20


@dataclass(frozen=True)
class Identity:
    target: str
    generation: int
    selection_id: str


@dataclass(frozen=True)
class Selection:
    identity: Optional[Identity]
    stamp: float                     # heartbeat timestamp for this state
    selected_at: Optional[float]     # immutable timestamp of generation change


@dataclass(frozen=True)
class Tracking:
    identity: Optional[Identity]
    state: str                     # exactly TRACKING or LOST
    stamp: float


@dataclass(frozen=True)
class MaskFrame:
    identity: Optional[Identity]
    stamp: float
    pixels: np.ndarray             # HxW mono mask; nonzero means selected target


@dataclass(frozen=True)
class Route:
    action: str                    # full | carved | pause
    reason: str
    depth: Optional[np.ndarray]
    excluded: Optional[np.ndarray]


def _finite(*values):
    return all(math.isfinite(float(v)) for v in values)


def _carve(depth: np.ndarray, mask: np.ndarray, dilate_px: int):
    if depth.ndim != 2 or mask.ndim != 2 or depth.size == 0 or mask.size == 0:
        raise ValueError("depth and mask must be non-empty 2D arrays")
    if depth.shape != mask.shape:
        raise ValueError("depth and mask dimensions must match")
    if isinstance(dilate_px, bool) or not isinstance(dilate_px, int) or dilate_px < 0 or dilate_px > 32:
        raise ValueError("dilate_px must be an integer in [0,32]")
    binary = (mask > 0).astype(np.uint8)
    fraction = float(binary.mean())
    if not MIN_MASK_FRACTION < fraction < MAX_MASK_FRACTION:
        raise ValueError(f"mask fraction {fraction:.5f} outside accepted range")
    if dilate_px:
        size = 2 * dilate_px + 1
        binary = cv2.dilate(binary, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (size, size)))
    excluded = binary.astype(bool)
    out = np.array(depth, copy=True)
    out[excluded] = 0
    if not np.array_equal(out[~excluded], depth[~excluded], equal_nan=True):
        raise AssertionError("background depth changed during target carve")
    return out, excluded


def route_depth(
    depth: np.ndarray,
    *,
    depth_stamp: float,
    now: float,
    selection: Selection,
    tracking: Optional[Tracking],
    mask: Optional[MaskFrame],
    dilate_px: int = 8,
) -> Route:
    """Choose full, target-carved, or paused depth without guessing state."""
    d = np.asarray(depth)
    if d.ndim != 2 or d.size == 0:
        return Route("pause", "Malformed depth image", None, None)
    if not _finite(depth_stamp, now, selection.stamp):
        return Route("pause", "Non-finite timestamp", None, None)
    if now - depth_stamp < -FUTURE_TOL_S:
        return Route("pause", "Depth stamp is in the future", None, None)
    if now - depth_stamp > MASK_MAX_AGE_S:
        return Route("pause", "Depth is stale", None, None)
    selection_age = now - selection.stamp
    if not (0 <= selection_age <= SELECTION_MAX_AGE_S):
        return Route("pause", f"Selection state is stale or future-stamped ({selection_age:.2f}s)", None, None)

    ident = selection.identity
    if ident is None:
        # Explicit fresh no-target heartbeat is the only state that permits raw depth.
        if selection.selected_at is not None:
            return Route("pause", "No-target state has a selection timestamp", None, None)
        return Route("full", "No target selected; retaining all scene objects", np.array(d, copy=True), None)
    if ident.target not in TARGETS or ident.generation < 1 or not ident.selection_id:
        return Route("pause", "Selection identity is invalid", None, None)
    selected_at = selection.selected_at
    if selected_at is None or not _finite(selected_at) or selected_at > selection.stamp + FUTURE_TOL_S:
        return Route("pause", "Selection change timestamp is missing or invalid", None, None)
    if selected_at > depth_stamp + FUTURE_TOL_S:
        return Route("pause", "Depth predates the active selection", None, None)
    if tracking is None or mask is None:
        return Route("pause", "Selected target lacks tracking or mask evidence", None, None)
    if tracking.identity != ident or mask.identity != ident:
        return Route("pause", "Tracking or mask identity differs from current selection", None, None)
    if tracking.state != "TRACKING":
        return Route("pause", f"Selected target tracker state is {tracking.state!r}", None, None)
    if not _finite(tracking.stamp, mask.stamp):
        return Route("pause", "Non-finite tracker or mask timestamp", None, None)
    if not (selected_at <= tracking.stamp <= now + FUTURE_TOL_S):
        return Route("pause", "Tracker state predates selection or is future-stamped", None, None)
    if not (selected_at <= mask.stamp <= now + FUTURE_TOL_S):
        return Route("pause", "Mask predates selection or is future-stamped", None, None)
    if now - tracking.stamp > MASK_MAX_AGE_S:
        return Route("pause", "Tracker state is stale", None, None)
    if now - mask.stamp > MASK_MAX_AGE_S:
        return Route("pause", "Target mask is stale", None, None)
    if abs(depth_stamp - mask.stamp) > DEPTH_MASK_SLOP_S:
        return Route("pause", "Depth and mask are not source-stamp matched", None, None)
    try:
        carved, excluded = _carve(d, mask.pixels, dilate_px)
    except (ValueError, AssertionError, cv2.error) as exc:
        return Route("pause", f"Target mask refused: {exc}", None, None)
    return Route("carved", f"Carved selected target {ident.target}", carved, excluded)
