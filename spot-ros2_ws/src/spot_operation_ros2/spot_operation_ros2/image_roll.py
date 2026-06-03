"""Shared hand-camera roll-correction helpers.

Spot's hand camera rolls about its optical axis as the wrist rotates. That roll
breaks appearance-based perception: the VLM detects worse on a tilted object, and
SAM2's video predictor (which propagates appearance frame-to-frame) loses lock when
the object rotates between frames. The fix is to de-roll the RGB to upright before
inference, then map the result (bbox for the VLM, full mask for SAM2) back to the
native camera orientation so it still aligns with the un-rotated depth nvblox uses.

The roll angle comes from gravity expressed in the camera frame (TF), so the lookup
stays in each node (it needs the node's tf_buffer); only the quaternion->angle math
and the image/coord transforms are factored here.
"""

import math

import cv2
import numpy as np
from PIL import Image


def roll_deg_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    """Roll (deg) to rotate the image so gravity-up points up.

    `(x,y,z,w)` is the rotation of lookup_transform(reference, camera) — i.e. it
    maps camera-frame vectors into the gravity-up reference frame. We need world-up
    expressed in the CAMERA frame, which is the third ROW of that matrix (= R^T's
    third column). Its (x,y) part is where world-up lands in the image plane
    (camera x=right, y=down); atan2 gives the in-plane tilt to undo.

    This is the fix for the roll being invariant: the previous version used the
    third COLUMN (the camera's optical-axis direction in world), but rolling about
    the optical axis does not move that axis, so the angle never tracked the roll.
    World-up in the camera frame DOES rotate 1:1 with roll. Degenerate only when the
    camera looks almost exactly along gravity (world-up projects to ~0 in-image)."""
    up_x = 2.0 * (x * z - w * y)   # world-up . camera-x  (image right)
    up_y = 2.0 * (y * z + w * x)   # world-up . camera-y  (image down)
    return math.degrees(math.atan2(up_x, -up_y))


def build_rotation_matrix(orig_w: int, orig_h: int, angle_deg: float):
    """Forward rotation matrix + expanded canvas size (no content clipped)."""
    center = (orig_w / 2.0, orig_h / 2.0)
    M = cv2.getRotationMatrix2D(center, angle_deg, 1.0)
    cos_a = abs(M[0, 0])
    sin_a = abs(M[0, 1])
    rot_w = int(orig_h * sin_a + orig_w * cos_a)
    rot_h = int(orig_h * cos_a + orig_w * sin_a)
    M[0, 2] += (rot_w - orig_w) / 2.0
    M[1, 2] += (rot_h - orig_h) / 2.0
    return M, (rot_w, rot_h)


def rotate_image_upright(img_pil: "Image.Image", angle_deg: float):
    """Rotate a PIL image by angle_deg (CCW positive). Expands the canvas so no
    content is clipped; fills the border with neutral gray. Returns
    (rotated_pil, M_forward, (rot_w, rot_h))."""
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
    return Image.fromarray(rotated_np), M, (rot_w, rot_h)


def reroll_mask_to_original(mask_upright, M_forward, original_size):
    """Map a full mask from upright (rotated) space back to the native camera
    orientation, so it aligns with the un-rotated depth. Inverse of the forward
    rotation, nearest-neighbor (labels must stay crisp), cropped to original WxH.
    `mask_upright` is a HxW uint8 array; returns a uint8 array of original size."""
    orig_w, orig_h = original_size
    M_inv = cv2.invertAffineTransform(M_forward)
    return cv2.warpAffine(
        mask_upright, M_inv, (orig_w, orig_h),
        flags=cv2.INTER_NEAREST, borderValue=0,
    )


def inverse_rotate_coords_1000(bbox_1000, grasps_1000, M_forward, rotated_size, original_size):
    """Map a bbox + grasp points from rotated [0-1000] space back to original
    [0-1000] space (used by the single-shot VLM detect path)."""
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
