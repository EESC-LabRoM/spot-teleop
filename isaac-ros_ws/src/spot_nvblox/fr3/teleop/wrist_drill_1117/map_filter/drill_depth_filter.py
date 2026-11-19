"""Timestamped depth gating for map integration.

No selected target passes raw depth so every scene object remains an ESDF
obstacle. An active target requires its own fresh matching mask. Lost or
unknown selection state pauses integration by withholding depth.
"""
from __future__ import annotations

import collections
import json
import math
import time

import cv2
import numpy as np


class IntegrationPaused(ValueError):
    """Selection or mask state is unsafe for map integration."""


def scale_depth(depth: np.ndarray, scale: float) -> np.ndarray:
    """Apply a positive depth scale to finite positive ranges, preserving 0/NaN."""
    if not math.isfinite(scale) or scale <= 0:
        raise ValueError("depth scale must be finite and positive")
    out = depth.copy()
    valid = np.isfinite(depth) & (depth > 0)
    out[valid] = depth[valid] * scale
    assert np.array_equal(out[~valid], depth[~valid], equal_nan=True)
    return out


def carve(depth: np.ndarray, mask: np.ndarray, dilate: int = 8):
    if depth.ndim != 2 or mask.ndim != 2:
        raise ValueError("2D images required")
    if abs(depth.shape[1] / depth.shape[0] - mask.shape[1] / mask.shape[0]) > .002:
        raise ValueError("unaligned aspect ratios")
    if depth.shape != mask.shape:
        mask = cv2.resize(mask, (depth.shape[1], depth.shape[0]),
                          interpolation=cv2.INTER_NEAREST)
    mask = (mask > 0).astype(np.uint8)
    fraction = float(mask.mean())
    if not .0001 < fraction < .20:
        raise ValueError("empty or oversized target mask")
    if dilate:
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE,
                                           (2 * dilate + 1, 2 * dilate + 1))
        mask = cv2.dilate(mask, kernel)
    removed = mask > 0
    out = depth.copy()
    out[removed] = 0
    assert np.array_equal(out[~removed], depth[~removed], equal_nan=True)
    return out, removed


def apply_target_gate(depth: np.ndarray, state: str, mask: np.ndarray | None = None,
                      *, dilate: int = 8, min_remaining_pixels: int = 1):
    """Return filtered depth and removed pixels for explicit tracker state.

    Supported states are ``NO_TARGET``, ``ACTIVE:<object_id>``, and
    ``LOST:<object_id>``. Unknown/lost states pause integration. A valid active
    state must be paired with a nonempty mask by the caller.
    """
    if state == "NO_TARGET":
        removed = np.zeros(depth.shape, dtype=bool)
        out = depth.copy()
    elif state.startswith("LOST:"):
        raise IntegrationPaused("selected target mask lost")
    elif state.startswith("ACTIVE:"):
        if mask is None:
            raise IntegrationPaused("selected target has no fresh matching mask")
        out, removed = carve(depth, mask, dilate=dilate)
    else:
        raise IntegrationPaused(f"unknown target state {state!r}")
    remaining = int(np.count_nonzero(np.isfinite(out) & (out > 0)))
    if remaining < min_remaining_pixels:
        raise IntegrationPaused(f"only {remaining} valid depth pixels remain")
    return out, removed


class Pairing:
    def __init__(self, max_age=1.5, slop=.025):
        self.max_age = max_age
        self.slop = slop
        self.last = 0.0

    def valid(self, depth_stamp, mask_stamp, now):
        return (all(math.isfinite(x) for x in (depth_stamp, mask_stamp, now))
                and depth_stamp > self.last
                and abs(depth_stamp - mask_stamp) <= self.slop
                and 0 <= now - mask_stamp <= self.max_age
                and 0 <= now - depth_stamp <= self.max_age)


def main():
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data, QoSProfile, ReliabilityPolicy
    from sensor_msgs.msg import Image
    from std_msgs.msg import String

    rclpy.init()
    n = Node("wrist_drill_depth_filter")
    depths = collections.deque(maxlen=60)
    masks = collections.deque(maxlen=20)
    pairing = Pairing()
    counts = collections.Counter()
    recent = {}
    mask_topic = n.declare_parameter(
        "mask_topic", "/vm/view_mask").value
    state_topic = n.declare_parameter(
        "target_state_topic", "/wrist_map/target_state").value
    mask_object_id = n.declare_parameter("mask_object_id", "drill").value
    state_timeout = float(n.declare_parameter("state_timeout_s", 3.0).value)
    dilation_px = int(n.declare_parameter("dilation_px", 0).value)
    depth_scale = float(n.declare_parameter("depth_scale", 0.9755371809).value)
    min_remaining = int(n.declare_parameter("min_remaining_depth_pixels", 100).value)
    pub = n.create_publisher(Image, "/wrist_map/drill_free_depth",
                             qos_profile_sensor_data)
    maskpub = n.create_publisher(Image, "/wrist_map/exclusion_mask",
                                 qos_profile_sensor_data)
    selected = {"state": None, "received": 0.0}

    def stamp(msg):
        return msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

    def publish_depth(d, dep, raw, removed, mode, mask_stamp=None, now=None):
        msg = Image(header=d.header, height=d.height, width=d.width,
                    encoding="32FC1", is_bigendian=False, step=d.width * 4,
                    data=np.ascontiguousarray(raw, dtype="<f4").tobytes())
        mm = Image(header=d.header, height=d.height, width=d.width,
                   encoding="mono8", step=d.width,
                   data=(removed.astype(np.uint8) * 255).tobytes())
        pub.publish(msg)
        maskpub.publish(mm)
        pairing.last = stamp(d)
        counts["published"] += 1
        counts[mode] += 1
        outside = ~removed
        recent.update(
            mode=mode,
            depth_scale=depth_scale,
            mask_topic=mask_topic,
            depth_mask_delta_ms=(stamp(d) - mask_stamp) * 1000 if mask_stamp else None,
            target_state_age_ms=(time.monotonic() - selected["received"]) * 1000,
            removed_fraction=float(removed.mean()),
            remaining_nonzero=int(np.count_nonzero(raw)),
            outside_changed=int(np.count_nonzero(~np.isclose(
                raw[outside], dep[outside], equal_nan=True))),
            stamp=stamp(d),
        )

    def process():
        now = n.get_clock().now().nanoseconds * 1e-9
        state = selected["state"]
        state_age = time.monotonic() - selected["received"] if state else float("inf")
        if state is None or state_age > state_timeout:
            counts["selection_state_missing_or_stale"] += 1
            return
        for d in list(depths):
            ds = stamp(d)
            if ds <= pairing.last:
                depths.remove(d)
                continue
            depth_age = now - ds
            if not 0 <= depth_age <= pairing.max_age:
                if depth_age > pairing.max_age:
                    depths.remove(d)
                    counts["stale_depth"] += 1
                continue
            if d.encoding != "32FC1":
                depths.remove(d)
                counts["refused_encoding"] += 1
                continue
            dtype = np.dtype(">f4" if d.is_bigendian else "<f4")
            dep = np.frombuffer(d.data, dtype=dtype).reshape(
                d.height, d.step // 4)[:, :d.width]
            scaled = scale_depth(dep, depth_scale)
            try:
                if state == "NO_TARGET":
                    out, removed = apply_target_gate(
                        scaled, state, dilate=dilation_px,
                        min_remaining_pixels=min_remaining)
                    publish_depth(d, scaled, out, removed, "no_target_full_depth", now=now)
                    depths.remove(d)
                    continue
                if state.startswith("LOST:"):
                    raise IntegrationPaused("selected target mask lost")
                if not state.startswith("ACTIVE:"):
                    raise IntegrationPaused(f"unknown target state {state!r}")
                selected_id = state.partition(":")[2]
                if selected_id != mask_object_id:
                    raise IntegrationPaused(
                        f"active target {selected_id!r} does not match mask topic "
                        f"for {mask_object_id!r}")
                if not masks:
                    continue
                m = min(masks, key=lambda item: abs(stamp(item) - ds))
                if not pairing.valid(ds, stamp(m), now):
                    if now - stamp(m) > pairing.max_age or stamp(m) <= pairing.last - .025:
                        masks.remove(m)
                        counts["stale_unmatched_mask"] += 1
                    continue
                if m.encoding != "mono8":
                    masks.remove(m)
                    counts["refused_mask_encoding"] += 1
                    continue
                mask = np.frombuffer(m.data, np.uint8).reshape(
                    m.height, m.step)[:, :m.width]
                out, removed = apply_target_gate(
                    scaled, state, mask, dilate=dilation_px,
                    min_remaining_pixels=min_remaining)
                publish_depth(d, scaled, out, removed, "target_carved", stamp(m), now)
                depths.remove(d)
                masks.remove(m)
            except (ValueError, AssertionError) as exc:
                counts["refused"] += 1
                recent["refusal"] = str(exc)
                depths.remove(d)

    def depth_cb(msg):
        depths.append(msg)
        counts["depth"] += 1
        process()

    def mask_cb(msg):
        if msg.encoding != "mono8" or msg.step < msg.width or len(msg.data) < msg.step * msg.height:
            counts["refused_mask_encoding"] += 1
            return
        masks.append(msg)
        process()

    def state_cb(msg):
        value = msg.data.strip()
        selected.update(state=value, received=time.monotonic())
        counts["selection_state"] += 1
        recent["state"] = value
        process()

    def report():
        data = dict(wall=time.time(), counts=dict(counts), recent=recent)
        print(json.dumps(data), flush=True)
        with open("/tmp/wrist_map_filter/status.json", "w") as stream:
            json.dump(data, stream)

    n.create_subscription(Image, "/franka_seg_zed1/depth", depth_cb,
                          qos_profile_sensor_data)
    n.create_subscription(Image, mask_topic, mask_cb,
                          QoSProfile(depth=2, reliability=ReliabilityPolicy.RELIABLE))
    n.create_subscription(String, state_topic, state_cb, 10)
    n.create_timer(2.0, report)
    try:
        rclpy.spin(n)
    finally:
        report()
        n.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
