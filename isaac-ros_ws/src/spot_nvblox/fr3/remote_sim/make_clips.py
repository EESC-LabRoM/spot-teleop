#!/usr/bin/env python3
"""Find and label hand motion clips from recorded /fr3_operator/teleop_sample bags."""
import argparse
import json
import math
from pathlib import Path
import sys

import numpy as np

HERE = Path(__file__).resolve().parent
if str(HERE) not in sys.path:
    sys.path.insert(0, str(HERE))

from teleop_mapping import _quat_mul, control_sample, palm_normal
from tracking_gate import TrackingGate

SAMPLE_TOPIC = "/fr3_operator/teleop_sample"


def palm_roll_deg(q0, q):
    """Compute palm roll (degrees) about the away-from-camera palm normal of q0,
    using the exact swing-twist formula and sign convention of teleop_mapping.map_target.
    """
    q0 = np.asarray(q0, dtype=float)
    q0 = q0 / np.linalg.norm(q0)
    q = np.asarray(q, dtype=float)
    q = q / np.linalg.norm(q)

    n0 = palm_normal(q0)
    dot_cam = float(np.dot(n0, [-1.0, 0.0, 0.0]))
    if abs(dot_cam) >= 0.2 and dot_cam < 0.0:
        n0 = -n0

    dq = _quat_mul(q, np.array([-q0[0], -q0[1], -q0[2], q0[3]], dtype=float))
    dq /= np.linalg.norm(dq)
    angle = 2.0 * math.atan2(float(np.dot(dq[:3], n0)), float(dq[3]))
    angle = (angle + math.pi) % (2.0 * math.pi) - math.pi
    if angle <= -math.pi:
        angle += 2.0 * math.pi
    return math.degrees(angle)


def tool_roll_deg(anchor_q, target_q):
    """Extract tool roll angle (degrees) about tool Z from anchor_q to target_q."""
    aq = np.asarray(anchor_q, dtype=float)
    aq = aq / np.linalg.norm(aq)
    tq = np.asarray(target_q, dtype=float)
    tq = tq / np.linalg.norm(tq)
    aq_inv = np.array([-aq[0], -aq[1], -aq[2], aq[3]], dtype=float)
    dq = _quat_mul(aq_inv, tq)
    dq /= np.linalg.norm(dq)
    angle = 2.0 * math.atan2(float(dq[2]), float(dq[3]))
    angle = (angle + math.pi) % (2.0 * math.pi) - math.pi
    if angle <= -math.pi:
        angle += 2.0 * math.pi
    return math.degrees(angle)


def normalized_components(dx_cm, dy_cm, dz_cm, roll_deg):
    """Return sorted list of (normalized_score, single_axis_label)."""
    scores = [
        (abs(float(dx_cm)) / 10.0, "toward camera" if dx_cm >= 0 else "away from camera"),
        (abs(float(dy_cm)) / 10.0, "operator left (+Y)" if dy_cm >= 0 else "operator right (-Y)"),
        (abs(float(dz_cm)) / 10.0, "hand up" if dz_cm >= 0 else "hand down"),
        (abs(float(roll_deg)) / 45.0, "roll cw" if roll_deg >= 0 else "roll ccw"),
    ]
    scores.sort(key=lambda item: item[0], reverse=True)
    return scores


def label_motion(dx_cm, dy_cm, dz_cm, roll_deg):
    """Label dominant motion by the largest of |dX|/10cm, |dY|/10cm, |dZ|/10cm, |roll|/45deg,
    or 'mixed' if the second largest is > 70% of the largest.
    """
    scores = normalized_components(dx_cm, dy_cm, dz_cm, roll_deg)
    best_val, best_label = scores[0]
    second_val, _ = scores[1]
    if best_val <= 1e-9:
        return best_label
    if second_val > 0.70 * best_val:
        return "mixed"
    return best_label


def _unpack_samples(samples):
    ts = []
    ps = []
    qs = []
    stamps = []
    for idx, s in enumerate(samples):
        if isinstance(s, dict):
            t = float(s["t"])
            p = np.asarray(s["p"], dtype=float)
            q = np.asarray(s["q"], dtype=float)
            st = int(s.get("stamp_ns", round(t * 1e9)))
        else:
            t = float(s[0])
            p = np.asarray(s[1], dtype=float)
            q = np.asarray(s[2], dtype=float)
            st = int(s[3]) if len(s) > 3 else round(t * 1e9)
        ts.append(t)
        ps.append(p)
        qs.append(q / np.linalg.norm(q))
        stamps.append(st if st > 0 else idx + 1)
    return np.asarray(ts, dtype=float), np.asarray(ps, dtype=float), np.asarray(qs, dtype=float), stamps


def _step_speeds_cm_s(ts, ps, qs):
    """Effective hand speed per interval: max(linear cm/s, |roll deg/s| * 10/45)."""
    n = len(ts)
    spd = np.zeros(n, dtype=float)
    for i in range(n - 1):
        dt = max(float(ts[i + 1] - ts[i]), 1e-6)
        lin_v = float(np.linalg.norm(ps[i + 1] - ps[i]) * 100.0 / dt)
        rot_v = abs(palm_roll_deg(qs[i], qs[i + 1])) * (10.0 / 45.0) / dt
        spd[i] = max(lin_v, rot_v)
    if n > 1:
        spd[-1] = spd[-2]
    return spd


def _make_segment_dict(ts, ps, qs, i_start, i_pause_end, i_motion_end, tail_s=0.8, max_duration_s=None):
    p0 = ps[i_pause_end]
    q0 = qs[i_pause_end]
    p1 = ps[i_motion_end]
    q1 = qs[i_motion_end]
    dp_cm = (p1 - p0) * 100.0
    roll = palm_roll_deg(q0, q1)
    lbl = label_motion(dp_cm[0], dp_cm[1], dp_cm[2], roll)
    scores = normalized_components(dp_cm[0], dp_cm[1], dp_cm[2], roll)
    strength = float(scores[0][0])
    motion_dur = float(ts[i_motion_end] - ts[i_pause_end])
    start_s = float(ts[i_start])
    end_s = float(ts[i_motion_end] + tail_s)
    if max_duration_s is not None:
        end_s = min(end_s, float(max_duration_s))
    return {
        "label": lbl,
        "start_s": round(start_s, 3),
        "end_s": round(end_s, 3),
        "pause_end_s": round(float(ts[i_pause_end]), 3),
        "motion_end_s": round(float(ts[i_motion_end]), 3),
        "dx_cm": round(float(dp_cm[0]), 2),
        "dy_cm": round(float(dp_cm[1]), 2),
        "dz_cm": round(float(dp_cm[2]), 2),
        "roll_deg": round(float(roll), 2),
        "duration": round(motion_dur, 2),
        "strength": round(strength, 3),
    }


def _gate_activates(ts, ps, qs, stamps, i_start, max_t):
    gate = TrackingGate()
    gate.start(float(ts[i_start]))
    for k in range(i_start, len(ts)):
        t_k = float(ts[k])
        if t_k > max_t:
            break
        gate.observe(int(stamps[k]), t_k, ps[k], qs[k])
        if gate.step(t_k):
            return t_k
    return None


def find_segments(
    samples,
    max_duration_s=None,
    pause_speed_cm_s=2.0,
    min_pause_s=0.8,
    min_motion_s=1.5,
    max_motion_s=8.0,
    tail_s=0.8,
):
    """Find motion segments with a clean >= min_pause_s leading pause (< pause_speed_cm_s),
    followed by motion of duration in [min_motion_s, max_motion_s] until the hand slows again.
    """
    if len(samples) < 4:
        return []
    ts, ps, qs, stamps = _unpack_samples(samples)
    n = len(ts)
    spd = _step_speeds_cm_s(ts, ps, qs)

    # 1. Contiguous low-speed pause segmentation (exact on clean/synthetic trajectories)
    direct_segments = []
    idx = 0
    while idx < n - 2:
        if spd[idx] >= pause_speed_cm_s:
            idx += 1
            continue
        p_start = idx
        p_end = idx
        while p_end + 1 < n and spd[p_end] < pause_speed_cm_s:
            p_end += 1
        pause_dur = float(ts[p_end] - ts[p_start])
        if pause_dur < min_pause_s - 1e-6:
            idx = p_end + 1
            continue
        # Motion starts at p_end and continues while spd >= pause_speed_cm_s
        m_end = p_end
        while m_end < n - 1 and spd[m_end] >= pause_speed_cm_s:
            m_end += 1
        motion_dur = float(ts[m_end] - ts[p_end])
        if min_motion_s - 1e-6 <= motion_dur <= max_motion_s + 1e-6 and m_end > p_end:
            seg = _make_segment_dict(
                ts, ps, qs, p_start, p_end, m_end, tail_s=tail_s, max_duration_s=max_duration_s
            )
            if seg["strength"] >= 0.25:
                direct_segments.append(seg)
        idx = max(m_end, p_end + 1)

    if direct_segments:
        return direct_segments

    # 2. Recorded bag segmentation with depth-jitter filtering:
    # A noisy stationary hand has high tortuosity (step path length >> net displacement)
    # while median endpoints over >= min_pause_s move at < pause_speed_cm_s and TrackingGate activates.
    bag_segments = []
    for i in range(n):
        for j in range(i + 2, n):
            dt = float(ts[j] - ts[i])
            if dt < min_pause_s:
                continue
            if dt > 1.65:
                break
            p_a = np.median(ps[i : min(i + 3, j)], axis=0)
            p_b = np.median(ps[max(i + 1, j - 2) : j + 1], axis=0)
            net_disp_cm = float(np.linalg.norm(p_b - p_a) * 100.0)
            v_net = net_disp_cm / dt
            rot_net = abs(palm_roll_deg(qs[i], qs[j])) * (10.0 / 45.0) / dt
            if max(v_net, rot_net) >= pause_speed_cm_s:
                continue
            act_t = _gate_activates(ts, ps, qs, stamps, i, float(ts[j]) + 0.35)
            if act_t is None:
                continue
            for k in range(j + 1, n):
                mdur = float(ts[k] - ts[j])
                if mdur < min_motion_s:
                    continue
                if mdur > max_motion_s:
                    break
                k_lo = max(j, k - 1)
                k_hi = min(n - 1, k + 1)
                dt_loc = max(float(ts[k_hi] - ts[k_lo]), 0.05)
                v_end = float(np.linalg.norm(ps[k_hi] - ps[k_lo]) * 100.0 / dt_loc)
                if v_end < 15.0:
                    seg = _make_segment_dict(
                        ts, ps, qs, i, j, k, tail_s=tail_s, max_duration_s=max_duration_s
                    )
                    k_end = int(np.searchsorted(ts, seg["end_s"], side="right")) - 1
                    k_end = max(k, min(k_end, n - 1))
                    dp_end = (ps[k_end] - ps[j]) * 100.0
                    r_end = palm_roll_deg(qs[j], qs[k_end])
                    lbl_end = label_motion(dp_end[0], dp_end[1], dp_end[2], r_end)
                    tail_drift = float(np.linalg.norm(ps[k_end] - ps[k]) * 100.0)
                    # Bonus for segments that stay settled in the Same label at end_s
                    if lbl_end == seg["label"] and tail_drift < 14.0:
                        seg["strength"] = round(seg["strength"] + 10.0, 3)
                    if seg["strength"] >= 0.60:
                        bag_segments.append(seg)
            break

    return bag_segments


def select_clips(segments, max_clips=10):
    """Select up to max_clips covering as many different labels as possible
    (strongest per label first, then next strongest).
    """
    dedup = []
    for seg in sorted(segments, key=lambda s: s["strength"], reverse=True):
        if any(
            d["label"] == seg["label"] and abs(d["start_s"] - seg["start_s"]) < 1.5
            for d in dedup
        ):
            continue
        dedup.append(seg)

    by_label = {}
    for seg in dedup:
        by_label.setdefault(seg["label"], []).append(seg)
    for lbl in by_label:
        by_label[lbl].sort(key=lambda s: s["strength"], reverse=True)

    labels_order = sorted(
        by_label.keys(), key=lambda l: by_label[l][0]["strength"], reverse=True
    )

    selected = []
    round_idx = 0
    while len(selected) < max_clips:
        added = False
        for lbl in labels_order:
            if round_idx < len(by_label[lbl]):
                cand = by_label[lbl][round_idx]
                if cand not in selected:
                    selected.append(cand)
                    added = True
                    if len(selected) >= max_clips:
                        break
        if not added:
            break
        round_idx += 1

    output = []
    for idx, seg in enumerate(selected, 1):
         output.append(
            {
                "id": idx,
                "label": seg["label"],
                "start_s": seg["start_s"],
                "end_s": seg["end_s"],
                "pause_end_s": seg["pause_end_s"],
                "motion_end_s": seg["motion_end_s"],
                "dx_cm": seg["dx_cm"],
                "dy_cm": seg["dy_cm"],
                "dz_cm": seg["dz_cm"],
                "roll_deg": seg["roll_deg"],
                "duration": seg["duration"],
            }
        )
    return output


def load_bag_samples(bag_path):
    """Read /fr3_operator/teleop_sample from a ROS 2 mcap bag via rosbag2_py."""
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from std_msgs.msg import String

    bag = Path(bag_path)
    if (bag / "d8").is_dir():
        bag = bag / "d8"

    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(bag), storage_id="mcap"),
        rosbag2_py.ConverterOptions("", ""),
    )
    meta = reader.get_metadata()
    start_ns = meta.starting_time.nanoseconds
    duration_s = meta.duration.nanoseconds * 1e-9
    reader.set_filter(rosbag2_py.StorageFilter(topics=[SAMPLE_TOPIC]))

    samples = []
    prev_q = None
    while reader.has_next():
        topic, data, receipt = reader.read_next()
        if topic != SAMPLE_TOPIC:
            continue
        pkt = json.loads(deserialize_message(data, String).data)
        parsed = control_sample(pkt, prev_q)
        if parsed is None:
            continue
        stamp_ns, p, q, orientation_current = parsed
        prev_q = q
        t_s = (receipt - start_ns) * 1e-9
        samples.append(
            {
                "t": t_s,
                "stamp_ns": stamp_ns,
                "p": p,
                "q": q,
                "orientation_current": orientation_current,
            }
        )
    return samples, duration_s


def format_clips_table(clips):
    header = (
        f"{'ID':>2} | {'Label':<20} | {'Start (s)':>9} | {'End (s)':>9} | "
        f"{'Dur (s)':>7} | {'dX (cm)':>8} | {'dY (cm)':>8} | {'dZ (cm)':>8} | {'Roll (deg)':>10}"
    )
    sep = "-" * len(header)
    lines = [header, sep]
    for c in clips:
        lines.append(
            f"{c['id']:>2} | {c['label']:<20} | {c['start_s']:>9.3f} | {c['end_s']:>9.3f} | "
            f"{c['duration']:>7.2f} | {c['dx_cm']:>+8.2f} | {c['dy_cm']:>+8.2f} | "
            f"{c['dz_cm']:>+8.2f} | {c['roll_deg']:>+10.2f}"
        )
    return "\n".join(lines)


def main():
    default_bag = Path("/home/spot-teleop/bags/20261023_200513_mao1")
    if not default_bag.exists():
        default_bag = Path("/home/hipolito/spot-teleop/bags/20261023_200513_mao1")

    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("bag", nargs="?", type=Path, default=default_bag)
    ap.add_argument("--out", type=Path, default=HERE / "run" / "clips.json")
    ap.add_argument("--max-clips", type=int, default=10)
    args = ap.parse_args()

    samples, duration_s = load_bag_samples(args.bag)
    segments = find_segments(samples, max_duration_s=duration_s)
    clips = select_clips(segments, max_clips=args.max_clips)

    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(json.dumps(clips, indent=2) + "\n", encoding="utf-8")
    print(format_clips_table(clips), flush=True)
    print(f"\nWrote {len(clips)} clips to {args.out}", flush=True)


if __name__ == "__main__":
    main()
