#!/usr/bin/env python3
"""Replay selected hand motion clips through sim_session + cuRobo MPC + MuJoCo and produce side-by-side review videos."""
import argparse
import csv
import json
import math
import os
from pathlib import Path
import shutil
import subprocess
import sys
import threading
import time
from urllib.error import HTTPError, URLError
from urllib.request import Request, urlopen

import cv2
import numpy as np

HERE = Path(__file__).resolve().parent
if str(HERE) not in sys.path:
    sys.path.insert(0, str(HERE))

from home_pose import HOME_Q, HOME_TCP_P, HOME_TCP_Q
from make_clips import label_motion, palm_roll_deg, tool_roll_deg
from teleop_mapping import control_sample
from review_rotation import profile_for_label, hand_angle, tool_angle, direction

SIM_URL = "http://127.0.0.1:8791"
BAG_URL = "http://127.0.0.1:8792"
ARM_JOINTS = [f"fr3_joint{i}" for i in range(1, 8)]


def http_get_json(url, timeout=2.0):
    with urlopen(url, timeout=timeout) as resp:
        return json.load(resp)


def post_sim_action(action, value=None, timeout=3.0):
    payload = {"action": action}
    if value is not None:
        payload["value"] = value
    req = Request(
        f"{SIM_URL}/control",
        data=json.dumps(payload).encode("utf-8"),
        headers={"Content-Type": "application/json", "X-Sim-Control": "1"},
        method="POST",
    )
    with urlopen(req, timeout=timeout) as resp:
        return json.load(resp)


def post_bag_control(action, start_s=None, end_s=None, timeout=3.0):
    payload = {"action": action}
    if start_s is not None:
        payload["start_s"] = float(start_s)
    if end_s is not None:
        payload["end_s"] = float(end_s)
    req = Request(
        f"{BAG_URL}/control",
        data=json.dumps(payload).encode("utf-8"),
        headers={"Content-Type": "application/json", "X-Bag-Control": "1"},
        method="POST",
    )
    with urlopen(req, timeout=timeout) as resp:
        return json.load(resp)


def evaluate_intended_sign(label, hand_delta, robot_delta):
    """Evaluate whether robot TCP motion matches the intended mapping sign convention:
      - hand toward camera (+dX) -> TCP down (-dz)
      - hand away (-dX) -> TCP up (+dz)
      - hand operator right (-dY) -> base yaws right (-yaw_deg / -dy)
      - hand operator left (+dY) -> base yaws left (+yaw_deg / +dy)
      - hand up (+dZ) -> reach out (+dr / +dx)
      - hand down (-dZ) -> reach in (-dr / -dx)
      - palm roll (+roll_deg) -> gripper roll about tool z (+roll_deg)
    """
    h_dx = float(hand_delta["dx_cm"])
    h_dy = float(hand_delta["dy_cm"])
    h_dz = float(hand_delta["dz_cm"])
    h_roll = float(hand_delta["roll_deg"])

    r_dx = float(robot_delta["dx_cm"])
    r_dy = float(robot_delta["dy_cm"])
    r_dz = float(robot_delta["dz_cm"])
    r_dr = float(robot_delta["dr_cm"])
    r_yaw = float(robot_delta["yaw_deg"])
    r_roll = float(robot_delta["roll_deg"])

    per_axis = {
        "hand_x_to_tcp_z": (r_dz * h_dx < 0.0) if abs(h_dx) >= 1.0 else None,
        "hand_y_to_base_yaw": (r_yaw * h_dy > 0.0) if abs(h_dy) >= 1.0 else None,
        "hand_z_to_reach_r": (r_dr * h_dz > 0.0) if abs(h_dz) >= 1.0 else None,
        "palm_roll_to_tool_roll": (r_roll * h_roll > 0.0) if abs(h_roll) >= 3.0 else None,
    }

    if label == "toward camera":
        dominant_ok = r_dz < -0.2
        detail = f"hand dX={h_dx:+.2f} cm (>0) -> expected TCP dz<0, got dz={r_dz:+.2f} cm"
    elif label == "away from camera":
        dominant_ok = r_dz > 0.2
        detail = f"hand dX={h_dx:+.2f} cm (<0) -> expected TCP dz>0, got dz={r_dz:+.2f} cm"
    elif label == "operator right (-Y)":
        dominant_ok = (r_yaw < -0.2) and (r_dy < -0.2)
        detail = f"hand dY={h_dy:+.2f} cm (<0) -> expected yaw<0, got yaw={r_yaw:+.2f} deg (dy={r_dy:+.2f} cm)"
    elif label == "operator left (+Y)":
        dominant_ok = (r_yaw > 0.2) and (r_dy > 0.2)
        detail = f"hand dY={h_dy:+.2f} cm (>0) -> expected yaw>0, got yaw={r_yaw:+.2f} deg (dy={r_dy:+.2f} cm)"
    elif label == "hand up":
        dominant_ok = r_dr > 0.2
        detail = f"hand dZ={h_dz:+.2f} cm (>0) -> expected radial reach dr>0, got dr={r_dr:+.2f} cm (dx={r_dx:+.2f} cm)"
    elif label == "hand down":
        dominant_ok = r_dr < -0.2
        detail = f"hand dZ={h_dz:+.2f} cm (<0) -> expected radial reach dr<0, got dr={r_dr:+.2f} cm (dx={r_dx:+.2f} cm)"
    elif label == "roll cw":
        dominant_ok = r_roll > 0.5
        detail = f"hand roll={h_roll:+.2f} deg (>0) -> expected tool roll>0, got roll={r_roll:+.2f} deg"
    elif label == "roll ccw":
        dominant_ok = r_roll < -0.5
        detail = f"hand roll={h_roll:+.2f} deg (<0) -> expected tool roll<0, got roll={r_roll:+.2f} deg"
    else:
        # mixed: check the two strongest normalized hand components
        comps = [
            (abs(h_dx) / 10.0, "hand_x_to_tcp_z"),
            (abs(h_dy) / 10.0, "hand_y_to_base_yaw"),
            (abs(h_dz) / 10.0, "hand_z_to_reach_r"),
            (abs(h_roll) / 45.0, "palm_roll_to_tool_roll"),
        ]
        comps.sort(key=lambda x: x[0], reverse=True)
        top_keys = [comps[0][1], comps[1][1]]
        checks = [per_axis[k] for k in top_keys if per_axis[k] is not None]
        dominant_ok = bool(checks and all(checks))
        detail = f"mixed top axes {top_keys}: { {k: per_axis[k] for k in top_keys} }"

    return bool(dominant_ok), detail, per_axis


def safe_ratio(num, den, min_den=0.5):
    if abs(float(den)) < min_den:
        return None
    return round(float(num) / float(den), 3)


def compute_ratios(hand_delta, robot_delta):
    h_dx = float(hand_delta["dx_cm"])
    h_dy = float(hand_delta["dy_cm"])
    h_dz = float(hand_delta["dz_cm"])
    h_roll = float(hand_delta["roll_deg"])

    r_dx = float(robot_delta["dx_cm"])
    r_dy = float(robot_delta["dy_cm"])
    r_dz = float(robot_delta["dz_cm"])
    r_dr = float(robot_delta["dr_cm"])
    r_yaw = float(robot_delta["yaw_deg"])
    r_roll = float(robot_delta["roll_deg"])

    return {
        "mapped": {
            "tcp_dz_over_hand_dx": safe_ratio(r_dz, h_dx, 0.5),
            "tcp_dy_over_hand_dy": safe_ratio(r_dy, h_dy, 0.5),
            "base_yaw_deg_over_hand_dy_cm": safe_ratio(r_yaw, h_dy, 0.5),
            "tcp_dr_over_hand_dz": safe_ratio(r_dr, h_dz, 0.5),
            "tcp_dx_over_hand_dz": safe_ratio(r_dx, h_dz, 0.5),
            "tool_roll_over_hand_roll": safe_ratio(r_roll, h_roll, 2.0),
        },
        "cartesian": {
            "dx_over_dX": safe_ratio(r_dx, h_dx, 0.5),
            "dy_over_dY": safe_ratio(r_dy, h_dy, 0.5),
            "dz_over_dZ": safe_ratio(r_dz, h_dz, 0.5),
            "roll_over_roll": safe_ratio(r_roll, h_roll, 2.0),
        },
    }


def _nearest_index(times, target_t):
    if not times:
        return None
    idx = int(np.searchsorted(times, target_t))
    if idx <= 0:
        return 0
    if idx >= len(times):
        return len(times) - 1
    if abs(times[idx] - target_t) < abs(times[idx - 1] - target_t):
        return idx
    return idx - 1


def _latest_at_or_before(times, target_t):
    if not times:
        return None
    idx = int(np.searchsorted(times, target_t, side="right")) - 1
    return max(0, idx)


def ffmpeg_has_libx264():
    ffmpeg_bin = shutil.which("ffmpeg")
    if not ffmpeg_bin:
        return False
    try:
        res = subprocess.run(
            [ffmpeg_bin, "-encoders"],
            capture_output=True,
            text=True,
            timeout=5,
            check=False,
        )
        return "libx264" in (res.stdout or "")
    except Exception:
        return False


def transcode_to_h264(src_mp4, dst_mp4):
    ffmpeg_bin = shutil.which("ffmpeg")
    if not ffmpeg_bin:
        raise RuntimeError("ffmpeg not found")
    tmp_out = str(dst_mp4) + ".h264.tmp.mp4"
    cmd = [
        ffmpeg_bin,
        "-y",
        "-loglevel",
        "error",
        "-i",
        str(src_mp4),
        "-c:v",
        "libx264",
        "-preset",
        "fast",
        "-crf",
        "22",
        "-pix_fmt",
        "yuv420p",
        "-movflags",
        "+faststart",
        tmp_out,
    ]
    subprocess.run(cmd, check=True, timeout=120)
    os.replace(tmp_out, str(dst_mp4))


def render_side_by_side_video(
    clip_id,
    label,
    op_frames,
    mj_frames,
    hand_series,
    robot_series,
    phase_series,
    out_mp4,
    fps=15.0,
):
    """Build side-by-side 1280x408 (title 48px + two 640x360 panels) video aligned on wall time at 15 fps."""
    review_profile = profile_for_label(label)
    all_t = [f["wall_time"] for f in op_frames] + [f["wall_time"] for f in mj_frames]
    if not all_t:
        t0, t1 = time.time(), time.time() + 1.0
    else:
        t0 = min(all_t)
        t1 = max(all_t)
        if op_frames and mj_frames:
            t0 = max(op_frames[0]["wall_time"], mj_frames[0]["wall_time"])
            t1 = min(op_frames[-1]["wall_time"], mj_frames[-1]["wall_time"])
            if t1 <= t0:
                t0 = min(all_t)
                t1 = max(all_t)

    dt = 1.0 / float(fps)
    grid = np.arange(t0, t1 + 1e-6, dt)
    if len(grid) < 2:
        grid = np.array([t0, t0 + dt])

    op_times = [f["wall_time"] for f in op_frames]
    mj_times = [f["wall_time"] for f in mj_frames]
    hand_times = [h["t"] for h in hand_series]
    robot_times = [r["t"] for r in robot_series]
    phase_times = [p["t"] for p in phase_series]

    title_h = 48
    panel_w, panel_h = 640, 360
    total_w = panel_w * 2
    total_h = title_h + panel_h

    raw_mp4 = Path(str(out_mp4) + ".raw.mp4")
    writer = None
    used_codec = None
    for codec in ("avc1", "mp4v"):
        w = cv2.VideoWriter(
            str(raw_mp4),
            cv2.VideoWriter_fourcc(*codec),
            float(fps),
            (total_w, total_h),
        )
        if w.isOpened():
            writer = w
            used_codec = f"cv2:{codec}"
            break
    if writer is None:
        raise RuntimeError("Failed to open cv2.VideoWriter")

    font = cv2.FONT_HERSHEY_SIMPLEX
    for idx_f, t_cur in enumerate(grid):
        canvas = np.zeros((total_h, total_w, 3), dtype=np.uint8)
        canvas[:title_h, :] = (28, 30, 36)

        # Left: operator RGB
        left_img = np.zeros((panel_h, panel_w, 3), dtype=np.uint8)
        oi = _nearest_index(op_times, t_cur)
        if oi is not None:
            im = cv2.imread(str(op_frames[oi]["path"]), cv2.IMREAD_COLOR)
            if im is not None:
                if (im.shape[1], im.shape[0]) != (panel_w, panel_h):
                    im = cv2.resize(im, (panel_w, panel_h), interpolation=cv2.INTER_AREA)
                left_img = im

        # Right: MuJoCo view
        right_img = np.zeros((panel_h, panel_w, 3), dtype=np.uint8)
        mi = _nearest_index(mj_times, t_cur)
        if mi is not None:
            im = cv2.imread(str(mj_frames[mi]["path"]), cv2.IMREAD_COLOR)
            if im is not None:
                if (im.shape[1], im.shape[0]) != (panel_w, panel_h):
                    im = cv2.resize(im, (panel_w, panel_h), interpolation=cv2.INTER_AREA)
                right_img = im

        canvas[title_h:, :panel_w] = left_img
        canvas[title_h:, panel_w:] = right_img

        # Lookup phase, hand delta, robot delta at t_cur
        pi = _latest_at_or_before(phase_times, t_cur)
        phase_str = phase_series[pi]["phase"] if pi is not None else "idle"

        hi = _latest_at_or_before(hand_times, t_cur)
        h_cur = hand_series[hi] if hi is not None else {"dx_cm": 0.0, "dy_cm": 0.0, "dz_cm": 0.0, "roll_deg": 0.0}

        ri = _latest_at_or_before(robot_times, t_cur)
        r_cur = (
            robot_series[ri]
            if ri is not None
            else {"dx_cm": 0.0, "dy_cm": 0.0, "dz_cm": 0.0, "dr_cm": 0.0, "yaw_deg": 0.0, "roll_deg": 0.0}
        )

        # Title bar text
        title_txt = f"clip {clip_id:02d}: {label}   |   t={t_cur - t0:4.1f}s   |   phase: {phase_str}"
        cv2.putText(canvas, title_txt, (16, 31), font, 0.68, (240, 240, 240), 2, cv2.LINE_AA)

        # Overlay bars at bottom of each panel
        overlay_top = total_h - 62
        canvas[overlay_top:total_h, :panel_w] = (
            canvas[overlay_top:total_h, :panel_w].astype(np.float32) * 0.30
        ).astype(np.uint8)
        canvas[overlay_top:total_h, panel_w:] = (
            canvas[overlay_top:total_h, panel_w:].astype(np.float32) * 0.30
        ).astype(np.uint8)

        hand_line1 = (
            f"HAND delta: dX={h_cur['dx_cm']:+5.1f}  dY={h_cur['dy_cm']:+5.1f}  "
            f"dZ={h_cur['dz_cm']:+5.1f} cm"
        )
        hand_line2 = f"Palm roll: {h_cur['roll_deg']:+5.1f} deg (rel. to session anchor)"
        if review_profile:
            hand_degrees=h_cur.get('review_angle_deg',0.)
            hand_line1=f"MAO: {hand_degrees:+5.1f} graus | {direction(hand_degrees)}"
            hand_line2="Inclinacao dos dedos" if review_profile=='pitch' else "Palma para os lados"
        cv2.putText(canvas, hand_line1, (12, total_h - 36), font, 0.50, (120, 255, 160), 1, cv2.LINE_AA)
        cv2.putText(canvas, hand_line2, (12, total_h - 12), font, 0.48, (210, 240, 220), 1, cv2.LINE_AA)

        rob_line1 = (
            f"ROBOT TCP: dx={r_cur['dx_cm']:+5.1f}  dy={r_cur['dy_cm']:+5.1f}  "
            f"dz={r_cur['dz_cm']:+5.1f} cm (dr={r_cur['dr_cm']:+5.1f})"
        )
        rob_line2 = (
            f"Base yaw: {r_cur['yaw_deg']:+5.1f} deg   |   Tool roll: {r_cur['roll_deg']:+5.1f} deg"
        )
        if review_profile:
            measured=r_cur.get('review_angle_deg',0.)
            rob_line1=f"GARRA REAL: {measured:+5.1f} graus | {direction(measured)}"
            rob_line2="Haste laranja = orientacao MEDIDA da garra"
        cv2.putText(
            canvas, rob_line1, (panel_w + 12, total_h - 36), font, 0.50, (120, 220, 255), 1, cv2.LINE_AA
        )
        cv2.putText(
            canvas, rob_line2, (panel_w + 12, total_h - 12), font, 0.48, (220, 235, 250), 1, cv2.LINE_AA
        )

        # Divider line between panels
        cv2.line(canvas, (panel_w, title_h), (panel_w, total_h), (90, 90, 100), 2)
        writer.write(canvas)

    writer.release()

    if ffmpeg_has_libx264():
        transcode_to_h264(raw_mp4, out_mp4)
        raw_mp4.unlink(missing_ok=True)
        return f"{used_codec} -> ffmpeg libx264 (yuv420p)"
    else:
        os.replace(str(raw_mp4), str(out_mp4))
        return used_codec


def write_index_html(out_root, summary_data):
    clips = summary_data.get("clips", [])
    rows_html = []
    for c in clips:
        cid = c["id"]
        lbl = c["label"]
        mp4_rel = f"clip_{cid:02d}.mp4"
        hd = c["hand_delta"]
        rd = c["robot_tcp_delta"]
        sign_ok = c["intended_sign_match"]
        badge_color = "#2ea043" if sign_ok else "#da3633"
        badge_txt = "MATCHES INTENDED SIGN" if sign_ok else "CONTRADICTS / CHECK SIGN"
        err_html = f"<p style='color:#ff7b72'><b>Error/Warning:</b> {c['error']}</p>" if c.get("error") else ""
        ratios = c.get("ratio_robot_hand", {}).get("mapped", {})
        rows_html.append(
            f"""
            <div class="card">
              <h2>Clip {cid:02d}: {lbl}
                <span class="badge" style="background:{badge_color}">{badge_txt}</span>
              </h2>
              <p class="meta">Bag window: <b>{c['start_s']:.2f}s – {c['end_s']:.2f}s</b> (motion {c['duration']:.2f}s) | Active anchor at: {c.get('active_after_s')}s</p>
              {err_html}
              <video controls preload="metadata" width="1280" src="{mp4_rel}"></video>
              <table>
                <tr>
                  <th>Source</th>
                  <th>dX / dx (cm)</th>
                  <th>dY / dy (cm)</th>
                  <th>dZ / dz (cm)</th>
                  <th>Reach dr (cm)</th>
                  <th>Base Yaw (deg)</th>
                  <th>Roll (deg)</th>
                </tr>
                <tr>
                  <td><b>Hand (rel. anchor)</b></td>
                  <td>{hd['dx_cm']:+.2f}</td>
                  <td>{hd['dy_cm']:+.2f}</td>
                  <td>{hd['dz_cm']:+.2f}</td>
                  <td>—</td>
                  <td>—</td>
                  <td>{hd['roll_deg']:+.2f}</td>
                </tr>
                <tr>
                  <td><b>Robot TCP (rel. active)</b></td>
                  <td>{rd['dx_cm']:+.2f}</td>
                  <td>{rd['dy_cm']:+.2f}</td>
                  <td>{rd['dz_cm']:+.2f}</td>
                  <td>{rd['dr_cm']:+.2f}</td>
                  <td>{rd['yaw_deg']:+.2f}</td>
                  <td>{rd['roll_deg']:+.2f}</td>
                </tr>
              </table>
              <p class="ratios">
                <b>Mapped Ratios (Robot / Hand):</b>
                TCP_dz / Hand_dX = <code>{ratios.get('tcp_dz_over_hand_dx')}</code> |
                Base_yaw / Hand_dY = <code>{ratios.get('base_yaw_deg_over_hand_dy_cm')}</code> deg/cm (TCP_dy / Hand_dY = <code>{ratios.get('tcp_dy_over_hand_dy')}</code>) |
                TCP_dr / Hand_dZ = <code>{ratios.get('tcp_dr_over_hand_dz')}</code> |
                Tool_roll / Palm_roll = <code>{ratios.get('tool_roll_over_hand_roll')}</code>
              </p>
              <p class="detail">{c.get('sign_detail', '')}</p>
            </div>
            """
        )

    html = f"""<!doctype html>
<html lang="en">
<head>
  <meta charset="utf-8">
  <title>FR3 Teleop Bag Review Clips ({summary_data.get('run_id')})</title>
  <style>
    body {{ background: #14161a; color: #e6edf3; font: 15px/1.45 -apple-system, BlinkMacSystemFont, sans-serif; max-width: 1320px; margin: 24px auto; padding: 0 16px; }}
    h1 {{ border-bottom: 1px solid #30363d; padding-bottom: 10px; }}
    .card {{ background: #1c2026; border: 1px solid #30363d; border-radius: 8px; padding: 16px; margin-bottom: 24px; }}
    .badge {{ font-size: 13px; padding: 4px 10px; border-radius: 12px; color: #fff; margin-left: 12px; vertical-align: middle; }}
    video {{ width: 100%; max-width: 1280px; border-radius: 6px; background: #000; margin: 8px 0; }}
    table {{ border-collapse: collapse; width: 100%; margin: 10px 0; }}
    th, td {{ border: 1px solid #30363d; padding: 8px 10px; text-align: right; }}
    th:first-child, td:first-child {{ text-align: left; }}
    th {{ background: #22272e; }}
    code {{ background: #2d333b; padding: 2px 6px; border-radius: 4px; }}
    .meta, .ratios, .detail {{ margin: 6px 0; color: #adbac7; }}
  </style>
</head>
<body>
  <h1>FR3 Teleop Review Clips — Run <code>{summary_data.get('run_id')}</code></h1>
  <p>Encoder: <code>{summary_data.get('encoder')}</code> | Total run time: <b>{summary_data.get('total_run_time_s')} s</b></p>
  {''.join(rows_html)}
</body>
</html>
"""
    index_path = Path(out_root) / "index.html"
    index_path.write_text(html, encoding="utf-8")
    return index_path


def transcode_run_dir(run_dir):
    run_path = Path(run_dir).resolve()
    summary_path = run_path / "summary.json"
    summary = json.loads(summary_path.read_text(encoding="utf-8"))
    for c in summary.get("clips", []):
        cid = int(c["id"])
        mp4_path = run_path / f"clip_{cid:02d}.mp4"
        if mp4_path.is_file():
            transcode_to_h264(mp4_path, mp4_path)
            clip_sub_mp4 = run_path / f"clip_{cid:02d}" / f"clip_{cid:02d}.mp4"
            if clip_sub_mp4.parent.is_dir():
                shutil.copy2(mp4_path, clip_sub_mp4)
        c["encoder"] = "cv2:mp4v -> host ffmpeg libx264 (yuv420p)"
    summary["encoder"] = "cv2:mp4v -> host ffmpeg libx264 (yuv420p)"
    summary_path.write_text(json.dumps(summary, indent=2) + "\n", encoding="utf-8")
    write_index_html(run_path, summary)


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--clips", type=Path, default=HERE / "run" / "clips.json")
    ap.add_argument("--out-base", type=Path, default=Path("/home/spot-teleop/clips_out"))
    ap.add_argument("--run-id", type=str, default=None)
    ap.add_argument("--transcode-dir", type=Path, default=None)
    args = ap.parse_args()

    if args.transcode_dir is not None:
        transcode_run_dir(args.transcode_dir)
        print(f"Transcoded {args.transcode_dir} to H.264 yuv420p", flush=True)
        return

    import rclpy
    from geometry_msgs.msg import PoseStamped
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import Image, JointState
    from std_msgs.msg import Empty, String

    class ClipMonitor(Node):
        def __init__(self):
            super().__init__("fr3_run_clips_monitor")
            self.lock = threading.Lock()
            self.tcp_p = None
            self.tcp_q = None
            self.tcp_at = 0.0
            self.q = None
            self.qd = None
            self.js_at = 0.0
            self.last_sample = None

            self.recording = False
            self.op_dir = None
            self.op_idx = 0
            self.op_frames = []
            self.tcp_log = []
            self.js_log = []
            self.sample_log = []

            self.reset_pub = self.create_publisher(Empty, "/fr3_sim/reset", 10)
            self.record_pub = self.create_publisher(String, "/fr3_sim/record", 10)

            self.create_subscription(PoseStamped, "/fr3_sim/tcp", self.on_tcp, 10)
            self.create_subscription(JointState, "/fr3_sim/joint_states", self.on_js, 10)
            self.create_subscription(
                Image, "/fr3_sim/operator/rgb", self.on_rgb, qos_profile_sensor_data
            )
            self.create_subscription(
                String, "/fr3_sim/operator/sample", self.on_sample, qos_profile_sensor_data
            )

        def start_clip_capture(self, op_dir):
            op_dir = Path(op_dir)
            op_dir.mkdir(parents=True, exist_ok=True)
            with self.lock:
                self.op_dir = op_dir
                self.op_idx = 0
                self.op_frames = []
                self.tcp_log = []
                self.js_log = []
                self.sample_log = []
                self.recording = True

        def stop_clip_capture(self):
            with self.lock:
                self.recording = False
                return (
                    list(self.op_frames),
                    list(self.tcp_log),
                    list(self.js_log),
                    list(self.sample_log),
                )

        def on_tcp(self, msg):
            now_w = time.time()
            now_m = time.monotonic()
            p = msg.pose.position
            q = msg.pose.orientation
            tp = np.array([p.x, p.y, p.z], dtype=np.float64)
            tq = np.array([q.x, q.y, q.z, q.w], dtype=np.float64)
            with self.lock:
                self.tcp_p = tp
                self.tcp_q = tq
                self.tcp_at = now_m
                if self.recording:
                    self.tcp_log.append(
                        {"wall_time": now_w, "p": tp.tolist(), "q": tq.tolist()}
                    )

        def on_js(self, msg):
            now_w = time.time()
            now_m = time.monotonic()
            pos_map = dict(zip(msg.name, msg.position))
            vel_map = (
                dict(zip(msg.name, msg.velocity))
                if len(msg.velocity) == len(msg.name)
                else {}
            )
            if all(j in pos_map for j in ARM_JOINTS):
                q_arr = [float(pos_map[j]) for j in ARM_JOINTS]
                qd_arr = [float(vel_map.get(j, 0.0)) for j in ARM_JOINTS]
                with self.lock:
                    self.q = np.array(q_arr, dtype=np.float64)
                    self.qd = np.array(qd_arr, dtype=np.float64)
                    self.js_at = now_m
                    if self.recording:
                        self.js_log.append(
                            {"wall_time": now_w, "q": q_arr, "qd": qd_arr}
                        )

        def on_sample(self, msg):
            now_w = time.time()
            try:
                pkt = json.loads(msg.data)
            except Exception:
                return
            with self.lock:
                prev_q = None if self.last_sample is None else self.last_sample[1]
            parsed = control_sample(pkt, prev_q)
            if parsed is None:
                return
            stamp_ns, p, q, cur = parsed
            with self.lock:
                self.last_sample = (p, q)
                if self.recording:
                    self.sample_log.append(
                        {
                            "wall_time": now_w,
                            "stamp_ns": int(stamp_ns),
                            "p": p.tolist(),
                            "q": q.tolist(),
                            "orientation_current": bool(cur),
                        }
                    )

        def on_rgb(self, msg):
            now_w = time.time()
            with self.lock:
                if not self.recording or self.op_dir is None:
                    return
                idx = self.op_idx
                self.op_idx += 1
                op_dir = self.op_dir
            try:
                arr = (
                    np.frombuffer(msg.data, dtype=np.uint8)
                    .reshape(msg.height, msg.step)[:, : msg.width * 3]
                    .reshape(msg.height, msg.width, 3)
                )
                if (arr.shape[1], arr.shape[0]) != (640, 360):
                    arr = cv2.resize(arr, (640, 360), interpolation=cv2.INTER_AREA)
                fpath = op_dir / f"op_{idx:05d}.jpg"
                cv2.imwrite(str(fpath), arr, [cv2.IMWRITE_JPEG_QUALITY, 85])
                with self.lock:
                    self.op_frames.append(
                        {"frame": idx, "wall_time": now_w, "path": str(fpath)}
                    )
            except Exception:
                pass

    clips = json.loads(args.clips.read_text(encoding="utf-8"))
    run_id = args.run_id or time.strftime("%Y%m%d_%H%M%S")
    out_root = (args.out_base / run_id).resolve()
    out_root.mkdir(parents=True, exist_ok=True)

    t_run_start = time.monotonic()
    rclpy.init(domain_id=188)
    monitor = ClipMonitor()
    spin_thread = threading.Thread(target=rclpy.spin, args=(monitor,), daemon=True)
    spin_thread.start()

    summary_clips = []
    encoder_used = "unknown"

    try:
        # Wait for 8791 and 8792 to be ready
        t_wait0 = time.monotonic()
        while True:
            if time.monotonic() - t_wait0 > 30.0:
                raise RuntimeError("Timeout waiting for 8791/8792 and /fr3_sim/tcp")
            try:
                st8791 = http_get_json(f"{SIM_URL}/state")
                st8792 = http_get_json(f"{BAG_URL}/state")
            except (URLError, HTTPError, OSError):
                time.sleep(0.1)
                continue
            with monitor.lock:
                has_tcp = monitor.tcp_p is not None
            if st8791.get("mpc_ready") and st8791.get("mpc_warmed") and has_tcp and st8792.get("domain") == 188:
                break
            time.sleep(0.05)

        for clip in clips:
            cid = int(clip["id"])
            label = clip["label"]
            start_s = float(clip["start_s"])
            end_s = float(clip["end_s"])
            clip_name = f"clip_{cid:02d}"
            clip_dir = out_root / clip_name
            clip_dir.mkdir(parents=True, exist_ok=True)
            op_dir = clip_dir / "operator"

            print(
                f"\n=== Running {clip_name} ({label}): bag [{start_s:.3f}s .. {end_s:.3f}s] ===",
                flush=True,
            )

            # 1. POST 8791 stop
            post_sim_action("stop")

            # 2. Publish /fr3_sim/reset and wait until /fr3_sim/tcp within 2 mm of HOME_TCP_P and phase is idle
            t_rst0 = time.monotonic()
            monitor.reset_pub.publish(Empty())
            reset_ok = False
            while time.monotonic() - t_rst0 < 6.0:
                with monitor.lock:
                    tp = None if monitor.tcp_p is None else monitor.tcp_p.copy()
                    t_at = monitor.tcp_at
                st = http_get_json(f"{SIM_URL}/state")
                if tp is not None and t_at > t_rst0:
                    err_mm = float(np.linalg.norm(tp - HOME_TCP_P) * 1000.0)
                    if err_mm < 2.0 and st.get("phase") == "idle":
                        reset_ok = True
                        break
                time.sleep(0.03)
            time.sleep(0.15)

            # 3. Start local capture and send /fr3_sim/record start to MuJoCo
            monitor.start_clip_capture(op_dir)
            monitor.record_pub.publish(
                String(data=json.dumps({"cmd": "start", "dir": str(clip_dir), "fps": 15}))
            )
            time.sleep(0.10)

            # 4. Call 8792 seek(start_s, end_s), then POST 8791 start
            clip_error = None if reset_ok else "Reset to HOME exceeded 2 mm tolerance"
            try:
                post_bag_control("seek", start_s=start_s, end_s=end_s)
                post_sim_action("start")
            except Exception as err:
                clip_error = f"Start/seek error: {err}"

            # 5. Poll state until replay ended AND 2.5 s more
            state_polls = []
            t_clip0 = time.monotonic()
            t_clip_wall0 = time.time()
            ended_since = None
            max_wait_s = (end_s - start_s) + 12.0
            active_wall_t = None

            while True:
                now_m = time.monotonic()
                now_w = time.time()
                elapsed_clip = now_m - t_clip0
                try:
                    st_sim = http_get_json(f"{SIM_URL}/state")
                    st_bag = http_get_json(f"{BAG_URL}/state")
                except Exception as err:
                    clip_error = clip_error or f"HTTP poll error: {err}"
                    break

                phase = st_sim.get("phase")
                if phase == "active" and active_wall_t is None:
                    active_wall_t = now_w

                state_polls.append(
                    {
                        "wall_time": now_w,
                        "phase": phase,
                        "relative_xyz_cm": st_sim.get("relative_xyz_cm"),
                        "relative_rpy_deg": st_sim.get("relative_rpy_deg"),
                        "command_tcp": st_sim.get("command_tcp"),
                        "gesture": st_sim.get("gesture"),
                        "gripper": st_sim.get("gripper"),
                        "finger_width_m": st_sim.get("finger_width_m"),
                        "simulation_performance": st_sim.get("simulation_performance"),
                        "mpc_metrics": st_sim.get("mpc_metrics"),
                        "bag_elapsed_s": st_bag.get("elapsed_s"),
                        "bag_ended": st_bag.get("ended"),
                    }
                )

                if st_bag.get("ended"):
                    if ended_since is None:
                        ended_since = now_m
                    elif (now_m - ended_since) >= 2.5:
                        break
                if elapsed_clip >= max_wait_s:
                    clip_error = clip_error or "Timeout waiting for bag replay window to end"
                    break

                time.sleep(0.05)

            # 6. Stop MuJoCo recording and stop sim_session
            monitor.record_pub.publish(String(data=json.dumps({"cmd": "stop"})))
            time.sleep(0.15)
            try:
                post_sim_action("stop")
            except Exception:
                pass

            op_frames, tcp_log, js_log, sample_log = monitor.stop_clip_capture()

            # Read MuJoCo frames.csv
            mj_frames = []
            csv_path = clip_dir / "frames.csv"
            if csv_path.is_file():
                with open(csv_path, "r", encoding="utf-8") as f:
                    for row in csv.reader(f):
                        if not row or row[0] == "frame":
                            continue
                        fidx = int(row[0])
                        sim_t = float(row[1])
                        wall_t = float(row[2])
                        fpath = clip_dir / f"frame_{fidx:05d}.jpg"
                        if fpath.is_file():
                            mj_frames.append(
                                {
                                    "frame": fidx,
                                    "sim_time": sim_t,
                                    "wall_time": wall_t,
                                    "path": str(fpath),
                                }
                            )

            # Determine session anchor (hand_p0, hand_q0, tcp_p0, tcp_q0) at active_wall_t
            if active_wall_t is None:
                clip_error = clip_error or "Session never became active (hand not stable)"

            tcp_times = [x["wall_time"] for x in tcp_log]
            sample_times = [x["wall_time"] for x in sample_log]

            if active_wall_t is not None and tcp_log:
                ti0 = _latest_at_or_before(tcp_times, active_wall_t)
                tcp_p0 = np.asarray(tcp_log[ti0]["p"], dtype=float)
                tcp_q0 = np.asarray(tcp_log[ti0]["q"], dtype=float)
            else:
                tcp_p0 = HOME_TCP_P.copy()
                tcp_q0 = HOME_TCP_Q.copy()

            if active_wall_t is not None and sample_log:
                si0 = _latest_at_or_before(sample_times, active_wall_t)
                hand_p0 = np.asarray(sample_log[si0]["p"], dtype=float)
                hand_q0 = np.asarray(sample_log[si0]["q"], dtype=float)
                # Refine hand_p0 from sim_session's exact relative_xyz_cm if present
                for sp in state_polls:
                    if sp["phase"] == "active" and sp.get("relative_xyz_cm") is not None:
                        si_sp = _latest_at_or_before(sample_times, sp["wall_time"])
                        if si_sp is not None:
                            hand_p0 = (
                                np.asarray(sample_log[si_sp]["p"], dtype=float)
                                - np.asarray(sp["relative_xyz_cm"], dtype=float) / 100.0
                            )
                            break
            elif sample_log:
                hand_p0 = np.asarray(sample_log[0]["p"], dtype=float)
                hand_q0 = np.asarray(sample_log[0]["q"], dtype=float)
            else:
                hand_p0 = np.zeros(3, dtype=float)
                hand_q0 = np.array([0.0, 0.0, 0.0, 1.0], dtype=float)

            # Build time series of hand deltas and robot TCP deltas relative to anchor
            hand_series = []
            for s in sample_log:
                wt = s["wall_time"]
                if active_wall_t is None or wt < active_wall_t:
                    hand_series.append(
                        {"t": wt, "dx_cm": 0.0, "dy_cm": 0.0, "dz_cm": 0.0, "roll_deg": 0.0}
                    )
                else:
                    p_cur = np.asarray(s["p"], dtype=float)
                    q_cur = np.asarray(s["q"], dtype=float)
                    dp = (p_cur - hand_p0) * 100.0
                    rl = palm_roll_deg(hand_q0, q_cur)
                    hand_series.append(
                        {
                            "t": wt,
                            "dx_cm": round(float(dp[0]), 2),
                            "dy_cm": round(float(dp[1]), 2),
                            "dz_cm": round(float(dp[2]), 2),
                            "roll_deg": round(float(rl), 2),
                        }
                    )

            r0_base = math.hypot(tcp_p0[0], tcp_p0[1])
            theta0_base = math.atan2(tcp_p0[1], tcp_p0[0])
            robot_series = []
            for r in tcp_log:
                wt = r["wall_time"]
                if active_wall_t is None or wt < active_wall_t:
                    robot_series.append(
                        {
                            "t": wt,
                            "dx_cm": 0.0,
                            "dy_cm": 0.0,
                            "dz_cm": 0.0,
                            "dr_cm": 0.0,
                            "yaw_deg": 0.0,
                            "roll_deg": 0.0,
                        }
                    )
                else:
                    tp = np.asarray(r["p"], dtype=float)
                    tq = np.asarray(r["q"], dtype=float)
                    dp = (tp - tcp_p0) * 100.0
                    r_cur = math.hypot(tp[0], tp[1])
                    dr_cm = (r_cur - r0_base) * 100.0
                    theta_cur = math.atan2(tp[1], tp[0])
                    dtheta = (theta_cur - theta0_base + math.pi) % (2.0 * math.pi) - math.pi
                    yaw_deg = math.degrees(dtheta)
                    t_roll = tool_roll_deg(tcp_q0, tq)
                    robot_series.append(
                        {
                            "t": wt,
                            "dx_cm": round(float(dp[0]), 2),
                            "dy_cm": round(float(dp[1]), 2),
                            "dz_cm": round(float(dp[2]), 2),
                            "dr_cm": round(float(dr_cm), 2),
                            "yaw_deg": round(float(yaw_deg), 2),
                            "roll_deg": round(float(t_roll), 2),
                        }
                    )

            review_profile=profile_for_label(label)
            if review_profile:
                for point,sample in zip(hand_series,sample_log):
                    point['review_angle_deg']=0. if active_wall_t is None or sample['wall_time']<active_wall_t else round(hand_angle(hand_q0,sample['q'],review_profile),2)
                for point,sample in zip(robot_series,tcp_log):
                    point['review_angle_deg']=0. if active_wall_t is None or sample['wall_time']<active_wall_t else round(tool_angle(tcp_q0,sample['q'],review_profile),2)
            phase_series = [
                {"t": sp["wall_time"], "phase": sp["phase"] or "idle"} for sp in state_polls
            ]

            # Check MPC feasibility
            feasible_flags = [
                bool(sp["mpc_metrics"].get("feasible", True))
                for sp in state_polls
                if sp.get("phase") == "active" and isinstance(sp.get("mpc_metrics"), dict)
            ]
            feasible_ratio = (
                round(float(sum(feasible_flags)) / len(feasible_flags), 4)
                if feasible_flags
                else 1.0
            )
            if feasible_flags and not any(feasible_flags):
                clip_error = clip_error or "MPC infeasible throughout active phase"

            # Hand delta and robot TCP delta at motion_end (before the 0.8s tail return) and at clip end
            end_hand_delta = (
                {k: v for k, v in hand_series[-1].items() if k != "t"}
                if hand_series
                else {
                    "dx_cm": clip["dx_cm"],
                    "dy_cm": clip["dy_cm"],
                    "dz_cm": clip["dz_cm"],
                    "roll_deg": clip["roll_deg"],
                }
            )
            end_robot_delta = (
                {k: v for k, v in robot_series[-1].items() if k != "t"}
                if robot_series
                else {
                    "dx_cm": 0.0,
                    "dy_cm": 0.0,
                    "dz_cm": 0.0,
                    "dr_cm": 0.0,
                    "yaw_deg": 0.0,
                    "roll_deg": 0.0,
                }
            )

            motion_end_s = float(clip.get("motion_end_s", end_s - 0.8))
            if hand_series and robot_series:
                t_mend_wall = hand_series[0]["t"] + (motion_end_s - start_s)
                h_mend = min(hand_series, key=lambda x: abs(x["t"] - t_mend_wall))
                r_mend = min(robot_series, key=lambda x: abs(x["t"] - (t_mend_wall + 0.45)))
                mend_hand_delta = {k: v for k, v in h_mend.items() if k != "t"}
                mend_robot_delta = {k: v for k, v in r_mend.items() if k != "t"}
            else:
                mend_hand_delta = end_hand_delta
                mend_robot_delta = end_robot_delta

            # Use motion_end deltas as primary if the 0.8s tail moved back by > 5 cm
            tail_return_cm = math.hypot(
                end_hand_delta["dx_cm"] - mend_hand_delta["dx_cm"],
                math.hypot(
                    end_hand_delta["dy_cm"] - mend_hand_delta["dy_cm"],
                    end_hand_delta["dz_cm"] - mend_hand_delta["dz_cm"],
                ),
            )
            if tail_return_cm > 5.0:
                final_hand_delta = mend_hand_delta
                final_robot_delta = mend_robot_delta
            else:
                final_hand_delta = end_hand_delta
                final_robot_delta = end_robot_delta

            sign_ok, sign_detail, per_axis_sign = evaluate_intended_sign(
                label, final_hand_delta, final_robot_delta
            )
            ratios = compute_ratios(final_hand_delta, final_robot_delta)

            # Write trace.json
            trace_data = {
                "clip": clip,
                "active_after_s": (
                    None if active_wall_t is None else round(active_wall_t - t_clip_wall0, 3)
                ),
                "anchor": {
                    "hand_p0": hand_p0.tolist(),
                    "hand_q0": hand_q0.tolist(),
                    "tcp_p0": tcp_p0.tolist(),
                    "tcp_q0": tcp_q0.tolist(),
                },
                "final_hand_delta": final_hand_delta,
                "final_robot_tcp_delta": final_robot_delta,
                "ratio_robot_hand": ratios,
                "intended_sign_match": sign_ok,
                "sign_detail": sign_detail,
                "per_axis_sign": per_axis_sign,
                "mpc_feasible_ratio": feasible_ratio,
                "error": clip_error,
                "operator_rgb_count": len(op_frames),
                "mujoco_frame_count": len(mj_frames),
                "state_polls": state_polls,
                "hand_series": hand_series,
                "sample_log": sample_log,
                "robot_series": robot_series,
                "tcp_log": tcp_log,
                "joint_states_count": len(js_log),
            }
            trace_path = clip_dir / "trace.json"
            trace_path.write_text(json.dumps(trace_data, indent=2) + "\n", encoding="utf-8")

            # Build clip_NN.mp4
            mp4_path = out_root / f"{clip_name}.mp4"
            encoder_used = render_side_by_side_video(
                cid,
                label,
                op_frames,
                mj_frames,
                hand_series,
                robot_series,
                phase_series,
                mp4_path,
                fps=15.0,
            )
            shutil.copy2(mp4_path, clip_dir / f"{clip_name}.mp4")

            clip_summary = {
                "id": cid,
                "clip_name": clip_name,
                "label": label,
                "start_s": start_s,
                "end_s": end_s,
                "duration": clip["duration"],
                "active_after_s": trace_data["active_after_s"],
                "planned_hand_delta": {
                    "dx_cm": clip["dx_cm"],
                    "dy_cm": clip["dy_cm"],
                    "dz_cm": clip["dz_cm"],
                    "roll_deg": clip["roll_deg"],
                },
                "hand_delta": final_hand_delta,
                "robot_tcp_delta": final_robot_delta,
                "ratio_robot_hand": ratios,
                "intended_sign_match": sign_ok,
                "sign_detail": sign_detail,
                "per_axis_sign": per_axis_sign,
                "mpc_feasible_ratio": feasible_ratio,
                "operator_frames": len(op_frames),
                "mujoco_frames": len(mj_frames),
                "mp4_path": str(mp4_path),
                "trace_path": str(trace_path),
                "encoder": encoder_used,
                "error": clip_error,
            }
            summary_clips.append(clip_summary)
            print(
                f"  -> {clip_name} done: hand={final_hand_delta} | robot={final_robot_delta} | "
                f"sign_match={sign_ok} ({sign_detail})",
                flush=True,
            )

        total_time_s = round(time.monotonic() - t_run_start, 2)
        summary_data = {
            "run_id": run_id,
            "out_dir": str(out_root),
            "encoder": encoder_used,
            "total_run_time_s": total_time_s,
            "clip_count": len(summary_clips),
            "clips": summary_clips,
        }
        summary_path = out_root / "summary.json"
        summary_path.write_text(json.dumps(summary_data, indent=2) + "\n", encoding="utf-8")
        index_path = write_index_html(out_root, summary_data)

        try:
            subprocess.run(["chown", "-R", "1000:1000", str(args.out_base)], check=False)
        except Exception:
            pass

        print(f"\nCompleted {len(summary_clips)} clips in {total_time_s:.2f}s", flush=True)
        print(f"Summary JSON: {summary_path}", flush=True)
        print(f"Index HTML:   {index_path}", flush=True)
    finally:
        monitor.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        spin_thread.join(timeout=1.0)


if __name__ == "__main__":
    main()
