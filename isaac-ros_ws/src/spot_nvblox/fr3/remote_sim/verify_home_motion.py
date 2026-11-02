#!/usr/bin/env python3
"""Verify MuJoCo FR3 HOME reset and real cuRobo MPC closed-loop motion on DDS 188 / HTTP 8791.

Sequence:
  1. Publish /fr3_sim/reset -> wait until /fr3_sim/tcp is within 2 mm of HOME_TCP_P
  2. POST action 'test' (-3 cm along Y) -> wait until phase returns to 'idle'
  3. POST action 'home' -> wait until phase returns to 'idle'
  4. POST action 'rotate' (+10 deg about Z) -> wait until phase returns to 'idle'
  5. POST action 'home' -> wait until phase returns to 'idle'

Samples /fr3_sim/joint_states, /fr3_sim/tcp, /fr3_sim/joint_command, and HTTP /state at >= 20 Hz.
Saves full report to R/run/verify_home_motion.json and prints PASS/FAIL criteria.
"""
import json
import math
import os
import sys
import threading
import time
from pathlib import Path
from urllib.error import HTTPError, URLError
from urllib.request import Request, urlopen

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Empty

HERE = Path(__file__).resolve().parent
if str(HERE) not in sys.path:
    sys.path.insert(0, str(HERE))
if str(HERE.parents[4] / "spot-ros2_ws/src/arm_pose_estimator") not in sys.path:
    sys.path.insert(0, str(HERE.parents[4] / "spot-ros2_ws/src/arm_pose_estimator"))

from arm_pose_estimator.hand_frame import matrix_quaternion, quaternion_matrix
from home_pose import HOME_Q, HOME_TCP_P, HOME_TCP_Q

ARM_JOINTS = [f"fr3_joint{i}" for i in range(1, 8)]
BASE_URL = "http://127.0.0.1:8791"
SAMPLE_DT = 0.035  # ~28.5 Hz (>= 20 Hz)


def quat_angle_deg(q1, q2):
    d = min(1.0, abs(float(np.dot(q1, q2))))
    return math.degrees(2.0 * math.acos(d))


def get_state():
    with urlopen(f"{BASE_URL}/state", timeout=2.0) as resp:
        return json.load(resp)


def post_action(action, value=None):
    payload = {"action": action}
    if value is not None:
        payload["value"] = value
    req = Request(
        f"{BASE_URL}/control",
        data=json.dumps(payload).encode("utf-8"),
        headers={"Content-Type": "application/json", "X-Sim-Control": "1"},
        method="POST",
    )
    with urlopen(req, timeout=2.0) as resp:
        return json.load(resp)


class VerifyMonitor(Node):
    def __init__(self):
        super().__init__("verify_home_motion_monitor")
        self.lock = threading.Lock()
        self.q = None
        self.qd = None
        self.js_at = 0.0
        self.tcp_p = None
        self.tcp_q = None
        self.tcp_at = 0.0
        self.cmd_q = None
        self.cmd_names = None
        self.cmd_at = 0.0

        self.reset_pub = self.create_publisher(Empty, "/fr3_sim/reset", 10)
        self.create_subscription(JointState, "/fr3_sim/joint_states", self.on_js, 10)
        self.create_subscription(PoseStamped, "/fr3_sim/tcp", self.on_tcp, 10)
        self.create_subscription(JointState, "/fr3_sim/joint_command", self.on_cmd, 10)

    def on_js(self, msg):
        pos_map = dict(zip(msg.name, msg.position))
        vel_map = dict(zip(msg.name, msg.velocity)) if len(msg.velocity) == len(msg.name) else {}
        if all(j in pos_map for j in ARM_JOINTS):
            with self.lock:
                self.q = np.array([pos_map[j] for j in ARM_JOINTS], dtype=np.float64)
                self.qd = np.array([vel_map.get(j, 0.0) for j in ARM_JOINTS], dtype=np.float64)
                self.js_at = time.monotonic()

    def on_tcp(self, msg):
        p = msg.pose.position
        q = msg.pose.orientation
        with self.lock:
            self.tcp_p = np.array([p.x, p.y, p.z], dtype=np.float64)
            self.tcp_q = np.array([q.x, q.y, q.z, q.w], dtype=np.float64)
            self.tcp_at = time.monotonic()

    def on_cmd(self, msg):
        pos_map = dict(zip(msg.name, msg.position))
        if all(j in pos_map for j in ARM_JOINTS):
            with self.lock:
                self.cmd_q = np.array([pos_map[j] for j in ARM_JOINTS], dtype=np.float64)
                self.cmd_names = list(msg.name)
                self.cmd_at = time.monotonic()

    def snapshot(self):
        with self.lock:
            return {
                "q": None if self.q is None else self.q.copy(),
                "qd": None if self.qd is None else self.qd.copy(),
                "tcp_p": None if self.tcp_p is None else self.tcp_p.copy(),
                "tcp_q": None if self.tcp_q is None else self.tcp_q.copy(),
                "cmd_q": None if self.cmd_q is None else self.cmd_q.copy(),
                "cmd_names": None if self.cmd_names is None else list(self.cmd_names),
                "js_at": self.js_at,
                "tcp_at": self.tcp_at,
                "cmd_at": self.cmd_at,
            }


def count_vel_sign_changes(samples, window_s=1.0, deadband_m_s=0.002):
    """Count sign changes of TCP velocity in the last window_s of a leg."""
    if len(samples) < 3:
        return {"x": 0, "y": 0, "z": 0, "max_axis": 0}
    t_end = samples[-1]["t"]
    recent = [s for s in samples if (t_end - s["t"]) <= window_s]
    if len(recent) < 3:
        recent = samples[-3:]
    vels = []
    for a, b in zip(recent[:-1], recent[1:]):
        dt = max(b["t"] - a["t"], 1e-6)
        vels.append((b["tcp_p"] - a["tcp_p"]) / dt)
    vels = np.asarray(vels, dtype=np.float64)
    counts = {}
    for axis_idx, axis_name in enumerate(("x", "y", "z")):
        v = vels[:, axis_idx]
        signs = []
        for val in v:
            if abs(val) > deadband_m_s:
                signs.append(1 if val > 0 else -1)
        changes = sum(1 for s1, s2 in zip(signs[:-1], signs[1:]) if s1 != s2)
        counts[axis_name] = int(changes)
    counts["max_axis"] = max(counts["x"], counts["y"], counts["z"])
    return counts


def run_leg(monitor, leg_name, action, target_p, target_q, timeout_s=10.0):
    t0 = time.monotonic()
    resp = post_action(action)
    samples = []
    reached_at = None

    while True:
        loop_t0 = time.monotonic()
        elapsed = loop_t0 - t0
        st = get_state()
        snap = monitor.snapshot()
        if snap["tcp_p"] is not None and snap["q"] is not None:
            pos_err_mm = float(np.linalg.norm(snap["tcp_p"] - target_p) * 1000.0)
            rot_err_deg = float(quat_angle_deg(snap["tcp_q"], target_q))
            mpc_metrics = st.get("mpc_metrics") or {}
            feasible = bool(mpc_metrics.get("feasible", True))
            cmd_err = (
                float(np.max(np.abs(snap["cmd_q"] - snap["q"])))
                if snap["cmd_q"] is not None
                else 0.0
            )
            max_qd = float(np.max(np.abs(snap["qd"]))) if snap["qd"] is not None else 0.0
            samples.append(
                {
                    "t": elapsed,
                    "phase": st.get("phase"),
                    "tcp_p": snap["tcp_p"],
                    "tcp_q": snap["tcp_q"],
                    "q": snap["q"],
                    "qd": snap["qd"],
                    "cmd_q": snap["cmd_q"],
                    "pos_err_mm": pos_err_mm,
                    "rot_err_deg": rot_err_deg,
                    "feasible": feasible,
                    "cmd_minus_state_max_rad": cmd_err,
                    "max_qd_rad_s": max_qd,
                }
            )
            if (
                reached_at is None
                and elapsed >= 0.05
                and st.get("phase") == "idle"
                and pos_err_mm < 8.0
                and rot_err_deg < 2.0
            ):
                reached_at = elapsed

        if reached_at is not None and (elapsed - reached_at) >= 0.25:
            break
        if elapsed >= timeout_s:
            break

        sleep_s = SAMPLE_DT - (time.monotonic() - loop_t0)
        if sleep_s > 0:
            time.sleep(sleep_s)

    steady_samples = [
        s for s in samples if reached_at is not None and s["t"] >= reached_at
    ]
    if not steady_samples:
        steady_samples = samples[-5:] if len(samples) >= 5 else samples

    final_s = samples[-1]
    feasible_list = [s["feasible"] for s in samples]
    feasible_ratio = float(sum(feasible_list)) / max(len(feasible_list), 1)
    osc = count_vel_sign_changes(samples, window_s=1.0)

    return {
        "leg": leg_name,
        "action": action,
        "post_response": resp,
        "sample_count": len(samples),
        "sample_rate_hz": round(len(samples) / max(samples[-1]["t"], 1e-6), 2),
        "reached": reached_at is not None,
        "time_to_reach_s": round(float(reached_at if reached_at is not None else samples[-1]["t"]), 4),
        "final_tcp_error_mm": round(final_s["pos_err_mm"], 4),
        "final_rotation_error_deg": round(final_s["rot_err_deg"], 4),
        "max_steady_state_cmd_minus_state_rad": round(
            float(max(s["cmd_minus_state_max_rad"] for s in steady_samples)), 6
        ),
        "mpc_feasible_all": bool(all(feasible_list)),
        "mpc_feasible_ratio": round(feasible_ratio, 4),
        "max_joint_speed_rad_s": round(float(max(s["max_qd_rad_s"] for s in samples)), 4),
        "oscillation_vel_sign_changes_last_1s": osc,
        "final_tcp_p": [round(float(x), 6) for x in final_s["tcp_p"]],
        "final_tcp_q": [round(float(x), 6) for x in final_s["tcp_q"]],
        "final_q": [round(float(x), 6) for x in final_s["q"]],
    }


def main():
    rclpy.init(domain_id=188)
    monitor = VerifyMonitor()
    spin_thread = threading.Thread(target=rclpy.spin, args=(monitor,), daemon=True)
    spin_thread.start()

    try:
        # Wait for MPC ready & warmed and ROS topics active
        t_wait0 = time.monotonic()
        while True:
            if time.monotonic() - t_wait0 > 30.0:
                raise RuntimeError("Timeout waiting for sim_session MPC warmup and /fr3_sim/tcp")
            try:
                st = get_state()
            except (URLError, HTTPError, OSError):
                time.sleep(0.1)
                continue
            snap = monitor.snapshot()
            if st.get("mpc_ready") and st.get("mpc_warmed") and snap["tcp_p"] is not None and snap["q"] is not None:
                break
            time.sleep(0.05)

        # 1. Publish /fr3_sim/reset and wait until /fr3_sim/tcp is within 2 mm of HOME_TCP_P
        t_reset0 = time.monotonic()
        monitor.reset_pub.publish(Empty())
        reset_reached_s = None
        reset_tcp_err_mm = None
        reset_rot_err_deg = None
        while time.monotonic() - t_reset0 < 5.0:
            snap = monitor.snapshot()
            if snap["tcp_p"] is not None and snap["tcp_at"] > t_reset0:
                err_mm = float(np.linalg.norm(snap["tcp_p"] - HOME_TCP_P) * 1000.0)
                rot_deg = float(quat_angle_deg(snap["tcp_q"], HOME_TCP_Q))
                if err_mm < 2.0:
                    reset_reached_s = time.monotonic() - t_reset0
                    reset_tcp_err_mm = err_mm
                    reset_rot_err_deg = rot_deg
                    break
            time.sleep(0.02)

        # Hold 0.25 s after reset to confirm steady HOME before starting legs
        time.sleep(0.25)
        snap_after_reset = monitor.snapshot()
        reset_tcp_err_mm = float(np.linalg.norm(snap_after_reset["tcp_p"] - HOME_TCP_P) * 1000.0)
        reset_rot_err_deg = float(quat_angle_deg(snap_after_reset["tcp_q"], HOME_TCP_Q))
        reset_q_err_rad = float(np.max(np.abs(snap_after_reset["q"] - np.array(HOME_Q))))

        # 2. Leg 1: action 'test' (3 cm along -y)
        snap0 = monitor.snapshot()
        test_target_p = snap0["tcp_p"] + np.array([0.0, -0.03, 0.0], dtype=np.float64)
        test_target_q = snap0["tcp_q"].copy()
        leg_test = run_leg(monitor, "test_minus_3cm_y", "test", test_target_p, test_target_q, timeout_s=8.0)

        # 3. Leg 2: action 'home'
        leg_home1 = run_leg(monitor, "home_after_test", "home", HOME_TCP_P, HOME_TCP_Q, timeout_s=10.0)

        # 4. Leg 3: action 'rotate' (10 deg about Z)
        snap_rot0 = monitor.snapshot()
        ang = math.radians(10.0)
        rz = np.array(
            [
                [math.cos(ang), -math.sin(ang), 0.0],
                [math.sin(ang), math.cos(ang), 0.0],
                [0.0, 0.0, 1.0],
            ],
            dtype=np.float64,
        )
        rot_target_p = snap_rot0["tcp_p"].copy()
        rot_target_q = matrix_quaternion(rz @ quaternion_matrix(snap_rot0["tcp_q"]))
        leg_rotate = run_leg(monitor, "rotate_10deg_z", "rotate", rot_target_p, rot_target_q, timeout_s=8.0)

        # 5. Leg 4: action 'home'
        leg_home2 = run_leg(monitor, "home_after_rotate", "home", HOME_TCP_P, HOME_TCP_Q, timeout_s=10.0)

        final_snap = monitor.snapshot()
        final_q = final_snap["q"]
        abs_diff_per_joint = [
            round(float(abs(final_q[i] - HOME_Q[i])), 6) for i in range(7)
        ]
        signed_diff_per_joint = [
            round(float(final_q[i] - HOME_Q[i]), 6) for i in range(7)
        ]

        legs = [leg_test, leg_home1, leg_rotate, leg_home2]

        # Evaluate PASS criteria
        crit_reset = reset_reached_s is not None and reset_tcp_err_mm < 2.0
        crit_test = (
            leg_test["reached"]
            and leg_test["final_tcp_error_mm"] < 8.0
            and leg_test["time_to_reach_s"] <= 6.0
        )
        crit_home1 = (
            leg_home1["reached"]
            and leg_home1["final_tcp_error_mm"] < 8.0
            and leg_home1["final_rotation_error_deg"] < 2.0
            and leg_home1["time_to_reach_s"] <= 8.0
        )
        crit_rotate = (
            leg_rotate["reached"]
            and leg_rotate["final_tcp_error_mm"] < 8.0
            and leg_rotate["final_rotation_error_deg"] < 2.0
            and leg_rotate["time_to_reach_s"] <= 8.0
        )
        crit_home2 = (
            leg_home2["reached"]
            and leg_home2["final_tcp_error_mm"] < 8.0
            and leg_home2["final_rotation_error_deg"] < 2.0
            and leg_home2["time_to_reach_s"] <= 8.0
        )
        crit_feasible = all(lg["mpc_feasible_ratio"] >= 0.90 for lg in legs)
        all_pass = bool(
            crit_reset and crit_test and crit_home1 and crit_rotate and crit_home2 and crit_feasible
        )

        report = {
            "overall_pass": all_pass,
            "joint_command_names": final_snap["cmd_names"],
            "reset": {
                "time_to_within_2mm_s": None if reset_reached_s is None else round(float(reset_reached_s), 4),
                "tcp_error_mm": round(reset_tcp_err_mm, 4),
                "rotation_error_deg": round(reset_rot_err_deg, 4),
                "max_joint_error_from_home_rad": round(reset_q_err_rad, 6),
                "pass": bool(crit_reset),
            },
            "legs": {
                "test": leg_test,
                "home_after_test": leg_home1,
                "rotate": leg_rotate,
                "home_after_rotate": leg_home2,
            },
            "final_home_joint_comparison": {
                "joint_names": ARM_JOINTS,
                "home_q_rad": [round(float(x), 6) for x in HOME_Q],
                "final_q_rad": [round(float(x), 6) for x in final_q],
                "signed_diff_rad": signed_diff_per_joint,
                "abs_diff_per_joint_rad": abs_diff_per_joint,
                "max_abs_diff_rad": round(float(max(abs_diff_per_joint)), 6),
            },
            "criteria": {
                "after_reset_tcp_within_2mm": bool(crit_reset),
                "test_within_8mm_within_6s": bool(crit_test),
                "home_after_test_within_8mm_2deg_within_8s": bool(crit_home1),
                "rotate_within_8mm_2deg_within_8s": bool(crit_rotate),
                "home_after_rotate_within_8mm_2deg_within_8s": bool(crit_home2),
                "every_leg_mpc_feasible_ge_90pct": bool(crit_feasible),
            },
        }

        out_path = HERE / "run" / "verify_home_motion.json"
        out_path.parent.mkdir(parents=True, exist_ok=True)
        out_path.write_text(json.dumps(report, indent=2))

        print(json.dumps(report, indent=2), flush=True)
        print("-" * 72, flush=True)
        print(
            f"[{'PASS' if crit_reset else 'FAIL'}] after reset TCP within 2 mm of HOME: "
            f"err={reset_tcp_err_mm:.4f} mm, rot_err={reset_rot_err_deg:.4f} deg, t={reset_reached_s:.4f} s",
            flush=True,
        )
        print(
            f"[{'PASS' if crit_test else 'FAIL'}] 'test' reaches within 8 mm within 6 s: "
            f"t={leg_test['time_to_reach_s']:.3f} s, tcp_err={leg_test['final_tcp_error_mm']:.3f} mm, "
            f"rot_err={leg_test['final_rotation_error_deg']:.3f} deg",
            flush=True,
        )
        print(
            f"[{'PASS' if crit_home1 else 'FAIL'}] 'home' (after test) returns within 8 mm / 2 deg within 8 s: "
            f"t={leg_home1['time_to_reach_s']:.3f} s, tcp_err={leg_home1['final_tcp_error_mm']:.3f} mm, "
            f"rot_err={leg_home1['final_rotation_error_deg']:.3f} deg",
            flush=True,
        )
        print(
            f"[{'PASS' if crit_rotate else 'FAIL'}] 'rotate' (10 deg) reaches within 8 mm / 2 deg within 8 s: "
            f"t={leg_rotate['time_to_reach_s']:.3f} s, tcp_err={leg_rotate['final_tcp_error_mm']:.3f} mm, "
            f"rot_err={leg_rotate['final_rotation_error_deg']:.3f} deg",
            flush=True,
        )
        print(
            f"[{'PASS' if crit_home2 else 'FAIL'}] 'home' (after rotate) returns within 8 mm / 2 deg within 8 s: "
            f"t={leg_home2['time_to_reach_s']:.3f} s, tcp_err={leg_home2['final_tcp_error_mm']:.3f} mm, "
            f"rot_err={leg_home2['final_rotation_error_deg']:.3f} deg",
            flush=True,
        )
        print(
            f"[{'PASS' if crit_feasible else 'FAIL'}] every leg MPC feasible >= 90%: "
            + ", ".join(f"{lg['leg']}={lg['mpc_feasible_ratio']*100:.1f}%" for lg in legs),
            flush=True,
        )
        print(
            f"[INFO] Final 'home' joint vector vs HOME_Q abs diff per joint (rad): {abs_diff_per_joint} "
            f"(max={max(abs_diff_per_joint):.6f} rad)",
            flush=True,
        )
        print(f"OVERALL: {'PASS' if all_pass else 'FAIL'}", flush=True)
        sys.exit(0 if all_pass else 1)
    finally:
        monitor.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        spin_thread.join(timeout=1.0)


if __name__ == "__main__":
    main()
