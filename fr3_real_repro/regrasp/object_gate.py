"""Jump gate for the object position the 6D grasp target rides on (2026-11-06).

Why: the tracker's target_object TF (sam2_tracker -> /tracking_3d_point -> vm-target-bridge) is good while the arm
is still, but it jumps while the arm moves through the cameras' view. Pad test 21:21Z: (0.442,-0.103,0.232) ->
(0.504,0.056,0.095) -> (0.444,-0.099,0.228) -> (0.564,0.136,0.035) ... within seconds, drill untouched. /grasp_target
= TF + offset jumped with it, so the MPC goal snapped between the drill and the table ("attracts and bounces back").

Rule: hold the last trusted position; take a new one only when all of these hold over the last `window_s`:
  - the arm is still (fr3_hand moved < arm_still_m),
  - the TF is stable (every sample within tf_stable_m of the window median),
  - the hand is not close to the grasp pose (< near_m: the hand itself occludes the object there),
  - the height stays within max_dz_m of the bind height: a standing drill slides, it does not change height
    (21:30Z, arm still at home: the tracker point slid along the line drill (0.47,-0.08,0.23) -> table
    (0.58,0.19,0.02), a centroid of a mask with a second blob, at 1.6 Hz tracker updates; a knocked-over drill
    needs a new grasp, not an offset),
and the window median differs from the held position by > min_update_m. Samples off the height band are dropped
(not used, and they no longer reset the window). A real drill move is therefore picked
up once the arm pauses for ~2.5 s; anything seen while the arm moves is ignored.
"""
from collections import deque

import numpy as np


class ObjectGate:
    def __init__(self, init_pos, offset, window_s=2.5, min_span_s=2.0, max_dz_m=0.02, arm_still_m=0.01, tf_stable_m=0.01,
                 near_m=0.15, min_update_m=0.003):
        self.held = np.asarray(init_pos, float).copy()
        self.offset = np.asarray(offset, float).copy()
        self.window_s, self.min_span_s = window_s, min_span_s
        self.arm_still_m, self.tf_stable_m = arm_still_m, tf_stable_m
        self.near_m, self.min_update_m = near_m, min_update_m
        self.max_dz_m, self.z_ref = max_dz_m, float(self.held[2])
        self.buf = deque()                      # (t, tf_pos, hand_pos)
        self.n_updates = 0

    def update(self, t, tf_pos, hand_pos):
        """Feed one sample (either position may be None). Returns (held object position, reason)."""
        if tf_pos is None or hand_pos is None:
            self.buf.clear()                    # a gap breaks the stillness/stability evidence
            return self.held, 'hold: no TF' if tf_pos is None else 'hold: no hand TF'
        while self.buf and t - self.buf[0][0] > self.window_s:
            self.buf.popleft()
        if abs(float(tf_pos[2]) - self.z_ref) > self.max_dz_m:
            # drop the implausible sample but KEEP the evidence (22:35Z: the tracker jumped to the table blob
            # (0.55,0.10,0.02) every few s with the arm still; resetting the window on each jump delayed adopting a
            # moved drill by ~2 min)
            return self.held, 'hold: TF height off'
        self.buf.append((t, np.asarray(tf_pos, float), np.asarray(hand_pos, float)))
        if t - self.buf[0][0] < self.min_span_s:
            return self.held, 'hold: collecting'
        tfs = np.array([b[1] for b in self.buf])
        hands = np.array([b[2] for b in self.buf])
        if np.max(np.linalg.norm(hands - hands[-1], axis=1)) > self.arm_still_m:
            return self.held, 'hold: arm moving'
        if np.linalg.norm(hands[-1] - (self.held + self.offset)) < self.near_m:
            return self.held, 'hold: hand near grasp'
        med = np.median(tfs, axis=0)
        if np.max(np.linalg.norm(tfs - med, axis=1)) > self.tf_stable_m:
            return self.held, 'hold: TF unstable'
        if np.linalg.norm(med - self.held) <= self.min_update_m:
            return self.held, 'steady'
        self.held = med
        self.n_updates += 1
        return self.held, 'updated'


YAW_OK_REASONS = ('steady', 'updated')


def gate_yaw(rel_new, rel_held, gate_reason):
    """Yaw rides the same evidence as the position (2026-11-07): take the loop's new relative yaw only on a tick
    where the position gate saw the arm still, the hand away from the grasp and a stable TF ('steady'/'updated').
    Otherwise keep the held yaw: near the drill the hand occludes it and the robot segmenter erases the cloud
    around the hand, so the PCA yaw drifts exactly then, and a yaw error swings the target by |offset| * angle
    (17 cm offset: 10 deg = 3 cm) plus the same angle in orientation, the snap the position gate removed."""
    return float(rel_new) if gate_reason in YAW_OK_REASONS else float(rel_held)
