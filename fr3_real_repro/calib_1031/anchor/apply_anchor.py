#!/usr/bin/env python3
"""Compute (print only, publishes nothing) the camera pose corrected by odometry since the anchor:
   python3 apply_anchor.py zed1|zed2 [ANCHOR_DIR]   -> reads anchor_CURRENT + last_good.json
 T_base_link_now = T_base_link_ref @ inv(O_ref) @ O_now ;  T_base_optical_now = T_base_link_now @ T_link_optical
 Refuses if the camera's epoch changed since the anchor (odometry continuity lost: JUMP/GAP/reset in events.log)."""
import sys, json, os, numpy as np, math
cam = sys.argv[1]; D = sys.argv[2] if len(sys.argv) > 2 else "/home/spot_ws/zed_ws/calib_anchor"
A = json.load(open(f"{D}/" + open(f"{D}/ANCHOR_CURRENT").read().strip()))["cams"][cam]; L = json.load(open(f"{D}/last_good.json"))["cams"][cam]
def q2R(q):
    x, y, z, w = q; return np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)], [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)], [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])
if L["epoch"] != 0: sys.exit(f"REFUSED: epoch {L['epoch']} != 0: odometry continuity lost since the anchor (see events.log); re-verify visually (reference image/depth) or recalibrate")
Oref = np.array(A["odom_pose_matrix"]); Onow = np.eye(4); Onow[:3, :3] = q2R(L["q_xyzw"]); Onow[:3, 3] = L["p"]
Tbl = np.array(A["T_base_link"]) @ np.linalg.inv(Oref) @ Onow; Tbo = Tbl @ np.array(A["T_link_optical"])
D0 = np.linalg.inv(np.array(A["T_base_optical"])) @ Tbo
print("camera moved since the anchor by %.1f mm / %.2f deg (odometry-estimated)" % (np.linalg.norm(D0[:3, 3]) * 1000, math.degrees(math.acos(max(-1, min(1, (np.trace(D0[:3, :3]) - 1) / 2))))))
print("T_base_optical_now =", json.dumps(np.round(Tbo, 6).tolist()))
