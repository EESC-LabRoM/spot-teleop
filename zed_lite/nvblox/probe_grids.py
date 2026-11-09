"""Which nvblox layer holds the target? Occupied cells of the static / dynamic / combined 2D occupancy-grid slices inside an xy box
(robot base frame), and the distance-slice values there. The slice is at esdf_slice_height above the base plane.
   python3 probe_grids.py --box x0 y0 x1 y1 [--node nvblox_node] [--wait 40]"""
import argparse, json, sys, time
import numpy as np, rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
ap = argparse.ArgumentParser(); ap.add_argument("--box", type=float, nargs=4, required=True); ap.add_argument("--node", default="nvblox_node"); ap.add_argument("--wait", type=float, default=40)
a = ap.parse_args(rclpy.utilities.remove_ros_args(sys.argv)[1:]); x0, y0, x1, y1 = a.box
rclpy.init(); n = Node("probe_grids"); got = {}
def cb(name):
    def f(m):
        g = np.array(m.data, np.int8).reshape(m.info.height, m.info.width); r = m.info.resolution; o = m.info.origin.position
        xs = o.x + (np.arange(m.info.width) + 0.5) * r; ys = o.y + (np.arange(m.info.height) + 0.5) * r
        inx = (xs > x0) & (xs < x1); iny = (ys > y0) & (ys < y1); sub = g[np.ix_(iny, inx)]
        got[name] = {"frame": m.header.frame_id, "res": round(r, 4), "grid": [m.info.width, m.info.height], "box_cells": int(sub.size), "occupied_in_box": int((sub > 50).sum()),
                     "free_in_box": int(((sub >= 0) & (sub <= 50)).sum()), "unknown_in_box": int((sub < 0).sum()), "occupied_total": int((g > 50).sum())}
    return f
for rel in (ReliabilityPolicy.RELIABLE, ReliabilityPolicy.BEST_EFFORT):
    q = QoSProfile(depth=1, reliability=rel, durability=DurabilityPolicy.VOLATILE)
    for nm in ("static_occupancy_grid", "dynamic_occupancy_grid", "combined_occupancy_grid"):
        n.create_subscription(OccupancyGrid, f"/{a.node}/{nm}", cb(nm), q)
t0 = time.time()
while time.time() - t0 < a.wait and len(got) < 3: rclpy.spin_once(n, timeout_sec=0.5)
print(json.dumps(got, indent=1))
