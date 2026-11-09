"""Carve check: query ESDF around the drill, report free/occupied/unknown stats.

Usage: python3 carve_check.py [cx cy cz half_xy half_z]
Defaults center on the drill pick XY with a body-sized box.
Exit 0 prints stats; a carved drill shows ~0 occupied in the drill band.
"""
import sys
import time

import numpy as np
import rclpy
from rclpy.node import Node
from nvblox_msgs.srv import EsdfAndGradients
from geometry_msgs.msg import Point, Vector3

cx, cy, cz = float(sys.argv[1]) if len(sys.argv) > 1 else 0.556, \
    float(sys.argv[2]) if len(sys.argv) > 2 else -0.228, \
    float(sys.argv[3]) if len(sys.argv) > 3 else 0.215
hx = float(sys.argv[4]) if len(sys.argv) > 4 else 0.055
hz = float(sys.argv[5]) if len(sys.argv) > 5 else 0.065

rclpy.init()
node = Node('carve_check')
cli = node.create_client(EsdfAndGradients, '/nvblox_curobo/get_esdf_and_gradient')
if not cli.wait_for_service(timeout_sec=15.0):
    print('NO SERVICE')
    raise SystemExit(1)
req = EsdfAndGradients.Request()
req.update_esdf = True
req.use_aabb = True
req.frame_id = 'fr3_link0'
req.aabb_min_m = Point(x=cx - hx, y=cy - hx, z=cz - hz)
req.aabb_size_m = Vector3(x=2 * hx, y=2 * hx, z=2 * hz)
t0 = time.time()
fut = cli.call_async(req)
rclpy.spin_until_future_complete(node, fut, timeout_sec=30.0)
dt = time.time() - t0
res = fut.result()
if res is None or not res.success:
    print('CALL FAILED')
    raise SystemExit(1)
dims = {d.label: d.size for d in res.esdf_and_gradients.layout.dim}
E = np.array(res.esdf_and_gradients.data, np.float32).reshape(
    dims['x'], dims['y'], dims['z'])
O = np.array([res.origin_m.x, res.origin_m.y, res.origin_m.z])
vx = float(res.voxel_size_m)
known = E > -999.0
occ = known & (E <= 0.0)
free = known & (E > 0.0)
print(f'service: {dt * 1000:.0f} ms, dims {dims}, voxel {vx * 1000:.0f} mm')
print(f'cells: total {E.size}, known {known.sum()} ({100 * known.mean():.1f}%), '
      f'occupied {occ.sum()}, free {free.sum()}')
if known.sum():
    print(f'min dist (known): {E[known].min():.3f} m, '
          f'max: {E[known].max():.3f} m')
# value at box center
ix = tuple(int(round((c - o) / vx)) for c, o in zip((cx, cy, cz), O))
ix = tuple(min(max(i, 0), d - 1) for i, d in zip(ix, E.shape))
print(f'center cell {ix}: {E[ix]:.3f} m')
node.destroy_node()
rclpy.shutdown()
