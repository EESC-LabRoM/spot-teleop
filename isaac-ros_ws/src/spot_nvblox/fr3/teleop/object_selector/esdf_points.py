"""Read-only: nvblox ESDF at given points (stdin JSON Nx3, fr3_link0). Prints JSON list (None = out of AABB)."""
import json, sys
import numpy as np
import rclpy
from geometry_msgs.msg import Point, Vector3
from nvblox_msgs.srv import EsdfAndGradients
P = np.asarray(json.load(sys.stdin), float)
lo, hi = P.min(0) - .06, P.max(0) + .06
rclpy.init(); node = rclpy.create_node('read_only_esdf_points')
cli = node.create_client(EsdfAndGradients, '/nvblox_curobo/get_esdf_and_gradient')
if not cli.wait_for_service(timeout_sec=8): raise SystemExit('ESDF service unavailable')
rq = EsdfAndGradients.Request(); rq.update_esdf = False; rq.visualize_esdf = False; rq.use_aabb = True
rq.frame_id = 'fr3_link0'; rq.aabb_min_m = Point(x=lo[0], y=lo[1], z=lo[2]); rq.aabb_size_m = Vector3(x=hi[0]-lo[0], y=hi[1]-lo[1], z=hi[2]-lo[2])
f = cli.call_async(rq); rclpy.spin_until_future_complete(node, f, timeout_sec=20)
r = f.result(); dims = r.esdf_and_gradients.layout.dim; vals = r.esdf_and_gradients.data
o = np.array([r.origin_m.x, r.origin_m.y, r.origin_m.z]); v = r.voxel_size_m
out = []
for p in P:
    idx = [int(round((p[i]-o[i])/v - .5)) for i in range(3)]
    if any(idx[i] < 0 or idx[i] >= dims[i].size for i in range(3)): out.append(None); continue
    out.append(float(vals[sum(idx[i]*(dims[i].stride//dims[i].size) for i in range(3))]))
print(json.dumps(dict(success=r.success, voxel=v, esdf=out)))
