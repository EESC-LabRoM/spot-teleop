"""one ESDF service call over the FR3 grid box, prints timing, shape and occupancy (debug helper)."""
import time, numpy as np, rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, Vector3
from nvblox_msgs.srv import EsdfAndGradients
rclpy.init(); n = Node("esdf_probe"); c = n.create_client(EsdfAndGradients, "/nvblox_node/get_esdf_and_gradient")
print("service ready:", c.wait_for_service(timeout_sec=5.0))
r = EsdfAndGradients.Request(); r.update_esdf, r.use_aabb, r.frame_id = True, True, "fr3_link0"
r.aabb_min_m = Point(x=-0.35, y=-0.8, z=-0.05); r.aabb_size_m = Vector3(x=1.4, y=1.6, z=1.2)
t0 = time.monotonic(); f = c.call_async(r); rclpy.spin_until_future_complete(n, f, timeout_sec=30.0)
res = f.result(); print(f"done={f.done()} after {time.monotonic() - t0:.1f} s")
if res is not None:
    d = np.asarray(res.esdf_and_gradients.data, np.float32)
    print("success", res.success, "voxel", res.voxel_size_m, "dims", [x.size for x in res.esdf_and_gradients.layout.dim], "n", d.size,
          "origin", round(res.origin_m.x, 3), round(res.origin_m.y, 3), round(res.origin_m.z, 3),
          "occupied", int(((d < 0) & (d > -999)).sum()), "unknown", int((d < -999).sum()))
