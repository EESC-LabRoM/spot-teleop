import time, numpy as np, rclpy
from rclpy.node import Node
from nvblox_msgs.srv import EsdfAndGradients
from geometry_msgs.msg import Point, Vector3
rclpy.init(); n = Node("claude_esdf_cost_probe")
def call(name, amin, size, reps=50):
    cli = n.create_client(EsdfAndGradients, name)
    if not cli.wait_for_service(timeout_sec=5): return f"{name}: service not available"
    lat = []; info = None
    for _ in range(reps):
        r = EsdfAndGradients.Request(); r.use_aabb = True; r.update_esdf = True; r.visualize_esdf = False; r.frame_id = "fr3_link0"
        r.aabb_min_m = Point(x=amin[0], y=amin[1], z=amin[2]); r.aabb_size_m = Vector3(x=size[0], y=size[1], z=size[2])
        t0 = time.time(); f = cli.call_async(r)
        while not f.done() and time.time() - t0 < 5: rclpy.spin_once(n, timeout_sec=0.001)
        if f.done() and f.result() is not None and f.result().success: lat.append((time.time() - t0) * 1000); res = f.result()
        time.sleep(0.15)
    if not lat: return f"{name}: no successful response"
    dims = [d.size for d in res.esdf_and_gradients.layout.dim] if hasattr(res, "esdf_and_gradients") else []
    nbytes = len(res.esdf_and_gradients.data) * 4 if hasattr(res, "esdf_and_gradients") else 0
    slow=[round(x) for x in lat if x>200]
    return f"{name}: voxel {res.voxel_size_m*100:.1f} cm, grid dims {dims}, payload {nbytes/1e6:.2f} MB; latency ms median {np.median(lat):.0f}, p90 {np.percentile(lat,90):.0f}, max {max(lat):.0f} (n={len(lat)}); calls slower than 200 ms: {len(slow)} -> {slow}"
box = ((-0.35, -0.80, -0.05), (1.40, 1.60, 1.20))        # the MPC's AABB in fr3_link0
for nm in ("nvblox_curobo","nvblox_test3"): print(call(f"/{nm}/get_esdf_and_gradient", *box))
