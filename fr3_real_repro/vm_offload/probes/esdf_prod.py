import sys, time
import rclpy
from rclpy.node import Node
from rclpy.serialization import serialize_message
from nvblox_msgs.srv import EsdfAndGradients
from geometry_msgs.msg import Point, Vector3

rclpy.init()
node = Node('esdf_probe')
cli = node.create_client(EsdfAndGradients,
                         '/nvblox_curobo/get_esdf_and_gradient')
if not cli.wait_for_service(timeout_sec=15.0):
    print('NO SERVICE')
    raise SystemExit(1)
req = EsdfAndGradients.Request()
req.update_esdf = True
req.visualize_esdf = False
req.use_aabb = True
req.frame_id = 'fr3_link0'
req.aabb_min_m = Point(x=0.2, y=-0.5, z=0.0)
req.aabb_size_m = Vector3(x=0.5, y=0.6, z=0.45)
for i in range(3):
    t0 = time.time()
    fut = cli.call_async(req)
    rclpy.spin_until_future_complete(node, fut, timeout_sec=30.0)
    dt = time.time() - t0
    if fut.result() is None:
        print(f'call {i}: TIMEOUT/FAIL')
    else:
        wire = len(serialize_message(fut.result()))
        print(f'call {i}: {dt*1000:.0f} ms, reply {wire/1e6:.2f} MB', flush=True)
node.destroy_node()
rclpy.shutdown()
