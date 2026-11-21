"""Read-only: average of 20 /joint_states samples (arm joints) as JSON."""
import json, time
import numpy as np, rclpy
from sensor_msgs.msg import JointState
rclpy.init(); n = rclpy.create_node('read_joints_readonly'); S = []
NAMES = [f'fr3_joint{i}' for i in range(1, 8)]
def cb(m):
    d = dict(zip(m.name, m.position))
    if all(k in d for k in NAMES): S.append([d[k] for k in NAMES])
n.create_subscription(JointState, '/joint_states', cb, 10)
t0 = time.time()
while len(S) < 20 and time.time() - t0 < 5: rclpy.spin_once(n, timeout_sec=0.1)
if len(S) < 5: raise SystemExit('no joint_states')
A = np.array(S); print(json.dumps(dict(q=A.mean(0).tolist(), spread=float(A.std(0).max()), n=len(S))))
