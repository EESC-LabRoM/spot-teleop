import sys, time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from nvblox_msgs.msg import Mesh

topic = sys.argv[1]
for rel in (ReliabilityPolicy.BEST_EFFORT, ReliabilityPolicy.RELIABLE):
    rclpy.init()
    node = Node('mesh_stat')
    qos = QoSProfile(depth=5, reliability=rel, history=HistoryPolicy.KEEP_LAST)
    st = {'n': 0, 'verts': 0, 'tris': 0}
    def cb(m):
        st['n'] += 1
        if 'fields' not in st:
            st['fields'] = [s for s in type(m).__slots__ if s != '_header']
            print('mesh fields:', st['fields'], flush=True)
            for s in st['fields']:
                v = getattr(m, s)
                try:
                    print(f'  {s}: len={len(v)}', flush=True)
                except TypeError:
                    print(f'  {s}: {v}', flush=True)
        st['verts'] = sum(len(getattr(m, s)) for s in st['fields']
                          if hasattr(getattr(m, s, None), '__len__'))
    node.create_subscription(Mesh, topic, cb, qos)
    t0 = time.time()
    while time.time() - t0 < 10 and st['n'] == 0:
        rclpy.spin_once(node, timeout_sec=0.2)
    if st['n'] == 0:
        node.destroy_node(); rclpy.shutdown()
        continue
    t0 = time.time(); n0 = st['n']
    while time.time() - t0 < 12:
        rclpy.spin_once(node, timeout_sec=0.2)
    print(f'{topic} [{rel.name}]: {(st["n"] - n0) / (time.time() - t0):.1f} Hz, '
          f'elems={st["verts"]}', flush=True)
    node.destroy_node(); rclpy.shutdown()
    break
else:
    print(f'{topic}: NO DATA')
