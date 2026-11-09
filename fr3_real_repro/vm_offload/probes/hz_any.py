import sys, time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image

topic = sys.argv[1]
for rel in (ReliabilityPolicy.RELIABLE, ReliabilityPolicy.BEST_EFFORT):
    rclpy.init()
    node = Node('hz_any')
    qos = QoSProfile(depth=10, reliability=rel, history=HistoryPolicy.KEEP_LAST)
    st = {'n': 0}
    node.create_subscription(Image, topic, lambda m: st.__setitem__('n', st['n'] + 1), qos)
    t0 = time.time()
    while time.time() - t0 < 8 and st['n'] == 0:
        rclpy.spin_once(node, timeout_sec=0.2)
    if st['n'] == 0:
        node.destroy_node(); rclpy.shutdown()
        continue
    t0 = time.time(); n0 = st['n']
    while time.time() - t0 < 10:
        rclpy.spin_once(node, timeout_sec=0.2)
    print(f'{topic} [{rel.name}]: {(st["n"] - n0) / (time.time() - t0):.1f} Hz', flush=True)
    node.destroy_node(); rclpy.shutdown()
    break
else:
    print(f'{topic}: NO DATA')
