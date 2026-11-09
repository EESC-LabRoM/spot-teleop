"""Record nvblox's mesh topic into one file, as RViz shows it: nvblox publishes only the mesh blocks that changed, so keep
the latest version of every block (an empty block = removed; clear = start over) and when each block was first meshed
(sim time, from the message stamp). Writes <out> (npz: vertices, colors, triangles, per-triangle first-seen time, and
the triangle count over time) every --save-every s and at exit (SIGINT/SIGTERM).
    python3 mesh_recorder.py --topic /nvblox_node/mesh --out /ws/nvblox/out/<run>/mesh_2cm.npz
"""
import argparse, signal, sys, time
import numpy as np
import rclpy
from nvblox_msgs.msg import Mesh
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

ap = argparse.ArgumentParser()
ap.add_argument("--topic", default="/nvblox_node/mesh")
ap.add_argument("--out", required=True)
ap.add_argument("--save-every", type=float, default=20.0)
a = ap.parse_args(rclpy.utilities.remove_ros_args(sys.argv)[1:])


class Rec(Node):
    def __init__(self):
        super().__init__("mesh_recorder_" + a.topic.strip("/").replace("/", "_"))
        self.blocks, self.first, self.hist, self.n_msgs, self.frame = {}, {}, [], 0, ""
        for rel in (ReliabilityPolicy.RELIABLE, ReliabilityPolicy.BEST_EFFORT):
            self.create_subscription(Mesh, a.topic, self.cb, QoSProfile(depth=20, reliability=rel, durability=DurabilityPolicy.VOLATILE))
        self.last_save = time.time()

    def cb(self, m):
        t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9; self.frame = m.header.frame_id; self.n_msgs += 1
        if m.clear: self.blocks.clear()
        for idx, b in zip(m.block_indices, m.blocks):
            k = (idx.x, idx.y, idx.z)
            if not len(b.triangles):
                self.blocks.pop(k, None); continue
            V = np.array([(p.x, p.y, p.z) for p in b.vertices], np.float32)
            C = (np.clip(np.array([(c.r, c.g, c.b) for c in b.colors], np.float32), 0, 1) * 255).astype(np.uint8) \
                if len(b.colors) == len(b.vertices) else np.full((len(V), 3), 180, np.uint8)
            self.blocks[k] = (V, C, np.array(b.triangles, np.int32).reshape(-1, 3))
            self.first.setdefault(k, t)
        self.hist.append((t, sum(len(v[2]) for v in self.blocks.values())))
        if time.time() - self.last_save > a.save_every: self.save(); self.last_save = time.time()

    def save(self):
        if not self.blocks: return
        Vs, Cs, Ts, Fs, off = [], [], [], [], 0
        for k, (V, C, T) in self.blocks.items():
            Vs.append(V); Cs.append(C); Ts.append(T + off); Fs.append(np.full(len(T), self.first[k], np.float64)); off += len(V)
        np.savez_compressed(a.out, vertices=np.concatenate(Vs), colors=np.concatenate(Cs), triangles=np.concatenate(Ts),
                            tri_first_seen=np.concatenate(Fs), history=np.array(self.hist), frame=self.frame,
                            block_size=0.0, messages=self.n_msgs)
        self.get_logger().info(f"saved {a.out}: {len(self.blocks)} blocks, {off} vertices, {sum(len(t) for t in Ts)} triangles")


rclpy.init(); n = Rec()
def stop(*_):
    n.save(); sys.exit(0)
signal.signal(signal.SIGTERM, stop); signal.signal(signal.SIGINT, stop)
rclpy.spin(n)
