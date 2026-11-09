"""Lab half of the isolated TCP ESDF transport probe (TEST ONLY, not deployed).

Serves a TEST-ONLY service name in domain 142 (never the live proxy name, so
no live service duplication) and forwards each request over persistent TCP to
the VM half. Exact serialized bytes, generation ids, strict deadline, no
cache, fail-closed on overlap/timeout/mismatch.

Run on the LAB host (needs lab DDS domain 142 + route to the VM):
  python3 tcp_esdf_lab.py --vm-host 10.0.0.3 --tcp-port 47861
"""
import argparse
import socket
import struct
import threading
import time

import rclpy
from rclpy.context import Context
from rclpy.serialization import serialize_message, deserialize_message
from nvblox_msgs.srv import EsdfAndGradients

TEST_SERVICE = "/tcp_esdf_probe/get_esdf_and_gradient"


def recv_exact(conn, n):
    buf = bytearray()
    while len(buf) < n:
        chunk = conn.recv(n - len(buf))
        if not chunk:
            raise ConnectionError("peer closed")
        buf += chunk
    return bytes(buf)


class Forwarder:
    def __init__(self, vm_host, tcp_port, deadline):
        self.addr = (vm_host, tcp_port)
        self.deadline = deadline
        self.lock = threading.Lock()
        self.gen = 0
        self.conn = None

    def _dial(self):
        if self.conn is not None:
            return
        c = socket.create_connection(self.addr, timeout=self.deadline)
        c.settimeout(self.deadline)
        self.conn = c

    def _drop(self):
        try:
            if self.conn is not None:
                self.conn.close()
        finally:
            self.conn = None

    def query(self, req):
        # Single-flight: fail closed on overlap (one in-flight max).
        if not self.lock.acquire(blocking=False):
            return None
        try:
            self.gen += 1
            gen = self.gen
            payload = struct.pack(">Q", gen) + serialize_message(req)
            try:
                self._dial()
                self.conn.sendall(struct.pack(">Q", len(payload)) + payload)
                (n,) = struct.unpack(">Q", recv_exact(self.conn, 8))
                if n > 64 * 1024 * 1024 or n < 8:
                    raise ValueError(f"bad frame len {n}")
                frame = recv_exact(self.conn, n)
            except (OSError, ValueError, socket.timeout):
                self._drop()
                return None
            rgen = struct.unpack(">Q", frame[:8])[0]
            if rgen != gen or len(frame) == 8:
                return None  # generation mismatch or upstream failure signal
            try:
                return deserialize_message(frame[8:], EsdfAndGradients.Response)
            except Exception:
                return None
        finally:
            self.lock.release()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--vm-host", default="10.0.0.3")
    ap.add_argument("--tcp-port", type=int, default=47861)
    ap.add_argument("--domain", type=int, default=142)
    ap.add_argument("--deadline", type=float, default=0.7)
    a = ap.parse_args()

    ctx = Context()
    rclpy.init(context=ctx, domain_id=a.domain)
    node = rclpy.create_node("tcp_esdf_lab_half", context=ctx)
    fwd = Forwarder(a.vm_host, a.tcp_port, a.deadline)

    def handler(req, res):
        t0 = time.monotonic()
        out = fwd.query(req)
        dt = (time.monotonic() - t0) * 1000
        if out is None:
            print(f"[lab] FAIL {dt:.0f}ms", flush=True)
            res.success = False
            return res
        print(f"[lab] ok {dt:.0f}ms floats={len(out.esdf_and_gradients.data)}",
              flush=True)
        return out

    node.create_service(EsdfAndGradients, TEST_SERVICE, handler)
    print(f"[lab] serving TEST {TEST_SERVICE} in domain {a.domain} "
          f"-> {a.vm_host}:{a.tcp_port} (deadline {a.deadline}s, no cache)",
          flush=True)
    rclpy.spin(node, context=ctx)


if __name__ == "__main__":
    main()
