"""VM half of the isolated TCP ESDF transport probe (TEST ONLY, not deployed).

Listens on TCP; for each framed request queries the LIVE domain-8 nvblox
service (client only -- a second client duplicates nothing) and returns the
framed serialized response. No cache, strict upstream deadline, fail-closed.

Framing: u64be length N, then N bytes = u64be generation_id + message bytes.
Empty message bytes after the generation id = upstream failure signal.
"""
import argparse
import socket
import struct
import threading
import time

import rclpy
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.serialization import serialize_message, deserialize_message
from nvblox_msgs.srv import EsdfAndGradients


def recv_exact(conn, n):
    buf = bytearray()
    while len(buf) < n:
        chunk = conn.recv(n - len(buf))
        if not chunk:
            raise ConnectionError("peer closed")
        buf += chunk
    return bytes(buf)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--tcp-host", default="127.0.0.1")
    ap.add_argument("--tcp-port", type=int, default=47861)
    ap.add_argument("--domain", type=int, default=8)
    ap.add_argument("--service", default="/nvblox_curobo/get_esdf_and_gradient")
    ap.add_argument("--upstream-timeout", type=float, default=0.5)
    a = ap.parse_args()

    ctx = Context()
    rclpy.init(context=ctx, domain_id=a.domain)
    node = rclpy.create_node("tcp_esdf_vm_half", context=ctx)
    cli = node.create_client(EsdfAndGradients, a.service)
    ex = SingleThreadedExecutor(context=ctx)
    ex.add_node(node)
    threading.Thread(target=ex.spin, daemon=True).start()

    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind((a.tcp_host, a.tcp_port))
    srv.listen(1)
    print(f"[vm] listening {a.tcp_host}:{a.tcp_port} -> domain{a.domain}{a.service}",
          flush=True)
    conn, addr = srv.accept()
    print(f"[vm] peer {addr}", flush=True)
    try:
        while True:
            hdr = recv_exact(conn, 8)
            (n,) = struct.unpack(">Q", hdr)
            if n > 64 * 1024 * 1024 or n < 8:
                raise ValueError(f"bad frame len {n}")
            payload = recv_exact(conn, n)
            gen = struct.unpack(">Q", payload[:8])[0]
            t0 = time.monotonic()
            resp_bytes = b""
            try:
                req = deserialize_message(payload[8:], EsdfAndGradients.Request)
                if cli.service_is_ready():
                    ev = threading.Event()
                    box = {}
                    fut = cli.call_async(req)
                    fut.add_done_callback(
                        lambda f: (box.setdefault("r", f.result())
                                   if f.done() and not f.cancelled()
                                   and f.result() is not None else None,
                                   ev.set()))
                    if ev.wait(a.upstream_timeout) and box.get("r") is not None:
                        resp_bytes = serialize_message(box["r"])
            except Exception as e:
                print(f"[vm] gen={gen} upstream error: {e}", flush=True)
            out = struct.pack(">Q", gen) + resp_bytes
            conn.sendall(struct.pack(">Q", len(out)) + out)
            dt = (time.monotonic() - t0) * 1000
            print(f"[vm] gen={gen} upstream+serialize {dt:.0f}ms "
                  f"resp_bytes={len(resp_bytes)}", flush=True)
    except (ConnectionError, ValueError) as e:
        print(f"[vm] connection ended: {e}", flush=True)
    finally:
        conn.close()
        srv.close()


if __name__ == "__main__":
    main()
