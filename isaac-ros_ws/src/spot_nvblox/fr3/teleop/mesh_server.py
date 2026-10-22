"""FR3 lab dashboard server (one page: cameras, segmentation, interactive nvblox map, live robot, teleop speed, gripper force).
Interactive nvblox map for the browser: keeps the incremental /<node>/mesh blocks (vertices, vertex colours, triangles; an empty
block = cleared, clear=True = reset) and serves them to map_viewer.html (three.js, orbit/zoom/pan) together with the live FR3, whose
link poses and meshes are passed through from the FR3 dashboard on 127.0.0.1:8088 (/config, /stream, /mesh/...).
  GET /mesh.bin?v=N  -> 204 if the map version is still N, else little-endian: uint32 version, n_vertices, n_triangles,
                        float32 xyz[n_v*3], uint8 rgba[n_v*4], uint32 indices[n_t*3]   (frame fr3_link0)
  POST /speed, /grip, /padtest -> passed to the FR3 dashboard (same guards: JSON + X-Dashboard header; that server clamps and the
                        teleop validates). GET / = lab_dashboard.html (everything), /map = map_viewer.html (map + robot only).
  GET /snap?topic=... -> one camera JPEG from web_video_server (:8787) /snapshot, for the https page (it cannot load http://...:8787
                        images). The page asks for the next frame only when one has arrived: MJPEG streams fill the ssh tunnel over
                        the DERP relay and the remote pad's WebSocket then waits behind them (measured 10-20: rtt 25 ms -> seconds).
  Remote PS4 pad (browser Gamepad API needs https): https://nexus.tail0994ab.ts.net:8443 -> ssh tunnel -> 127.0.0.1:8788 here.
  /ws, /padcfg (= its /config) and /fr3/... are passed raw to the remote-pad dashboard on 127.0.0.1:8089, which keeps all the
  interlocks (trusted loopback peer, Tailscale-User-Login allow-list, same origin, one client). ONLY on the 127.0.0.1 listener:
  through the 100.88.x listener anyone on the tailnet would reach :8089 as a loopback peer with a forged user header.
    python3 mesh_server.py [--node nvblox_human] [--port 8788,8795] [--bind 127.0.0.1,100.88.134.127]   (isaac-build, ROS_DOMAIN_ID=8)"""
import argparse
import os
import signal
import socket
import struct
import threading
import time
import urllib.error
import urllib.request
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import parse_qs, urlparse

import numpy as np
import rclpy
from nvblox_msgs.msg import Mesh
from rclpy.node import Node

ap = argparse.ArgumentParser()
ap.add_argument("--node", default="nvblox_human")
ap.add_argument("--port", default="8788,8795", help="comma-separated ports")
ap.add_argument("--bind", default="127.0.0.1,100.88.134.127")
ap.add_argument("--robot-dash", default="http://127.0.0.1:8088", help="FR3 dashboard to take the robot pose stream and meshes from")
ap.add_argument("--pad-port", type=int, default=8089, help="remote-pad FR3 dashboard on 127.0.0.1 (/ws, /padcfg, /fr3/...)")
ap.add_argument("--cam-port", type=int, default=8787, help="web_video_server on 127.0.0.1 (/snap)")
a = ap.parse_args()
HERE = os.path.dirname(os.path.abspath(__file__))


class MeshStore(Node):
    def __init__(self):
        super().__init__("mesh_server")
        self.blocks, self.lock, self.version, self.t_update = {}, threading.Lock(), 0, 0.0
        self.cache = (-1, b"")
        self.create_subscription(Mesh, f"/{a.node}/mesh", self.on_mesh, 10)

    def on_mesh(self, m):
        new = {}
        for idx, b in zip(m.block_indices, m.blocks):
            k = (idx.x, idx.y, idx.z)
            if not b.vertices or not b.triangles: new[k] = None; continue
            v = np.array([(p.x, p.y, p.z) for p in b.vertices], np.float32)
            if len(b.colors) == len(v):
                c = (np.clip(np.array([(q.r, q.g, q.b, q.a) for q in b.colors], np.float32), 0, 1) * 255).astype(np.uint8)
            else:
                c = np.full((len(v), 4), 180, np.uint8)
            new[k] = (v, c, np.asarray(b.triangles, np.uint32))
        with self.lock:
            if m.clear: self.blocks.clear()
            for k, val in new.items():
                if val is None: self.blocks.pop(k, None)
                else: self.blocks[k] = val
            self.version += 1; self.t_update = time.time()

    def blob(self):
        with self.lock:
            if self.cache[0] == self.version: return self.version, self.cache[1]
            ver, vals = self.version, list(self.blocks.values())
        if vals:
            off = np.cumsum([0] + [len(v[0]) for v in vals[:-1]]).astype(np.uint32)
            V = np.concatenate([v[0] for v in vals]); C = np.concatenate([v[1] for v in vals])
            T = np.concatenate([v[2] + o for v, o in zip(vals, off)])
        else:
            V, C, T = np.zeros((0, 3), np.float32), np.zeros((0, 4), np.uint8), np.zeros(0, np.uint32)
        data = struct.pack("<3I", ver, len(V), len(T) // 3) + V.astype("<f4").tobytes() + C.tobytes() + T.astype("<u4").tobytes()
        with self.lock: self.cache = (ver, data)
        return ver, data


rclpy.init()
signal.signal(signal.SIGTERM, lambda *_: os._exit(0))         # rclpy's own handler only stops ROS; the HTTP threads would keep the ports
store = MeshStore()
threading.Thread(target=rclpy.spin, args=(store,), daemon=True).start()


class H(BaseHTTPRequestHandler):
    def log_message(self, *args): pass

    def send(self, code, body=b"", ctype="text/plain", extra=()):
        self.send_response(code); self.send_header("Content-Type", ctype); self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-cache")
        for k, v in extra: self.send_header(k, v)
        self.end_headers(); self.wfile.write(body)

    def local(self):                                                     # came in on the loopback listener (= the ssh tunnel)
        return self.server.server_address[0] == "127.0.0.1"

    def pipe(self, port, path):
        """Raw passthrough to 127.0.0.1:<port> (WebSocket upgrade, MJPEG, SSE, plain): the request as received, then bytes both ways."""
        ws = self.headers.get("Upgrade", "").lower() == "websocket"
        try: up = socket.create_connection(("127.0.0.1", port), timeout=5)
        except OSError as e: return self.send(502, str(e).encode())
        head = f"{self.command} {path} HTTP/1.1\r\n" + "".join(f"{k}: {v}\r\n" for k, v in self.headers.items()
                                                             if ws or k.lower() not in ("connection", "keep-alive"))
        n = min(int(self.headers.get("Content-Length") or 0), 1 << 16)
        up.sendall((head + ("" if ws else "Connection: close\r\n") + "\r\n").encode() + (self.rfile.read(n) if n else b""))
        up.settimeout(None); self.close_connection = True

        def client_to_up():
            try:
                while True:
                    b = self.rfile.read1(1 << 16)
                    if not b: break
                    up.sendall(b)
            except OSError: pass
            finally:
                try: up.shutdown(socket.SHUT_WR)
                except OSError: pass
        if ws: threading.Thread(target=client_to_up, daemon=True).start()
        try:
            while True:
                b = up.recv(1 << 16)
                if not b: break
                self.wfile.write(b)
        except OSError: pass
        finally: up.close()

    def pad_route(self, path):                                           # /ws, /padcfg, /fr3/... -> the remote-pad dashboard, tunnel only
        if not self.local(): return self.send(403, b"remote pad only through https://nexus.tail0994ab.ts.net:8443")
        return self.pipe(a.pad_port, {"/ws": "/ws", "/padcfg": "/config"}.get(path) or self.path[len("/fr3"):])

    def do_GET(self):
        u = urlparse(self.path); path = u.path
        if path == "/snap": return self.pipe(a.cam_port, "/snapshot?" + u.query)
        if path in ("/ws", "/padcfg") or path.startswith("/fr3/"): return self.pad_route(path)
        if path in ("/", "/index.html"):
            return self.send(200, open(os.path.join(HERE, "lab_dashboard.html"), "rb").read(), "text/html; charset=utf-8")
        if path == "/map":
            return self.send(200, open(os.path.join(HERE, "map_viewer.html"), "rb").read(), "text/html; charset=utf-8")
        if path == "/mesh.bin":
            have = parse_qs(u.query).get("v", ["-1"])[0]
            ver, data = store.blob()
            if str(ver) == have: return self.send(204)
            return self.send(200, data, "application/octet-stream", [("X-Map-Age", f"{time.time() - store.t_update:.1f}")])
        if path == "/config" or path.startswith("/mesh/"):            # robot model from the FR3 dashboard
            try:
                with urllib.request.urlopen(a.robot_dash + path, timeout=5) as r:
                    return self.send(200, r.read(), r.headers.get("Content-Type", "application/octet-stream"))
            except Exception as e:
                return self.send(502, str(e).encode())
        if path == "/stream":                                           # robot link poses (server-sent events), passed through
            try:
                r = urllib.request.urlopen(a.robot_dash + "/stream", timeout=10)
            except Exception as e:
                return self.send(502, str(e).encode())
            self.send_response(200); self.send_header("Content-Type", "text/event-stream"); self.send_header("Cache-Control", "no-cache")
            self.end_headers(); last = 0.0
            try:
                for line in r:
                    if line.startswith(b"data:"):
                        now = time.time()
                        if now - last < 0.04: continue                  # 25 Hz is plenty for the viewer
                        last = now
                    self.wfile.write(line); self.wfile.flush()
            except (BrokenPipeError, ConnectionResetError, OSError):
                pass
            finally:
                r.close()
            return
        self.send(404, b"not found")

    def do_POST(self):
        path = urlparse(self.path).path
        if path.startswith("/fr3/"): return self.pad_route(path)
        if path == "/zed-control":
            if not self.local() and self.client_address[0] != self.server.server_address[0]:
                return self.send(403, b"Use local dashboard or authenticated HTTPS dashboard")
            return self.pipe(a.pad_port, path)
        if path not in ("/speed", "/grip", "/padtest", "/input-source") or self.headers.get("X-Dashboard") != "1" \
                or "json" not in (self.headers.get("Content-Type") or ""):
            return self.send(403, b"forbidden")
        if path == "/input-source":
            origin = self.headers.get("Origin")
            if origin and urlparse(origin).netloc != self.headers.get("Host"):
                return self.send(403, b"forbidden")
        body = self.rfile.read(min(int(self.headers.get("Content-Length") or 0), 200))
        req = urllib.request.Request(a.robot_dash + path, data=body, method="POST",
                                     headers={"Content-Type": "application/json", "X-Dashboard": "1"})
        try:
            with urllib.request.urlopen(req, timeout=5) as r: code, out = r.status, r.read()
        except urllib.error.HTTPError as e: code, out = e.code, e.read()
        except Exception as e: code, out = 502, str(e).encode()
        self.send(code, out, "application/json")


servers = [ThreadingHTTPServer((h, int(port)), H) for h in a.bind.split(",") for port in a.port.split(",")]
for s in servers[1:]: threading.Thread(target=s.serve_forever, daemon=True).start()
print(f"map viewer on {a.bind}:{a.port}  (mesh /{a.node}/mesh, robot from {a.robot_dash})", flush=True)
servers[0].serve_forever()
