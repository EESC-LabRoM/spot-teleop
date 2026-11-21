#!/usr/bin/env python3
"""FR3 Lab Live dashboard backend on :8899 (2026-11-20, RAL lane).

Replaces the Nov-16 frozen preview (bench-max/wdel z2corr_preview, which served
frozen mesh/drill snapshots and 404'd graspjson/snap/speed): same page bytes
(with the run tabs), every route live.
  Page/static:    / /lab_dashboard.html (snapshotted page, see README), /health
  Live proxies:   /mesh.bin /config /mesh/* /drilljson /targetjson /graspjson -> tower :8788
                  /livecloudjson -> VM :8791/livecloud.json
                  /snap /stream -> tower :8787 snap_server via the 127.0.0.1:18787 ssh
                  tunnel (snap is mapped to the tower /snapshot route; no VM-local
                  snap server exists — 2026-11-20 fix, VM :8787 was a phantom).
  Read-through:   /correction.json (seg+tag study file, owner's bytes)
  Commands:       POST /speed {scale?, rot_scale?} 0..1 -> ROS params on /fr3_joy_teleop (tower ssh)
                  POST /grip {force} 1..70 N -> grasp_force param (next close only)
  Refused by design (501 + reason): POST /zed-control (executor switch + wrist input:
                  motion-path changes, need the motion go + design, not a dashboard click).
Nothing here moves the arm: speed/grip params only reshape future stick deflections
(and do nothing while disengaged). Run: setsid -f python3 lab_live_server.py >>logs/lab_live_8899.log 2>&1
"""
import json
import os
import subprocess
import time
import urllib.request
import urllib.error
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import urlsplit

HERE = Path(__file__).parent
PAGE = (HERE / "lab_dashboard.html").read_bytes()
CORR_SRC = Path("/home/nexus/bench-max/wdel/spot-teleop/z2corr_preview/correction.json")
TOWER = "http://100.88.134.127:8788"
SNAP = "http://127.0.0.1:18787"  # tower :8787 via ssh tunnel (no VM-local snap server)
VIEWER = "http://127.0.0.1:8791"
LABKEY = os.path.expanduser("~/.ssh/id_ed25519_lab_claude")
LABHOST = "murilo@100.88.134.127"
TIMEOUT = 8

# Manual regrasp trigger: runs ONLY planning stages (cloud 3 + grasp 4, DRY) via
# run_loop.sh in a background thread; the panel picks the new run up automatically
# (_latest_run). Single-flight. No motion possible: carve/request/execute stages
# are never invoked here. Seed verified 2026-11-20 on the live drill; the drill
# freshness guard refuses when the live centroid moved >15 cm since.
REGRASP = {"job": None}
REGRASP_SEED = {"bbox": [462, 717, 550, 911], "seed_3d": [0.038, 0.224, 0.671]}
REGRASP_SEED_CENTER = [0.4804, -0.1678, 0.1324]
GRASP_LOOP = HERE.parent / "grasp_loop"


def _regrasp_run(job):
    try:
        p = subprocess.Popen(
            ["./run_loop.sh", "--seed", "462,717,550,911", "--label", "drill",
             "--from", "3", "--to", "4", "--run-dir", job["dir"]],
            cwd=str(GRASP_LOOP), stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
        try:
            out, _ = p.communicate(timeout=720)
        except subprocess.TimeoutExpired:
            p.kill()
            job.update(state="failed", error="timeout 12 min")
            return
        job["log"] = out.splitlines()[-25:]
        pk = Path(job["dir"]) / "pick.json"
        if p.returncode == 0 and pk.exists():
            pick = json.loads(pk.read_text())
            job.update(state="done", pick={k: pick.get(k) for k in ("pos", "conf", "score")})
            print(f"{time.strftime('%T')} regrasp done {job['run_id']} pick={job['pick']}", flush=True)
        else:
            job.update(state="failed", error=f"exit={p.returncode}")
    except Exception as e:  # noqa: BLE001 (job status, never a stack trace)
        job.update(state="failed", error=str(e)[:200])


def fetch(url, timeout=TIMEOUT):
    with urllib.request.urlopen(url, timeout=timeout) as r:
        return r.status, r.headers.get_content_type(), r.read()


def tower_param_set(node, name, value):
    """Set one ROS param on the tower driver container (read back to confirm)."""
    inner = ("docker exec fr3-ros2 bash -c 'source /opt/ros/jazzy/setup.bash; "
             "source /ros2_ws/install/setup.bash 2>/dev/null; export ROS_DOMAIN_ID=77; "
             f"ros2 param set {node} {name} {value}' 2>&1")  # no pipe: it would mask ros2's exit code
    p = subprocess.run(["ssh", "-n", "-o", "BatchMode=yes", "-o", "ConnectTimeout=8",
                        "-i", LABKEY, LABHOST, inner], capture_output=True, text=True, timeout=25)
    return p.returncode == 0, (p.stdout + p.stderr)[-300:]


class H(BaseHTTPRequestHandler):
    server_version = "lab-live/1.0"

    def log_message(self, *a):
        pass

    def _send(self, code, ctype, body):
        if isinstance(body, str):
            body = body.encode()
        self.send_response(code)
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-cache")
        self.end_headers()
        self.wfile.write(body)

    def _proxy(self, base, prefix=""):
        u = urlsplit(self.path)
        url = base + prefix + u.path + (("?" + u.query) if u.query else "")
        try:
            code, ctype, body = fetch(url)
        except Exception as e:  # noqa: BLE001 (upstream text into a 502, never a stack trace)
            return self._send(502, "text/plain", f"upstream {base} failed: {e}")
        return self._send(code, ctype or "application/octet-stream", body)

    def _object_target_proxy(self, method):
        # Fixed loopback upstream; never proxy arbitrary paths, hosts or methods.
        # Until the coordinator runs, return unavailable, never fake readiness.
        if method == "POST":
            if self.headers.get("X-Dashboard") != "1":
                return self._send(403, "application/json", b'{"ok":false,"error":"dashboard header required"}')
            try:
                length = int(self.headers.get("Content-Length", "0"))
                if not 0 < length <= 1024:
                    raise ValueError("invalid content length")
                obj = json.loads(self.rfile.read(length))
                if not isinstance(obj, dict) or set(obj) != {"target"} or obj["target"] not in ("drill", "tennis_ball", "sprite"):
                    raise ValueError("invalid target")
                payload = json.dumps(obj).encode()
            except (ValueError, TypeError, json.JSONDecodeError) as e:
                return self._send(400, "application/json", json.dumps({"ok":False,"error":str(e)}))
        else:
            payload = None
        request = urllib.request.Request("http://127.0.0.1:8898/object-target", data=payload,
                                         headers={"Content-Type":"application/json", "X-Dashboard":"1"}, method=method)
        try:
            with urllib.request.urlopen(request, timeout=3) as upstream:
                body = upstream.read(16385)
                if len(body) > 16384:
                    raise ValueError("upstream response too large")
                return self._send(upstream.status, "application/json", body)
        except urllib.error.HTTPError as e:
            return self._send(e.code, "application/json", e.read(16384))
        except Exception:
            return self._send(503, "application/json", b'{"ok":false,"error":"object selection unavailable"}')

    def do_GET(self):  # noqa: C901 (route table)
        path = urlsplit(self.path).path
        if path in ("/", "/lab_dashboard.html"):
            return self._send(200, "text/html; charset=utf-8", PAGE)
        if path == "/health":
            return self._send(200, "application/json", json.dumps({
                "ok": True, "routes": ["page", "mesh.bin", "config", "mesh/*", "drilljson",
                                       "targetjson", "graspjson", "livecloudjson", "correction.json",
                                       "snap", "stream", "POST speed", "POST grip",
                                       "GET+POST regrasp"]}))
        if path in ("/mesh.bin", "/config", "/drilljson", "/targetjson", "/graspjson"):
            return self._proxy(TOWER)
        if path.startswith("/mesh/"):
            return self._proxy(TOWER)
        if path == "/livecloudjson":
            try:
                code, ctype, body = fetch(VIEWER + "/livecloud.json")
            except Exception as e:  # noqa: BLE001
                return self._send(502, "text/plain", f"viewer failed: {e}")
            return self._send(code, ctype, body)
        if path == "/correction.json":
            try:
                return self._send(200, "application/json", CORR_SRC.read_bytes())
            except OSError:
                return self._send(404, "application/json", b'{"error":"study file gone with its checkout"}')
        if path == "/object-target":
            return self._object_target_proxy("GET")
        if path == "/regrasp":
            return self._regrasp_status()
        if path == "/snap":
            # All camera snaps come from the tower snap_server via the 18787 tunnel.
            # Map snap->snapshot (the tower route name).
            u = urlsplit(self.path)
            try:
                code, ctype, body = fetch(SNAP + "/snapshot?" + u.query)
            except Exception as e:  # noqa: BLE001
                return self._send(502, "text/plain", f"tower snap failed: {e}")
            return self._send(code, ctype, body)
        if path == "/stream":
            u = urlsplit(self.path)
            url = SNAP + "/stream" + (("?" + u.query) if u.query else "")
            try:
                up = urllib.request.urlopen(url, timeout=TIMEOUT)
            except Exception as e:  # noqa: BLE001
                return self._send(502, "text/plain", f"snap server failed: {e}")
            self.send_response(200)
            self.send_header("Content-Type", up.headers.get_content_type() or "multipart/x-mixed-replace")
            self.send_header("Cache-Control", "no-cache")
            self.end_headers()
            try:
                while True:
                    chunk = up.read(65536)
                    if not chunk:
                        break
                    self.wfile.write(chunk)
            except (BrokenPipeError, ConnectionResetError):
                pass
            return
        return self._send(404, "text/plain", b"unknown route (see /health)")

    def _param_post(self, fields):
        try:
            body = json.loads(self.rfile.read(int(self.headers.get("Content-Length") or 0)) or b"{}")
        except (ValueError, OSError):
            return self._send(400, "application/json", b'{"ok":false,"error":"bad JSON"}')
        out, ok_all = {}, True
        for page_key, ros_name, lo, hi in fields:
            if page_key not in body:
                continue
            try:
                v = float(body[page_key])
            except (TypeError, ValueError):
                ok_all = False
                out[page_key] = "not a number"
                continue
            if not (lo <= v <= hi):
                ok_all = False
                out[page_key] = f"out of range {lo}..{hi}"
                continue
            ok, txt = tower_param_set("/fr3_joy_teleop", ros_name, v)
            ok_all = ok_all and ok
            out[page_key] = "set" if ok else f"ssh/param failed: {txt[-120:]}"
            print(f"{time.strftime('%T')} POST {self.path} {page_key}={v} -> {'ok' if ok else 'FAIL'}", flush=True)
        return self._send(200 if ok_all else 502, "application/json",
                           json.dumps({"ok": ok_all, "set": out}).encode())

    def _regrasp_status(self):
        job = REGRASP["job"]
        if job is None:
            return self._send(200, "application/json", b'{"state":"idle"}')
        return self._send(200, "application/json", json.dumps(job).encode())

    def _regrasp_trigger(self):
        import threading
        job = REGRASP["job"]
        if job is not None and job["state"] == "running":
            return self._send(409, "application/json", b'{"ok":false,"error":"a regrasp is already running"}')
        try:
            _, _, sbody = fetch(VIEWER + "/stats.json")
            tracked = json.loads(sbody).get("state") == "TRACKING"
        except Exception:  # noqa: BLE001
            tracked = False
        if not tracked:
            return self._send(409, "application/json", b'{"ok":false,"error":"tracker not TRACKING"}')
        try:
            _, _, dbody = fetch(VIEWER + "/drill.json")
            c = json.loads(dbody).get("centroid") or json.loads(dbody).get("center")
            import math
            moved = math.dist(c, REGRASP_SEED_CENTER)
        except Exception:  # noqa: BLE001
            return self._send(409, "application/json", b'{"ok":false,"error":"no live drill centroid"}')
        if moved > 0.15:
            return self._send(409, "application/json",
                               json.dumps({"ok": False, "error": f"drill moved {moved:.2f} m since seed; re-seed needed"}).encode())
        run_id = time.strftime("%Y%m%d_%H%M%S", time.gmtime()) + "_drill_manual"  # UTC like run_loop.sh (latest_run sorts by name)
        rundir = GRASP_LOOP / "runs" / run_id
        rundir.mkdir(parents=True)
        (rundir / "seed.json").write_text(json.dumps(dict(REGRASP_SEED, label="drill", state="TRACKING")))
        job = {"state": "running", "run_id": run_id, "dir": str(rundir),
               "started": time.strftime("%T"), "log": [], "pick": None}
        REGRASP["job"] = job
        threading.Thread(target=_regrasp_run, args=(job,), daemon=True).start()
        print(f"{time.strftime('%T')} regrasp triggered {run_id}", flush=True)
        return self._send(202, "application/json", json.dumps({"ok": True, "run_id": run_id}).encode())

    def do_POST(self):
        path = urlsplit(self.path).path
        if path == "/object-target":
            return self._object_target_proxy("POST")
        if path == "/regrasp":
            return self._regrasp_trigger()
        if path == "/speed":
            return self._param_post([("scale", "speed_scale", 0.0, 1.0), ("rot_scale", "rot_scale", 0.0, 1.0)])
        if path == "/grip":
            return self._param_post([("force", "grasp_force", 1.0, 70.0)])
        if path == "/zed-control":
            return self._send(501, "application/json", json.dumps({
                "ok": False, "message": "executor/input switching needs the motion go + design; refused by this server"
            }).encode())
        return self._send(404, "text/plain", b"unknown route (see /health)")


class B(BaseHTTPRequestHandler):
    """Read-only mirror of the cuRobo bridge status (franka_teleop lane's :8784) for the page pill."""

    server_version = "bridge-status-mirror/1.0"

    def log_message(self, *a):
        pass

    def do_GET(self):
        if urlsplit(self.path).path != "/status":
            self.send_response(404)
            self.end_headers()
            return
        try:
            code, _, body = fetch("http://100.88.134.127:8784/status")
        except Exception as e:  # noqa: BLE001
            body = json.dumps({"error": f"bridge unreachable: {e}"}).encode()
            self.send_response(502)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)
            return
        self.send_response(code)
        self.send_header("Content-Type", "application/json")
        self.send_header("Cache-Control", "no-cache")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)


if __name__ == "__main__":
    import sys
    import threading
    port = int(sys.argv[1]) if len(sys.argv) > 1 else 8899
    srv = ThreadingHTTPServer(("0.0.0.0", port), H)
    srv.daemon_threads = True
    bport = int(sys.argv[2]) if len(sys.argv) > 2 else 8784
    bsrv = ThreadingHTTPServer(("0.0.0.0", bport), B)
    bsrv.daemon_threads = True
    threading.Thread(target=bsrv.serve_forever, daemon=True).start()
    print(f"lab-live backend on :{port} (page + {len(PAGE)} page bytes) + bridge mirror :{bport}", flush=True)
    srv.serve_forever()
