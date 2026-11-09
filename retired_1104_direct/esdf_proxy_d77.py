"""Forward the node's ESDF service calls from the direct domain (77) to the
live nvblox_curobo in domain 8 (one rclpy process, two contexts).

v2 (2026-11-03): the v1 retry loop re-sent the same 1.4 MB request every 0.4 s,
so one slow upstream call snowballed into a self-DDoS that starved every later
call. v2 sends ONE upstream call per node call (8 s budget) and caches identical
requests for 1.5 s (the node polls the same AABB at 4 Hz; the static scene does
not change between polls). Same request in, same response out.
"""
import threading
import time

import rclpy
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor

from nvblox_msgs.srv import EsdfAndGradients

SVC = "/nvblox_curobo/get_esdf_and_gradient"
UPSTREAM_TIMEOUT_S = 8.0
CACHE_TTL_S = 1.5

ca, cb = Context(), Context()
rclpy.init(context=ca, domain_id=77)
rclpy.init(context=cb, domain_id=8)
na = rclpy.create_node("d77_esdf_proxy_srv", context=ca)
nb = rclpy.create_node("d77_esdf_proxy_cli", context=cb)
cli = nb.create_client(EsdfAndGradients, SVC)
eb = SingleThreadedExecutor(context=cb)
eb.add_node(nb)
threading.Thread(target=eb.spin, daemon=True).start()

n = {"calls": 0, "ok": 0, "fail": 0, "cache": 0}
cache = {}  # key -> (expires_at, response)


def key_of(req):
    m, s = req.aabb_min_m, req.aabb_size_m
    return (req.update_esdf, req.visualize_esdf, req.use_aabb, req.frame_id,
            round(m.x, 4), round(m.y, 4), round(m.z, 4),
            round(s.x, 4), round(s.y, 4), round(s.z, 4),
            len(req.aabbs_to_clear_min_m), len(req.spheres_to_clear_center_m))


def handler(req, res):
    t0 = time.time()
    n["calls"] += 1
    now = t0
    # Expire cheaply: drop stale entries when the cache grows.
    if len(cache) > 8:
        for k in [k for k, (e, _) in cache.items() if e < now]:
            del cache[k]
    k = key_of(req)
    hit = cache.get(k)
    if hit is not None and hit[0] >= now:
        n["cache"] += 1
        if n["calls"] % 50 == 0:
            print(f"[proxy] calls={n['calls']} ok={n['ok']} fail={n['fail']} "
                  f"cache={n['cache']}", flush=True)
        return hit[1]
    if not cli.service_is_ready():
        n["fail"] += 1
        if n["calls"] <= 20:
            print(f"[proxy] call {n['calls']}: live service not ready",
                  flush=True)
        res.success = False
        return res
    ev = threading.Event()
    box = {}

    def done(fu):
        try:
            if fu.exception() is None and fu.result() is not None:
                box["r"] = fu.result()
        finally:
            ev.set()

    cli.call_async(req).add_done_callback(done)
    ev.wait(UPSTREAM_TIMEOUT_S)
    if box.get("r") is None:
        n["fail"] += 1
        if n["calls"] <= 20 or n["fail"] % 25 == 1:
            print(f"[proxy] call {n['calls']}: upstream timeout "
                  f"({UPSTREAM_TIMEOUT_S:.0f}s)", flush=True)
        res.success = False
        return res
    n["ok"] += 1
    r = box["r"]
    cache[k] = (time.time() + CACHE_TTL_S, r)
    dt_ms = 1000 * (time.time() - t0)
    if n["calls"] <= 5 or n["calls"] % 20 == 0:
        print(f"[proxy] call {n['calls']} ok={n['ok']} fail={n['fail']} "
              f"cache={n['cache']}: {dt_ms:.0f} ms, success={r.success}, "
              f"voxel={r.voxel_size_m}", flush=True)
    return r


na.create_service(EsdfAndGradients, SVC, handler)
print(f"[proxy] v2 serving {SVC} in domain 77 -> live domain 8", flush=True)
ea = SingleThreadedExecutor(context=ca)
ea.add_node(na)
ea.spin()
