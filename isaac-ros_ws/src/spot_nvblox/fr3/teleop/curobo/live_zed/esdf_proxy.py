"""Forward the node's ESDF service calls from the shadow domain (142) to the live nvblox_curobo in domain 8 (one rclpy process, two contexts).
Same request in, same response out; only what the MPC node itself would have sent (no clearing unless the node asks). 10-29 benchmark."""
import threading, time
import rclpy
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from nvblox_msgs.srv import EsdfAndGradients
SVC = "/nvblox_curobo/get_esdf_and_gradient"
ca, cb = Context(), Context()
rclpy.init(context=ca, domain_id=142); rclpy.init(context=cb, domain_id=8)
na = rclpy.create_node("claude_esdf_proxy_srv", context=ca); nb = rclpy.create_node("claude_esdf_proxy_cli", context=cb)
cli = nb.create_client(EsdfAndGradients, SVC)
eb = SingleThreadedExecutor(context=cb); eb.add_node(nb); threading.Thread(target=eb.spin, daemon=True).start()
n = {"calls": 0, "ok": 0, "fail": 0, "retried": 0}
RETRY_S = 0.4
def handler(req, res):
    t0 = time.time(); n["calls"] += 1
    if not cli.service_is_ready():
        n["fail"] += 1; print(f"[proxy] call {n['calls']}: live service not ready", flush=True); res.success = False; return res
    # the multi-MB reply crosses domain 8 over UDP; when a fragment is lost Fast DDS repairs it only at the next 3 s heartbeat,
    # which trips the bridge's 1.5 s ESDF freshness gate. Re-ask every RETRY_S and keep the first complete answer (read-only query).
    ev = threading.Event(); box = {}
    def done(fu):
        if not fu.exception() and fu.result() is not None and "r" not in box: box["r"] = fu.result(); ev.set()
    end = time.time() + 5.0; tries = 0
    while time.time() < end and not ev.is_set():
        cli.call_async(req).add_done_callback(done); tries += 1
        ev.wait(min(RETRY_S, max(0.0, end - time.time())))
    if tries > 1: n["retried"] += 1
    if box.get("r") is None:
        n["fail"] += 1; print(f"[proxy] call {n['calls']}: no answer / exception", flush=True); res.success = False; return res
    n["ok"] += 1
    if n["calls"] <= 3 or n["calls"] % 10 == 0:
        r = box["r"]; print(f"[proxy] call {n['calls']} ok={n['ok']} fail={n['fail']} retried={n['retried']}: {1000*(time.time()-t0):.0f} ms, success={r.success}, voxel={r.voxel_size_m}", flush=True)
    return box["r"]
na.create_service(EsdfAndGradients, SVC, handler)
print(f"[proxy] serving {SVC} in domain 142 -> live domain 8", flush=True)
ea = SingleThreadedExecutor(context=ca); ea.add_node(na); ea.spin()
