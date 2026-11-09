#!/usr/bin/env python3
"""2 Hz live feed for the real-drill web view (no viewer/loop changes).

Subscribes TF target_object + /grasp_target + /drill_cloud_state (domain 8, humble), fetches the loop
viewer's :8791/grasp.json cloud, writes state/live_real.json + state/live_real_cloud.json (atomic tmp+rename).
rclpy/tf2 imports live inside main() so the pure assemble() stays offline-testable (tests/test_live_feed.py).
Run in vm-viewer (repo mount + humble + host net):
  docker exec -d vm-viewer bash -c "source /opt/ros/humble/setup.bash; export ROS_DOMAIN_ID=8; \
    cd /home/nexus/spot-teleop/fr3_real_repro/regrasp && exec python3 real_tf_poll.py \
    >>state/live_tf_poll.log 2>&1"
"""
import argparse
import json
import math
import os
import time
import urllib.request

ST = os.path.join(os.path.dirname(os.path.abspath(__file__)), "state")
VIEWER = "http://127.0.0.1:8791/grasp.json"
MAX_PTS = 4000


def quat_to_R(q):
    """xyzw quaternion -> 3x3 row-major rotation matrix (pure python, no numpy)."""
    x, y, z, w = (float(v) for v in q)
    n = math.sqrt(x * x + y * y + z * z + w * w) or 1.0
    x, y, z, w = x / n, y / n, z / n, w / n
    return [[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]]


def r4(v, nd=4):
    return [round(float(a), nd) for a in v]


def assemble(tf_xyz, tf_wall, target, target_wall, cloud_payload, binding, loop_state, now):
    """Pure assembly -> (live_dict, cloud_points|None). All inputs may be None/stale; never raises."""
    live = {"src": "real", "t_wall": round(now, 1), "status": "ok"}
    live["tf"] = r4(tf_xyz) if tf_xyz is not None else None
    live["tf_age_s"] = round(now - tf_wall, 1) if tf_wall else None
    if target is not None:
        try:
            live["pick"] = {"pos": r4(target["pos"]),
                            "R": [[round(float(a), 4) for a in row] for row in quat_to_R(target["quat"])],
                            "conf": None}
        except (KeyError, TypeError, ValueError):
            live["pick"] = None
    else:
        live["pick"] = None
    live["target_age_s"] = round(now - target_wall, 1) if target_wall else None
    pts = None
    if isinstance(cloud_payload, dict) and isinstance(cloud_payload.get("cloud"), list) \
            and cloud_payload["cloud"]:
        raw = cloud_payload["cloud"]
        stride = max(1, len(raw) // MAX_PTS)
        pts = [[round(float(p[0]), 4), round(float(p[1]), 4), round(float(p[2]), 4)]
               for p in raw[::stride]]
        live["n_points"] = len(raw)
        live["cloud_run"] = str(cloud_payload.get("run", ""))
        lv = cloud_payload.get("live") or {}
        live["cloud_age_s"] = lv.get("cloud_age_s", lv.get("age_s"))
    else:
        live["n_points"] = 0
        live["cloud_run"] = ""
        live["cloud_age_s"] = None
    if isinstance(binding, dict):
        live["binding_run"] = str(binding.get("run_id", ""))
        live["binding_age_s"] = round(now - float(binding.get("bound_at", now)), 1) \
            if binding.get("bound_at") else None
    else:
        live["binding_run"], live["binding_age_s"] = "", None
    live["loop"] = loop_state if isinstance(loop_state, dict) else None
    if tf_xyz is None and target is None and pts is None:
        live["status"] = "empty"
    return live, pts


def atomic_json(path, obj):
    tmp = path + ".tmp"
    json.dump(obj, open(tmp, "w"))
    os.replace(tmp, path)


def read_json(path):
    try:
        with open(path) as f:
            return json.load(f)
    except (OSError, ValueError):
        return None


def main():
    import rclpy  # noqa: E402
    import tf2_ros  # noqa: E402
    from geometry_msgs.msg import PoseStamped  # noqa: E402
    from std_msgs.msg import String  # noqa: E402

    ap = argparse.ArgumentParser()
    ap.add_argument("--rate", type=float, default=2.0)
    ap.add_argument("--state-dir", default=ST)
    ap.add_argument("--viewer", default=VIEWER)
    a = ap.parse_args()
    os.makedirs(a.state_dir, exist_ok=True)

    rclpy.init()
    node = rclpy.create_node("real_tf_poll")
    buf = tf2_ros.Buffer()
    tf2_ros.TransformListener(buf, node)
    got = {"target": None, "target_wall": None, "loop": None}

    def on_target(m):
        got["target"] = {"pos": [m.pose.position.x, m.pose.position.y, m.pose.position.z],
                         "quat": [m.pose.orientation.x, m.pose.orientation.y, m.pose.orientation.z,
                                  m.pose.orientation.w]}
        got["target_wall"] = time.time()

    def on_loop(m):
        try:
            got["loop"] = json.loads(m.data)
        except ValueError:
            got["loop"] = {"raw": m.data[:200]}

    node.create_subscription(PoseStamped, "/grasp_target", on_target, 10)
    node.create_subscription(String, "/drill_cloud_state", on_loop, 10)

    def tick():
        now = time.time()
        tf_xyz, tf_wall = None, None
        try:
            t = buf.lookup_transform("fr3_link0", "target_object", rclpy.time.Time())
            p = t.transform.translation
            tf_xyz, tf_wall = [p.x, p.y, p.z], now
        except Exception:
            pass
        try:
            with urllib.request.urlopen(a.viewer, timeout=4) as r:
                cloud_payload = json.load(r)
        except Exception:
            cloud_payload = None
        live, pts = assemble(tf_xyz, tf_wall, got["target"], got["target_wall"], cloud_payload,
                             read_json(os.path.join(a.state_dir, "grasp_binding.json")), got["loop"], now)
        atomic_json(os.path.join(a.state_dir, "live_real.json"), live)
        atomic_json(os.path.join(a.state_dir, "live_real_cloud.json"), {"points": pts or []})
        if int(now * a.rate) % 30 == 0:
            node.get_logger().info(f'live feed: tf={live["tf"]} target_age={live["target_age_s"]} '
                                   f'cloud_n={live["n_points"]} run={live["cloud_run"]}')

    node.create_timer(1.0 / a.rate, tick)
    node.get_logger().info("real_tf_poll: TF + /grasp_target + viewer cloud -> state/live_real*.json")
    rclpy.spin(node)


if __name__ == "__main__":
    main()
