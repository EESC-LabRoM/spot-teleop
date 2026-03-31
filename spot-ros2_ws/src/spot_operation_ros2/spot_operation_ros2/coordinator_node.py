import json
import time
import uuid

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger


class CoordinatorNode(Node):
    """
    Lightweight state machine:
    - Reads tracking state.
    - Requests relocalization via VLM service when needed.
    - Publishes seed command for perception_rt_node.
    """

    def __init__(self):
        super().__init__("coordinator_node")
        self.declare_parameter("tracking_state_topic", "/tracking_state")
        self.declare_parameter("seed_command_topic", "/perception/seed_command")
        self.declare_parameter("vlm_service_name", "/roll/trigger_relocalize")
        self.declare_parameter("loop_hz", 2.0)
        self.declare_parameter("max_seed_age_sec", 10.0)
        self.declare_parameter("relocalize_cooldown_sec", 1.5)
        self.declare_parameter("seed_apply_grace_sec", 2.0)

        tracking_state_topic = self.get_parameter("tracking_state_topic").value
        seed_command_topic = self.get_parameter("seed_command_topic").value
        service_name = self.get_parameter("vlm_service_name").value
        loop_hz = float(max(0.5, self.get_parameter("loop_hz").value))
        self.max_seed_age_sec = float(max(1.0, self.get_parameter("max_seed_age_sec").value))
        self.relocalize_cooldown_sec = float(
            max(0.0, self.get_parameter("relocalize_cooldown_sec").value)
        )
        self.seed_apply_grace_sec = float(
            max(0.0, self.get_parameter("seed_apply_grace_sec").value)
        )
        self._use_sim_time_effective = bool(self.get_parameter("use_sim_time").value)

        self.state = "IDLE"
        self.tracking_state = "UNKNOWN"
        self._vlm_inflight = False
        self._last_relocalize_attempt = 0.0
        self._seed_apply_deadline = 0.0
        self._tracking_sub = self.create_subscription(
            String, tracking_state_topic, self._tracking_state_cb, 10
        )
        self._seed_pub = self.create_publisher(String, seed_command_topic, 10)
        self._vlm_client = self.create_client(Trigger, service_name)
        self._timer = self.create_timer(1.0 / loop_hz, self._tick)
        self._diag_timer = self.create_timer(2.0, self._diag_cb)
        self.get_logger().info(
            f"Coordinator started. tracking_state={tracking_state_topic}, seed_out={seed_command_topic}, vlm_service={service_name}"
        )
        self._dbg_log(
            "H12",
            "coordinator_node.py:__init__",
            "coordinator_init_clock_mode",
            {"use_sim_time": self._use_sim_time_effective},
        )
        self._dbg_tick_count = 0

    def _dbg_log(self, hypothesis_id: str, location: str, message: str, data: dict):
        # #region agent log
        payload = {
            "sessionId": "eb5d37",
            "id": f"log_{int(time.time() * 1000)}_{uuid.uuid4().hex[:8]}",
            "timestamp": int(time.time() * 1000),
            "runId": "repro-1",
            "hypothesisId": hypothesis_id,
            "location": location,
            "message": message,
            "data": data,
        }
        try:
            with open("/home/spot-teleop/spot-ros2_ws/.cursor/debug-eb5d37.log", "a", encoding="utf-8") as f:
                f.write(json.dumps(payload, ensure_ascii=True) + "\n")
        except Exception:
            pass
        # #endregion

    def _tracking_state_cb(self, msg: String):
        self.tracking_state = msg.data.strip().upper()
        if self.tracking_state == "TRACKING":
            # Tracker locked on target: clear grace window.
            self._seed_apply_deadline = 0.0
        # #region agent log
        self._dbg_log(
            "H9",
            "coordinator_node.py:_tracking_state_cb",
            "tracking_state_rx",
            {"tracking_state": self.tracking_state},
        )
        # #endregion

    def _diag_cb(self):
        # #region agent log
        try:
            seed_topic_types = []
            for tn, ttypes in self.get_topic_names_and_types():
                if tn == "/perception/seed_command":
                    seed_topic_types = list(ttypes)
                    break
            sub_infos = []
            try:
                for info in self.get_subscriptions_info_by_topic("/perception/seed_command"):
                    sub_infos.append(
                        {
                            "node_name": str(info.node_name),
                            "node_namespace": str(info.node_namespace),
                            "topic_type": str(info.topic_type),
                            "qos_reliability": int(info.qos_profile.reliability),
                            "qos_durability": int(info.qos_profile.durability),
                        }
                    )
            except Exception as exc_info:
                sub_infos = [{"error": str(exc_info)[:200]}]
            self._dbg_log(
                "H8",
                "coordinator_node.py:_diag_cb",
                "seed_topic_diagnostics",
                {
                    "seed_subscriber_count": int(self._seed_pub.get_subscription_count()),
                    "seed_topic_types": seed_topic_types,
                    "seed_sub_infos": sub_infos,
                    "state": self.state,
                },
            )
        except Exception as exc:
            self._dbg_log(
                "H8",
                "coordinator_node.py:_diag_cb",
                "seed_topic_diagnostics_failed",
                {"error": str(exc)[:200]},
            )
        # #endregion

    def _tick(self):
        now = time.time()
        self._dbg_tick_count += 1
        if self._dbg_tick_count <= 5:
            self._dbg_log(
                "H1",
                "coordinator_node.py:_tick",
                "tick_state_snapshot",
                {
                    "tracking_state": self.tracking_state,
                    "state": self.state,
                    "vlm_inflight": self._vlm_inflight,
                    "seconds_since_last_attempt": now - self._last_relocalize_attempt,
                },
            )
        if self.tracking_state == "TRACKING":
            self.state = "TRACKING"
            return
        if now < self._seed_apply_deadline:
            self.state = "SEEDING"
            self._dbg_log(
                "H16",
                "coordinator_node.py:_tick",
                "seed_apply_grace_active",
                {"remaining_sec": float(self._seed_apply_deadline - now)},
            )
            return
        # Trigger relocalization only when explicitly LOST.
        # UNKNOWN/IDLE/other states should not spam API calls.
        if self.tracking_state != "LOST":
            self.state = "IDLE"
            return
        if self._vlm_inflight:
            self.state = "RELOCALIZING"
            return
        if now - self._last_relocalize_attempt < self.relocalize_cooldown_sec:
            return
        if not self._vlm_client.wait_for_service(timeout_sec=0.0):
            self.state = "DEGRADED"
            return
        self._last_relocalize_attempt = now
        self._vlm_inflight = True
        self.state = "RELOCALIZING"
        self._dbg_log(
            "H16",
            "coordinator_node.py:_tick",
            "relocalize_request_dispatched",
            {"tracking_state": self.tracking_state, "state": self.state},
        )
        fut = self._vlm_client.call_async(Trigger.Request())
        fut.add_done_callback(self._handle_vlm_result)

    def _handle_vlm_result(self, future):
        self._vlm_inflight = False
        try:
            result = future.result()
        except Exception as exc:
            self.state = "DEGRADED"
            self.get_logger().warn(f"Relocalize call failed: {exc}")
            return
        if result is None or not result.success:
            # #region agent log
            self._dbg_log(
                "H10",
                "coordinator_node.py:_handle_vlm_result",
                "vlm_result_unsuccessful",
                {
                    "result_is_none": result is None,
                    "result_success": bool(result.success) if result is not None else False,
                    "result_message": (result.message[:240] if result is not None else ""),
                },
            )
            # #endregion
            self.state = "DEGRADED"
            return
        try:
            seed = json.loads(result.message)
            stamp_sec = float(seed.get("frame_stamp_sec", 0.0))
            stamp_ns = int(seed.get("frame_stamp_nanosec", 0))
            seed_time = stamp_sec + stamp_ns * 1e-9
            now_ros = self.get_clock().now().nanoseconds * 1e-9
            seed_age = now_ros - seed_time
            self._dbg_log(
                "H2",
                "coordinator_node.py:_handle_vlm_result",
                "seed_age_check",
                {
                    "now_ros": now_ros,
                    "seed_time": seed_time,
                    "seed_age": seed_age,
                    "max_seed_age_sec": self.max_seed_age_sec,
                    "seed_has_bbox": isinstance(seed.get("bbox_1000"), list),
                },
            )

            if seed_age < -1000.0 or seed_age > 100000.0:
                self.get_logger().warn(
                    f"Dominio de tempo incoerente: now_ros={now_ros:.1f}, seed={seed_time:.1f}. "
                    "Verifique use_sim_time no coordinator."
                )

            if seed_age > 100.0 and seed_age < 100000.0:
                self.state = "DEGRADED"
                self.get_logger().warn(f"Dropping stale seed (age={seed_age:.1f}s)")
                return
            msg = String()
            msg.data = json.dumps(seed, ensure_ascii=True)
            self._seed_pub.publish(msg)
            self._seed_apply_deadline = time.time() + self.seed_apply_grace_sec
            self._dbg_log(
                "H8",
                "coordinator_node.py:_handle_vlm_result",
                "seed_published",
                {
                    "payload_len": len(msg.data),
                    "seed_subscriber_count": int(self._seed_pub.get_subscription_count()),
                },
            )
            self.get_logger().info(
                f"[COORD] seed published (subs={int(self._seed_pub.get_subscription_count())}, age={seed_age:.3f}s)",
                throttle_duration_sec=1.0,
            )
            self.state = "IDLE"
        except Exception as exc:
            self.state = "DEGRADED"
            self.get_logger().warn(f"Invalid VLM seed response payload: {exc}")


def main(args=None):
    rclpy.init(args=args)
    node = CoordinatorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

