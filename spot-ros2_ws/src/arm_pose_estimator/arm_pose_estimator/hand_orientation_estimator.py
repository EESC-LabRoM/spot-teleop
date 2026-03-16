#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import cv2
import numpy as np
import mediapipe as mp
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from geometry_msgs.msg import Quaternion
from std_msgs.msg import String
import time
import math
from collections import deque


def quaternion_from_euler(roll, pitch, yaw):
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)

    qw = cr * cp * cy + sr * sp * sy
    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy

    return [qx, qy, qz, qw]


class HandOrientationEstimator(Node):
    def __init__(self):
        super().__init__('hand_orientation_estimator')
        self.bridge = CvBridge()

        self.declare_parameter('color_topic', '/camera/camera/color/image_raw')
        self.color_topic = self.get_parameter('color_topic').value

        # EMA alpha for roll angle smoothing (0 = max smooth/lag, 1 = no smoothing)
        self.declare_parameter('ema_alpha', 0.2)
        self.ema_alpha = self.get_parameter('ema_alpha').value

        self.get_logger().info(f'Subscribing to color topic: {self.color_topic}')
        self.get_logger().info(f'EMA alpha: {self.ema_alpha}')

        self.image_sub = self.create_subscription(
            Image,
            self.color_topic,
            self.image_callback,
            10
        )
        self.quat_pub = self.create_publisher(Quaternion, "/hand_roll_quat", 10)
        self.gesture_sub = self.create_subscription(String, "hand_gesture", self._gesture_cb, 10)

        self.current_gesture = 0
        self.block_duration = 2.0
        self.block_until = 0.0
        self.last_quat_msg = None

        # MediaPipe
        mp_hands = mp.solutions.hands
        self.hands = mp_hands.Hands(
            static_image_mode=False, max_num_hands=1,
            min_detection_confidence=0.6, min_tracking_confidence=0.8)

        # EMA state (circular smoothing via sin/cos to handle angle wrapping)
        self._roll_sin = None
        self._roll_cos = None

        # Buffer for delay
        self.pub_delay = 0.20
        self.queue = deque()

        self.timer = self.create_timer(0.01, self._publish_delayed)
        self.get_logger().info('Hand orientation estimator (EMA + Roll) started.')

    def _gesture_cb(self, msg):
        gesture_name = msg.data
        prev = self.current_gesture

        if "Closed_Fist" in gesture_name:
            self.current_gesture = 1
        elif "Open_Palm" in gesture_name:
            self.current_gesture = 0

        if prev == 0 and self.current_gesture == 1:
            self.block_until = time.time() + self.block_duration
            self.get_logger().info("Grasp gesture detected! Freezing orientation.")

    def image_callback(self, msg):
        t_now = time.time()

        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().warn(f"Erro no CvBridge: {e}")
            return

        debug_frame = frame.copy()
        hand_detected = False
        roll_deg = 0.0
        dir_deg = 0.0
        normal_text = ""

        # If within the block period, publish last quaternion but still process frame for debug
        in_block = self.current_gesture == 1 and t_now < self.block_until and self.last_quat_msg
        if in_block:
            self.quat_pub.publish(self.last_quat_msg)

        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        res = self.hands.process(rgb)

        if res.multi_hand_landmarks and res.multi_hand_world_landmarks:
            hand_detected = True
            lm_world = res.multi_hand_world_landmarks[0].landmark
            world_pts = np.array([[lm.x, lm.y, lm.z] for lm in lm_world])

            p0, p5, p17 = world_pts[0], world_pts[5], world_pts[17]
            normal = np.cross(p17 - p0, p5 - p17)
            if np.linalg.norm(normal) < 1e-6:
                self._draw_debug(debug_frame, hand_detected=False, in_block=in_block)
                return
            normal /= np.linalg.norm(normal)

            # Negated to match gripper orientation (palm normal vs wrist rotation convention)
            roll_raw = -math.atan2(normal[1], normal[0])

            # Circular EMA to handle angle wrapping correctly
            s = math.sin(roll_raw)
            c = math.cos(roll_raw)
            if self._roll_sin is None:
                self._roll_sin, self._roll_cos = s, c
            else:
                alpha = self.ema_alpha
                self._roll_sin = alpha * s + (1.0 - alpha) * self._roll_sin
                self._roll_cos = alpha * c + (1.0 - alpha) * self._roll_cos
            roll_angle = math.atan2(self._roll_sin, self._roll_cos)

            qx, qy, qz, qw = quaternion_from_euler(0, 0, roll_angle)
            quat_msg = Quaternion()
            quat_msg.x = -qz
            quat_msg.y = qy
            quat_msg.z = qx
            quat_msg.w = qw
            self.last_quat_msg = quat_msg

            if not in_block:
                self.queue.append((time.time(), quat_msg))

            # Build debug overlay info
            roll_deg = math.degrees(roll_angle)
            normal_text = f"[{normal[0]:.2f}, {normal[1]:.2f}, {normal[2]:.2f}]"

            mp_draw = mp.solutions.drawing_utils
            mp_draw.draw_landmarks(
                debug_frame, res.multi_hand_landmarks[0], mp.solutions.hands.HAND_CONNECTIONS)

            h, w, _ = debug_frame.shape
            lm2d = res.multi_hand_landmarks[0].landmark
            p0_2d = (int(lm2d[0].x * w), int(lm2d[0].y * h))
            p9_2d = (int(lm2d[9].x * w), int(lm2d[9].y * h))
            dir_deg = math.degrees(math.atan2(p9_2d[1] - p0_2d[1], p9_2d[0] - p0_2d[0]))
            cv2.line(debug_frame, p0_2d, p9_2d, (255, 255, 255), 3)

            center = (w // 2, h // 2)
            length = 100
            end_pt = (int(center[0] + length * math.cos(roll_angle)),
                      int(center[1] + length * math.sin(roll_angle)))
            cv2.line(debug_frame, center, end_pt, (0, 255, 0), 3)
            cv2.circle(debug_frame, center, 5, (0, 0, 255), -1)

        self._draw_debug(debug_frame, hand_detected, in_block, roll_deg, dir_deg, normal_text)

    def _draw_debug(self, frame, hand_detected, in_block=False, roll_deg=0.0, dir_deg=0.0, normal_text=""):
        if hand_detected:
            cv2.putText(frame, f"Normal Roll: {roll_deg:.1f} deg", (50, 50),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
            cv2.putText(frame, f"Hand Dir: {dir_deg:.1f} deg", (50, 85),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
            if normal_text:
                cv2.putText(frame, f"Normal: {normal_text}", (50, 120),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (200, 200, 200), 2)
            if in_block:
                cv2.putText(frame, "FROZEN (fist)", (50, 155),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 165, 255), 2)
        else:
            cv2.putText(frame, "No hand detected", (50, 50),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 0, 255), 2)
            if in_block:
                cv2.putText(frame, "FROZEN (fist)", (50, 90),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 165, 255), 2)

        cv2.imshow("Hand Orientation Debug", frame)
        cv2.waitKey(1)

    def _publish_delayed(self):
        now = time.time()
        while self.queue and (now - self.queue[0][0] >= self.pub_delay):
            _, qm = self.queue.popleft()
            self.quat_pub.publish(qm)


def main(args=None):
    rclpy.init(args=args)
    node = HandOrientationEstimator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.hands.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
