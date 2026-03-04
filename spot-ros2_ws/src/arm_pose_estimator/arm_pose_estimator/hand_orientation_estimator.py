#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import cv2
import numpy as np
import mediapipe as mp
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from geometry_msgs.msg import Quaternion
from std_msgs.msg import Int32
import time
import math
from collections import deque

# ---------- One‑Euro filter ----------
class OneEuroFilter:
    def __init__(self, freq, min_cutoff=1.0, beta=0.0, d_cutoff=1.0):
        self.freq = freq
        self.min_cutoff = min_cutoff
        self.beta = beta
        self.d_cutoff = d_cutoff
        self.x_prev = None
        self.dx_prev = None
        self.t_prev = None

    def _alpha(self, cutoff, dt):
        tau = 1.0 / (2 * math.pi * cutoff)
        return 1.0 / (1.0 + tau / dt)

    def __call__(self, x, t):
        if self.t_prev is None:
            self.x_prev, self.dx_prev, self.t_prev = x, np.zeros_like(x), t
            return x
        dt = t - self.t_prev
        if dt <= 0.0: return x
        dx = (x - self.x_prev) / dt
        a_d = self._alpha(self.d_cutoff, dt)
        dx_hat = a_d * dx + (1 - a_d) * self.dx_prev
        cutoff = self.min_cutoff + self.beta * np.linalg.norm(dx_hat)
        a = self._alpha(cutoff, dt)
        x_hat = a * x + (1 - a) * self.x_prev
        self.x_prev, self.dx_prev, self.t_prev = x_hat, dx_hat, t
        return x_hat

def quaternion_from_euler(roll, pitch, yaw):
    """
    Simula o tf.transformations.quaternion_from_euler (para evitar problemas de dependencia no ROS 2)
    """
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
        
        self.image_sub = self.create_subscription(
            Image, 
            "/camera/color/image_raw", 
            self.image_callback, 
            10
        )
        self.quat_pub = self.create_publisher(Quaternion, "/hand_roll_quat", 10)
        self.gesture_sub = self.create_subscription(Int32, "/hand_gesture", self._gesture_cb, 10)

        self.current_gesture = 0
        self.block_duration = 2.0        # Time to hold the quaternion (seconds)
        self.block_until = 0.0           # Timestamp until when to block
        self.last_quat_msg = None        # Stores the last quaternion

        # MediaPipe
        mp_hands = mp.solutions.hands
        self.hands = mp_hands.Hands(
            static_image_mode=False, max_num_hands=1,
            min_detection_confidence=0.6, min_tracking_confidence=0.8)

        # Filters for 21 landmarks
        self.filters = [OneEuroFilter(freq=30, min_cutoff=0.4, beta=0.1) for _ in range(21)]

        # Buffer for delay
        self.pub_delay = 0.20                     # 200 ms (em segundos)
        self.queue = deque()                      # [(time_float, Quaternion), ...]
        
        # Timer to periodically publish delayed messages
        self.timer = self.create_timer(0.01, self._publish_delayed)
        self.get_logger().info('Hand orientation estimator (One-Euro + Roll) started.')

    def _gesture_cb(self, msg):
        prev = self.current_gesture
        self.current_gesture = msg.data
        # On transition from gesture 0 to 1, start the block timer
        if prev == 0 and self.current_gesture == 1:
            self.block_until = time.time() + self.block_duration

    def image_callback(self, msg):
        t_now = time.time()
        # If within the block period, republish the last quaternion and exit
        if self.current_gesture == 1 and t_now < self.block_until and self.last_quat_msg:
            self.quat_pub.publish(self.last_quat_msg)
            return

        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().warn(f"Erro no CvBridge: {e}")
            return

        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        res = self.hands.process(rgb)

        if res.multi_hand_landmarks and res.multi_hand_world_landmarks:
            lm_world = res.multi_hand_world_landmarks[0].landmark
            t_now = time.time()
            world_pts = np.array([[lm.x, lm.y, lm.z] for lm in lm_world])
            for i in range(21):
                world_pts[i] = self.filters[i](world_pts[i], t_now)

            # Use 0‑5‑17 for normal calculation
            p0, p5, p17 = world_pts[0], world_pts[5], world_pts[17]
            normal = np.cross(p17 - p0, p5 - p17)
            if np.linalg.norm(normal) < 1e-6:
                return
            normal /= np.linalg.norm(normal)

            # Calculate roll angle
            roll_angle = math.atan2(normal[1], normal[0])  # atan2(Y, X)

            # Create quaternion with only roll
            qx, qy, qz, qw = quaternion_from_euler(0, 0, roll_angle)

            # Save the quaternion for reuse during the block period
            quat_msg = Quaternion()
            quat_msg.x = -qz
            quat_msg.y = qy
            quat_msg.z = qx
            quat_msg.w = qw
            self.last_quat_msg = quat_msg

            # Add to buffer with current timestamp
            self.queue.append((time.time(), quat_msg))

    def _publish_delayed(self):
        """Publish all quaternions in the buffer older than pub_delay."""
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