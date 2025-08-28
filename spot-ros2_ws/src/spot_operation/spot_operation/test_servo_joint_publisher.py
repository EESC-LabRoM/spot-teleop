#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, TransformStamped
from tf2_ros import TransformBroadcaster
import math
import time

class ServoCommandTester(Node):
    def __init__(self):
        super().__init__('servo_command_tester')
        
        # Novo: Publisher pra target pose (pro tracking)
        self.pose_pub = self.create_publisher(
            PoseStamped,
            '/target_pose',
            10
        )
        
        # TF broadcaster mantém (body -> test)
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Posição/orientação inicial do target (ajustada para alcance do braço)
        self.target_position = [0.6, 0.0, 0.3]  # x, y, z - dentro do alcance do braço
        self.target_orientation = [0.0, 0.0, 0.0]  # roll, pitch, yaw
        
        # Timer pra enviar poses alvo
        self.servo_timer = self.create_timer(0.01, self.send_target_pose)  # 50 Hz
        
        # Vars pra movimento senoidal
        self.time_start = time.time()
        
        self.get_logger().info('Pose Tracking Tester iniciado!')
        self.get_logger().info('Enviando poses alvo para /target_pose')
        self.get_logger().info('Publicando TF: body -> test (pra visualizar)')
        self.get_logger().info('Monitore: /servo_node/status e /servo_node/delta_twist_cmds (gerado internamente)')

    def publish_test_tf(self):
        """Publica TF body -> test baseada na target pose"""
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "body"
        t.child_frame_id = "test"
        
        t.transform.translation.x = self.target_position[0]
        t.transform.translation.y = self.target_position[1]
        t.transform.translation.z = self.target_position[2]
        
        yaw = self.target_orientation[2]
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = math.sin(yaw / 2.0)
        t.transform.rotation.w = math.cos(yaw / 2.0)
        
        self.tf_broadcaster.sendTransform(t)

    def send_target_pose(self):
        """Calcula e envia uma pose alvo suave (senoidal)"""
        current_time = time.time() - self.time_start
        
        # Use real dt instead of fixed value
        now = time.time()
        dt = now - self.prev_time if hasattr(self, "prev_time") else 0.02
        self.prev_time = now
        
        # Movimento senoidal na pose alvo – pequeno pra evitar singularity
        delta_x = 0.03 * math.sin(0.2 * current_time)  # Reduzido para workspace menor
        delta_y = 0.03 * math.cos(0.2 * current_time)
        delta_z = 0.015 * math.sin(0.1 * current_time)
        delta_yaw = 0.1 * math.sin(0.1 * current_time)
        
        # Atualiza target acumulada (pra movimento contínuo)
        self.target_position[0] += delta_x * dt
        self.target_position[1] += delta_y * dt
        self.target_position[2] += delta_z * dt
        self.target_orientation[2] += delta_yaw * dt
        
        # Cria e publica PoseStamped
        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = "body"  # Frame de referência (seu planning frame)
        
        pose_msg.pose.position.x = self.target_position[0]
        pose_msg.pose.position.y = self.target_position[1]
        pose_msg.pose.position.z = self.target_position[2]
        
        yaw = self.target_orientation[2]
        pose_msg.pose.orientation.x = 0.0
        pose_msg.pose.orientation.y = 0.0
        pose_msg.pose.orientation.z = math.sin(yaw / 2.0)
        pose_msg.pose.orientation.w = math.cos(yaw / 2.0)
        
        self.pose_pub.publish(pose_msg)
        
        # Log a cada 2 seg
        if int(current_time) % 2 == 0:
            self.get_logger().info(f'Enviando target pose: pos=({pose_msg.pose.position.x:.2f}, {pose_msg.pose.position.y:.2f}, {pose_msg.pose.position.z:.2f}), yaw={yaw:.2f}')
        
        # Atualiza TF pra visualizar no RViz
        self.publish_test_tf()

def main(args=None):
    rclpy.init(args=args)
    node = ServoCommandTester()
    try:
        node.get_logger().info('Pressione Ctrl+C para parar...')
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Parando o pose tester...')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()