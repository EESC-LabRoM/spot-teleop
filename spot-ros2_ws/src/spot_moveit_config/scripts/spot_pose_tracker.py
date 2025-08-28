#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, TransformStamped
import tf2_ros
import tf2_geometry_msgs
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
import time

class SpotPoseTracker(Node):
    def __init__(self):
        super().__init__('spot_pose_tracker')
        
        # Publisher para comando de pose do servo
        self.pose_pub = self.create_publisher(
            PoseStamped,
            '/servo_node/pose_target_cmds',
            10
        )
        
        # TF2 setup
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Timer para buscar e publicar poses
        self.timer = self.create_timer(0.1, self.track_target_pose)  # 10 Hz
        
        # Frame de referência
        self.target_frame = "test"      # Frame que queremos seguir
        self.reference_frame = "body"   # Frame de referência para o comando
        
        self.get_logger().info('Spot Pose Tracker iniciado!')
        self.get_logger().info(f'Rastreando frame: {self.target_frame}')
        self.get_logger().info(f'Frame de referência: {self.reference_frame}')
        self.get_logger().info('Publicando comandos para: /servo_node/pose_target_cmds')
        
    def track_target_pose(self):
        """Obtém a pose do frame target e publica como comando para o servo"""
        try:
            # Obter a transformação do frame de referência para o frame target
            transform = self.tf_buffer.lookup_transform(
                self.reference_frame,
                self.target_frame,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.1)
            )
            
            # Converter para PoseStamped
            pose_msg = PoseStamped()
            pose_msg.header.stamp = self.get_clock().now().to_msg()
            pose_msg.header.frame_id = self.reference_frame
            
            # Extrair posição e orientação da transformação
            pose_msg.pose.position.x = transform.transform.translation.x
            pose_msg.pose.position.y = transform.transform.translation.y
            pose_msg.pose.position.z = transform.transform.translation.z
            
            pose_msg.pose.orientation.x = transform.transform.rotation.x
            pose_msg.pose.orientation.y = transform.transform.rotation.y
            pose_msg.pose.orientation.z = transform.transform.rotation.z
            pose_msg.pose.orientation.w = transform.transform.rotation.w
            
            # Publicar comando de pose
            self.pose_pub.publish(pose_msg)
            
            # Log ocasional para debug
            current_time = time.time()
            if not hasattr(self, 'last_log_time'):
                self.last_log_time = current_time
            
            if current_time - self.last_log_time > 2.0:  # Log a cada 2 segundos
                self.get_logger().info(
                    f'Rastreando pose: x={pose_msg.pose.position.x:.3f}, '
                    f'y={pose_msg.pose.position.y:.3f}, '
                    f'z={pose_msg.pose.position.z:.3f}'
                )
                self.last_log_time = current_time
                
        except tf2_ros.LookupException:
            # Frame não encontrado - isso é normal no início
            pass
        except tf2_ros.ConnectivityException:
            self.get_logger().warn(f'Conectividade perdida entre frames {self.reference_frame} e {self.target_frame}')
        except tf2_ros.ExtrapolationException:
            self.get_logger().warn(f'Erro de extrapolação temporal para frames {self.reference_frame} -> {self.target_frame}')
        except Exception as e:
            self.get_logger().error(f'Erro inesperado no pose tracking: {str(e)}')


def main(args=None):
    rclpy.init(args=args)
    
    node = SpotPoseTracker()
    
    try:
        node.get_logger().info('Pose tracker rodando. Pressione Ctrl+C para parar...')
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Parando o pose tracker...')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
