#!/usr/bin/env python3
"""
Script para detectar todos AprilTags dos tópicos da câmera ZED.

Este script se inscreve no tópico de imagem da câmera ZED, detecta
todos os AprilTags e mostra ID, tamanho em pixels e pose completa (posição 3D + orientação).

Uso:
    python3 apriltag_detector_zed.py
    
Pressione 'q' para sair.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2
import numpy as np


class AprilTagDetectorZED(Node):
    """Nó ROS2 para detectar pose completa de todos AprilTags da câmera ZED."""
    
    def __init__(self):
        super().__init__('apriltag_detector_zed')
        
        # Inicializar CV bridge
        self.bridge = CvBridge()
        
        # Configurar detector AprilTag usando cv2.aruco
        # AprilTag 36h11 é o formato mais comum
        self.apriltag_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
        self.apriltag_params = cv2.aruco.DetectorParameters()
        self.detector = cv2.aruco.ArucoDetector(self.apriltag_dict, self.apriltag_params)
        
        # Tamanho de referência do AprilTag em metros (usado para cálculo de pose)
        self.declare_parameter('tag_size', 0.2)  # 5cm por padrão
        self.tag_size = self.get_parameter('tag_size').value
        
        # Parâmetros da câmera (serão atualizados pelo tópico camera_info)
        self.camera_matrix = None
        self.dist_coeffs = None
        
        # Tópicos da ZED
        self.image_topic = '/zed/zed_node/rgb/image_rect_color'
        self.camera_info_topic = '/zed/zed_node/rgb/camera_info'
        
        # Criar subscribers
        self.image_sub = self.create_subscription(
            Image,
            self.image_topic,
            self.image_callback,
            10
        )
        
        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            self.camera_info_topic,
            self.camera_info_callback,
            10
        )
        
        # Estatísticas
        self.frame_count = 0
        self.detected_ids = {}  # Dicionário para rastrear detecções por ID
        
        self.get_logger().info('=== Detector de Pose AprilTag (Todos IDs) Iniciado ===')
        self.get_logger().info(f'Inscrito no tópico imagem: {self.image_topic}')
        self.get_logger().info(f'Inscrito no tópico camera_info: {self.camera_info_topic}')
        self.get_logger().info(f'Tamanho de referência: {self.tag_size}m (usado para cálculo de pose)')
        self.get_logger().info('Detectando todos AprilTags...')
        self.get_logger().info('Pressione "q" na janela para sair')
    
    def camera_info_callback(self, msg):
        """Receber parâmetros intrínsecos da câmera."""
        if self.camera_matrix is None:
            # Extrair matriz de câmera (K) e coeficientes de distorção
            self.camera_matrix = np.array(msg.k).reshape(3, 3)
            self.dist_coeffs = np.array(msg.d)
            
            self.get_logger().info('Parâmetros da câmera recebidos!')
            self.get_logger().info(f'Resolução: {msg.width}x{msg.height}')
            self.get_logger().info(f'Focal length (fx, fy): ({self.camera_matrix[0,0]:.2f}, {self.camera_matrix[1,1]:.2f})')

    def image_callback(self, msg):
        """Processar imagens recebidas e detectar pose de todos AprilTags."""
        try:
            # Converter mensagem ROS Image para formato OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            # Converter para escala de cinza para melhor detecção
            gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
            
            # Detectar AprilTags
            corners, ids, rejected = self.detector.detectMarkers(gray)
            
            # Criar cópia da imagem para desenhar
            display_image = cv_image.copy()
            
            # Verificar se algum tag foi detectado
            if ids is not None and len(ids) > 0:
                # Processar todos os tags detectados
                for i, tag_id in enumerate(ids):
                    tag_id_num = tag_id[0]
                    
                    # Atualizar contador de detecções por ID
                    if tag_id_num not in self.detected_ids:
                        self.detected_ids[tag_id_num] = 0
                    self.detected_ids[tag_id_num] += 1
                    
                    # Desenhar contorno do tag
                    cv2.aruco.drawDetectedMarkers(display_image, [corners[i]], ids[i])
                    
                    # Calcular centro e perímetro do tag (2D)
                    corner_points = corners[i][0]
                    center_x = int(np.mean(corner_points[:, 0]))
                    center_y = int(np.mean(corner_points[:, 1]))
                    
                    # Calcular perímetro em pixels (indicador de tamanho relativo)
                    perimeter = 0
                    for j in range(4):
                        p1 = corner_points[j]
                        p2 = corner_points[(j + 1) % 4]
                        perimeter += np.linalg.norm(p2 - p1)
                    
                    # Calcular área em pixels
                    area = cv2.contourArea(corner_points)
                    
                    # Calcular lado médio em pixels
                    avg_side = perimeter / 4.0
                    
                    # Calcular pose 3D se temos parâmetros da câmera
                    if self.camera_matrix is not None:
                        # Definir pontos 3D do AprilTag (origem no centro)
                        half_size = self.tag_size / 2.0
                        obj_points = np.array([
                            [-half_size, -half_size, 0],
                            [ half_size, -half_size, 0],
                            [ half_size,  half_size, 0],
                            [-half_size,  half_size, 0]
                        ], dtype=np.float32)
                        
                        # Estimar pose usando solvePnP
                        success, rvec, tvec = cv2.solvePnP(
                            obj_points,
                            corner_points,
                            self.camera_matrix,
                            self.dist_coeffs,
                            flags=cv2.SOLVEPNP_IPPE_SQUARE
                        )
                        
                        if success:
                            # Desenhar eixos 3D
                            cv2.drawFrameAxes(display_image, self.camera_matrix, 
                                            self.dist_coeffs, rvec, tvec, 
                                            self.tag_size * 0.5)
                            
                            # Converter vetor de rotação para ângulos de Euler
                            rmat, _ = cv2.Rodrigues(rvec)
                            sy = np.sqrt(rmat[0, 0]**2 + rmat[1, 0]**2)
                            singular = sy < 1e-6
                            
                            if not singular:
                                roll = np.arctan2(rmat[2, 1], rmat[2, 2])
                                pitch = np.arctan2(-rmat[2, 0], sy)
                                yaw = np.arctan2(rmat[1, 0], rmat[0, 0])
                            else:
                                roll = np.arctan2(-rmat[1, 2], rmat[1, 1])
                                pitch = np.arctan2(-rmat[2, 0], sy)
                                yaw = 0
                            
                            # Converter para graus
                            roll_deg = np.degrees(roll)
                            pitch_deg = np.degrees(pitch)
                            yaw_deg = np.degrees(yaw)
                            
                            # Extrair posição (translação)
                            x, y, z = tvec[0][0], tvec[1][0], tvec[2][0]
                            
                            # Estimar tamanho real baseado na distância e pixels
                            # pixels_per_meter = focal_length * real_size / distance
                            # Então: real_size_estimate = (avg_side * distance) / focal_length
                            focal_length = self.camera_matrix[0, 0]  # fx
                            # Usar valor absoluto de Z para evitar tamanhos negativos
                            # (Z negativo pode indicar ambiguidade na pose ou tag visto de trás)
                            estimated_size = (avg_side * abs(z)) / focal_length
                            
                            # Posição do texto (à direita do tag)
                            text_x = center_x + 50
                            text_y = center_y - 60
                            
                            # Cor baseada no ID (para diferenciar tags) - usar cores predefinidas
                            colors = [
                                (0, 255, 0),    # Verde
                                (255, 0, 0),    # Azul
                                (0, 0, 255),    # Vermelho
                                (255, 255, 0),  # Ciano
                                (255, 0, 255),  # Magenta
                                (0, 255, 255),  # Amarelo
                                (128, 255, 0),  # Verde-claro
                                (255, 128, 0),  # Azul-claro
                                (128, 0, 255),  # Rosa
                                (0, 128, 255),  # Laranja
                            ]
                            color = colors[tag_id_num % len(colors)]
                            
                            # Mostrar informações próximas ao tag
                            cv2.putText(display_image, f"ID: {tag_id_num}", 
                                      (text_x, text_y), cv2.FONT_HERSHEY_SIMPLEX, 
                                      0.6, color, 2)
                            
                            cv2.putText(display_image, f"Tamanho pixels: {avg_side:.1f}px", 
                                      (text_x, text_y + 25), cv2.FONT_HERSHEY_SIMPLEX, 
                                      0.5, color, 2)
                            
                            # Adicionar aviso se Z é negativo (possível ambiguidade)
                            size_text = f"Tamanho estimado: {estimated_size*100:.1f}cm"
                            if z < 0:
                                size_text += " (?)"  # Indica possível ambiguidade na pose
                            cv2.putText(display_image, size_text, 
                                      (text_x, text_y + 45), cv2.FONT_HERSHEY_SIMPLEX, 
                                      0.5, color, 2)
                            
                            cv2.putText(display_image, f"Pos: [{x:.2f}, {y:.2f}, {z:.2f}]m", 
                                      (text_x, text_y + 65), cv2.FONT_HERSHEY_SIMPLEX, 
                                      0.5, color, 2)
                            
                            cv2.putText(display_image, f"Rot: [R:{roll_deg:.0f} P:{pitch_deg:.0f} Y:{yaw_deg:.0f}]", 
                                      (text_x, text_y + 85), cv2.FONT_HERSHEY_SIMPLEX, 
                                      0.5, color, 2)
                            
                            # Log de detecção periódico
                            if self.detected_ids[tag_id_num] % 30 == 0:
                                self.get_logger().info(
                                    f'AprilTag ID {tag_id_num} - Tamanho: ~{estimated_size*100:.1f}cm ({avg_side:.0f}px) - '
                                    f'Pos: [{x:.2f}, {y:.2f}, {z:.2f}]m - '
                                    f'Rot: [R:{roll_deg:.0f}°, P:{pitch_deg:.0f}°, Y:{yaw_deg:.0f}°]'
                                )
                        else:
                            # Pose não calculada, mostrar apenas info 2D
                            cv2.putText(display_image, f"ID: {tag_id_num} - {avg_side:.0f}px", 
                                      (center_x + 20, center_y), cv2.FONT_HERSHEY_SIMPLEX, 
                                      0.6, (0, 255, 0), 2)
                    else:
                        # Aguardando parâmetros da câmera
                        cv2.putText(display_image, f"ID: {tag_id_num} - {avg_side:.0f}px - Aguardando camera_info...", 
                                  (center_x + 20, center_y), cv2.FONT_HERSHEY_SIMPLEX, 
                                  0.6, (0, 255, 255), 2)
            
            # Adicionar informações gerais na imagem
            self.frame_count += 1
            total_detections = sum(self.detected_ids.values())
            num_unique_ids = len(self.detected_ids)
            
            info_text = f"Frame: {self.frame_count} | Tags detectados: {num_unique_ids} IDs | Total deteccoes: {total_detections}"
            cv2.putText(display_image, info_text, (10, display_image.shape[0] - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
            
            # Listar IDs detectados
            if self.detected_ids:
                ids_list = ", ".join([f"#{id}" for id in sorted(self.detected_ids.keys())])
                cv2.putText(display_image, f"IDs ativos: {ids_list}", (10, display_image.shape[0] - 30),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
            
            # Mostrar imagem
            cv2.imshow('AprilTag Detector (Todos IDs) - ZED Camera', display_image)
            cv2.waitKey(1)
            
        except Exception as e:
            self.get_logger().error(f'Erro ao processar imagem: {str(e)}')

    def destroy_node(self):
        """Limpar recursos ao destruir o nó."""
        cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    """Função principal."""
    rclpy.init(args=args)
    
    detector = AprilTagDetectorZED()
    
    try:
        rclpy.spin(detector)
    except KeyboardInterrupt:
        detector.get_logger().info('Interrompido pelo usuário')
    finally:
        # Limpar
        detector.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
