#!/usr/bin/env python3

"""
Real-time Valve Detection Script using YOLOv11n
Subscribes to /camera/rgb topic and performs valve detection with bounding box visualization
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2D, ObjectHypothesisWithPose, BoundingBox2D
from rclpy.qos import qos_profile_sensor_data
from cv_bridge import CvBridge
import cv2
import numpy as np
from ultralytics import YOLO
import os
from pathlib import Path


class ValveDetectionNode(Node):
    """ROS2 Node for real-time valve detection using YOLOv11n"""
    
    def __init__(self):
        super().__init__('valve_detection_node')
        
        # Initialize cv_bridge for ROS-OpenCV conversion
        self.bridge = CvBridge()
        
        # Get model path relative to this package
        package_path = Path(__file__).parent.parent
        model_path = package_path / "models" / "best.pt"
        
        # Load the best.pt model
        self.get_logger().info(f"Loading custom YOLO model from: {model_path}")
        self.model = YOLO(str(model_path))
        self.get_logger().info("Custom YOLO model loaded successfully")
        
        # (opcional) parametrizar tópicos e conf
        self.declare_parameter('rgb_topic', '/camera/rgb')
        self.declare_parameter('det_topic', '/detection2_d')
        self.declare_parameter('conf_thres', 0.5)
        self.declare_parameter('target_class', 'lever_10')  # Classe alvo para publicação
        rgb_topic = self.get_parameter('rgb_topic').get_parameter_value().string_value
        det_topic = self.get_parameter('det_topic').get_parameter_value().string_value
        self.conf_thres = self.get_parameter('conf_thres').get_parameter_value().double_value
        self.target_class = self.get_parameter('target_class').get_parameter_value().string_value
        
        # Use default QoS (RELIABLE) for compatibility with Detection2DToMask
        self.det_pub = self.create_publisher(Detection2D, det_topic, 10)
        
        # usar o tópico parametrizado com QoS de sensor
        self.subscription = self.create_subscription(Image, rgb_topic, self.image_callback, qos_profile_sensor_data)
        
        self.get_logger().info(f"Valve Detection Node initialized. Subscribing to {rgb_topic}")
        self.get_logger().info(f"Publishing detections to {det_topic} with confidence threshold: {self.conf_thres}")
        self.get_logger().info(f"Target class for publishing: '{self.target_class}'")
        
        # Performance tracking
        self.frame_count = 0
        self.detection_count = 0
        
    def image_callback(self, msg):
        """
        Callback function for processing incoming camera images
        """
        # Convert ROS Image message to OpenCV format
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        
        # Run YOLO inference on the frame
        results = self.model(cv_image, conf=self.conf_thres, verbose=False)
        res = results[0]
        
        # --- PUBLICA APENAS A MELHOR DETECÇÃO DA CLASSE ALVO ---
        if res.boxes is not None and len(res.boxes) > 0:
            xyxy = res.boxes.xyxy.cpu().numpy()
            conf = res.boxes.conf.cpu().numpy()
            cls  = res.boxes.cls.cpu().numpy().astype(int)

            # Filtra apenas detecções da classe alvo
            target_indices = []
            for i in range(len(cls)):
                class_name = self.model.names.get(cls[i], '')
                if class_name == self.target_class:
                    target_indices.append(i)
            
            # Se encontrou detecções da classe alvo
            if len(target_indices) > 0:
                # Pega as confidências apenas das detecções da classe alvo
                target_conf = conf[target_indices]
                # Encontra a melhor entre as detecções da classe alvo
                best_target_idx = target_indices[np.argmax(target_conf)]
                
                x1, y1, x2, y2 = xyxy[best_target_idx]
                p = conf[best_target_idx]
                c = cls[best_target_idx]

                # Cria Detection2D individual
                det = Detection2D()
                det.header = msg.header

                hyp = ObjectHypothesisWithPose()
                hyp.hypothesis.class_id = str(c)
                hyp.hypothesis.score = float(p)
                det.results.append(hyp)

                bb = BoundingBox2D()
                bb.center.position.x = float((x1 + x2) / 2.0)
                bb.center.position.y = float((y1 + y2) / 2.0)
                bb.center.theta = 0.0
                bb.size_x = float(max(0.0, x2 - x1))
                bb.size_y = float(max(0.0, y2 - y1))
                det.bbox = bb

                # Publica Detection2D individual
                self.det_pub.publish(det)
        # --- FIM PUBLICAÇÃO ---
        
        # Process results and draw bounding boxes
        annotated_frame = self.draw_detections(cv_image, res)
        
        # Display the frame with detections
        cv2.imshow("Valve Detection - Real Time", annotated_frame)
        cv2.waitKey(1)  # Non-blocking wait for key press
        
        # Update counters
        self.frame_count += 1
        
        # Log detection info periodically
        if self.frame_count % 30 == 0:  # Every 30 frames
            self.get_logger().info(f"Processed {self.frame_count} frames, "
                                 f"Total detections: {self.detection_count}")
    
    def draw_detections(self, image, result):
        """
        Draw bounding boxes and labels on the image for detected valves
        
        Args:
            image: OpenCV image (BGR format)
            result: YOLO detection result
            
        Returns:
            annotated_image: Image with bounding boxes and labels
        """
        annotated_image = image.copy()
        
        # Check if there are any detections
        if result.boxes is not None and len(result.boxes.xyxy) > 0:
            boxes = result.boxes
            
            # Get detection data
            xyxy = boxes.xyxy.cpu().numpy().astype(int)  # Bounding boxes
            conf = boxes.conf.cpu().numpy()  # Confidence scores
            cls = boxes.cls.cpu().numpy().astype(int)  # Class IDs
            
            # Draw each detection
            for i in range(len(xyxy)):
                x1, y1, x2, y2 = xyxy[i]
                confidence = conf[i]
                class_id = cls[i]
                
                # Get class name (default to 'valve' if not available)
                class_name = self.model.names.get(class_id, 'valve')
                
                # Define colors for different classes (BGR format)
                colors = [
                    (0, 255, 0),    # Green
                    (255, 0, 0),    # Blue
                    (0, 0, 255),    # Red
                    (255, 255, 0),  # Cyan
                    (255, 0, 255),  # Magenta
                    (0, 255, 255),  # Yellow
                ]
                color = colors[class_id % len(colors)]
                
                # Draw bounding box
                cv2.rectangle(annotated_image, (x1, y1), (x2, y2), color, 2)
                
                # Prepare label text
                label = f"{class_name}: {confidence:.2f}"
                
                # Get text size for background rectangle
                (text_width, text_height), _ = cv2.getTextSize(
                    label, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 2
                )
                
                # Draw label background
                cv2.rectangle(
                    annotated_image,
                    (x1, y1 - text_height - 10),
                    (x1 + text_width, y1),
                    color,
                    -1
                )
                
                # Draw label text
                cv2.putText(
                    annotated_image,
                    label,
                    (x1, y1 - 5),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.6,
                    (255, 255, 255),  # White text
                    2
                )
                
                self.detection_count += 1
                
                # Log detection
                self.get_logger().debug(
                    f"Detected {class_name} at ({x1},{y1},{x2},{y2}) "
                    f"with confidence {confidence:.3f}"
                )
        
        # Add frame info overlay
        info_text = f"Frame: {self.frame_count} | Detections: {self.detection_count}"
        cv2.putText(
            annotated_image,
            info_text,
            (10, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (255, 255, 255),
            2
        )
        
        return annotated_image
    
    def cleanup(self):
        """Clean up resources"""
        cv2.destroyAllWindows()
        self.get_logger().info("Valve Detection Node shutting down")


def main(args=None):
    """Main function to start the valve detection node"""
    rclpy.init(args=args)
    
    # Create and run the valve detection node
    valve_detector = ValveDetectionNode()
    
    try:
        rclpy.spin(valve_detector)
    except KeyboardInterrupt:
        valve_detector.get_logger().info("Keyboard interrupt received")
    finally:
        valve_detector.cleanup()
        valve_detector.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
