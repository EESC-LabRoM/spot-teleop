#!/usr/bin/env python3
import rospy
import moveit_commander
from moveit_msgs.msg import JointConstraint, Constraints
from geometry_msgs.msg import PoseStamped, Pose
import subprocess
from std_msgs.msg import Int32
from abc import ABC, abstractmethod

# Latency measurement imports
from spot_latency_logger.msg import YoloDetection, SpotCommand, SpotFeedback

import bosdyn.client
import bosdyn.client.util
from bosdyn.client.robot_command import RobotCommandBuilder, RobotCommandClient
from bosdyn.client.robot_state import RobotStateClient
from bosdyn.client.frame_helpers import ODOM_FRAME_NAME, get_a_tform_b
from bosdyn.client.math_helpers import SE3Pose, Quat
from bosdyn.client.lease import LeaseClient, LeaseWallet, add_lease_wallet_processors, LeaseKeepAlive
from bosdyn.client.lease import ResourceAlreadyClaimedError

import tf2_ros
import tf2_geometry_msgs
from geometry_msgs.msg import TransformStamped

import cv2
import numpy as np
import pandas as pd
from ultralytics import YOLO

from bosdyn.client.image import ImageClient
from bosdyn.client.manipulation_api_client import ManipulationApiClient
from bosdyn.api import geometry_pb2, manipulation_api_pb2, image_pb2
from std_srvs.srv import Trigger

import time
import math
from bosdyn.client.robot_command import RobotCommandBuilder


class RobotClientManager:
    """Gerencia conexões e clientes do Spot."""
    
    def __init__(self, hostname="192.168.80.3", username="admin", password="spotadmin2017",
                 cmd_pub=None, feedback_pub=None):
        self.hostname = hostname
        self.username = username
        self.password = password
        self.sdk = None
        self.robot = None
        self.command_client = None
        self.state_client = None
        self.lease_client = None
        self.image_client = None
        self.manipulation_client = None
        self.lease_keepalive = None
        self.lease_wallet = None
        # Publishers para latência (opcionais, não alteram comportamento)
        self._cmd_pub = cmd_pub
        self._feedback_pub = feedback_pub
        
    def connect(self):
        """Estabelece conexão com o robô e inicializa clientes necessários."""
        self.sdk = bosdyn.client.create_standard_sdk("SpotOperationSDK")
        self.robot = self.sdk.create_robot(self.hostname)
        self.robot.authenticate(self.username, self.password)
        self.robot.time_sync.wait_for_sync()
        
        if not self.robot.has_arm():
            rospy.logerr("Spot não possui braço. Abortando.")
            return False
            
        # Inicializa os clientes principais
        self.command_client = self.robot.ensure_client(RobotCommandClient.default_service_name)
        self.state_client = self.robot.ensure_client(RobotStateClient.default_service_name)
        self.lease_client = self.robot.ensure_client(LeaseClient.default_service_name)
        self.image_client = self.robot.ensure_client(ImageClient.default_service_name)
        self.manipulation_client = self.robot.ensure_client(ManipulationApiClient.default_service_name)
        
        return True
        
    def acquire_lease(self):
        """Adquire lease para controle do robô."""
        try:
            root_lease = self.lease_client.acquire()
        except ResourceAlreadyClaimedError:
            rospy.logwarn("Lease já em uso; forçando aquisição com take().")
            root_lease = self.lease_client.take()
            
        # Configura wallet e processadores
        self.lease_wallet = LeaseWallet()
        add_lease_wallet_processors(self.command_client, self.lease_wallet)
        self.lease_wallet.add(root_lease)
        
        # Mantém lease ativo
        self.lease_keepalive = LeaseKeepAlive(self.lease_client, self.lease_wallet,
                                           must_acquire=True, return_at_exit=False)
        return self.lease_keepalive
        
    def power_on(self):
        """Liga os motores do robô."""
        return self.robot.power_on(timeout_sec=20)
        
    def get_robot_state(self):
        """Obtém o estado atual do robô."""
        return self.state_client.get_robot_state()
        
    def send_arm_command(self, odom_T_hand):
        """Envia comando de posição para o braço."""
        # T5: publica intenção de comando
        if self._cmd_pub:
            cmd_msg = SpotCommand()
            cmd_msg.header.stamp = rospy.Time.now()
            cmd_msg.command_type = "arm_pose"
            cmd_msg.has_pose = False  # manter sem pose para não alterar comportamento
            self._cmd_pub.publish(cmd_msg)
        arm_command = RobotCommandBuilder.arm_pose_command(
            odom_T_hand.x, odom_T_hand.y, odom_T_hand.z,
            odom_T_hand.rot.w, odom_T_hand.rot.x,
            odom_T_hand.rot.y, odom_T_hand.rot.z,
            ODOM_FRAME_NAME, 0.5
        )
        cmd_id = self.command_client.robot_command(arm_command)
        # T6: confirmação imediata de envio (não é sucesso de execução)
        if self._feedback_pub:
            fb = SpotFeedback()
            fb.header.stamp = rospy.Time.now()
            fb.command_type = "arm_pose"
            fb.success = True
            fb.message = "command_dispatched"
            self._feedback_pub.publish(fb)
        return cmd_id
        
    def open_gripper(self):
        """Abre a garra do robô."""
        if self._cmd_pub:
            cmd_msg = SpotCommand()
            cmd_msg.header.stamp = rospy.Time.now()
            cmd_msg.command_type = "gripper_open"
            cmd_msg.has_pose = False
            self._cmd_pub.publish(cmd_msg)
        gripper_command = RobotCommandBuilder.claw_gripper_open_command()
        cmd_id = self.command_client.robot_command(gripper_command)
        if self._feedback_pub:
            fb = SpotFeedback()
            fb.header.stamp = rospy.Time.now()
            fb.command_type = "gripper_open"
            fb.success = True
            fb.message = "command_dispatched"
            self._feedback_pub.publish(fb)
        return cmd_id
        
    def close_gripper(self, block=False, timeout_sec=2.0):
        """Fecha a garra. O parâmetro 'block' é ignorado pois não há suporte para block_until_cmd_id."""
        if self._cmd_pub:
            cmd_msg = SpotCommand()
            cmd_msg.header.stamp = rospy.Time.now()
            cmd_msg.command_type = "gripper_close"
            cmd_msg.has_pose = False
            self._cmd_pub.publish(cmd_msg)
        gripper_cmd = RobotCommandBuilder.claw_gripper_close_command()
        cmd_id = self.command_client.robot_command(gripper_cmd)
        # block_until_cmd_id removido pois não existe na API
        if self._feedback_pub:
            fb = SpotFeedback()
            fb.header.stamp = rospy.Time.now()
            fb.command_type = "gripper_close"
            fb.success = True
            fb.message = "command_dispatched"
            self._feedback_pub.publish(fb)
        return cmd_id


class MoveItManager:
    """Gerencia a interface com o MoveIt para planejamento de movimento."""
    
    def __init__(self, group_name="manipulator", end_effector_link="arm_link_fngr"):
        self.group_name = group_name
        self.end_effector_link = end_effector_link
        moveit_commander.roscpp_initialize([])
        self.group = moveit_commander.MoveGroupCommander(group_name)
        self.group.set_end_effector_link(end_effector_link)
        rospy.loginfo("MoveIt inicializado: End-effector (%s) em relação a: %s",
                     self.end_effector_link, self.group.get_pose_reference_frame())
                     
    def apply_wrist_lock(self):
        """Aplica restrições nas juntas específicas para travar o punho."""
        names = self.group.get_active_joints()
        vals = self.group.get_current_joint_values()
        
        # Trava a junta arm_wr0
        idx_wr0 = names.index("arm_wr0")
        locked_value_wr0 = vals[idx_wr0]
        rospy.loginfo("🔒 Travando arm_wr0 em %.3f rad", locked_value_wr0)
        jc_wr0 = JointConstraint(joint_name="arm_wr0",
                                 position=locked_value_wr0,
                                 tolerance_above=0.0,
                                 tolerance_below=0.0,
                                 weight=1.0)
        
        # Trava a junta arm_wr1
        idx_wr1 = names.index("arm_wr1")
        locked_value_wr1 = vals[idx_wr1]
        rospy.loginfo("🔒 Travando arm_wr1 em %.3f rad", locked_value_wr1)
        jc_wr1 = JointConstraint(joint_name="arm_wr1",
                                 position=locked_value_wr1,
                                 tolerance_above=0.0,
                                 tolerance_below=0.0,
                                 weight=1.0)
        
        # Aplica as restrições
        cs = Constraints()
        cs.joint_constraints.extend([jc_wr0, jc_wr1])
        self.group.set_path_constraints(cs)
        
    def get_current_pose(self):
        """Obtém a pose atual do end-effector."""
        rospy.sleep(0.1)  # Pequena pausa para garantir atualização
        return self.group.get_current_pose().pose


class TFManager:
    """Gerencia transformações de coordenadas usando TF."""
    
    def __init__(self):
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        
    def get_pose(self, target_frame="wrist", reference_frame="body", timeout=1.0):
        """Obtém a pose de um frame em relação a outro usando TF."""
        try:
            transform = self.tf_buffer.lookup_transform(
                reference_frame, target_frame, rospy.Time(0), rospy.Duration(timeout)
            )
            pose = PoseStamped()
            pose.header = transform.header
            pose.pose.position.x = transform.transform.translation.x
            pose.pose.position.y = transform.transform.translation.y
            pose.pose.position.z = transform.transform.translation.z
            pose.pose.orientation = transform.transform.rotation
            return pose.pose
        except Exception as e:
            rospy.logwarn(f"❗ TF lookup falhou: {e}")
            return None


class ObjectDetector:
    """Gerencia detecção de objetos usando YOLO."""
    
    def __init__(self, model_path, allowed_objects_csv):
        """
        Inicializa detector com modelo YOLO e CSV de objetos permitidos.
        
        Args:
            model_path: Caminho para o arquivo do modelo YOLO
            allowed_objects_csv: Caminho para CSV com objetos permitidos e prioridades
        """
        self.model = YOLO(model_path)
        self.allowed_objects = pd.read_csv(allowed_objects_csv)
        self.allowed_objects['class'] = self.allowed_objects['class'].str.strip().str.lower()
        
    def detect_objects(self, image):
        """Detecta objetos na imagem e retorna informações relevantes."""
        results = self.model(cv2.cvtColor(image, cv2.COLOR_BGR2RGB))
        boxes = results[0].boxes.xyxy.cpu().numpy()
        classes = results[0].boxes.cls.cpu().numpy().astype(int)
        names = [results[0].names[c].lower() for c in classes]
        
        return boxes, classes, names
        
    def filter_allowed_objects(self, boxes, names):
        """Filtra objetos detectados mantendo apenas os permitidos."""
        candidates = []
        
        for i, name in enumerate(names):
            df = self.allowed_objects[self.allowed_objects['class'] == name]
            if df.empty:
                continue
                
            priority = int(df['priority'].iloc[0])
            x1, y1, x2, y2 = boxes[i]
            area = (x2 - x1) * (y2 - y1)
            
            # Armazena (prioridade, -área, índice) para ordenação
            candidates.append((priority, -area, i))
            
        # Ordena por prioridade (menor é melhor) e depois por área (maior é melhor)
        candidates.sort()
        return candidates


class GraspStrategyBase(ABC):
    """Interface base para estratégias de grasp."""
    
    @abstractmethod
    def execute(self, robot_manager):
        """
        Executa a estratégia de grasp.
        
        Args:
            robot_manager: Gerenciador de clientes do robô
            
        Returns:
            bool: True se o grasp foi bem-sucedido, False caso contrário
        """
        pass


class YOLOGraspStrategy(GraspStrategyBase):
    """Estratégia de grasp baseada em detecção YOLO."""
    
    def __init__(self, detector, image_source="hand_color_image", cmd_pub=None, feedback_pub=None, yolo_pub=None):
        self.detector = detector
        self.image_source = image_source
        self.cmd_pub = cmd_pub
        self.feedback_pub = feedback_pub
        self.yolo_pub = yolo_pub
        
    def execute(self, robot_manager):
        """Executa grasp com base na detecção YOLO."""
        # Captura imagem do robô
        resp = robot_manager.image_client.get_image_from_sources([self.image_source])[0]
        arr = np.frombuffer(resp.shot.image.data, dtype=np.uint8)
        image = (arr.reshape(resp.shot.image.rows, resp.shot.image.cols) 
               if resp.shot.image.format == image_pb2.Image.FORMAT_RAW
               else cv2.imdecode(arr, -1))
            
        # Detecta e filtra objetos
        boxes, _, names = self.detector.detect_objects(image)
        candidates = self.detector.filter_allowed_objects(boxes, names)
        
        # Publica detecção YOLO para medição de latência
        if candidates and self.yolo_pub:
            yolo_msg = YoloDetection()
            yolo_msg.header.stamp = rospy.Time.now()
            yolo_msg.object_class = names[candidates[0][2]]  # Nome do objeto detectado
            self.yolo_pub.publish(yolo_msg)
        
        if not candidates:
            rospy.logwarn("[GRASP] Nenhum objeto permitido detectado na imagem.")
            return False
            
        # Seleciona o melhor candidato
        _, _, selected_idx = candidates[0]
        x1, y1, x2, y2 = boxes[selected_idx]
        cx, cy = int((x1 + x2) / 2), int((y1 + y2) / 2)
        
        # Executa grasp na posição determinada
        pick = manipulation_api_pb2.PickObjectInImage(
            pixel_xy=geometry_pb2.Vec2(x=cx, y=cy),
            transforms_snapshot_for_camera=resp.shot.transforms_snapshot,
            frame_name_image_sensor=resp.shot.frame_name_image_sensor,
            camera_model=resp.source.pinhole
        )
        
        req = manipulation_api_pb2.ManipulationApiRequest(pick_object_in_image=pick)
        
        # Publica comando para medição de latência
        if self.cmd_pub:
            cmd_msg = SpotCommand()
            cmd_msg.header.stamp = rospy.Time.now()
            cmd_msg.command_type = "manipulation_api"
            cmd_msg.has_pose = False
            self.cmd_pub.publish(cmd_msg)
        
        cmd_resp = robot_manager.manipulation_client.manipulation_api_command(req, timeout=5.0)
        
        # Monitora feedback
        result = self._monitor_grasp_feedback(robot_manager, cmd_resp)
        return result
        
    def _monitor_grasp_feedback(self, robot_manager, cmd_resp, poll_interval=0.2):
        """Monitora o feedback do comando de grasp."""
        while True:
            fb_req = manipulation_api_pb2.ManipulationApiFeedbackRequest(
                manipulation_cmd_id=cmd_resp.manipulation_cmd_id
            )
            fb = robot_manager.manipulation_client.manipulation_api_feedback_command(fb_req)
            state = manipulation_api_pb2.ManipulationFeedbackState.Name(fb.current_state)
            rospy.loginfo(f"[GRASP] Estado = {state}")
            
            if fb.current_state == manipulation_api_pb2.MANIP_STATE_GRASP_SUCCEEDED:
                # Publica feedback de sucesso para medição de latência
                if self.feedback_pub:
                    feedback_msg = SpotFeedback()
                    feedback_msg.header.stamp = rospy.Time.now()
                    feedback_msg.command_type = "manipulation_api"
                    feedback_msg.success = True
                    feedback_msg.message = "MANIP_STATE_GRASP_SUCCEEDED"
                    self.feedback_pub.publish(feedback_msg)
                return True
            elif fb.current_state == manipulation_api_pb2.MANIP_STATE_GRASP_FAILED:
                # Publica feedback de falha para medição de latência
                if self.feedback_pub:
                    feedback_msg = SpotFeedback()
                    feedback_msg.header.stamp = rospy.Time.now()
                    feedback_msg.command_type = "manipulation_api"
                    feedback_msg.success = False
                    feedback_msg.message = "MANIP_STATE_GRASP_FAILED"
                    self.feedback_pub.publish(feedback_msg)
                return False
                
            rospy.sleep(poll_interval)


class GraspManager:
    """Gerencia diferentes estratégias de grasp."""
    
    def __init__(self):
        self.strategies = {}
        self.last_used_strategy = None
        self.success_count = {}
        self.attempt_count = {}
        
    def register_strategy(self, name, strategy):
        """Registra uma nova estratégia de grasp."""
        self.strategies[name] = strategy
        self.success_count[name] = 0
        self.attempt_count[name] = 0
        
    def execute_grasp(self, strategy_name, robot_manager):
        """
        Executa a estratégia de grasp especificada.
        
        Args:
            strategy_name: Nome da estratégia a ser executada
            robot_manager: Gerenciador de clientes do robô
            
        Returns:
            bool: True se o grasp foi bem-sucedido, False caso contrário
        """
        if strategy_name not in self.strategies:
            rospy.logerr(f"Estratégia '{strategy_name}' não encontrada.")
            return False
            
        strategy = self.strategies[strategy_name]
        self.last_used_strategy = strategy_name
        self.attempt_count[strategy_name] += 1
        
        # Executa a estratégia e registra resultado
        result = strategy.execute(robot_manager)
        if result:
            self.success_count[strategy_name] += 1
            
        # Loga estatísticas de sucesso
        success_rate = self.success_count[strategy_name] / self.attempt_count[strategy_name] * 100
        rospy.loginfo(f"Estratégia '{strategy_name}' - Taxa de sucesso: {success_rate:.1f}% ({self.success_count[strategy_name]}/{self.attempt_count[strategy_name]})")
        
        return result
        
    def get_success_rates(self):
        """Retorna as taxas de sucesso de todas as estratégias registradas."""
        rates = {}
        for name in self.strategies:
            if self.attempt_count[name] > 0:
                rates[name] = self.success_count[name] / self.attempt_count[name]
            else:
                rates[name] = 0.0
        return rates


class FingerCountClient:
    """Calls /finger_count_node/get_finger_count until it gets 1 or 2."""
    def __init__(self, service_name="/finger_count_node/get_finger_count", finger_count_pub=None):
        rospy.wait_for_service(service_name)
        self._proxy = rospy.ServiceProxy(service_name, Trigger)
        self._finger_count_pub = finger_count_pub

    def request_mode(self):
        """Blocks until receiving 1 (manual) or 2 (semi-autonomous)."""
        while not rospy.is_shutdown():
            try:
                resp = self._proxy()
            except rospy.ServiceException as e:
                rospy.logwarn("FingerCount service failed: %s", e)
                rospy.sleep(0.5)
                continue

            if resp.success and resp.message in ("1", "2"):
                # Publica T3_finger_count para medição de latência
                if self._finger_count_pub:
                    from std_msgs.msg import Header
                    finger_msg = Header()
                    finger_msg.stamp = rospy.Time.now()
                    self._finger_count_pub.publish(finger_msg)
                return int(resp.message)

            rospy.loginfo("FingerCount returned '%s'. Retrying...", resp.message)
            rospy.sleep(0.3)


def compute_attractive(current: np.ndarray, target: np.ndarray, k_att: float = 0.2) -> np.ndarray:
    """Compute a *gentle* attractive force.

    Args:
        current: np.array([x, y, z]) – end‑effector in odom.
        target : np.array([x, y, z]) – object position in odom.
        k_att  : small gain.
    Returns:
        np.array – delta vector toward the target.
    """
    return k_att * (target - current)


class SpotController:
    """Controller with light potential field support in MANUAL mode."""

    def __init__(self, spot_hostname="192.168.80.3",
                 model_path="/root/ws_moveit/src/spot_operation/scripts/yolo11n.pt",
                 allowed_objects_csv="/root/ws_moveit/src/spot_operation/config/allowed_objects.csv"):
        # Inicializa ROS
        rospy.init_node("continuous_moveit_pose_to_spot_real", anonymous=True)
        
        # Publishers para medição de latência
        self.yolo_pub = rospy.Publisher('/yolo/detection', YoloDetection, queue_size=1)
        self.cmd_pub = rospy.Publisher('/spot/cmd_sent', SpotCommand, queue_size=1)
        self.feedback_pub = rospy.Publisher('/spot/exec_done', SpotFeedback, queue_size=1)
        # Publisher para T3 finger count (usando Header como mensagem simples)
        from std_msgs.msg import Header
        self.finger_count_pub = rospy.Publisher('/finger_count_result', Header, queue_size=1)
        
        # Configuração de gestos
        self.current_gesture = 0
        self.manipulation_mode = False
        rospy.Subscriber("/hand_gesture", Int32, self._gesture_callback)
        
        # Inicializa gerenciadores
        self.tf_manager = TFManager()
        self.moveit_manager = MoveItManager()
        
        # Aplica restrições de juntas
        self.moveit_manager.apply_wrist_lock()
            
        # Conecta ao Spot
        self.robot_manager = RobotClientManager(
            hostname=spot_hostname,
            cmd_pub=self.cmd_pub,
            feedback_pub=self.feedback_pub
        )
        
        if not self.robot_manager.connect():
            rospy.logerr("Falha ao conectar ao Spot. Abortando.")
            return
            
        # Inicializa detector de objetos
        self.object_detector = ObjectDetector(
            model_path=model_path,
            allowed_objects_csv=allowed_objects_csv
        )
        
        # Inicializa gerenciador de grasp
        self.grasp_manager = GraspManager()
        
        # Registra múltiplas estratégias de grasp com diferentes fontes de imagem
        # Câmera principal (frontal)
        front_grasp = YOLOGraspStrategy(
            detector=self.object_detector,
            image_source="hand_color_image",
            cmd_pub=self.cmd_pub,
            feedback_pub=self.feedback_pub,
            yolo_pub=self.yolo_pub
        )
        self.grasp_manager.register_strategy("front_yolo", front_grasp)
        
        # Câmera lateral (se disponível)
        side_grasp = YOLOGraspStrategy(
            detector=self.object_detector,
            image_source="frontleft_fisheye_image",
            cmd_pub=self.cmd_pub,
            feedback_pub=self.feedback_pub,
            yolo_pub=self.yolo_pub
        )
        self.grasp_manager.register_strategy("side_yolo", side_grasp)
        
        # Estratégia padrão
        self.default_grasp_strategy = "front_yolo"
        # Estratégia fallback (para usar quando a principal falhar)
        self.fallback_grasp_strategy = "side_yolo"

        # === Select operation mode using the service ===
        finger_client = FingerCountClient(finger_count_pub=self.finger_count_pub)
        mode_code = finger_client.request_mode()  # 1 or 2
        self.mode = "manual" if mode_code == 1 else "semi"
        rospy.loginfo("🚀 Operation mode selected: %s", self.mode.upper())

        # === PF state ===
        self._pf_target_odom = None  # np.array([x, y, z])
        self._last_detection_time = 0.0
        self._target_valid_duration = 3.0  # Keep target valid for 3 seconds
        
    def _gesture_callback(self, msg):
        """Callback para mensagens de gestos."""
        self.current_gesture = msg.data
        
    def run(self):
        """Executa o loop principal de controle."""
        # Adquire lease
        with self.robot_manager.acquire_lease():
            # Liga o robô
            self.robot_manager.power_on()
            rospy.loginfo("Robô ligado. Iniciando sincronização contínua...")
            
            # Loop principal
            self._control_loop()
    
    def _enter_manipulation_mode(self):
        """Entra no modo de manipulação após um grasp bem-sucedido."""
        self.manipulation_mode = True
        rospy.loginfo("🤖 Entrando no modo de MANIPULAÇÃO. Aguardando gesto '0' para liberar objeto...")
    
    def _exit_manipulation_mode(self):
        """Sai do modo de manipulação e retorna ao modo normal."""
        # Abre a garra por 3 segundos
        rospy.loginfo("Liberando objeto: Abrindo garra por 3 segundos...")
        self.robot_manager.open_gripper()
        rospy.sleep(3.0)
        
        # Fecha a garra novamente com bloqueio
        rospy.loginfo("Fechando garra...")
        self.robot_manager.close_gripper(block=True)
        
        # Sai do modo de manipulação
        self.manipulation_mode = False
        rospy.loginfo("🤖 Saindo do modo de MANIPULAÇÃO. Retornando ao modo normal.")


    def _control_loop(self):
        """Main control loop."""
        # Ensure safety variables are initialized
        self.frozen_orientation = None
        self.is_orientation_locked = False

        rate = rospy.Rate(5)  # 5 Hz gives smoother PF updates

        while not rospy.is_shutdown():
            now = time.time()
            # -------------- MANUAL (MODE 1) --------------------------------
            if self.mode == "manual":
                # Update target every 1 second to save bandwidth
                if now - self._last_detection_time > 1.0:
                    new_target = self._yolo_depth_deproject()
                    if new_target is not None:
                        self._pf_target_odom = new_target
                        self._last_detection_time = now
                    elif now - self._last_detection_time > self._target_valid_duration:
                        self._pf_target_odom = None  # Clear target if too old

                # Gripper command based on gesture
                if self.current_gesture == 0:
                    self.robot_manager.open_gripper()
                else:
                    self.robot_manager.close_gripper()

                sim_pose = self.tf_manager.get_pose()
                if sim_pose is None:
                    rate.sleep()
                    continue

                # --------------- build current pose --------------------
                current_pos = np.array([sim_pose.position.x,
                                         sim_pose.position.y,
                                         sim_pose.position.z])
                target_pos = self._pf_target_odom

                use_pf = (target_pos is not None and self.current_gesture == 0)

                if use_pf:
                    dist = np.linalg.norm(target_pos - current_pos)
                    # atrai só se estiver MAIS PERTO que 40 cm
                    if dist < 0.40:
                        delta = compute_attractive(current_pos, target_pos, k_att=0.2)
                        new_pos = current_pos + delta
                    else:
                        new_pos = current_pos
                else:
                    new_pos = current_pos

                # --------------- send command -------------------------
                robot_state = self.robot_manager.get_robot_state()
                odom_T_body = get_a_tform_b(robot_state.kinematic_state.transforms_snapshot,
                                            ODOM_FRAME_NAME, "body")
                quat_to_use = (self.frozen_orientation if self.is_orientation_locked
                               else sim_pose.orientation)
                flat_body_T_hand = SE3Pose(x=new_pos[0], y=new_pos[1], z=new_pos[2],
                                           rot=Quat(w=quat_to_use.w,
                                                    x=quat_to_use.x,
                                                    y=quat_to_use.y,
                                                    z=quat_to_use.z))
                odom_T_hand = odom_T_body * flat_body_T_hand
                self.robot_manager.send_arm_command(odom_T_hand)

                self._show_debug_overlay(use_pf)  # Show debug visualization

                rate.sleep()
                continue  # restart loop

            # ----------------- (Rest of original _control_loop unchanged) ----
            # Verifica se estamos no modo de manipulação
            if self.manipulation_mode:
                # Verifica se recebemos o gesto para liberar o objeto
                if self.current_gesture == 0:
                    self._exit_manipulation_mode()
                else:
                    # Continua no modo de manipulação
                    # Obtém pose da simulação
                    sim_pose = self.tf_manager.get_pose()
                    if sim_pose is not None:
                        # Obtém transformação do corpo para a mão
                        robot_state = self.robot_manager.get_robot_state()
                        odom_T_body = get_a_tform_b(
                            robot_state.kinematic_state.transforms_snapshot,
                            ODOM_FRAME_NAME, "body"
                        )
                        
                        # Aplica transformação da simulação
                        quat_to_use = (self.frozen_orientation if self.is_orientation_locked
                                       else sim_pose.orientation)
                        flat_body_T_hand = SE3Pose(
                            x=sim_pose.position.x,
                            y=sim_pose.position.y,
                            z=sim_pose.position.z,
                            rot=Quat(
                                w=quat_to_use.w,
                                x=quat_to_use.x,
                                y=quat_to_use.y,
                                z=quat_to_use.z
                            )
                        )
                        
                        # Calcula pose final
                        odom_T_hand = odom_T_body * flat_body_T_hand
                        
                        # Envia comando para o braço
                        self.robot_manager.send_arm_command(odom_T_hand)
                        
                        rospy.loginfo("Modo MANIPULAÇÃO: Comando enviado: Pose alvo = [%.3f, %.3f, %.3f]",
                                     odom_T_hand.x, odom_T_hand.y, odom_T_hand.z)
                
                rate.sleep()
                continue
            
            # Modo normal (não está no modo de manipulação)
            # Verifica gestos para iniciar grasp
            if self.current_gesture == 1:
                rospy.loginfo("✋ Gesto de agarrar detectado!")
                
                # Congela orientação atual antes de fechar a garra
                sim_pose = self.tf_manager.get_pose()
                if sim_pose:
                    self.frozen_orientation = sim_pose.orientation
                    self.is_orientation_locked = True
                    rospy.loginfo("🔒 Orientação congelada durante fechamento da garra")

                # Fecha a garra (manda o comando)

                # Fecha a garra e espera resposta
                self.robot_manager.close_gripper(block=True, timeout_sec=2.0)

                rospy.loginfo("✅ Garra fechada. Voltando a liberar orientação")
                self.is_orientation_locked = False

                # Tenta executar a estratégia primária
                result = self.grasp_manager.execute_grasp(self.default_grasp_strategy, self.robot_manager)
                
                # Se falhar, tenta o fallback
                if not result:
                    rospy.logwarn("Grasp primário falhou. Tentando estratégia alternativa...")
                    result = self.grasp_manager.execute_grasp(self.fallback_grasp_strategy, self.robot_manager)
                    if not result:
                        rospy.logerr("Todas as estratégias de grasp falharam.")
                    else:
                        rospy.loginfo("Grasp alternativo concluído com sucesso!")
                        # Entra no modo de manipulação após grasp bem-sucedido
                        self._enter_manipulation_mode()
                else:
                    rospy.loginfo("Grasp primário concluído com sucesso!")
                    # Entra no modo de manipulação após grasp bem-sucedido
                    self._enter_manipulation_mode()
                
                # Reseta o gesto apenas se não entramos no modo de manipulação
                if not self.manipulation_mode:
                    self.current_gesture = 0
                    
                rospy.sleep(1.0)
                continue
                
            # Obtém pose da simulação
            sim_pose = self.tf_manager.get_pose()
            if sim_pose is None:
                rospy.logwarn("⚠️ Não foi possível obter a pose do wrist.")
                rate.sleep()
                continue
                
            # Obtém transformação do corpo para a mão
            robot_state = self.robot_manager.get_robot_state()
            odom_T_body = get_a_tform_b(
                robot_state.kinematic_state.transforms_snapshot,
                ODOM_FRAME_NAME, "body"
            )
            
            # Aplica transformação da simulação
            quat_to_use = (self.frozen_orientation if self.is_orientation_locked
                           else sim_pose.orientation)
            flat_body_T_hand = SE3Pose(
                x=sim_pose.position.x,
                y=sim_pose.position.y,
                z=sim_pose.position.z,
                rot=Quat(
                    w=1.0,
                    x=0.0,
                    y=0.0,
                    z=0.0
                )
            )
            
            # Calcula pose final
            odom_T_hand = odom_T_body * flat_body_T_hand
            
            # Envia comando para o braço
            self.robot_manager.send_arm_command(odom_T_hand)
            
            rospy.loginfo("Comando enviado: Pose alvo = [%.3f, %.3f, %.3f]",
                         odom_T_hand.x, odom_T_hand.y, odom_T_hand.z)
                         
            rate.sleep()

    def _yolo_depth_deproject(self):
        """Detecta o objeto em 'hand_color_image' e retorna as coordenadas 3D no quadro de coordenadas do robô."""
        try:
            rgb_resp, depth_resp = self.robot_manager.image_client.get_image_from_sources(
                ["hand_color_image", "hand_depth_in_hand_color_frame"])
        except Exception as exc:
            rospy.logwarn_throttle(2.0, "Falha ao obter imagem: %s", exc)
            return None

        # --- Decodifica a imagem RGB ---
        rgb_arr = np.frombuffer(rgb_resp.shot.image.data, dtype=np.uint8)
        if rgb_resp.shot.image.format == image_pb2.Image.FORMAT_RAW:
            rgb_img = rgb_arr.reshape(rgb_resp.shot.image.rows, rgb_resp.shot.image.cols, 1)
            rgb_img = cv2.cvtColor(rgb_img, cv2.COLOR_GRAY2BGR)
        else:
            rgb_img = cv2.imdecode(rgb_arr, cv2.IMREAD_COLOR)
        self._debug_img = rgb_img.copy()

        # --- Decodifica a profundidade logo em seguida ---
        depth_np = np.frombuffer(depth_resp.shot.image.data, dtype=np.uint16).reshape(
            depth_resp.shot.image.rows, depth_resp.shot.image.cols)
        depth_rows, depth_cols = depth_np.shape

        # --- Obtém a orientação da garra ---
        sim_pose = self.tf_manager.get_pose()
        if sim_pose is None:
            rospy.logwarn("Falha ao obter a pose da garra.")
            return None

        # --- Calcula o ângulo de rotação completo a partir do quaternion ---
        quat = sim_pose.orientation
        # Extrai os ângulos de Euler (roll, pitch, yaw) do quaternion
        # Fórmula para converter quaternion para ângulos de Euler (em radianos)
        # Roll (rotação em torno do eixo X)
        sinr_cosp = 2 * (quat.w * quat.x + quat.y * quat.z)
        cosr_cosp = 1 - 2 * (quat.x * quat.x + quat.y * quat.y)
        roll = math.atan2(sinr_cosp, cosr_cosp)

        # Pitch (rotação em torno do eixo Y)
        sinp = 2 * (quat.w * quat.y - quat.z * quat.x)
        if abs(sinp) >= 1:
            pitch = math.copysign(math.pi / 2, sinp)  # Usa 90 graus se sinp for +/- 1
        else:
            pitch = math.asin(sinp)

        # Yaw (rotação em torno do eixo Z)
        siny_cosp = 2 * (quat.w * quat.z + quat.x * quat.y)
        cosy_cosp = 1 - 2 * (quat.y * quat.y + quat.z * quat.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        # Convertendo para graus
        roll_deg = roll * 180.0 / math.pi
        pitch_deg = pitch * 180.0 / math.pi
        yaw_deg = yaw * 180.0 / math.pi

        # No caso da câmera da garra, geralmente uma combinação das rotações é mais útil
        # Dependendo da orientação da câmera em relação à garra
        # Vamos experimentar usar o ângulo combinado que melhor representa a rotação visual
        angle_deg = yaw_deg  # Começamos com yaw, que é a rotação em Z

        # Se a câmera estiver montada de forma que o eixo principal aponte no sentido do X ou Y do robô,
        # podemos precisar usar roll ou pitch em vez de yaw
        if abs(roll_deg) > abs(yaw_deg) and abs(roll_deg) > abs(pitch_deg):
            angle_deg = roll_deg
        elif abs(pitch_deg) > abs(yaw_deg) and abs(pitch_deg) > abs(roll_deg):
            angle_deg = pitch_deg

        # Log para debug
        rospy.loginfo(f"Ângulos de Euler: roll={roll_deg:.2f}°, pitch={pitch_deg:.2f}°, yaw={yaw_deg:.2f}°")
        rospy.loginfo(f"Usando ângulo de rotação: {angle_deg:.2f}°")

        # --- Rotação da imagem usando abordagem mais robusta ---
        h, w = rgb_img.shape[:2]
        center = (w // 2, h // 2)
        rotation_matrix_2d = cv2.getRotationMatrix2D(center, -angle_deg, 1.0)
        aligned_image = cv2.warpAffine(rgb_img, rotation_matrix_2d, (w, h), 
                                    flags=cv2.INTER_LINEAR, 
                                    borderMode=cv2.BORDER_CONSTANT,
                                    borderValue=(0, 0, 0))
        
        # --- Salva a imagem rotacionada para visualização de debug ---
        self._debug_img_rotated = aligned_image.copy()

        # --- Detecta e filtra objetos na imagem rotacionada ---
        boxes, _, names = self.object_detector.detect_objects(aligned_image)
        candidates = self.object_detector.filter_allowed_objects(boxes, names)
        
        if not candidates:
            self._debug_box = None
            self._debug_box_original = None
            self._debug_dist_m = None
            return None

        # --- Seleciona o melhor candidato ---
        _, _, idx = candidates[0]
        x1, y1, x2, y2 = boxes[idx]
        cx, cy = int((x1 + x2) / 2), int((y1 + y2) / 2)
        
        # --- Armazena o box para visualização na imagem rotacionada ---
        self._debug_box_rotated = (int(x1), int(y1), int(x2), int(y2))
        
        # --- Projeta o box de volta para a imagem original ---
        h, w = aligned_image.shape[:2]
        center = (w // 2, h // 2)

        # Matriz de rotação inversa (usando o negativo do ângulo)
        inv_rotation_matrix_2d = cv2.getRotationMatrix2D(center, angle_deg, 1.0)

        # Pontos a serem transformados: cantos e centro do objeto
        points = np.array([[x1, y1], [x2, y2], [cx, cy]], dtype=np.float32).reshape(-1, 1, 2)

        # Aplica a transformação inversa
        transformed_points = cv2.transform(points, inv_rotation_matrix_2d)

        # Extrai os pontos transformados
        x1_orig, y1_orig = transformed_points[0][0]
        x2_orig, y2_orig = transformed_points[1][0]
        cx_orig, cy_orig = transformed_points[2][0]

        # Converte para inteiros e garante que estão dentro dos limites da imagem
        depth_rows, depth_cols = depth_np.shape
        cx_orig_int = min(max(0, int(cx_orig)), depth_cols - 1)
        cy_orig_int = min(max(0, int(cy_orig)), depth_rows - 1)

        # Armazena o box para visualização na imagem original
        self._debug_box = (int(x1_orig), int(y1_orig), int(x2_orig), int(y2_orig))
        self._debug_center_orig = (int(cx_orig), int(cy_orig))

        # --- Alinhamento da profundidade (usando coordenadas na imagem original) ---
        depth_np = np.frombuffer(depth_resp.shot.image.data, dtype=np.uint16).reshape(
            depth_resp.shot.image.rows, depth_resp.shot.image.cols)

        # Converte para inteiros e garante que estão dentro dos limites da imagem
        depth_rows, depth_cols = depth_np.shape
        x1_orig_safe = min(max(0, int(x1_orig)), depth_cols - 1)
        y1_orig_safe = min(max(0, int(y1_orig)), depth_rows - 1)
        x2_orig_safe = min(max(0, int(x2_orig)), depth_cols - 1)
        y2_orig_safe = min(max(0, int(y2_orig)), depth_rows - 1)
        cx_orig_safe = min(max(0, int(cx_orig)), depth_cols - 1)
        cy_orig_safe = min(max(0, int(cy_orig)), depth_rows - 1)

        # Armazena o box para visualização na imagem original
        self._debug_box = (x1_orig_safe, y1_orig_safe, x2_orig_safe, y2_orig_safe)
        self._debug_center_orig = (cx_orig_safe, cy_orig_safe)

        # Usa o centro do objeto na imagem original para obter a profundidade
        raw_mm = int(depth_np[cy_orig_safe, cx_orig_safe])

        min_depth_mm = 300   # 30 cm
        max_depth_mm = 3000  # 3 m

        depth_mm = raw_mm if (min_depth_mm <= raw_mm <= max_depth_mm) else min_depth_mm
        depth_m = depth_mm / 1000.0

        self._debug_dist_m = depth_m  # Salva a distância em metros

        # --- De-projeção do pixel -> quadro de câmera (usando coordenadas na imagem original) ---
        pinhole = depth_resp.source.pinhole
        fx = pinhole.intrinsics.focal_length.x
        fy = pinhole.intrinsics.focal_length.y
        cx0 = pinhole.intrinsics.principal_point.x
        cy0 = pinhole.intrinsics.principal_point.y
        x_cam = (cx_orig - cx0) * depth_m / fx
        y_cam = (cy_orig - cy0) * depth_m / fy
        z_cam = depth_m

        # --- Quadro de câmera → quadro de odom ---
        try:
            odom_T_cam = get_a_tform_b(depth_resp.shot.transforms_snapshot,
                                     ODOM_FRAME_NAME,
                                     depth_resp.shot.frame_name_image_sensor)
        except Exception as exc:
            rospy.logwarn_throttle(2.0, "Falha ao obter a transformação cam→odom: %s", exc)
            return None
        cam_T_obj = SE3Pose(x_cam, y_cam, z_cam, Quat())
        odom_T_obj = odom_T_cam * cam_T_obj
        return np.array([odom_T_obj.x, odom_T_obj.y, odom_T_obj.z])

    def _show_debug_overlay(self, pf_active: bool):
        """Displays image with bounding box, distance, and PF status."""
        if getattr(self, "_debug_img", None) is None:
            return  # Nothing to display

        # Cria visualização para imagem original
        vis_orig = self._debug_img.copy()
        if self._debug_box:
            x1, y1, x2, y2 = self._debug_box
            color = (0, 255, 0) if pf_active else (0, 0, 255)
            cv2.rectangle(vis_orig, (x1, y1), (x2, y2), color, 2)
            
            # Desenha o centro do objeto
            if hasattr(self, "_debug_center_orig"):
                cx, cy = self._debug_center_orig
                cv2.circle(vis_orig, (cx, cy), 4, color, -1)
                
            if self._debug_dist_m:
                cv2.putText(vis_orig, f"{self._debug_dist_m:.2f} m",
                          (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX,
                          0.6, color, 2)

        # Adiciona indicação da orientação da garra (como uma seta)
        sim_pose = self.tf_manager.get_pose()
        if sim_pose is not None:
            h, w = vis_orig.shape[:2]
            center_x, center_y = w // 2, h // 2
            arrow_length = 50
            quat = sim_pose.orientation
            # Cria vetor de direção simplificado a partir do quaternion
            # Este é um cálculo simplificado para visualização
            dx = 2 * (quat.x * quat.z + quat.w * quat.y)
            dy = 2 * (quat.y * quat.z - quat.w * quat.x)
            dz = 1 - 2 * (quat.x * quat.x + quat.y * quat.y)
            
            # Desenha seta de orientação
            end_x = int(center_x + arrow_length * dx)
            end_y = int(center_y + arrow_length * dy)
            cv2.arrowedLine(vis_orig, (center_x, center_y), (end_x, end_y), (255, 0, 0), 2)

        # Display PF status na imagem original
        txt = "PF: ON" if pf_active else "PF: OFF"
        cv2.putText(vis_orig, txt, (10, 30), cv2.FONT_HERSHEY_SIMPLEX,
                    0.8, (255, 255, 0), 2)
        cv2.putText(vis_orig, "ORIGINAL", (10, 60), cv2.FONT_HERSHEY_SIMPLEX,
                    0.8, (255, 255, 0), 2)

        # Cria visualização para imagem rotacionada
        if hasattr(self, "_debug_img_rotated"):
            vis_rot = self._debug_img_rotated.copy()
            if hasattr(self, "_debug_box_rotated"):
                x1, y1, x2, y2 = self._debug_box_rotated
                color = (0, 255, 0) if pf_active else (0, 0, 255)
                cv2.rectangle(vis_rot, (x1, y1), (x2, y2), color, 2)
                cx, cy = (x1 + x2) // 2, (y1 + y2) // 2
                cv2.circle(vis_rot, (cx, cy), 4, color, -1)
                
            # Adiciona texto para identificar a imagem rotacionada
            cv2.putText(vis_rot, "ROTACIONADA", (10, 60), cv2.FONT_HERSHEY_SIMPLEX,
                        0.8, (255, 255, 0), 2)
                
            # Combina as duas imagens lado a lado
            h1, w1 = vis_orig.shape[:2]
            h2, w2 = vis_rot.shape[:2]
            h = max(h1, h2)
            w = w1 + w2
            combined = np.zeros((h, w, 3), dtype=np.uint8)
            combined[:h1, :w1] = vis_orig
            combined[:h2, w1:w1+w2] = vis_rot
                
            cv2.imshow("debug_view", combined)
        else:
            # Se a imagem rotacionada não estiver disponível, mostre apenas a original
            cv2.imshow("debug_view", vis_orig)
            
        cv2.waitKey(1)

def get_rotation_matrix_from_quat(quat):
    """Converte o quaternion da orientação da garra para uma matriz de rotação 3x3."""
    q = np.array([quat.x, quat.y, quat.z, quat.w])
    norm_q = np.dot(q, q)
    if norm_q < np.finfo(float).eps:
        return np.eye(3)
    q *= math.sqrt(2.0 / norm_q)
    q_outer = np.outer(q, q)
    return np.array([
        [1.0 - q_outer[1, 1] - q_outer[2, 2], q_outer[0, 1] - q[2], q_outer[0, 2] + q[1]],
        [q_outer[0, 1] + q[2], 1.0 - q_outer[0, 0] - q_outer[2, 2], q_outer[1, 2] - q[0]],
        [q_outer[0, 2] - q[1], q_outer[1, 2] + q[0], 1.0 - q_outer[0, 0] - q_outer[1, 1]]
    ])

def rotate_image(image, rotation_matrix):
    """Aplica a rotação à imagem usando a matriz de rotação."""
    h, w = image.shape[:2]
    center = (w // 2, h // 2)
    rotation_matrix_2d = cv2.getRotationMatrix2D(center, 0, 1.0)
    rotation_matrix_2d[:2, :2] = rotation_matrix[:2, :2]
    return cv2.warpAffine(image, rotation_matrix_2d, (w, h))


def main():
    """Função principal."""
    # Inicializa e executa o controlador
    controller = SpotController()
    controller.run()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass