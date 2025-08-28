#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy

from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from moveit_msgs.srv import GetPositionIK
from moveit_msgs.msg import RobotState
from control_msgs.msg import JointTrajectoryControllerState

import tf2_ros
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException


class SpotTeleopNode(Node):
    def __init__(self):
        super().__init__('spot_teleop_node')
        self.callback_group = ReentrantCallbackGroup()

        # ---------------------- Params ----------------------
        self.base_frame = self.declare_parameter('base_frame', 'body').get_parameter_value().string_value
        self.target_frame = self.declare_parameter('target_frame', 'wrist').get_parameter_value().string_value  # legacy
        self.alpha = self.declare_parameter('lp_alpha', 0.4).get_parameter_value().double_value
        self.publish_period = self.declare_parameter('publish_period', 0.005).get_parameter_value().double_value  # 200 Hz
        self.target_pose_topic = self.declare_parameter('target_pose_topic', '/wrist_target').get_parameter_value().string_value

        # ------------------------ TF ------------------------
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # ---------------------- State -----------------------
        self.current_joints: JointState | None = None
        self.controller_joint_names: list[str] | None = None
        self.last_pose = None  # Pose (smoothed)
        self.warned_missing_ctrl_names = False
        self.frames_warn_countdown = 0
        self.latest_target_pose: PoseStamped | None = None
        self.last_valid_pose_in_base: PoseStamped | None = None
        self.ik_inflight = False
        self.request_pending = False
        self.last_success_positions = None
        self.ik_seq = 0
        self.latest_ik_seq_requested = -1

        # Sub: robot joint states
        self.joint_sub = self.create_subscription(
            JointState, '/joint_states', self.joint_callback, 50,
            callback_group=self.callback_group
        )

        # Sub: controller_state
        self.ctrl_state_sub = self.create_subscription(
            JointTrajectoryControllerState,
            '/arm_controller/controller_state',
            self.ctrl_state_callback,
            10,
            callback_group=self.callback_group,
        )

        # Sub: PoseStamped alvo do punho (QoS keep-last=1 para teleop responsivo)
        pose_qos = QoSProfile(depth=1)
        pose_qos.history = QoSHistoryPolicy.KEEP_LAST
        pose_qos.reliability = QoSReliabilityPolicy.RELIABLE
        self.pose_sub = self.create_subscription(
            PoseStamped,
            self.target_pose_topic,
            self.pose_callback,
            pose_qos,
            callback_group=self.callback_group,
        )

        # Pub: joint trajectory to controller
        self.traj_pub = self.create_publisher(JointTrajectory, '/arm_controller/joint_trajectory', 10)

        # Client: MoveIt IK service
        self.ik_client = self.create_client(GetPositionIK, '/compute_ik', callback_group=self.callback_group)
        while not self.ik_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Aguardando service /compute_ik...')

        # Timer para teleop contínuo em alta taxa
        self.timer = self.create_timer(self.publish_period, self.teleop_timer_callback, callback_group=self.callback_group)
        self.get_logger().info(f'Teleop real-time: IK contínuo (periodo={self.publish_period:.3f}s), seguindo "{self.target_pose_topic}" (base: {self.base_frame}).')

    # ------------------- Callbacks -------------------
    def joint_callback(self, msg: JointState):
        self.current_joints = msg

    def ctrl_state_callback(self, msg: JointTrajectoryControllerState):
        if not self.controller_joint_names:
            self.controller_joint_names = list(msg.joint_names)
            self.get_logger().info(f"Controller joints detectados (ordem): {self.controller_joint_names}")

    def pose_callback(self, msg: PoseStamped):
        self.latest_target_pose = msg

    # ------------------- Helpers ---------------------
    def smooth_pose(self, current_pose):
        if self.last_pose is None:
            self.last_pose = current_pose
        else:
            self.last_pose.position.x = self.alpha * self.last_pose.position.x + (1 - self.alpha) * current_pose.position.x
            self.last_pose.position.y = self.alpha * self.last_pose.position.y + (1 - self.alpha) * current_pose.position.y
            self.last_pose.position.z = self.alpha * self.last_pose.position.z + (1 - self.alpha) * current_pose.position.z
            self.last_pose.orientation = current_pose.orientation
        return self.last_pose

    def get_joint_map(self, joint_state: JointState) -> dict:
        return {n: p for n, p in zip(joint_state.name, joint_state.position)}

    def current_positions_ordered(self):
        if not (self.current_joints and self.controller_joint_names):
            return None
        cur_map = self.get_joint_map(self.current_joints)
        out = []
        for jn in self.controller_joint_names:
            if jn in cur_map:
                out.append(cur_map[jn])
            else:
                return None
        return out

    def build_ordered_trajectory(self, ik_names: list[str], ik_positions: list[float]):
        assert self.controller_joint_names, 'controller_joint_names não inicializado'
        ik_map = {n: p for n, p in zip(ik_names, ik_positions)}
        cur_map = self.get_joint_map(self.current_joints) if self.current_joints else {}

        out_names = []
        out_positions = []
        for jn in self.controller_joint_names:
            if jn in ik_map:
                out_names.append(jn)
                out_positions.append(ik_map[jn])
            elif jn in cur_map:
                out_names.append(jn)
                out_positions.append(cur_map[jn])
            else:
                self.get_logger().error(f'Controller exige junta "{jn}", mas ela não está no IK e também não apareceu em /joint_states.')
                return [], []
        return out_names, out_positions

    def filtered_arm_robot_state(self) -> RobotState:
        rs = RobotState()
        if not (self.current_joints and self.controller_joint_names):
            return rs
        cur_map = self.get_joint_map(self.current_joints)
        names = []
        pos = []
        for jn in self.controller_joint_names:
            if jn in cur_map:
                names.append(jn)
                pos.append(cur_map[jn])
        js = JointState()
        js.name = names
        js.position = pos
        rs.joint_state = js
        return rs

    # ------------------- Main loop -------------------
    def teleop_timer_callback(self):
        # Sempre envie comandos para o controller na última solução conhecida
        if self.controller_joint_names:
            positions = self.last_success_positions or self.current_positions_ordered()
            if positions:
                traj = JointTrajectory()
                traj.joint_names = self.controller_joint_names
                point = JointTrajectoryPoint()
                point.positions = positions
                point.time_from_start = Duration(seconds=0, nanoseconds=50_000_000).to_msg()  # 50 ms
                traj.points = [point]
                traj.header.stamp = self.get_clock().now().to_msg()
                self.traj_pub.publish(traj)

        if self.current_joints is None or not self.controller_joint_names:
            return

        if not self.ik_inflight:
            self.try_solve_ik()
        else:
            self.request_pending = True

    # ------------------- IK pipeline -------------------
    def try_solve_ik(self):
        # Escolhe a última Pose recebida; se não houver, usa a última válida
        src_pose = self.latest_target_pose or self.last_valid_pose_in_base
        if src_pose is None:
            return

        # Transformar para base se necessário (timeout curto)
        try:
            if src_pose.header.frame_id and src_pose.header.frame_id != self.base_frame:
                pose_in_base: PoseStamped = self.tf_buffer.transform(
                    src_pose,
                    self.base_frame,
                    timeout=Duration(seconds=0, nanoseconds=20_000_000)  # 20 ms
                )
            else:
                pose_in_base = src_pose
        except (LookupException, ConnectivityException, ExtrapolationException):
            if self.frames_warn_countdown <= 0:
                frames = self.tf_buffer.all_frames_as_string()
                self.get_logger().warn(
                    f"Não foi possível transformar Pose de '{src_pose.header.frame_id}' para '{self.base_frame}'. Frames disponíveis:\n{frames}"
                )
                self.frames_warn_countdown = 10
            else:
                self.frames_warn_countdown -= 1
            return

        # Guarda última pose válida em base e suaviza
        self.last_valid_pose_in_base = pose_in_base
        smooth_pose = self.smooth_pose(pose_in_base.pose)

        # Monta requisição IK
        target_pose = PoseStamped()
        target_pose.header.frame_id = self.base_frame
        target_pose.header.stamp = self.get_clock().now().to_msg()
        target_pose.pose = smooth_pose

        req = GetPositionIK.Request()
        req.ik_request.group_name = 'arm'
        req.ik_request.pose_stamped = target_pose
        req.ik_request.avoid_collisions = False
        req.ik_request.timeout = Duration(seconds=0, nanoseconds=60_000_000).to_msg()  # 60 ms
        req.ik_request.robot_state = self.filtered_arm_robot_state()

        self.ik_inflight = True
        self.request_pending = False
        self.ik_seq += 1
        seq = self.ik_seq
        self.latest_ik_seq_requested = seq
        future = self.ik_client.call_async(req)
        future.add_done_callback(lambda f, seq=seq: self._on_ik_done(f, seq))

    def _on_ik_done(self, future, seq):
        # Descarte respostas antigas se um novo pedido já foi enviado
        if seq != self.latest_ik_seq_requested and self.request_pending:
            self.ik_inflight = False
            self.try_solve_ik()
            return

        try:
            resp = future.result()
        except Exception as e:
            self.get_logger().warn(f'IK exception: {e}')
            self.ik_inflight = False
            if self.request_pending:
                self.try_solve_ik()
            return

        if not resp:
            self.ik_inflight = False
            if self.request_pending:
                self.try_solve_ik()
            return

        if resp.error_code.val != resp.error_code.SUCCESS:
            self.ik_inflight = False
            if self.request_pending:
                self.try_solve_ik()
            return

        ordered_names, ordered_positions = self.build_ordered_trajectory(
            list(resp.solution.joint_state.name),
            list(resp.solution.joint_state.position)
        )
        if ordered_names:
            traj = JointTrajectory()
            traj.joint_names = ordered_names
            point = JointTrajectoryPoint()
            point.positions = ordered_positions
            point.time_from_start = Duration(seconds=0, nanoseconds=50_000_000).to_msg()  # 50 ms
            traj.points = [point]
            traj.header.stamp = self.get_clock().now().to_msg()
            self.traj_pub.publish(traj)
            self.last_success_positions = ordered_positions

        self.ik_inflight = False
        if self.request_pending:
            self.try_solve_ik()


def main(args=None):
    rclpy.init(args=args)
    node = SpotTeleopNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
