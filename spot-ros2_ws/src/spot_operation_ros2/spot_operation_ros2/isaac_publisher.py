#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from control_msgs.msg import JointTrajectoryControllerState
from spot_msgs.msg import JointCommand


class JointStateRelay(Node):
    def __init__(self):
        super().__init__("joint_state_relay")

        # Publisher para joint_command_isaac
        self.pub = self.create_publisher(JointState, "joint_command_isaac", 10)
        
        # Armazenar último comando do gripper
        self.gripper_position = None

        # Parâmetro para tópico de entrada (default: /arm_controller/controller_state)
        self.declare_parameter("input_topic", "/arm_controller/controller_state")
        input_topic = self.get_parameter("input_topic").get_parameter_value().string_value

        # Subscriber para estado do controlador
        self.sub = self.create_subscription(
            JointTrajectoryControllerState,
            input_topic,
            self.joint_states_callback,
            10,
        )
        
        # Subscriber para comandos do gripper
        self.gripper_sub = self.create_subscription(
            JointCommand,
            "/spot_joint_controller/joint_commands",
            self.gripper_command_callback,
            10,
        )

        self.get_logger().info("Joint State Relay inicializado")
        self.get_logger().info(f"Subscrevendo em: {input_topic}")
        self.get_logger().info("Subscrevendo em: /spot_joint_controller/joint_commands")
        self.get_logger().info("Publicando em: joint_command_isaac")

    def joint_states_callback(self, msg):
        # msg: control_msgs/msg/JointTrajectoryControllerState
        names = list(getattr(msg, "joint_names", []) or [])

        # Seleciona o melhor vetor de posições disponível (ordem de preferência)
        positions = None
        for attr in ("actual", "output", "feedback", "reference", "desired"):
            part = getattr(msg, attr, None)
            if part is None:
                continue
            if hasattr(part, "positions") and part.positions:
                positions = list(part.positions)
                break

        if not names or not positions:
            return

        # Filtra apenas as juntas do braço que começam com "arm_" e renomeia para "arm0_"
        arm_joints_indices = []
        new_names = []
        for i, joint_name in enumerate(names):
            if joint_name.startswith("arm_"):
                arm_joints_indices.append(i)
                new_names.append(joint_name.replace("arm_", "arm0_", 1))

        if not arm_joints_indices:
            return

        new_msg = JointState()
        new_msg.header.stamp = self.get_clock().now().to_msg()
        new_msg.name = new_names
        new_msg.position = [positions[i] for i in arm_joints_indices]

        # Tenta preencher velocity/effort se disponíveis em campos semelhantes
        vel = getattr(msg, "actual", None) and getattr(msg.actual, "velocities", None)
        if not vel:
            vel = getattr(msg, "output", None) and getattr(msg.output, "velocities", None)
        eff = getattr(msg, "actual", None) and getattr(msg.actual, "effort", None)

        new_msg.velocity = [vel[i] for i in arm_joints_indices] if vel else []
        new_msg.effort = [eff[i] for i in arm_joints_indices] if eff else []

        # Adicionar gripper se disponível
        if self.gripper_position is not None:
            new_msg.name.append("arm0_f1x")  # Renomear para Isaac
            new_msg.position.append(self.gripper_position)
            if new_msg.velocity:
                new_msg.velocity.append(0.0)  # Velocity não disponível do comando
            if new_msg.effort:
                new_msg.effort.append(0.0)    # Effort não disponível do comando

        self.pub.publish(new_msg)

    def gripper_command_callback(self, msg):
        """Callback para comandos do gripper via spot_joint_controller"""
        # msg: spot_msgs/msg/JointCommand
        try:
            # Procurar arm_f1x nos comandos
            if "arm_f1x" in msg.name:
                idx = msg.name.index("arm_f1x")
                if idx < len(msg.position):
                    self.gripper_position = msg.position[idx]
                    self.get_logger().debug(f"Gripper command: {self.gripper_position}")
        except Exception as e:
            self.get_logger().warn(f"Erro ao processar comando do gripper: {e}")


def main():
    rclpy.init()
    node = JointStateRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
