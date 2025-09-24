#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from control_msgs.msg import JointTrajectoryControllerState


class JointStateRelay(Node):
    def __init__(self):
        super().__init__("joint_state_relay")

        # Publisher para joint_command_isaac
        self.pub = self.create_publisher(JointState, "joint_command_isaac", 10)

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

        self.get_logger().info("Joint State Relay inicializado")
        self.get_logger().info(f"Subscrevendo em: {input_topic}")
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

        self.pub.publish(new_msg)


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
