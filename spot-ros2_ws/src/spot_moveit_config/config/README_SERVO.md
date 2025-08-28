# Configuração do MoveIt Servo para o Spot Robot

Este arquivo (`spot_servo_config.yaml`) contém a configuração do MoveIt Servo para controle em tempo real do braço do robô Spot.

## Configurações Principais

### 1. Configurações Básicas
- **move_group_name**: "arm" - Grupo de planejamento definido no SRDF
- **planning_frame**: "base_link" - Frame de referência base do robô
- **robot_link_command_frame**: "arm_link_fngr" - Frame do end-effector (gripper)

### 2. Tipos de Comando
- **command_in_type**: "speed_units" - Comandos em unidades de velocidade (m/s e rad/s)
- **command_out_type**: "trajectory_msgs/JointTrajectory" - Compatível com o JointTrajectoryController

### 3. Escalas de Velocidade
- **linear**: 0.2 m/s - Velocidade máxima linear
- **rotational**: 0.5 rad/s - Velocidade máxima rotacional
- **joint**: 0.5 rad/s - Velocidade máxima das juntas

### 4. Tópicos de Comunicação
- **joint_topic**: "/joint_states" - Estado atual das juntas
- **command_out_topic**: "/arm_controller/joint_trajectory" - Comandos para o controlador
- **cartesian_command_in_topic**: "/servo_commands/delta_twist_cmds" - Entrada de comandos cartesianos
- **joint_command_in_topic**: "/servo_commands/delta_joint_cmds" - Entrada de comandos de junta

## Como Usar

### 1. Comandos Cartesianos (Twist)
```bash
# Publicar comando de velocidade cartesiana
ros2 topic pub /servo_commands/delta_twist_cmds geometry_msgs/TwistStamped "
header:
  frame_id: 'base_link'
twist:
  linear:
    x: 0.1
    y: 0.0
    z: 0.0
  angular:
    x: 0.0
    y: 0.0
    z: 0.0"
```

### 2. Comandos de Junta
```bash
# Publicar comando de velocidade de junta
ros2 topic pub /servo_commands/delta_joint_cmds control_msgs/JointJog "
header:
  frame_id: 'base_link'
joint_names: ['arm_sh0', 'arm_sh1', 'arm_el0', 'arm_el1', 'arm_wr0', 'arm_wr1']
displacements: [0.1, 0.0, 0.0, 0.0, 0.0, 0.0]
velocities: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
duration: 1.0"
```

### 3. Iniciar o Servo
```bash
# Carregaar a configuração e iniciar o servo
ros2 launch moveit_servo servo_example.launch.py config_file_name:=spot_servo_config.yaml
```

## Configurações de Segurança

### Detecção de Singularidades
- **threshold**: 0.15 - Limiar para detecção de singularidades
- **hard_stop_singularity_threshold**: 0.2 - Parada completa em singularidades críticas

### Detecção de Colisões
- **distance_threshold**: 0.07m - Distância mínima para evitar colisões
- **collision_margin_distance**: 0.025m - Margem de segurança para colisões

### Timeouts
- **incoming_command_timeout**: 0.1s - Timeout para comandos de entrada
- **publish_period**: 0.005s - Período de publicação (200 Hz)

## Personalização

### Para Robô Real vs Simulação
- **use_gazebo**: false - Configurado para robô real
- Ajuste **publish_period** conforme necessário para performance em tempo real

### Ajuste de Velocidades
Modifique as escalas conforme a aplicação:
- Reduza para movimentos mais precisos
- Aumente para movimentos mais rápidos (com cuidado)

### Filtros de Suavização
- **low_pass_filter_coeff**: 2.0 - Filtro passa-baixa para suavizar comandos
- **smoothing_filter_plugin_name**: Plugin Butterworth para suavização adicional

## Troubleshooting

1. **Servo não responde**: Verifique se o tópico `/joint_states` está publicando
2. **Movimento muito lento**: Aumente as escalas de velocidade
3. **Movimento instável**: Reduza as escalas ou ajuste o filtro passa-baixa
4. **Colisões frequentes**: Aumente `distance_threshold` ou revise a cena de colisão
