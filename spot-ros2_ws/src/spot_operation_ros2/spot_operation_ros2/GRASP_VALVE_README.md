# Grasp Valve Node - Native ROS2 Action API

## 📝 Overview

Este código foi **reescrito** para usar as **Action APIs nativas do ROS2** do MoveIt2, removendo a dependência do `moveit_commander` (pacote legado do ROS1).

## 🔄 Principais Mudanças

### ❌ Removido: `moveit_commander`
```python
from moveit_commander import MoveGroupCommander
arm_group = MoveGroupCommander("arm")
plan = arm_group.plan()
arm_group.execute(trajectory)
```

### ✅ Adicionado: Native ROS2 Action Clients
```python
from moveit_msgs.action import MoveGroup, ExecuteTrajectory

# Action client para planejamento
move_group_client = ActionClient(self, MoveGroup, '/move_action')

# Action client para execução
execute_trajectory_client = ActionClient(self, ExecuteTrajectory, '/execute_trajectory')
```

## 🏗️ Arquitetura

### Action Servers Utilizados

1. **`/move_action`** (tipo: `moveit_msgs/action/MoveGroup`)
   - Usado para planejamento de trajetórias
   - Recebe: `MotionPlanRequest` com pose alvo
   - Retorna: Trajetória planejada

2. **`/execute_trajectory`** (tipo: `moveit_msgs/action/ExecuteTrajectory`)
   - Usado para executar trajetórias planejadas
   - Recebe: Trajetória do planejador
   - Controla: `/arm_controller/follow_joint_trajectory`

### Fluxo de Execução

```
[ValveGraspNode]
    ↓
1. Open Gripper (gripper_controller)
    ↓
2. Compute Grasp Pose (TF transforms + YAML config)
    ↓
3. Plan Motion
    ├→ create_motion_plan_request()
    ├→ send to /move_action
    └→ receive planned_trajectory
    ↓
4. Execute Trajectory
    ├→ send trajectory to /execute_trajectory
    └→ wait for completion
    ↓
5. Close Gripper (gripper_controller)
```

## 🔧 Novos Métodos

### `create_motion_plan_request(target_pose: PoseStamped) -> MotionPlanRequest`
Cria uma requisição de planejamento MoveIt com:
- **Workspace parameters**: Define limites do espaço de trabalho
- **Start state**: Estado atual do robô (is_diff=True)
- **Goal constraints**: 
  - `PositionConstraint`: Tolerância de 1cm
  - `OrientationConstraint`: Tolerância de ~5.7°
- **Planning options**: 
  - 10 tentativas
  - 5s timeout
  - Velocidade/aceleração a 10% do máximo

### `plan_to_pose(target_pose: PoseStamped) -> (success, trajectory)`
1. Cria goal do tipo `MoveGroup.Goal`
2. Configura `planning_options.plan_only = True`
3. Envia para `/move_action` via action client
4. Aguarda resultado (timeout 15s)
5. Retorna sucesso + trajetória planejada

### `execute_trajectory(trajectory) -> success`
1. Cria goal do tipo `ExecuteTrajectory.Goal`
2. Envia trajetória para `/execute_trajectory`
3. Aguarda execução completa (timeout 30s)
4. Retorna sucesso/falha

## 📦 Dependências

### Removidas:
- ❌ `moveit_commander` (não precisa mais!)

### Adicionadas:
- ✅ `moveit_msgs.action.MoveGroup`
- ✅ `moveit_msgs.action.ExecuteTrajectory`
- ✅ `moveit_msgs.msg.MotionPlanRequest`
- ✅ `moveit_msgs.msg.RobotState`
- ✅ `moveit_msgs.msg.PlanningOptions`
- ✅ `moveit_msgs.msg.WorkspaceParameters`

## 🚀 Como Usar

### 1. Construir o workspace
```bash
cd /home/spot-teleop/spot-ros2_ws
colcon build --packages-select spot_operation_ros2
source install/setup.bash
```

### 2. Launch MoveIt (se ainda não estiver rodando)
```bash
ros2 launch spot_moveit_config spot_moveit_all.launch.py
```

### 3. Verificar action servers
```bash
ros2 action list
# Deve mostrar:
# /move_action
# /execute_trajectory
# /arm_controller/follow_joint_trajectory
```

### 4. Rodar o nó
```bash
ros2 run spot_operation_ros2 grasp_valve
```

### 5. Trigger grasp
```bash
ros2 service call /grasp_valve std_srvs/srv/Trigger
```

## ⚙️ Configurações Ajustáveis

### Planning Parameters (em `create_motion_plan_request`)
```python
req.num_planning_attempts = 10           # Tentativas de planejamento
req.allowed_planning_time = 5.0          # Timeout de planejamento (s)
req.max_velocity_scaling_factor = 0.1    # Velocidade (10% do max)
req.max_acceleration_scaling_factor = 0.1 # Aceleração (10% do max)
```

### Tolerances
```python
# Position tolerance
box.dimensions = [0.01, 0.01, 0.01]  # 1cm em cada eixo

# Orientation tolerance
orientation_constraint.absolute_x_axis_tolerance = 0.1  # ~5.7°
orientation_constraint.absolute_y_axis_tolerance = 0.1
orientation_constraint.absolute_z_axis_tolerance = 0.1
```

### Timeouts
```python
# Planning timeout
rclpy.spin_until_future_complete(self, result_future, timeout_sec=15.0)

# Execution timeout
rclpy.spin_until_future_complete(self, result_future, timeout_sec=30.0)
```

## 🐛 Troubleshooting

### "MoveGroup action server not available"
```bash
# Verificar se move_group está rodando
ros2 node list | grep move_group

# Verificar actions disponíveis
ros2 action list
```

### "Planning failed with error code: X"
Error codes comuns (de `moveit_msgs/msg/MoveItErrorCodes`):
- `-1` (FAILURE): Falha geral
- `-2` (PLANNING_FAILED): Sem solução IK ou caminho obstruído
- `-3` (INVALID_MOTION_PLAN): Trajetória inválida
- `-7` (TIMED_OUT): Timeout de planejamento

### "Execution failed"
```bash
# Verificar controller
ros2 control list_controllers

# Verificar se arm_controller está ativo
ros2 control list_controllers | grep arm_controller
```

## 📊 Comparação: Commander vs Action API

| Aspecto | moveit_commander | Action API (atual) |
|---------|------------------|-------------------|
| **Origem** | ROS1 (legado) | ROS2 (nativo) |
| **Dependências** | Python wrapper C++ | Apenas msgs/actions |
| **Build** | Precisa compilar pacote | Já disponível |
| **Controle** | High-level, simples | Low-level, flexível |
| **Debugging** | Limitado | Completo (error codes) |
| **Performance** | Extra overhead | Direto ao action server |
| **Manutenção** | Descontinuado no ROS2 | Suportado oficialmente |

## ✨ Vantagens da Abordagem Atual

1. ✅ **Sem dependência de moveit_commander** (não precisa buildar)
2. ✅ **API nativa ROS2** (alinhado com padrões atuais)
3. ✅ **Mais controle**: Acesso direto a todos os parâmetros
4. ✅ **Melhor debugging**: Error codes detalhados
5. ✅ **Separação clara**: Plan e Execute em etapas distintas
6. ✅ **Timeout configurável**: Para cada etapa
7. ✅ **Async callbacks**: Não bloqueia o nó

## 📚 Referências

- [MoveIt2 Documentation](https://moveit.picknik.ai/humble/index.html)
- [moveit_msgs Actions](https://github.com/ros-planning/moveit_msgs)
- [ROS2 Actions Tutorial](https://docs.ros.org/en/humble/Tutorials/Intermediate/Writing-an-Action-Server-Client/Py.html)
