# Valve Grasping System for Boston Dynamics Spot

Sistema de grasp de válvula para o robô Spot usando detecção 6D (FoundationPose) e planejamento de movimento (MoveIt).

## Arquitetura

### Nós

1. **`grasp_valve`** - Nó principal de grasp
   - Subscreve: `/valve_pose` (vision_msgs/Detection3DArray)
   - Publica: `/gripper/command` (std_msgs/Float64)
   - Serviço: `/grasp_valve` (std_srvs/Trigger)
   - **Conecta ao `move_group` existente** (não inicializa MoveIt)
   - Delega controle da gripper para o `gripper_controller`
   - **Carrega ângulos da gripper dinamicamente do `spot_grasp.yaml`**

2. **`gripper_controller`** - Controlador dedicado da gripper
   - Subscreve: `/gripper/command` (std_msgs/Float64)
   - Publica: `/spot_joint_controller/joint_commands` (spot_msgs/JointCommand)
   - Executa movimentos suaves da gripper com interpolação linear
   - Pode ser usado por qualquer nó que precise controlar a gripper

### Dependências Externas (devem estar rodando)

3. **`move_group`** (do `spot_moveit_all.launch.py`)
   - Planning + execution
   - Já configurado com parâmetros ideais
   - `grasp_valve` apenas envia goals

4. **`robot_state_publisher`** (do `spot_moveit_all.launch.py`)
   - TF tree
   - URDF/SRDF

5. **Controllers** (do `spot_moveit_all.launch.py`)
   - `arm_controller`: FollowJointTrajectory
   - `spot_joint_controller`: Para gripper

### TF Bridges Estáticos (inclusos no launch)

6. **`fp_object` ↔ `Mesh`** (FoundationPose ↔ Isaac Sim)
   - Conecta detecção do FoundationPose com modelo do Isaac
   - Transformação identidade (0 0 0 0 0 0 1)
   - Ambos frames representam o center do mesmo `.obj`

7. **`arm_link_fngr` ↔ `arm0_link_fngr`** (ROS ↔ Isaac nomenclatura)
   - Conecta frame da gripper do MoveIt com frame do Isaac
   - Transformação identidade (0 0 0 0 0 0)
   - Bridge de nomenclatura

### Fluxo de Dados

```
spot_moveit_all.launch.py (PRÉ-REQUISITO)
    ├─► move_group (planning/execution)
    ├─► robot_state_publisher (TF)
    └─► controllers (arm + gripper)

TF Bridges (inclusos no valve_grasp.launch.py):
    ├─► fp_object ≡ Mesh (FoundationPose ↔ Isaac)
    └─► arm_link_fngr ≡ arm0_link_fngr (ROS ↔ Isaac)

FoundationPose → /valve_pose → grasp_valve ─┬─► move_group (goals)
         │                                    │
         └─► hand_cam → fp_object (TF)       └─► /gripper/command → gripper_controller
                           ↓
                         Mesh → lever_pivot (Isaac)
```

### Design Philosophy

**⚠️ IMPORTANTE:** Este nó é **lightweight** e assume que a infraestrutura MoveIt já está rodando!

- ✅ **Não inicializa MoveIt** - Conecta ao `move_group` existente
- ✅ **Não configura parâmetros** - Usa configurações do launch file
- ✅ **Apenas envia goals** - Planejamento/execução é feito pelo `move_group`
- ✅ **Desacoplado** - Gripper é controlada por nó separado

**Benefícios:**
- Código mais simples e limpo
- Evita conflitos de inicialização
- Pode rodar múltiplas instâncias (ex: grasp + pick&place)
- Fácil debug (componentes separados)

### TF Tree Esperada

```
body → hand_cam → fp_object → Mesh → lever_pivot
```

## Instalação e Build

```bash
# No workspace root
cd /home/spot-teleop/spot-ros2_ws

# Build o package
colcon build --packages-select spot_operation_ros2

# Source o workspace
source install/setup.bash
```

## Pré-requisitos

**IMPORTANTE:** O nó `grasp_valve` assume que o MoveIt já está rodando. Você **DEVE** iniciar o MoveIt primeiro:

```bash
# Terminal 1: Iniciar MoveIt + Controllers
ros2 launch spot_moveit_config spot_moveit_all.launch.py
```

Este launch file já inicia:
- ✅ `move_group` node (planning + execution)
- ✅ `robot_state_publisher` (TF tree)
- ✅ Controllers (`arm_controller`, `spot_joint_controller`)
- ✅ Planning scene monitor
- ✅ Trajectory execution manager

**O nó `grasp_valve` apenas se conecta ao `move_group` existente!**

## Uso

### Opção 1: Launch File (Recomendado)

```bash
# Terminal 1: MoveIt DEVE estar rodando primeiro!
ros2 launch spot_moveit_config spot_moveit_all.launch.py

# Terminal 2: Lança gripper_controller + grasp_valve
ros2 launch spot_operation_ros2 valve_grasp.launch.py

# Terminal 3: Executar grasp
ros2 service call /grasp_valve std_srvs/srv/Trigger
```

### Opção 2: Manual (nós separados)

```bash
# Terminal 1: MoveIt DEVE estar rodando primeiro!
ros2 launch spot_moveit_config spot_moveit_all.launch.py

# Terminal 2: Iniciar o gripper controller
ros2 run spot_operation_ros2 gripper_controller

# Terminal 3: Iniciar o nó de grasp
ros2 run spot_operation_ros2 grasp_valve

# Terminal 4: Executar grasp
ros2 service call /grasp_valve std_srvs/srv/Trigger
```

## Comandos da Gripper

Você pode controlar a gripper diretamente de qualquer nó ou terminal:

```bash
# Abrir gripper (pre-grasp)
ros2 topic pub --once /gripper/command std_msgs/Float64 "data: -1.57"

# Fechar gripper (grasp)
ros2 topic pub --once /gripper/command std_msgs/Float64 "data: -0.199"

# Posição personalizada
ros2 topic pub --once /gripper/command std_msgs/Float64 "data: -0.8"
```

## Sequência do Grasp

1. **Abrir gripper** - Posição pre-grasp (-1.57 rad)
2. **Calcular pose** - Lookup de TF e aplicação de offset do `spot_grasp.yaml`
3. **Planejar movimento** - MoveIt planeja trajetória para `arm_link_wr1`
4. **Executar movimento** - Braço se move para posição de grasp
5. **Fechar gripper** - Posição de grasp (-0.199 rad)

## Configuração

### spot_grasp.yaml

Define a pose do grasp em relação ao `lever_pivot`. **Os ângulos da gripper são extraídos automaticamente deste arquivo**:

```yaml
grasps:
   "grasp_0":
      position: [-0.039, 0.222, 0.020]
      orientation: {w: -0.703, xyz: [-0.004, 0.022, 0.711]}
      cspace_position:
         arm0_f1x: -0.19896012544631958  # Gripper fechada (extraído dinamicamente)
      pregrasp_cspace_position:
         arm0_f1x: -1.5707999467849731  # Gripper aberta (extraído dinamicamente)
```

**Nota importante**: Os valores de `arm0_f1x` em `cspace_position` e `pregrasp_cspace_position` são carregados dinamicamente pelo nó `grasp_valve`. Você pode modificar esses valores no YAML para ajustar o comportamento da gripper sem precisar alterar código.

### Múltiplos Grasps

Você pode definir múltiplas configurações de grasp no YAML:

```yaml
grasps:
   "grasp_0":
      # Configuração 1...
      cspace_position:
         arm0_f1x: -0.199  # Fechada
      pregrasp_cspace_position:
         arm0_f1x: -1.571  # Aberta
   
   "grasp_1":
      # Configuração 2 com valores diferentes...
      cspace_position:
         arm0_f1x: -0.5  # Parcialmente fechada
      pregrasp_cspace_position:
         arm0_f1x: -1.2  # Parcialmente aberta
```

**Atualmente, o nó usa `grasp_0` por padrão**. Para usar outros grasps, modifique a chave no código ou implemente seleção dinâmica.

## Parâmetros

### Gripper Controller
- **Control Frequency**: 50 Hz
- **Default Duration**: 1.5 segundos
- **Gains**: k_q_p=16.0, k_qd_p=0.32

### Grasp Node
- **Planning Frame**: `body`
- **Planning Group**: `arm`
- **End Effector**: `arm_link_wr1`
- **Pose Averaging**: Janela de 5 detecções
- **Velocity Scaling**: 0.3 (30%)
- **Acceleration Scaling**: 0.3 (30%)

## Dependências

- `rclpy`
- `moveit_commander`
- `tf2_ros`, `tf2_geometry_msgs`
- `vision_msgs`, `geometry_msgs`, `std_msgs`, `std_srvs`
- `spot_msgs`
- `synchros2`

## Debugging

### Verificar detecções de válvula

```bash
ros2 topic echo /valve_pose
```

### Verificar TF tree

```bash
ros2 run tf2_tools view_frames
```

### Verificar estado da gripper

```bash
ros2 topic echo /joint_states_mapped | grep arm_f1x
```

### Logs do grasp

```bash
ros2 run spot_operation_ros2 grasp_valve --ros-args --log-level debug
```

## Troubleshooting

### "Failed to compute grasp pose"
- Verifique se `/valve_pose` está publicando
- Verifique TF tree: `body` → `lever_pivot` deve existir
- Certifique-se de ter pelo menos 3 detecções acumuladas

### "Motion planning failed"
- Verifique se MoveIt está rodando
- Pose pode estar fora do workspace do braço
- Tente ajustar velocity/acceleration scaling

### "Gripper não se move"
- Verifique se `gripper_controller` está rodando
- Verifique se `spot_joint_controller` está ativo
- Cheque `/joint_states_mapped` para feedback

## Integração com outros nós

Qualquer nó pode controlar a gripper publicando em `/gripper/command`:

```python
from std_msgs.msg import Float64

# No seu nó
self.gripper_pub = self.create_publisher(Float64, 'gripper/command', 10)

# Abrir
msg = Float64()
msg.data = -1.57
self.gripper_pub.publish(msg)

# Fechar
msg.data = -0.199
self.gripper_pub.publish(msg)
```

## Próximos Passos (Futuro)

- [ ] Implementar retreat após grasp
- [ ] Adicionar giro da válvula
- [ ] Controle de força/torque da gripper
- [ ] Recuperação de falhas
- [ ] Visualização no RViz
