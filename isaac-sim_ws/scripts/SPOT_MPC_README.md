# Spot Arm MPC Scripts Documentation

Scripts para controle MPC (Model Predictive Control) do braço do Boston Dynamics Spot usando cuRobo + Isaac Sim.

## Scripts

### mpc_spot_example.py

Exemplo básico de MPC com obstáculos primitivos (cuboids).

**Uso:**
```bash
/isaac-sim/python.sh /workspace/scripts/mpc_spot_example.py
```

**Funcionalidades:**
- Carrega o Spot arm no Isaac Sim
- Cria um target (cubo vermelho) que o braço segue
- Obstáculos definidos como cuboids no código
- Visualiza rollouts do MPC como pontos

**Argumentos:**
- `--headless_mode`: Rodar sem GUI (`native` ou `websocket`)
- `--visualize_spheres`: Visualizar esferas de colisão
- `--robot`: Arquivo de configuração do robô (default: `spot_arm.yml`)

---

### mpc_spot_nvblox_example.py

MPC com integração nvblox via ROS2 - obstáculos vêm do mapa 3D reconstruído.

**Uso:**
```bash
export ROS_DISTRO=humble
/isaac-sim/python.sh /workspace/scripts/mpc_spot_nvblox_example.py
```

**Funcionalidades:**
- Tudo do `mpc_spot_example.py`
- Subscriber ROS2 para `/nvblox_node/pessimistic_static_esdf_pointcloud`
- Converte pontos ESDF ocupados em cuboids para colisão
- Atualiza obstáculos periodicamente (~2 Hz)

**Argumentos adicionais:**
- `--esdf_topic`: Tópico ESDF (default: `/nvblox_node/pessimistic_static_esdf_pointcloud`)
- `--update_hz`: Taxa de atualização de obstáculos (default: `2.0`)

**Requisitos:**
- nvblox node rodando e publicando ESDF pointcloud
- ROS2 Humble (ou usar libs internas do Isaac Sim com `export ROS_DISTRO=humble`)

---

## Configuração do Robô

Arquivos em `/workspace/config/`:
- `spot_arm.yml`: Configuração principal do Spot arm
- `spheres/spot_arm.yml`: Esferas de colisão

---

## Notas

### TF2 (para robô real)
O código TF está comentado pois requer ROS2 completo instalado. Para habilitar:
1. Instalar: `apt install ros-humble-tf2-ros`
2. Descomentar imports e código TF no script
3. Source ROS2: `source /opt/ros/humble/setup.bash`

### Parâmetros de Colisão
- `voxel_size`: Tamanho dos cuboids de obstáculo (default: 5cm)
- `distance_threshold`: ESDF < threshold = obstáculo (default: 1cm)
- `max_obstacles`: Limite de obstáculos por frame (default: 200)
