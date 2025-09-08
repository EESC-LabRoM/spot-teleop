# Spot ROS2 Multi-Workspace

Este repositório contém múltiplos workspaces ROS2 para robótica com Boston Dynamics Spot, câmeras ZED, RealSense e simulação Isaac Sim.

## 📁 Estrutura dos Workspaces

- **`spot-ros2_ws/`** - Workspace principal do Spot ROS2 + RealSense + MoveIt
- **`zed_ws/`** - Workspace ZED + Isaac ROS + NVBlox  
- **`isaac-sim_ws/`** - Workspace Isaac Sim + ZED Isaac Sim

## 🚀 Clonagem

```bash
# Clone com todos os submódulos
git clone --recursive https://github.com/murilo-vinicius04/spot-teleop.git
cd spot-teleop

# Ou se já clonou, inicialize os submódulos
git submodule update --init --recursive

# Configurar Git LFS para isaac_ros_nitros
cd zed_ws/src/isaac_ros_nitros
git lfs install
git lfs pull
cd ../../..
```

## 🐳 Build com Docker

### Pré-requisitos
- Docker
- Docker Compose
- NVIDIA Docker Runtime (para GPU)

### Build dos Containers

```bash
# Build todos os containers
docker-compose build

# Ou build individual
docker-compose build spot-ros2
docker-compose build zed
docker-compose build isaac-sim
```

## 🏃 Execução

### Spot ROS2 + RealSense
```bash
docker-compose up spot-ros2
```

### ZED + NVBlox
```bash
docker-compose up zed
```

### Isaac Sim
```bash
docker-compose up isaac-sim
```

### Todos os serviços
```bash
docker-compose up
```

## 📦 Submódulos Incluídos

### ZED Workspace
- `isaac_ros_nitros` - NVIDIA Isaac ROS Nitros
- `negotiated` - Negotiated QoS
- `zed-ros2-wrapper` - ZED ROS2 Wrapper
- `isaac_ros_nvblox` - NVIDIA Isaac ROS NVBlox
- `zed-ros2-interfaces` - ZED ROS2 Interfaces
- `isaac_ros_common` - NVIDIA Isaac ROS Common

### Spot ROS2 Workspace
- `spot_ros2` - Boston Dynamics Spot ROS2
- `moveit2` - MoveIt2 Motion Planning
- `moveit_msgs` - MoveIt2 Messages
- `moveit_resources` - MoveIt2 Resources
- `moveit_task_constructor` - MoveIt Task Constructor
- `moveit2_tutorials` - MoveIt2 Tutorials

### Isaac Sim Workspace
- `zed-isaac-sim` - ZED Isaac Sim Integration

## 🔧 Configuração

### Variáveis de Ambiente
```bash
export SPOT_NAME=YourSpotName
export ROS_DOMAIN_ID=0
```

### GPU Support
Certifique-se de ter o NVIDIA Docker runtime instalado:
```bash
# Ubuntu/Debian
distribution=$(. /etc/os-release;echo $ID$VERSION_ID)
curl -s -L https://nvidia.github.io/nvidia-docker/gpgkey | sudo apt-key add -
curl -s -L https://nvidia.github.io/nvidia-docker/$distribution/nvidia-docker.list | sudo tee /etc/apt/sources.list.d/nvidia-docker.list

sudo apt-get update && sudo apt-get install -y nvidia-docker2
sudo systemctl restart docker
```

## 📝 Notas

- O container Isaac Sim requer aceitação da EULA da NVIDIA
- Certifique-se de ter as permissões adequadas para dispositivos USB (RealSense, ZED)
- Para desenvolvimento, monte volumes adequados conforme necessário

## 🤝 Contribuição

1. Fork o projeto
2. Crie uma branch para sua feature
3. Commit suas mudanças
4. Push para a branch
5. Abra um Pull Request
