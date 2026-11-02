# Instalação do cuRobo em Venv (Standalone - sem Isaac Sim)

Este documento descreve o processo completo de compilação e instalação do **cuRobo** em um ambiente virtual Python para rodar o **curobo_mpc_node** do pacote `spot_operation_ros2`.

---

## 📋 Requisitos do Sistema Atual

| Componente | Versão Detectada |
|------------|------------------|
| Sistema Operacional | Ubuntu 22.04.5 LTS |
| Python | 3.10.12 |
| PyTorch | 2.10.0+cu128 |
| CUDA (via PyTorch) | 12.8 |
| CXX11_ABI | True |
| GPU | NVIDIA RTX A2000 12GB (x2) |
| Compute Capability | 8.6 (Ampere - sm_86) |
| Driver NVIDIA | 575.57.08 |
| CUDA Toolkit (nvcc) | ❌ **NÃO INSTALADO** |

---

## 🎯 Objetivo

Rodar o nó ROS2 `curobo_mpc_node.py` que requer:
- **cuRobo** (MPC Solver, Motion Generation)
- **PyTorch** com CUDA
- **Pinocchio** (opcional, para inverse dynamics)
- **ROS2** (Humble assumido)

---

## ⚠️ Problema Identificado

O sistema possui PyTorch com suporte CUDA 12.8, mas **não tem o CUDA Toolkit (nvcc) instalado**. O cuRobo precisa compilar kernels CUDA customizados, então precisamos:

1. Instalar o CUDA Toolkit 12.8 (com nvcc)
2. Criar um venv Python
3. Instalar cuRobo com as flags corretas para A2000

---

## 🚀 Plano de Instalação

### Fase 1: Instalar CUDA Toolkit 12.8

O CUDA Toolkit é necessário para compilar os kernels CUDA do cuRobo.

```bash
# Adicionar repositório NVIDIA CUDA
wget https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2204/x86_64/cuda-keyring_1.1-1_all.deb
sudo dpkg -i cuda-keyring_1.1-1_all.deb
sudo apt-get update

# Instalar CUDA Toolkit 12.8 (apenas toolkit, não driver)
sudo apt-get install -y cuda-toolkit-12-8

# Configurar PATH
echo 'export CUDA_HOME=/usr/local/cuda-12.8' >> ~/.bashrc
echo 'export PATH=$CUDA_HOME/bin:$PATH' >> ~/.bashrc
echo 'export LD_LIBRARY_PATH=$CUDA_HOME/lib64:$LD_LIBRARY_PATH' >> ~/.bashrc
source ~/.bashrc

# Verificar instalação
nvcc --version
```

**Alternativa via pip** (experimental):
```bash
pip install nvidia-cuda-nvcc-cu12
# Nota: Isso pode não funcionar corretamente para compilação de extensões C++
```

---

### Fase 2: Criar Virtual Environment

```bash
cd /home/spot-teleop/spot-ros2_ws

# Criar venv com acesso aos pacotes do sistema (para ROS2)
python3 -m venv curobo_venv --system-site-packages

# Ativar o venv
source curobo_venv/bin/activate

# Verificar Python
which python
python --version
```

---

### Fase 3: Instalar Dependências Base

```bash
# Ativar venv
source /home/spot-teleop/spot-ros2_ws/curobo_venv/bin/activate

# Atualizar pip
pip install --upgrade pip setuptools wheel

# Instalar PyTorch (se não herdado do sistema)
# O venv com --system-site-packages já deve ter o PyTorch instalado
python -c "import torch; print(torch.__version__, torch.cuda.is_available())"

# Se PyTorch não estiver disponível:
# pip install torch torchvision --index-url https://download.pytorch.org/whl/cu128

# Instalar dependências do cuRobo
pip install pybind11 networkx numpy numpy-quaternion pyyaml trimesh scipy tqdm scikit-image
pip install yourdfpy>=0.0.53
pip install warp-lang>=0.9.0
pip install importlib_resources
```

---

### Fase 4: Compilar e Instalar cuRobo

```bash
# Ativar venv
source /home/spot-teleop/spot-ros2_ws/curobo_venv/bin/activate

# Entrar no diretório do cuRobo (já clonado)
cd /home/spot-teleop/spot-ros2_ws/src/curobo

# Verificar CXX11_ABI (deve ser True)
python -c "import torch; print(f'CXX11_ABI: {torch._C._GLIBCXX_USE_CXX11_ABI}')"

# Compilar cuRobo com flags para RTX A2000 (Ampere sm_86)
TORCH_CUDA_ARCH_LIST="8.6" \
CUDA_HOME=/usr/local/cuda-12.8 \
pip install -e . --no-build-isolation -v

# Tempo esperado: 5-15 minutos dependendo do sistema
```

#### Explicação das Variáveis:
- **`TORCH_CUDA_ARCH_LIST="8.6"`**: Arquitetura CUDA para RTX A2000 (Ampere)
  - RTX A2000/A4000/A5000: `"8.6"`
  - RTX 4090/4080: `"8.9"` (Ada Lovelace)
  - RTX 3090/3080: `"8.6"` (Ampere)
  - L40S: `"8.9"` (Ada Lovelace)
- **`CUDA_HOME`**: Path do CUDA Toolkit instalado
- **`--no-build-isolation`**: Usa o ambiente atual ao invés de criar um isolado
- **`-e`**: Instalação em modo editável (útil para desenvolvimento)

---

### Fase 5: Instalar Pinocchio (Opcional - para Inverse Dynamics)

O `curobo_mpc_node.py` usa Pinocchio para calcular esforços (torques). É opcional mas recomendado.

```bash
# Ativar venv
source /home/spot-teleop/spot-ros2_ws/curobo_venv/bin/activate

# Instalar Pinocchio via pip
pip install pin

# Ou via conda (se preferir):
# conda install -c conda-forge pinocchio
```

---

### Fase 6: Verificar Instalação

```bash
# Ativar venv
source /home/spot-teleop/spot-ros2_ws/curobo_venv/bin/activate

# Script de verificação
python -c "
import torch
print(f'PyTorch: {torch.__version__}')
print(f'CUDA disponível: {torch.cuda.is_available()}')
print(f'GPU: {torch.cuda.get_device_name(0)}')
print(f'CXX11_ABI: {torch._C._GLIBCXX_USE_CXX11_ABI}')

import curobo
print(f'cuRobo: {curobo.__version__}')

from curobo.types.math import Pose
from curobo.wrap.reacher.mpc import MpcSolver, MpcSolverConfig
print('✅ cuRobo MPC disponível!')

try:
    import pinocchio as pin
    print(f'✅ Pinocchio: {pin.__version__}')
except ImportError:
    print('⚠️ Pinocchio não instalado (inverse dynamics desabilitado)')

print()
print('✅ cuRobo instalado com sucesso!')
"
```

---

### Fase 7: Configurar para Uso com ROS2

```bash
# Criar script de ativação combinado
cat > /home/spot-teleop/spot-ros2_ws/activate_curobo.sh << 'EOF'
#!/bin/bash
# Ativar ROS2 Humble
source /opt/ros/humble/setup.bash

# Ativar venv cuRobo
source /home/spot-teleop/spot-ros2_ws/curobo_venv/bin/activate

# Ativar workspace ROS2
source /home/spot-teleop/spot-ros2_ws/install/setup.bash

# Configurar CUDA
export CUDA_HOME=/usr/local/cuda-12.8
export PATH=$CUDA_HOME/bin:$PATH
export LD_LIBRARY_PATH=$CUDA_HOME/lib64:$LD_LIBRARY_PATH

echo "✅ Ambiente cuRobo + ROS2 ativado!"
echo "   Python: $(which python)"
echo "   ROS_DISTRO: $ROS_DISTRO"
EOF

chmod +x /home/spot-teleop/spot-ros2_ws/activate_curobo.sh
```

---

### Fase 8: Executar curobo_mpc_node

```bash
# Ativar ambiente
source /home/spot-teleop/spot-ros2_ws/activate_curobo.sh

# Rodar o nó (exemplo com parâmetros default)
ros2 run spot_operation_ros2 curobo_mpc_node

# Ou com parâmetros customizados:
ros2 run spot_operation_ros2 curobo_mpc_node --ros-args \
    -p debug_mode:=true \
    -p control_rate:=30.0 \
    -p use_effort:=true
```

---

## 🔍 Troubleshooting

### Erro: "nvcc not found" ou "CUDA_HOME is None"
**Causa**: CUDA Toolkit não instalado ou PATH não configurado.

**Solução**:
```bash
sudo apt-get install -y cuda-toolkit-12-8
export CUDA_HOME=/usr/local/cuda-12.8
export PATH=$CUDA_HOME/bin:$PATH
```

### Erro: "CUDA error: no kernel image is available"
**Causa**: TORCH_CUDA_ARCH_LIST incorreto para RTX A2000.

**Solução**: Recompilar com a arquitetura correta:
```bash
cd /home/spot-teleop/spot-ros2_ws/src/curobo
pip uninstall nvidia_curobo -y
TORCH_CUDA_ARCH_LIST="8.6" CUDA_HOME=/usr/local/cuda-12.8 pip install -e . --no-build-isolation -v
```

### Erro: "undefined symbol: _ZN3c1021_..."
**Causa**: CXX11_ABI mismatch.

**Solução**: Verificar que o PyTorch e cuRobo usam o mesmo ABI:
```bash
python -c "import torch; print(torch._C._GLIBCXX_USE_CXX11_ABI)"
# Deve retornar True
```

### Erro: "cannot find module 'curobo'"
**Causa**: venv não ativado ou instalação falhou.

**Solução**:
```bash
source /home/spot-teleop/spot-ros2_ws/curobo_venv/bin/activate
pip show nvidia_curobo
```

### Erro de memória GPU durante compilação
**Causa**: Compilação paralela excessiva.

**Solução**: Limitar threads:
```bash
MAX_JOBS=2 TORCH_CUDA_ARCH_LIST="8.6" CUDA_HOME=/usr/local/cuda-12.8 pip install -e . --no-build-isolation -v
```

---

## 📊 Comparação: Isaac Sim vs Standalone

| Aspecto | Isaac Sim | Standalone (este guia) |
|---------|-----------|------------------------|
| Python | 3.11 (embutido) | 3.10 (sistema) |
| PyTorch | 2.7.0+cu128 | 2.10.0+cu128 |
| Instalação | `[isaacsim]` extras | Padrão |
| Simulação | Completa | Não disponível |
| GPU requerida | NVIDIA (8GB+) | RTX A2000 (12GB) ✅ |
| ROS2 | Via extensão | Nativo |

---

## 📝 Dependências do curobo_mpc_node.py

Baseado na análise do código:

```python
# Obrigatórias
torch                    # Tensores GPU
numpy                    # Arrays
rclpy                    # ROS2 Python
geometry_msgs            # ROS2 messages
sensor_msgs              # ROS2 messages
tf2_ros                  # ROS2 TF

# cuRobo
curobo.geom.sdf.world    # Collision checking
curobo.rollout.rollout_base  # Goals
curobo.types.base        # TensorDeviceType
curobo.types.math        # Pose
curobo.types.state       # JointState
curobo.util.logger       # Logging
curobo.util_file         # Config loading
curobo.wrap.reacher.mpc  # MpcSolver

# Opcionais
pinocchio                # Inverse dynamics (para torques)
```

---

## 📅 Informações da Instalação

- **Data**: 21 de Janeiro de 2026
- **Target GPU**: NVIDIA RTX A2000 12GB (sm_86)
- **Python**: 3.10.12
- **PyTorch**: 2.10.0+cu128
- **CUDA**: 12.8
- **cuRobo Source**: `/home/spot-teleop/spot-ros2_ws/src/curobo`

---

## ✅ Checklist de Instalação

- [ ] CUDA Toolkit 12.8 instalado (`nvcc --version`)
- [ ] Venv criado (`curobo_venv`)
- [ ] PyTorch com CUDA funcionando
- [ ] cuRobo compilado com `TORCH_CUDA_ARCH_LIST="8.6"`
- [ ] Pinocchio instalado (opcional)
- [ ] Script `activate_curobo.sh` criado
- [ ] Teste de importação passou
- [ ] `curobo_mpc_node` executa sem erros
