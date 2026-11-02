# Instalação do cuRobo no Isaac Sim 5.0

Este documento descreve o processo completo de compilação e instalação do **cuRobo** no ambiente do **Isaac Sim 5.0.0**.

---

## 📋 Requisitos

| Componente | Versão |
|------------|--------|
| Isaac Sim | 5.0.0 |
| Python | 3.11 |
| PyTorch | 2.7.0+cu128 |
| CUDA Toolkit | 12.8 |
| GPU | NVIDIA L40S (ou compatível) |

---

## 🔧 Informações do Ambiente

### GPU
```
NVIDIA L40S - 46GB VRAM
Compute Capability: 8.9 (Ada Lovelace - sm_89)
Driver: 580.95.05
```

### Paths Importantes
```bash
ISAAC_SIM_PATH=/isaac-sim
CUDA_PATH=/usr/local/cuda-12.8
PYTHON=/isaac-sim/python.sh
```

---

## 🚀 Processo de Instalação

### 1. Clonar o repositório cuRobo

```bash
cd /workspace
git clone https://github.com/NVlabs/curobo.git
cd curobo
```

### 2. Verificar CXX11_ABI do PyTorch

**IMPORTANTE**: O Isaac Sim 5.0 usa `CXX11_ABI=True`, diferente das versões anteriores.

```bash
/isaac-sim/python.sh -c "import torch; print(torch._C._GLIBCXX_USE_CXX11_ABI)"
# Resultado esperado: True
```

### 3. Instalar cuRobo com as flags corretas

```bash
cd /workspace/curobo

TORCH_CUDA_ARCH_LIST="8.9" \
CUDA_HOME=/usr/local/cuda-12.8 \
/isaac-sim/python.sh -m pip install -e ".[isaacsim]" \
  --no-build-isolation \
  -v
```

#### Explicação das variáveis:
- **`TORCH_CUDA_ARCH_LIST="8.9"`**: Arquitetura CUDA para L40S (Ada Lovelace)
  - RTX 4090/4080: `"8.9"`
  - RTX 3090/3080: `"8.6"`
  - A100: `"8.0"`
  - V100: `"7.0"`
- **`CUDA_HOME`**: Path do CUDA toolkit (usado apenas para compilação)
- **`--no-build-isolation`**: Usa o ambiente do Isaac Sim ao invés de criar um isolado
- **`-e`**: Instalação em modo editável

### 4. Tempo de Compilação

A compilação dos kernels CUDA leva aproximadamente **3-5 minutos** na L40S.

---

## ✅ Verificação da Instalação

```bash
/isaac-sim/python.sh -c "
import torch
print(f'PyTorch: {torch.__version__}')
print(f'CUDA disponível: {torch.cuda.is_available()}')
print(f'GPU: {torch.cuda.get_device_name(0)}')

import curobo
print(f'cuRobo: {curobo.__version__}')

from curobo.types.math import Pose
from curobo.wrap.reacher.motion_gen import MotionGen, MotionGenConfig
print('✅ cuRobo instalado com sucesso!')
"
```

Resultado esperado:
```
PyTorch: 2.7.0+cu128
CUDA disponível: True
GPU: NVIDIA L40S
cuRobo: 0.7.7.post1.dev5
✅ cuRobo instalado com sucesso!
```

---

## 🎮 Executando Exemplos

### Motion Generation (Batch Planning)

```bash
cd /workspace/curobo/examples/isaac_sim
/isaac-sim/python.sh motion_gen_reacher.py
```

Este exemplo demonstra:
- Planejamento de trajetória completa
- MotionGen.plan_single() para gerar path do início ao fim
- Interpolação e execução suave

### MPC Example (Servo-Style Control)

```bash
cd /workspace/curobo/examples/isaac_sim
/isaac-sim/python.sh mpc_example.py
```

Este exemplo demonstra:
- Controle MPC em tempo real (~50Hz)
- `mpc.step()` para receding horizon control
- MPPI (Model Predictive Path Integral)
- CUDA Graph para otimização de performance

#### Interação:
- Mova o cubo vermelho (target) na cena
- O robô seguirá automaticamente evitando colisões

---

## 🔍 Arquitetura cuRobo

### MotionGen vs MpcSolver

| Característica | MotionGen | MpcSolver |
|----------------|-----------|-----------|
| Uso | Planejamento batch | Controle reativo |
| Frequência | Uma vez por goal | 50Hz contínuo |
| Método | `plan_single()` | `step()` |
| Horizonte | Trajetória completa | Receding horizon |
| Aplicação | Pick & place | Teleoperation, tracking |

---

## ⚠️ Troubleshooting

### Erro: "CUDA error: no kernel image is available"
**Causa**: TORCH_CUDA_ARCH_LIST incorreto para sua GPU.

**Solução**: Verifique a compute capability da sua GPU:
```bash
nvidia-smi --query-gpu=compute_cap --format=csv,noheader
```

### Erro: "undefined symbol: _ZN3c1021_..."
**Causa**: CXX11_ABI mismatch entre cuRobo e PyTorch.

**Solução**: Reinstale com `--no-build-isolation` para usar o ambiente correto.

### Processo zumbi após execução
**Causa**: Isaac Sim não finalizou corretamente.

**Solução**: 
```bash
pkill -9 -f "python|isaac|kit|omni"
```

---

## 📝 Notas sobre nvblox

O **nvblox** (para collision avoidance com depth cameras) **não é compatível** com CUDA 12.8 devido a conflitos nos headers NVTX.

**Alternativa**: Rodar nvblox em container separado e compartilhar ESDF via ROS 2 topics.

---

## 📚 Referências

- [cuRobo GitHub](https://github.com/NVlabs/curobo)
- [cuRobo Documentation](https://curobo.org/)
- [Isaac Sim Documentation](https://docs.omniverse.nvidia.com/isaacsim/latest/)

---

## 📅 Última Atualização

- **Data**: 2 de Dezembro de 2025
- **Isaac Sim**: 5.0.0
- **cuRobo**: 0.7.7.post1.dev5
- **Testado em**: NVIDIA L40S (sm_89)
