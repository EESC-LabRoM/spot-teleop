#!/bin/bash
# Script para ativar o ambiente cuRobo + ROS2 Humble

# 1. Ativar ROS2 Humble
source /opt/ros/humble/setup.bash

# 2. Ativar Workspace ROS2 (se houver builds)
if [ -f "/home/spot-teleop/spot-ros2_ws/install/setup.bash" ]; then
    source /home/spot-teleop/spot-ros2_ws/install/setup.bash
fi

# 3. Ativar venv cuRobo
source /home/spot-teleop/spot-ros2_ws/curobo_venv/bin/activate

# 4. Configurar PATH do CUDA Toolkit 12.8
export CUDA_HOME=/usr/local/cuda-12.8
export PATH=$CUDA_HOME/bin:$PATH
export LD_LIBRARY_PATH=$CUDA_HOME/lib64:$LD_LIBRARY_PATH

# 5. Garantir visibilidade da GPU (Fix para o erro de CUDA unavailable)
export CUDA_VISIBLE_DEVICES=0
export NVIDIA_VISIBLE_DEVICES=all

echo "🚀 Ambiente cuRobo + ROS2 ativado!"
echo "📍 Python: $(which python)"
echo "🎮 GPU: $(python -c 'import torch; print(torch.cuda.get_device_name(0))')"
