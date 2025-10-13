#!/bin/bash
# Script para configurar submodules nas branches corretas

set -e

echo "🔧 Configurando submodules..."

# Inicializa e atualiza submodules para as branches configuradas
echo "📥 Atualizando submodules para branches configuradas no .gitmodules..."
git submodule update --init --recursive --remote

# Lista de submodules com branches específicas
SUBMODULES_WITH_BRANCHES=(
    "spot-ros2_ws/src/moveit2:humble"
    "spot-ros2_ws/src/moveit_msgs:humble"
    "spot-ros2_ws/src/moveit_resources:humble"
    "spot-ros2_ws/src/moveit_task_constructor:humble"
    "spot-ros2_ws/src/moveit2_tutorials:humble"
)

echo ""
echo "🌿 Verificando branches dos submodules..."
for entry in "${SUBMODULES_WITH_BRANCHES[@]}"; do
    IFS=':' read -r path branch <<< "$entry"
    if [ -d "$path" ]; then
        echo "  ✓ $path -> $branch"
        cd "$path"
        current_branch=$(git branch --show-current)
        if [ -z "$current_branch" ]; then
            echo "    ⚠️  Detached HEAD detectado, fazendo checkout de $branch..."
            git checkout "$branch"
            git pull origin "$branch"
        elif [ "$current_branch" != "$branch" ]; then
            echo "    ⚠️  Branch atual: $current_branch, mudando para $branch..."
            git checkout "$branch"
            git pull origin "$branch"
        else
            echo "    ✓ Já está em $branch"
        fi
        cd - > /dev/null
    fi
done

echo ""
echo "✅ Submodules configurados com sucesso!"
echo ""
echo "📋 Status dos submodules:"
git submodule foreach --recursive 'echo "  $name: $(git branch --show-current || echo \"detached HEAD\")"'
