#!/usr/bin/env python3
"""
Script para adicionar câmeras fisheye ao Spot Robot no USD Stage.

Este script adiciona as câmeras frontleft_fisheye e frontright_fisheye ao robô Spot,
posicionadas no frame 'base'.

Uso:
    python add_spot_cameras.py <usd_file> [--output <output_file>]

Exemplo:
    python add_spot_cameras.py scene.usd --output scene_with_cameras.usd
"""

import sys
import os
import argparse
import math

# Setup environment for USD libs
usd_libs_path = "/isaac-sim/extscache/omni.usd.libs-1.0.1+8131b85d.lx64.r.cp311"
if os.path.exists(usd_libs_path):
    lib_path = os.path.join(usd_libs_path, "bin")
    if lib_path not in os.environ.get("LD_LIBRARY_PATH", ""):
        os.environ["LD_LIBRARY_PATH"] = f"{lib_path}:{os.environ.get('LD_LIBRARY_PATH', '')}"
    if usd_libs_path not in sys.path:
        sys.path.insert(0, usd_libs_path)

try:
    from pxr import Usd, UsdGeom, Sdf, Gf
except ImportError as e:
    print(f"Erro ao importar pxr: {e}")
    print("Execute este script através do Isaac Sim Python ou configure o ambiente")
    sys.exit(1)


# =============================================================================
# CONFIGURAÇÃO DAS CÂMERAS
# =============================================================================
# Parâmetros intrínsecos das câmeras fisheye do Spot (dados reais)
#
# Matriz K (intrínsecos):
#   [[fx,  0, cx],
#    [ 0, fy, cy],
#    [ 0,  0,  1]]
#
# Conversão para USD:
#   focal_length = fx * horizontal_aperture / image_width
#   horizontal_aperture_offset = (cx - image_width/2) * horizontal_aperture / image_width
#   vertical_aperture_offset = (cy - image_height/2) * vertical_aperture / image_height
#
# position: (x, y, z) em metros, relativo ao frame 'base' do Spot
#   - x: frente (+) / trás (-)
#   - y: esquerda (+) / direita (-)  
#   - z: cima (+) / baixo (-)
#
# rotation: (roll, pitch, yaw) em graus
#   - roll: rotação da imagem
#   - pitch: olhar para baixo (+) / cima (-)
#   - yaw: olhar para esquerda (+) / direita (-)

# Resolução das câmeras
IMAGE_WIDTH = 640
IMAGE_HEIGHT = 480

# Tamanho do sensor (escolhido para cálculo - padrão full frame)
SENSOR_WIDTH = 36.0   # mm (horizontal_aperture)
SENSOR_HEIGHT = 27.0  # mm (vertical_aperture) - mantém aspect ratio 4:3

CAMERAS = {
    "frontleft_fisheye": {
        # Posição e orientação
        "position": (0.44, 0.06, 0.03),
        "rotation": (0.0, 12.0, 15.0),       # pitch=12° baixo, yaw=15° esquerda
        
        # Intrínsecos originais (para referência):
        # fx=329.92, fy=329.98, cx=316.99, cy=236.62
        # Nota: USD usa focal_length único, então usamos média de fx/fy
        
        # Parâmetros USD calculados:
        "focal_length": 18.560,              # média(fx,fy) * 36 / 640
        "horizontal_aperture": SENSOR_WIDTH,
        "vertical_aperture": SENSOR_HEIGHT,
        "horizontal_aperture_offset": -0.169,  # (316.99 - 320) * 36 / 640
        "vertical_aperture_offset": -0.190,    # (236.62 - 240) * 27 / 480
    },
    "frontright_fisheye": {
        # Posição e orientação
        "position": (0.44, -0.06, 0.03),
        "rotation": (0.0, 12.0, -15.0),      # pitch=12° baixo, yaw=15° direita
        
        # Intrínsecos originais (para referência):
        # fx=330.20, fy=330.58, cx=330.60, cy=245.74
        # Nota: USD usa focal_length único, então usamos média de fx/fy
        
        # Parâmetros USD calculados:
        "focal_length": 18.596,              # média(fx,fy) * 36 / 640
        "horizontal_aperture": SENSOR_WIDTH,
        "vertical_aperture": SENSOR_HEIGHT,
        "horizontal_aperture_offset": 0.596,   # (330.60 - 320) * 36 / 640
        "vertical_aperture_offset": 0.323,     # (245.74 - 240) * 27 / 480
    },
}


def euler_to_camera_quaternion(roll_deg, pitch_deg, yaw_deg):
    """
    Converte ângulos de Euler para quaternion de câmera USD.
    
    No USD, câmeras olham para -Z por padrão.
    Para olhar para +X (frente do robô), usamos pitch_base=-90.
    """
    base_pitch = -90.0
    
    total_pitch = base_pitch + pitch_deg
    total_yaw = yaw_deg
    total_roll = roll_deg
    
    r = math.radians(total_roll)
    p = math.radians(total_pitch)
    y = math.radians(total_yaw)
    
    cy = math.cos(y * 0.5)
    sy = math.sin(y * 0.5)
    cp = math.cos(p * 0.5)
    sp = math.sin(p * 0.5)
    cr = math.cos(r * 0.5)
    sr = math.sin(r * 0.5)
    
    qw = cr * cp * cy + sr * sp * sy
    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy
    
    return Gf.Quatf(qw, qx, qy, qz)


def create_camera(stage, parent_path, camera_name, config):
    """
    Cria uma câmera no stage USD.
    
    A transformação (translate, orient, scale) é aplicada DIRETAMENTE no Camera prim,
    seguindo o padrão da hand_cam do Spot, para que o TF seja publicado corretamente.
    
    Estrutura criada:
        parent_path/
            camera_name       <- Camera prim com translate + orient + scale
    """
    # Caminho do Camera prim (diretamente no parent, sem Xform intermediário)
    camera_path = f"{parent_path}/{camera_name}"
    
    # Verificar se já existe
    if stage.GetPrimAtPath(camera_path):
        print(f"⚠ Câmera {camera_name} já existe. Pulando.")
        return None
    
    # Criar Camera prim diretamente
    camera = UsdGeom.Camera.Define(stage, camera_path)
    if not camera:
        print(f"✗ Erro ao criar Camera: {camera_path}")
        return None
    
    # Configurar transformação DIRETAMENTE no Camera prim (igual hand_cam)
    pos = config["position"]
    rot = config["rotation"]
    
    # Translation
    translate_op = camera.AddTranslateOp(precision=UsdGeom.XformOp.PrecisionFloat)
    translate_op.Set(Gf.Vec3f(float(pos[0]), float(pos[1]), float(pos[2])))
    
    # Rotation (quaternion)
    quat = euler_to_camera_quaternion(rot[0], rot[1], rot[2])
    orient_op = camera.AddOrientOp(precision=UsdGeom.XformOp.PrecisionFloat)
    orient_op.Set(quat)
    
    # Scale (1,1,1) - igual hand_cam
    scale_op = camera.AddScaleOp(precision=UsdGeom.XformOp.PrecisionFloat)
    scale_op.Set(Gf.Vec3f(1.0, 1.0, 1.0))
    
    # Configurar propriedades da câmera (baseado na matriz K real)
    camera.GetFocalLengthAttr().Set(config["focal_length"])
    camera.GetHorizontalApertureAttr().Set(config["horizontal_aperture"])
    camera.GetVerticalApertureAttr().Set(config["vertical_aperture"])
    
    # Offset do ponto principal (cx, cy diferente do centro)
    camera.GetHorizontalApertureOffsetAttr().Set(config["horizontal_aperture_offset"])
    camera.GetVerticalApertureOffsetAttr().Set(config["vertical_aperture_offset"])
    
    # Clipping range
    camera.GetClippingRangeAttr().Set(Gf.Vec2f(0.01, 100.0))
    
    print(f"✓ Câmera criada: {camera_path}")
    print(f"    Posição: {pos}")
    print(f"    Rotação (RPY): {rot}")
    print(f"    Focal length: {config['focal_length']:.3f} mm")
    print(f"    Aperture: {config['horizontal_aperture']}x{config['vertical_aperture']} mm")
    print(f"    Offset: ({config['horizontal_aperture_offset']:.3f}, {config['vertical_aperture_offset']:.3f}) mm")
    
    return camera_path
    
    return camera_prim_path


def find_spot_base(stage):
    """Encontra o prim 'base' do Spot no stage."""
    possible_paths = [
        "/Root/spot_with_arm/base",
        "/Root/spot/base",
        "/spot_with_arm/base",
        "/spot/base",
        "/World/spot_with_arm/base",
        "/World/spot/base",
    ]
    
    for path in possible_paths:
        prim = stage.GetPrimAtPath(path)
        if prim:
            return path
    
    # Busca genérica por "base" em prims que contêm "spot"
    for prim in stage.Traverse():
        path = str(prim.GetPath())
        if "spot" in path.lower() and path.endswith("/base"):
            return path
    
    return None


def add_cameras_to_spot(usd_file, output_file=None):
    """
    Adiciona as câmeras fisheye ao Spot no arquivo USD.
    
    Args:
        usd_file: Caminho para o arquivo USD de entrada
        output_file: Caminho para salvar (se None, usa nome automático)
    
    Returns:
        True se sucesso, False se erro
    """
    if not os.path.exists(usd_file):
        print(f"✗ Arquivo não encontrado: {usd_file}")
        return False
    
    print(f"Abrindo: {usd_file}")
    print()
    
    stage = Usd.Stage.Open(usd_file)
    if not stage:
        print("✗ Erro ao abrir o arquivo USD")
        return False
    
    # Encontrar o base do Spot
    base_path = find_spot_base(stage)
    
    if not base_path:
        print("✗ Não foi possível encontrar o prim 'base' do Spot")
        print("  Procurei em caminhos comuns como /Root/spot_with_arm/base")
        return False
    
    print(f"✓ Base do Spot: {base_path}")
    print()
    
    # Criar as câmeras
    cameras_created = []
    
    for camera_name, config in CAMERAS.items():
        camera_path = create_camera(stage, base_path, camera_name, config)
        if camera_path:
            cameras_created.append(camera_path)
    
    print()
    
    if not cameras_created:
        print("✗ Nenhuma câmera foi criada")
        return False
    
    # Salvar
    if output_file is None:
        base_name = os.path.splitext(usd_file)[0]
        output_file = f"{base_name}_with_cameras.usd"
    
    stage.GetRootLayer().Export(output_file)
    print(f"✓ Salvo: {output_file}")
    
    # Resumo
    print()
    print("=" * 50)
    print("RESUMO")
    print("=" * 50)
    print(f"Câmeras criadas: {len(cameras_created)}")
    for cam in cameras_created:
        print(f"  - {cam}")
    
    return True


def main():
    parser = argparse.ArgumentParser(
        description="Adiciona câmeras fisheye ao Spot Robot em um arquivo USD"
    )
    parser.add_argument("usd_file", help="Arquivo USD de entrada")
    parser.add_argument("--output", "-o", help="Arquivo USD de saída (opcional)")
    
    args = parser.parse_args()
    
    success = add_cameras_to_spot(args.usd_file, args.output)
    sys.exit(0 if success else 1)


if __name__ == "__main__":
    main()
