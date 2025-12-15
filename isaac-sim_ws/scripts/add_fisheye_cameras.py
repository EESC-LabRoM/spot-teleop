#!/usr/bin/env python3
"""
Script para adicionar câmeras fisheye ao Spot Robot no USD Stage.

Este script adiciona as câmeras frontleft_fisheye e frontright_fisheye ao robô Spot,
posicionadas no frame 'base', e cria o ActionGraph para publicar os dados via ROS2.

Uso:
    python add_fisheye_cameras.py <usd_file> [--output <output_file>]

Autor: Copilot
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
    print("Execute este script através do run_usd_analyzer.sh ou configure o ambiente Isaac Sim")
    sys.exit(1)


# Dados das câmeras do Spot (posição e orientação relativas ao body frame)
# Formato: position (x, y, z) em metros, rotation (roll, pitch, yaw) em graus
# 
# As câmeras fisheye do Spot apontam para frente com leve ângulo lateral:
# - yaw: ângulo lateral (positivo = esquerda, negativo = direita)
# - pitch: ângulo vertical (positivo = para baixo)
# - roll: rotação da imagem (90° CCW como no Spot real)
#
# Valores aproximados baseados no Spot real
CAMERA_TRANSFORMS = {
    "frontleft_fisheye": {
        "position": (0.44, 0.06, 0.03),      # Frente esquerda
        "rotation": (0.0, 12.0, 15.0),        # yaw=15° para esquerda, pitch=12° para baixo
        "focal_length": 1.93,                  # Fisheye lens (muito curta)
        "horizontal_aperture": 10.0,           # Amplo FOV
    },
    "frontright_fisheye": {
        "position": (0.44, -0.06, 0.03),     # Frente direita
        "rotation": (0.0, 12.0, -15.0),       # yaw=-15° para direita, pitch=12° para baixo
        "focal_length": 1.93,
        "horizontal_aperture": 10.0,
    },
}

# Configuração do ROS2
ROS2_CONFIG = {
    "frontleft_fisheye": {
        "rgb_topic": "/frontleft_fisheye/image_raw",
        "depth_topic": "/frontleft_fisheye/depth",
        "info_topic": "/frontleft_fisheye/camera_info",
        "frame_id": "frontleft_fisheye_optical",
    },
    "frontright_fisheye": {
        "rgb_topic": "/frontright_fisheye/image_raw",
        "depth_topic": "/frontright_fisheye/depth",
        "info_topic": "/frontright_fisheye/camera_info",
        "frame_id": "frontright_fisheye_optical",
    },
}


def euler_to_quaternion_simple(roll_deg, pitch_deg, yaw_deg):
    """Converte ângulos de Euler (graus) para quaternion Gf.Quatd."""
    roll = math.radians(roll_deg)
    pitch = math.radians(pitch_deg)
    yaw = math.radians(yaw_deg)
    
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    
    w = cr * cp * cy + sr * sp * sy
    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy
    
    return Gf.Quatd(w, x, y, z)


def multiply_quaternions(q1, q2):
    """Multiplica dois quaternions: q1 * q2 (aplica q2 primeiro, depois q1)"""
    w1, x1, y1, z1 = q1.GetReal(), q1.GetImaginary()[0], q1.GetImaginary()[1], q1.GetImaginary()[2]
    w2, x2, y2, z2 = q2.GetReal(), q2.GetImaginary()[0], q2.GetImaginary()[1], q2.GetImaginary()[2]
    
    w = w1*w2 - x1*x2 - y1*y2 - z1*z2
    x = w1*x2 + x1*w2 + y1*z2 - z1*y2
    y = w1*y2 - x1*z2 + y1*w2 + z1*x2
    z = w1*z2 + x1*y2 - y1*x2 + z1*w2
    
    return Gf.Quatd(w, x, y, z)


def euler_to_camera_quaternion(roll_deg, pitch_deg, yaw_deg):
    """
    Converte ângulos de Euler para quaternion de câmera USD.
    
    No USD/Isaac Sim, câmeras por padrão olham para -Z com +Y para cima.
    Para uma câmera no robô olhar "para frente" (+X no frame do robô):
    
    Descoberta através de teste: pitch=-90 (em ZYX) faz -Z -> +X
    
    Os ângulos da câmera são:
    - roll: rotação da imagem (em torno do eixo de visão)
    - pitch: olhar para cima/baixo (positivo = para baixo no mundo)
    - yaw: olhar para esquerda/direita (positivo = esquerda)
    
    A rotação base é pitch=-90 para apontar para +X.
    Depois aplicamos os ajustes da câmera.
    """
    # Rotação base: pitch=-90 para fazer câmera olhar +X
    # Depois aplicamos os ajustes:
    # - pitch_deg: adiciona ao pitch base (positivo = baixo)
    # - yaw_deg: rotaciona horizontalmente (positivo = esquerda)
    # - roll_deg: rotaciona a imagem
    
    base_pitch = -90.0
    
    # Pitch total: base (-90) + ajuste da câmera
    # Se pitch_deg=12 (para baixo), precisamos diminuir o pitch total
    total_pitch = base_pitch + pitch_deg
    
    # Yaw: rotação horizontal
    total_yaw = yaw_deg
    
    # Roll: rotação da imagem
    total_roll = roll_deg
    
    # Converter para quaternion usando ZYX
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


def create_camera_prim(stage, parent_path, camera_name, cam_config):
    """Cria um prim de câmera com a configuração especificada."""
    camera_path = f"{parent_path}/{camera_name}"
    
    # Criar Xform para a câmera (permite transformações)
    xform_path = camera_path
    xform = UsdGeom.Xform.Define(stage, xform_path)
    
    if not xform:
        print(f"Erro ao criar Xform em {xform_path}")
        return None
    
    # Definir transformação
    pos = cam_config["position"]
    rot = cam_config["rotation"]
    
    # Criar operações de transformação
    xform_ops = xform.GetOrderedXformOps()
    
    # Translation (usar Vec3f para compatibilidade)
    translate_op = xform.AddTranslateOp(precision=UsdGeom.XformOp.PrecisionFloat)
    translate_op.Set(Gf.Vec3f(float(pos[0]), float(pos[1]), float(pos[2])))
    
    # Rotation (como quaternion) - usar precision float
    # Usar a função corrigida que aplica a rotação base da câmera USD
    quat = euler_to_camera_quaternion(rot[0], rot[1], rot[2])
    orient_op = xform.AddOrientOp(precision=UsdGeom.XformOp.PrecisionFloat)
    orient_op.Set(quat)
    
    # Criar a câmera como filha do Xform
    camera_prim_path = f"{xform_path}/Camera"
    camera = UsdGeom.Camera.Define(stage, camera_prim_path)
    
    if not camera:
        print(f"Erro ao criar Camera em {camera_prim_path}")
        return None
    
    # Configurar propriedades da câmera fisheye
    camera.GetFocalLengthAttr().Set(cam_config["focal_length"])
    camera.GetHorizontalApertureAttr().Set(cam_config["horizontal_aperture"])
    
    # Clipping planes
    camera.GetClippingRangeAttr().Set(Gf.Vec2f(0.01, 100.0))
    
    print(f"✓ Câmera criada: {camera_prim_path}")
    print(f"  Posição: {pos}")
    print(f"  Rotação (RPY): {rot}")
    print(f"  Focal length: {cam_config['focal_length']}mm")
    
    return camera_prim_path


def create_action_graph_node(stage, graph_path, node_name, node_type, node_version=1, position=(0, 0)):
    """
    Cria um nó no ActionGraph com a estrutura correta do OmniGraph.
    
    Baseado na análise de ActionGraphs existentes, os nodes precisam:
    - Tipo de prim: OmniGraphNode
    - Atributo node:type com o tipo do nó
    - Atributo node:typeVersion com a versão
    - Atributos ui:nodegraph:node:* para posição visual
    """
    node_path = f"{graph_path}/{node_name}"
    
    # Criar prim do tipo OmniGraphNode
    prim = stage.DefinePrim(node_path, "OmniGraphNode")
    
    if prim:
        # Atributos obrigatórios do OmniGraph node
        prim.CreateAttribute("node:type", Sdf.ValueTypeNames.Token).Set(node_type)
        prim.CreateAttribute("node:typeVersion", Sdf.ValueTypeNames.Int).Set(node_version)
        
        # Atributos visuais do editor
        prim.CreateAttribute("ui:nodegraph:node:pos", Sdf.ValueTypeNames.Float2).Set(position)
        prim.CreateAttribute("ui:nodegraph:node:expansionState", Sdf.ValueTypeNames.Token).Set("open")
        
        return prim
    
    return None


def set_node_input(node_prim, input_name, value, value_type):
    """Define um atributo de entrada para um nó do OmniGraph."""
    attr_name = f"inputs:{input_name}"
    attr = node_prim.CreateAttribute(attr_name, value_type)
    if attr:
        attr.Set(value)
    return attr


def create_action_graph_for_cameras(stage, graph_name, cameras_created):
    """
    Cria um ActionGraph para as câmeras fisheye.
    
    Baseado na estrutura do ActionGraph_02 existente:
    - OnPlaybackTick (trigger)
    - IsaacCreateRenderProduct (para cada câmera)
    - ROS2CameraHelper (RGB)
    - ROS2CameraHelper (Depth)
    - ROS2CameraInfoHelper
    - ROS2PublishTransformTree
    - IsaacReadSimulationTime
    """
    graph_path = f"/Root/{graph_name}"
    
    # Verificar se já existe
    existing = stage.GetPrimAtPath(graph_path)
    if existing:
        print(f"⚠ ActionGraph {graph_path} já existe. Pulando criação.")
        return None
    
    print(f"\n=== Criando ActionGraph: {graph_path} ===")
    
    # Criar o container do ActionGraph com atributos obrigatórios
    graph_prim = stage.DefinePrim(graph_path, "OmniGraph")
    
    if not graph_prim:
        print(f"Erro ao criar ActionGraph em {graph_path}")
        return None
    
    # Configurar atributos do OmniGraph (baseado nos existentes)
    graph_prim.CreateAttribute("evaluationMode", Sdf.ValueTypeNames.Token).Set("Automatic")
    graph_prim.CreateAttribute("evaluator:type", Sdf.ValueTypeNames.Token).Set("execution")
    graph_prim.CreateAttribute("fabricCacheBacking", Sdf.ValueTypeNames.Token).Set("Shared")
    graph_prim.CreateAttribute("fileFormatVersion", Sdf.ValueTypeNames.Int2).Set(Gf.Vec2i(1, 9))
    graph_prim.CreateAttribute("pipelineStage", Sdf.ValueTypeNames.Token).Set("pipelineStageSimulation")
    
    # Posição inicial para layout dos nodes
    pos_x = -800
    pos_y = 0
    
    # 1. OnPlaybackTick (trigger) - compartilhado
    node = create_action_graph_node(stage, graph_path, "on_playback_tick", 
                             "omni.graph.action.OnPlaybackTick", 
                             node_version=2, position=(pos_x, pos_y))
    print(f"  ✓ Nó on_playback_tick criado")
    
    # 2. IsaacReadSimulationTime - compartilhado
    node = create_action_graph_node(stage, graph_path, "isaac_read_simulation_time",
                             "isaacsim.core.nodes.IsaacReadSimulationTime",
                             node_version=1, position=(pos_x, pos_y + 150))
    if node:
        set_node_input(node, "resetOnStop", True, Sdf.ValueTypeNames.Bool)
    print(f"  ✓ Nó isaac_read_simulation_time criado")
    
    # 3. ROS2Context - necessário para publicar em ROS2
    node = create_action_graph_node(stage, graph_path, "ros2_context",
                             "isaacsim.ros2.bridge.ROS2Context",
                             node_version=2, position=(pos_x - 200, pos_y + 100))
    if node:
        set_node_input(node, "domain_id", 0, Sdf.ValueTypeNames.Int)
        set_node_input(node, "useDomainIDEnvVar", False, Sdf.ValueTypeNames.Bool)
    print(f"  ✓ Nó ros2_context criado")
    
    pos_x += 250  # Mover para direita
    
    for idx, camera_path in enumerate(cameras_created):
        camera_name = camera_path.split("/")[-2]  # Nome da câmera (sem /Camera)
        ros_config = ROS2_CONFIG.get(camera_name, {})
        
        prefix = camera_name.replace("_fisheye", "")  # frontleft ou frontright
        row_offset = idx * 250  # Separar verticalmente cada câmera
        
        # 4. IsaacCreateRenderProduct
        rp_node = f"isaac_create_render_product_{prefix}"
        node = create_action_graph_node(stage, graph_path, rp_node,
                                        "isaacsim.core.nodes.IsaacCreateRenderProduct",
                                        node_version=2, position=(pos_x, pos_y + row_offset))
        if node:
            # Definir o caminho da câmera
            set_node_input(node, "cameraPrim", camera_path, Sdf.ValueTypeNames.Token)
            set_node_input(node, "enabled", True, Sdf.ValueTypeNames.Bool)
            set_node_input(node, "width", 640, Sdf.ValueTypeNames.Int)
            set_node_input(node, "height", 480, Sdf.ValueTypeNames.Int)
        print(f"  ✓ Nó {rp_node} criado")
        
        # 5. ROS2CameraHelper para RGB
        rgb_node = f"ros2_camera_helper_rgb_{prefix}"
        node = create_action_graph_node(stage, graph_path, rgb_node,
                                        "isaacsim.ros2.bridge.ROS2CameraHelper",
                                        node_version=2, position=(pos_x + 300, pos_y + row_offset))
        if node:
            set_node_input(node, "type", "rgb", Sdf.ValueTypeNames.Token)
            set_node_input(node, "topicName", ros_config.get("rgb_topic", f"/{camera_name}/image_raw"), Sdf.ValueTypeNames.String)
            set_node_input(node, "frameId", ros_config.get("frame_id", f"{camera_name}_optical"), Sdf.ValueTypeNames.String)
        print(f"  ✓ Nó {rgb_node} criado")
        
        # 6. ROS2CameraHelper para Depth
        depth_node = f"ros2_camera_helper_depth_{prefix}"
        node = create_action_graph_node(stage, graph_path, depth_node,
                                        "isaacsim.ros2.bridge.ROS2CameraHelper",
                                        node_version=2, position=(pos_x + 300, pos_y + row_offset + 100))
        if node:
            set_node_input(node, "type", "depth", Sdf.ValueTypeNames.Token)
            set_node_input(node, "topicName", ros_config.get("depth_topic", f"/{camera_name}/depth"), Sdf.ValueTypeNames.String)
            set_node_input(node, "frameId", ros_config.get("frame_id", f"{camera_name}_optical"), Sdf.ValueTypeNames.String)
        print(f"  ✓ Nó {depth_node} criado")
        
        # 7. ROS2CameraInfoHelper
        info_node = f"ros2_camera_info_helper_{prefix}"
        node = create_action_graph_node(stage, graph_path, info_node,
                                        "isaacsim.ros2.bridge.ROS2CameraInfoHelper",
                                        node_version=1, position=(pos_x + 600, pos_y + row_offset))
        if node:
            set_node_input(node, "topicName", ros_config.get("info_topic", f"/{camera_name}/camera_info"), Sdf.ValueTypeNames.String)
            set_node_input(node, "frameId", ros_config.get("frame_id", f"{camera_name}_optical"), Sdf.ValueTypeNames.String)
        print(f"  ✓ Nó {info_node} criado")
        
        # 8. ROS2PublishTransformTree
        tf_node = f"ros2_publish_transform_tree_{prefix}"
        node = create_action_graph_node(stage, graph_path, tf_node,
                                        "isaacsim.ros2.bridge.ROS2PublishTransformTree",
                                        node_version=1, position=(pos_x + 600, pos_y + row_offset + 100))
        if node:
            set_node_input(node, "staticPublisher", False, Sdf.ValueTypeNames.Bool)
            # targetPrims precisa ser relationship, não attribute simples
            # Por enquanto deixamos sem para o usuário conectar
        print(f"  ✓ Nó {tf_node} criado")
    
    print(f"\n✓ ActionGraph {graph_name} criado com sucesso!")
    print(f"\n⚠ NOTA: Os nodes foram criados mas as conexões precisam ser feitas manualmente no Isaac Sim.")
    print(f"  Passos:")
    print(f"  1. Abra o arquivo no Isaac Sim")
    print(f"  2. Vá para Window > Visual Scripting > Action Graph")
    print(f"  3. Selecione o graph '{graph_name}'")
    print(f"  4. Conecte os nodes:")
    print(f"     - on_playback_tick.tick -> isaac_create_render_product_*.execIn")
    print(f"     - isaac_create_render_product_*.renderProductPath -> ros2_camera_helper_*.renderProductPath")
    print(f"     - ros2_context.context -> ros2_camera_helper_*.context")
    return graph_path


def add_fisheye_cameras(usd_file, output_file=None):
    """
    Adiciona as câmeras fisheye ao arquivo USD.
    
    Args:
        usd_file: Caminho para o arquivo USD de entrada
        output_file: Caminho para salvar (se None, modifica in-place)
    """
    if not os.path.exists(usd_file):
        print(f"Erro: Arquivo não encontrado: {usd_file}")
        return False
    
    print(f"=== Abrindo USD: {usd_file} ===\n")
    
    stage = Usd.Stage.Open(usd_file)
    if not stage:
        print("Erro ao abrir o arquivo USD")
        return False
    
    # Encontrar o prim base do Spot
    base_path = "/Root/spot_with_arm/base"
    base_prim = stage.GetPrimAtPath(base_path)
    
    if not base_prim:
        # Tentar caminho alternativo
        alt_paths = [
            "/Root/spot_with_arm/spot/base",
            "/Root/spot/base",
            "/spot_with_arm/base",
        ]
        for alt_path in alt_paths:
            base_prim = stage.GetPrimAtPath(alt_path)
            if base_prim:
                base_path = alt_path
                break
    
    if not base_prim:
        print(f"Erro: Não foi possível encontrar o prim 'base' do Spot")
        print("Procurei em: /Root/spot_with_arm/base e alternativas")
        return False
    
    print(f"✓ Base do Spot encontrada: {base_path}\n")
    
    # Criar as câmeras
    cameras_created = []
    
    print("=== Criando Câmeras Fisheye ===")
    for camera_name, cam_config in CAMERA_TRANSFORMS.items():
        camera_path = create_camera_prim(stage, base_path, camera_name, cam_config)
        if camera_path:
            cameras_created.append(camera_path)
    
    if not cameras_created:
        print("Erro: Nenhuma câmera foi criada")
        return False
    
    # Criar ActionGraph para as câmeras
    graph_path = create_action_graph_for_cameras(stage, "ActionGraph_FisheyeCameras", cameras_created)
    
    # Salvar
    if output_file:
        save_path = output_file
    else:
        # Criar backup e salvar no original
        backup_path = usd_file + ".backup"
        if not os.path.exists(backup_path):
            import shutil
            shutil.copy2(usd_file, backup_path)
            print(f"\n✓ Backup criado: {backup_path}")
        save_path = usd_file
    
    stage.GetRootLayer().Export(save_path)
    print(f"\n✓ USD salvo: {save_path}")
    
    # Resumo
    print("\n" + "=" * 60)
    print("RESUMO")
    print("=" * 60)
    print(f"Câmeras criadas: {len(cameras_created)}")
    for cam in cameras_created:
        print(f"  - {cam}")
    if graph_path:
        print(f"\nActionGraph: {graph_path}")
        print("  Tópicos ROS2:")
        for cam_name, config in ROS2_CONFIG.items():
            print(f"    {cam_name}:")
            print(f"      RGB: {config['rgb_topic']}")
            print(f"      Depth: {config['depth_topic']}")
            print(f"      Info: {config['info_topic']}")
    
    return True


def main():
    parser = argparse.ArgumentParser(
        description="Adiciona câmeras fisheye ao Spot Robot em um arquivo USD"
    )
    parser.add_argument(
        "usd_file",
        help="Arquivo USD de entrada (ex: zed_streamer_warehouse.usd)"
    )
    parser.add_argument(
        "-o", "--output",
        help="Arquivo de saída (se não especificado, modifica o original)"
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="Apenas mostra o que seria feito, sem modificar arquivos"
    )
    
    args = parser.parse_args()
    
    if args.dry_run:
        print("=== DRY RUN MODE ===")
        print(f"Arquivo: {args.usd_file}")
        print(f"Output: {args.output or args.usd_file}")
        print("\nCâmeras que seriam criadas:")
        for name, config in CAMERA_TRANSFORMS.items():
            print(f"  {name}:")
            print(f"    Posição: {config['position']}")
            print(f"    Rotação: {config['rotation']}")
        print("\nActionGraph que seria criado: ActionGraph_FisheyeCameras")
        return
    
    success = add_fisheye_cameras(args.usd_file, args.output)
    sys.exit(0 if success else 1)


if __name__ == "__main__":
    main()
