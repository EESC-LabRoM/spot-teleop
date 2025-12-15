"""
EXEMPLO: Franka com RMPflow + Desvio de Obstáculos
====================================================

Este exemplo mostra como adicionar desvio de obstáculos ao RMPflow.
O robô vai AUTOMATICAMENTE desviar dos cubos enquanto tenta alcançar o target!

Conceitos demonstrados:
1. Como registrar obstáculos com RMPflow
2. Como os potential fields funcionam
3. Visualização das esferas de colisão (opcional)
"""

import numpy as np
import os

from isaacsim.core.utils.extensions import get_extension_path_from_name
from isaacsim.core.utils.stage import add_reference_to_stage
from isaacsim.core.prims import SingleArticulation as Articulation
from isaacsim.core.utils.nucleus import get_assets_root_path
from isaacsim.core.prims import SingleXFormPrim as XFormPrim
from isaacsim.core.utils.numpy.rotations import euler_angles_to_quats
from isaacsim.core.api.objects.cuboid import FixedCuboid

from isaacsim.robot_motion.motion_generation import RmpFlow, ArticulationMotionPolicy

class FrankaRmpFlowWithObstacles():
    def __init__(self):
        self._rmpflow = None
        self._articulation_rmpflow = None
        self._articulation = None
        self._target = None
        self._obstacles = []  # Lista de obstáculos
        self._is_initialized = False
        self._frame_count = 0
        self._physics_dt = 1.0 / 60.0

    def load_example_assets(self):
        """Carrega robô, target e OBSTÁCULOS"""
        
        # 1. Carrega o Franka
        robot_prim_path = "/panda"
        path_to_robot_usd = get_assets_root_path() + "/Isaac/Robots/FrankaRobotics/FrankaPanda/franka.usd"
        add_reference_to_stage(path_to_robot_usd, robot_prim_path)
        self._articulation = Articulation(robot_prim_path)

        # 2. Carrega o target
        add_reference_to_stage(get_assets_root_path() + "/Isaac/Props/UIElements/frame_prim.usd", "/World/target")
        self._target = XFormPrim("/World/target")
        self._target.set_local_scale(np.array([.04, .04, .04]))

        # 3. ADICIONA OBSTÁCULOS! 🚧
        # Obstáculo 1: Cubo VERMELHO entre o robô e o target
        obstacle1 = FixedCuboid(
            prim_path="/World/obstacle1",
            size=0.1,  # 10cm de lado
            position=np.array([0.4, 0.0, 0.5]),  # No caminho do robô
            color=np.array([1.0, 0.0, 0.0])  # Vermelho
        )
        self._obstacles.append(obstacle1)

        # Obstáculo 2: Cubo AZUL ao lado
        obstacle2 = FixedCuboid(
            prim_path="/World/obstacle2",
            size=0.08,
            position=np.array([0.3, 0.2, 0.6]),
            color=np.array([0.0, 0.0, 1.0])  # Azul
        )
        self._obstacles.append(obstacle2)

        # Obstáculo 3: Cubo VERDE embaixo
        obstacle3 = FixedCuboid(
            prim_path="/World/obstacle3",
            size=0.12,
            position=np.array([0.35, -0.15, 0.4]),
            color=np.array([0.0, 1.0, 0.0])  # Verde
        )
        self._obstacles.append(obstacle3)

        return self._articulation, self._target, *self._obstacles

    def setup(self, enable_debug_visualization=False):
        """
        Configura RMPflow e registra obstáculos
        
        Args:
            enable_debug_visualization: Se True, mostra esferas de colisão
        """
        mg_extension_path = get_extension_path_from_name("isaacsim.robot_motion.motion_generation")
        rmp_config_dir = os.path.join(mg_extension_path, "motion_policy_configs")

        # Cria o RMPflow
        self._rmpflow = RmpFlow(
            robot_description_path = rmp_config_dir + "/franka/rmpflow/robot_descriptor.yaml",
            urdf_path = rmp_config_dir + "/franka/lula_franka_gen.urdf",
            rmpflow_config_path = rmp_config_dir + "/franka/rmpflow/franka_rmpflow_common.yaml",
            end_effector_frame_name = "right_gripper",
            maximum_substep_size = 0.0025
        )

        # 🔥 REGISTRA CADA OBSTÁCULO COM RMPFLOW 🔥
        print("\n🚧 Registrando obstáculos com RMPflow:")
        for i, obstacle in enumerate(self._obstacles):
            self._rmpflow.add_obstacle(obstacle)
            pos = obstacle.get_world_pose()[0]
            print(f"   ✓ Obstáculo {i+1}: {obstacle.prim_path} em {pos}")
        
        print(f"\n💡 RMPflow agora vai DESVIAR de {len(self._obstacles)} obstáculos!")
        
        # Debug: Visualizar esferas de colisão (opcional)
        if enable_debug_visualization:
            self._rmpflow.visualize_collision_spheres()
            print("   🔍 Visualização de esferas de colisão ATIVADA")

        self._target.set_world_pose(np.array([.5, 0, .7]), euler_angles_to_quats([0, np.pi, 0]))

    def _initialize_robot(self):
        """Inicializa robô após physics começar"""
        if not self._is_initialized:
            self._articulation.initialize()
            self._articulation_rmpflow = ArticulationMotionPolicy(self._articulation, self._rmpflow)
            self._is_initialized = True
            print("[INFO] ✓ Robô inicializado com desvio de obstáculos ativo!")

    def update(self):
        """Loop de controle - roda a 60 Hz"""
        if not self._is_initialized:
            self._initialize_robot()
            return
        
        self._frame_count += 1
        
        # Pega posição do target
        target_position, target_orientation = self._target.get_world_pose()
        
        # ⚡ ATUALIZA ESTADO DO MUNDO ⚡
        # RMPflow consulta posição de TODOS os obstáculos aqui
        self._rmpflow.update_world()
        
        # Informa posição do robô
        robot_base_translation, robot_base_orientation = self._articulation.get_world_pose()
        self._rmpflow.set_robot_base_pose(robot_base_translation, robot_base_orientation)
        
        # Define target
        self._rmpflow.set_end_effector_target(target_position, target_orientation)

        # 🎯 CALCULA AÇÃO COM DESVIO DE OBSTÁCULOS 🎯
        # Internamente:
        # - Campo ATRATIVO puxa para o target
        # - Campos REPULSIVOS empurram longe dos obstáculos
        # - Combinação resulta em trajetória que desvia!
        action = self._articulation_rmpflow.get_next_articulation_action(self._physics_dt)
        self._articulation.apply_action(action)
        
        # Log com distância até target
        if self._frame_count % 60 == 0:
            ee_pos = self._articulation.get_world_pose()[0]  # Simplificação
            dist = np.linalg.norm(target_position - ee_pos)
            print(f"[Frame {self._frame_count}] Distância até target: {dist:.3f}m")

    def reset(self):
        """Reseta simulação"""
        self._target.set_world_pose(np.array([.5, 0, .7]), euler_angles_to_quats([0, np.pi, 0]))
        self._is_initialized = False
        self._frame_count = 0


# ==================== EXECUÇÃO ====================
if __name__ == "__main__":
    import omni.kit.app
    import omni.timeline
    from pxr import Usd
    import omni.physx
    
    global franka_example, physics_subscription
    
    franka_example = FrankaRmpFlowWithObstacles()
    
    # Carrega assets (robô + target + 3 obstáculos)
    assets = franka_example.load_example_assets()
    
    # Configurar RMPflow
    # MUDE PARA True PARA VER AS ESFERAS DE COLISÃO!
    franka_example.setup(enable_debug_visualization=False)
    
    timeline = omni.timeline.get_timeline_interface()
    physx_interface = omni.physx.get_physx_interface()
    
    error_count = [0]
    
    def on_physics_step(dt):
        if timeline.is_playing():
            try:
                franka_example.update()
                error_count[0] = 0
            except Exception as e:
                if error_count[0] < 3:
                    print(f"[ERROR] {e}")
                    import traceback
                    traceback.print_exc()
                    error_count[0] += 1
    
    physics_subscription = physx_interface.subscribe_physics_step_events(on_physics_step)
    
    print("\n" + "="*60)
    print("🤖 FRANKA COM DESVIO DE OBSTÁCULOS")
    print("="*60)
    print("\n✓ Robô Franka carregado")
    print("✓ Target (frame azul) em [0.5, 0, 0.7]")
    print("✓ 3 Obstáculos adicionados (vermelho, azul, verde)")
    print("✓ RMPflow configurado com desvio automático")
    print("\n📋 COMO FUNCIONA:")
    print("   • RMPflow cria campos REPULSIVOS ao redor de cada obstáculo")
    print("   • Campo ATRATIVO puxa o end effector para o target")
    print("   • Combinação resulta em trajetória que DESVIA automaticamente!")
    print("\n🎮 INSTRUÇÕES:")
    print("   1. Dê PLAY no timeline")
    print("   2. Observe o robô DESVIAR dos cubos coloridos!")
    print("   3. Mova o target - o robô sempre desviará dos obstáculos")
    print("   4. OPCIONAL: Mova os cubos - o robô se adapta em tempo real!")
    print("\n🔧 DEBUG:")
    print("   • Ver esferas de colisão: Edite setup(enable_debug_visualization=True)")
    print("   • Parar: physics_subscription.unsubscribe()")
    print("   • Reset: franka_example.reset()")
    print("="*60 + "\n")
