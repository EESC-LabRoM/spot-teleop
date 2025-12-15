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

class FrankaRmpFlowExample():
    def __init__(self):
        self._rmpflow = None
        self._articulation_rmpflow = None

        self._articulation = None
        self._target = None
        self._is_initialized = False
        self._frame_count = 0
        self._physics_dt = 1.0 / 60.0  # Standard Isaac Sim physics timestep

    def load_example_assets(self):
        # Add the Franka and target to the stage
        # The position in which things are loaded is also the position in which they

        robot_prim_path = "/panda"
        path_to_robot_usd = get_assets_root_path() + "/Isaac/Robots/FrankaRobotics/FrankaPanda/franka.usd"

        add_reference_to_stage(path_to_robot_usd, robot_prim_path)
        self._articulation = Articulation(robot_prim_path)

        add_reference_to_stage(get_assets_root_path() + "/Isaac/Props/UIElements/frame_prim.usd", "/World/target")
        self._target = XFormPrim("/World/target")
        self._target.set_local_scale(np.array([.04, .04, .04]))

        # Return assets that were added to the stage so that they can be registered with the core.World
        return self._articulation, self._target

    def setup(self):
        # RMPflow config files for supported robots are stored in the motion_generation extension under "/motion_policy_configs"
        mg_extension_path = get_extension_path_from_name("isaacsim.robot_motion.motion_generation")
        rmp_config_dir = os.path.join(mg_extension_path, "motion_policy_configs")

        # Initialize an RmpFlow object with smaller substep for smoother motion
        self._rmpflow = RmpFlow(
            robot_description_path = rmp_config_dir + "/franka/rmpflow/robot_descriptor.yaml",
            urdf_path = rmp_config_dir + "/franka/lula_franka_gen.urdf",
            rmpflow_config_path = rmp_config_dir + "/franka/rmpflow/franka_rmpflow_common.yaml",
            end_effector_frame_name = "right_gripper",
            maximum_substep_size = 0.0025  # Reduced for smoother trajectories (was 0.00334)
        )

        # Don't create ArticulationMotionPolicy yet - wait for initialization
        # self._articulation_rmpflow will be created in update() after first initialization

        self._target.set_world_pose(np.array([.5,0,.7]),euler_angles_to_quats([0,np.pi,0]))
    
    def _initialize_robot(self):
        """Initialize the robot articulation and motion policy"""
        if not self._is_initialized:
            # Initialize the articulation
            self._articulation.initialize()
            
            # Now create the ArticulationMotionPolicy
            self._articulation_rmpflow = ArticulationMotionPolicy(self._articulation, self._rmpflow)
            
            # Optional: Increase PD gains for faster/more responsive motion
            # Uncomment and adjust if you want even faster response
            # current_kp, current_kd = self._articulation.get_articulation_controller().get_gains()
            # self._articulation.get_articulation_controller().set_gains(
            #     kps=current_kp * 1.5,  # Increase proportional gain by 50%
            #     kds=current_kd * 1.2   # Increase derivative gain by 20%
            # )
            
            self._is_initialized = True
            print("[INFO] Robot articulation initialized successfully!")
            print("[INFO] Using enhanced update rate with physics callback")

    def update(self):
        # Use fixed physics timestep
        
        # Initialize robot on first update after timeline starts
        if not self._is_initialized:
            self._initialize_robot()
            return
        
        self._frame_count += 1
        
        # Get target position and orientation
        target_position, target_orientation = self._target.get_world_pose()
        
        # CRITICAL: Update RMPflow with current world state
        self._rmpflow.update_world()
        
        # CRITICAL: Inform RMPflow about robot base pose
        robot_base_translation, robot_base_orientation = self._articulation.get_world_pose()
        self._rmpflow.set_robot_base_pose(robot_base_translation, robot_base_orientation)
        
        # Set the end effector target
        self._rmpflow.set_end_effector_target(
            target_position, target_orientation
        )

        # Get and apply the action using fixed timestep
        action = self._articulation_rmpflow.get_next_articulation_action(self._physics_dt)
        self._articulation.apply_action(action)
        
        # Log occasionally (every 60 frames ~1 second at 60fps)
        if self._frame_count % 60 == 0:
            print(f"[UPDATE] Frame {self._frame_count}: Target at {target_position}")

    def reset(self):
        # Rmpflow is stateless unless it is explicitly told not to be
        self._target.set_world_pose(np.array([.5,0,.7]),euler_angles_to_quats([0,np.pi,0]))
        
        # Reset initialization flag so robot reinitializes on next update
        self._is_initialized = False
        self._frame_count = 0


# Executar quando o script for rodado
if __name__ == "__main__":
    import omni.kit.app
    import omni.timeline
    from pxr import Usd, PhysxSchema
    import omni.physx
    
    # Criar a instância global para poder acessar depois
    global franka_example, physics_subscription
    
    franka_example = FrankaRmpFlowExample()
    
    # Carregar os assets no stage
    articulation, target = franka_example.load_example_assets()
    
    # Configurar o RMPflow
    franka_example.setup()
    
    # Pegar interface do timeline
    timeline = omni.timeline.get_timeline_interface()
    
    # Get physics interface for better update rate
    physx_interface = omni.physx.get_physx_interface()
    
    # Contador para evitar spam de erros
    error_count = [0]
    
    # Registrar callback PHYSICS para sincronizar com steps de física
    def on_physics_step(dt):
        # Só executar se o timeline estiver rodando
        if timeline.is_playing():
            try:
                franka_example.update()
                # Reset error count on success
                error_count[0] = 0
            except Exception as e:
                # Mostrar apenas os primeiros erros para debug
                if error_count[0] < 3:
                    print(f"[ERROR] {e}")
                    import traceback
                    traceback.print_exc()
                    error_count[0] += 1
    
    # Subscribe to PHYSICS events for better sync and update rate
    physics_subscription = physx_interface.subscribe_physics_step_events(on_physics_step)
    
    print("✓ Franka carregado com sucesso!")
    print("✓ Target (frame azul) posicionado em [0.5, 0, 0.7]")
    print("✓ Update automático ativado com PHYSICS CALLBACK - resposta mais rápida!")
    print("✓ RMPflow configurado com substeps menores para trajetórias mais suaves")
    print("\nAgora:")
    print("  1. Dê PLAY no timeline")
    print("  2. Mova o target na viewport - o robô vai seguir automaticamente!")
    print("\nMelhorias aplicadas:")
    print("  • Physics callback (sincronizado com simulação)")
    print("  • Substep reduzido para 0.0025 (trajetórias mais suaves)")
    print("  • Taxa de update = taxa de physics (60 Hz)")
    print("\nPara parar o update: physics_subscription.unsubscribe()")
    print("Para reset: franka_example.reset()")