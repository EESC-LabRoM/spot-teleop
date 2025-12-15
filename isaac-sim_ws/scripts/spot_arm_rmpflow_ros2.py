"""
Spot Arm RMPflow + ROS2 Publisher
=================================

This script wires NVIDIA Isaac Sim's RMPflow motion policy to the Spot arm that is
already present on the stage under the prim path ``/Root/spot_with_arm``.  Instead
of applying the computed joint targets directly to the articulation, the results
are published on a ROS 2 ``sensor_msgs/JointState`` topic so that an existing
Action Graph / articulation controller can consume them.

How it works (high level):
1. Creates a target frame (``/Root/target``) plus a few fixed cuboid obstacles.
2. Loads the Spot arm RMPflow configuration (URDF + descriptor + gains).
3. Wraps the articulation with ``ArticulationMotionPolicy`` so RMPflow can read
   the current joint state straight from PhysX.
4. Each physics tick:
   • RMPflow updates world + target + obstacles
   • Computes joint targets with obstacle avoidance
   • Publishes the command on ``/spot/arm/joint_command`` (ROS 2)

Run it from the Isaac Sim Script Editor while the ROS 2 Bridge extension is
enabled.  Make sure the stage already contains the Spot with arm articulation
at ``/Root/spot_with_arm`` and that your Action Graph is listening to
``/spot/arm/joint_command``.
"""

import os
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped

from isaacsim.core.utils.stage import add_reference_to_stage
from isaacsim.core.utils.nucleus import get_assets_root_path
from isaacsim.core.prims import SingleArticulation as Articulation
from isaacsim.core.prims import SingleXFormPrim as XFormPrim
from isaacsim.core.api.objects.cuboid import FixedCuboid
from isaacsim.core.utils.numpy.rotations import euler_angles_to_quats

from isaacsim.robot_motion.motion_generation import RmpFlow, ArticulationMotionPolicy

import omni.timeline
import omni.physx
import omni.usd

# ---------------------------------------------------------------------------
# Configuration constants
# ---------------------------------------------------------------------------
ROBOT_PRIM_PATH = "/Root/spot_with_arm/base"
TARGET_PRIM_PATH = "/Root/target"
ROS_TOPIC_NAME = "/spot/arm/joint_command"
END_EFFECTOR_FRAME_NAME = "arm0_link_fngr"
ACTIVE_JOINT_NAMES = [
    "arm0_sh0",
    "arm0_sh1",
    "arm0_el0",
    "arm0_el1",
    "arm0_wr0",
    "arm0_wr1",
]

WORKSPACE_ROOT = os.environ.get("SPOT_RMPFLOW_ROOT", "/workspace")
ROBOT_DESCRIPTION_PATH = os.path.join(WORKSPACE_ROOT, "spot_robot_description_stage.yaml")
URDF_PATH = os.path.join(WORKSPACE_ROOT, "spot_description/urdf/standalone_arm.urdf")
RMPFLOW_CONFIG_PATH = os.path.join(WORKSPACE_ROOT, "spot_rmpflow.yaml")

# Obstacle positions relative to robot base
OBSTACLE_SPECS = [
    {"prim_path": "/Root/spot_obstacle_red", "size": 0.14, "offset": [0.55, 0.0, 0.65], "color": [1.0, 0.25, 0.25]},
    {"prim_path": "/Root/spot_obstacle_blue", "size": 0.12, "offset": [0.45, -0.25, 0.55], "color": [0.25, 0.45, 1.0]},
    {"prim_path": "/Root/spot_obstacle_green", "size": 0.16, "offset": [0.35, 0.25, 0.5], "color": [0.25, 1.0, 0.45]},
]

# Target position relative to robot base
TARGET_RELATIVE_OFFSET = np.array([0.65, 0.0, 0.5])
TARGET_DEFAULT_ORIENTATION = euler_angles_to_quats([0.0, 0.0, 0.0])  # Fixed: was [0, π, 0] causing upside-down orientation

# ---------------------------------------------------------------------------
# ROS 2 helper
# ---------------------------------------------------------------------------
class JointStatePublisher(Node):
    """Minimal ROS 2 publisher that sends JointState messages and subscribes to relative target commands."""

    def __init__(self, topic_name: str, joint_names: list[str]):
        super().__init__("spot_rmpflow_policy")
        self._joint_names = joint_names
        self._publisher = self.create_publisher(JointState, topic_name, 10)
        
        # Subscribe to relative target commands
        self._target_subscriber = self.create_subscription(
            PoseStamped,
            "/spot/arm/target_relative",
            self._target_callback,
            10
        )
        self._relative_target_pose = None
        self._last_target_time = None

    def _target_callback(self, msg: PoseStamped):
        """Receive target pose relative to robot base."""
        self._relative_target_pose = msg
        self._last_target_time = self.get_clock().now()
        
    def get_relative_target(self):
        """Get the most recent relative target command, or None."""
        return self._relative_target_pose

    def publish_command(self, joint_positions, joint_velocities=None):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self._joint_names
        msg.position = joint_positions.tolist()
        if joint_velocities is not None:
            msg.velocity = joint_velocities.tolist()
        self._publisher.publish(msg)

# ---------------------------------------------------------------------------
# Main example class
# ---------------------------------------------------------------------------
class SpotArmRmpFlowRos2:
    def __init__(self):
        self._articulation = None
        self._target = None
        self._obstacles = []
        self._rmpflow = None
        self._articulation_rmpflow = None
        self._physics_dt = 1.0 / 60.0
        self._is_initialized = False
        self._frame_count = 0
        self._init_delay_frames = 10  # Wait more frames for articulation to be ready

        self._ros_node: JointStatePublisher | None = None
        self._ros_initialized = False

    # ------------------------------------------------------------------
    # Setup helpers
    # ------------------------------------------------------------------
    def load_assets(self):
        """Attach to the Spot arm articulation and make sure target/obstacles exist."""
        # Verify robot exists on stage
        stage = omni.usd.get_context().get_stage()
        robot_prim = stage.GetPrimAtPath(ROBOT_PRIM_PATH)
        
        if not robot_prim or not robot_prim.IsValid():
            raise RuntimeError(
                f"Robot articulation root not found at {ROBOT_PRIM_PATH}!\n"
                f"Please ensure the Spot robot base is at this path."
            )
        
        print(f"[SpotRMPflow] ✓ Using articulation root at {ROBOT_PRIM_PATH}")
        
        self._articulation = Articulation(ROBOT_PRIM_PATH)
        self._target = self._ensure_target_prim()
        # Obstacles will be created in setup() after we have robot position
        return self._articulation, self._target

    def setup(self):
        """Configure RMPflow and register obstacles."""
        self._validate_config_files()
        self._rmpflow = RmpFlow(
            robot_description_path=ROBOT_DESCRIPTION_PATH,
            urdf_path=URDF_PATH,
            rmpflow_config_path=RMPFLOW_CONFIG_PATH,
            end_effector_frame_name=END_EFFECTOR_FRAME_NAME,
            maximum_substep_size=0.0025,
        )

        # Get robot base position for relative positioning
        robot_base_position, _ = self._articulation.get_world_pose()
        print(f"[SpotRMPflow] Robot base at: {robot_base_position}")
        
        # Create obstacles relative to robot base
        self._obstacles = self._create_obstacles(robot_base_position)
        for obstacle in self._obstacles:
            self._rmpflow.add_obstacle(obstacle)

        # Position target relative to robot base
        target_position = robot_base_position + TARGET_RELATIVE_OFFSET
        print(f"[SpotRMPflow] Target positioned at: {target_position} (offset: {TARGET_RELATIVE_OFFSET})")
        
        self._target.set_world_pose(target_position, TARGET_DEFAULT_ORIENTATION)

    def _ensure_target_prim(self):
        target = XFormPrim(TARGET_PRIM_PATH)
        if not target.is_valid():
            frame_asset = get_assets_root_path() + "/Isaac/Props/UIElements/frame_prim.usd"
            add_reference_to_stage(frame_asset, TARGET_PRIM_PATH)
            target = XFormPrim(TARGET_PRIM_PATH)
        target.set_local_scale(np.array([0.04, 0.04, 0.04]))
        return target

    def _create_obstacles(self, robot_base_position):
        """Create obstacles at positions relative to robot base."""
        obstacles = []
        for spec in OBSTACLE_SPECS:
            # Calculate absolute position from robot base + offset
            absolute_position = robot_base_position + np.array(spec["offset"], dtype=float)
            
            obstacle = FixedCuboid(
                prim_path=spec["prim_path"],
                size=spec["size"],
                position=absolute_position,
                color=np.array(spec["color"], dtype=float),
            )
            obstacles.append(obstacle)
            print(f"[SpotRMPflow]   • Obstacle at {spec['prim_path']}: offset {spec['offset']} → position {absolute_position}")
        return obstacles

    def _validate_config_files(self):
        missing = [
            path
            for path in (ROBOT_DESCRIPTION_PATH, URDF_PATH, RMPFLOW_CONFIG_PATH)
            if not os.path.exists(path)
        ]
        if missing:
            raise FileNotFoundError(f"Missing RMPflow config files: {missing}")

    def _initialize_robot(self):
        """Initialize the robot articulation and motion policy after physics is ready."""
        if self._is_initialized:
            return
        
        print("[SpotRMPflow] Attempting to initialize robot...")
        
        if self._articulation is None:
            print("[SpotRMPflow][ERROR] Articulation is None!")
            return
            
        print("[SpotRMPflow] Calling articulation.initialize()...")
        self._articulation.initialize()
        
        # Validate that articulation has DOFs
        num_dof = self._articulation.num_dof
        dof_names = self._articulation.dof_names
        
        print(f"[SpotRMPflow] Articulation has {num_dof} DOFs")
        print(f"[SpotRMPflow] DOF names: {dof_names}")
        
        if num_dof == 0:
            raise RuntimeError(
                f"Articulation at {ROBOT_PRIM_PATH} has 0 DOFs! "
                "Please ensure:\n"
                "  1. The robot USD has proper articulation/physics setup\n"
                "  2. The articulation root prim is correct\n"
                "  3. The robot has been properly loaded to the stage\n"
                f"  Expected joints: {ACTIVE_JOINT_NAMES}"
            )
        
        # Verify that our expected joints exist
        missing_joints = [name for name in ACTIVE_JOINT_NAMES if name not in dof_names]
        if missing_joints:
            print(f"[SpotRMPflow][WARNING] Expected joints not found: {missing_joints}")
            print(f"[SpotRMPflow][WARNING] Available joints: {dof_names}")
        
        print("[SpotRMPflow] Creating ArticulationMotionPolicy...")
        self._articulation_rmpflow = ArticulationMotionPolicy(
            self._articulation, self._rmpflow, default_physics_dt=self._physics_dt
        )
        
        self._is_initialized = True
        print("[SpotRMPflow] ✓ Articulation initialized and motion policy bound.")
        print(f"[SpotRMPflow] ✓ Motion policy tracking {num_dof} active joints")

    def _ensure_ros(self):
        if self._ros_initialized:
            return
        if not rclpy.ok():
            rclpy.init(args=None)
        self._ros_node = JointStatePublisher(ROS_TOPIC_NAME, ACTIVE_JOINT_NAMES)
        self._ros_initialized = True
        print(f"[SpotRMPflow] Publishing joint commands on '{ROS_TOPIC_NAME}'.")
        print(f"[SpotRMPflow] Listening for relative targets on '/spot/arm/target_relative'.")

    def _convert_relative_to_world(self, relative_pose_msg):
        """Convert relative PoseStamped to world coordinates."""
        base_translation, base_orientation = self._articulation.get_world_pose()
        
        # Extract relative position
        rel_pos = np.array([
            relative_pose_msg.pose.position.x,
            relative_pose_msg.pose.position.y,
            relative_pose_msg.pose.position.z
        ])
        
        # Extract relative orientation (quaternion: x, y, z, w)
        rel_quat = np.array([
            relative_pose_msg.pose.orientation.w,  # Isaac Sim uses w, x, y, z
            relative_pose_msg.pose.orientation.x,
            relative_pose_msg.pose.orientation.y,
            relative_pose_msg.pose.orientation.z
        ])
        
        # Transform position to world space (simple translation for now)
        # TODO: For full rotation support, multiply position by base rotation matrix
        world_position = base_translation + rel_pos
        
        # Transform orientation to world space (quaternion multiplication)
        # For now, using relative orientation directly
        # TODO: Multiply base_orientation * rel_quat for full relative orientation
        world_orientation = rel_quat
        
        return world_position, world_orientation

    # ------------------------------------------------------------------
    # Main update
    # ------------------------------------------------------------------
    def update(self, physics_dt: float):
        # Wait several frames for physics and articulation to be ready
        if not self._is_initialized:
            self._frame_count += 1
            if self._frame_count < self._init_delay_frames:
                if self._frame_count == 1:
                    print(f"[SpotRMPflow] Waiting {self._init_delay_frames} frames for articulation to be ready...")
                return
            self._initialize_robot()
            return

        self._ensure_ros()

        # Check if we have a relative target command via ROS2
        relative_target = self._ros_node.get_relative_target() if self._ros_node else None
        
        if relative_target is not None:
            # Use relative target from ROS2 (converted to world space)
            target_position, target_orientation = self._convert_relative_to_world(relative_target)
            # Update visual target prim for debugging
            self._target.set_world_pose(target_position, target_orientation)
        else:
            # Use manual target prim position (original behavior)
            target_position, target_orientation = self._target.get_world_pose()
        
        self._rmpflow.update_world()
        base_translation, base_orientation = self._articulation.get_world_pose()
        self._rmpflow.set_robot_base_pose(base_translation, base_orientation)
        self._rmpflow.set_end_effector_target(target_position, target_orientation)

        action = self._articulation_rmpflow.get_next_articulation_action(physics_dt or self._physics_dt)
        self._publish_action(action)

        self._frame_count += 1
        if self._frame_count % 120 == 0:
            ee_translation = self._articulation.get_world_pose()[0]
            dist = np.linalg.norm(target_position - ee_translation)
            mode = "RELATIVE" if relative_target else "MANUAL"
            print(f"[SpotRMPflow] Frame {self._frame_count} [{mode}]: target distance {dist:.3f} m")

    def _publish_action(self, action):
        if self._ros_node is None:
            return
        joint_positions = action.joint_positions
        joint_velocities = action.joint_velocities
        self._ros_node.publish_command(joint_positions, joint_velocities)
        rclpy.spin_once(self._ros_node, timeout_sec=0.0)

    def shutdown(self):
        if self._ros_node is not None:
            self._ros_node.destroy_node()
            self._ros_node = None
        if rclpy.ok():
            rclpy.shutdown()
        print("[SpotRMPflow] Shutdown complete.")



# ---------------------------------------------------------------------------
# Entrypoint for Script Editor
# ---------------------------------------------------------------------------
def _main():
    example = SpotArmRmpFlowRos2()
    example.load_assets()
    example.setup()

    timeline = omni.timeline.get_timeline_interface()
    physx_interface = omni.physx.get_physx_interface()
    error_count = [0]

    def on_physics_step(dt):
        if not timeline.is_playing():
            return
        try:
            example.update(dt or example._physics_dt)
            error_count[0] = 0
        except Exception as exc:  # noqa: BLE001
            error_count[0] += 1
            print(f"[SpotRMPflow][ERROR] {exc}")
            if error_count[0] >= 3:
                print("[SpotRMPflow] Too many errors, unsubscribing from physics updates.")
                physics_subscription.unsubscribe()
                example.shutdown()

    physics_subscription = physx_interface.subscribe_physics_step_events(on_physics_step)

    print("\n" + "=" * 72)
    print("Spot Arm RMPflow + ROS2 Publisher")
    print("=" * 72)
    print(f"• Robot prim        : {ROBOT_PRIM_PATH}")
    print(f"• Joint topic       : {ROS_TOPIC_NAME}")
    print("• Target prim       : /Root/target (move it in the viewport!)")
    print("• Obstacles         : 3 FixedCuboid prims under /Root")
    print("• RMPflow configs   :")
    print(f"    - {ROBOT_DESCRIPTION_PATH}")
    print(f"    - {URDF_PATH}")
    print(f"    - {RMPFLOW_CONFIG_PATH}")
    print("\nModes:")
    print("  1. MANUAL MODE: Move the target frame with the gizmo in viewport")
    print("  2. RELATIVE MODE: Publish to /spot/arm/target_relative (geometry_msgs/PoseStamped)")
    print("     - Position is relative to robot base (x: forward, y: left, z: up)")
    print("     - Example: [0.5, 0.0, 0.3] = 50cm forward, 30cm up from base")
    print("\nControls:")
    print("  1. Press PLAY on the timeline.")
    print("  2a. Move the target frame with the gizmo (manual mode)")
    print("  2b. OR publish relative commands via ROS2 (relative mode)")
    print("  3. Your Action Graph should forward /spot/arm/joint_command to the articulation controller.")
    print("  4. Call physics_subscription.unsubscribe() and example.shutdown() to stop.")
    print("=" * 72 + "\n")

# Execute when run inside Script Editor
if __name__ == "__main__":
    _main()
