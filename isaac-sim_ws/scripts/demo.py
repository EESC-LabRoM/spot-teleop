import asyncio
import os
import random
import time
from itertools import chain

import carb.settings
import numpy as np
import omni.kit.app
import omni.replicator.core as rep
import omni.timeline
import omni.usd
import usdrt
from isaacsim.core.utils.semantics import add_labels, remove_labels, upgrade_prim_semantics_to_labels
from isaacsim.storage.native import get_assets_root_path
from omni.kit.viewport.utility import get_active_viewport
from omni.physx import get_physx_interface, get_physx_scene_query_interface
from pxr import Gf, PhysxSchema, Sdf, Usd, UsdGeom, UsdPhysics


# Add transformation properties to the prim (if not already present)
def set_transform_attributes(prim, location=None, orientation=None, rotation=None, scale=None):
    if location is not None:
        if not prim.HasAttribute("xformOp:translate"):
            UsdGeom.Xformable(prim).AddTranslateOp()
        prim.GetAttribute("xformOp:translate").Set(location)
    if orientation is not None:
        if not prim.HasAttribute("xformOp:orient"):
            UsdGeom.Xformable(prim).AddOrientOp()
        prim.GetAttribute("xformOp:orient").Set(orientation)
    if rotation is not None:
        if not prim.HasAttribute("xformOp:rotateXYZ"):
            UsdGeom.Xformable(prim).AddRotateXYZOp()
        prim.GetAttribute("xformOp:rotateXYZ").Set(rotation)
    if scale is not None:
        if not prim.HasAttribute("xformOp:scale"):
            UsdGeom.Xformable(prim).AddScaleOp()
        prim.GetAttribute("xformOp:scale").Set(scale)


# Enables collisions with the asset (without rigid body dynamics the asset will be static)
def add_colliders(root_prim):
    # Iterate descendant prims (including root) and add colliders to mesh or primitive types
    for desc_prim in Usd.PrimRange(root_prim):
        if desc_prim.IsA(UsdGeom.Mesh) or desc_prim.IsA(UsdGeom.Gprim):
            # Physics
            if not desc_prim.HasAPI(UsdPhysics.CollisionAPI):
                collision_api = UsdPhysics.CollisionAPI.Apply(desc_prim)
            else:
                collision_api = UsdPhysics.CollisionAPI(desc_prim)
            collision_api.CreateCollisionEnabledAttr(True)
            # PhysX
            if not desc_prim.HasAPI(PhysxSchema.PhysxCollisionAPI):
                physx_collision_api = PhysxSchema.PhysxCollisionAPI.Apply(desc_prim)
            else:
                physx_collision_api = PhysxSchema.PhysxCollisionAPI(desc_prim)
            # Set PhysX specific properties
            physx_collision_api.CreateContactOffsetAttr(0.001)
            physx_collision_api.CreateRestOffsetAttr(0.0)

        # Add mesh specific collision properties only to mesh types
        if desc_prim.IsA(UsdGeom.Mesh):
            # Add mesh collision properties to the mesh (e.g. collider approximation type)
            if not desc_prim.HasAPI(UsdPhysics.MeshCollisionAPI):
                mesh_collision_api = UsdPhysics.MeshCollisionAPI.Apply(desc_prim)
            else:
                mesh_collision_api = UsdPhysics.MeshCollisionAPI(desc_prim)
            mesh_collision_api.CreateApproximationAttr().Set("convexHull")


# Check if prim (or its descendants) has colliders
def has_colliders(root_prim):
    for desc_prim in Usd.PrimRange(root_prim):
        if desc_prim.HasAPI(UsdPhysics.CollisionAPI):
            return True
    return False


# Enables rigid body dynamics (physics simulation) on the prim
def add_rigid_body_dynamics(prim, disable_gravity=False, angular_damping=None):
    if has_colliders(prim):
        # Physics
        if not prim.HasAPI(UsdPhysics.RigidBodyAPI):
            rigid_body_api = UsdPhysics.RigidBodyAPI.Apply(prim)
        else:
            rigid_body_api = UsdPhysics.RigidBodyAPI(prim)
        rigid_body_api.CreateRigidBodyEnabledAttr(True)
        # PhysX
        if not prim.HasAPI(PhysxSchema.PhysxRigidBodyAPI):
            physx_rigid_body_api = PhysxSchema.PhysxRigidBodyAPI.Apply(prim)
        else:
            physx_rigid_body_api = PhysxSchema.PhysxRigidBodyAPI(prim)
        physx_rigid_body_api.GetDisableGravityAttr().Set(disable_gravity)
        if angular_damping is not None:
            physx_rigid_body_api.CreateAngularDampingAttr().Set(angular_damping)
    else:
        print(f"Prim '{prim.GetPath()}' has no colliders. Skipping rigid body dynamics properties.")


# Add dynamics properties to the prim (if mesh or primitive) (rigid body to root + colliders to the meshes)
# https://docs.omniverse.nvidia.com/kit/docs/omni_physics/latest/dev_guide/rigid_bodies_articulations/rigid_bodies.html#configure-a-rigid-body-with-multiple-colliders
def add_colliders_and_rigid_body_dynamics(prim, disable_gravity=False):
    # Add colliders to mesh or primitive types of the descendants of the prim (including root)
    add_colliders(prim)
    # Add rigid body dynamics properties (to the root only) only if it has colliders
    add_rigid_body_dynamics(prim, disable_gravity=disable_gravity)


# Creates collision box area wrapping the given working area with origin in (0, 0, 0) with thickness towards outside
def create_collision_box_walls(stage, path, width, depth, height, thickness=0.5, visible=False):
    # Define the walls (name, location, size) with thickness towards the outside of the working area
    walls = [
        ("floor", (0, 0, (height + thickness) / -2.0), (width, depth, thickness)),
        ("ceiling", (0, 0, (height + thickness) / 2.0), (width, depth, thickness)),
        ("left_wall", ((width + thickness) / -2.0, 0, 0), (thickness, depth, height)),
        ("right_wall", ((width + thickness) / 2.0, 0, 0), (thickness, depth, height)),
        ("front_wall", (0, (depth + thickness) / 2.0, 0), (width, thickness, height)),
        ("back_wall", (0, (depth + thickness) / -2.0, 0), (width, thickness, height)),
    ]
    for name, location, size in walls:
        prim = stage.DefinePrim(f"{path}/{name}", "Cube")
        scale = (size[0] / 2.0, size[1] / 2.0, size[2] / 2.0)
        set_transform_attributes(prim, location=location, scale=scale)
        add_colliders(prim)
        if not visible:
            UsdGeom.Imageable(prim).MakeInvisible()


# Create a random transformation values for location, rotation, and scale
def get_random_transform_values(
    loc_min=(0, 0, 0), loc_max=(1, 1, 1), rot_min=(0, 0, 0), rot_max=(360, 360, 360), scale_min_max=(0.1, 1.0)
):
    location = (
        random.uniform(loc_min[0], loc_max[0]),
        random.uniform(loc_min[1], loc_max[1]),
        random.uniform(loc_min[2], loc_max[2]),
    )
    rotation = (
        random.uniform(rot_min[0], rot_max[0]),
        random.uniform(rot_min[1], rot_max[1]),
        random.uniform(rot_min[2], rot_max[2]),
    )
    scale = tuple([random.uniform(scale_min_max[0], scale_min_max[1])] * 3)
    return location, rotation, scale


# Generate a random pose on a sphere looking at the origin
# https://docs.omniverse.nvidia.com/isaacsim/latest/reference_conventions.html
def get_random_pose_on_sphere(origin, radius, camera_forward_axis=(0, 0, -1), phi_range=(-1, 1), theta_range=(0, 2 * np.pi)):
    origin = Gf.Vec3f(origin)
    camera_forward_axis = Gf.Vec3f(camera_forward_axis)

    # Generate random angles for spherical coordinates with configurable ranges
    # theta: azimuth angle (horizontal rotation), theta_range in radians
    # phi: elevation angle (vertical), phi_range as sin(phi) for uniform distribution
    theta = np.random.uniform(theta_range[0], theta_range[1])
    phi = np.arcsin(np.random.uniform(phi_range[0], phi_range[1]))

    # Spherical to Cartesian conversion
    x = radius * np.cos(theta) * np.cos(phi)
    y = radius * np.sin(phi)
    z = radius * np.sin(theta) * np.cos(phi)

    location = origin + Gf.Vec3f(x, y, z)

    # Calculate direction vector from camera to look_at point
    direction = origin - location
    direction_normalized = direction.GetNormalized()

    # Calculate rotation from forward direction (rotateFrom) to direction vector (rotateTo)
    rotation = Gf.Rotation(Gf.Vec3d(camera_forward_axis), Gf.Vec3d(direction_normalized))
    orientation = Gf.Quatf(rotation.GetQuat())

    return location, orientation


# Enable or disable the render products and viewport rendering
def set_render_products_updates(render_products, enabled, include_viewport=False):
    for rp in render_products:
        rp.hydra_texture.set_updates_enabled(enabled)
    if include_viewport:
        get_active_viewport().updates_enabled = enabled


async def run_example_async(config):
    # Isaac nucleus assets root path
    assets_root_path = get_assets_root_path()
    stage = None

    # ENVIRONMENT
    # Create an empty or load a custom stage (clearing any previous semantics)
    env_url = config.get("env_url", "")
    if env_url:
        env_path = env_url if env_url.startswith("omniverse://") else assets_root_path + env_url
        omni.usd.get_context().open_stage(env_path)
        stage = omni.usd.get_context().get_stage()
        # Remove any previous semantics in the loaded stage
        for prim in stage.Traverse():
            # Make sure old semantics api are upgraded to the new labels api
            upgrade_prim_semantics_to_labels(prim, include_descendants=True)
            remove_labels(prim, include_descendants=True)
    else:
        omni.usd.get_context().new_stage()
        stage = omni.usd.get_context().get_stage()
        # Add a distant light to the empty stage
        distant_light = stage.DefinePrim("/World/Lights/DistantLight", "DistantLight")
        distant_light.CreateAttribute("inputs:intensity", Sdf.ValueTypeNames.Float).Set(400.0)
        if not distant_light.HasAttribute("xformOp:rotateXYZ"):
            UsdGeom.Xformable(distant_light).AddRotateXYZOp()
        distant_light.GetAttribute("xformOp:rotateXYZ").Set((0, 60, 0))

    # Get the working area size and bounds (width=x, depth=y, height=z)
    working_area_size = config.get("working_area_size", (3, 3, 3))
    working_area_min = (working_area_size[0] / -2, working_area_size[1] / -2, working_area_size[2] / -2)
    working_area_max = (working_area_size[0] / 2, working_area_size[1] / 2, working_area_size[2] / 2)

    # Create a collision box area around the assets to prevent them from drifting away
    create_collision_box_walls(
        stage, "/World/CollisionWalls", working_area_size[0], working_area_size[1], working_area_size[2]
    )

    # Create a physics scene to add or modify custom physics settings
    usdrt_stage = usdrt.Usd.Stage.Attach(omni.usd.get_context().get_stage_id())
    physics_scenes = usdrt_stage.GetPrimsWithAppliedAPIName("PhysxSceneAPI")
    if physics_scenes:
        physics_scene = physics_scenes[0]
    else:
        physics_scene = UsdPhysics.Scene.Define(stage, "/PhysicsScene")
        physx_scene = PhysxSchema.PhysxSceneAPI.Apply(stage.GetPrimAtPath("/PhysicsScene"))
    physx_scene.GetTimeStepsPerSecondAttr().Set(60)

    # TRAINING ASSETS
    # Add the objects to be trained in the environment with their labels and properties
    labeled_assets_and_properties = config.get("labeled_assets_and_properties", [])
    floating_labeled_prims = []
    falling_labeled_prims = []
    labeled_prims = []
    for obj in labeled_assets_and_properties:
        obj_url = obj.get("url", "")
        label = obj.get("label", "unknown")
        count = obj.get("count", 1)
        floating = obj.get("floating", False)
        scale_min_max = obj.get("randomize_scale", (1, 1))
        has_articulation = obj.get("has_articulation", False)
        label_prim_path = obj.get("label_prim_path", None)
        base_scale = obj.get("base_scale", 1.0)  # Unit conversion factor (e.g., 0.001 for mm to m)
        for i in range(count):
            # Create a prim and add the asset reference
            rand_loc, rand_rot, rand_scale = get_random_transform_values(
                loc_min=working_area_min, loc_max=working_area_max, scale_min_max=scale_min_max
            )
            # Apply base_scale to convert units (e.g., mm to meters) before randomization
            final_scale = tuple(s * base_scale for s in rand_scale)
            prim_path = omni.usd.get_stage_next_free_path(stage, f"/World/Labeled/{label}", False)
            prim = stage.DefinePrim(prim_path, "Xform")
            # Handle different path types: omniverse://, absolute filesystem path, or relative Isaac asset path
            if obj_url.startswith("omniverse://"):
                asset_path = obj_url
            elif obj_url.startswith("/"):
                # Absolute filesystem path (e.g., /workspace/...)
                asset_path = obj_url
            else:
                # Relative Isaac asset path
                asset_path = assets_root_path + obj_url
            prim.GetReferences().AddReference(asset_path)
            set_transform_attributes(prim, location=rand_loc, rotation=rand_rot, scale=final_scale)
            
            # Wait for reference to load before accessing child prims (multiple frames for complex USDs)
            for _ in range(3):
                await omni.kit.app.get_app().next_update_async()
            
            add_colliders(prim)
            # Only add rigid body if not an articulation (articulations handle their own rigid bodies)
            if not has_articulation:
                add_rigid_body_dynamics(prim, disable_gravity=floating)
            # Label the asset - if label_prim_path is specified, label only that specific prim
            if label_prim_path:
                # Navigate to the specific prim within the loaded USD
                target_prim = stage.GetPrimAtPath(prim.GetPath().AppendPath(label_prim_path))
                if target_prim and target_prim.IsValid():
                    add_labels(target_prim, labels=[label], instance_name="class")
                    print(f"✓ Labeled prim: {target_prim.GetPath()}")
                else:
                    # Fallback: search by prim name (last component of the path)
                    target_name = label_prim_path.split("/")[-1]
                    found = False
                    for child_prim in Usd.PrimRange(prim):
                        if child_prim.GetName() == target_name:
                            add_labels(child_prim, labels=[label], instance_name="class")
                            print(f"✓ Found and labeled prim by name '{target_name}': {child_prim.GetPath()}")
                            found = True
                            break
                    if not found:
                        print(f"Warning: Could not find prim with name '{target_name}' (from path {label_prim_path}), labeling root instead")
                        add_labels(prim, labels=[label], instance_name="class")
            else:
                # Label the entire asset (any previous 'class' label will be overwritten)
                add_labels(prim, labels=[label], instance_name="class")
            if floating:
                floating_labeled_prims.append(prim)
            else:
                falling_labeled_prims.append(prim)
    labeled_prims = floating_labeled_prims + falling_labeled_prims

    # DISTRACTORS
    # Add shape distractors to the environment as floating or falling objects
    shape_distractors_types = config.get("shape_distractors_types", ["capsule", "cone", "cylinder", "sphere", "cube"])
    shape_distractors_scale_min_max = config.get("shape_distractors_scale_min_max", (0.02, 0.2))
    shape_distractors_num = config.get("shape_distractors_num", 350)
    shape_distractors = []
    floating_shape_distractors = []
    falling_shape_distractors = []
    for i in range(shape_distractors_num):
        rand_loc, rand_rot, rand_scale = get_random_transform_values(
            loc_min=working_area_min, loc_max=working_area_max, scale_min_max=shape_distractors_scale_min_max
        )
        rand_shape = random.choice(shape_distractors_types)
        prim_path = omni.usd.get_stage_next_free_path(stage, f"/World/Distractors/{rand_shape}", False)
        prim = stage.DefinePrim(prim_path, rand_shape.capitalize())
        set_transform_attributes(prim, location=rand_loc, rotation=rand_rot, scale=rand_scale)
        add_colliders(prim)
        disable_gravity = random.choice([True, False])
        add_rigid_body_dynamics(prim, disable_gravity)
        if disable_gravity:
            floating_shape_distractors.append(prim)
        else:
            falling_shape_distractors.append(prim)
        shape_distractors.append(prim)

    # Add mesh distractors to the environment as floating of falling objects
    mesh_distactors_urls = config.get("mesh_distractors_urls", [])
    mesh_distactors_scale_min_max = config.get("mesh_distractors_scale_min_max", (0.1, 2.0))
    mesh_distactors_num = config.get("mesh_distractors_num", 10)
    mesh_distractors = []
    floating_mesh_distractors = []
    falling_mesh_distractors = []
    for i in range(mesh_distactors_num):
        rand_loc, rand_rot, rand_scale = get_random_transform_values(
            loc_min=working_area_min, loc_max=working_area_max, scale_min_max=mesh_distactors_scale_min_max
        )
        mesh_url = random.choice(mesh_distactors_urls)
        prim_name = os.path.basename(mesh_url).split(".")[0]
        prim_path = omni.usd.get_stage_next_free_path(stage, f"/World/Distractors/{prim_name}", False)
        prim = stage.DefinePrim(prim_path, "Xform")
        asset_path = mesh_url if mesh_url.startswith("omniverse://") else assets_root_path + mesh_url
        prim.GetReferences().AddReference(asset_path)
        set_transform_attributes(prim, location=rand_loc, rotation=rand_rot, scale=rand_scale)
        add_colliders(prim)
        disable_gravity = random.choice([True, False])
        add_rigid_body_dynamics(prim, disable_gravity=disable_gravity)
        if disable_gravity:
            floating_mesh_distractors.append(prim)
        else:
            falling_mesh_distractors.append(prim)
        # Remove any previous semantics on the mesh distractor
        upgrade_prim_semantics_to_labels(prim, include_descendants=True)
        remove_labels(prim, include_descendants=True)

    # REPLICATOR
    # Disable capturing every frame (capture will be triggered manually using the step function)
    rep.orchestrator.set_capture_on_play(False)

    # Create the camera prims and their properties
    cameras = []
    num_cameras = config.get("num_cameras", 1)
    camera_properties_kwargs = config.get("camera_properties_kwargs", {})
    for i in range(num_cameras):
        # Create camera and add its properties (focal length, focus distance, f-stop, clipping range, etc.)
        cam_prim = stage.DefinePrim(f"/World/Cameras/cam_{i}", "Camera")
        for key, value in camera_properties_kwargs.items():
            if cam_prim.HasAttribute(key):
                cam_prim.GetAttribute(key).Set(value)
            else:
                print(f"Unknown camera attribute with {key}:{value}")
        cameras.append(cam_prim)

    # Add collision spheres (disabled by default) to cameras to avoid objects overlapping with the camera view
    camera_colliders = []
    camera_collider_radius = config.get("camera_collider_radius", 0)
    if camera_collider_radius > 0:
        for cam in cameras:
            cam_path = cam.GetPath()
            cam_collider = stage.DefinePrim(f"{cam_path}/CollisionSphere", "Sphere")
            cam_collider.GetAttribute("radius").Set(camera_collider_radius)
            add_colliders(cam_collider)
            collision_api = UsdPhysics.CollisionAPI(cam_collider)
            collision_api.GetCollisionEnabledAttr().Set(False)
            UsdGeom.Imageable(cam_collider).MakeInvisible()
            camera_colliders.append(cam_collider)

    # Wait an app update to ensure the prim changes are applied
    await omni.kit.app.get_app().next_update_async()

    # Create render products using the cameras
    render_products = []
    resolution = config.get("resolution", (640, 480))
    for cam in cameras:
        rp = rep.create.render_product(cam.GetPath(), resolution)
        render_products.append(rp)

    # Enable rendering only at capture time
    disable_render_products_between_captures = config.get("disable_render_products_between_captures", True)
    if disable_render_products_between_captures:
        set_render_products_updates(render_products, False, include_viewport=False)

    # Create the writer and attach the render products
    writer_type = config.get("writer_type", None)
    writer_kwargs = config.get("writer_kwargs", {})
    # If not an absolute path, set it relative to the current working directory
    if out_dir := writer_kwargs.get("output_dir"):
        if not os.path.isabs(out_dir):
            out_dir = os.path.join(os.getcwd(), out_dir)
            writer_kwargs["output_dir"] = out_dir
        print(f"[SDG] Writing data to: {out_dir}")
    if writer_type is not None and len(render_products) > 0:
        writer = rep.writers.get(writer_type)
        writer.initialize(**writer_kwargs)
        writer.attach(render_products)

    # RANDOMIZERS
    # Apply a random (mostly) upwards velocity to the objects overlapping the 'bounce' area
    def on_overlap_hit(hit):
        prim = stage.GetPrimAtPath(hit.rigid_body)
        # Skip the camera collision spheres
        if prim not in camera_colliders:
            rand_vel = (random.uniform(-2, 2), random.uniform(-2, 2), random.uniform(4, 8))
            prim.GetAttribute("physics:velocity").Set(rand_vel)
        return True  # return True to continue the query

    # Area to check for overlapping objects (above the bottom collision box)
    overlap_area_thickness = 0.1
    overlap_area_origin = (0, 0, (-working_area_size[2] / 2) + (overlap_area_thickness / 2))
    overlap_area_extent = (
        working_area_size[0] / 2 * 0.99,
        working_area_size[1] / 2 * 0.99,
        overlap_area_thickness / 2 * 0.99,
    )

    # Triggered every physics update step to check for overlapping objects
    def on_physics_step(dt: float):
        hit_info = get_physx_scene_query_interface().overlap_box(
            carb.Float3(overlap_area_extent),
            carb.Float3(overlap_area_origin),
            carb.Float4(0, 0, 0, 1),
            on_overlap_hit,
            False,  # pass 'False' to indicate an 'overlap multiple' query.
        )

    # Subscribe to the physics step events to check for objects overlapping the 'bounce' area
    physx_sub = get_physx_interface().subscribe_physics_step_events(on_physics_step)

    # Pull assets towards the working area center by applying a random velocity towards the given target
    def apply_velocities_towards_target(assets, target=(0, 0, 0)):
        for prim in assets:
            loc = prim.GetAttribute("xformOp:translate").Get()
            strength = random.uniform(0.1, 1.0)
            pull_vel = (
                (target[0] - loc[0]) * strength,
                (target[1] - loc[1]) * strength,
                (target[2] - loc[2]) * strength,
            )
            prim.GetAttribute("physics:velocity").Set(pull_vel)

    # Apply random velocities to assets
    def randomize_floating_distractor_velocities(assets):
        for prim in assets:
            lin_vel = (random.uniform(-2.5, 2.5), random.uniform(-2.5, 2.5), random.uniform(-2.5, 2.5))
            ang_vel = (random.uniform(-45, 45), random.uniform(-45, 45), random.uniform(-45, 45))
            prim.GetAttribute("physics:velocity").Set(lin_vel)
            prim.GetAttribute("physics:angularVelocity").Set(ang_vel)

    # Randomize camera poses to look at a random target asset (random distance and center offset)
    camera_distance_to_target_min_max = config.get("camera_distance_to_target_min_max", (0.1, 0.5))
    camera_look_at_target_offset = config.get("camera_look_at_target_offset", 0.2)

    def randomize_camera_poses():
        for cam in cameras:
            # Get a random target asset to look at
            target_asset = random.choice(labeled_prims)
            # Add a look_at offset so the target is not always in the center of the camera view
            loc_offset = (
                random.uniform(-camera_look_at_target_offset, camera_look_at_target_offset),
                random.uniform(-camera_look_at_target_offset, camera_look_at_target_offset),
                random.uniform(-camera_look_at_target_offset, camera_look_at_target_offset),
            )
            target_loc = target_asset.GetAttribute("xformOp:translate").Get() + loc_offset
            # Get a random distance to the target asset
            distance = random.uniform(camera_distance_to_target_min_max[0], camera_distance_to_target_min_max[1])
            # Get a random pose of the camera looking at the target asset from the given distance
            cam_loc, quat = get_random_pose_on_sphere(origin=target_loc, radius=distance)
            set_transform_attributes(cam, location=cam_loc, orientation=quat)

    # Temporarily enable camera colliders and simulate for the given number of frames to push out any overlapping objects
    async def simulate_camera_collision_async(num_frames=1):
        for cam_collider in camera_colliders:
            collision_api = UsdPhysics.CollisionAPI(cam_collider)
            collision_api.GetCollisionEnabledAttr().Set(True)
        if not timeline.is_playing():
            timeline.play()
        for _ in range(num_frames):
            await omni.kit.app.get_app().next_update_async()
        for cam_collider in camera_colliders:
            collision_api = UsdPhysics.CollisionAPI(cam_collider)
            collision_api.GetCollisionEnabledAttr().Set(False)

    # Create a randomizer for the shape distractors colors, manually triggered at custom events
    with rep.trigger.on_custom_event(event_name="randomize_shape_distractor_colors"):
        shape_distractors_paths = [
            prim.GetPath() for prim in chain(floating_shape_distractors, falling_shape_distractors)
        ]
        shape_distractors_group = rep.create.group(shape_distractors_paths)
        with shape_distractors_group:
            rep.randomizer.color(colors=rep.distribution.uniform((0, 0, 0), (1, 1, 1)))

    # Create a randomizer for lights in the working area, manually triggered at custom events
    with rep.trigger.on_custom_event(event_name="randomize_lights"):
        lights = rep.create.light(
            light_type="Sphere",
            color=rep.distribution.uniform((0, 0, 0), (1, 1, 1)),
            temperature=rep.distribution.normal(6500, 500),
            intensity=rep.distribution.normal(35000, 5000),
            position=rep.distribution.uniform(working_area_min, working_area_max),
            scale=rep.distribution.uniform(0.1, 1),
            count=3,
        )

    # Create a randomizer for the dome background, manually triggered at custom events
    with rep.trigger.on_custom_event(event_name="randomize_dome_background"):
        dome_textures = [
            # ✅ Workshops/Industrial (mantidos)
            assets_root_path + "/NVIDIA/Assets/Skies/Indoor/autoshop_01_4k.hdr",
            assets_root_path + "/NVIDIA/Assets/Skies/Indoor/carpentry_shop_01_4k.hdr",
            
            # ✅ Warehouse & Depot (NOVOS - perfeitos para válvulas industriais!)
            assets_root_path + "/NVIDIA/Assets/Skies/Indoor/ZetoCG_com_WarehouseInterior2b.hdr",
            assets_root_path + "/NVIDIA/Assets/Skies/Indoor/old_bus_depot_4k.hdr",
            
            # ❌ REMOVIDOS (eram muito residenciais):
            # assets_root_path + "/NVIDIA/Assets/Skies/Indoor/hotel_room_4k.hdr",
            # assets_root_path + "/NVIDIA/Assets/Skies/Indoor/wooden_lounge_4k.hdr",
        ]
        dome_light = rep.create.light(light_type="Dome")
        with dome_light:
            rep.modify.attribute("inputs:texture:file", rep.distribution.choice(dome_textures))
            rep.randomizer.rotation()

    # Capture motion blur by combining the number of pathtraced subframes samples simulated for the given duration
    async def capture_with_motion_blur_and_pathtracing_async(duration=0.05, num_samples=8, spp=64):
        # For small step sizes the physics FPS needs to be temporarily increased to provide movements every syb sample
        orig_physics_fps = physx_scene.GetTimeStepsPerSecondAttr().Get()
        target_physics_fps = 1 / duration * num_samples
        if target_physics_fps > orig_physics_fps:
            print(f"[SDG] Changing physics FPS from {orig_physics_fps} to {target_physics_fps}")
            physx_scene.GetTimeStepsPerSecondAttr().Set(target_physics_fps)

        # Enable motion blur (if not enabled)
        is_motion_blur_enabled = carb.settings.get_settings().get("/omni/replicator/captureMotionBlur")
        if not is_motion_blur_enabled:
            carb.settings.get_settings().set("/omni/replicator/captureMotionBlur", True)
        # Number of sub samples to render for motion blur in PathTracing mode
        carb.settings.get_settings().set("/omni/replicator/pathTracedMotionBlurSubSamples", num_samples)

        # Set the render mode to PathTracing
        prev_render_mode = carb.settings.get_settings().get("/rtx/rendermode")
        carb.settings.get_settings().set("/rtx/rendermode", "PathTracing")
        carb.settings.get_settings().set("/rtx/pathtracing/spp", spp)
        carb.settings.get_settings().set("/rtx/pathtracing/totalSpp", spp)
        carb.settings.get_settings().set("/rtx/pathtracing/optixDenoiser/enabled", 0)

        # Make sure the timeline is playing
        if not timeline.is_playing():
            timeline.play()

        # Capture the frame by advancing the simulation for the given duration and combining the sub samples
        await rep.orchestrator.step_async(delta_time=duration, pause_timeline=False)

        # Restore the original physics FPS
        if target_physics_fps > orig_physics_fps:
            print(f"[SDG] Restoring physics FPS from {target_physics_fps} to {orig_physics_fps}")
            physx_scene.GetTimeStepsPerSecondAttr().Set(orig_physics_fps)

        # Restore the previous render and motion blur  settings
        carb.settings.get_settings().set("/omni/replicator/captureMotionBlur", is_motion_blur_enabled)
        print(f"[SDG] Restoring render mode from 'PathTracing' to '{prev_render_mode}'")
        carb.settings.get_settings().set("/rtx/rendermode", prev_render_mode)

    # Update the app until a given simulation duration has passed (simulate the world between captures)
    async def run_simulation_loop_async(duration):
        timeline = omni.timeline.get_timeline_interface()
        if not timeline.is_playing():
            timeline.play()
        elapsed_time = 0.0
        previous_time = timeline.get_current_time()
        app_updates_counter = 0
        while elapsed_time <= duration:
            await omni.kit.app.get_app().next_update_async()
            print(f"\t [{i}] Simulation loop at {timeline.get_current_time():.2f}")
            elapsed_time += timeline.get_current_time() - previous_time
            previous_time = timeline.get_current_time()
            app_updates_counter += 1
            print(
                f"\t Simulation loop at {timeline.get_current_time():.2f}, current elapsed time: {elapsed_time:.2f}, counter: {app_updates_counter}"
            )
        print(
            f"[SDG] Simulation loop finished in {elapsed_time:.2f} seconds at {timeline.get_current_time():.2f} with {app_updates_counter} app updates."
        )

    # SDG
    # Number of frames to capture
    num_frames = config.get("num_frames", 10)

    # Increase subframes if materials are not loaded on time, or ghosting artifacts appear on moving objects,
    # see: https://docs.omniverse.nvidia.com/extensions/latest/ext_replicator/subframes_examples.html
    rt_subframes = config.get("rt_subframes", -1)

    # Amount of simulation time to wait between captures
    sim_duration_between_captures = config.get("simulation_duration_between_captures", 0.025)

    # Initial trigger for randomizers before the SDG loop with several app updates (ensures materials/textures are loaded)
    rep.utils.send_og_event(event_name="randomize_shape_distractor_colors")
    rep.utils.send_og_event(event_name="randomize_dome_background")
    for _ in range(5):
        await omni.kit.app.get_app().next_update_async()

    # Set the timeline parameters (start, end, no looping) and start the timeline
    timeline = omni.timeline.get_timeline_interface()
    timeline.set_start_time(0)
    timeline.set_end_time(1000000)
    timeline.set_looping(False)
    # If no custom physx scene is created, a default one will be created by the physics engine once the timeline starts
    timeline.play()
    timeline.commit()
    await omni.kit.app.get_app().next_update_async()

    # Store the wall start time for stats
    wall_time_start = time.perf_counter()

    # Run the simulation and capture data triggering randomizations and actions at custom frame intervals
    for i in range(num_frames):
        # Cameras will be moved to a random position and look at a randomly selected labeled asset
        if i % 3 == 0:
            print(f"\t Randomizing camera poses")
            randomize_camera_poses()
            # Temporarily enable camera colliders and simulate for a few frames to push out any overlapping objects
            if camera_colliders:
                await simulate_camera_collision_async(num_frames=4)

        # Apply a random velocity towards the origin to the working area to pull the assets closer to the center
        if i % 10 == 0:
            print(f"\t Applying velocity towards the origin")
            apply_velocities_towards_target(chain(labeled_prims, shape_distractors, mesh_distractors))

        # Randomize lights locations and colors
        if i % 5 == 0:
            print(f"\t Randomizing lights")
            rep.utils.send_og_event(event_name="randomize_lights")

        # Randomize the colors of the primitive shape distractors
        if i % 15 == 0:
            print(f"\t Randomizing shape distractors colors")
            rep.utils.send_og_event(event_name="randomize_shape_distractor_colors")

        # Randomize the texture of the dome background
        if i % 25 == 0:
            print(f"\t Randomizing dome background")
            rep.utils.send_og_event(event_name="randomize_dome_background")

        # Apply a random velocity on the floating distractors (shapes and meshes)
        if i % 17 == 0:
            print(f"\t Randomizing shape distractors velocities")
            randomize_floating_distractor_velocities(chain(floating_shape_distractors, floating_mesh_distractors))

        # Enable render products only at capture time
        if disable_render_products_between_captures:
            set_render_products_updates(render_products, True, include_viewport=False)

        # Capture the current frame
        print(f"[SDG] Capturing frame {i}/{num_frames}, at simulation time: {timeline.get_current_time():.2f}")
        if i % 5 == 0:
            await capture_with_motion_blur_and_pathtracing_async(duration=0.025, num_samples=8, spp=128)
        else:
            await rep.orchestrator.step_async(delta_time=0.0, rt_subframes=rt_subframes, pause_timeline=False)

        # Disable render products between captures
        if disable_render_products_between_captures:
            set_render_products_updates(render_products, False, include_viewport=False)

        # Run the simulation for a given duration between frame captures
        if sim_duration_between_captures > 0:
            await run_simulation_loop_async(sim_duration_between_captures)
        else:
            await omni.kit.app.get_app().next_update_async()

    # Wait for the data to be written (default writer backends are asynchronous)
    await rep.orchestrator.wait_until_complete_async()

    # Get the stats
    wall_duration = time.perf_counter() - wall_time_start
    sim_duration = timeline.get_current_time()
    avg_frame_fps = num_frames / wall_duration
    num_captures = num_frames * num_cameras
    avg_capture_fps = num_captures / wall_duration
    print(
        f"[SDG] Captured {num_frames} frames, {num_captures} entries (frames * cameras) in {wall_duration:.2f} seconds.\n"
        f"\t Simulation duration: {sim_duration:.2f}\n"
        f"\t Simulation duration between captures: {sim_duration_between_captures:.2f}\n"
        f"\t Average frame FPS: {avg_frame_fps:.2f}\n"
        f"\t Average capture entries (frames * cameras) FPS: {avg_capture_fps:.2f}\n"
    )

    # Unsubscribe the physics overlap checks and stop the timeline
    physx_sub.unsubscribe()
    physx_sub = None
    await omni.kit.app.get_app().next_update_async()
    timeline.stop()


config = {
    "launch_config": {
        "renderer": "RayTracedLighting",
        "headless": False,
    },
    "env_url": "",
    "working_area_size": (5, 5, 3),
    "rt_subframes": 4,
    "num_frames": 1500,
    "num_cameras": 4,
    "camera_collider_radius": 1.25,
    "disable_render_products_between_captures": False,
    "simulation_duration_between_captures": 0.05,
    "resolution": (640, 480),
    "camera_properties_kwargs": {
        "focalLength": 24.0,
        "focusDistance": 400,
        "fStop": 0.0,
        "clippingRange": (0.01, 10000),
    },
    "camera_look_at_target_offset": 0.15,
    "camera_distance_to_target_min_max": (0.25, 0.75),
    "writer_type": "BasicWriter",
    "writer_kwargs": {
        "output_dir": "/workspace/datasets/valve_wheel",
        "rgb": True,
        "bounding_box_2d_tight": True,
        "bounding_box_2d_loose": False,
        "semantic_segmentation": False,
        "instance_segmentation": False,
    },
    "labeled_assets_and_properties": [
        {
            "url": "/workspace/valvula_no_rigid.usd",
            "label": "valve_lever",
            "label_prim_path": "valvula/valvula/lever_pivot",
            "base_scale": 0.001,  # Convert from millimeters to meters (SolidWorks export)
            "count": 5,
            "floating": True,
            "randomize_scale": (0.8, 1.5),
        },
        {
            "url": "/workspace/valve_wheel_no_rigid.usd",
            "label": "valve_wheel",
            "label_prim_path": "World/valve_wheel_yellow/wheel",
            "count": 5,
            "floating": True,
            "randomize_scale": (0.8, 1.5),
        },
    ],
    "shape_distractors_types": ["capsule", "cone", "cylinder", "sphere", "cube"],
    "shape_distractors_scale_min_max": (0.015, 0.15),
    "shape_distractors_num": 20,  # Reduced from 150
    "mesh_distractors_urls": [
        "/Isaac/Environments/Simple_Warehouse/Props/SM_CardBoxD_04_1847.usd",
        "/Isaac/Environments/Simple_Warehouse/Props/SM_CardBoxA_01_414.usd",
        "/Isaac/Environments/Simple_Warehouse/Props/S_TrafficCone.usd",
        "/Isaac/Environments/Simple_Warehouse/Props/S_WetFloorSign.usd",
        "/Isaac/Environments/Simple_Warehouse/Props/SM_BarelPlastic_B_03.usd",
        "/Isaac/Environments/Office/Props/SM_Board.usd",
        "/Isaac/Environments/Office/Props/SM_Book_03.usd",
        "/Isaac/Environments/Office/Props/SM_Book_34.usd",
        "/Isaac/Environments/Office/Props/SM_BookOpen_01.usd",
        "/Isaac/Environments/Office/Props/SM_Briefcase.usd",
        "/Isaac/Environments/Office/Props/SM_Extinguisher.usd",
        "/Isaac/Environments/Hospital/Props/SM_GasCart_01b.usd",
        "/Isaac/Environments/Hospital/Props/SM_MedicalBag_01a.usd",
        "/Isaac/Environments/Hospital/Props/SM_MedicalBox_01g.usd",
        "/Isaac/Environments/Hospital/Props/SM_Toweldispenser_01a.usd",
    ],
    "mesh_distractors_scale_min_max": (0.35, 1.35),
    "mesh_distractors_num": 35,  # Reduced from 175
}

asyncio.ensure_future(run_example_async(config))