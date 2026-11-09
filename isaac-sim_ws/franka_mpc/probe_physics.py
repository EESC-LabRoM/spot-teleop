"""Print the physics setup of the objects (collision approximation, mass, friction) and the Franka fingers."""
import argparse, os
ap = argparse.ArgumentParser(); ap.add_argument("--out", required=True); ap.add_argument("--objects", nargs="+", default=["drill"])
ap.add_argument("--stage", default=os.path.join(os.path.dirname(os.path.abspath(__file__)), "franka_workcell.usda")); a = ap.parse_args()
os.makedirs(a.out, exist_ok=True)
from isaacsim import SimulationApp
app = SimulationApp({"headless": True, "width": 640, "height": 360})
import omni.usd
from pxr import Usd, UsdGeom, UsdPhysics, PhysxSchema
from isaacsim.core.utils.stage import is_stage_loading
ctx = omni.usd.get_context(); ctx.open_stage(a.stage); stage = ctx.get_stage()
for i in range(3000):
    app.update()
    if i > 60 and not is_stage_loading(): break
def mat_of(p):
    from pxr import UsdShade
    m = UsdShade.MaterialBindingAPI(p).ComputeBoundMaterial("physics")[0]
    if not m: return None
    pm = UsdPhysics.MaterialAPI(m.GetPrim()); 
    return (str(m.GetPath()), pm.GetStaticFrictionAttr().Get() if pm else None, pm.GetDynamicFrictionAttr().Get() if pm else None)
for root in [f"/World/{o}" for o in a.objects] + ["/World/franka"]:
    for p in Usd.PrimRange(stage.GetPrimAtPath(root)):
        ap_ = p.GetAppliedSchemas()
        if not any(s in ap_ for s in ("PhysicsCollisionAPI", "PhysicsRigidBodyAPI", "PhysicsMassAPI", "PhysxConvexHullCollisionAPI", "PhysxConvexDecompositionCollisionAPI")): continue
        info = {"schemas": [s for s in ap_ if "Physics" in s or "Physx" in s][:8]}
        if p.HasAPI(UsdPhysics.MeshCollisionAPI): info["approx"] = str(UsdPhysics.MeshCollisionAPI(p).GetApproximationAttr().Get())
        if p.HasAPI(UsdPhysics.MassAPI): info["mass"] = UsdPhysics.MassAPI(p).GetMassAttr().Get(); info["density"] = UsdPhysics.MassAPI(p).GetDensityAttr().Get()
        if p.HasAPI(UsdPhysics.RigidBodyAPI): info["kinematic"] = UsdPhysics.RigidBodyAPI(p).GetKinematicEnabledAttr().Get()
        if "finger" in p.GetName() or root != "/World/franka": info["material"] = mat_of(p)
        if root == "/World/franka" and "finger" not in p.GetName() and "hand" not in p.GetName(): continue
        print("[probe]", p.GetPath(), info, flush=True)
import sys; sys.stdout.flush(); os._exit(0)
