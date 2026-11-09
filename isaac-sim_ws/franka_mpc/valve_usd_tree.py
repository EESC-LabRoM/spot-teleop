import argparse, os
ap = argparse.ArgumentParser(); ap.add_argument("--out", required=True); a = ap.parse_args()
from isaacsim import SimulationApp
app = SimulationApp({"headless": True})
from pxr import Usd, UsdPhysics
st = Usd.Stage.Open("/workspace/franka_mpc/franka_workcell_hive.usda")
for p in Usd.PrimRange(st.GetPrimAtPath("/World/hiveboard")):
    apis = [s for s in p.GetAppliedSchemas()]
    if p.GetTypeName() in ("Xform", "PhysicsRevoluteJoint", "PhysicsFixedJoint", "Mesh", "") or apis:
        if p.GetTypeName() == "Mesh" and not apis: continue
        print(p.GetPath(), p.GetTypeName(), apis)
for p in Usd.PrimRange(st.GetPrimAtPath("/World/hiveboard/valve")):
    if p.GetTypeName() == "PhysicsRevoluteJoint":
        for a in ("physics:lowerLimit", "physics:upperLimit", "physics:axis", "drive:angular:physics:stiffness", "drive:angular:physics:damping", "physxJoint:jointFriction", "physics:localPos0"):
            print("  ", a, p.GetAttribute(a).Get())
import sys; sys.stdout.flush(); os._exit(0)
