"""Equivalent local FR3 workcell; NVIDIA FR3 asset, procedural table/objects, no remote private assets."""
from pathlib import Path
from pxr import Usd,UsdGeom,UsdPhysics,UsdLux,Gf
P=Path(__file__).resolve().parent
s=Usd.Stage.CreateNew(str(P/'remote_workcell.usda'));UsdGeom.SetStageUpAxis(s,'Z');UsdGeom.SetStageMetersPerUnit(s,1.)
w=UsdGeom.Xform.Define(s,'/World');s.SetDefaultPrim(w.GetPrim())
UsdPhysics.Scene.Define(s,'/World/physics').CreateGravityMagnitudeAttr(9.81)
light=UsdLux.DomeLight.Define(s,'/World/light');light.CreateIntensityAttr(1500.)
def box(path,xyz,size,color,rigid=False):
 o=UsdGeom.Cube.Define(s,path);o.CreateSizeAttr(1.);o.CreateDisplayColorAttr([Gf.Vec3f(*color)])
 x=UsdGeom.Xformable(o);x.AddTranslateOp().Set(Gf.Vec3d(*xyz));x.AddScaleOp().Set(Gf.Vec3f(*size))
 UsdPhysics.CollisionAPI.Apply(o.GetPrim())
 if rigid:UsdPhysics.RigidBodyAPI.Apply(o.GetPrim());UsdPhysics.MassAPI.Apply(o.GetPrim()).CreateMassAttr(.2)
 return o
box('/World/floor',[0,0,-.04],[8,8,.08],[.18,.2,.22])
box('/World/table',[.35,0,.71],[1.4,1.1,.08],[.055,.055,.06])
for x in (-.25,.9):
 for y in (-.45,.45):box('/World/leg_'+str(x).replace('-','m').replace('.','_')+'_'+str(y).replace('-','m').replace('.','_'),[x,y,.35],[.055,.055,.7],[.1,.1,.1])
f=UsdGeom.Xform.Define(s,'/World/franka');f.GetPrim().GetReferences().AddReference('https://omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/Isaac/6.0/Isaac/Robots/FrankaRobotics/FrankaFR3/fr3.usd');f.AddTranslateOp().Set(Gf.Vec3d(0,0,.75))
d=UsdGeom.Xform.Define(s,'/World/drill');d.AddTranslateOp().Set(Gf.Vec3d(.45,-.2,.75));UsdPhysics.RigidBodyAPI.Apply(d.GetPrim());UsdPhysics.MassAPI.Apply(d.GetPrim()).CreateMassAttr(.18)
box('/World/drill/base',[0,0,.025],[.095,.065,.05],[.1,.65,.15])
box('/World/drill/handle',[0,0,.09],[.035,.045,.11],[.12,.5,.14])
box('/World/drill/head',[.02,0,.165],[.13,.055,.055],[.1,.8,.2])
box('/World/bottle',[.57,.08,.86],[.07,.07,.22],[.05,.2,.85],True)
box('/World/mug',[.35,.28,.81],[.09,.09,.12],[.9,.9,.85],True)
box('/World/box',[.5,.43,.8],[.18,.14,.1],[.55,.2,.55],True)
s.GetRootLayer().Save();print(P/'remote_workcell.usda')
