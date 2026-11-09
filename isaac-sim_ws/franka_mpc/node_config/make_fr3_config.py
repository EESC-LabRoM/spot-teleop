"""Build the Franka FR3 (the lab's arm) description for cuRobo from Franka's official franka_description (release 2.9.0):
  1. xacro robots/fr3/fr3.urdf.xacro with the Franka Hand -> fr3.urdf (fr3_* joint and link names, FR3 joint limits);
     finger_joint2's <mimic> is dropped, as in cuRobo's Panda URDF (cuRobo cannot lock a mimic joint).
     Mesh paths point at the collision meshes only (cuRobo collides with spheres; Isaac uses its own fr3.usd).
  2. cuRobo's Panda collision spheres (spheres/franka_mesh.yml) renamed panda_* -> fr3_*: FR3 and Panda share their
     kinematics (franka_description kinematics.yaml is identical), so the link frames are the same; coverage of the FR3
     collision meshes is checked by check_fr3_config.py.
  3. node_config/franka.yml (our MPC config: cuRobo's franka.yml minus panda_link0 collision) renamed -> fr3.yml, with
     the URDF and spheres above.
Writes into cuRobo's content tree (curobo-isaac6/src/curobo/content), so the same relative paths resolve on the host and
in the Isaac container (/workspace/curobo-isaac6), and node_config/fr3.yml (+ the same file as configs/robot/fr3.yml).
   python3 make_fr3_config.py [--franka-description ../../franka_description]
"""
import argparse, os, re, shutil, subprocess, tempfile

import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
ap = argparse.ArgumentParser()
ap.add_argument("--franka-description", default=os.path.normpath(os.path.join(HERE, "..", "..", "franka_description")))
ap.add_argument("--content", default=os.path.normpath(os.path.join(HERE, "..", "..", "curobo-isaac6", "src", "curobo", "content")))
a = ap.parse_args()
FD, C = a.franka_description, a.content
DST = os.path.join(C, "assets", "robot", "fr3_description")

# 1. URDF ---------------------------------------------------------------------------------------------------------------
with tempfile.TemporaryDirectory() as T:
    for d in ("robots", "end_effectors", "accessories"): shutil.copytree(os.path.join(FD, d), os.path.join(T, d))
    for root, _, files in os.walk(T):                         # $(find franka_description) needs a ROS install: inline it
        for f in files:
            if f.endswith(".xacro"):
                p = os.path.join(root, f); s = open(p).read()
                if "$(find franka_description)" in s: open(p, "w").write(s.replace("$(find franka_description)", T))
    urdf = subprocess.run(["uvx", "--from", "xacro", "xacro", os.path.join(T, "robots", "fr3", "fr3.urdf.xacro"),
                           "hand:=true", "ee_id:=franka_hand"], check=True, capture_output=True, text=True).stdout
def _coll(m):                                                 # visual .dae -> its collision .stl when one exists
    c = f"{m.group(1)}/collision/{m.group(2)}.stl"
    return f"package://franka_description/{c}" if os.path.exists(os.path.join(FD, c)) else m.group(0)
urdf = re.sub(r'package://franka_description/(meshes/[^"]*?)/visual/([^"/]*?)\.dae', _coll, urdf)
urdf = urdf.replace("package://franka_description/", "")
urdf = re.sub(r'\s*<mimic joint="fr3_finger_joint1"[^>]*/>', "", urdf)   # as cuRobo's Panda URDF: both fingers plain joints (locked)
# Joint position limits: the FCI's commanded-position limits for the FR3 (Franka "Robot and interface specifications":
# q_min < q_c < q_max; the same values Isaac 6.0's fr3.usd carries), tighter than franka_description's URDF limits.
FCI_Q_MIN = [-2.7437, -1.7837, -2.9007, -3.0421, -2.8065, 0.5445, -3.0159]
FCI_Q_MAX = [2.7437, 1.7837, 2.9007, -0.1518, 2.8065, 4.5169, 3.0159]
for i in range(7):
    blk = re.search(rf'<joint name="fr3_joint{i + 1}" type="revolute">.*?</joint>', urdf, re.S); b = blk.group(0)
    b2 = re.sub(r'lower="[^"]*"', f'lower="{FCI_Q_MIN[i]}"', b, count=1); b2 = re.sub(r'upper="[^"]*"', f'upper="{FCI_Q_MAX[i]}"', b2, count=1)
    urdf = urdf.replace(b, b2)
os.makedirs(DST, exist_ok=True); open(os.path.join(DST, "fr3.urdf"), "w").write(urdf)
for m in sorted(set(re.findall(r'filename="([^"]+)"', urdf))):
    os.makedirs(os.path.dirname(os.path.join(DST, m)), exist_ok=True); shutil.copy(os.path.join(FD, m), os.path.join(DST, m))
print("urdf:", os.path.join(DST, "fr3.urdf"), "| joints:", re.findall(r'<joint name="(fr3_(?:finger_)?joint\d)"', urdf))

# 2. spheres -------------------------------------------------------------------------------------------------------------
sp = yaml.safe_load(open(os.path.join(C, "configs", "robot", "spheres", "franka_mesh.yml")))
sp = {"collision_spheres": {k.replace("panda_", "fr3_"): v for k, v in sp["collision_spheres"].items()}}
# FR3's right finger frame is turned pi about z (finger_joint2 origin rpy 0 0 pi, axis +y) where the Panda's is not
# (axis -y): turn its sphere centres with it so the spheres land where the Panda's did
for s_ in sp["collision_spheres"]["fr3_rightfinger"]: s_["center"] = [-s_["center"][0], -s_["center"][1], s_["center"][2]]
yaml.safe_dump(sp, open(os.path.join(C, "configs", "robot", "spheres", "fr3_mesh.yml"), "w"), sort_keys=False)

# 3. robot config --------------------------------------------------------------------------------------------------------
src = open(os.path.join(HERE, "franka.yml")).read().replace("panda_", "fr3_")
src = src.replace('urdf_path: "robot/franka_description/franka_panda.urdf"', 'urdf_path: "robot/fr3_description/fr3.urdf"')
src = src.replace('asset_root_path: "robot/franka_description"', 'asset_root_path: "robot/fr3_description"')
src = src.replace('collision_spheres: "spheres/franka_mesh.yml"', 'collision_spheres: "spheres/fr3_mesh.yml"')
head = ("# Franka FR3 + Franka Hand, generated by node_config/make_fr3_config.py from node_config/franka.yml (Panda) and\n"
        "# franka_description 2.9.0: fr3_* names, FR3 URDF (joint limits from the URDF), Panda spheres renamed (same kinematics).\n")
assert "robot/fr3_description/fr3.urdf" in src and "spheres/fr3_mesh.yml" in src
for out in (os.path.join(HERE, "fr3.yml"), os.path.join(C, "configs", "robot", "fr3.yml")): open(out, "w").write(head + src)
print("config:", os.path.join(HERE, "fr3.yml"), "+ configs/robot/fr3.yml; spheres: configs/robot/spheres/fr3_mesh.yml")
