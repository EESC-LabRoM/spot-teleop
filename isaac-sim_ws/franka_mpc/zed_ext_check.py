# Does the Stereolabs ZED extension load in our Isaac Sim 6.0 image?  ./run_isaac.sh -s zed_ext_check.py zed_ext_check --ext-dir <dir>
import argparse, json, os
ap = argparse.ArgumentParser(); ap.add_argument("--out", required=True); ap.add_argument("--ext-dir", required=True)
a = ap.parse_args(); os.makedirs(a.out, exist_ok=True)
from isaacsim import SimulationApp
app = SimulationApp({"headless": True})
import omni.kit.app
ka = omni.kit.app.get_app(); em = ka.get_extension_manager()
em.add_path(a.ext_dir)
ok = em.set_extension_enabled_immediate("sl.sensor.camera", True)
for _ in range(10): app.update()
import omni.graph.core as og
types = [t for t in og.get_registered_nodes() if t.startswith("sl.sensor.camera")]
res = {"kit_version": ka.get_kit_version(), "enabled": bool(ok), "is_enabled": em.is_extension_enabled("sl.sensor.camera"),
       "zed_node_types": types}
try:
    import sl.sensor.camera as m; res["python_module"] = m.__file__
except Exception as e:
    res["python_module_error"] = repr(e)
print("[zedcheck] " + json.dumps(res), flush=True); json.dump(res, open(os.path.join(a.out, "RESULT.json"), "w"), indent=1)
os.system(f"chown -R 1000:1000 {a.out}"); os._exit(0)
