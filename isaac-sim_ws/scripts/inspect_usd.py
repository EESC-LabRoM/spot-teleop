"""Script to inspect USD file structure and find all prims"""
from pxr import Usd

def inspect_usd(usd_path):
    """Print all prims in a USD file"""
    print(f"\n{'='*60}")
    print(f"Inspecting: {usd_path}")
    print(f"{'='*60}\n")
    
    # Open the USD stage
    stage = Usd.Stage.Open(usd_path)
    
    if not stage:
        print(f"ERROR: Could not open {usd_path}")
        return
    
    # Traverse all prims
    print("Prim hierarchy:")
    for prim in stage.Traverse():
        # Get depth for indentation
        depth = len(str(prim.GetPath()).split('/')) - 2
        indent = "  " * depth
        prim_type = prim.GetTypeName()
        print(f"{indent}{prim.GetPath()} ({prim_type})")

if __name__ == "__main__":
    # Inspect both USD files
    inspect_usd("/workspace/valvula_no_rigid.usd")
    inspect_usd("/workspace/valve_wheel_no_rigid.usd")
