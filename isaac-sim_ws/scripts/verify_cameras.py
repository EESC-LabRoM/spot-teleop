#!/usr/bin/env python3
import sys
import os

usd_libs_path = "/isaac-sim/extscache/omni.usd.libs-1.0.1+8131b85d.lx64.r.cp311"
if os.path.exists(usd_libs_path):
    lib_path = os.path.join(usd_libs_path, "bin")
    os.environ["LD_LIBRARY_PATH"] = f"{lib_path}:{os.environ.get('LD_LIBRARY_PATH', '')}"
    sys.path.insert(0, usd_libs_path)

from pxr import Usd, UsdGeom
import math

def quaternion_to_euler(w, x, y, z):
    """Converte quaternion para Euler (roll, pitch, yaw) em graus"""
    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)
    
    sinp = 2 * (w * y - z * x)
    if abs(sinp) >= 1:
        pitch = math.copysign(math.pi / 2, sinp)
    else:
        pitch = math.asin(sinp)
    
    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    
    return math.degrees(roll), math.degrees(pitch), math.degrees(yaw)

# Comparar com hand_cam e novas câmeras
stage_orig = Usd.Stage.Open('/workspace/zed_streamer_warehouse.usd')
stage_new = Usd.Stage.Open('/workspace/zed_streamer_warehouse_with_fisheye_v2.usd')

print("=== Comparação de Orientações ===\n")

# Hand cam original
hand_cam = stage_orig.GetPrimAtPath("/Root/spot_with_arm/arm0_link_wr1/hand_cam")
if hand_cam:
    xformable = UsdGeom.Xformable(hand_cam)
    for op in xformable.GetOrderedXformOps():
        if "orient" in op.GetOpName():
            q = op.Get()
            w = q.GetReal()
            x, y, z = q.GetImaginary()[0], q.GetImaginary()[1], q.GetImaginary()[2]
            r, p, ya = quaternion_to_euler(w, x, y, z)
            print(f"hand_cam (referência - olha para frente):")
            print(f"  Quaternion: ({w:.4f}, {x:.4f}, {y:.4f}, {z:.4f})")
            print(f"  Euler: roll={r:.1f} deg, pitch={p:.1f} deg, yaw={ya:.1f} deg")
            print()

# Novas câmeras
cameras = [
    "/Root/spot_with_arm/base/frontleft_fisheye",
    "/Root/spot_with_arm/base/frontright_fisheye"
]

for cam_path in cameras:
    cam = stage_new.GetPrimAtPath(cam_path)
    if cam:
        xformable = UsdGeom.Xformable(cam)
        print(f"{cam_path.split('/')[-1]}:")
        for op in xformable.GetOrderedXformOps():
            if "orient" in op.GetOpName():
                q = op.Get()
                w = q.GetReal()
                x, y, z = q.GetImaginary()[0], q.GetImaginary()[1], q.GetImaginary()[2]
                r, p, ya = quaternion_to_euler(w, x, y, z)
                print(f"  Quaternion: ({w:.4f}, {x:.4f}, {y:.4f}, {z:.4f})")
                print(f"  Euler: roll={r:.1f} deg, pitch={p:.1f} deg, yaw={ya:.1f} deg")
        print()
