#!/usr/bin/env python3
import sys
import os
import math

usd_libs_path = "/isaac-sim/extscache/omni.usd.libs-1.0.1+8131b85d.lx64.r.cp311"
if os.path.exists(usd_libs_path):
    sys.path.insert(0, usd_libs_path)

from pxr import Gf

def rotate_vector_by_quaternion(v, q):
    """Rotaciona um vetor por um quaternion usando q * v * q^-1"""
    w, x, y, z = q.GetReal(), q.GetImaginary()[0], q.GetImaginary()[1], q.GetImaginary()[2]
    vx, vy, vz = v[0], v[1], v[2]
    
    qv = Gf.Quatd(0, vx, vy, vz)
    qc = Gf.Quatd(w, -x, -y, -z)  # conjugate
    result = Gf.Quatd(w, x, y, z) * qv * qc
    return (result.GetImaginary()[0], result.GetImaginary()[1], result.GetImaginary()[2])

# A camera USD olha para -Z por padrao
camera_forward = (0, 0, -1)

print("=== Direcao de visao das cameras (vetor forward) ===\n")
print("Coordenadas do frame do corpo do Spot: +X=frente, +Y=esquerda, +Z=cima\n")

# Hand cam
q = Gf.Quatd(0.5014, 0.5014, -0.4986, -0.4986)
forward = rotate_vector_by_quaternion(camera_forward, q)
print(f"hand_cam:")
print(f"  Forward: ({forward[0]:.3f}, {forward[1]:.3f}, {forward[2]:.3f})")
print(f"  -> Olha para: +X (frente)\n")

# Frontleft
q = Gf.Quatd(0.6259, 0.7729, -0.0812, -0.0658)
forward = rotate_vector_by_quaternion(camera_forward, q)
print(f"frontleft_fisheye:")
print(f"  Forward: ({forward[0]:.3f}, {forward[1]:.3f}, {forward[2]:.3f})")
angle_yaw = math.degrees(math.atan2(forward[1], forward[0]))
angle_pitch = math.degrees(math.atan2(-forward[2], math.sqrt(forward[0]**2 + forward[1]**2)))
print(f"  -> Yaw (de +X): {angle_yaw:.1f} graus, Pitch (para baixo): {angle_pitch:.1f} graus\n")

# Frontright
q = Gf.Quatd(0.0658, 0.0812, -0.7729, -0.6259)
forward = rotate_vector_by_quaternion(camera_forward, q)
print(f"frontright_fisheye:")
print(f"  Forward: ({forward[0]:.3f}, {forward[1]:.3f}, {forward[2]:.3f})")
angle_yaw = math.degrees(math.atan2(forward[1], forward[0]))
angle_pitch = math.degrees(math.atan2(-forward[2], math.sqrt(forward[0]**2 + forward[1]**2)))
print(f"  -> Yaw (de +X): {angle_yaw:.1f} graus, Pitch (para baixo): {angle_pitch:.1f} graus\n")

# O que esperavamos
print("=== Esperado ===")
print("frontleft: yaw=78 graus (esquerda), pitch=12 graus para baixo")
print("frontright: yaw=-78 graus (direita), pitch=12 graus para baixo")
