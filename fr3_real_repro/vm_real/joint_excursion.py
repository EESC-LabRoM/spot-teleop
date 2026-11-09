"""Posture check of a sim run: how far the arm's joints wander from where they started. joint_excursion.py <trace.npz>"""
import sys, numpy as np
Z = np.load(sys.argv[1], allow_pickle=True); q = np.asarray(Z["q"], float)[:, :7]; q0 = q[0]
lo = np.array([-2.7437, -1.7837, -2.9007, -3.0421, -2.8065, 0.5445, -3.0159]); hi = np.array([2.7437, 1.7837, 2.9007, -0.1518, 2.8065, 4.5169, 3.0159])
ex = np.abs(q - q0).max(0)
print(f"POSTURE: max |q - q_start| per joint {np.round(ex, 2).tolist()} | overall {ex.max():.2f} rad | final vs start {np.abs(q[-1] - q0).max():.2f} rad | nearest FR3 joint limit {np.minimum(q.min(0) - lo, hi - q.max(0)).min():.2f} rad")
