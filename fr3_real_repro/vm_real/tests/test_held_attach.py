"""held_attach.py: latch hysteresis, near-target gate, sphere fitting.

Run: docker exec vm-viewer python3 <this file> (numpy only, no ROS/curobo).
"""
import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(
    os.path.abspath(__file__))), 'node'))

from held_attach import (HeldLatch, attach_floor_ok, attached_holds,
                         fallback_spheres, fit_hand_spheres, holding_verdict,
                         lowest_world_point, should_attach)


def check(cond, msg):
    if not cond:
        raise AssertionError(msg)
    print(f"ok: {msg}")


# Latch: close <0.070, open >=0.075, None/NaN hold their state.
la = HeldLatch()
check(la.update(0.08) is None and not la.held, "open idle")
check(la.update(0.05) == 'latched' and la.held, "close latches")
check(la.update(0.05) is None and la.held, "closed stays")
check(la.update(0.072) is None and la.held, "0.072 hysteresis holds")
check(la.update(None) is None and la.held, "None keeps latch")
check(la.update(float('nan')) is None and la.held, "NaN keeps latch")
check(la.update(0.075) == 'released' and not la.held, "0.075 releases")
check(HeldLatch().update(None) is None, "None never latches blind")

# Near-target gate: the empty-close guard.
check(should_attach([0.5, 0.0, 0.3], [0.52, 0.01, 0.3]), "2 cm closes attach")
check(not should_attach([0.5, 0.0, 0.3], [0.7, 0.0, 0.3]),
      "20 cm closes refuse")
check(not should_attach([0.5, 0.0, 0.3], None), "missing target refuses")
check(not should_attach([np.nan, 0, 0], [0.5, 0, 0.3]), "NaN refuses")

# Verdict: width floor kills empty closes, near gate kills far closes.
v, r = holding_verdict(0.04, [0.5, 0.0, 0.3], [0.52, 0.01, 0.3])
check(v and r.startswith('holding'), f"close on drill holds ({r})")
check(holding_verdict(0.002, [0.5, 0.0, 0.3], [0.5, 0.0, 0.3])[0] is False,
      "missed grasp (~0 width) refuses")
check(holding_verdict(0.002, [0.5, 0.0, 0.3], [0.5, 0.0, 0.3])[1] == 'empty-close',
      "miss reason names the empty close")
check(holding_verdict(0.04, [0.5, 0.0, 0.3], [0.7, 0.0, 0.3])[1] == 'far-from-target',
      "far close refuses")
check(holding_verdict(None, [0.5, 0.0, 0.3], [0.5, 0.0, 0.3])[1] == 'no-width',
      "missing width refuses")
check(holding_verdict(0.04, [0.5, 0.0, 0.3], None)[1] == 'no-target',
      "missing target refuses")

# Settled-width floors (sim FINDING: the latch edge samples mid-travel).
check(attach_floor_ok(0.004) == (False, 'empty-settled'),
      "settled 0.004 refuses attach")
check(attach_floor_ok(0.035)[0] is True, "settled 0.035 attaches")
check(attach_floor_ok(None) == (False, 'no-width'),
      "missing width waits (no attach without evidence)")
check(attached_holds(0.004) is False,
      "drop to 0.004 while attached detaches")
check(attached_holds(0.035) is True, "0.035 while attached holds")
check(attached_holds(None) is True,
      "dead stream while attached holds collision")

# Fit: box 14x6x11 cm at identity -> 4 inset spheres along X in hand frame.
T = np.eye(4)
S = fit_hand_spheres([0.5, -0.2, 0.25], [0.07, 0.03, 0.055], T)
check(S.shape == (4, 4), f"4 spheres out ({S.shape})")
check(np.all(np.isfinite(S)) and np.all(S[:, 3] > 0), "finite, positive r")
check(abs(S[0, 1] - -0.2) < 1e-9 and abs(S[0, 2] - 0.25) < 1e-9,
      "centers on the box middle line")
check(abs(S[0, 3] - (0.055 + 0.005)) < 1e-9,
      f"radius = perp halves + pad ({S[0, 3]:.4f})")
check(S[:, 0].min() - S[:, 3].max() >= 0.5 - 0.07 - 1e-9
      and S[:, 0].max() + S[:, 3].max() <= 0.5 + 0.07 + 1e-9,
      "no bulge past the box ends")
# Table rule (RAL B1): a box standing on the table, fitted at latch, must
# reach lowest-world-point >= table + 1 cm once lifted to attach height.
TABLE_Z = 0.025
box_c = [0.5, -0.2, TABLE_Z + 0.028]
St = fit_hand_spheres(box_c, [0.10, 0.035, 0.028], np.eye(4))
dip = TABLE_Z - (St[:, 2].min() - St[:, 3].max())
check(0 < dip < 0.04, f"latch dip ({dip*1000:.0f} mm) under the 4 cm rise")
rise = St.copy()
rise[:, 2] += 0.04
check(lowest_world_point(rise, [0, 0, 0], [1, 0, 0, 0]) >= TABLE_Z + 0.01,
      "attach-height spheres clear table + 1 cm")
check(abs(lowest_world_point(St, [0, 0, 0.05], [1, 0, 0, 0])
          - (St[:, 2].min() - St[:, 3].max() + 0.05)) < 1e-9,
      "lowest point tracks hand translation")
# 90-deg yawed hand: lowest point follows the rotation.
lw = lowest_world_point(np.array([[0.05, 0.0, 0.0, 0.02]]), [0, 0, 0.3],
                        [np.sqrt(0.5), 0, 0, np.sqrt(0.5)])
check(abs(lw - 0.28) < 1e-9, f"lowest point rotates with hand ({lw:.4f})")

# Rotated 90 deg about Z: longest axis follows into hand Y.
Rz = np.array([[0, -1, 0, 0], [1, 0, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]], float)
S2 = fit_hand_spheres([0.5, -0.2, 0.25], [0.07, 0.03, 0.055], Rz)
check(S2[:, 1].max() - S2[:, 1].min() > S2[:, 0].max() - S2[:, 0].min(),
      "long axis tracked into hand frame")

# Fallback: symmetric, finite, covers the palm both ways.
F = fallback_spheres()
check(F.shape == (4, 4) and np.all(np.isfinite(F)), "fallback shape")
check(F[:, 2].min() < 0 < F[:, 2].max(), "fallback straddles the TCP")

# Bad inputs raise (caller falls back, never attaches NaN).
for bad in [lambda: fit_hand_spheres(None, [1, 1, 1], T),
            lambda: fit_hand_spheres([0, 0, 0], [1, -1, 1], T),
            lambda: fit_hand_spheres([0, 0, 0], [1, 1, 1], np.eye(3))]:
    try:
        bad()
        raise SystemExit("bad input did not raise")
    except ValueError:
        pass
print("ok: bad inputs raise ValueError")
print("held_attach: all green")
