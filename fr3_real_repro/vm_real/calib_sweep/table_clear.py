"""Gate for the calibration sweep: the planned world has NO drill/bottle/box on the table. Count live robot-free depth points above the table in the
region the arm will sweep; refuse if there are real objects there.   table_clear.py <segcloud.npz>"""
import sys, numpy as np
S = np.load(sys.argv[1]); bad = False
for c in ("zed1", "zed2"):
    if c not in S.files: print(f"  {c}: no cloud"); bad = True; continue
    P = S[c]; m = (P[:, 0] > 0.15) & (P[:, 0] < 0.60) & (np.abs(P[:, 1]) < 0.55) & (P[:, 2] > 0.10) & (P[:, 2] < 0.80)
    n = int(m.sum()); print(f"  {c}: {n} points above the table (z 0.10-0.80 m) in x 0.15-0.60, |y|<0.55" + (f", centre {np.round(P[m].mean(0), 2).tolist()}" if n else ""))
    if n > 150: bad = True
print("TABLE_CLEAR" if not bad else "TABLE_NOT_CLEAR"); sys.exit(1 if bad else 0)
