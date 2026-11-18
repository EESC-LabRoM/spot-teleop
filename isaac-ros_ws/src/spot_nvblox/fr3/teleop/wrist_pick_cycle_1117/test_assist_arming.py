from pathlib import Path
import sys,numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parent));from bounded_assist import ApproachArming
g=ApproachArming();target=np.array([0,0,.2]);p=np.array([0,0,.5])
assert not g.update(False,p,target)
for _ in range(100):assert not g.update(True,p,target)
assert not g.update(True,p+np.array([0,0,.02]),target)
assert g.update(True,p+np.array([0,0,.005]),target)
assert not g.update(False,p,target)
assert not g.update(True,p,target)
for _ in range(100):assert not g.update(True,p,target+.05)
print('PASS stationary engage, outward hand motion, inward arming, disengage reset, target movement alone')
