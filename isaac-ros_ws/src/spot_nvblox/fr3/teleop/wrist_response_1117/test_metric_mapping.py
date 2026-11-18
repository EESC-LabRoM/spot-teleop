"""Actual consumer AST: metric range, original axes and workspace return behavior."""
from pathlib import Path
import numpy as np
from consumer_kernel import Kernel
ROOT=Path(__file__).parent

def trial(preserve):
    origin=np.array([.4,0,.5]);q=np.array([1.,0,0,0]);k=Kernel(ROOT/'fr3_input_teleop_wrist_1117.py',origin,q);k.n.zed_preserve_workspace_anchor=preserve
    count=0
    def feed(physical_x,frames=1):
        nonlocal count
        for _ in range(frames):
            t=count*.02;count+=1
            packet=dict(p=[physical_x*1.315,0,0],q=[0,0,0,1],stamp_ns=int((t+1)*1e9),orientation_valid=True)
            k.step(t,packet,t,k.n.cmd_p.copy(),q)
    feed(0,30)
    for x in np.linspace(0,.1,100):feed(x)
    feed(.1,100)
    assert abs(k.n.cmd_p[0]-.5)<1e-6, '10cm physical input must produce10cm settled travel'
    for x in np.linspace(.1,1.,200):feed(x)
    feed(1.,100)
    assert k.n.cmd_p[0]<=.8+1e-9
    for x in np.linspace(1.,0,200):feed(x)
    feed(0,200)
    return k.n.cmd_p.copy(),k.n.zed_anchor[0].copy()
new,anchor=trial(True);old,oldanchor=trial(False)
assert np.allclose(new,[.4,0,.5],atol=1e-6)
assert np.allclose(anchor,[.4,0,.5],atol=1e-10)
assert abs(old[0]-.4)>.05, 'test must reproduce original anchor shift'
print('PASS metric1:1, original axes, workspace cap and return-to-neutral; old shift reproduced:',old[0], 'fixed:',new[0])
