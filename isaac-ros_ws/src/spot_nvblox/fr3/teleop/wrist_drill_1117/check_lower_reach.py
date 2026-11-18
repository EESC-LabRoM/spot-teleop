import json,numpy as np
from pathlib import Path
from fr3_kin import fk_tcp,ik,floor_margin,self_collision_margin,in_limits
p=Path(__file__).parent;s=json.loads((p/'after_fake_state.json').read_text().strip().splitlines()[-1]);seed=np.array(s['measured_joint_state']['position']);T=fk_tcp(seed);rows=[]
# Coarse geometric screen only: table at conservatively high 4cm in base.
for dx in [-.04,0,.04]:
 for dy in [-.04,0,.04]:
  q=seed.copy()
  for z in np.linspace(T[2,3],.14,25):
   target=T.copy();target[:3,3]=[T[0,3]+dx,T[1,3]+dy,z]
   q,ok=ik(target,q,q_null=seed)
   rows.append(dict(dx=dx,dy=dy,z=float(z),ik=ok,floor_margin_m=floor_margin(q,.04),self_margin_m=self_collision_margin(q),joint_limit_margin_ok=in_limits(q,.04)))
result=dict(screen='coarse FK capsules; no external-object or map validation',samples=len(rows),ik_success=sum(r['ik'] for r in rows),floor_min_m=min(r['floor_margin_m'] for r in rows),self_min_m=min(r['self_margin_m'] for r in rows),all_joint_margin_ok=all(r['joint_limit_margin_ok'] for r in rows),rows=rows)
(p/'lower_reach_screen.json').write_text(json.dumps(result,indent=2));print(json.dumps({k:v for k,v in result.items() if k!='rows'},indent=2))
