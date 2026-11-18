import sqlite3,json,sys
import numpy as np,cv2
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import Image
c=sqlite3.connect(sys.argv[1]);topics=dict(c.execute('select id,name from topics'));rows={}
for tid,t,data in c.execute('select topic_id,timestamp,data from messages order by timestamp'):
 name=topics[tid]
 if name not in ['/vm/cable/depth','/vm/view_mask']:continue
 m=deserialize_message(data,Image);s=m.header.stamp.sec+m.header.stamp.nanosec*1e-9
 rows.setdefault(name,[]).append((t*1e-9,s,m))
depths=rows['/vm/cable/depth'];out=[]
for t,s,m in rows['/vm/view_mask']:
 available=[r for r in depths if r[0]<=t]
 if not available:continue
 dt,ds,d=min(available[-60:],key=lambda r:abs(r[1]-s))
 D=np.frombuffer(d.data,np.float32).reshape(d.height,d.step//4)[:,:d.width]
 M=np.frombuffer(m.data,np.uint8).reshape(m.height,m.step)[:,:m.width]
 M=cv2.resize(M,(d.width,d.height),interpolation=cv2.INTER_NEAREST)
 M=cv2.erode((M>0).astype(np.uint8),np.ones((5,5),np.uint8))
 z=D[M>0];z=z[np.isfinite(z)&(z>.15)&(z<2.5)]
 zband=z[(z>np.percentile(z,10)-.02)&(z<np.percentile(z,10)+.2)] if len(z) else z
 out.append(dict(mask_age=t-s,pair_delta=abs(ds-s),mask_pixels=int(M.sum()),valid_depth=len(z),band_points=len(zband)))
a=lambda key: {k:float(np.percentile([r[key] for r in out],p)) for k,p in [('min',0),('p50',50),('p95',95),('max',100)]}
print(json.dumps(dict(depth_count=len(depths),mask_count=len(out),stats={k:a(k) for k in out[0]},passing=sum(r['pair_delta']<=.3 and r['band_points']>=150 for r in out),examples=out[::max(1,len(out)//5)]),indent=2))
