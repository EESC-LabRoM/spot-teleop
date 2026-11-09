import time, io, base64, numpy as np, rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
rclpy.init(); n=Node("claude_grab_passive"); qos=QoSProfile(depth=1,reliability=ReliabilityPolicy.BEST_EFFORT); D={}
for c in ("zed1","zed2"):
    n.create_subscription(Image,f"/{c}/zed_node/left/image_rect_color",lambda m,c=c:D.__setitem__(c+"_img",m),qos)
    n.create_subscription(Image,f"/{c}/zed_node/depth/depth_registered",lambda m,c=c:D.__setitem__(c+"_dep",m),qos)
t0=time.time()
while time.time()-t0<25 and len(D)<4: rclpy.spin_once(n,timeout_sec=0.2)
out={}
import sys
for k,m in D.items():
    if k.endswith("_img"):
        ch=m.step//m.width; a=np.frombuffer(bytes(m.data),dtype=np.uint8).reshape(m.height,m.width,ch)[:,:,:3]; out[k]=a; print(k,m.encoding,m.width,m.height,file=sys.stderr)
    else:
        out[k]=np.frombuffer(bytes(m.data),dtype=np.float32).reshape(m.height,m.width).astype(np.float16); print(k,m.width,m.height,file=sys.stderr)
b=io.BytesIO(); np.savez_compressed(b,**out); sys.stdout.write(base64.b64encode(b.getvalue()).decode())
