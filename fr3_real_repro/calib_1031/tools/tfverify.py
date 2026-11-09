import sys, json, time, math, numpy as np, rclpy
from tf2_ros import Buffer, TransformListener
def q2R(q):
    x,y,z,w=q; return np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],[2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],[2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])
cam,f=sys.argv[1],sys.argv[2]
exp=np.array(json.load(open(f))["T_base_cam"])
rclpy.init(); n=rclpy.create_node("claude_tfverify"); b=Buffer(); TransformListener(b,n)
t0=time.time()
while time.time()-t0<8: rclpy.spin_once(n,timeout_sec=0.1)
tr=b.lookup_transform("fr3_link0",f"{cam}_left_camera_optical_frame",rclpy.time.Time()).transform
T=np.eye(4); T[:3,:3]=q2R([tr.rotation.x,tr.rotation.y,tr.rotation.z,tr.rotation.w]); T[:3,3]=[tr.translation.x,tr.translation.y,tr.translation.z]
D=np.linalg.inv(exp)@T; mm=np.linalg.norm(D[:3,3])*1000; dg=math.degrees(math.acos(max(-1,min(1,(np.trace(D[:3,:3])-1)/2))))
print(f"{cam}: TF fr3_link0->optical origin {np.round(T[:3,3],4).tolist()} vs file {np.round(exp[:3,3],4).tolist()}  diff {mm:.2f} mm / {dg:.3f} deg", "OK" if mm<0.5 and dg<0.1 else "MISMATCH")
