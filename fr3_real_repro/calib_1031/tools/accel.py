import time, json, math, numpy as np, rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Imu
from tf2_ros import Buffer, TransformListener
def q2R(q):
    x,y,z,w=q; return np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],[2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],[2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])
rclpy.init(); n=Node("claude_accel_passive"); buf=Buffer(); TransformListener(buf,n)
qos=QoSProfile(depth=5,reliability=ReliabilityPolicy.BEST_EFFORT); I={}
for c in ("zed1","zed2"): n.create_subscription(Imu,f"/{c}/zed_node/imu/data",lambda m,c=c:I.setdefault(c,[]).append(m),qos)
t0=time.time()
while time.time()-t0<10: rclpy.spin_once(n,timeout_sec=0.05)
LIVE={"zed1":np.array([[0.93985531751948,-0.0520260173566148,0.33758743407948266],[-0.32471128128934884,-0.44277441666218215,0.8357711407753314],[0.10599323532089164,-0.8951223991541439,-0.43303732471778683]]),
      "zed2":np.array([[-0.12258687895933901,0.9881446542177355,-0.09242618377890126],[0.992398879391968,0.12103222958276975,-0.022263503403547578],[-0.010813014777785934,-0.09445285460560106,-0.9954706108008768]])}
NEWR={"zed1": [[0.9341506361961365, -0.05573511868715286, 0.35249990224838257], [-0.33720964193344116, -0.461233526468277, 0.8207029700279236], [0.11684276908636093, -0.8855265378952026, -0.44965600967407227]], "zed2": [[-0.6344212889671326, 0.7722437381744385, 0.03390238434076309], [0.7724210023880005, 0.6350250840187073, -0.010438979603350163], [-0.02959030121564865, 0.019564200192689896, -0.999370813369751]]}
for c in ("zed1","zed2"):
    L=I.get(c,[])
    if not L: print(c,"no imu"); continue
    fr=L[-1].header.frame_id; A=np.array([[m.linear_acceleration.x,m.linear_acceleration.y,m.linear_acceleration.z] for m in L]); a=A.mean(0); spread=A.std(0)
    for _ in range(30): rclpy.spin_once(n,timeout_sec=0.1)
    try: tr=buf.lookup_transform(f"{c}_left_camera_optical_frame",f"{c}_camera_center",rclpy.time.Time()).transform; r=tr.rotation; Rop_imu=q2R([r.x,r.y,r.z,r.w])   # imu vectors -> optical
    except Exception as e: print(c,"no tf",fr,str(e)[:80]); continue
    up_imu=a/np.linalg.norm(a); down_opt=-(Rop_imu@up_imu)
    print(f"## {c}: imu frame {fr}, |acc| {np.linalg.norm(a):.2f} m/s2 (std {spread.round(2).tolist()}), {len(L)} samples; IMU down in optical frame {np.round(down_opt,3).tolist()}")
    for nm,R in (("live10-23",LIVE[c]),("NEW staged",np.array(NEWR[c]))):
        d=R.T@np.array([0,0,-1.0]); ang=math.degrees(math.acos(np.clip(down_opt@d,-1,1))); print(f"   {nm:10s}: base-down in optical {np.round(d,3).tolist()}  -> angle to IMU-down {ang:5.2f} deg")
