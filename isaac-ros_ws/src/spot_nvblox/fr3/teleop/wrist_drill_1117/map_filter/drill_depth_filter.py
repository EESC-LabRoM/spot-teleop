"""Source-stamp matched target exclusion. Never re-stamp a cached object mask."""
import time,json,collections,math
import numpy as np
import cv2

def carve(depth,mask,dilate=8):
    if depth.ndim!=2 or mask.ndim!=2:raise ValueError('2D images required')
    if abs(depth.shape[1]/depth.shape[0]-mask.shape[1]/mask.shape[0])>.002:raise ValueError('unaligned aspect ratios')
    if depth.shape!=mask.shape:mask=cv2.resize(mask,(depth.shape[1],depth.shape[0]),interpolation=cv2.INTER_NEAREST)
    mask=(mask>0).astype(np.uint8)
    fraction=float(mask.mean())
    if not .0001<fraction<.20:raise ValueError('empty or oversized target mask')
    if dilate:mask=cv2.dilate(mask,cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(2*dilate+1,2*dilate+1)))
    removed=mask>0;out=depth.copy();out[removed]=0
    assert np.array_equal(out[~removed],depth[~removed],equal_nan=True)
    return out,removed

class Pairing:
    def __init__(self,max_age=.7,slop=.025):self.max_age=max_age;self.slop=slop;self.last=0
    def valid(self,depth_stamp,mask_stamp,now):
        return (all(math.isfinite(x) for x in [depth_stamp,mask_stamp,now]) and depth_stamp>self.last and abs(depth_stamp-mask_stamp)<=self.slop and 0<=now-mask_stamp<=self.max_age and 0<=now-depth_stamp<=self.max_age)

def main():
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data,QoSProfile,ReliabilityPolicy
    from sensor_msgs.msg import Image,CompressedImage
    rclpy.init();n=Node('wrist_drill_depth_filter');depths=collections.deque(maxlen=60);masks=collections.deque(maxlen=20);gate=Pairing(max_age=1.5);counts=collections.Counter();recent={}
    mask_topic=n.declare_parameter('mask_topic','/wrist_map/drill_mask/compressed').value
    pub=n.create_publisher(Image,'/wrist_map/drill_free_depth',qos_profile_sensor_data)
    maskpub=n.create_publisher(Image,'/wrist_map/exclusion_mask',qos_profile_sensor_data)
    def stamp(m):return m.header.stamp.sec+m.header.stamp.nanosec*1e-9
    def process():
        now=n.get_clock().now().nanoseconds*1e-9
        for m in list(masks):
            if not depths:break
            d=min(depths,key=lambda d:abs(stamp(d)-stamp(m)))
            if not gate.valid(stamp(d),stamp(m),now):
                if now-stamp(m)>gate.max_age or stamp(m)<=gate.last-.025:masks.remove(m);counts['stale_unmatched']+=1
                continue
            masks.remove(m)
            try:
                if d.encoding!='32FC1' or m.encoding!='mono8':raise ValueError('unsupported encoding')
                dtype=np.dtype('>f4' if d.is_bigendian else '<f4')
                dep=np.frombuffer(d.data,dtype=dtype).reshape(d.height,d.step//4)[:,:d.width]
                mask=np.frombuffer(m.data,np.uint8).reshape(m.height,m.step)[:,:m.width]
                out,removed=carve(dep,mask)
                msg=Image(header=d.header,height=d.height,width=d.width,encoding='32FC1',is_bigendian=False,step=d.width*4,data=np.ascontiguousarray(out,dtype='<f4').tobytes())
                mm=Image(header=d.header,height=d.height,width=d.width,encoding='mono8',step=d.width,data=(removed.astype(np.uint8)*255).tobytes())
                pub.publish(msg);maskpub.publish(mm);gate.last=stamp(d);counts['published']+=1
                recent.update(depth_mask_delta_ms=(stamp(d)-stamp(m))*1000,mask_capture_age_ms=(now-stamp(m))*1000,removed_fraction=float(removed.mean()),masked_nonzero=int(np.count_nonzero(out[removed])),outside_changed=int(np.count_nonzero(~np.isclose(out[~removed],dep[~removed],equal_nan=True))),stamp=stamp(d))
            except (ValueError,AssertionError) as e:counts['refused']+=1;recent['refusal']=str(e)
    def depth(m):depths.append(m);counts['depth']+=1;process()
    def mask(m):masks.append(m);counts['mask']+=1;process()
    def report():
        data=dict(wall=time.time(),counts=dict(counts),recent=recent);print(json.dumps(data),flush=True)
        with open('/tmp/wrist_map_filter/status.json','w') as f:json.dump(data,f)
    n.create_subscription(Image,'/franka_seg_zed1/depth',depth,qos_profile_sensor_data)
    def compressed(m):
        if 'png' not in m.format.lower():counts['non_png_refused']+=1;return
        a=cv2.imdecode(np.frombuffer(m.data,np.uint8),cv2.IMREAD_GRAYSCALE)
        if a is None:counts['decode_refused']+=1;return
        raw=Image(header=m.header,height=a.shape[0],width=a.shape[1],encoding='mono8',step=a.shape[1],data=a.tobytes())
        counts['compressed_bytes']+=len(m.data);mask(raw)
    n.create_subscription(CompressedImage,mask_topic,compressed,QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE))
    n.create_timer(2.,report)
    try:rclpy.spin(n)
    finally:report();n.destroy_node();rclpy.shutdown()
if __name__=='__main__':main()
