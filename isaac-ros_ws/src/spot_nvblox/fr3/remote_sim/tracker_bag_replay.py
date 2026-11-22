"""Offline, deterministic replay of an operator RGB-D bag through operator_hand_tracker.py.

Drives OperatorHandTracker.frame() directly (no DDS, no spin) on the bag's same-stamp RGB/depth pairs, with the
module's clock replaced by bag time, and compares every output against the tracker output recorded in the bag.

Clock: time.time_ns()/time.monotonic() inside the tracker return  stamp + input_age  for the frame being processed,
held constant during the frame, where input_age is the live run's recorded input_age_ms for that stamp (or --latency-ms
when absent / forced). Perception (detection, depth, palm frame, filters) is therefore reproduced exactly run to run;
compute-latency drops ("Processamento atrasado") are reported from the recording instead of re-simulated.

Frames: by default only the stamps the live tracker processed (--smooth-preview drops frames under load), so filter
dt and identity gates match the live run; --frames all processes every synchronized pair.

Arm calibration: the bags start already calibrated, so the tracker is warm-started like the live restore file
(length/scale from the first recorded diagnostic, shoulder from a MediaPipe Pose pre-pass over the opening frames).
--cold starts uncalibrated instead (tests calibration itself).

Usage (inside the operator image, repository mounted at /spot-teleop):
  python3 tracker_bag_replay.py BAG --out DIR [--video] [--frames recorded|all] [--cold] [--set module.Class.attr=value]
"""
import argparse,json,math,sys,time as real_time,types
from pathlib import Path
import numpy as np

HERE=Path(__file__).resolve().parent
sys.path.insert(0,str(HERE))

CAMERA=dict(rgb='/fr3_operator_cam/rgb',depth='/fr3_operator_cam/depth',info='/fr3_operator_cam/camera_info')
RECORDED=dict(diag='/fr3_operator/tracking',pose='/fr3_operator/hand_camera_pose',sample='/fr3_operator/teleop_sample')


class BagClock(types.ModuleType):
    """Stands in for the tracker module's `time`: bag time for gates, real perf_counter for timing diagnostics."""
    def __init__(self):
        super().__init__('time');self.now_ns=0
    def time_ns(self):return self.now_ns
    def time(self):return self.now_ns*1e-9
    def monotonic(self):return self.now_ns*1e-9
    def perf_counter(self):return real_time.perf_counter()
    def sleep(self,s):pass


def stamp_ns(header):return header.stamp.sec*10**9+header.stamp.nanosec


def reader(bag,topics):
    import rosbag2_py
    r=rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=str(bag),storage_id='sqlite3'),rosbag2_py.ConverterOptions('cdr','cdr'))
    r.set_filter(rosbag2_py.StorageFilter(topics=list(topics)))
    return r


def read_recorded(bag):
    from rclpy.serialization import deserialize_message
    from std_msgs.msg import String
    from geometry_msgs.msg import PoseStamped
    r=reader(bag,RECORDED.values());diags={};poses={};samples={}
    while r.has_next():
        topic,data,_=r.read_next()
        if topic==RECORDED['diag']:
            d=json.loads(deserialize_message(data,String).data);diags[d['stamp_ns']]=d
        elif topic==RECORDED['pose']:
            m=deserialize_message(data,PoseStamped);p=m.pose.position;o=m.pose.orientation
            poses[stamp_ns(m.header)]=(np.array([p.x,p.y,p.z]),np.array([o.x,o.y,o.z,o.w]))
        else:
            s=json.loads(deserialize_message(data,String).data);samples[s['stamp_ns']]=s
    return diags,poses,samples


def frame_pairs(bag):
    """Same-stamp (color, depth) pairs in stamp order, plus the camera info; mirrors the tracker's TimeSynchronizer."""
    from rclpy.serialization import deserialize_message
    from sensor_msgs.msg import Image,CameraInfo
    r=reader(bag,CAMERA.values());rgb={};depth={};info=None
    while r.has_next():
        topic,data,_=r.read_next()
        if topic==CAMERA['info']:
            if info is None:info=deserialize_message(data,CameraInfo);yield 'info',info
            continue
        m=deserialize_message(data,Image);ns=stamp_ns(m.header)
        (rgb if topic==CAMERA['rgb'] else depth)[ns]=m
        if ns in rgb and ns in depth:yield 'pair',(rgb.pop(ns),depth.pop(ns))
        for pending in (rgb,depth):   # TimeSynchronizer queue 6: old unmatched halves expire
            while len(pending)>6:pending.pop(min(pending))


def color_array(color):
    import cv2
    a=np.frombuffer(color.data,np.uint8).reshape(color.height,color.step)[:,:color.width*3].reshape(color.height,color.width,3)
    return a.copy() if color.encoding.lower()=='bgr8' else cv2.cvtColor(a,cv2.COLOR_RGB2BGR)


def image_arrays(color,depth_msg):
    image=color_array(color)
    d=np.frombuffer(depth_msg.data,np.float32).reshape(depth_msg.height,depth_msg.step//4)[:,:depth_msg.width]
    return image,d


def shoulder_prepass(bag,k,seconds=6.):
    """Median metric right shoulder over the opening seconds (MediaPipe Pose, as the tracker's arm reference)."""
    import cv2,mediapipe as mp
    from arm_pose_estimator.hand_frame import robust_depth
    pose=mp.solutions.pose.Pose(model_complexity=1,enable_segmentation=False,min_detection_confidence=.5,min_tracking_confidence=.5)
    found=[];first=None
    for kind,item in frame_pairs(bag):
        if kind!='pair':continue
        color,depth_msg=item;ns=stamp_ns(color.header);first=first or ns
        if (ns-first)*1e-9>seconds and len(found)>=5:break
        image,depth=image_arrays(color,depth_msg);h,w=image.shape[:2]
        result=pose.process(np.ascontiguousarray(cv2.cvtColor(image,cv2.COLOR_BGR2RGB)))
        if not result.pose_landmarks:continue
        s=result.pose_landmarks.landmark[12]
        if s.visibility<.7:continue
        u,v=s.x*w,s.y*h
        if not (4<=u<w-4 and 4<=v<h-4):continue
        z=robust_depth(depth,int(u),int(v),4,(.3,3.))
        if z is not None:found.append([(u-k[0,2])*z/k[0,0],(v-k[1,2])*z/k[1,1],z])
        if (ns-first)*1e-9>3*seconds:break
    pose.close()
    return (np.median(np.array(found),axis=0),len(found)) if found else (None,0)


STATE_COLORS={'6d':(90,200,90),'position':(60,180,235),'paused':(80,80,230)}   # BGR: green, amber, red
STATE_LABELS={'6d':'6D TRACKED','position':'POSITION ONLY (rotation held)','paused':'NO COMMAND (robot paused)'}


def ascii_text(text):
    import unicodedata
    return unicodedata.normalize('NFKD',str(text)).encode('ascii','ignore').decode()


def frame_state(diag):
    if diag.get('valid') and diag.get('orientation_valid'):return '6d'
    return 'position' if diag.get('valid') else 'paused'


def render_tracking(color,node,ns,diag,t):
    """Camera frame with what MediaPipe sees this frame: Pose skeleton, every Hands detection (operator's highlighted),
    palm axes on 6D frames, and a status banner with the operator alert."""
    import cv2,mediapipe as mp
    image=color_array(color);h,w=image.shape[:2]
    pose=node.preview_pose
    if pose is not None and 0<=ns-pose[0]<400_000_000:   # Pose runs ~3 Hz once calibrated: hold it briefly
        pts=[(int(x*w),int(y*h),v) for x,y,v in pose[1]]
        for a,b in mp.solutions.pose.POSE_CONNECTIONS:
            if pts[a][2]>.5 and pts[b][2]>.5:cv2.line(image,pts[a][:2],pts[b][:2],(200,170,110),1,cv2.LINE_AA)
        for a,b in ((12,14),(14,16)):   # right arm (orange): what calibration and identity use
            if pts[a][2]>.5 and pts[b][2]>.5:cv2.line(image,pts[a][:2],pts[b][:2],(40,150,255),4,cv2.LINE_AA)
        for i in (12,14,16):
            if pts[i][2]>.5:cv2.circle(image,pts[i][:2],6,(40,150,255),-1,cv2.LINE_AA)
    chosen=node.preview_hand[1] if node.preview_hand is not None and node.preview_hand[0]==ns else None
    for stamp,pixels in node.preview_hands:
        if stamp!=ns:continue
        operator=chosen is not None and np.allclose(pixels,chosen)
        line,dot=((80,230,120),(30,90,255)) if operator else ((150,150,150),(110,110,110))
        p=pixels.astype(int)
        for a,b in mp.solutions.hands.HAND_CONNECTIONS:cv2.line(image,tuple(p[a]),tuple(p[b]),line,2,cv2.LINE_AA)
        for q in p:cv2.circle(image,tuple(q),3,dot,-1,cv2.LINE_AA)
        if operator:cv2.circle(image,tuple(p[0]),9,(80,230,120),2,cv2.LINE_AA)
    axes=node.preview_axes
    if axes is not None and axes[0]==ns and node.k is not None:
        k=node.k;origin,rotation=axes[1],axes[2]
        project=lambda q:(int(k[0,0]*q[0]/q[2]+k[0,2]),int(k[1,1]*q[1]/q[2]+k[1,2]))
        for i,c in enumerate(((0,0,255),(0,255,0),(255,0,0))):
            end=origin+.08*rotation[:,i]
            if end[2]>.05:cv2.arrowedLine(image,project(origin),project(end),c,3,cv2.LINE_AA,tipLength=.25)
    state=frame_state(diag)
    overlay=image.copy();cv2.rectangle(overlay,(0,0),(w,64),(20,20,20),-1);image=cv2.addWeighted(overlay,.6,image,.4,0)
    cv2.rectangle(image,(0,0),(8,64),STATE_COLORS[state],-1)
    cv2.putText(image,STATE_LABELS[state],(18,25),cv2.FONT_HERSHEY_SIMPLEX,.7,STATE_COLORS[state],2,cv2.LINE_AA)
    info=f't {t:6.1f} s | hands {diag.get("hands_detected","-")}'
    if diag.get('depth_m'):info+=f' | depth {diag["depth_m"]:.2f} m'
    if diag.get('gesture'):info+=f' | {diag["gesture"]}'
    (tw,_),_=cv2.getTextSize(info,cv2.FONT_HERSHEY_SIMPLEX,.5,1)
    cv2.putText(image,info,(w-tw-12,25),cv2.FONT_HERSHEY_SIMPLEX,.5,(230,230,230),1,cv2.LINE_AA)
    message=diag.get('alert') or ('' if state=='6d' else diag.get('reason',''))
    if message:cv2.putText(image,ascii_text(message)[:95],(18,52),cv2.FONT_HERSHEY_SIMPLEX,.5,(235,235,235),1,cv2.LINE_AA)
    return image


def add_timeline(directory,states,stamps):
    """Second pass: a state strip (green 6D / amber position / red paused) with a cursor under every rendered frame."""
    import cv2
    files=sorted(Path(directory).glob('*.jpg'));t0,t1=stamps[0],stamps[-1];span=max(1,t1-t0)
    first=cv2.imread(str(files[0]));w=first.shape[1];strip=np.zeros((22,w,3),np.uint8)
    xs=[int((s-t0)/span*(w-1)) for s in stamps]
    for i,(x,state) in enumerate(zip(xs,states)):
        x2=xs[i+1] if i+1<len(xs) else w
        strip[4:18,x:max(x+1,x2)]=STATE_COLORS[state]
    for f,x in zip(files,xs):
        img=cv2.imread(str(f));bar=strip.copy();cv2.line(bar,(x,0),(x,21),(255,255,255),2)
        cv2.imwrite(str(f),np.vstack([img,bar]),[cv2.IMWRITE_JPEG_QUALITY,93])


def quat_angle(a,b):
    return math.degrees(2*math.acos(min(1.,abs(float(np.dot(a/np.linalg.norm(a),b/np.linalg.norm(b)))))))


def apply_overrides(pairs):
    """--set module.attr=value or module.Class.attr=value (e.g. hand_tracking_math.OneEuroPosition.__init__ defaults
    are not reachable this way; use it for module constants and class attributes)."""
    import importlib
    for item in pairs:
        target,value=item.split('=',1);parts=target.split('.')
        obj=importlib.import_module(parts[0])
        for p in parts[1:-1]:obj=getattr(obj,p)
        setattr(obj,parts[-1],json.loads(value));print(f'override {target} = {value}',flush=True)


def main():
    ap=argparse.ArgumentParser(description=__doc__,formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('bag');ap.add_argument('--out',required=True)
    ap.add_argument('--frames',choices=('recorded','all'),default='recorded')
    ap.add_argument('--latency-ms',type=float,default=None,help='force a constant input age instead of the recorded one')
    ap.add_argument('--max-input-age',type=float,default=.25);ap.add_argument('--max-output-age',type=float,default=.35)
    ap.add_argument('--cold',action='store_true',help='no warm start: calibrate the arm from the bag')
    ap.add_argument('--video',action='store_true',help='write the tracker overlay as overlay.mp4')
    ap.add_argument('--render',action='store_true',help='write a MediaPipe tracking visualization as render/NNNNN.jpg '
                    '(+ render/fps.txt for encoding at the bag rate)')
    ap.add_argument('--set',action='append',default=[],metavar='MOD.ATTR=JSON')
    ap.add_argument('--tracker-kwargs',default='{}',help='JSON extra OperatorHandTracker kwargs (e.g. robot_reach)')
    args=ap.parse_args()
    out=Path(args.out);out.mkdir(parents=True,exist_ok=True)

    import rclpy,cv2
    import operator_hand_tracker as oht
    apply_overrides(args.set)
    clock=BagClock();oht.time=clock

    diags,poses,samples=read_recorded(args.bag)
    if not diags:sys.exit('bag has no recorded tracker diagnostics; run with --frames all --latency-ms N')
    first=diags[min(diags)]
    median_age=float(np.median([d.get('input_age_ms',0.) for d in diags.values()]))

    rclpy.init(args=[])
    node=oht.OperatorHandTracker(max_input_age=args.max_input_age,max_output_age=args.max_output_age,
        calibrate_arm=True,smooth_preview=False,**json.loads(args.tracker_kwargs))
    captured={}
    def capture(name):
        def publish(msg):captured[name]=msg
        return publish
    for name in ('sample_pub','camera_pose_pub','diag_pub','overlay_pub'):getattr(node,name).publish=capture(name)
    node.tf_broadcaster.sendTransform=lambda *a,**k:None
    control_wrist=node.control_wrist;origins={}
    def recording_control_wrist(origin,ns):   # the camera-frame wrist behind every published command
        origins[ns]=np.asarray(origin,float).tolist();return control_wrist(origin,ns)
    node.control_wrist=recording_control_wrist

    warm={}
    rows=[];writer=None;started=real_time.perf_counter();processed=0
    for kind,item in frame_pairs(args.bag):
        if kind=='info':
            node.info(item)
            if not args.cold and node.arm.scale is None:
                shoulder,n=shoulder_prepass(args.bag,node.k)
                if shoulder is None:print('WARNING: no shoulder in the opening frames; falling back to --cold',flush=True)
                else:
                    node.arm.length=float(first['arm_length_m']);node.arm.scale=float(first['arm_scale']);node.arm.shoulder=shoulder
                    warm=dict(length=node.arm.length,scale=node.arm.scale,shoulder=shoulder.round(3).tolist(),shoulder_frames=n)
                    print(f'warm start: arm {node.arm.length:.3f} m, scale {node.arm.scale:.3f}, shoulder {warm["shoulder"]} ({n} frames)',flush=True)
            continue
        color,depth_msg=item;ns=stamp_ns(color.header)
        live=diags.get(ns)
        if args.frames=='recorded' and live is None:continue
        age=args.latency_ms if args.latency_ms is not None else (live.get('input_age_ms',median_age) if live else median_age)
        clock.now_ns=ns+int(age*1e6)
        captured.clear()
        node.frame(color,depth_msg);processed+=1
        diag=json.loads(captured['diag_pub'].data)
        sample=json.loads(captured['sample_pub'].data) if 'sample_pub' in captured else None
        pose=None
        if 'camera_pose_pub' in captured:
            m=captured['camera_pose_pub'].pose
            pose=[m.position.x,m.position.y,m.position.z,m.orientation.x,m.orientation.y,m.orientation.z,m.orientation.w]
        rec_pose=poses.get(ns)
        body=node.body_wrist[1].tolist() if node.body_wrist is not None and node.body_wrist[0]==ns else None
        hand_px=node.preview_hand[1][0].round(1).tolist() if node.preview_hand is not None and node.preview_hand[0]==ns else None
        row=dict(stamp_ns=ns,input_age_ms=age,diag=diag,sample=sample,pose=pose,live=live,origin=origins.pop(ns,None),body_wrist=body,hand_px=hand_px,
            body_shoulder=node.body_shoulder[1].tolist() if node.body_shoulder is not None and node.body_shoulder[0]==ns else None,
            live_pose=(rec_pose[0].tolist()+rec_pose[1].tolist()) if rec_pose is not None else None,
            live_sample=samples.get(ns))
        rows.append(row)
        if args.render:
            render_dir=out/'render';render_dir.mkdir(exist_ok=True)
            cv2.imwrite(str(render_dir/f'{len(rows)-1:05d}.jpg'),render_tracking(color,node,ns,diag,(ns-rows[0]['stamp_ns'])*1e-9),
                        [cv2.IMWRITE_JPEG_QUALITY,93])
        if args.video:   # one video frame per row (black when the tracker drew none), so frame index == row index
            if 'overlay_pub' in captured:
                o=captured['overlay_pub'];img=np.frombuffer(bytes(o.data),np.uint8).reshape(o.height,o.width,3).copy()
            else:
                o=color;img=np.zeros((color.height,color.width,3),np.uint8)
                cv2.putText(img,diag['reason'][:80],(8,22),cv2.FONT_HERSHEY_SIMPLEX,.48,(40,60,255),1)
            live_text=f'LIVE: {"6D" if live and live["valid"] and rec_pose is not None else (live["reason"][:60] if live else "-")}'
            cv2.putText(img,live_text,(8,o.height-12),cv2.FONT_HERSHEY_SIMPLEX,.45,(255,255,0),1)
            cv2.putText(img,f't={(ns-min(diags))*1e-9:6.2f}s',(o.width-110,o.height-12),cv2.FONT_HERSHEY_SIMPLEX,.45,(255,255,255),1)
            if writer is None:writer=cv2.VideoWriter(str(out/'overlay.mp4'),cv2.VideoWriter_fourcc(*'mp4v'),10.,(o.width,o.height))
            writer.write(img)
    if writer is not None:writer.release()
    if args.render and rows:
        stamps=[r['stamp_ns'] for r in rows]
        add_timeline(out/'render',[frame_state(r['diag']) for r in rows],stamps)
        span=(stamps[-1]-stamps[0])*1e-9
        (out/'render'/'fps.txt').write_text(f'{(len(rows)-1)/span if span>0 else 10.:.4f}\n')
    node.destroy_node();rclpy.shutdown()
    with open(out/'frames.jsonl','w') as f:
        for r in rows:f.write(json.dumps(r)+'\n')
    summary=summarize(rows)
    summary.update(bag=str(args.bag),frames_mode=args.frames,cold=args.cold,warm_start=warm,overrides=args.set,
        replay_seconds=round(real_time.perf_counter()-started,1),processed=processed,
        live_latency_drops=sum(1 for d in diags.values() if d['reason'] in ('Processamento atrasado; pose descartada','Quadro atrasado ou repetido')))
    (out/'summary.json').write_text(json.dumps(summary,indent=1))
    print(json.dumps(summary,indent=1))


def gaps(stamps,valid,threshold):
    """Seconds-long stretches without a valid output, between the first and last processed frame."""
    out=[];last=stamps[0]
    for s,v in zip(stamps,valid):
        if v:
            if (s-last)*1e-9>threshold:out.append(round((s-last)*1e-9,2))
            last=s
    if (stamps[-1]-last)*1e-9>threshold:out.append(round((stamps[-1]-last)*1e-9,2))
    return out


def summarize(rows):
    from collections import Counter
    n=len(rows);stamps=[r['stamp_ns'] for r in rows]
    six=[r['pose'] is not None for r in rows]
    pos=[r['sample'] is not None for r in rows]
    live6=[r['live_pose'] is not None for r in rows]
    livepos=[r['live_sample'] is not None for r in rows]
    s=dict(frames=n,duration_s=round((stamps[-1]-stamps[0])*1e-9,1),
        replay=dict(valid_6d=sum(six),position_any=sum(pos),valid_6d_pct=round(100*sum(six)/n,1)),
        live=dict(valid_6d=sum(live6),position_any=sum(livepos),valid_6d_pct=round(100*sum(live6)/n,1)),
        reasons=dict(Counter(r['diag']['reason'] for r in rows).most_common()),
        agreement=dict(both_6d=sum(a and b for a,b in zip(six,live6)),replay_only=sum(a and not b for a,b in zip(six,live6)),
                       live_only=sum(b and not a for a,b in zip(six,live6))))
    both=[r for r in rows if r['pose'] and r['live_pose']]
    if both:
        dp=np.array([np.linalg.norm(np.array(r['pose'][:3])-np.array(r['live_pose'][:3])) for r in both])*1000
        dq=np.array([quat_angle(np.array(r['pose'][3:]),np.array(r['live_pose'][3:])) for r in both])
        s['agreement'].update(pos_diff_mm=dict(p50=round(float(np.median(dp)),1),p95=round(float(np.percentile(dp,95)),1),max=round(float(dp.max()),1)),
                              rot_diff_deg=dict(p50=round(float(np.median(dq)),2),p95=round(float(np.percentile(dq,95)),2),max=round(float(dq.max()),2)))
    s['dropouts']=dict(replay_gaps_over_1s=gaps(stamps,pos,1.),live_gaps_over_1s=gaps(stamps,livepos,1.),
                       replay_6d_gaps_over_1s=gaps(stamps,six,1.))
    s['motion']=dict(replay=motion([(r['stamp_ns'],r['pose']) for r in rows if r['pose']],[(r['stamp_ns'],r['sample']) for r in rows if r['sample']]),
                     live=motion([(r['stamp_ns'],r['live_pose']) for r in rows if r['live_pose']],[(r['stamp_ns'],r['live_sample']) for r in rows if r['live_sample']]))
    g=[r['sample'].get('gesture') for r in rows if r['sample'] and r['sample'].get('orientation_valid')]
    s['gesture']=dict(Counter(g).most_common(),flips=sum(1 for a,b in zip(g,g[1:]) if a and b and a!=b))
    return s


def motion(poses,samples):
    """Camera-frame wrist/palm steps (measurement) and robot-frame command steps (filtered, scaled)."""
    out={}
    if len(poses)>2:
        P=np.array([p[:3] for _,p in poses]);Q=np.array([p[3:] for _,p in poses]);t=np.array([s for s,_ in poses])*1e-9
        dt=np.diff(t);close=dt<.35
        step=np.linalg.norm(np.diff(P,axis=0),axis=1)[close]*1000
        rot=np.array([quat_angle(a,b) for a,b in zip(Q,Q[1:])])[close]
        out['camera']=dict(pos_std_mm=np.round(P.std(axis=0)*1000,1).tolist(),
            step_mm=dict(p50=round(float(np.median(step)),1),p95=round(float(np.percentile(step,95)),1),max=round(float(step.max()),1)) if len(step) else None,
            rot_step_deg=dict(p50=round(float(np.median(rot)),2),p95=round(float(np.percentile(rot,95)),2),max=round(float(rot.max()),2)) if len(rot) else None,
            pos_jumps_over_100mm=int((step>100).sum()) if len(step) else 0,rot_jumps_over_30deg=int((rot>30).sum()) if len(rot) else 0)
    if len(samples)>2:
        P=np.array([s['p'] for _,s in samples]);t=np.array([k for k,_ in samples])*1e-9
        dt=np.diff(t);close=dt<.35
        step=np.linalg.norm(np.diff(P,axis=0),axis=1)[close]*1000
        out['command']=dict(pos_std_mm=np.round(P.std(axis=0)*1000,1).tolist(),
            step_mm=dict(p50=round(float(np.median(step)),1),p95=round(float(np.percentile(step,95)),1),max=round(float(step.max()),1)) if len(step) else None,
            jumps_over_100mm=int((step>100).sum()) if len(step) else 0,jumps_over_300mm=int((step>300).sum()) if len(step) else 0)
    return out


if __name__=='__main__':main()
