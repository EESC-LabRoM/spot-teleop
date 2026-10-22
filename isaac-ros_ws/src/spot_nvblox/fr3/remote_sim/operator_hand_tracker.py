"""Simulation operator tracking: full-frame VIDEO hand model, fresh ZED depth, no torso dependency."""
import argparse,json,math,socket,sys,time,threading
from pathlib import Path
from array import array
from http.server import BaseHTTPRequestHandler,ThreadingHTTPServer
from mediapipe.framework.formats import landmark_pb2
import numpy as np
import cv2
import mediapipe as mp
import message_filters
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile,ReliabilityPolicy
from sensor_msgs.msg import Image,CameraInfo
from geometry_msgs.msg import PoseStamped,TransformStamped
from tf2_ros import TransformBroadcaster
from std_msgs.msg import String
HERE=Path(__file__).resolve().parent
sys.path.insert(0,str(HERE.parents[4]/'spot-ros2_ws/src/arm_pose_estimator'))
from arm_pose_estimator.hand_frame import (robust_depth,palm_rotation,matrix_quaternion,
    quaternion_matrix,HAND_TO_FR3_TCP,OrientationFilter,GestureOrientationLock,fingers_open)
from hand_tracking_math import CAMERA_TO_ROBOT,OneEuroPosition,fused_wrist_depth,finger_distance_signals
from arm_calibration import ArmCalibration

class OperatorHandTracker(Node):
    def __init__(self, domain=188, camera_prefix="/fr3_sim/operator", output_prefix="/fr3_sim/operator", max_input_age=.55, max_output_age=.65, calibrate_arm=False, calibration_port=0, smooth_preview=False, preview_port=0):
        super().__init__('fr3_remote_hand_tracker')
        self.max_input_age,self.max_output_age=max_input_age,max_output_age
        self.preview_port=preview_port;self.jpeg=None;self.jpeg_ns=0;self.ros_preview_at=0.;self.preview_servers=[];self.preview_axes=None;self.preview_body_axes=None
        self.smooth_preview=smooth_preview;self.preview_pose=None;self.preview_hand=None;self.preview_diag={};self.preview_at=0.
        self.pending=None;self.condition=threading.Condition();self.stopping=False;self.reset_requested=False;self.force_reset_requested=False
        self.arm=ArmCalibration() if calibrate_arm else None
        self.body=mp.solutions.pose.Pose(model_complexity=1,enable_segmentation=False,min_detection_confidence=.5,min_tracking_confidence=.5) if calibrate_arm else None
        self.body_at=0.;self.body_marks=None;self.hand_identity=None
        self.calibration_socket=None
        if calibration_port:
            if not calibrate_arm:raise ValueError('Calibration port requires arm calibration')
            self.calibration_socket=socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
            self.calibration_socket.bind(('127.0.0.1',calibration_port));self.calibration_socket.setblocking(False)
            self.create_timer(.02,self.calibration_requests)
        self.k=None;self.last_ns=-1;self.times=[];self.frames=0
        self.position=OneEuroPosition();self.orientation=OrientationFilter(timeout=.65)
        self.clutch=GestureOrientationLock(duration=.4)
        options=mp.tasks.vision.GestureRecognizerOptions(
            base_options=mp.tasks.BaseOptions(model_asset_path=str(HERE.parents[4]/'spot-ros2_ws/src/arm_pose_estimator/models/gesture_recognizer.task')),
            running_mode=mp.tasks.vision.RunningMode.VIDEO,num_hands=2,
            min_hand_detection_confidence=.5,min_hand_presence_confidence=.5,min_tracking_confidence=.3)
        self.detector=mp.tasks.vision.GestureRecognizer.create_from_options(options)
        self.qos=QoSProfile(depth=1,reliability=ReliabilityPolicy.BEST_EFFORT)
        self.sample_pub=self.create_publisher(String,output_prefix+'/sample',self.qos)
        self.camera_pose_pub=self.create_publisher(PoseStamped,output_prefix+'/hand_camera_pose',self.qos)
        self.tf_broadcaster=TransformBroadcaster(self)
        self.diag_pub=self.create_publisher(String,output_prefix+'/tracking',self.qos)
        self.overlay_pub=self.create_publisher(Image,output_prefix+'/overlay',QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE))
        self.create_subscription(CameraInfo,camera_prefix+'/camera_info',self.info,1)
        self.rgb=message_filters.Subscriber(self,Image,camera_prefix+'/rgb',qos_profile=self.qos)
        self.depth=message_filters.Subscriber(self,Image,camera_prefix+'/depth',qos_profile=self.qos)
        self.sync=message_filters.TimeSynchronizer([self.rgb,self.depth],2)
        if smooth_preview:
            self.sync.registerCallback(self.enqueue)
            self.rgb.registerCallback(self.preview)
            self.worker=threading.Thread(target=self.process_latest,daemon=True);self.worker.start()
        else:self.sync.registerCallback(self.frame)
        if preview_port:
            node=self
            class PreviewHandler(BaseHTTPRequestHandler):
                def log_message(self,*args):pass
                def do_GET(self):
                    data,stamp=node.jpeg,node.jpeg_ns
                    age=(time.time_ns()-stamp)*1e-9
                    if self.path.split('?')[0]!='/snapshot' or data is None or age>1.:
                        self.send_response(503);self.end_headers();return
                    self.send_response(200);self.send_header('Content-Type','image/jpeg')
                    self.send_header('Cache-Control','no-store');self.send_header('Content-Length',str(len(data)))
                    self.send_header('X-Capture-Age-Ms',str(round(age*1000)));self.end_headers()
                    try:self.wfile.write(data)
                    except (BrokenPipeError,ConnectionResetError):pass
            for host in ('127.0.0.1','100.88.134.127'):
                server=ThreadingHTTPServer((host,preview_port),PreviewHandler);server.daemon_threads=True
                threading.Thread(target=server.serve_forever,daemon=True).start();self.preview_servers.append(server)
        print(f'DIRECT HAND TRACKER ready: same-frame RGB-D, Right hand, DDS {domain}, output {output_prefix}; publishes perception only',flush=True)

    def calibration_requests(self):
        try:
            while True:
                data,_=self.calibration_socket.recvfrom(512)
                try:request=json.loads(data)
                except (ValueError,UnicodeError):continue
                if request in ({'action':'calibrate_arm','version':1},{'action':'reset_person','version':1}):
                    force=request['action']=='reset_person'
                    if self.smooth_preview:
                        self.force_reset_requested=self.force_reset_requested or force;self.reset_requested=True
                    else:self.reset_calibration(force=force)
        except BlockingIOError:pass

    def reset_calibration(self,force=False):
        if force:self.arm.reset()
        self.position=OneEuroPosition();self.orientation=OrientationFilter(timeout=.65)
        self.clutch=GestureOrientationLock(duration=.4);self.body_at=0.;self.hand_identity=None
        self.preview_hand=None;self.preview_pose=None
        print('Start: preserving measured arm calibration/progress; resetting relative hand reference only',flush=True)

    def enqueue(self,color,depth):
        with self.condition:
            self.pending=(color,depth);self.condition.notify()

    def process_latest(self):
        while True:
            with self.condition:
                self.condition.wait_for(lambda:self.pending is not None or self.stopping,timeout=.2)
                if self.stopping:return
                if self.pending is None:continue
                pair=self.pending;self.pending=None
            if self.reset_requested:
                self.reset_requested=False
                force=self.force_reset_requested;self.force_reset_requested=False;self.reset_calibration(force=force)
            self.frame(*pair)

    @staticmethod
    def bgr(color):
        encoding=color.encoding.lower();channels=4 if encoding in ('bgra8','rgba8') else 3
        a=np.frombuffer(color.data,np.uint8).reshape(color.height,color.step)[:,:color.width*channels].reshape(color.height,color.width,channels)
        if encoding=='bgra8':return cv2.cvtColor(a,cv2.COLOR_BGRA2BGR)
        if encoding=='rgba8':return cv2.cvtColor(a,cv2.COLOR_RGBA2BGR)
        if encoding=='rgb8':return cv2.cvtColor(a,cv2.COLOR_RGB2BGR)
        if encoding=='bgr8':return a.copy()
        raise ValueError('Formato RGB não suportado')

    def preview(self,color):
        now=time.monotonic()
        if now-self.preview_at<.045:return
        self.preview_at=now
        ns=color.header.stamp.sec*10**9+color.header.stamp.nanosec
        if not 0<=(time.time_ns()-ns)*1e-9<self.max_output_age:return
        try:
            a=self.bgr(color);h,w=a.shape[:2]
            pose=self.preview_pose
            if pose is not None and 0<=ns-pose[0]<800000000:
                landmarks=landmark_pb2.NormalizedLandmarkList(landmark=[landmark_pb2.NormalizedLandmark(x=float(x),y=float(y),visibility=float(v)) for x,y,v in pose[1]])
                mp.solutions.drawing_utils.draw_landmarks(a,landmarks,mp.solutions.pose.POSE_CONNECTIONS,landmark_drawing_spec=mp.solutions.drawing_styles.get_default_pose_landmarks_style())
                x,y,visibility=pose[1][16]
                if visibility>.5:cv2.circle(a,(int(x*w),int(y*h)),10,(0,255,0),-1)
            for axes in (self.preview_body_axes,self.preview_axes):
                if axes is None or not 0<=ns-axes[0]<500000000 or self.k is None:continue
                origin,rotation=axes[1:]
                def project(point):
                    return (int(self.k[0,0]*point[0]/point[2]+self.k[0,2]),int(self.k[1,1]*point[1]/point[2]+self.k[1,2]))
                start=project(origin)
                for i,color_axis in enumerate(((0,0,255),(0,255,0),(255,0,0))):
                    endpoint=origin+.08*rotation[:,i]
                    if endpoint[2]>.05:cv2.arrowedLine(a,start,project(endpoint),color_axis,2)
            hand=self.preview_hand
            if hand is not None and 0<=ns-hand[0]<300000000:
                cv2.circle(a,tuple(hand[1][0].astype(int)),10,(0,255,0),-1)
                points=landmark_pb2.NormalizedLandmarkList(landmark=[landmark_pb2.NormalizedLandmark(x=float(x/w),y=float(y/h)) for x,y in hand[1]])
                mp.solutions.drawing_utils.draw_landmarks(a,points,mp.solutions.hands.HAND_CONNECTIONS)
            d=self.preview_diag
            cv2.putText(a,str(d.get('reason','Aguardando MediaPipe')),(8,20),cv2.FONT_HERSHEY_SIMPLEX,.44,(40,220,40) if d.get('valid') else (40,60,255),1)
            cv2.putText(a,f'Captura {(time.time_ns()-ns)*1e-6:.0f} ms | tracking {d.get("fps",0)} Hz | {d.get("gesture") or d.get("raw_gesture") or "sem mao/gesto"}',(8,41),cv2.FONT_HERSHEY_SIMPLEX,.44,(255,255,255),1)
            if d.get('arm_scale'):
                cv2.putText(a,f'Braco {d["arm_length_m"]:.2f} m | escala {d["arm_scale"]:.2f}',(8,62),cv2.FONT_HERSHEY_SIMPLEX,.44,(255,180,20),1)
            if self.preview_port:
                ok,jpeg=cv2.imencode('.jpg',a,[cv2.IMWRITE_JPEG_QUALITY,65])
                if ok:self.jpeg=jpeg.tobytes();self.jpeg_ns=ns
            if not self.preview_port or now-self.ros_preview_at>=.5:
                self.ros_preview_at=now
                out=Image();out.header=color.header;out.height,out.width=h,w;out.step=w*3;out.encoding='bgr8';out.data=array('B',a.tobytes());self.overlay_pub.publish(out)
        except (ValueError,cv2.error):pass

    def arm_reference(self,rgb,depth,ns,image,diag):
        if self.arm is None:return
        now=time.monotonic()
        interval=.18 if self.arm.scale is None else .33
        if now-self.body_at>=interval:
            self.body_at=now
            result=self.body.process(rgb)
            self.preview_pose=(ns,[(p.x,p.y,p.visibility) for p in result.pose_landmarks.landmark]) if result.pose_landmarks else None
            self.body_marks=None
            if result.pose_landmarks:
                points=result.pose_landmarks.landmark
                ids=(12,14,16)
                pixels=np.array([[points[i].x*image.shape[1],points[i].y*image.shape[0]] for i in ids])
                xyz=[]
                if all(points[i].visibility>=.7 for i in ids):
                    for u,v in pixels:
                        z=robust_depth(depth,int(u),int(v),4,(.3,3.)) if 4<=u<image.shape[1]-4 and 4<=v<image.shape[0]-4 else None
                        if z is None:break
                        xyz.append([(u-self.k[0,2])*z/self.k[0,0],(v-self.k[1,2])*z/self.k[1,1],z])
                    if len(xyz)==3 and self.arm.observe(*xyz,ns*1e-9):
                        self.body_marks=pixels;self.preview_body_axes=(ns,np.array(xyz[0]),CAMERA_TO_ROBOT.T)
        if self.body_marks is not None and now-self.body_at<.5:
            pts=self.body_marks.astype(int)
            cv2.polylines(image,[pts],False,(255,180,20),2)
            for point in pts:cv2.circle(image,tuple(point),5,(255,180,20),-1)
        diag.update(self.arm.status())

    def info(self,msg):self.k=np.asarray(msg.k).reshape(3,3)

    def frame(self,color,depth_msg):
        started=time.perf_counter();ns=color.header.stamp.sec*10**9+color.header.stamp.nanosec
        diag=dict(valid=False,reason='Mostre a mão direita',stamp_ns=ns)
        image=None;gesture=None;gesture_score=0.;support=0;distance=None
        try:
            age=(time.time_ns()-ns)*1e-9
            diag['input_age_ms']=round(age*1000,1)
            if self.k is None:raise ValueError('Aguardando calibração da câmera')
            if ns<=self.last_ns or not 0<=age<self.max_input_age:raise ValueError('Quadro atrasado ou repetido')
            self.last_ns=ns
            channels=4 if color.encoding.lower() in ('bgra8','rgba8') else 3
            if color.encoding.lower() not in ('bgr8','rgb8','bgra8','rgba8'):raise ValueError('Formato RGB não suportado')
            image=np.frombuffer(color.data,np.uint8).reshape(color.height,color.step)[:,:color.width*channels].reshape(color.height,color.width,channels)
            if color.encoding.lower()=='bgra8':image=cv2.cvtColor(image,cv2.COLOR_BGRA2BGR)
            elif color.encoding.lower()=='rgba8':image=cv2.cvtColor(image,cv2.COLOR_RGBA2BGR)
            elif color.encoding.lower()=='rgb8':image=cv2.cvtColor(image,cv2.COLOR_RGB2BGR)
            else:image=image.copy()
            if depth_msg.encoding!='32FC1' or (depth_msg.height,depth_msg.width)!=(color.height,color.width):
                raise ValueError('RGB/profundidade precisam estar alinhados e em metros')
            depth=np.frombuffer(depth_msg.data,np.float32).reshape(depth_msg.height,depth_msg.step//4)[:,:depth_msg.width]
            rgb=np.ascontiguousarray(cv2.cvtColor(image,cv2.COLOR_BGR2RGB))
            self.arm_reference(rgb,depth,ns,image,diag)
            t=time.perf_counter()
            result=self.detector.recognize_for_video(mp.Image(image_format=mp.ImageFormat.SRGB,data=rgb),ns//10**6)
            diag['model_ms']=round((time.perf_counter()-t)*1000,1)
            candidates=[]
            for i,hand in enumerate(result.hand_landmarks):
                category=result.handedness[i][0]
                anchored=self.arm is not None and self.arm.scale is not None
                if category.score>=.5 and (anchored or category.category_name=='Right'):
                    candidates.append((float(category.score),i))
            diag['hands_detected']=len(result.hand_landmarks)
            if not candidates:raise ValueError('Mão não detectada: mantenha dedos e punho visíveis acima da mesa')
            if self.arm is None and len(candidates)>1 and abs(candidates[0][0]-candidates[1][0])<.1:
                raise ValueError('Separe as mãos para identificar a direita')
            if self.arm is not None and self.arm.scale is not None:
                reference=None
                if self.hand_identity is not None and ns*1e-9-self.hand_identity[1]<1.5:
                    reference,ref_stamp=self.hand_identity
                elif self.arm.reference_stamp is not None and ns*1e-9-self.arm.reference_stamp<2.5:
                    reference,ref_stamp=self.arm.wrist_reference,self.arm.reference_stamp
                if reference is None:raise ValueError('Mostre o braço direito para confirmar o operador após a perda')
                associated=[]
                for score,candidate in candidates:
                    h=result.hand_landmarks[candidate];w=result.hand_world_landmarks[candidate];ids=(0,5,9,13,17)
                    measurements=[robust_depth(depth,int(h[i].x*color.width),int(h[i].y*color.height),3,(.3,3.)) for i in ids]
                    z,_=fused_wrist_depth(measurements,[w[i].z for i in ids])
                    if z is None:continue
                    u,v=h[0].x*color.width,h[0].y*color.height
                    xyz=np.array([(u-self.k[0,2])*z/self.k[0,0],(v-self.k[1,2])*z/self.k[1,1],z])
                    error=np.linalg.norm(xyz-reference)
                    if error<min(.4,.28+.7*(ns*1e-9-ref_stamp)):associated.append((error,candidate))
                if not associated:raise ValueError('Mão direita não corresponde ao braço calibrado')
                if len(associated)>1 and abs(associated[0][0]-associated[1][0])<.08:
                    raise ValueError('Duas mãos próximas; mostre apenas a mão do operador')
                _,index=min(associated)
            else:_,index=max(candidates)
            hand=result.hand_landmarks[index];world=result.hand_world_landmarks[index]
            pixels=np.array([[v.x*color.width,v.y*color.height] for v in hand])
            self.preview_hand=(ns,pixels.copy())
            categories=result.gestures[index]
            if categories:
                diag.update(raw_gesture=categories[0].category_name,raw_gesture_score=round(float(categories[0].score),3))
            if self.arm is not None and self.arm.scale is None:
                raise ValueError(f'Calibrando braço {len(self.arm.samples)}/{self.arm.minimum}: mostre ombro, cotovelo e punho direitos')
            if not (2<=pixels[0,0]<color.width-2 and 2<=pixels[0,1]<color.height-2):raise ValueError('Punho fora do quadro')
            for a,b in mp.solutions.hands.HAND_CONNECTIONS:
                cv2.line(image,tuple(pixels[a].astype(int)),tuple(pixels[b].astype(int)),(40,220,200),2)
            for u,v in pixels:cv2.circle(image,(int(u),int(v)),3,(20,80,255),-1)
            ids=(0,5,9,13,17)
            measured=[robust_depth(depth,int(pixels[i,0]),int(pixels[i,1]),3,(.3,3.)) for i in ids]
            distance,support=fused_wrist_depth(measured,[world[i].z for i in ids])
            if distance is None:raise ValueError('Sem profundidade consistente na palma/punho')
            u,v=pixels[0];origin=np.array([(u-self.k[0,2])*distance/self.k[0,0],(v-self.k[1,2])*distance/self.k[1,1],distance])
            world_points=np.array([[v.x,v.y,v.z] for v in world])
            points=origin+world_points[list(ids)]-world_points[0]
            self.hand_identity=(origin.copy(),ns*1e-9)  # measured identity survives a rejected orientation
            rotation=palm_rotation(points)
            self.preview_axes=(ns,origin.copy(),rotation.copy())
            q=matrix_quaternion(CAMERA_TO_ROBOT@rotation@HAND_TO_FR3_TCP)
            categories=result.gestures[index]
            if categories and categories[0].category_name in ('Open_Palm','Closed_Fist') and categories[0].score>=.75:
                gesture=categories[0].category_name;gesture_score=float(categories[0].score)
            signals=finger_distance_signals(world_points)
            if signals is not None:
                diag.update(grip=round(signals['grip'],3),pinch=round(signals['pinch'],3))
                geometric=signals['gesture']
                if geometric is not None:
                    if gesture is None or gesture==geometric:gesture,gesture_score=geometric,1.
                    else:gesture,gesture_score=None,0.  # Conflicting strong evidence holds the gripper.
            elif fingers_open(world_points):gesture='Open_Palm';gesture_score=1.
            q=self.clutch.update(q,gesture,ns*1e-9,anchor=self.orientation.q)
            if self.clutch.held is not None and self.orientation.q is not None:q=self.orientation.hold(ns*1e-9)
            else:q=self.orientation.update(q,ns*1e-9)
            p=self.position.update(CAMERA_TO_ROBOT@origin*(self.arm.scale if self.arm is not None else 1.315),ns*1e-9)
            if (time.time_ns()-ns)*1e-9>self.max_output_age:raise ValueError('Processamento atrasado; pose descartada')
            packet=dict(version=2,orientation_mode='full_6d',frame='fr3_link0',stamp_ns=ns,orientation_stamp_ns=ns,
                p=p.tolist(),q=q.tolist(),gesture=gesture,gesture_score=gesture_score)
            self.sample_pub.publish(String(data=json.dumps(packet,allow_nan=False)))
            self.hand_identity=(origin.copy(),ns*1e-9)
            camera_q=matrix_quaternion(rotation)
            measured=PoseStamped();measured.header=color.header
            measured.pose.position.x,measured.pose.position.y,measured.pose.position.z=map(float,origin)
            measured.pose.orientation.x,measured.pose.orientation.y,measured.pose.orientation.z,measured.pose.orientation.w=map(float,camera_q)
            self.camera_pose_pub.publish(measured)
            transform=TransformStamped();transform.header=color.header;transform.child_frame_id='fr3_operator_right_wrist'
            transform.transform.translation.x,transform.transform.translation.y,transform.transform.translation.z=map(float,origin)
            transform.transform.rotation=measured.pose.orientation;self.tf_broadcaster.sendTransform(transform)
            diag.update(valid=True,reason='Mão direita 6D rastreada',gesture=gesture,depth_m=round(distance,3),depth_support=support)
            # Draw measured palm axes in the optical image; orientation is independent of finger curl.
            start=tuple(pixels[0].astype(int))
            for axis,color_axis in enumerate(((0,0,255),(0,255,0),(255,0,0))):
                point=origin+.08*rotation[:,axis]
                end=(int(self.k[0,0]*point[0]/point[2]+self.k[0,2]),int(self.k[1,1]*point[1]/point[2]+self.k[1,2]))
                cv2.arrowedLine(image,start,end,color_axis,2)
        except (ValueError,TypeError,IndexError,cv2.error,RuntimeError) as error:
            diag['reason']=str(error)
        finally:
            now=time.monotonic();self.times=[v for v in self.times if now-v<2.]+[now]
            diag['processing_ms']=round((time.perf_counter()-started)*1000,1)
            diag['capture_age_ms']=round((time.time_ns()-ns)*1e-6,1)
            diag['fps']=round((len(self.times)-1)/(self.times[-1]-self.times[0]),1) if len(self.times)>1 else 0.
            if self.arm is not None and self.arm.scale is None:
                diag['reason']=f'Calibrando braço {len(self.arm.samples)}/{self.arm.minimum}: mostre ombro, cotovelo e punho direitos'
            self.preview_diag=diag
            self.diag_pub.publish(String(data=json.dumps(diag,allow_nan=False)))
            if image is not None and not self.smooth_preview:
                label=('6D DIREITA' if diag['valid'] else diag['reason'])
                if self.arm is not None and self.arm.scale is not None:
                    cv2.putText(image,f'Braco {self.arm.length:.2f} m | escala {self.arm.scale:.2f}',(8,86),cv2.FONT_HERSHEY_SIMPLEX,.45,(255,180,20),1)
                cv2.putText(image,label,(8,22),cv2.FONT_HERSHEY_SIMPLEX,.48,(40,220,40) if diag['valid'] else (40,60,255),1)
                cv2.putText(image,f'{gesture or "sem gesto"} | {diag["fps"]:.1f} Hz | {diag["processing_ms"]:.1f} ms',(8,43),cv2.FONT_HERSHEY_SIMPLEX,.45,(255,255,255),1)
                if distance is not None:cv2.putText(image,f'ZED {distance:.2f} m | pontos {support}/5',(8,65),cv2.FONT_HERSHEY_SIMPLEX,.45,(255,255,255),1)
                out=Image();out.header=color.header;out.height,out.width=image.shape[:2];out.step=out.width*3;out.encoding='bgr8';out.data=array('B',image.tobytes());self.overlay_pub.publish(out)
            self.frames+=1
            if self.frames%50==0:print(json.dumps(diag),flush=True)

    def destroy_node(self):
        if self.smooth_preview:
            with self.condition:self.stopping=True;self.condition.notify()
            self.worker.join(timeout=3.)
        for server in self.preview_servers:server.shutdown();server.server_close()
        self.detector.close()
        if self.body is not None:self.body.close()
        if self.calibration_socket is not None:self.calibration_socket.close()
        return super().destroy_node()

def main():
    ap=argparse.ArgumentParser()
    ap.add_argument('--domain',type=int,default=188)
    ap.add_argument('--calibrate-arm',action='store_true')
    ap.add_argument('--smooth-preview',action='store_true')
    ap.add_argument('--preview-port',type=int,default=0)
    ap.add_argument('--calibration-port',type=int,default=0)
    ap.add_argument('--output-prefix',default='/fr3_sim/operator')
    ap.add_argument('--max-input-age',type=float,default=.55)
    ap.add_argument('--max-output-age',type=float,default=.65)
    args,ros_args=ap.parse_known_args()
    rclpy.init(args=ros_args,domain_id=args.domain)
    node=OperatorHandTracker(domain=args.domain,output_prefix=args.output_prefix,max_input_age=args.max_input_age,max_output_age=args.max_output_age,calibrate_arm=args.calibrate_arm,calibration_port=args.calibration_port,smooth_preview=args.smooth_preview,preview_port=args.preview_port)
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()

if __name__=='__main__':main()
