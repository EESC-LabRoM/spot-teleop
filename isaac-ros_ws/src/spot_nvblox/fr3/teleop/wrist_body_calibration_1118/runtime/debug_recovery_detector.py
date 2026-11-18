"""Current-image version of the wrist-debug rotated-crop experiment.
Association uses the live detector/current wrist, never cached bag references.
"""
from types import SimpleNamespace as NS
from pathlib import Path
import os
import cv2,numpy as np,mediapipe as mp
from solution_hand_detector import SolutionHandDetector
from palm_orientation import palm_frame,angular_distance
from rgbd_pose import measure_wrist
from hand_tracking_math import finger_distance_signals
class DebugRecoveryDetector:
    def __init__(self):
        self.full=SolutionHandDetector(engine=mp.solutions.hands.Hands(static_image_mode=False,max_num_hands=1,
            model_complexity=1,min_detection_confidence=.6,min_tracking_confidence=.8))
        options=mp.tasks.vision.HandLandmarkerOptions(base_options=mp.tasks.BaseOptions(model_asset_path=os.environ.get('HAND_LANDMARKER_MODEL','/deploy/hand_landmarker.task')),
            running_mode=mp.tasks.vision.RunningMode.IMAGE,num_hands=2,
            min_hand_detection_confidence=.5,min_hand_presence_confidence=.5,min_tracking_confidence=.5)
        self.tasks=mp.tasks.vision.HandLandmarker.create_from_options(options)
        self.last_route=None;self.attempts=0;self.ref=None;self.ref_ms=None;self.previous=None;self.context=None;self.route='waiting'
    @staticmethod
    def frame(world):
        try:return palm_frame(world)
        except (ValueError,TypeError):return None
    def recognize_for_video(self,rgb,timestamp_ms):
        self.attempts=0
        original=self.full.recognize_for_video(rgb,timestamp_ms);h,w=rgb.shape[:2];ref=None
        # A currently classified right hand supplies current wrist guidance.
        for i,landmarks in enumerate(original.hand_landmarks):
            if original.handedness[i][0].category_name!='Right':continue
            pix=np.array([[p.x*w,p.y*h] for p in landmarks]);world=np.array([[p.x,p.y,p.z] for p in original.hand_world_landmarks[i]])
            ref=pix[0]
            if self.frame(world) is not None:
                self.ref=ref;self.ref_ms=timestamp_ms;self.previous=self.frame(world);self.route='debug original palm'
                return original
            break
        if ref is None and self.ref is not None and 0<=timestamp_ms-self.ref_ms<=500:ref=self.ref.copy()
        if ref is None:self.route='no current operator reference';return original
        candidates=[]
        def infer(image,angle=0,roi=None):
            self.attempts+=1
            result=self.tasks.detect(mp.Image(image_format=mp.ImageFormat.SRGB,data=np.ascontiguousarray(image)))
            for landmarks,worlds in zip(result.hand_landmarks,result.hand_world_landmarks):
                pts=np.array([[p.x,p.y,p.z] for p in worlds]);pix=np.array([[p.x*image.shape[1],p.y*image.shape[0]] for p in landmarks])
                a=np.deg2rad(angle);R=np.array([[np.cos(a),-np.sin(a),0],[np.sin(a),np.cos(a),0],[0,0,1]])
                pts=pts@R.T
                if roi:
                    x,y,side=roi
                    if angle==90:pix=np.column_stack((side-1-pix[:,1],pix[:,0]))
                    elif angle==180:pix=side-1-pix
                    elif angle==270:pix=np.column_stack((pix[:,1],side-1-pix[:,0]))
                    pix+=np.array([x,y])
                rotation=self.frame(pts)
                if rotation is None or np.linalg.norm(pix[0]-ref)>40*w/640:continue
                candidates.append((pts,pix,rotation,'debug Tasks full' if roi is None else 'debug crop '+str(angle)))
        # Prefer the most recent successful current-frame view. Every candidate
        # still needs current RGB-D support and the same anatomical palm checks.
        views=[]
        if self.last_route is not None:views.append(self.last_route)
        views.append(None)
        views.extend((fraction,angle) for fraction in [.4,.6] for angle in [0,90,180,270])
        seen=set();selected=None
        for view in views:
            if view in seen:continue
            seen.add(view);candidates.clear()
            if view is None:infer(rgb)
            else:
                fraction,angle=view
                side=min(int(w*fraction),w,h);x=int(np.clip(ref[0]-side/2,0,w-side));y=int(np.clip(ref[1]-side/2,0,h-side))
                infer(np.rot90(rgb[y:y+side,x:x+side],angle//90).copy(),angle,(x,y,side))
            if self.previous is not None:candidates.sort(key=lambda c:angular_distance(self.previous,c[2]))
            for candidate in candidates:
                pts,pix,rotation,route=candidate
                if self.context is not None:
                    try:measure_wrist(pix,pts,*self.context)
                    except ValueError:continue
                # Ambiguous flips go through the existing temporal confirmation.
                if self.previous is not None and angular_distance(self.previous,rotation)>np.deg2rad(70):continue
                selected=candidate;self.last_route=view;break
            if selected is not None:break
        if selected is None:self.route='debug no supported current recovery';return original
        pts,pix,rotation,self.route=selected;self.ref=pix[0];self.ref_ms=timestamp_ms;self.previous=rotation
        hand=[NS(x=float(p[0]/w),y=float(p[1]/h),z=0.) for p in pix]
        world=[NS(x=float(p[0]),y=float(p[1]),z=float(p[2])) for p in pts]
        signals=finger_distance_signals(pts);gesture=signals.get('gesture') if signals else None
        return NS(hand_landmarks=[hand],hand_world_landmarks=[world],
            handedness=[[NS(category_name='Right',score=1.)]],
            gestures=[[NS(category_name=gesture,score=1.)]] if gesture else [[]])
    def close(self):
        self.full.close();self.tasks.close()
