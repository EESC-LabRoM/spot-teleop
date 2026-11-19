"""Offline four-box SAM2 preview. No ROS imports, topics, TF or map writes."""
import base64,io,json,sys,time
import cv2,numpy as np,torch
from ultralytics import SAM
request=json.load(sys.stdin);raw=base64.b64decode(request['jpeg_b64']);image=cv2.imdecode(np.frombuffer(raw,np.uint8),cv2.IMREAD_COLOR)
assert image is not None
h,w=image.shape[:2];targets=request['targets'];boxes=[]
for t in targets:
 b=np.array(t['box_norm'],float);assert b.shape==(4,) and np.isfinite(b).all() and (b>=0).all() and (b<=1).all() and b[0]<b[2] and b[1]<b[3]
 boxes.append((b*np.array([w,h,w,h])).tolist())
torch.set_num_threads(2);torch.cuda.set_device(0);torch.cuda.reset_peak_memory_stats();start=time.perf_counter();model=SAM('/tmp/trackpkg/sam2.1_t.pt');loaded=time.perf_counter();result=model.predict(source=image,bboxes=boxes,device=0,imgsz=1024,save=False,verbose=False);torch.cuda.synchronize();elapsed=time.perf_counter()-loaded
assert len(result)==1 and result[0].masks is not None
masks=(result[0].masks.data>0).to(torch.uint8).cpu().numpy();assert len(masks)==len(targets),(len(masks),len(targets))
out=[];overlay=image.copy()
for target,mask in zip(targets,masks):
 if mask.shape!=(h,w):mask=cv2.resize(mask,(w,h),interpolation=cv2.INTER_NEAREST)
 color=np.array(target['color_bgr'],np.uint8);sel=mask>0;overlay[sel]=(overlay[sel]*.45+color*.55).astype(np.uint8)
 contours,_=cv2.findContours(mask,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE);cv2.drawContours(overlay,contours,-1,tuple(int(x) for x in color),1)
 ys,xs=np.where(sel);pixels=int(sel.sum());x1,y1,x2,y2=(int(v) for v in (np.array(target['box_norm'])*np.array([w,h,w,h])))
 cv2.rectangle(overlay,(x1,y1),(x2,y2),tuple(int(x) for x in color),1)
 cv2.putText(overlay,target['name'],(x1,max(18,y1-5)),cv2.FONT_HERSHEY_SIMPLEX,.38,(0,0,0),3);cv2.putText(overlay,target['name'],(x1,max(18,y1-5)),cv2.FONT_HERSHEY_SIMPLEX,.38,tuple(int(x) for x in color),1)
 ok,encoded=cv2.imencode('.png',mask*255);assert ok
 eroded=cv2.erode(mask,np.ones((5,5),np.uint8));out.append(dict(id=target['id'],name=target['name'],pixels=pixels,coverage_pct=100*pixels/(h*w),survives_drill_erosion_pixels=int(eroded.sum()),mask_png_b64=base64.b64encode(encoded).decode(),status='preview_needs_visual_check' if 5<pixels<.2*h*w else 'suspicious_mask',box_norm=target['box_norm']))
ok,jpg=cv2.imencode('.jpg',overlay,[cv2.IMWRITE_JPEG_QUALITY,90]);assert ok
print('INSPECTOR_RESULT='+json.dumps(dict(scope='snapshot_preview_only_not_routed_to_map',targets=out,overlay_jpeg_b64=base64.b64encode(jpg).decode(),image_width=w,image_height=h,model_load_s=loaded-start,inference_s=elapsed,gpu_peak_mb=torch.cuda.max_memory_allocated()/1024**2)))
