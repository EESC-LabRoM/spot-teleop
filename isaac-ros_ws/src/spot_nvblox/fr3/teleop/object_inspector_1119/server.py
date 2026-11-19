"""Read-only mask/map inspector; default loopback, no ROS or control operations."""
import argparse,json,pathlib,subprocess,threading,time,urllib.request
from http.server import BaseHTTPRequestHandler,ThreadingHTTPServer
from urllib.parse import urlparse
ROOT=pathlib.Path(__file__).parent
state={'updated_at':0,'filter':None,'live':None,'errors':[]};lock=threading.Lock()
def audit(filt,live,now):
 r=(filt or {}).get('recent') or {};stamp=r.get('stamp');age=now-stamp if isinstance(stamp,(int,float)) else None
 heartbeat=now-filt.get('wall',0) if filt else None;coverage=(live or {}).get('coverage_pct');mask_age=(live or {}).get('mask_age_s')
 mask_ok=isinstance(coverage,(float,int)) and coverage>0 and isinstance(mask_age,(float,int)) and 0<=mask_age<1.5
 current=mask_ok and age is not None and 0<=age<2 and heartbeat is not None and 0<=heartbeat<5
 return dict(mask_available=mask_ok,map_input_current=current,accepted_frame_age_s=age,heartbeat_age_s=heartbeat,leak_check='Input exclusion verified' if current and r.get('masked_nonzero')==0 and r.get('outside_changed')==0 else 'NOT VERIFIED',masked_nonzero=r.get('masked_nonzero') if current else None,outside_changed=r.get('outside_changed') if current else None,pair_delta_ms=r.get('depth_mask_delta_ms') if current else None,last_refusal=r.get('refusal'))
def poll():
 while True:
  filt=live=None;errors=[]
  try:
   p=subprocess.run(['docker','exec','isaac-build','cat','/tmp/wrist_map_filter/status.json'],capture_output=True,text=True,timeout=3);p.check_returncode();filt=json.loads(p.stdout)
  except Exception as e:errors.append('Depth filter: '+str(e))
  try:
   with urllib.request.urlopen('http://10.0.0.3:8791/stats.json',timeout=2) as r:live=json.load(r)
  except Exception as e:errors.append('Live mask: '+str(e))
  with lock:state.update(updated_at=time.time(),filter=filt,live=live,errors=errors)
  time.sleep(2)
class Handler(BaseHTTPRequestHandler):
 def log_message(self,*args):pass
 def send(self,status,data,mime):
  self.send_response(status);self.send_header('Content-Type',mime);self.send_header('Cache-Control','no-store');self.send_header('Content-Length',str(len(data)));self.end_headers()
  try:self.wfile.write(data)
  except (BrokenPipeError,ConnectionResetError):pass
 def do_GET(self):
  path=urlparse(self.path).path
  if path in ['/','/index.html']:return self.send(200,(ROOT/'index.html').read_bytes(),'text/html; charset=utf-8')
  if path=='/api/status':
   with lock:d=json.loads(json.dumps(state))
   d['audit']=audit(d.get('filter'),d.get('live'),time.time())
   if time.time()-d.get('updated_at',0)>6:d['audit'].update(map_input_current=False,mask_available=False,leak_check='NOT VERIFIED')
   try:d['preview']=json.loads((ROOT/'preview.json').read_text())
   except (OSError,ValueError):d['preview']=None
   return self.send(200,json.dumps(d,allow_nan=False).encode(),'application/json')
  if path=='/live-mask.jpg':
   try:
    with urllib.request.urlopen('http://127.0.0.1:8788/segsnap',timeout=3) as r:data=r.read()
    return self.send(200,data,'image/jpeg')
   except Exception as e:return self.send(503,str(e).encode(),'text/plain')
  if path in ['/source.jpg','/segmented.jpg','/drill_mask.png','/tennis_ball_mask.png','/sprite_mask.png','/screwdriver_mask.png']:
   try:return self.send(200,(ROOT/path[1:]).read_bytes(),'image/png' if path.endswith('.png') else 'image/jpeg')
   except OSError:return self.send(503,b'Preview unavailable','text/plain')
  self.send(404,b'Not found','text/plain')
 def do_POST(self):self.send(405,b'Read-only inspector','text/plain')
if __name__=='__main__':
 p=argparse.ArgumentParser();p.add_argument('--bind',action='append');p.add_argument('--port',type=int,default=47814);p.add_argument('--ttl',type=float,default=1500);a=p.parse_args();hosts=a.bind or ['127.0.0.1'];assert set(hosts)<={'127.0.0.1','100.88.134.127'}
 threading.Thread(target=poll,daemon=True).start();servers=[ThreadingHTTPServer((h,a.port),Handler) for h in hosts]
 for s in servers:threading.Thread(target=s.serve_forever,daemon=True).start()
 print('OBJECT_INSPECTOR_READY',hosts,a.port,flush=True);end=time.monotonic()+a.ttl
 while time.monotonic()<end:time.sleep(max(0,min(1,end-time.monotonic())))
 for s in servers:s.shutdown();s.server_close()
