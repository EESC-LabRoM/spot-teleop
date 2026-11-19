"""Finite operator-packet replay through the existing guarded physical cuRobo path.

Default is read-only preflight. --execute sends rebased operator observations,
never recorded joints, on the dedicated bag input port. Start uses the existing
dashboard's controller handover policy. A Stop, stale status, planner failure,
or travel bound ends the run. Nothing changes collision/freshness/joint guards.
"""
import argparse,collections,fcntl,json,math,socket,subprocess,sys,threading,time
from pathlib import Path
from urllib.request import Request,urlopen
from urllib.error import HTTPError
import numpy as np

REPO=Path('/home/hipolito/spot-teleop')
STAGES={'00_neutro':'neutral','01_giro_plano_imagem':'roll',
 '02_inclinar_dedos':'pitch','03_virar_palma':'yaw','04_frente_tras':'height',
 '05_direita_esquerda':'horizontal','05_direita_esquerda_2':'horizontal',
 '06_cima_baixo':'reach','07_abrir_fechar':'gripper','08_cruz':'cross',
 '09_gesto_em_movimento':'translation','10_perda_retorno':'translation',
 '11_distancias':'translation'}
def get(url):
 with urlopen(url,timeout=2) as r:return json.load(r)
def post(path,data):
 req=Request('http://127.0.0.1:8088'+path,data=json.dumps(data).encode(),
  headers={'Content-Type':'application/json','X-Dashboard':'1'})
 try:
  with urlopen(req,timeout=5) as r:result=json.load(r)
 except HTTPError as e:raise RuntimeError('Dashboard refused: '+e.read().decode()) from e
 if not result.get('ok'):raise RuntimeError(result)
 return result
class Observer:
 def __init__(self):
  self.state=None;self.at=0.;self.error=None;self.stop=False
  self.worker=threading.Thread(target=self.run,daemon=True);self.worker.start()
 def run(self):
  try:
   with urlopen('http://127.0.0.1:8088/stream',timeout=3) as f:
    for line in f:
     if self.stop:return
     if line.startswith(b'data:'):
      self.state=json.loads(line[5:]);self.at=time.monotonic()
  except Exception as e:self.error=repr(e)
 def current(self):
  if self.error or self.state is None or time.monotonic()-self.at>.35:raise RuntimeError('Robot dashboard state absent/stale: '+str(self.error))
  if not 0<=time.time()-self.state['t']<.4:raise RuntimeError('Robot feedback stale')
  return self.state
def packets(path):
 out=[]
 for line in path.read_text().splitlines():
  x=json.loads(line)
  if x.get('topic')=='/fr3_operator/teleop_sample':
   p=np.asarray(x.get('p'),float)
   if p.shape!=(3,) or not np.all(np.isfinite(p)):raise ValueError('Invalid recorded wrist')
   if x.get('position_valid') is not True:continue
   if x.get('orientation_valid'):
    q=np.asarray(x['q'],float)
    if q.shape!=(4,) or not np.all(np.isfinite(q)) or abs(np.linalg.norm(q)-1)>.01:raise ValueError('Invalid recorded quaternion')
    if x['orientation_stamp_ns']!=x['stamp_ns']:raise ValueError('Unpaired recording')
   out.append(x)
 if len(out)<3:raise ValueError('Too few observations')
 if any(y['stamp_ns']<=x['stamp_ns'] for x,y in zip(out,out[1:])):raise ValueError('Repeated/out-of-order recording')
 return out
def rebase(x,first,epoch):
 x=dict(x);x.pop('topic',None)
 for key in ('stamp_ns','orientation_stamp_ns','roll_stamp_ns'):
  if x.get(key) is not None:x[key]=epoch+x[key]-first
 return x
def preflight(observer,profile):
 s=observer.current();z=s.get('zed') or {}
 if s['engaged'] or z.get('home_running'):raise RuntimeError('Stop teleop/Home first')
 if z.get('control_backend')!='curobo' or z.get('exercise_profile')!=profile:raise RuntimeError('Wrong backend or exercise; configure paused bag follower')
 # Current joint feedback, native feasibility, map freshness and exact ownership
 # are all checked by the existing physical bridge, rather than inferred here.
 ready=0;rows=[]
 deadline=time.monotonic()+8
 while time.monotonic()<deadline:
  b=get('http://127.0.0.1:8784/status');rows.append(b)
  if b.get('armed') or b.get('hardware_publisher_count')!=1:raise RuntimeError('Unexpected hardware owner/armed state')
  ready=ready+1 if b.get('command_ready') else 0
  if ready>=10:return rows
  time.sleep(.1)
 raise RuntimeError('No sustained fresh feasible MPC output: '+str(collections.Counter(r.get('packet_reason') for r in rows)))
def main():
 p=argparse.ArgumentParser(description=__doc__);p.add_argument('--take',choices=STAGES,required=True)
 p.add_argument('--duration',type=float,default=8.);p.add_argument('--max-travel',type=float,default=.04)
 p.add_argument('--max-angle-deg',type=float,default=10.)
 p.add_argument('--port',type=int,default=47816);p.add_argument('--execute',action='store_true')
 p.add_argument('--output',type=Path,required=True);a=p.parse_args()
 if not 0<a.duration<=60 or not 0<a.max_travel<=.45 or not 0<a.max_angle_deg<=90:raise ValueError('Finite duration/travel/angle required')
 # A dedicated input prevents the live hand relay from interleaving packets.
 if a.port!=47816:raise ValueError('Physical bag follower must use dedicated port47816')
 lock=open('/tmp/fr3_physical_bag_replay.lock','w');fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
 source=REPO/'clips_out/zed_protocol_analysis/reprocessed_v1'/a.take/'samples.jsonl'
 events=packets(source);observer=Observer()
 deadline=time.monotonic()+3
 while observer.state is None and time.monotonic()<deadline:time.sleep(.05)
 profile=STAGES[a.take]
 if a.execute:
  s=observer.current()
  if s['engaged'] or not s.get('zed') or s['zed'].get('home_running') or s['zed'].get('control_backend')!='curobo':raise RuntimeError('Paused cuRobo follower required')
  post('/zed-profile',dict(profile=profile))
  # Reset only the private planner state from current measured joints; this
  # does not enable hardware or change controllers.
  seeded=subprocess.run(['docker','exec','fr3-zed-relay','bash','-c','source /opt/ros/jazzy/setup.bash; timeout 8 ros2 service call /fr3_joystick_mpc/set_active std_srvs/srv/SetBool "{data: true}"'],capture_output=True,text=True,timeout=12)
  if seeded.returncode or 'success=True' not in seeded.stdout:raise RuntimeError('Planner reseed failed: '+seeded.stdout+seeded.stderr)
  deadline=time.monotonic()+2
  while (observer.current().get('zed') or {}).get('exercise_profile')!=profile:
   if time.monotonic()>deadline:raise RuntimeError('Exercise selection not acknowledged')
   time.sleep(.02)
 checks=preflight(observer,profile)
 summary=dict(take=a.take,profile=profile,source=str(source),condition='corrected offline RGB-D operator packets, not new live inference',
  execute=a.execute,duration_limit_s=a.duration,travel_limit_m=a.max_travel,angle_limit_deg=a.max_angle_deg,preflight=checks,physical_commands=False)
 if not a.execute:print(json.dumps(summary,indent=2));observer.stop=True;return
 a.output.mkdir(parents=True,exist_ok=False);journal=(a.output/'trace.jsonl').open('x')
 start=observer.current();p0=np.asarray(start['tcp'][:3]);q0=np.asarray(start['tcp'][3:]);first=events[0]['stamp_ns']
 was_active=False;began=False;sent=0;max_distance=0.;rows=[];planner_bad_at=None
 try:
  # Recheck after all preparation; this is the sole action that prepares/arms
  # the guarded JTC. It initially holds measured joints while calibrating.
  preflight(observer,profile)
  began=True
  requested_at=time.time()
  print(post('/zed-control',dict(action='start')),flush=True)
  # The service reply precedes the next 5 Hz status publication. Wait for
  # acknowledgment of THIS Start rather than treating the old idle frame as Stop.
  acknowledged=False
  deadline=time.monotonic()+1.5
  while time.monotonic()<deadline:
   current=observer.current()
   if current['t']>=requested_at and (current.get('zed') or {}).get('phase')=='calibrating':
    acknowledged=True;break
   time.sleep(.02)
  if not acknowledged:raise RuntimeError('Start calibration acknowledgment missing')
  epoch=time.time_ns()+150_000_000
  with socket.socket(socket.AF_INET,socket.SOCK_DGRAM) as sock:
   for event in events:
    offset=(event['stamp_ns']-first)*1e-9
    if offset>a.duration:break
    due=epoch/1e9+offset
    while time.time()<due:
     time.sleep(min(.02,max(0.,due-time.time())))
    if time.time()-due>.12:raise RuntimeError('Replay scheduler delayed')
    s=observer.current();z=s.get('zed') or {};phase=z.get('phase')
    if phase=='active':was_active=True
    if was_active and (not s['engaged'] or phase=='idle'):raise RuntimeError('Teleop stopped: '+str(z.get('note')))
    if not was_active and phase!='calibrating':raise RuntimeError('Calibration interrupted: '+str(z.get('note')))
    measured=np.asarray(s['tcp'][:3]);distance=float(np.linalg.norm(measured-p0));max_distance=max(max_distance,distance)
    angle=math.degrees(2*math.acos(float(np.clip(abs(np.dot(q0,s['tcp'][3:])),0,1))))
    if angle>=a.max_angle_deg:summary['stop']='pilot angle reached';break
    if distance>=a.max_travel:summary['stop']='pilot travel reached';break
    b=get('http://127.0.0.1:8784/status')
    if b.get('hardware_publisher_count')!=1 or not b.get('armed') or not b.get('controller_ready'):raise RuntimeError('Hardware ownership/readiness lost: '+str(b))
    if not b.get('command_ready'):
     if planner_bad_at is None:planner_bad_at=time.monotonic()
     if time.monotonic()-planner_bad_at>.25:raise RuntimeError('Planner invalid: '+str(b.get('packet_reason')))
    else:planner_bad_at=None
    packet=rebase(event,first,epoch)
    sock.sendto(json.dumps(packet,allow_nan=False).encode(),('127.0.0.1',a.port));sent+=1
    row=dict(t=time.time(),source_offset_s=offset,tcp=s['tcp'],cmd=s.get('cmd'),phase=phase,note=z.get('note'),bridge=b)
    rows.append(row);journal.write(json.dumps(row)+'\n');journal.flush()
   else:summary['stop']='recording ended'
  if 'stop' not in summary:summary['stop']='duration limit reached'
  if not was_active:raise RuntimeError('No physical engagement; not a movement test')
  summary['physical_commands']=any(r['bridge'].get('sent',0)>checks[-1].get('sent',0) for r in rows)
 except Exception as e:summary['error']=repr(e);raise
 finally:
  # Count accepted hardware commands even when a guarded stop raises above.
  summary['physical_commands']=any(r['bridge'].get('sent',0)>checks[-1].get('sent',0) for r in rows)
  if began:
   try:post('/zed-control',dict(action='disengage'))
   except Exception as e:summary['stop_error']=repr(e)
   cleanup=subprocess.run(['docker','exec','fr3-ros2','bash','-c','source /opt/ros/jazzy/setup.bash; source /ros2_ws/install/setup.bash; export FASTRTPS_DEFAULT_PROFILES_FILE=/lab/fr3_zed_runtime/live_transport.xml; python3 /lab/fr3_zed_runtime/replay_controller_standby.py'],capture_output=True,text=True,timeout=15)
   summary['standby_restore']=dict(returncode=cleanup.returncode,stdout=cleanup.stdout,stderr=cleanup.stderr)
  summary.update(packet_count=sent,was_active=was_active,max_measured_travel_m=max_distance,end_time=time.time())
  journal.close();(a.output/'summary.json').write_text(json.dumps(summary,indent=2));observer.stop=True
  print(json.dumps({k:v for k,v in summary.items() if k!='preflight'},indent=2),flush=True)
if __name__=='__main__':main()
