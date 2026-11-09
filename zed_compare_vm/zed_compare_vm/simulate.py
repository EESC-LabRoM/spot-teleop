"""Register timestamped derived recordings and render measured GPU simulation replays."""
import json,time,sys,subprocess,shutil,os
from pathlib import Path
from urllib.request import Request,urlopen
HERE=Path(__file__).resolve().parent;sys.path.insert(0,str(HERE.parent/'remote_sim'))
from cartesian_review import PROFILES,STAGES
ROOT=HERE.parents[4];OUT=ROOT/'clips_out/zed_protocol_analysis';DERIVED=OUT/'reprocessed_v1';BAGS=ROOT/'bags/zed_protocol_20261030_005244';REPORT=OUT/'simulation_review.json'
def get(url):
 with urlopen(url,timeout=15) as r:return json.load(r)
def post(port,path,data,header):
 req=Request(f'http://127.0.0.1:{port}{path}',data=json.dumps(data).encode(),headers={'Content-Type':'application/json',header:'1'})
 with urlopen(req,timeout=30) as r:return json.load(r)
def ctl(action,value=None):return post(8791,'/control',dict(action=action,value=value),'X-Sim-Control')
def save(path,data):
 tmp=path.with_suffix('.tmp');tmp.write_text(json.dumps(data,ensure_ascii=False,indent=2));tmp.replace(path)
def register(folder):
 subprocess.run(['docker','exec','fr3-realsense-capture','chown','-R',str(os.getuid())+':'+str(os.getgid()),'/home/spot-teleop/'+str(folder.relative_to(ROOT))],check=True)
 s=json.loads((folder/'summary.json').read_text());sid='zed_protocol_20261030_'+folder.name+'_reprocessed_v1';dest=ROOT/'bags'/sid/'replay'
 dest.parent.mkdir(exist_ok=True)
 if not (dest/'metadata.yaml').exists():shutil.copytree(folder/'replay',dest,copy_function=shutil.copy2,dirs_exist_ok=True)
 if not (folder/'tracking.mp4').exists():
  subprocess.run(['ffmpeg','-nostdin','-hide_banner','-loglevel','error','-f','concat','-safe','0','-i',str(folder/'frames/list.txt'),'-vsync','vfr','-c:v','libx264','-threads','2','-preset','fast','-crf','23','-pix_fmt','yuv420p','-movflags','+faststart',str(folder/'tracking.mp4')],check=True)
 library_path=ROOT/'clips_out/editor_media/library.json';lib=json.loads(library_path.read_text());result=json.loads((BAGS/folder.name/'result.json').read_text())
 item=dict(id=sid,label='ZED · '+result['title'],group='ZED · protocolo real reprocessado',path=str((folder/'tracking.mp4').relative_to(ROOT)),thumbnail=str((folder/'timeline.jpg').relative_to(ROOT)),duration=s['duration_s'],bag=str(dest.relative_to(ROOT)),bag_offset_s=0.,frames=s['paired_frames'],fps=s['paired_frames']/s['duration_s'],replay_only=True)
 if sid not in {i['id'] for i in lib}:lib.append(item);save(library_path,lib)
 get('http://127.0.0.1:8793/api/library')
 return item,s,result

def main():
 names=[p.name for p in sorted(BAGS.iterdir()) if (p/'d8').exists()]
 report=dict(revision=1,status='running',dataset='ZED RGBD completo reprocessado',mapping='per-exercise review profiles; explicit isolation, not a universal gesture classifier',runs=[dict(source_id='zed_protocol_20261030_'+name+'_reprocessed_v1',stage=name,label=name,status='pending',expected=PROFILES[STAGES[int(name[:2])]]['label']) for name in names])
 if REPORT.exists():
  previous=json.loads(REPORT.read_text());old={r['source_id']:r for r in previous['runs']}
  for i,row in enumerate(report['runs']):
   if old.get(row['source_id'],{}).get('status')=='done':report['runs'][i]=old[row['source_id']]
   force=next((arg.split('=',1)[1].split(',') for arg in sys.argv if arg.startswith('--rerun-stages=')),[])
   if row['stage'] in force:
    report['runs'][i]=dict(row,attempts=[old.get(row['source_id'],{})])
 save(REPORT,report)
 deadline=time.monotonic()+180
 while True:
  state=get('http://127.0.0.1:8791/state')
  if state.get('simulation_only') and state.get('domain')==188 and state.get('mpc_warmed') and state.get('sim_age_ms') is not None and state['sim_age_ms']<500:break
  if time.monotonic()>deadline:raise TimeoutError('GPU simulation not ready')
  time.sleep(1)
 # Prioritize concrete translations and gripper before remaining diagnostic clips.
 order=sorted(range(len(names)),key=lambda i:([5,4,6,7,1,2,3,0,8,9,10,11].index(int(names[i][:2]))))
 for i in order:
  row=report['runs'][i]
  if row['status']=='done':continue
  folder=DERIVED/names[i];deadline=time.monotonic()+1200
  while not (folder/'summary.json').exists():
   if time.monotonic()>deadline:raise TimeoutError('Reprocessamento não terminou: '+names[i])
   time.sleep(3)
  try:
   source,summary,result=register(folder);stage=int(names[i][:2]);profile=STAGES[stage];row.update(label=source['label'],profile=profile,tracking=summary['counts'])
   ctl('stop');ctl('mapping',dict(mode='cartesian',profile=profile,position_gains=[1.,1.,1.],rotation_gain=1.2 if profile in ('pitch','yaw') else 1.6,tilt_gain=0.));ctl('speed',1.)
   events=[json.loads(x) for x in (BAGS/names[i]/'events.jsonl').read_text().splitlines() if x.strip()];stop=min([e['t'] for e in events if e.get('event')=='sent' and e.get('cmd')=='stop']+[result['t1']]);t0=summary['capture_start_ns']*1e-9
   # Include pre-engage frames to let simulation capture a fresh reference.
   start=max(0,(result.get('engaged_at') or t0)-t0-2.5);end=min(source['duration'],stop-t0)
   if end-start<2:end=source['duration']
   clip=post(8793,'/api/clips',dict(source_id=source['id'],start_s=start,end_s=end,label='ZED RGBD · '+source['label']),'X-Clip-Editor')
   job=post(8793,'/api/simulate',dict(id=clip['id']),'X-Clip-Editor');row.update(clip_id=clip['id'],job_id=job['id'],status='running',start_s=start,end_s=end);save(REPORT,report);print('START',names[i],profile,flush=True)
   deadline=time.monotonic()+end-start+180
   while time.monotonic()<deadline:
    job=next(j for j in get('http://127.0.0.1:8793/api/library')['jobs'] if j['id']==row['job_id'])
    if job['status'] in ('done','error'):break
    time.sleep(2)
   else:raise TimeoutError('Simulation job')
   if job['status']=='error':raise RuntimeError(job.get('error'))
   metrics=get('http://127.0.0.1:8793'+job['manifest_url'])['clips'][0]
   row.update(status='done',url=job['url'],manifest_url=job['manifest_url'],metrics=metrics)
   if metrics.get('error'):raise RuntimeError(metrics['error'])
   trace=json.loads((ROOT/'clips_out/manual_clips'/job['id']/'simulation/clip_01/trace.json').read_text())
   import numpy as np
   active=next((p['wall_time'] for p in trace['state_polls'] if p['phase']=='active'),float('inf'))
   points=np.array([x['p'] for x in trace['tcp_log'] if x['wall_time']>=active+1])
   row['measured_peak_to_peak_cm']=(np.ptp(points,axis=0)*100).round(2).tolist() if len(points) else None
   print('DONE',names[i],row['measured_peak_to_peak_cm'],flush=True)
  except Exception as e:row.update(status='error',error=str(e));print('ERROR',names[i],str(e),flush=True)
  save(REPORT,report)
 ctl('stop');report['status']='complete' if all(r['status']=='done' for r in report['runs']) else 'complete_with_errors';save(REPORT,report)
if __name__=='__main__':main()
