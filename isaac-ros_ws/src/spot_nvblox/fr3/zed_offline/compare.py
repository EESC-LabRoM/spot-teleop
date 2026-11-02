"""Compare original telemetry and full RGB-D replay, then measure actual sim paths."""
import json,sys
from pathlib import Path
import numpy as np
HERE=Path(__file__).resolve().parent;ROOT=HERE.parents[4];OUT=ROOT/'clips_out/zed_protocol_analysis'
sys.path.insert(0,str(HERE.parent/'remote_sim'))
from cartesian_review import PROFILES

def build():
 original={s['stage']:s for s in json.loads((OUT/'analysis.json').read_text())['stages']};review=json.loads((OUT/'simulation_review.json').read_text());rows=[]
 for f in sorted((OUT/'reprocessed_v1').glob('*/summary.json')):
  s=json.loads(f.read_text());old=original[s['stage']];row=dict(stage=s['stage'],source_rgb=s['source_rgb'],paired_frames=s['paired_frames'],position_fraction=s['counts']['position_valid']/s['paired_frames'],orientation_fraction=s['counts']['orientation_valid']/s['paired_frames'],original_emitted_orientation_fraction=old['full_orientation_fraction'],original_hand_hz=old['hand_output_fps'],capture_pair_hz=s['paired_frames']/s['duration_s'],original_real_span_cm=old['measured_span_robot_xyz_cm'])
  run=next((r for r in review['runs'] if r['stage']==s['stage']),{})
  if run.get('status')=='done':
   trace=json.loads((ROOT/'clips_out/manual_clips'/run['job_id']/'simulation/clip_01/trace.json').read_text());states=[p for p in trace['state_polls'] if p['phase']=='active'];active=states[0]['wall_time'] if states else float('inf');tcp=[p for p in trace['tcp_log'] if p['wall_time']>=active+1];profile=run['profile'];mask=np.array(PROFILES[profile]['translation']);xyz=np.array([p['p'] for p in tcp]);qs=np.array([p['q'] for p in tcp]);row.update(profile=profile,job_id=run['job_id'],simulation_solver='GPU IK / wrist-only; GPU PhysX measured feedback',active_status_polls=len(states),paused_status_polls=sum(p['phase']=='paused' for p in trace['state_polls']),simulated_span_cm=(np.ptp(xyz,axis=0)*100).round(3).tolist() if len(xyz) else None)
   if len(xyz):
    span=np.ptp(xyz,axis=0)*100;row['off_axis_max_cm']=float(max(span[mask==0])) if np.any(mask==0) else None;row['isolated_translation_pass']=bool(row['off_axis_max_cm']<1.) if row['off_axis_max_cm'] is not None else None
    dot=np.clip(np.abs(qs@qs[0]),0,1);row['orientation_excursion_deg']=float(np.degrees(2*np.arccos(dot)).max())
   row['feasible_fraction']=run.get('metrics',{}).get('mpc_feasible_ratio')
   widths=[v['finger_width_m']*1000 for v in trace['state_polls'] if v.get('finger_width_m') is not None]
   if widths:row['gripper_measured_width_range_mm']=[round(min(widths),3),round(max(widths),3)]
   changes=[]
   for v in trace['state_polls']:
    if v.get('gripper') and (not changes or changes[-1]['command']!=v['gripper']):changes.append(dict(t=v['wall_time']-active,command=v['gripper']))
   if changes:row['gripper_transitions']=changes
   perf=[v['simulation_performance']['physics_hz'] for v in trace['state_polls'] if v.get('simulation_performance')]
   if perf:row['physics_hz_range']=[round(min(perf),1),round(max(perf),1)]
   # Signed correlation uses known approved axes; never pick a sign from the result.
   samples=trace.get('sample_log',[])
   if len(samples)>5 and len(tcp)>5 and profile in ('horizontal','height','reach'):
    hi,ri,sign={'horizontal':(1,1,1),'height':(0,2,-1),'reach':(2,0,1)}[profile]
    ht=np.array([v['wall_time'] for v in samples]);hp=np.array([v['p'][hi] for v in samples]);rt=np.array([v['wall_time'] for v in tcp]);actual=xyz[:,ri]
    expected=sign*np.interp(rt,ht,hp)
    if np.std(expected)>.001 and np.std(actual)>.001:row['signed_hand_robot_correlation']=round(float(np.corrcoef(expected,actual)[0,1]),3)
   # Gripper state is available in the recorder's robot series, not inferred from command.
   row['robot_series_example']=trace.get('robot_series',[])[:1]
  rows.append(row)
 result=dict(recordings=len(rows),source_rgb=sum(r['source_rgb'] for r in rows),paired_frames=sum(r['paired_frames'] for r in rows),position_valid=sum(json.loads(f.read_text())['counts']['position_valid'] for f in (OUT/'reprocessed_v1').glob('*/summary.json')),orientation_valid=sum(json.loads(f.read_text())['counts']['orientation_valid'] for f in (OUT/'reprocessed_v1').glob('*/summary.json')),rows=rows)
 (OUT/'comparison.json').write_text(json.dumps(result,ensure_ascii=False,indent=2));print(json.dumps({k:v for k,v in result.items() if k!='rows'}));print([(r['stage'],r.get('simulated_span_cm'),r.get('isolated_translation_pass')) for r in rows if 'profile' in r])
if __name__=='__main__':build()
