"""Summarize offline RGB-D inference without claiming robot/contact validation."""
from pathlib import Path
import json,math
import numpy as np
root=Path(__file__).resolve().parent

def rows(path):
 text=path.read_text()
 # Earlier reference harnesses wrote literal newline separators.
 if text.count(chr(10))<2:text=text.replace('}\\n{','}\n{').removesuffix('\\n')
 return [json.loads(x) for x in text.splitlines() if x.strip()]

def stats(a):
 return {k:float(np.percentile(a,p)) for k,p in [('p50',50),('p95',95),('max',100)]} if len(a) else None

def summarize(data):
 valid=[r for r in data if r.get('sample') and r['sample'].get('position_valid')]
 orient=[r for r in valid if r['sample'].get('orientation_valid')]
 result=dict(frames=len(data),position_valid=len(valid),orientation_valid=len(orient),processing_ms=stats([r['processing_ms'] for r in data]),capture_age_ms=stats([r['diagnostics']['capture_age_ms'] for r in data if r.get('diagnostics') and 'capture_age_ms' in r['diagnostics']]),recorded_input_age_ms=stats([r['recorded_input_age_ms'] for r in data]))
 if valid:
  p=np.array([r['sample']['p'] for r in valid]);t=np.array([r['offset_s'] for r in valid]);dt=np.diff(t);d=np.linalg.norm(np.diff(p,axis=0),axis=1);continuous=(dt>0)&(dt<.1)
  gains=np.array([r['sample'].get('position_gain',(r.get('diagnostics') or {}).get('position_gain',(r.get('arm') or {}).get('arm_scale') or 1.315)) for r in valid]);physical=p/gains[:,None];steps=np.linalg.norm(np.diff(physical,axis=0),axis=1)
  result.update(hand_equivalent_range_m=np.ptp(physical,axis=0).tolist(),hand_equivalent_step_m=stats(steps[continuous]),hand_equivalent_steps_over_5cm=int(((steps>.05)&continuous).sum()))
  result.update(position_range_m=np.ptp(p,axis=0).tolist(),steps_over_5cm=int(((d>.05)&continuous).sum()),continuous_steps=int(continuous.sum()),step_m=stats(d[continuous]),max_position_gap_s=float(max(np.diff(t),default=0)))
 if orient:
  q=np.array([r['sample']['q'] for r in orient]);dt=np.diff([r['offset_s'] for r in orient]);ang=np.degrees(2*np.arccos(np.clip(np.abs(np.sum(q[1:]*q[:-1],axis=1)),0,1)))
  result['orientation_step_deg']=stats(ang[(dt>0)&(dt<.1)])
 return result

def main():
 out={}
 for mode in ['rgbd_fixed','rgbd_body','rgbd_body_profile','rgbd_body_safe']:
  path=root/'results'/mode/'trace.jsonl'
  if not path.exists():continue
  data=rows(path);summary=summarize(data);summary['by_take']={take:summarize([r for r in data if r['take']==take]) for take in sorted({r['take'] for r in data})}
  arm=[r for r in data if r.get('arm') and r['arm']['arm_calibrated']]
  if arm:
   ready=[r for r in arm if r['body_reference']['body_reference_ready']]
   summary.update(calibration_ready_s=ready[0]['offset_s'] if ready else None,arm_length_m=arm[-1]['arm']['arm_length_m'],arm_scale=arm[-1]['arm']['arm_scale'],arm_upper_m=arm[-1]['arm'].get('arm_upper_m'),arm_forearm_m=arm[-1]['arm'].get('arm_forearm_m'),scale_range=float(np.ptp([r['arm']['arm_scale'] for r in arm])),monitor_length_m=stats([r['arm']['arm_check_length_m'] for r in arm if r['arm']['arm_check_length_m'] is not None]))
   axes=[np.array(r['body_reference']['body_axes']) for r in ready if r['body_reference'].get('body_axes') is not None]
   if axes:summary['max_body_axes_change_deg']=float(max(np.degrees(np.arccos(np.clip((np.trace(a@axes[0].T)-1)/2,-1,1))) for a in axes))
  if mode in ('rgbd_body_profile','rgbd_body_safe'):
   held=[r for r in data if r['take']!='04'];summary['heldout']=summarize(held)
  out[mode]=summary
 (root/'comparison.json').write_text(json.dumps(out,indent=2)+'\n')
 print(json.dumps({m:{k:v for k,v in s.items() if k!='by_take'} for m,s in out.items()},indent=2))

if __name__=='__main__':main()
