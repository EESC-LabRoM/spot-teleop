import json,pathlib,numpy as np
root=pathlib.Path('/home/murilo/franka_ws/lab/wrist_bag_1118');output={}
for case in ['z06','z12']:
 p=root/case
 if not (p/'result.json').exists():continue
 rows=[json.loads(x) for x in (p/'trace.jsonl').read_text().splitlines()];cmd=None;errors=[];angles=[];poses=[];joints=[];inp=[];cmdrows=[];meas=[];fwd=0
 for d in rows:
  if d['kind']=='command':cmd=d;cmdrows.append(d)
  if d['kind']=='forwarded':fwd+=1
  if d['kind']=='input_3d_locked':inp.append(d)
  if d['kind']=='measured':
   poses.append(d['p']);angles.append(d['orientation_change_deg']);meas.append(d)
   if cmd is not None and 0<=d['wall']-cmd['wall']<.1:errors.append(float(np.linalg.norm(np.array(d['p'])-cmd['p'])))
  if d['kind']=='joints':joints.append(d['q'])
 text=(p/'consumer.log').read_text();r=json.loads((p/'result.json').read_text());r.update(forwarded_commands=fwd,following_resets=text.count('arm not following'),workspace_clamp_log_lines=text.count('workspace box clamps'),error_samples=len(errors),command_to_TCP_error_m={'p50':float(np.percentile(errors,50)) if errors else None,'p95':float(np.percentile(errors,95)) if errors else None,'max':max(errors,default=None)},TCP_span_m=np.ptp(poses,axis=0).tolist() if poses else [],orientation_change_max_deg=max(angles,default=None),joint_span_deg=np.degrees(np.ptp(joints,axis=0)).tolist() if joints else [],input_capture_age_ms={'p50':float(np.percentile([x['age_s']*1000 for x in inp],50)) if inp else None,'p95':float(np.percentile([x['age_s']*1000 for x in inp],95)) if inp else None})
 output[case]=r
print(json.dumps(output,indent=2));pathlib.Path('/home/hipolito/franka_teleop_checks/bag_hardware_1118/results.json').write_text(json.dumps(output,indent=2))
