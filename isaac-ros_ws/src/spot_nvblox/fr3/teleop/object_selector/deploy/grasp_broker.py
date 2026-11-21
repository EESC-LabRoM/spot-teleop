#!/usr/bin/env python3
"""Host-only Unix-socket transport to the existing VM grasp planner/lock.

No ROS and no motion commands. Private SSH keys stay on the host. Planning
imports Murilo's canonical grasp_plan_lib; it never uses the viewer's rendered
pick/orientation as an executable pose.
"""
import argparse
import json
import os
from pathlib import Path
import socket
import socketserver
import re
import subprocess

VM_SCRIPT = r'''
import json, sys, time, fcntl
from pathlib import Path
import numpy as np
base=Path('/home/nexus/spot-teleop/fr3_real_repro')
sys.path.insert(0,str(base/'grasp_loop'))
import grasp_plan_lib as gp
request=json.loads(sys.stdin.readline())
lock=base/'regrasp/state/regrasp.lock'
owner=request['owner']
if request['action']=='lock':
    guard=base/'regrasp/state/joystick_lock.guard'
    seqfile=base/'regrasp/state/joystick_lock_sequence.json'
    with guard.open('a') as guardfile:
        fcntl.flock(guardfile,fcntl.LOCK_EX)
        record=json.loads(seqfile.read_text()) if seqfile.exists() else {}
        old=json.loads(lock.read_text()) if lock.exists() else {}
        seq=int(request['seq'])
        previous=int(record.get(owner,0))
        if seq>previous:
            if old and old.get('owner')!=owner:
                raise RuntimeError('another grasp owns regrasp.lock; release it through its owner')
            state=request.get('state')
            if state is None:
                if old: lock.unlink()
            else:
                tmp=lock.with_suffix('.joystick.tmp')
                # PID1 is the existing persistent vm-viewer supervisor; the
                # five-second host heartbeat supplies RUN_ACTIVE liveness.
                tmp.write_text(json.dumps(dict(state=state,owner=owner,pid=1,ts=time.time(),run_dir='joystick')))
                tmp.replace(lock)
            record[owner]=seq
            seqfile.write_text(json.dumps(record))
    print(json.dumps(dict(ok=True)))
else:
    if lock.exists(): raise RuntimeError('regrasp.lock is occupied; finish/release the previous grasp first')
    # 2026-11-21 object selector: non-drill targets plan from the live cloud of the
    # selected object, only when the selector reports it ready with Y enabled and the
    # tracker/map gate are on that same object. Drill keeps the binding path below.
    import urllib.request
    try: sel=json.loads(urllib.request.urlopen('http://127.0.0.1:8898/object-target',timeout=5).read())
    except Exception as exc: raise RuntimeError('object selector unavailable: '+str(exc))
    tgt=sel.get('selected')
    if tgt!='drill':
        trk=sel.get('tracking') or {}
        if not (sel.get('phase')=='ready' and sel.get('y_enabled') is True and not sel.get('busy')):
            raise RuntimeError(f'{tgt} selected; Y locked: '+str((sel.get('grasp') or {}).get('detail') or sel.get('reason')))
        if trk.get('map_state')!='ACTIVE:'+str(tgt) or trk.get('tracking')!='TRACKING' or not isinstance(trk.get('mask_age_s'),(int,float)) or trk['mask_age_s']>1.5:
            raise RuntimeError(f'{tgt}: tracker/map not on the selected object ({trk.get("map_state")}, {trk.get("tracking")})')
        sys.path.insert(0,str(base/'object_selector'))
        import object_grasps
        if tgt not in object_grasps.PLANNERS: raise RuntimeError(f'no grasp planner for {tgt}')
        live=gp.load_grasp('http://127.0.0.1:8791/grasp.json')
        age=float((live.get('live') or {}).get('cloud_age_s',999))
        if not 0<=age<=2: raise RuntimeError('live object cloud absent/stale')
        cloud=np.asarray(live['cloud'],float)
        mesh=gp.load_mesh('http://127.0.0.1:18089/mesh.bin')
        if len(mesh)<100: raise RuntimeError('nvblox mesh absent/empty')
        start=request.get('start_q') or gp.Q_HOME
        plan=object_grasps.PLANNERS[tgt](gp,cloud,mesh,start)
        if plan['problems']: raise RuntimeError('plan rejected: '+str(plan['problems']))
        print(json.dumps(dict(ok=True,plan=plan,run=plan['run'],cloud_age_s=age,drift_m=0.0,
                             planned_at=time.time(),binding_age_s=0.0)))
        sys.exit(0)
    binding=json.loads((base/'regrasp/state/grasp_binding.json').read_text())
    run=binding['run_id']
    if Path(run).name!=run: raise RuntimeError('invalid binding run id')
    source=base/'grasp_loop/runs'/run/'grasp_plan_input.json'
    g=gp.load_grasp(source.as_uri())
    live=gp.load_grasp('http://127.0.0.1:8791/grasp.json')
    meta=live.get('live') or {}
    age=float(meta.get('cloud_age_s',999))
    if not 0<=age<=2: raise RuntimeError('live object cloud absent/stale')
    cloud=np.asarray(live['cloud'],float)
    original=np.asarray(g['cloud'],float)
    if cloud.ndim!=2 or cloud.shape[1]!=3 or len(cloud)<300 or not np.isfinite(cloud).all():
        raise RuntimeError('invalid live object cloud')
    drift=float(np.linalg.norm(np.median(cloud[:,:2],0)-np.median(original[:,:2],0)))
    dz=abs(float(np.percentile(cloud[:,2],95)-np.percentile(original[:,2],95)))
    if drift>.03 or dz>.03:
        raise RuntimeError('object changed since selected grasp (xy %.3fm, top %.3fm); regenerate with existing regrasp loop'% (drift,dz))
    # Recheck the canonical selected pose against CURRENT measured points/map.
    # Rendering shifts/yaw on /grasp.json are deliberately never copied.
    g['cloud']=cloud.tolist()
    mesh=gp.load_mesh('http://127.0.0.1:18089/mesh.bin')
    if len(mesh)<100: raise RuntimeError('nvblox mesh absent/empty')
    plan=gp.plan_grasp(g,mesh,depth=float(binding.get('depth',.02)),lift=.10,
                       cart_step=.15,wp_min_time=6.,vmax=.10,start_q=request.get('start_q'))
    if plan['problems']: raise RuntimeError('plan rejected: '+str(plan['problems']))
    print(json.dumps(dict(ok=True,plan=plan,run=run,cloud_age_s=age,drift_m=drift,
                         planned_at=time.time(),binding_age_s=time.time()-binding['bound_at'])))
'''


def exchange(path, request, timeout=90):
    with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as sock:
        sock.settimeout(timeout)
        sock.connect(str(path))
        sock.sendall(json.dumps(request).encode()+b'\n')
        with sock.makefile('rb') as stream:
            line=stream.readline(2_000_000)
    result=json.loads(line)
    if not result.get('ok'): raise RuntimeError(result.get('error','broker request failed'))
    return result


def remote(request, key, known_hosts):
    command=['ssh','-T','-o','BatchMode=yes','-o','ConnectTimeout=8',
             '-o','HostKeyAlias=omnigent-vm','-o','StrictHostKeyChecking=yes',
             '-o','UserKnownHostsFile='+known_hosts,'-i',key,
             'nexus@100.84.207.115','docker exec -i vm-viewer python3 -c '+__import__('shlex').quote(VM_SCRIPT)]
    result=subprocess.run(command,input=json.dumps(request)+'\n',text=True,
                          capture_output=True,timeout=80)
    if result.returncode:
        raise RuntimeError((result.stderr.strip().splitlines() or ['VM planner failed'])[-1])
    return json.loads(result.stdout.strip().splitlines()[-1])


def main():
    ap=argparse.ArgumentParser()
    ap.add_argument('--socket',required=True)
    ap.add_argument('--key',default='/home/hipolito/.ssh/id_ed25519')
    ap.add_argument('--known-hosts',default='/home/hipolito/.ssh/known_hosts')
    args=ap.parse_args()
    path=Path(args.socket)
    if path.exists():
        try:
            with socket.socket(socket.AF_UNIX) as s: s.connect(str(path))
        except ConnectionRefusedError: path.unlink()
        else: raise SystemExit('broker already running')
    class Handler(socketserver.StreamRequestHandler):
        def handle(self):
            try:
                self.connection.settimeout(10)
                req=json.loads(self.rfile.readline(4096))
                if req.get('action') not in ('plan','lock') or not re.fullmatch('[0-9a-f]{32}',str(req.get('owner',''))):
                    raise ValueError('invalid request')
                if req['action']=='lock' and req.get('state') not in (None,'RUN_ACTIVE','HELD_OBJECT'):
                    raise ValueError('invalid lock state')
                sq=req.get('start_q')   # optional 7 measured joints: plan S1 from here (2026-11-16)
                if sq is not None and not (isinstance(sq,list) and len(sq)==7
                        and all(isinstance(v,(int,float)) and abs(v)<4 for v in sq)):
                    raise ValueError('invalid start_q')
                out=remote(req,args.key,args.known_hosts)
                plan=out.get('plan') if isinstance(out,dict) else None
                if out.get('ok') and isinstance(plan,dict) and plan.get('object') not in (None,'drill'):
                    run=str(plan['run'])
                    if Path(run).name!=run or not re.fullmatch('[0-9_a-z]+',run): raise ValueError('invalid plan run id')
                    held=plan.get('held_spheres_hand')
                    model=dict(run=run,frame='fr3_hand',spheres=held,margin_m=0.007,coverage=None,
                               source=f'object_selector {plan.get("object")} model')
                    tmp=path.parent/'attached_spheres'/(run+'.json.tmp')
                    tmp.write_text(json.dumps(model,indent=1));tmp.replace(tmp.with_suffix(''))
            except Exception as exc: out=dict(ok=False,error=str(exc))
            try: self.wfile.write(json.dumps(out).encode()+b'\n')
            except BrokenPipeError: pass
    class Server(socketserver.ThreadingUnixStreamServer): daemon_threads=True
    with Server(str(path),Handler) as server:
        os.chmod(path,0o660)
        print('grasp broker ready:',path,flush=True)
        try: server.serve_forever()
        finally: path.unlink(missing_ok=True)


if __name__=='__main__': main()
