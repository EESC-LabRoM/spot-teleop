#!/usr/bin/env python3
"""Install this runtime without editing /lab originals. Refuses an engaged teleop.
python3 deploy.py --apply    # install + replace the existing teleop/dashboards, preserving arguments
python3 deploy.py --restore  # restore the saved original process commands
"""
import argparse
import json
import os
from pathlib import Path
import re
import shlex
import subprocess
import time

HERE = Path(__file__).resolve().parent
STATE = Path.home()/'.local/state/fr3-zed/deployment.json'
DEST = '/lab/fr3_zed_runtime'
CONTAINER = 'fr3-ros2'
ENV = 'source /opt/ros/jazzy/setup.bash; source /ros2_ws/install/setup.bash; export PYTHONPATH=/lab:${PYTHONPATH:-}; '


def docker(*args, check=True):
    return subprocess.run(['docker', 'exec', CONTAINER, *args], text=True, capture_output=True, check=check)


def processes():
    code = '''import json,pathlib
out=[]
for p in pathlib.Path('/proc').iterdir():
 if not p.name.isdigit(): continue
 try:
  a=(p/'cmdline').read_bytes().decode().rstrip('\\0').split('\\0')
  if len(a)>1 and a[0].endswith('python3') and pathlib.Path(a[1]).name in ('fr3_joy_teleop.py','fr3_input_teleop.py','fr3_input_teleop_nobox.py','fr3_input_teleop_review.py','fr3_web_dashboard.py'):
   out.append(dict(pid=int(p.name),args=a))
 except (OSError,UnicodeError): pass
print(json.dumps(out))'''
    return json.loads(docker('python3','-c',code).stdout)


def log_path(args):
    if Path(args[1]).name != 'fr3_web_dashboard.py': return '/runs/fr3-ros2.teleop.log'
    return '/runs/fr3-ros2.dashboard-remote.log' if 'port:=8089' in args else '/runs/fr3-ros2.dashboard.log'


def guard():
    log = docker('tail','-150','/runs/fr3-ros2.teleop.log').stdout
    log = re.sub(r'\x1b\[[0-9;]*m','',log)
    states = re.findall(r'\[(\d+\.\d+)\].*engaged=(True|False)',log)
    states += [(stamp,'False') for stamp in re.findall(r'\[(\d+\.\d+)\].*\[fr3_joy_teleop\]: disengaged:',log)]
    states += [(stamp,'True') for stamp in re.findall(r'\[(\d+\.\d+)\].*\[fr3_joy_teleop\]: engaged at',log)]
    states.sort(key=lambda x:float(x[0]))
    if not states: raise RuntimeError('No fresh teleop state; refusing restart')
    stamp, engaged = states[-1]
    if engaged != 'False' or not 0 <= time.time()-float(stamp) < 20:
        raise RuntimeError('Teleop engaged or state stale; disengage with Back before installing')


def stop(p):
    docker('kill','-INT',str(p['pid']))
    for _ in range(30):
        if docker('kill','-0',str(p['pid']),check=False).returncode: return
        time.sleep(.1)
    raise RuntimeError('Process did not exit; refusing to start a second writer')


def start(args):
    log = log_path(args)
    cmd = ENV + 'exec "$@" > '+shlex.quote(log)+' 2>&1'
    subprocess.run(['docker','exec','-d',CONTAINER,'bash','-c',cmd,'fr3-zed',*args],check=True)
    time.sleep(2)
    if not any(p['args']==args for p in processes()):
        raise RuntimeError('New process failed; see '+log)


def main():
    ap=argparse.ArgumentParser(); ap.add_argument('--apply',action='store_true'); ap.add_argument('--restore',action='store_true'); ap.add_argument('--refresh',action='store_true'); a=ap.parse_args()
    current=processes()
    tele=[p for p in current if Path(p['args'][1]).name!='fr3_web_dashboard.py']
    if len(tele)!=1: raise RuntimeError('Expected exactly one teleop process')
    if not a.restore and not a.refresh and Path(tele[0]['args'][1]).name=='fr3_input_teleop.py':
        print('Unified teleop is already running; no restart performed.'); return
    if a.restore:
        saved=json.loads(STATE.read_text())
        plans=[]
        for p in current:
            old=next(x['args'] for x in saved if log_path(x['args'])==log_path(p['args']))
            plans.append((p,old))
    else:
        plans=[]
        for p in current:
            args=p['args'].copy()
            args[1]=DEST+'/'+('fr3_web_dashboard.py' if Path(args[1]).name=='fr3_web_dashboard.py' else (Path(args[1]).name if Path(args[1]).name in ('fr3_input_teleop_nobox.py','fr3_input_teleop_review.py') else 'fr3_input_teleop.py'))
            plans.append((p,args))
    print(json.dumps([{'pid':p['pid'],'script':args[1],'log':log_path(args)} for p,args in plans],indent=2))
    if not (a.apply or a.restore or a.refresh): print('Dry run. Pass --apply to install.'); return
    guard()
    if a.restore:
        docker('rm','-f',DEST+'/.enabled')
    if not a.restore:
        if not a.refresh:
            STATE.parent.mkdir(parents=True,exist_ok=True)
            STATE.write_text(json.dumps(current,indent=2)); STATE.chmod(0o600)
        docker('mkdir','-p',DEST)
        subprocess.run(['docker','cp',str(HERE)+'/.',CONTAINER+':'+DEST+'/'],check=True)
    for p,args in plans:
        guard()  # every restart requires a recent disengaged state
        stop(p)
        try: start(args)
        except Exception:
            start(p['args'])
            raise
    print('Restored originals.' if a.restore else 'Installed disengaged. Click Start teleop ZED in the dashboard.')

if __name__=='__main__':main()
