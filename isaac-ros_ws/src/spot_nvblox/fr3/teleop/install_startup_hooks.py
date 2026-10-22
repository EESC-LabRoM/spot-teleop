#!/usr/bin/env python3
"""Use persistent ZED runtime on lab real-robot startup, preserving original Python programs.
Run after deploy.py has copied a complete runtime to /lab/fr3_zed_runtime.
FR3_ZED_RUNTIME=0 or removal of .enabled restores the original startup choice.
"""
from pathlib import Path
import shutil
import subprocess
LAB=Path('/home/murilo/franka_ws/lab')
RUNTIME=LAB/'fr3_zed_runtime'

def main():
    required=('fr3_input_teleop.py','fr3_web_dashboard.py','fr3_dashboard.html')
    if not all((RUNTIME/n).is_file() for n in required):raise RuntimeError('Deploy persistent runtime first')
    changes=[]
    p=LAB/'fr3_ros2.sh';s=p.read_text()
    if '# FR3 ZED persistent runtime' not in s:
        anchor='LABDIR=$(cd "$(dirname "$0")" && pwd)\n'
        assert s.count(anchor)==1
        s=s.replace(anchor,anchor+'''# FR3 ZED persistent runtime: starts disengaged; retain the original joystick programs.
TELEOP_SCRIPT=/lab/fr3_joy_teleop.py; DASHBOARD_SCRIPT=/lab/fr3_web_dashboard.py
if [ "${FR3_ZED_RUNTIME:-1}" = 1 ] && [ -f "$LABDIR/fr3_zed_runtime/.enabled" ]; then
  TELEOP_SCRIPT=/lab/fr3_zed_runtime/fr3_input_teleop.py
  DASHBOARD_SCRIPT=/lab/fr3_zed_runtime/fr3_web_dashboard.py
fi
''')
        for old,new in [('python3 /lab/fr3_web_dashboard.py --ros-args -p joy_device:=auto -p source:=','python3 $DASHBOARD_SCRIPT --ros-args -p joy_device:=auto -p source:='),('python3 /lab/fr3_joy_teleop.py --ros-args --params-file /lab/fr3_joy_teleop.yaml -p joy_device:=auto','python3 $TELEOP_SCRIPT --ros-args --params-file /lab/fr3_joy_teleop.yaml -p joy_device:=auto')]:
            assert s.count(old)==1,old
            s=s.replace(old,new)
        changes.append((p,s))
    p=LAB/'start_remote_dashboard.sh';s=p.read_text()
    if '# FR3 ZED persistent runtime' not in s:
        anchor='set -uo pipefail\n';assert s.count(anchor)==1
        s=s.replace(anchor,anchor+'''# FR3 ZED persistent runtime
DASHBOARD_SCRIPT=/lab/fr3_web_dashboard.py
if [ "${FR3_ZED_RUNTIME:-1}" = 1 ] && [ -f "$(dirname "$0")/fr3_zed_runtime/.enabled" ]; then
  DASHBOARD_SCRIPT=/lab/fr3_zed_runtime/fr3_web_dashboard.py
fi
''').replace('python3 /lab/fr3_web_dashboard.py --ros-args','python3 $DASHBOARD_SCRIPT --ros-args')
        changes.append((p,s))
    p=LAB/'restart_teleop_remote.sh';s=p.read_text()
    old="pids=$(d pgrep -f '^python3 /lab/fr3_joy_teleop.py')"
    if old in s:
        s=s.replace(old+"; ", """# Wait for the detached startup shell to finish sourcing ROS.
for attempt in 1 2 3 4 5; do
  pids=$(d pgrep -f '^python3 /lab/(fr3_joy_teleop|fr3_zed_runtime/fr3_input_teleop)\\.py' || true)
  [ -n "$pids" ] && break
  sleep 0.5
done
""")
        changes.append((p,s))
    for p,s in changes:
        subprocess.run(['bash','-n'],input=s,text=True,check=True)
    for p,s in changes:
        backup=p.with_name(p.name+'.bak-zed-runtime-20261020')
        if not backup.exists():shutil.copy2(p,backup)
        p.write_text(s)
    dependency=RUNTIME/'remote_pad.py'
    if not dependency.exists():dependency.symlink_to('../remote_pad.py')
    (RUNTIME/'.enabled').touch()
    print('Persistent ZED startup enabled. No processes restarted and no hardware action requested.')
if __name__=='__main__':main()
