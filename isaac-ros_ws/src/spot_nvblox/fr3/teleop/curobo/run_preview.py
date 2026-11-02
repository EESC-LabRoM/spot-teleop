#!/usr/bin/env python3
"""Start an isolated MPC preview. Never loads/switches a hardware controller."""
import json,subprocess
from pathlib import Path
HERE=Path(__file__).resolve().parent
NAME='fr3-zed-mpc-preview'
def run(*a):return subprocess.run(a,check=True,capture_output=True,text=True).stdout
source=json.loads(run('docker','inspect','franka-mpc-real'))[0]
if NAME in run('docker','ps','-a','--format','{{.Names}}').splitlines():raise SystemExit(NAME+' already exists; inspect it before replacing')
args=['docker','run','-d','--name',NAME,'--network','host','--ipc','host','--gpus','device=0']
for value in source['Config']['Env']:
 if value.split('=',1)[0] in ('ROS_DISTRO','RMW_IMPLEMENTATION','LD_LIBRARY_PATH','PYTHONPATH','PYTHONUNBUFFERED','SETUPTOOLS_SCM_PRETEND_VERSION','PYTHONDONTWRITEBYTECODE','ACCEPT_EULA'):
  args+=['-e',value]
args+=['-e','ROS_DOMAIN_ID=8']
for mount in source['Mounts']:
 if mount['Destination'] in ('/node','/out'):continue
 args+=['-v',mount['Source']+':'+mount['Destination']+('' if mount['RW'] else ':ro')]
out=HERE/'preview_logs';out.mkdir(exist_ok=True)
args+=['-v',str(HERE)+':/zed_mpc:ro','-v',str(out)+':/out', '--entrypoint','bash',source['Config']['Image'],'-c',
 '/isaac-sim/python.sh /zed_mpc/mpc_node.py --ros-args -r __node:=fr3_zed_mpc '
 '-p robot:=franka -p base_frame:=fr3_link0 -p robot_config:=/workspace/franka_mpc/node_config/fr3.yml '
 '-p use_sim:=false -p use_sim_time:=false -p use_ros2_control:=false -p use_esdf:=true '
 '-p voxel_size:=0.02 -p grid_center_m:=[0.35,0.0,0.55] -p grid_size_m:=[1.4,1.6,1.2] '
 '-p esdf_service_name:=/nvblox_human/get_esdf_and_gradient -p esdf_frame_id:=fr3_link0 -p esdf_global_frame:=fr3_link0 '
 '-p feedback_mode:=measured -p joint_stale_s:=0.4 -p pf_mode:=off -p log_csv_path:=/out/mpc_node.csv '
 '-r /wrist_pose:=/fr3_zed_mpc/wrist_goal -r /arm/joint_command:=/fr3_zed_mpc/joint_preview '
 '> /out/stdout.log 2>&1']
args[-1]=args[-1].replace('pf_mode:=off', "'pf_mode:=\"off\"'")
print(run(*args).strip());print('Output is /fr3_zed_mpc/joint_preview only; no hardware bridge or controller switch.')
