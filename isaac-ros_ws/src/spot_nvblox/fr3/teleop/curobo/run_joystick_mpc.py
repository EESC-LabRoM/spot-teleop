#!/usr/bin/env python3
"""Start an isolated MPC preview. Never loads/switches a hardware controller."""
import json,subprocess
from pathlib import Path
HERE=Path(__file__).resolve().parent
NAME='fr3-joystick-mpc'
def run(*a):return subprocess.run(a,check=True,capture_output=True,text=True).stdout
source=json.loads(run('docker','inspect','franka-mpc-real'))[0]
if NAME in run('docker','ps','-a','--format','{{.Names}}').splitlines():raise SystemExit(NAME+' already exists; inspect it before replacing')
args=['docker','run','-d','--name',NAME,'--network','host','--ipc','host','--gpus','device=1']
for value in source['Config']['Env']:
 if value.split('=',1)[0] in ('ROS_DISTRO','RMW_IMPLEMENTATION','LD_LIBRARY_PATH','PYTHONPATH','PYTHONUNBUFFERED','SETUPTOOLS_SCM_PRETEND_VERSION','PYTHONDONTWRITEBYTECODE','ACCEPT_EULA'):
  args+=['-e',value]
args+=['-e','ROS_DOMAIN_ID=8','-e','NVIDIA_VISIBLE_DEVICES=1','-e','CUDA_VISIBLE_DEVICES=0']
for mount in source['Mounts']:
 if mount['Destination'] in ('/node','/out'):continue
 args+=['-v',mount['Source']+':'+mount['Destination']+('' if mount['RW'] else ':ro')]
out=HERE/'joystick_logs';out.mkdir(exist_ok=True)
args+=['-v',str(HERE)+':/zed_mpc:ro','-v',str(out)+':/out', '--entrypoint','bash',source['Config']['Image'],'-c',
 '/isaac-sim/python.sh /zed_mpc/joystick_mpc.py --ros-args -r __node:=fr3_joystick_mpc '
 '-p robot:=franka -p base_frame:=fr3_link0 -p robot_config:=/workspace/franka_mpc/node_config/fr3_vcap1.2_acc10.yml '
 '-p use_sim:=false -p use_sim_time:=false -p use_ros2_control:=false -p use_esdf:=true '
 '-p voxel_size:=0.05 -p grid_center_m:=[0.35,0.0,0.55] -p grid_size_m:=[1.4,1.6,1.2] '
 '-p esdf_service_name:=/nvblox_curobo/get_esdf_and_gradient -p esdf_frame_id:=fr3_link0 -p esdf_global_frame:=fr3_link0 '
 '-p target_clear_frame:=fr3_hand -p target_clear_radius_m:=0.15 -p target_clear_box_topic:=/fr3_joystick_mpc/unused_clear_box '  # self-clear: residual hand/finger/wrist-camera voxels made every state infeasible (5 cm map, 10-23)
 '-p safe_packet_port:=47827 -p feedback_mode:=measured -p joint_stale_s:=0.4 -p pf_mode:=off -p log_csv_path:=/out/mpc_node.csv '
 '-r /wrist_pose:=/fr3_joystick_mpc/wrist_goal -r /arm/joint_command:=/fr3_joystick_mpc/joint_preview '
 '> /out/stdout.log 2>&1']
args[-1]=args[-1].replace('pf_mode:=off', "'pf_mode:=\"off\"'")
print(run(*args).strip());print('MPC on GPU1 with guarded UDP packets; no controller switch or hardware bridge enabled.')
