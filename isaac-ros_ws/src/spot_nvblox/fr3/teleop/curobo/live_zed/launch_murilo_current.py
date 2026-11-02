"""Replace our inactive preview MPC with the current Murilo source; retain rollback."""
import subprocess,json,time
from pathlib import Path
from urllib.request import urlopen
HERE=Path(__file__).resolve().parent
state=json.load(urlopen('http://127.0.0.1:8784/status',timeout=3))
if state.get('armed'):raise SystemExit('Disarm output before changing planner')
source=json.loads(subprocess.check_output(['docker','inspect','fr3-zed-live-mpc'],text=True))[0]
old='fr3-zed-live-mpc-before-murilo-'+str(int(time.time()))
(HERE/'backup'/('before-murilo-'+str(int(time.time()))+'.json')).write_text(json.dumps(source,indent=2))
command=source['Config']['Cmd'][-1]
if '/zed_live/murilo_current.py' in command:raise SystemExit('Current Murilo adapter already installed; inspect instead of replacing')
command=command.replace('/zed_live/live_mpc.py','/zed_live/murilo_current.py')
options=' -p esdf_update_rate:=4.0 -p command_type:=joint_trajectory -p command_period:=0.02 -p mpc_on_infeasible:=keep'
assert ' > /out/mpc_stdout.log' in command
command=command.replace(' > /out/mpc_stdout.log',options+' > /out/mpc_stdout.log')
subprocess.run(['docker','stop','-t','15','fr3-zed-live-mpc'],check=True)
subprocess.run(['docker','rename','fr3-zed-live-mpc',old],check=True)
args=['docker','run','-d','--name','fr3-zed-live-mpc','--network','host','--ipc','host','--gpus','device=0','--cap-add','SYS_NICE','--ulimit','rtprio=99:99']
for env in source['Config']['Env']:
 if env.split('=',1)[0]!='HOSTNAME':args+=['-e',env]
for m in source['Mounts']:args+=['-v',m['Source']+':'+m['Destination']+('' if m['RW'] else ':ro')]
args+=['--entrypoint','bash',source['Config']['Image'],'-c',command]
subprocess.run(args,check=True)
(HERE/'logs/murilo_launch.json').write_text(json.dumps(dict(rollback_container=old,command=command,hardware_armed=False,original_source='/home/murilo/franka_ws/hub-curobo/node/curobo_mpc_node.py'),indent=2))
