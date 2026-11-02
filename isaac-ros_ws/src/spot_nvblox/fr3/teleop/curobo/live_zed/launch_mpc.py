"""Run the current Murilo MPC configuration with guarded private output, initially standby."""
import json
import subprocess
from pathlib import Path

HERE = Path(__file__).resolve().parent
source = json.loads((HERE / 'backup/hub-curobo-mpc.json').read_text())[0]
name = 'fr3-zed-live-mpc'
running = subprocess.check_output(['docker', 'ps', '-a', '--format', '{{.Names}}'], text=True).splitlines()
if name in running:
    raise SystemExit('Inspect existing ' + name + ' before launching again')
args = ['docker', 'run', '-d', '--name', name, '--network', 'host', '--ipc', 'host', '--gpus', 'device=0',
        '--cap-add', 'SYS_NICE', '--ulimit', 'rtprio=99:99']
for env in source['Config']['Env']:
    if env.split('=', 1)[0] not in ('HOSTNAME', 'NVIDIA_VISIBLE_DEVICES', 'FASTDDS_BUILTIN_TRANSPORTS'):
        args += ['-e', env]
# shared memory + UDP: the multi-MB ESDF replies from claude-esdf-proxy (same host, ipc host) were losing UDP fragments
# under load and waiting for the 3 s Fast DDS heartbeat repair, which tripped the bridge's 1.5 s ESDF freshness gate
args += ['-e', 'FASTDDS_BUILTIN_TRANSPORTS=DEFAULT']
for mount in source['Mounts']:
    if mount['Destination'] == '/out':
        continue
    args += ['-v', mount['Source'] + ':' + mount['Destination'] + (':ro' if not mount['RW'] else ':rw')]
out = HERE / 'logs'
out.mkdir(exist_ok=True)
args += ['-v', str(HERE) + ':/zed_live:ro', '-v', str(out) + ':/out:rw']
command = source['Config']['Cmd'][-1]
command = command.replace('feedback_mode:=hybrid','feedback_mode:=measured')
command = command.replace('/node/curobo_mpc_node.py', '/zed_live/live_mpc.py')
command = command.replace('--ros-args', '--ros-args -r __node:=fr3_joystick_mpc -p safe_packet_port:=47927 -r /wrist_pose:=/fr3_joystick_mpc/wrist_goal', 1)
command = command.replace('/curobo/joint_command', '/fr3_joystick_mpc/joint_preview')
command = command.replace('fr3_vcap0.8_acc10.yml', 'fr3_vcap1.2_acc10.yml')   # joystick speed (1.2 rad/s); command_guard already allows 1.201
args += ['--entrypoint', 'bash', source['Config']['Image'], '-c', command]
subprocess.run(args, check=True)
