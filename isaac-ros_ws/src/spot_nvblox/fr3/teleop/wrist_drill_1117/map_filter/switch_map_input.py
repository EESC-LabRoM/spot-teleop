import json,subprocess,time
from pathlib import Path
p=Path(__file__).parent
args=json.loads((p/'nvblox_original_args.json').read_text());assert args.count('camera_0/depth/image:=/franka_seg_zed1/depth')==1
new=['camera_0/depth/image:=/wrist_map/drill_free_depth' if a=='camera_0/depth/image:=/franka_seg_zed1/depth' else a for a in args]
(p/'nvblox_filtered_args.json').write_text(json.dumps(new,indent=2))
def d(*a):return subprocess.run(['docker','exec','isaac-build',*a],capture_output=True,text=True,check=True,timeout=20).stdout
assert (p/'map_before_drill_exclusion.nvblx').stat().st_size>100000
s=json.loads(d('cat','/tmp/wrist_map_filter/status.json'))
assert time.time()-s['wall']<3 and s['counts'].get('published',0)>=5
assert s['recent']['masked_nonzero']==0 and s['recent']['outside_changed']==0
assert time.time()-s['recent']['stamp']<3,'filtered output stalled'
# No active wrist writers; real JTC already deactivated.
ps=subprocess.check_output(['docker','exec','fr3-ros2','ps','-eo','args'],text=True)
assert '/tmp/direct_topic_fwd.py' not in ps and '/lab/fw_rev/direct_goal_fwd.py' not in ps
kill='''import pathlib,os,signal,time
p=pathlib.Path('/proc/220/cmdline');assert '/nvblox_ros/nvblox_node' in p.read_bytes().decode();os.kill(220,signal.SIGINT)
for _ in range(80):
 if not p.exists():break
 time.sleep(.1)
else:raise RuntimeError('old map did not stop')
'''
print(d('python3','-c',kill),flush=True)
# Clear the dashboard's old incremental mesh cache before the new map emits blocks.
print(d('bash','-lc','source /opt/ros/humble/setup.bash; source /workspaces/isaac_ros-dev/install/setup.bash; exec timeout 8 ros2 topic pub --once /nvblox_curobo/mesh nvblox_msgs/msg/Mesh "{clear: true}"'),flush=True)
subprocess.run(['docker','exec','-d','isaac-build','bash','-lc','source /opt/ros/humble/setup.bash; source /workspaces/isaac_ros-dev/install/setup.bash; export CUDA_VISIBLE_DEVICES=0; exec "$@" > /tmp/wrist_map_filter/nvblox_filtered.log 2>&1','masked-map',*new],check=True)
print('Filtered map launched, original map backed up; old mesh cache cleared.',flush=True)
