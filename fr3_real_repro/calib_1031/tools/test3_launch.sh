set -e
P=2865987
tr '\0' ' ' < /proc/$P/cmdline | grep -q "__node:=nvblox_curobo" || { echo "pid $P is not nvblox_curobo: abort"; exit 1; }
pgrep -f "__node:=nvblox_test3" >/dev/null 2>&1 && { echo "nvblox_test3 already running"; exit 1; }
sed 's/^    voxel_size: 0.05$/    voxel_size: 0.03/' /tmp/launch_params_809iy4xu > /tmp/launch_params_test3; grep -n "voxel_size" /tmp/launch_params_test3
cp /proc/$P/environ /tmp/t3_environ.bin
python3 - <<'PY'
a = open('/proc/2865987/cmdline','rb').read()
a = a.replace(b"__node:=nvblox_curobo", b"__node:=nvblox_test3").replace(b"/tmp/launch_params_809iy4xu", b"/tmp/launch_params_test3")
open('/tmp/t3_cmdline.bin','wb').write(a)
open('/tmp/t3_launch.py','w').write("""import os
args = open('/tmp/t3_cmdline.bin', 'rb').read().split(b'\\0')[:-1]
env = dict(e.split(b'=', 1) for e in open('/tmp/t3_environ.bin', 'rb').read().split(b'\\0') if b'=' in e)
os.chdir('/workspaces/isaac_ros-dev'); os.execve(args[0], args, env)
""")
print("cmdline node:", [x for x in a.split(b'\0') if b'__node' in x], "| params:", [x for x in a.split(b'\0') if b'launch_params' in x])
PY
setsid nohup python3 /tmp/t3_launch.py > /tmp/nvblox_test3.log 2>&1 < /dev/null &
sleep 25
echo "pids: $(pgrep -f '__node:=nvblox_test3' | tr '\n' ' ')"; tail -n 3 /tmp/nvblox_test3.log | cut -c1-200
