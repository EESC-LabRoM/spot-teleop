set -e
P=""; for p in $(pgrep -x nvblox_node); do tr '\0' ' ' < /proc/$p/cmdline | grep -q "__node:=nvblox_curobo" && P=$p; done
[ -n "$P" ] || { echo "nvblox_curobo not found: abort"; exit 1; }
cp /proc/$P/cmdline /tmp/nvc_cmdline.bin; cp /proc/$P/environ /tmp/nvc_environ.bin; readlink /proc/$P/cwd > /tmp/nvc_cwd.txt
[ -s /tmp/nvc_cmdline.bin ] && [ -s /tmp/nvc_environ.bin ] || { echo "capture empty: abort"; exit 1; }
grep -q "^    voxel_size: 0.05$" $(tr '\0' '\n' < /tmp/nvc_cmdline.bin | grep launch_params_ | head -1) || { echo "unexpected params: abort"; exit 1; }
cat > /tmp/nvc_launch.py <<'PY'
import os
args = open('/tmp/nvc_cmdline.bin','rb').read().split(b'\0')[:-1]
env = dict(e.split(b'=',1) for e in open('/tmp/nvc_environ.bin','rb').read().split(b'\0') if b'=' in e)
os.chdir(open('/tmp/nvc_cwd.txt').read().strip()); os.execve(args[0], args, env)
PY
kill -TERM $P; for i in $(seq 1 30); do kill -0 $P 2>/dev/null || break; sleep 0.5; done
if kill -0 $P 2>/dev/null && ! ps -o stat= -p $P | grep -q Z; then echo SIGKILL; kill -KILL $P; sleep 1; fi
setsid nohup python3 /tmp/nvc_launch.py > /tmp/nvblox_curobo_1031b.log 2>&1 < /dev/null &
sleep 5; NP=""; for p in $(pgrep -x nvblox_node); do tr '\0' ' ' < /proc/$p/cmdline | grep -q "__node:=nvblox_curobo" && NP=$p; done
echo "nvblox_curobo restarted clean (same params, 5 cm): pid $NP (old $P)"
