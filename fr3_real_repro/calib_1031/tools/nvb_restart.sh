set -e
P=353149
tr '\0' ' ' < /proc/$P/cmdline | grep -q "__node:=nvblox_curobo" || { echo "pid $P is not nvblox_curobo: abort"; exit 1; }
cp /proc/$P/cmdline /tmp/nvb_cmdline.bin; cp /proc/$P/environ /tmp/nvb_environ.bin; readlink /proc/$P/cwd > /tmp/nvb_cwd.txt
[ -s /tmp/nvb_cmdline.bin ] && [ -s /tmp/nvb_environ.bin ] || { echo "capture empty: abort"; exit 1; }
cat > /tmp/nvb_launch.py <<'PY'
import os
args = open('/tmp/nvb_cmdline.bin', 'rb').read().split(b'\0')[:-1]
env = dict(e.split(b'=', 1) for e in open('/tmp/nvb_environ.bin', 'rb').read().split(b'\0') if b'=' in e)
os.chdir(open('/tmp/nvb_cwd.txt').read().strip()); os.execve(args[0], args, env)
PY
echo "captured: $(wc -c < /tmp/nvb_cmdline.bin) B cmdline, $(wc -c < /tmp/nvb_environ.bin) B environ, cwd $(cat /tmp/nvb_cwd.txt)"
kill -TERM $P
for i in $(seq 1 30); do kill -0 $P 2>/dev/null || break; sleep 0.5; done
if kill -0 $P 2>/dev/null && ! ps -o stat= -p $P | grep -q Z; then echo "still alive after 15 s: SIGKILL"; kill -KILL $P; sleep 1; fi
echo "old gone: $(ps -o stat= -p $P 2>/dev/null || echo yes)"
setsid nohup python3 /tmp/nvb_launch.py > /tmp/nvblox_curobo_1031.log 2>&1 < /dev/null &
sleep 3; echo "new nvblox_curobo pids: $(pgrep -f '__node:=nvblox_curobo' | tr '\n' ' ')"
