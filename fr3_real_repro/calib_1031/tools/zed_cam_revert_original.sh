# inside zed-build: bash -s CAM  -> relaunch that camera exactly as originally started (no override file)
CAM=$1; D=/home/spot_ws/zed_ws/zed1080; TS=$(date -u +%Y%m%dT%H%M%SZ)
python3 - $CAM <<'PY'
import sys
cam=sys.argv[1]; D='/home/spot_ws/zed_ws/zed1080'
a=open(f'{D}/{cam}_cmdline.1080.bin','rb').read().split(b'\0')[:-1]
a=[x for x in a if not x.startswith(b'ros_params_override_path:=')]
open(f'{D}/{cam}_cmdline.orig2.bin','wb').write(b'\0'.join(a)+b'\0')
open(f'{D}/revert_{cam}.py','w').write(f"import os\nD='{D}'\nargs=open(D+'/{cam}_cmdline.orig2.bin','rb').read().split(b'\\0')[:-1]\nenv=dict(e.split(b'=',1) for e in open(D+'/{cam}_environ.bin','rb').read().split(b'\\0') if b'=' in e)\nos.chdir(open(D+'/{cam}_cwd.txt').read().strip()); os.execve(args[0], args, env)\n")
print('original cmdline restored:', b' '.join(a[-4:]).decode())
PY
PIDS=$(ps -eo pid,comm,args | awk -v c="camera_name:=$CAM " '$2=="ros2" || $2=="python3" {if (index($0,"zed_camera.launch.py") && index($0,c)) print $1}' | tr '\n' ' ')
echo "$CAM launch pids: $PIDS"; [ -n "$PIDS" ] || exit 1
cp /tmp/$CAM.log /tmp/$CAM.log.bak_1080_run_$TS
for p in $PIDS; do kill -INT $p; done
for i in $(seq 1 60); do alive=0; for p in $PIDS; do kill -0 $p 2>/dev/null && alive=1; done; [ $alive = 0 ] && break; sleep 0.5; done
LEFT=$(ps -eo pid,args | grep -E "__ns:=/$CAM( |$)|${CAM}_state_publisher|camera_name:=$CAM " | grep -v -E "grep|bash -c" | awk '{print $1}' | tr '\n' ' ')
[ -z "$LEFT" ] || { kill -TERM $LEFT; sleep 5; LEFT=$(ps -eo pid,args | grep -E "__ns:=/$CAM( |$)|${CAM}_state_publisher|camera_name:=$CAM " | grep -v -E "grep|bash -c" | awk '{print $1}' | tr '\n' ' '); }
[ -z "$LEFT" ] || { echo "still alive: $LEFT"; exit 2; }
sleep 3; setsid nohup python3 $D/revert_$CAM.py > /tmp/$CAM.log 2>&1 < /dev/null &
echo "relaunched $CAM (original settings) at $(date -u +%T)"
for i in $(seq 1 20); do sleep 5; tr -d '\033' < /tmp/$CAM.log | grep -a -q "Starting Positional Tracking" && { echo "up after $((i*5)) s"; break; }; done
tr -d '\033' < /tmp/$CAM.log | grep -a -E "Resolution|resolution|FPS" | head -3 | cut -c1-140
