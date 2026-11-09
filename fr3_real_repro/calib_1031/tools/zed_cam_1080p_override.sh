# usage (inside zed-build): bash -s CAM SERIAL   -> restart that camera's wrapper with the 1080p15 override (published 640x360)
CAM=$1; SER=$2; D=/home/spot_ws/zed_ws/zed1080; TS=$(date -u +%Y%m%dT%H%M%SZ)
cat > $D/override_1080p15.yaml <<'YAML'
---
# 2026-10-31 user go: camera grab HD1080 @ 15 fps (depth computed at 1080p), published at 1/3 size = 640x360 as before (960x540 lost frames over DDS).
/**:
    ros__parameters:
        general:
            grab_resolution: 'HD1080'
            grab_frame_rate: 15
            pub_resolution: 'CUSTOM'
            pub_downscale_factor: 3.0
            pub_frame_rate: 15.0
YAML
# launch pids: python ros2-launch processes for this camera (not the bash -c wrapper)
PIDS=$(ps -eo pid,comm,args | awk -v c="camera_name:=$CAM " '$2=="ros2" || $2=="python3" {if (index($0,"zed_camera.launch.py") && index($0,c)) print $1}' | tr '\n' ' ')
[ -n "$PIDS" ] || { echo "no launch for $CAM found"; exit 1; }
LIVE=""; for p in $PIDS; do ps -eo ppid,args | awk -v p=$p '$1==p && /component_container/ {f=1} END{exit !f}' && LIVE=$p; done; [ -n "$LIVE" ] || LIVE=${PIDS%% *}
echo "$CAM launch pids: $PIDS (live: $LIVE)"
[ -f $D/${CAM}_environ.bin ] || cp /proc/$LIVE/environ $D/${CAM}_environ.bin
readlink /proc/$LIVE/cwd > $D/${CAM}_cwd.txt
python3 - $CAM $LIVE <<'PY'
import sys
cam, live = sys.argv[1:3]; D='/home/spot_ws/zed_ws/zed1080'
a = open(f'/proc/{live}/cmdline','rb').read().split(b'\0')[:-1]
a = [x for x in a if not x.startswith(b'ros_params_override_path:=')] + [b'ros_params_override_path:=' + D.encode() + b'/override_1080p15.yaml']
open(f'{D}/{cam}_cmdline.1080.bin','wb').write(b'\0'.join(a)+b'\0')
open(f'{D}/launch_{cam}.py','w').write(f"import os\nD='{D}'\nargs=open(D+'/{cam}_cmdline.1080.bin','rb').read().split(b'\\0')[:-1]\nenv=dict(e.split(b'=',1) for e in open(D+'/{cam}_environ.bin','rb').read().split(b'\\0') if b'=' in e)\nos.chdir(open(D+'/{cam}_cwd.txt').read().strip()); os.execve(args[0], args, env)\n")
PY
cp /tmp/$CAM.log /tmp/$CAM.log.bak_before_1080_$TS 2>/dev/null
for p in $PIDS; do kill -INT $p; done
for i in $(seq 1 60); do alive=0; for p in $PIDS; do kill -0 $p 2>/dev/null && alive=1; done; [ $alive = 0 ] && break; sleep 0.5; done
LEFT=$(ps -eo pid,args | grep -E "__ns:=/$CAM( |$)|${CAM}_state_publisher|camera_name:=$CAM " | grep -v -E "grep|bash -c" | awk '{print $1}' | tr '\n' ' ')
echo "after SIGINT left: [$LEFT]"
if [ -n "$LEFT" ]; then kill -TERM $LEFT; sleep 5; LEFT=$(ps -eo pid,args | grep -E "__ns:=/$CAM( |$)|${CAM}_state_publisher|camera_name:=$CAM " | grep -v -E "grep|bash -c" | awk '{print $1}' | tr '\n' ' '); echo "after SIGTERM left: [$LEFT]"; fi
[ -z "$LEFT" ] || { echo "$CAM processes still alive: NOT relaunching"; exit 2; }
sleep 3; setsid nohup python3 $D/launch_$CAM.py > /tmp/$CAM.log 2>&1 < /dev/null &
echo "relaunched $CAM at $(date -u +%T)"
for i in $(seq 1 24); do sleep 5; C=$(ps -eo args | grep -c "[c]omponent_container.*__ns:=/$CAM\b")
  L=$(tr -d '\033' < /tmp/$CAM.log | grep -a -E "Camera Model|Depth mode|Resolution|Camera Opened|ERROR|Corrupted|REBOOT|Initialized|Starting Positional" | tail -1 | cut -c1-150); echo "[$((i*5))s] container=$C ${L}"
  [ $i -ge 4 ] && [ "$C" -ge 1 ] && tr -d '\033' < /tmp/$CAM.log | grep -a -q "Starting Positional Tracking" && break; done
