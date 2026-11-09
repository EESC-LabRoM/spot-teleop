D=/home/spot_ws/zed_ws/zed1080
sed -i 's/os.execve(args\[0\], args, env)/os.execvpe(args[0], args, env)/' $D/tfpub_zed1_launch.py
grep -c execvpe $D/tfpub_zed1_launch.py
setsid nohup python3 $D/tfpub_zed1_launch.py > /tmp/tf_zed1_live1031b.log 2>&1 < /dev/null &
sleep 4; echo "zed1 TF publisher pids: $(pgrep -f 'real_extrinsics_tf.py.*--cam zed1' | tr '\n' ' ')"; tail -2 /tmp/tf_zed1_live1031b.log | cut -c1-230
