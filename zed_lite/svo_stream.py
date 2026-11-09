"""Stream a recorded SVO in a loop on a ZED streaming port, as the Isaac ZED extension does, so that zed_wrapper (sim_mode:=true sim_port:=<port>)
runs its real depth pipeline WITHOUT Isaac (MPC rate tests under camera load, 10-29). No depth here (sender only grabs + encodes); EOF -> frame 0.
   python3 svo_stream.py --svo rec_zed.svo2 --port 30000 [--fps 30]"""
import argparse, time
import pyzed.sl as sl

ap = argparse.ArgumentParser(); ap.add_argument("--svo", required=True); ap.add_argument("--port", type=int, default=30000); ap.add_argument("--fps", type=float, default=30.0)
a = ap.parse_args()
init = sl.InitParameters(); init.set_from_svo_file(a.svo); init.svo_real_time_mode = False; init.depth_mode = sl.DEPTH_MODE.NONE
cam = sl.Camera(); err = cam.open(init)
if err != sl.ERROR_CODE.SUCCESS: raise SystemExit(f"open {a.svo}: {err}")
n = cam.get_svo_number_of_frames(); res = cam.get_camera_information().camera_configuration.resolution
sp = sl.StreamingParameters(); sp.port = a.port; sp.codec = sl.STREAMING_CODEC.H265
err = cam.enable_streaming(sp)
if err != sl.ERROR_CODE.SUCCESS: raise SystemExit(f"enable_streaming: {err}")
print(f"[svo_stream] {a.svo}: {n} frames {res.width}x{res.height}, streaming on {a.port} at {a.fps} fps", flush=True)
rt = sl.RuntimeParameters(); period = 1.0 / a.fps; t_next = time.time(); sent = 0; loops = 0
while True:
    e = cam.grab(rt)
    if e == sl.ERROR_CODE.END_OF_SVOFILE_REACHED:
        cam.set_svo_position(0); loops += 1; continue
    if e != sl.ERROR_CODE.SUCCESS: print(f"[svo_stream] grab: {e}", flush=True); time.sleep(0.1); continue
    sent += 1
    if sent % 300 == 0: print(f"[svo_stream] {sent} frames sent, {loops} loops", flush=True)
    t_next += period; d = t_next - time.time()
    if d > 0: time.sleep(d)
    else: t_next = time.time()
