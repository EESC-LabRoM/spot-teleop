"""Timed expiry of this lane's perception-only warm-up and standby MPC."""
import pathlib,subprocess,time
r=pathlib.Path(__file__).parent
end=time.monotonic()+1200
while time.monotonic()<end:time.sleep(max(0,min(1,end-time.monotonic())))
# The operator-facing live entry must cancel this watchdog before physical execution.
for name in ['wrist-standing-perception-1119','codex-fr3-wrist-response-1117']:
 subprocess.run(['docker','stop','--timeout','10',name],stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL)
(r/'warmup_expired').write_text(str(time.time()))
