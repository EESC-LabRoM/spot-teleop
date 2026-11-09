"""Tiny UDP forwarder (stdlib only), for the lab side of the VM 103 <-> lab cable.
   python3 udp_fwd.py LISTEN_IP:PORT TO_IP:PORT [--allow SRC_IP]
Drops anything not from --allow (when given). Prints a count every 10 s."""
import socket, sys, time
src, dst = sys.argv[1], sys.argv[2]; allow = sys.argv[sys.argv.index("--allow") + 1] if "--allow" in sys.argv else None
lh, lp = src.rsplit(":", 1); dh, dp = dst.rsplit(":", 1)
s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM); s.bind((lh, int(lp))); o = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
n = drop = 0; t = time.monotonic(); print(f"udp_fwd {src} -> {dst} allow={allow}", flush=True)
while True:
    d, a = s.recvfrom(65536)
    if allow and a[0] != allow: drop += 1
    else: o.sendto(d, (dh, int(dp))); n += 1
    if time.monotonic() - t > 10: print(f"fwd {n} dropped {drop}", flush=True); t = time.monotonic(); n = drop = 0
