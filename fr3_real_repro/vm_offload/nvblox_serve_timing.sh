#!/usr/bin/env bash
# In-node nvblox ESDF serve time from container logs (read-only).
# Pairs each "Received request for ESDF" stamp with the next "Successfully wrote"
# stamp. Valid while queries don't overlap (true: single-threaded proxy server
# + MPC _esdf_call_pending guard serializes them; mis-pairs would show as
# negative deltas and are rejected below).
# Usage: ./nvblox_serve_timing.sh [docker-logs-since] [container]
SINCE=${1:-30m}
NAME=${2:-vm-nvblox-prod}
docker logs --since "$SINCE" "$NAME" 2>&1 \
  | grep -B1 -E "Received request for ESDF|Successfully wrote requested ESDF" \
  | grep -oE '\[1[0-9]+\.[0-9]+\]' | tr -d '[]' \
  | python3 -c "
import sys, statistics
ts = [float(x) for x in sys.stdin.read().split()]
dts = sorted((ts[i+1]-ts[i])*1000 for i in range(0, len(ts)-1, 2) if ts[i+1] >= ts[i])
n = len(dts)
print(f'samples: {n}')
if n:
    print('in-node serve ms: min %.0f p50 %.0f p90 %.0f p99 %.0f max %.0f'
          % (dts[0], dts[n//2], dts[int(n*0.9)], dts[int(n*0.99)], dts[-1]))
    print('>1s: %d  >3s: %d' % (sum(1 for d in dts if d > 1000),
                                sum(1 for d in dts if d > 3000)))
"
