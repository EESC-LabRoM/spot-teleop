# Handoff — NASA-TLX / tempos-teleop restart — 2026-11-05 (~18:30 UTC)

Session: `01a1ad99-a528-7000-bf92-9469e7adc3e0` (Omnigent ACP lane). User asked for the
RAL-times form address, then asked to bring the server back up and confirm yesterday's data survived.

## What was done
- Confirmed address from prior handoffs + live tailnet IP: port `8901`, bind `100.84.207.115`
  (`ip addr` still shows `100.84.207.115/32` on tailscale0).
- Checked saved data BEFORE restart — all intact:
  - `nasa-tlx/times.csv`: 52 rows + header (latest `2026-11-04T18:27:41Z`, Murilo ball runs).
  - `nasa-tlx/submissions.csv`: 7 rows + header (latest `2026-11-04T18:28:52Z`, ball TLX).
- Restarted server detached: `setsid -f python3 /home/nexus/spot-teleop/nasa-tlx/serve.py 8901 100.84.207.115 </dev/null >>/tmp/tlx-serve.log 2>&1`
  - PID `2877595`, listening on `100.84.207.115:8901` (verified via `ss -tlnp`).
  - Health check: `curl http://100.84.207.115:8901/api/count` → `{"submissions": 7, "times": 52}`; `GET /tempos` → HTTP 200.

## Where things are
- Server: [nasa-tlx/serve.py](nasa-tlx/serve.py) (stdlib only, defaults `8901`, `0.0.0.0`).
- RAL times form: http://100.84.207.115:8901/tempos → [nasa-tlx/tempos-teleop.html](nasa-tlx/tempos-teleop.html)
- TLX form: http://100.84.207.115:8901/ → [nasa-tlx/NASA-TLX-teleop.html](nasa-tlx/NASA-TLX-teleop.html)
- Data (git-ignored, do NOT delete): [nasa-tlx/times.csv](nasa-tlx/times.csv), [nasa-tlx/submissions.csv](nasa-tlx/submissions.csv)
- Log: `/tmp/tlx-serve.log`
- Prior detail: [HANDOFF_2026-11-04_tlx-tempos-unified-flow.md](HANDOFF_2026-11-04_tlx-tempos-unified-flow.md), [HANDOFF_2026-11-04_tempos-teleop.md](HANDOFF_2026-11-04_tempos-teleop.md), [HANDOFF_2026-11-04_nasa-tlx-form.md](HANDOFF_2026-11-04_nasa-tlx-form.md)

## What is left / watch-outs
- Nothing pending on this task — server is up and data is safe.
- If the server is down again: re-run the `setsid` command above (bind `100.84.207.115`, port `8901`).
- If the tailnet IP ever changes, re-check with `ip -4 addr show tailscale0` and restart with the new IP (or `0.0.0.0`).
