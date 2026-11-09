# Handoff — Status site tempos / NASA-TLX — 2026-11-06 (~23:20 UTC)

Session: `01a1b3ce-1657-7000-9745-f03453d578fc` (Omnigent ACP lane "Status site tempos NASA", project RAL).
User ask (PT): "quero que ele veja qual o link do site pra preencher os tempos e o nasa, se já tem dados salvos e se ele tá de pé".
Read-only lane: no code changed, no restart, no files touched except this handoff.

## What was done
- Answered all three live (verified this session, 23:20 UTC):
  - `/` (TLX form) → HTTP 200, 26,216 bytes.
  - `/tempos` (times form) → HTTP 200, 33,443 bytes.
  - `/api/count` → `{"submissions": 8, "times": 57}`.
- Confirmed server still the Nov-05 restart: PID `2877595`, started `Thu Nov 5 18:41:01 2026`,
  cmd `python3 /home/nexus/spot-teleop/nasa-tlx/serve.py 8901 100.84.207.115`,
  listening on `100.84.207.115:8901` (via `ss -tlnp`).
- Data grew since the restart handoff (was 7/52): now 8 submissions (9 lines w/ header)
  + 57 times rows (58 lines w/ header). Newest rows are Murilo joystick/screwdriver 06/11:
  times `2026-11-06T21:17:44Z` + `21:19:43Z`, TLX `2026-11-06T21:25:06Z`.
- Reported links + counts + `curl -s http://100.84.207.115:8901/api/count` verify recipe to user.

## Where things are
- Server: [nasa-tlx/serve.py](nasa-tlx/serve.py) (stdlib only, defaults `8901`, `0.0.0.0`).
- TLX form: http://100.84.207.115:8901/ → [nasa-tlx/NASA-TLX-teleop.html](nasa-tlx/NASA-TLX-teleop.html)
- Tempos form: http://100.84.207.115:8901/tempos → [nasa-tlx/tempos-teleop.html](nasa-tlx/tempos-teleop.html)
- Data (git-ignored, do NOT delete): [nasa-tlx/times.csv](nasa-tlx/times.csv), [nasa-tlx/submissions.csv](nasa-tlx/submissions.csv)
- Log: `/tmp/tlx-serve.log`
- Prior detail: [HANDOFF_2026-11-05_nasa-tlx-restart.md](HANDOFF_2026-11-05_nasa-tlx-restart.md) (restart recipe + pre-restart data check)
- Note: `ss` also shows an unrelated `0.0.0.0:18901` listener — not this server.

## What is left / watch-outs
- Nothing pending — server up, data intact, no action needed.
- If down: `setsid -f python3 /home/nexus/spot-teleop/nasa-tlx/serve.py 8901 100.84.207.115 </dev/null >>/tmp/tlx-serve.log 2>&1`, then `curl -s http://100.84.207.115:8901/api/count`.
- Tailnet-only: links open from tailnet devices only. If the tailnet IP changes, re-check `ip -4 addr show tailscale0` and restart with the new IP (or `0.0.0.0`).
- Did NOT touch other RAL lanes' files (live grasp/cloud lane active — left alone).
