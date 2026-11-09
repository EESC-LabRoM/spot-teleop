# Handoff — TLX + tempos unified flow, outcome/contact marking (2026-11-04)

Lane: `Formulário NASA TLX` (RAL). Session `01a1a8cc-5315-7000-a117-81e162bf1baa`.

Prior notes (still valid for background): [nasa-tlx-form](HANDOFF_2026-11-04_nasa-tlx-form.md) (TLX form, scoring bug, ethics) + [tempos-teleop](HANDOFF_2026-11-04_tempos-teleop.md) (times schema v1, endpoints). This note covers the unification session only.

## What was asked

User (PT): on `http://100.84.207.115:8901/tempos` the two separate forms are not intuitive to fill (header twice), and there is no way to mark obstacle contact or failure per attempt.

## What was done

### 1. `nasa-tlx/serve.py` — schema extended (stdlib only)

- `times.csv` header is now 12 cols: `timestamp,operator,date,condition,task,attempt,obstacle,duration_s,outcome,contact,contact_count,notes`.
  - `outcome`: `success` (default) | `failure`, with PT aliases (`sucesso`/`falha`/`ok`/`erro`…).
  - `contact`: `yes` | `no` (default), with aliases (`tocou`/`sim`/`1`…); `contact_count` int 0–99; `count > 0` forces `contact=yes`.
- `POST /api/times` validates the new fields (422 on bad values); legacy 9-col rows are migrated in place on read (empty `outcome/contact/contact_count`, shown as `—` in UI).
- `GET /api/times` + `GET /api/consolidated[.csv]` accept `outcome=` / `contact=` filters; consolidated header = `TIMES_COLUMNS` + TLX cols.
- `POST /submit` + `submissions.csv` 19-col format untouched.

### 2. `nasa-tlx/tempos-teleop.html` — rewritten as guided 3-step run flow (657 lines)

- Step 1: block header filled **once** (operator, date, condition, task).
- Step 2: attempts with timer + result toggle (Sucesso/Falha) + contact toggle (Sem contato/Tocou) + touch stepper.
- Step 3: TLX handoff button carrying the header over (URL params + shared localStorage key).
- Block panel: live summary of current block (attempts, success/failures, contacts, mean time per obstacle condition) + TLX status (pending vs. closed with scores).
- Table + CSV exports gained result/contact columns with filters.

### 3. `nasa-tlx/NASA-TLX-teleop.html` — header sync back

- Prefills header from URL params, then shared key (only into empty fields); back-link to `/tempos`; banner showing how many attempts the current block already has. Scoring/submit paths untouched.

## Current state (verified 2026-11-04 ~20:0x UTC)

- Server running: `python3 /home/nexus/spot-teleop/nasa-tlx/serve.py 8901 100.84.207.115`, PID `2352125` (since 13:59), on `100.84.207.115:8901`.
- `curl -s http://100.84.207.115:8901/api/count` → `{"submissions": 7, "times": 52}`.
- `node smoke_test.js` → `ALL PASS` (14/14, TLX format untouched).
- Data is now **mixed demo + real**: `submissions.csv` 7 rows (4× Hipólito 03/11 demo + 3× Murilo 04/11/26 bottle/ball real); `times.csv` 52 rows (3 legacy valve rows with empty outcome/contact + 49 with outcome/contact, incl. Murilo's real bottle/ball runs 17:06–18:27 UTC). Do NOT blanket-truncate without asking — real collection happened.

## Where things are

| Artifact | Path / URL |
|---|---|
| Run page (unified flow) | [nasa-tlx/tempos-teleop.html](nasa-tlx/tempos-teleop.html) — http://100.84.207.115:8901/tempos |
| TLX form | [nasa-tlx/NASA-TLX-teleop.html](nasa-tlx/NASA-TLX-teleop.html) — http://100.84.207.115:8901/ |
| Server | [nasa-tlx/serve.py](nasa-tlx/serve.py) |
| Times data (git-ignored) | [nasa-tlx/times.csv](nasa-tlx/times.csv) |
| TLX data (git-ignored) | [nasa-tlx/submissions.csv](nasa-tlx/submissions.csv) |
| Test | [nasa-tlx/smoke_test.js](nasa-tlx/smoke_test.js) — `node smoke_test.js` |
| Prior handoffs | [HANDOFF_2026-11-04_tempos-teleop.md](HANDOFF_2026-11-04_tempos-teleop.md), [HANDOFF_2026-11-04_nasa-tlx-form.md](HANDOFF_2026-11-04_nasa-tlx-form.md) |

Restart: `python3 /home/nexus/spot-teleop/nasa-tlx/serve.py 8901 100.84.207.115` (detached: `setsid -f … </dev/null >>/tmp/tlx-serve.log 2>&1`).

## What is left / next steps

1. **User visual check pending** — asked them to hard-refresh (Ctrl+Shift+R), save one attempt as Falha + 2 touches, and confirm row/summary/layout. No browser on this host; only endpoints + smoke test verified here.
2. **Ethics still unresolved** (from prior note): Gustavo protocol route dead per user; confirm a consent path before any identifiable collection beyond the authors.
3. **Demo vs. real data**: only the 3 legacy valve rows (empty outcome/contact, `2026-11-03T20:26:51Z`) are safe demo candidates now; the rest includes Murilo's real 04/11 runs. Ask before deleting.
4. **Persistence**: still a manual process; needs systemd unit / persistent tmux to survive reboot.
5. **Don't break**: keep `POST /submit` + 19-col TLX format (smoke test depends on it); keep `TIMES_COLUMNS` order (consolidated CSV + UI depend on it). Flat CSVs, no locking — fine for manual collection only.
