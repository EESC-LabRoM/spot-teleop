# Handoff — NASA-TLX teleop form + collector (2026-11-04)

Lane: `Revisão formulário NASA TLX` (RAL). Session `01a1a488-26ee-7000-8f27-c3466caef717`.

Sibling work (separate lane, already documented): tempos de teleoperação — see
[HANDOFF_2026-11-04_tempos-teleop.md](HANDOFF_2026-11-04_tempos-teleop.md).
That lane extended `serve.py` and added `tempos-teleop.html`; this note covers the
TLX form itself and the collector.

## What was asked

1. Check reviewer notes for any mention of collecting operator data with the NASA form (NASA-TLX).
2. Provide a ready-to-use NASA-TLX model for teleop on the robot, so operators can review it.
3. Make it easy for people to fill in; fix scoring display issues reported by the user.

## What was done

### 1. Reviewer investigation (recovered from chat history)

- `~/RA-L/` folder is gone from disk; review texts were recovered from the RAL session
  history, session `4a1867870bb74b44b3fb98f1ddd42887` (positions 59–61 = full AE
  report + both reviews; 212, 222, 295 = plan versions v1/v2/v3).
- **Reviewer 2 (verbatim):** "…operator workload is not directly evaluated… A user
  study with more participants, comparisons with pure teleoperation or conventional
  interfaces such as joystick-based control, and standard workload measures such as
  NASA-TLX would considerably strengthen the paper." Explicitly **not** to be left as
  future work. Reviewer 21 separately flagged "no quantitative assessment of
  operational workload or usability."
- Plan history: v1 = full 12-participant user study with NASA-TLX; v2 (no ethics
  approval) = cut all questionnaires; v3 (current) = NASA-TLX only as internal
  authors-only study under Gustavo's approved protocol — scope never confirmed.
- Standing rule was: collect **no** NASA-TLX data until Gustavo confirms coverage.
  **Update this session:** user said the Gustavo protocol route fell through
  ("já foi pro caralho"). Ethics/consent status for any operator collection is now
  **unresolved** — do not assume coverage. Open items: Hipólito co-author vs. consent
  form, no personal data in logs/videos.

### 2. `nasa-tlx/NASA-TLX-teleop.html` — ready-to-use form

- Single self-contained file: works opened directly or served. Print CSS included.
- **Part 1 — Ratings:** 6 standard scales (Mental, Physical, Temporal, Performance,
  Effort, Frustration), 0–100 step 5, one fill per interface condition. Validated
  wording kept; teleop examples added under each scale. Performance anchored
  Perfect→Failure (only scale where left = good), flagged in instructions.
- **Part 2 — Weighting:** all 15 pairwise comparisons (verified complete), once per
  operator. This calibrates per-operator weights — it is NOT the joystick-vs-system
  comparison (that comes from filling the whole form once per condition).
- **Part 3 — Scoring table:** was a static hand-entry sheet; now **auto-fills live**
  from Parts 1–2 (rating + weight per row, Weight×Rating column, Total row with
  sums and `÷ 15 = …` weighted score). Same behavior on print.
- Sticky black score bar pinned to viewport bottom: live progress
  (`Ratings 4/6 · pairs 10/15`) then `Raw TLX: … · Weighted TLX: …`.
- Header fields: operator, date, condition, task. Autosaves to localStorage
  (survives refresh). Submit posts to collector; Download CSV fallback for
  direct-file use.

### 3. `nasa-tlx/serve.py` + `submissions.csv` — collector

- `POST /submit` appends one row (19 cols: ratings, weights, raw + weighted TLX).
  Bound to tailnet IP only (`100.84.207.115:8901`), not the open internet.
- `submissions.csv` and `times.csv` are git-ignored (`.gitignore`); operator data
  must never be committed.
- Later extended by the tempos lane with `/api/tlx`, `/api/times`,
  `/api/consolidated`, etc. — see the tempos handoff; `POST /submit` and the TLX
  CSV format were kept intact.

### 4. Bug fixes (from user reports "no score appeared")

1. Score line only existed at the page bottom → added sticky score bar.
2. **Real bug:** pair labels (`Mental Demand`) never matched uppercase scale keys
   (`MENTAL DEMAND`), so all weights counted as zero → Weighted TLX `0.0` on screen
   and zero weights stored. Fixed in both display and submit paths.
3. Part 3 table was static → made it auto-populate live.

### 5. `nasa-tlx/smoke_test.js` — regression test (kept in repo)

- Dependency-free (`node smoke_test.js`); runs the form's exact script with stub DOM,
  simulates a full fill. **14/14 PASS**, incl. Raw 35.0 / Weighted 34.7 on known
  inputs, table auto-fill + totals, submit payload, CSV export, autosave restore,
  and clear-reset.

## Current state (verified this session)

- Server running: `python3 /home/nexus/spot-teleop/nasa-tlx/serve.py 8901 100.84.207.115`,
  PID `3339211`, on `100.84.207.115:8901`.
- `curl -s http://100.84.207.115:8901/api/count` → `{"submissions": 4, "times": 3}`
  (demo data from Hipólito, 03/11/2026 — see "left" below).
- `node smoke_test.js` → `ALL PASS` (14 checks).
- Manual process, log `/tmp/tlx-serve.log` (or `/tmp/nasa-tlx.log` from first launch);
  **will not survive a VM reboot**.

## Where things are

| Artifact | Path / URL |
|---|---|
| TLX form | [nasa-tlx/NASA-TLX-teleop.html](nasa-tlx/NASA-TLX-teleop.html) — http://100.84.207.115:8901/ |
| Tempos form (sibling lane) | [nasa-tlx/tempos-teleop.html](nasa-tlx/tempos-teleop.html) — http://100.84.207.115:8901/tempos |
| Server | [nasa-tlx/serve.py](nasa-tlx/serve.py) |
| TLX data (git-ignored) | [nasa-tlx/submissions.csv](nasa-tlx/submissions.csv) |
| Times data (git-ignored) | [nasa-tlx/times.csv](nasa-tlx/times.csv) |
| Test | [nasa-tlx/smoke_test.js](nasa-tlx/smoke_test.js) — `node smoke_test.js` |
| Sibling handoff | [HANDOFF_2026-11-04_tempos-teleop.md](HANDOFF_2026-11-04_tempos-teleop.md) |

Restart: `python3 /home/nexus/spot-teleop/nasa-tlx/serve.py 8901 100.84.207.115`
(detached: `setsid -f … </dev/null >>/tmp/tlx-serve.log 2>&1`).

## What is left / next steps

1. **Operators must hard-refresh** (Ctrl+Shift+R) — browsers cache the old form; the
   last two fixes (sticky bar, auto-fill table) only show after that. Last user
   message confirmed the table was still empty pre-refresh; fix is served, awaiting
   their retest.
2. **Resolve ethics before real collection.** Gustavo protocol route is dead per user;
   no replacement decided. Do not collect identifiable operator data until a protocol
   or consent path is confirmed.
3. **Demo data:** 4 TLX rows + 3 times rows (Hipólito / joystick, 03/11/2026). Truncate
   (keep headers) or delete via UI before real collection.
4. **No real-browser visual check** on this machine (no chromium); only HTTP 200 +
   smoke test verified. Open `/` and `/tempos` in a lab browser.
5. **Persistence:** still a manual process; create systemd unit / persistent tmux if it
   must survive reboot.
6. **Offered, not requested yet:** PT-BR version of the form alongside the EN one;
   full reviewer texts dumped to workspace files (currently only in chat DB session
   `4a1867870bb74b44b3fb98f1ddd42887`).
7. **Don't break:** keep `POST /submit` + `submissions.csv` 19-col format (smoke test
   and prior collections depend on it). Flat CSVs have no locking — fine for manual
   collection, not high concurrency.
