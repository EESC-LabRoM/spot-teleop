#!/usr/bin/env python3
"""Serve the NASA-TLX teleop form + tempos collector. Stdlib only.

Usage:  python3 serve.py [port] [bind]   (defaults: 8901, tailnet-or-all)
"""
import csv
import json
import os
import sys
from datetime import datetime, timezone
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import urlparse, parse_qs

HERE = os.path.dirname(os.path.abspath(__file__))
FORM = os.path.join(HERE, "NASA-TLX-teleop.html")
TEMPOS = os.path.join(HERE, "tempos-teleop.html")
CSV_PATH = os.path.join(HERE, "submissions.csv")
TIMES_PATH = os.path.join(HERE, "times.csv")

COLUMNS = ["timestamp", "operator", "date", "condition", "task",
           "mental", "physical", "temporal", "performance", "effort",
           "frustration", "raw_tlx",
           "w_mental", "w_physical", "w_temporal", "w_performance",
           "w_effort", "w_frustration", "weighted_tlx"]
SCALES = ["mental", "physical", "temporal", "performance", "effort",
          "frustration"]

TIMES_COLUMNS = ["timestamp", "operator", "date", "condition", "task",
                 "attempt", "obstacle", "duration_s", "outcome",
                 "contact", "contact_count", "notes"]
OBSTACLES = {"sem_obstaculo", "com_obstaculo"}
OUTCOMES = {"success", "failure"}
CONTACTS = {"yes", "no"}
# header used by times.csv before outcome/contact columns existed
TIMES_COLUMNS_LEGACY = ["timestamp", "operator", "date", "condition", "task",
                        "attempt", "obstacle", "duration_s", "notes"]


def ensure_csv():
    if not os.path.exists(CSV_PATH):
        with open(CSV_PATH, "w", newline="") as f:
            csv.writer(f).writerow(COLUMNS)

def _migrate_times_csv():
    """Upgrade a legacy times.csv header in place, preserving rows.

    Old rows get empty outcome/contact/contact_count (shown as '—' in UI).
    """
    try:
        with open(TIMES_PATH, newline="", encoding="utf-8") as f:
            reader = csv.DictReader(f)
            if reader.fieldnames == TIMES_COLUMNS:
                return
            rows = list(reader) if reader.fieldnames else []
    except FileNotFoundError:
        return
    with open(TIMES_PATH, "w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=TIMES_COLUMNS)
        w.writeheader()
        for r in rows:
            w.writerow({k: (r.get(k, "") or "") for k in TIMES_COLUMNS})

def ensure_times_csv():
    if not os.path.exists(TIMES_PATH):
        with open(TIMES_PATH, "w", newline="") as f:
            csv.writer(f).writerow(TIMES_COLUMNS)
    else:
        _migrate_times_csv()

def count_rows():
    ensure_csv()
    with open(CSV_PATH, newline="") as f:
        return max(0, sum(1 for _ in f) - 1)

def count_times():
    ensure_times_csv()
    with open(TIMES_PATH, newline="") as f:
        return max(0, sum(1 for _ in f) - 1)

def read_csv_json(path, columns):
    if not os.path.exists(path):
        return []
    rows = []
    with open(path, newline="", encoding="utf-8") as f:
        reader = csv.DictReader(f)
        for r in reader:
            # normalize: keep as strings, convert numeric where possible
            rows.append(dict(r))
    return rows

def read_tlx_rows():
    ensure_csv()
    return read_csv_json(CSV_PATH, COLUMNS)

def read_times_rows():
    ensure_times_csv()
    return read_csv_json(TIMES_PATH, TIMES_COLUMNS)


class Handler(BaseHTTPRequestHandler):
    server_version = "TLXCollector/1.0"

    def log_message(self, *args):
        pass

    def _send(self, code, body, ctype="text/plain; charset=utf-8"):
        data = body.encode() if isinstance(body, str) else body
        self.send_response(code)
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(data)))
        # simple CORS for tailnet fetch
        self.send_header("Access-Control-Allow-Origin", "*")
        self.send_header("Access-Control-Allow-Methods", "GET, POST, DELETE, OPTIONS")
        self.send_header("Access-Control-Allow-Headers", "Content-Type")
        self.end_headers()
        self.wfile.write(data)

    def _send_json(self, code, obj):
        self._send(code, json.dumps(obj, ensure_ascii=False), "application/json; charset=utf-8")

    def do_OPTIONS(self):
        self.send_response(204)
        self.send_header("Access-Control-Allow-Origin", "*")
        self.send_header("Access-Control-Allow-Methods", "GET, POST, DELETE, OPTIONS")
        self.send_header("Access-Control-Allow-Headers", "Content-Type")
        self.send_header("Content-Length", "0")
        self.end_headers()

    def do_GET(self):
        parsed = urlparse(self.path)
        path = parsed.path
        qs = parse_qs(parsed.query)
        if path in ("/", "/index.html"):
            with open(FORM, "rb") as f:
                self._send(200, f.read(), "text/html; charset=utf-8")
        elif path in ("/tempos", "/tempos.html", "/tempos-teleop.html", "/times", "/times.html"):
            if not os.path.exists(TEMPOS):
                self._send(404, "tempos-teleop.html not found - deploy it first")
                return
            with open(TEMPOS, "rb") as f:
                self._send(200, f.read(), "text/html; charset=utf-8")
        elif path == "/count":
            self._send(200, json.dumps({"submissions": count_rows()}),
                       "application/json")
        elif path == "/api/count":
            self._send_json(200, {"submissions": count_rows(), "times": count_times()})
        elif path == "/api/tlx":
            rows = read_tlx_rows()
            self._send_json(200, rows)
        elif path == "/api/times":
            rows = read_times_rows()
            # optional filter by query params
            def match(r):
                for k in ("operator", "date", "condition", "task", "obstacle", "attempt",
                          "outcome", "contact"):
                    vals = qs.get(k)
                    if vals and r.get(k, "").strip() != vals[0]:
                        return False
                return True
            if qs:
                rows = [r for r in rows if match(r)]
            self._send_json(200, rows)
        elif path == "/api/consolidated":
            tlx_rows = read_tlx_rows()
            times_rows = read_times_rows()
            # build index for TLX: key = operator|date|condition|task normalized lower-trim
            def key_tlx(r):
                return (
                    r.get("operator","").strip().lower(),
                    r.get("date","").strip().lower(),
                    r.get("condition","").strip().lower(),
                    r.get("task","").strip().lower(),
                )
            # group tlx by key, keep latest timestamp per key
            tlx_by_key = {}
            for r in tlx_rows:
                k = key_tlx(r)
                # keep latest (overwrites, csv is appended chronologically)
                tlx_by_key[k] = r
            out = []
            for t in times_rows:
                k = (
                    t.get("operator","").strip().lower(),
                    t.get("date","").strip().lower(),
                    t.get("condition","").strip().lower(),
                    t.get("task","").strip().lower(),
                )
                tlx = tlx_by_key.get(k)
                merged = dict(t)
                if tlx:
                    for col in COLUMNS:
                        if col not in ("timestamp","operator","date","condition","task"):
                            merged[col] = tlx.get(col, "")
                    merged["tlx_timestamp"] = tlx.get("timestamp","")
                    merged["tlx_matched"] = True
                else:
                    for col in COLUMNS:
                        if col not in ("timestamp","operator","date","condition","task"):
                            merged[col] = ""
                    merged["tlx_timestamp"] = ""
                    merged["tlx_matched"] = False
                out.append(merged)
            # also allow filtering
            if qs:
                def match2(r):
                    for k in ("operator","date","condition","task","obstacle","attempt",
                              "outcome","contact"):
                        vals = qs.get(k)
                        if vals and r.get(k,"").strip() != vals[0]:
                            return False
                    return True
                out = [r for r in out if match2(r)]
            self._send_json(200, out)
        elif path == "/api/times.csv":
            ensure_times_csv()
            with open(TIMES_PATH, "rb") as f:
                data = f.read()
            self.send_response(200)
            self.send_header("Content-Type", "text/csv; charset=utf-8")
            self.send_header("Content-Disposition", "attachment; filename=times.csv")
            self.send_header("Content-Length", str(len(data)))
            self.send_header("Access-Control-Allow-Origin", "*")
            self.end_headers()
            self.wfile.write(data)
        elif path == "/api/consolidated.csv":
            # generate consolidated CSV on the fly
            tlx_rows = read_tlx_rows()
            times_rows = read_times_rows()
            def key_tlx(r):
                return (
                    r.get("operator","").strip().lower(),
                    r.get("date","").strip().lower(),
                    r.get("condition","").strip().lower(),
                    r.get("task","").strip().lower(),
                )
            tlx_by_key = {}
            for r in tlx_rows:
                tlx_by_key[key_tlx(r)] = r
            import io
            out_io = io.StringIO()
            w = csv.writer(out_io)
            header = TIMES_COLUMNS + [c for c in COLUMNS if c not in ("timestamp","operator","date","condition","task")] + ["tlx_timestamp","tlx_matched"]
            w.writerow(header)
            for t in times_rows:
                k = (
                    t.get("operator","").strip().lower(),
                    t.get("date","").strip().lower(),
                    t.get("condition","").strip().lower(),
                    t.get("task","").strip().lower(),
                )
                tlx = tlx_by_key.get(k)
                row = [t.get(c,"") for c in TIMES_COLUMNS]
                if tlx:
                    row += [tlx.get(c,"") for c in COLUMNS if c not in ("timestamp","operator","date","condition","task")]
                    row += [tlx.get("timestamp",""), "1"]
                else:
                    row += ["" for c in COLUMNS if c not in ("timestamp","operator","date","condition","task")]
                    row += ["", "0"]
                w.writerow(row)
            data = out_io.getvalue().encode("utf-8")
            self.send_response(200)
            self.send_header("Content-Type", "text/csv; charset=utf-8")
            self.send_header("Content-Disposition", "attachment; filename=consolidado_tempos_tlx.csv")
            self.send_header("Content-Length", str(len(data)))
            self.send_header("Access-Control-Allow-Origin", "*")
            self.end_headers()
            self.wfile.write(data)
        elif path in ("/submissions.csv", "/api/submissions.csv"):
            ensure_csv()
            with open(CSV_PATH, "rb") as f:
                data = f.read()
            self.send_response(200)
            self.send_header("Content-Type", "text/csv; charset=utf-8")
            self.send_header("Content-Disposition", "attachment; filename=submissions.csv")
            self.send_header("Content-Length", str(len(data)))
            self.send_header("Access-Control-Allow-Origin", "*")
            self.end_headers()
            self.wfile.write(data)
        else:
            self._send(404, "not found")

    def do_POST(self):
        path = urlparse(self.path).path
        if path == "/submit":
            try:
                length = int(self.headers.get("Content-Length", 0))
                body = json.loads(self.rfile.read(length) or b"{}")
            except Exception:
                return self._send(400, "bad json")
            ratings = body.get("ratings") or {}
            if not body.get("operator") or any(ratings.get(k) is None
                                               for k in SCALES):
                return self._send(422, "need operator + all 6 ratings")
            weights = body.get("weights") or {}

            def num(x):
                return x if isinstance(x, (int, float)) else ""

            row = [datetime.now(timezone.utc).strftime("%Y-%m-%dT%H:%M:%S.%f")[:-3]+"Z",
                   str(body.get("operator", ""))[:64],
                   str(body.get("date", ""))[:32],
                   str(body.get("condition", ""))[:64],
                   str(body.get("task", ""))[:64]]
            row += [num(ratings.get(k)) for k in SCALES]
            row += [num(body.get("raw_tlx"))]
            row += [num(weights.get(k)) for k in SCALES]
            row += [num(body.get("weighted_tlx"))]
            ensure_csv()
            with open(CSV_PATH, "a", newline="") as f:
                csv.writer(f).writerow(row)
            self._send(200, json.dumps({"ok": True, "n": count_rows()}),
                       "application/json")
            return

        if path == "/api/times":
            try:
                length = int(self.headers.get("Content-Length", 0))
                body = json.loads(self.rfile.read(length) or b"{}")
            except Exception:
                return self._send(400, "bad json")
            # validate
            operator = str(body.get("operator","")).strip()
            duration_s = body.get("duration_s")
            attempt = str(body.get("attempt","")).strip()
            obstacle = str(body.get("obstacle","")).strip().lower()
            if not operator:
                return self._send(422, "need operator")
            # duration_s must be number >0
            try:
                duration_s = float(duration_s)
            except Exception:
                return self._send(422, "need duration_s numeric")
            if not (0 < duration_s < 36000):
                return self._send(422, "duration_s out of range")
            if obstacle not in OBSTACLES:
                # also accept some aliases
                alias = {"livre":"sem_obstaculo","sem obstáculo":"sem_obstaculo","sem_obstaculo":"sem_obstaculo",
                         "com obstáculo":"com_obstaculo","com_obstaculo":"com_obstaculo","com obstaculo":"com_obstaculo"}
                obstacle = alias.get(obstacle, obstacle)
                if obstacle not in OBSTACLES:
                    return self._send(422, "need obstacle sem_obstaculo|com_obstaculo")
            if not attempt:
                attempt = "1"
            # outcome: success | failure (aliases accepted, default success)
            outcome = str(body.get("outcome", "success") or "success").strip().lower()
            outcome_alias = {"success": "success", "ok": "success", "sucesso": "success",
                             "s": "success", "1": "success",
                             "failure": "failure", "fail": "failure", "falha": "failure",
                             "f": "failure", "0": "failure", "error": "failure"}
            outcome = outcome_alias.get(outcome, outcome)
            if outcome not in OUTCOMES:
                return self._send(422, "need outcome success|failure")
            # contact with obstacle: yes | no + count
            contact = str(body.get("contact", "no") or "no").strip().lower()
            contact_alias = {"yes": "yes", "y": "yes", "1": "yes", "true": "yes",
                             "sim": "yes", "s": "yes", "tocou": "yes", "contato": "yes",
                             "no": "no", "n": "no", "0": "no", "false": "no",
                             "nao": "no", "não": "no", "sem": "no", "none": "no"}
            contact = contact_alias.get(contact, contact)
            if contact not in CONTACTS:
                return self._send(422, "need contact yes|no")
            try:
                contact_count = int(body.get("contact_count", 0) or 0)
            except Exception:
                return self._send(422, "need contact_count integer")
            if not (0 <= contact_count <= 99):
                return self._send(422, "contact_count out of range")
            if contact_count > 0:
                contact = "yes"
            # sanitize
            row = [datetime.now(timezone.utc).strftime("%Y-%m-%dT%H:%M:%S.%f")[:-3]+"Z",
                   operator[:64],
                   str(body.get("date","")).strip()[:32],
                   str(body.get("condition","")).strip()[:64],
                   str(body.get("task","")).strip()[:64],
                   attempt[:16],
                   obstacle,
                   f"{duration_s:.3f}",
                   outcome,
                   contact,
                   str(contact_count),
                   str(body.get("notes","")).strip()[:256]]
            ensure_times_csv()
            with open(TIMES_PATH, "a", newline="") as f:
                csv.writer(f).writerow(row)
            self._send_json(200, {"ok": True, "n": count_times(), "row": dict(zip(TIMES_COLUMNS, row))})
            return

        if path == "/api/times/delete":
            try:
                length = int(self.headers.get("Content-Length", 0))
                body = json.loads(self.rfile.read(length) or b"{}")
            except Exception:
                return self._send(400, "bad json")
            ts = str(body.get("timestamp","")).strip()
            if not ts:
                return self._send(422, "need timestamp")
            ensure_times_csv()
            rows = read_times_rows()
            kept = [r for r in rows if r.get("timestamp") != ts]
            if len(kept) == len(rows):
                return self._send(404, "not found")
            with open(TIMES_PATH, "w", newline="") as f:
                w = csv.DictWriter(f, fieldnames=TIMES_COLUMNS)
                w.writeheader()
                for r in kept:
                    w.writerow({k: r.get(k,"") for k in TIMES_COLUMNS})
            self._send_json(200, {"ok": True, "n": len(kept)})
            return

        return self._send(404, "not found")

    def do_DELETE(self):
        # allow DELETE /api/times?timestamp=...
        parsed = urlparse(self.path)
        if parsed.path == "/api/times":
            qs = parse_qs(parsed.query)
            ts = (qs.get("timestamp") or [""])[0]
            if not ts:
                return self._send(422, "need ?timestamp=")
            ensure_times_csv()
            rows = read_times_rows()
            kept = [r for r in rows if r.get("timestamp") != ts]
            if len(kept) == len(rows):
                return self._send(404, "not found")
            import csv as _csv
            with open(TIMES_PATH, "w", newline="") as f:
                w = _csv.DictWriter(f, fieldnames=TIMES_COLUMNS)
                w.writeheader()
                for r in kept:
                    w.writerow({k: r.get(k,"") for k in TIMES_COLUMNS})
            self._send_json(200, {"ok": True, "n": len(kept)})
            return
        return self._send(404, "not found")


if __name__ == "__main__":
    port = int(sys.argv[1]) if len(sys.argv) > 1 else 8901
    bind = sys.argv[2] if len(sys.argv) > 2 else "0.0.0.0"
    ensure_csv()
    ensure_times_csv()
    server = ThreadingHTTPServer((bind, port), Handler)
    print(f"TLX collector on {bind}:{port} -> {CSV_PATH} + {TIMES_PATH}", flush=True)
    server.serve_forever()
