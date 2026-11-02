#!/usr/bin/env python3
"""Web page for the ZED -> FR3 movement protocol on the real robot (same 12 movements as the RealSense page, 8794).
Pick a movement, click "Gravar e iniciar": both recordings start (perception domain 8 + robot domain 77, record_zed_teleop.sh) and
the ZED teleop calibrates/engages as with "Start teleop ZED". Click "Parar" (or Stop in the lab dashboard): the recording stops
1 s later and the robot goes home (/teleop/home). The next movement is selected automatically.
Saved in ~/spot-teleop/bags/zed_protocol_<stamp>/<NN_movement>/{d8,d77,events.jsonl,result.json} + session.json.
  python3 zed_protocol_server.py [--port 8797] [--session zed_protocol_<stamp>]  ->  http://localhost:8797/  (and the Tailscale address)
"""
import argparse, json, subprocess, sys, threading, time, urllib.request
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import urlsplit

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent.parent / "realsense"))
from motion_protocol import PROTOCOL  # noqa: E402

REC = HERE.parent / "record_zed_teleop.sh"; BAGS = Path.home() / "spot-teleop/bags"
ROS = ("source /opt/ros/jazzy/setup.bash; source /ros2_ws/install/setup.bash; "
       "export ROS_DOMAIN_ID=77 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST; cd /ros2_ws; ")


def sh(cmd, timeout=60):
    return subprocess.run(["bash", "-c", cmd], capture_output=True, text=True, timeout=timeout).stdout.strip()


class Session:
    def __init__(self, resume=None):
        self.name = resume or "zed_protocol_" + time.strftime("%Y%m%d_%H%M%S"); self.dir = BAGS / self.name; self.dir.mkdir(parents=True, exist_ok=bool(resume))
        self.lock = threading.Lock(); self.robot = {}; self.rec = None; self.busy = None; self.msg = ""; self.saved = []
        self.proc = None; self.log = None
        teleop = sh("docker exec fr3-ros2 bash -c 'for p in $(pgrep -f \"^python3 /lab/fr3_zed_runtime/fr3_input_teleop\"); do tr \"\\0\" \" \" < /proc/$p/cmdline; echo; done'")
        self.meta = dict(session=self.name, started=time.time(), teleop_command=teleop, git=sh(f"git -C {HERE} rev-parse --short HEAD"), stages=[])
        if resume and (self.dir / "session.json").exists():          # continue a session: keep its stages (and the saved list)
            old = json.load(open(self.dir / "session.json")); self.meta["stages"] = old.get("stages", []); self.saved = list(self.meta["stages"])
            self.meta.setdefault("resumed", []).append(time.time())
        self.save_meta()
        subprocess.run(["docker", "cp", str(HERE / "robot_side.py"), "fr3-ros2:/tmp/zed_protocol_robot_side.py"], check=True)
        threading.Thread(target=self.robot_link, daemon=True).start()
        threading.Thread(target=self.watch, daemon=True).start()

    def save_meta(self):
        json.dump(self.meta, open(self.dir / "session.json", "w"), indent=1, ensure_ascii=False)

    def robot_link(self):                       # persistent helper inside fr3-ros2: status in, commands out; restarted if it dies
        while True:
            self.proc = subprocess.Popen(["docker", "exec", "-i", "fr3-ros2", "bash", "-c", ROS + "exec python3 /tmp/zed_protocol_robot_side.py serve"],
                                         stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, text=True, bufsize=1)
            for line in self.proc.stdout:
                try: e = json.loads(line)
                except ValueError: continue
                with self.lock:
                    if e["event"] == "status":
                        prev = self.robot; self.robot = e
                        if self.log and (prev.get("note"), prev.get("phase"), prev.get("engaged")) != (e.get("note"), e.get("phase"), e.get("engaged")):
                            self.log.write(json.dumps(e, ensure_ascii=False) + "\n"); self.log.flush()
                    else:
                        if self.log: self.log.write(json.dumps(e, ensure_ascii=False) + "\n"); self.log.flush()
                        if e["event"] == "reply" and not e.get("ok"): self.msg = f"{e['cmd']}: {e.get('msg')}"
            time.sleep(1.0)

    def send(self, cmd):
        try: self.proc.stdin.write(cmd + "\n"); self.proc.stdin.flush()
        except Exception as e: self.msg = f"robot link: {e}"

    def start(self, label):
        stage = next((s for s in PROTOCOL if s["id"] == label), None)
        with self.lock:
            if stage is None: return 400, "movimento desconhecido"
            if self.rec or self.busy: return 409, "já gravando / ocupado"
            if self.robot.get("home_running"): return 409, "robô voltando para a home: espere"
            if self.robot.get("engaged"): return 409, "teleop engajado: clique Parar antes"
            self.busy = "iniciando gravação…"
        n = sum(1 for s in self.meta["stages"] if s["id"] == label)
        rel = f"{self.name}/{label}" + (f"_{n + 1}" if n else ""); (BAGS / rel).mkdir(parents=True, exist_ok=True)
        out = sh(f"REC_DIR={rel} bash {REC} start", timeout=40)
        with self.lock:
            if "recording" not in out: self.busy = None; self.msg = "falha ao iniciar a gravação: " + out[-300:]; return 500, self.msg
            self.log = open(BAGS / rel / "events.jsonl", "w")
            self.rec = dict(id=label, title=stage["title"], dir=str(BAGS / rel), rel=rel, t0=time.time(), engaged_at=None, stopping=False)
            self.busy = None; self.msg = "gravando: siga a instrução da calibração e faça o movimento"
        self.send("zed_start")
        return 200, "gravando"

    def stop(self, reason="botão Parar"):
        with self.lock:
            if not self.rec or self.rec["stopping"]: return 409, "nenhuma gravação ativa"
            self.rec["stopping"] = True; self.rec["stop_reason"] = reason; self.busy = "salvando…"
        self.send("stop")
        threading.Thread(target=self.finish, daemon=True).start()
        return 200, "parando"

    def finish(self):
        time.sleep(1.0)                                              # keep the hold after Stop in the bag
        out = sh(f"bash {REC} stop", timeout=90)
        with self.lock:
            r = self.rec; r.update(t1=time.time(), stop_output=out); engaged = r["engaged_at"] is not None
            self.busy = "robô voltando para a home…" if engaged else None
        home = None
        for attempt in range(3 if engaged else 0):   # the home script gives up when the loaded controller_manager answers late: retry
            with self.lock: self.busy = "robô voltando para a home…" + (f" (tentativa {attempt + 1}/3)" if attempt else "")
            self.send("home"); t = time.time()
            while time.time() - t < 3 and not self.robot.get("home_running"): time.sleep(0.1)
            while time.time() - t < 120 and self.robot.get("home_running"): time.sleep(0.2)
            time.sleep(0.5); home = (self.robot.get("home_result") or "").replace("\x1b[0m", "")
            if "conclu" in home.lower() or "timed out" not in home: break
        with self.lock:
            r["home_result"] = home
            sizes = sh(f"du -sh {r['dir']}/d8 {r['dir']}/d77 2>/dev/null").replace("\n", " · ")
            res = {k: r[k] for k in ("id", "title", "dir", "t0", "t1", "engaged_at", "stop_reason", "home_result")}; res["sizes"] = sizes
            json.dump(res, open(Path(r["dir"]) / "result.json", "w"), indent=1, ensure_ascii=False)
            self.meta["stages"].append(res); self.save_meta(); self.saved.append(res)
            if self.log: self.log.close(); self.log = None
            self.rec = None; self.busy = None
            self.msg = f"salvo {r['id']} ({sizes})" + (f" · {home}" if home else " · nunca engajou: robô não foi movido")

    def watch(self):                             # Stop from the lab dashboard (or teleop drop) also ends the movement
        while True:
            time.sleep(0.1)
            with self.lock:
                r = self.rec; eng = self.robot.get("engaged")
                if not r or r["stopping"]: continue
                if eng and r["engaged_at"] is None: r["engaged_at"] = time.time()
                ended = r["engaged_at"] is not None and not eng
            if ended: self.stop("teleop desengajado (Stop no painel ou perda)")

    def status(self):
        with self.lock:
            r = self.rec
            return dict(session=str(self.dir), robot=self.robot, busy=self.busy, msg=self.msg, protocol=PROTOCOL,
                        recording=None if not r else dict(id=r["id"], title=r["title"], elapsed_s=round(time.time() - r["t0"], 1),
                                                          engaged=r["engaged_at"] is not None, stopping=r["stopping"]),
                        saved=[dict(id=s["id"], sizes=s["sizes"], home=s["home_result"], engaged=s["engaged_at"] is not None) for s in self.saved])


PAGE = r'''<!doctype html><html lang="pt-BR"><meta charset="utf-8"><meta name="viewport" content="width=device-width"><title>ZED · movimentos no FR3</title>
<style>body{background:#10171c;color:#e4edf5;font:16px system-ui;margin:24px auto;max-width:1300px;padding:0 18px}h1{font-size:25px}.grid{display:grid;grid-template-columns:1fr 1fr;gap:12px}img{width:100%;min-height:140px;background:#060809;border-radius:10px}button,select{padding:12px;border:1px solid #568069;border-radius:8px;background:#20352b;color:white;font:inherit;margin:5px 0}button:disabled{opacity:.4}#stop{background:#4a2020;border-color:#a05050}select{width:100%}.card{padding:16px;margin-top:16px;border:1px solid #304238;border-radius:12px}#state{white-space:pre-wrap;font:14px monospace;color:#b5e2c0;min-height:7.5em}#instruction{font-size:20px;min-height:3em}#feedback{color:#ffc57e;min-height:1.4em}small{color:#a8b8c4}.rec{color:#ff6b6b;font-weight:bold}ul{margin:4px 0;padding-left:20px}@media(max-width:700px){.grid{grid-template-columns:1fr}}</style>
<h1>ZED · movimentos no FR3 real</h1><small>Cada movimento grava duas bags (percepção + robô). Parar salva e manda o robô para a home. Fique com alguém no botão de emergência.</small>
<div class="grid"><div><h3>Mão do operador (ZED3)</h3><img id="hand"></div><div><h3>Robô (ZED1 frontal)</h3><img id="robot"></div></div>
<div class="card"><h2>Escolha o movimento</h2><select id="motion"></select><p id="instruction"></p>
<p>Comece com 2–3 s de mão parada, faça 3 ciclos lentos e termine parado. Nos giros, punho no mesmo ponto.</p>
<button id="start">● Gravar e iniciar teleop ZED</button> <button id="stop">■ Parar, salvar e voltar à home</button><p id="feedback"></p></div>
<div class="card"><pre id="state">Conectando ao robô…</pre></div>
<div class="card"><h3>Gravados nesta sessão</h3><ul id="saved"><li>nenhum ainda</li></ul></div>
<script>const $=id=>document.getElementById(id);let protocol=[],busy=false,lastSaved=0;
async function action(action){if(busy)return;busy=true;try{const r=await fetch('/control',{method:'POST',headers:{'Content-Type':'application/json','X-Zed-Protocol':'1'},body:JSON.stringify({action,label:$('motion').value})});const d=await r.json();$('feedback').textContent=d.msg}catch(e){$('feedback').textContent=e.message}finally{busy=false}}
$('start').onclick=()=>action('start');$('stop').onclick=()=>action('stop');$('motion').onchange=()=>{const m=protocol.find(p=>p.id===$('motion').value);$('instruction').textContent=m?.instruction||'';$('start').textContent='● Gravar «'+(m?.title||'')+'» e iniciar teleop ZED'};
async function poll(){try{const s=await(await fetch('/status')).json();if(!protocol.length){protocol=s.protocol;for(const p of protocol){const o=document.createElement('option');o.value=p.id;o.textContent=p.title+' · ~'+p.seconds+' s';$('motion').append(o)}$('motion').onchange()}
const r=s.robot||{},rec=s.recording;
$('state').innerHTML=(rec?'<span class="rec">● GRAVANDO '+rec.title+' · '+rec.elapsed_s+' s'+(rec.engaged?' · robô seguindo a mão':' · aguardando calibração/engajar')+'</span>':'Sem gravação ativa')+(s.busy?'\n'+s.busy:'')
+'\nTeleop: '+(r.engaged?'ENGAJADO':'parado')+' · fase '+(r.phase||'—')+'\n'+(r.note||'')+'\nMão: '+(r.tracking?'rastreada':'não rastreada')+' · orientação '+(r.orientation_valid?'válida':'—')+(r.gesture?' · '+r.gesture:'')+'\n'+(r.input||'')
+'\nPonta do robô (m): '+(r.tcp?r.tcp.join(', '):'—')+(r.home_running?'\nHOME em andamento…':'');
if(s.msg)$('feedback').innerHTML=/falhou/i.test(s.msg)?'<b style="color:#ff6b6b">⚠ '+s.msg+' — use Home no painel antes do próximo</b>':s.msg;
$('start').disabled=busy||!!rec||!!s.busy||r.home_running||r.engaged;$('stop').disabled=busy||!rec||rec.stopping;
if(s.saved.length){$('saved').innerHTML=s.saved.map(x=>'<li>✓ '+x.id+' · '+x.sizes+' · '+(x.engaged?(x.home||''):'nunca engajou')+'</li>').join('');
 if(s.saved.length>lastSaved){lastSaved=s.saved.length;const i=protocol.findIndex(p=>p.id===s.saved[s.saved.length-1].id);if(i>=0&&i+1<protocol.length){$('motion').value=protocol[i+1].id;$('motion').onchange()}}}}catch(e){$('state').textContent=e.message}}
const feeds={hand:()=>'/hand.jpg?_='+Date.now(),robot:()=>'/robot.jpg?_='+Date.now()};
for(const k in feeds){const im=$(k);const next=()=>setTimeout(()=>im.src=feeds[k](),k==='hand'?60:250);im.onload=next;im.onerror=()=>setTimeout(next,1000);next()}
setInterval(poll,400);poll();</script></html>'''


FEEDS = {"/hand.jpg": "http://127.0.0.1:8786/snapshot",
         "/robot.jpg": "http://100.88.134.127:8787/snapshot?topic=/zed1/zed_node/left/image_rect_color&quality=60"}


def handler(sess):
    class H(BaseHTTPRequestHandler):
        def log_message(self, *a): pass
        def send(self, code, data, kind):
            self.send_response(code); self.send_header("Content-Type", kind); self.send_header("Cache-Control", "no-store"); self.end_headers(); self.wfile.write(data)
        def do_GET(self):
            path = urlsplit(self.path).path
            if path == "/": return self.send(200, PAGE.encode(), "text/html; charset=utf-8")
            if path == "/status": return self.send(200, json.dumps(sess.status(), ensure_ascii=False).encode(), "application/json")
            if path in FEEDS:                        # the images come through this page's own port: the browser may only reach 8797
                try:
                    with urllib.request.urlopen(FEEDS[path], timeout=3) as r: return self.send(200, r.read(), "image/jpeg")
                except Exception: return self.send(503, b"", "image/jpeg")
            self.send(404, b"{}", "application/json")
        def do_POST(self):
            origin = self.headers.get("Origin")
            if self.path != "/control" or self.headers.get("X-Zed-Protocol") != "1" or (origin and urlsplit(origin).netloc != self.headers.get("Host")):
                return self.send(403, b'{"msg":"forbidden"}', "application/json")
            try: d = json.loads(self.rfile.read(int(self.headers.get("Content-Length", 0))))
            except ValueError: return self.send(400, b'{"msg":"bad json"}', "application/json")
            code, msg = sess.start(d.get("label")) if d.get("action") == "start" else sess.stop() if d.get("action") == "stop" else (400, "ação desconhecida")
            self.send(code, json.dumps(dict(msg=msg), ensure_ascii=False).encode(), "application/json")
    return H


if __name__ == "__main__":
    ap = argparse.ArgumentParser(); ap.add_argument("--port", type=int, default=8797); ap.add_argument("--bind", default="127.0.0.1,100.88.134.127")
    ap.add_argument("--session", default=None, help="continue this session folder (zed_protocol_<stamp>) instead of starting a new one")
    a = ap.parse_args(); sess = Session(a.session)
    for host in a.bind.split(","):
        srv = ThreadingHTTPServer((host, a.port), handler(sess)); srv.daemon_threads = True
        threading.Thread(target=srv.serve_forever, daemon=True).start()
    print(f"ZED protocol page: http://localhost:{a.port}/  · session {sess.dir}", flush=True)
    while True: time.sleep(3600)
