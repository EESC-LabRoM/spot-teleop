#!/usr/bin/env python3
"""Guided ZED -> FR3 movement protocol on the real robot (same 12 movements as the RealSense protocol).
Per movement: ENTER starts the recording (perception domain 8 + robot domain 77, record_zed_teleop.sh), you click Start teleop ZED,
make the movement, click Stop; the recording stops and the robot goes home (/teleop/home) before the next one.
Everything lands in ~/spot-teleop/bags/zed_protocol_<stamp>/<NN_movement>/{d8,d77,events.jsonl} plus session.json.
  python3 zed_protocol.py [--from 04] [--only 04_frente_tras,05_direita_esquerda] [--no-home]
Ctrl+C stops and saves the active recording (the robot is not moved).
"""
import argparse, json, os, subprocess, sys, time
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent.parent / "realsense"))
from motion_protocol import PROTOCOL  # noqa: E402

REC = HERE.parent / "record_zed_teleop.sh"; BAGS = Path.home() / "spot-teleop/bags"
ROS = ("source /opt/ros/jazzy/setup.bash; source /ros2_ws/install/setup.bash; "
       "export ROS_DOMAIN_ID=77 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST; cd /ros2_ws; ")

ap = argparse.ArgumentParser(); ap.add_argument("--from", dest="start", default=None); ap.add_argument("--only", default=None)
ap.add_argument("--no-home", action="store_true"); a = ap.parse_args()
stages = PROTOCOL
if a.only: stages = [s for s in stages if s["id"] in a.only.split(",")]
if a.start: stages = [s for s in stages if s["id"] >= a.start]
if not stages: sys.exit("nenhuma etapa selecionada")

subprocess.run(["docker", "cp", str(HERE / "robot_side.py"), "fr3-ros2:/tmp/zed_protocol_robot_side.py"], check=True)
session = "zed_protocol_" + time.strftime("%Y%m%d_%H%M%S"); sdir = BAGS / session; sdir.mkdir(parents=True)

def sh(cmd): return subprocess.run(["bash", "-c", cmd], capture_output=True, text=True).stdout.strip()
def robot(mode, log, timeout):
    """Runs robot_side.py, echoes and logs its JSON events; returns (exit code, events)."""
    p = subprocess.Popen(["docker", "exec", "fr3-ros2", "bash", "-c", ROS + f"python3 /tmp/zed_protocol_robot_side.py {mode} --timeout {timeout}"],
                         stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, text=True)
    events = []
    for line in p.stdout:
        try: e = json.loads(line)
        except ValueError: continue
        events.append(e); log.write(json.dumps(dict(e, step=mode)) + "\n"); log.flush()
        ev = e["event"]
        if ev == "waiting_start": print("\a>>> Clique  Start teleop ZED  no painel e faça o movimento. Clique  Stop  quando terminar.", flush=True)
        elif ev == "engaged": print("    ENGAJADO — gravando o movimento", flush=True)
        elif ev == "note" and e.get("note"): print("    ·", e["note"], flush=True)
        elif ev == "disengaged": print("    parado", flush=True)
        elif ev == "home_started": print("    robô voltando para a home…", flush=True)
        elif ev in ("home_done", "home_timeout", "home_refused", "error", "timeout"): print("   ", ev, e.get("result") or e.get("msg") or "", flush=True)
    return p.wait(), events

teleop = sh("docker exec fr3-ros2 bash -c 'for p in $(pgrep -f \"^python3 /lab/fr3_zed_runtime/fr3_input_teleop\"); do tr \"\\0\" \" \" < /proc/$p/cmdline; echo; done'")
ctrls = sh(f"docker exec fr3-ros2 bash -c '{ROS} timeout 30 ros2 control list_controllers 2>/dev/null' | sed 's/\\x1b\\[[0-9;]*m//g'")
meta = dict(session=session, started=time.time(), teleop_command=teleop, controllers=ctrls,
            git=sh(f"git -C {HERE} rev-parse --short HEAD"), stages=[])
print(f"ROTEIRO ZED → FR3 ({len(stages)} movimentos). Sessão: {sdir}\n"
      "Cada movimento: ENTER grava, você clica Start teleop ZED, faz o movimento, clica Stop; o robô volta para a home.\n"
      "Ctrl+C interrompe e salva a gravação ativa (não move o robô).\n", flush=True)
recording = False
try:
    for s in stages:
        print("\n" + s["title"].upper() + f" — ~{s['seconds']} s\n" + s["instruction"], flush=True)
        input("Enquadre ombro, cotovelo e mão direita na ZED do operador e pressione ENTER: ")
        rel = f"{session}/{s['id']}"; (BAGS / rel).mkdir(parents=True, exist_ok=True)
        out = sh(f"REC_DIR={rel} bash {REC} start"); recording = True
        print("    gravando:", BAGS / rel, flush=True)
        st = dict(id=s["id"], dir=str(BAGS / rel), rec_start=time.time())
        with open(BAGS / rel / "events.jsonl", "w") as log:
            code, ev = robot("cycle", log, 600)
            time.sleep(1.0)                                         # keep the hold after Stop in the bag
            print("   ", sh(f"bash {REC} stop").replace("\n", " | "), flush=True); recording = False
            st.update(rec_stop=time.time(), cycle_exit=code,
                      engaged=next((e["t"] for e in ev if e["event"] == "engaged"), None),
                      disengaged=next((e["t"] for e in ev if e["event"] == "disengaged"), None))
            if not a.no_home and code == 0:
                hc, hev = robot("home", log, 120); st.update(home_exit=hc, home=[e for e in hev if e["event"].startswith("home")])
                if hc != 0: input("    Home não confirmou. Verifique o robô e pressione ENTER para continuar (Ctrl+C sai): ")
        meta["stages"].append(st); json.dump(meta, open(sdir / "session.json", "w"), indent=1)
except KeyboardInterrupt:
    print("\nInterrompido.", flush=True)
    if recording: print(sh(f"bash {REC} stop"), flush=True)
meta["finished"] = time.time(); json.dump(meta, open(sdir / "session.json", "w"), indent=1)
print("\nSalvo em", sdir, flush=True); print(sh(f"du -sh {sdir}/*"), flush=True)
