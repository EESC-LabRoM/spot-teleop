"""Real-time SAM2 segmentation viewer (VM103, domain 8).

One process does everything:
  - subscribes ZED1 rgb (/vm/cable/rgb) + SAM2 mask (/zed1/segmentation_mask),
  - publishes annotated /zed1/segmentation_overlay (rgb8, BEST_EFFORT),
  - serves HTTP on :8791 with /seg (viewer page), /seg.mjpeg (live stream),
    /stats.json (coverage/age/fps), /drill.json (live drill 3D box in
    fr3_link0 from tracker mask + depth, for the dashboard map overlay),
    /grasp_target.json (static pick/TCP marker for the same overlay).

Run as its own container (vm-grasp-loop image, host net, repo mount):
  docker run -d --name vm-viewer --net host --ipc host \\
    -e ROS_DOMAIN_ID=8 -e RMW_IMPLEMENTATION=rmw_fastrtps_cpp \\
    -e ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET -e PYTHONUNBUFFERED=1 \\
    -v /home/nexus/spot-teleop:/home/nexus/spot-teleop:rw \\
    vm-grasp-loop:latest bash -c 'pip -q install matplotlib trimesh pycollada >/dev/null 2>&1; \\
    source /opt/ros/humble/setup.bash && \\
    exec python3 /home/nexus/spot-teleop/fr3_real_repro/vm_offload/seg_view.py'
View from the lab: http://10.0.0.3:8791/ (index: /seg + /grasp)
"""
import json
import os
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import parse_qs, urlparse

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import grasp_render  # noqa: E402

RUNS = '/home/nexus/spot-teleop/fr3_real_repro/grasp_loop/runs'
TARGET_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                           'grasp_target.json')
_grasp_cache = {}  # run_id -> (expires_at, png, meta)
_GRASP_TTL = 120.0
_grasp_json_cache = {}  # run_id -> (expires_at, payload_dict)
MERGED_NPZ = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                          '..', 'zed2_align', 'merged_z1z2.npz')
_merge_json_cache = {}  # 'payload' -> (mtime, json)


def _merge_json():
    """Decimated dual-ZED merged cloud for /merge.json (static file, cached)."""
    try:
        mt = os.path.getmtime(MERGED_NPZ)
    except OSError as e:
        raise FileNotFoundError(f'no merged cloud: {e}')
    hit = _merge_json_cache.get('payload')
    if hit is not None and hit[0] == mt:
        return hit[1]
    d = np.load(MERGED_NPZ)
    P, C, M = d['points'], d['colors'], d['cam']
    stride = max(1, int(len(P) / 60000) + 1)
    P, C, M = P[::stride], C[::stride], M[::stride]
    payload = json.dumps({
        'points': [[round(float(x), 4) for x in row] for row in P],
        'colors': [list(map(int, row)) for row in C],
        'cam': [int(v) for v in M],
        'n_points': len(P), 'n_zed1': int((M == 0).sum()),
        'n_zed2': int((M == 1).sum()),
        'source': f'cap_fresh pair, fr3_link0 (stride {stride}, {len(P)}/{len(d["points"])} pts)',
        'note': 'ZED1 chain: trusted Nov-2; ZED2 chain: deployed Nov-4 tag-reg. '
                '~34mm inter-camera table bias under investigation (ZED1 drift suspect).'})
    _merge_json_cache['payload'] = (mt, payload)
    return payload

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image

PORT = 8791

PAGE = """<!doctype html>
<html lang="en"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>SAM2 segmentation (ZED1, live)</title>
<style>
:root{color-scheme:dark}
body{margin:0;background:#0f1214;color:#e8e6e1;
  font:14px/1.5 ui-sans-serif,system-ui,-apple-system,"Segoe UI",sans-serif;
  padding:16px;max-width:760px}
h1{margin:0 0 4px;font-size:18px;font-weight:650}
.sub{color:#8e959b;margin:0 0 12px;font-size:13px}
.stats{display:flex;flex-wrap:wrap;gap:6px;margin:0 0 12px}
.pill{border:1px solid #2a3137;border-radius:999px;padding:2px 10px;font-size:13px;
  color:#8e959b;background:#171b1e;font-variant-numeric:tabular-nums}
.pill b{color:#e8e6e1;font-weight:600}
.pill.ok b{color:#5fd39a}.pill.bad b{color:#e8776d}
img{width:100%;border:1px solid #2a3137;border-radius:6px;background:#000;display:block}
</style></head><body>
<h1>SAM2 segmentation &mdash; ZED1 live</h1>
<p class="sub">tracker mask over left rectified color &middot; VM103 :8791 &middot; zero lab load</p>
<div class="stats">
<span class="pill" id="pCov">mask <b>&mdash;</b></span>
<span class="pill" id="pAge">age <b>&mdash;</b></span>
<span class="pill" id="pFps">view <b>&mdash;</b></span>
<span class="pill" id="pState">tracker <b>&mdash;</b></span>
</div>
<img src="/seg.mjpeg" alt="live segmentation overlay">
<script>
async function poll(){
  try{
    const s = await (await fetch('/stats.json',{cache:'no-store'})).json();
    const set=(id,v,ok)=>{const e=document.getElementById(id);
      e.querySelector('b').textContent=v;e.className='pill '+(ok?'ok':'bad');};
    set('pCov',s.coverage_pct.toFixed(2)+'%',s.coverage_pct>0.05);
    set('pAge',s.mask_age_s.toFixed(1)+' s',s.mask_age_s<3);
    set('pFps',s.view_fps.toFixed(1)+' Hz',s.view_fps>3);
    set('pState',s.state,s.state==='TRACKING');
  }catch(e){}
  setTimeout(poll,1000);
}
poll();
</script></body></html>
"""

state = {'jpg': None, 'jpg2': None, 'cov': 0.0, 'age': 999.0, 'fps': 0.0,
         'tracking': False, 'lock': threading.Lock(),
         'drill': {'ok': False, 'age_s': 999.0, 'n_points': 0,
                   'aabb_min': [0, 0, 0], 'aabb_max': [0, 0, 0],
                   'centroid': [0, 0, 0], 'frame': 'fr3_link0',
                   'updated_wall': 0.0},
         'track3d': None,
         'live_cloud': None}  # {'pts': Nx3 fr3_link0, 'wall': t} from live mask+depth


def _live3d_ok(t3_age_s, mask_age_s, tracking):
    """Gate for drill.json live_3d.ok: fresh point AND fresh mask AND tracking.

    The tracker's /tracking_3d_point can stay fresh while the mask goes stale
    (LOST video state still publishes), so point age alone must not imply ok.
    """
    if t3_age_s is None or mask_age_s is None:
        return False
    return bool(t3_age_s <= 3.0 and mask_age_s <= 3.0 and tracking)


class Handler(BaseHTTPRequestHandler):
    def log_message(self, *a):
        pass

    def _send(self, code, ctype, body):
        raw = body if isinstance(body, bytes) else body.encode()
        self.send_response(code)
        self.send_header('Content-Type', ctype)
        self.send_header('Content-Length', str(len(raw)))
        self.send_header('Cache-Control', 'no-store')
        self.end_headers()
        self.wfile.write(raw)

INDEX = """<!doctype html>
<html lang="en"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>grasp lane viewers</title>
<style>
:root{color-scheme:dark}
body{margin:0;background:#0f1214;color:#e8e6e1;
  font:14px/1.5 ui-sans-serif,system-ui,-apple-system,"Segoe UI",sans-serif;
  padding:16px;max-width:640px}
h1{margin:0 0 12px;font-size:18px;font-weight:650}
a{color:#6aa7ff;text-decoration:none}
a:hover{text-decoration:underline}
.row{border:1px solid #2a3137;border-radius:6px;background:#171b1e;
  padding:10px 14px;margin:0 0 10px}
.row b{font-size:15px}
.row p{margin:4px 0 0;color:#8e959b;font-size:13px}
code{font-family:ui-monospace,monospace;font-size:12px;color:#8e959b}
</style></head><body>
<h1>grasp lane viewers (VM103)</h1>
<div class="row"><b><a href="/seg">/seg</a></b> &mdash; live SAM2 mask over ZED1
<p>real-time MJPEG + coverage/age/rate pills. Backed by
<code>/zed1/segmentation_overlay</code>.</p></div>
<div class="row"><b><a href="/grasp">/grasp</a></b> &mdash; chosen grasp vs cloud (static render)
<p>matplotlib figure of the pick frame against the masked cloud, per loop run.
<code>?run=&lt;id&gt;</code> selects a run, default is latest.</p></div>
<div class="row"><b><a href="/grasp3d">/grasp3d</a></b> &mdash; interactive 3D grasp inspector
<p>orbit / zoom / pan the cloud, gripper meshes, TCP and grasp frame.
Same <code>?run=</code> selector. Data: <code>/grasp.json</code>.</p></div>
<div class="row"><b><a href="/merge3d">/merge3d</a></b> &mdash; dual-ZED merged cloud (3D)
<p>ZED1 + ZED2 static clouds merged in fr3_link0. True RGB, or tint by camera.
Data: <code>/merge.json</code>.</p></div>
</body></html>
"""

GRASP_PAGE = """<!doctype html>
<html lang="en"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>grasp vs cloud: {run}</title>
<style>
:root{color-scheme:dark}
body{margin:0;background:#0f1214;color:#e8e6e1;
  font:14px/1.5 ui-sans-serif,system-ui,-apple-system,"Segoe UI",sans-serif;
  padding:16px;max-width:960px}
h1{margin:0 0 4px;font-size:18px;font-weight:650}
.sub{color:#8e959b;margin:0 0 12px;font-size:13px}
.sub a{color:#6aa7ff;text-decoration:none}
.stats{display:flex;flex-wrap:wrap;gap:6px;margin:0 0 12px}
.pill{border:1px solid #2a3137;border-radius:999px;padding:2px 10px;font-size:13px;
  color:#8e959b;background:#171b1e;font-variant-numeric:tabular-nums}
.pill b{color:#e8e6e1;font-weight:600}
img{width:100%;border:1px solid #2a3137;border-radius:6px;background:#000;display:block}
</style></head><body>
<h1>grasp vs cloud &mdash; {run}</h1>
<p class="sub"><a href="/">&larr; viewers</a> &middot; {src} &middot; {grip} &middot; render takes a few seconds</p>
<div class="stats">
<span class="pill">pick <b>{pick}</b></span>
<span class="pill">tcp <b>{tcp}</b></span>
<span class="pill">tcp&harr;cloud <b>{dmin} mm</b></span>
<span class="pill">points <b>{n}</b></span>
</div>
<img src="/grasp.png?run={run}" alt="grasp vs cloud render">
</body></html>
"""


GRASP3D_PAGE = """<!doctype html>
<html lang="en"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>grasp vs cloud (3D)</title>
<style>
:root{color-scheme:dark}
body{margin:0;background:#0f1214;color:#e8e6e1;
  font:14px/1.5 ui-sans-serif,system-ui,-apple-system,"Segoe UI",sans-serif;
  padding:16px;max-width:960px}
h1{margin:0 0 4px;font-size:18px;font-weight:650}
.sub{color:#8e959b;margin:0 0 12px;font-size:13px}
.sub a{color:#6aa7ff;text-decoration:none}
.stats{display:flex;flex-wrap:wrap;gap:6px;margin:0 0 12px}
.pill{border:1px solid #2a3137;border-radius:999px;padding:2px 10px;font-size:13px;
  color:#8e959b;background:#171b1e;font-variant-numeric:tabular-nums}
.pill b{color:#e8e6e1;font-weight:600}
#wrap{position:relative;border:1px solid #2a3137;border-radius:6px;background:#000}
#view{display:block;width:100%;aspect-ratio:16/10}
#err{position:absolute;left:8px;top:8px;right:8px;display:none;background:#3a1414;
  border:1px solid #e8776d;border-radius:6px;padding:8px 12px;font-size:13px}
#legend{position:absolute;left:8px;bottom:8px;background:rgba(15,18,20,.82);
  border:1px solid #2a3137;border-radius:6px;padding:6px 10px;font-size:12px;color:#8e959b}
#legend i{display:inline-block;width:10px;height:10px;border-radius:2px;margin-right:6px}
.row{display:flex;gap:8px;margin:0 0 12px}
button{border:1px solid #2a3137;border-radius:6px;background:#171b1e;color:#e8e6e1;
  padding:6px 14px;font-size:13px;cursor:pointer}
button:hover{border-color:#6aa7ff}
.hint{color:#8e959b;font-size:12px;margin:8px 0 0}
</style></head><body>
<h1>grasp vs cloud (3D) &mdash; <span id="run">…</span></h1>
<p class="sub"><a href="/">&larr; viewers</a> &middot; <span id="src"></span> &middot; drag orbit / wheel zoom / right-drag pan</p>
<div class="stats">
<span class="pill">pick <b id="pPick">&mdash;</b></span>
<span class="pill">tcp <b id="pTcp">&mdash;</b></span>
<span class="pill">tcp&harr;cloud <b id="pDmin">&mdash;</b></span>
<span class="pill">points <b id="pN">&mdash;</b></span>
</div>
<div class="row"><button type="button" id="bReload">Reload data</button><button type="button" id="bHome">Reset view</button></div>
<div id="wrap"><canvas id="view"></canvas><div id="err"></div>
<div id="legend"><i style="background:#5ec962"></i>cloud (z-coloured) &nbsp;<i style="background:#cfd4d9"></i>hand &nbsp;<i style="background:#e8b86d"></i>fingers &nbsp;<i style="background:#ff2222"></i>TCP &nbsp;<i style="background:#ff22ff"></i>pick</div></div>
<p class="hint">Gripper open 80 mm (descend state). Cloud: masked ZED1 depth in fr3_link0.</p>
<script type="importmap">{"imports":{"three":"https://cdn.jsdelivr.net/npm/three@0.160.0/build/three.module.js","three/addons/":"https://cdn.jsdelivr.net/npm/three@0.160.0/examples/jsm/"}}</script>
<script type="module">
import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';
const errBox = document.getElementById('err');
function fail(m){ errBox.style.display = 'block'; errBox.textContent = m; }
window.addEventListener('error', e => fail('page error: ' + (e.message || e.type)));
window.addEventListener('unhandledrejection', e => fail('load failed: ' + (e.reason && e.reason.message || e.reason)));
THREE.Object3D.DEFAULT_UP.set(0, 0, 1);
const canvas = document.getElementById('view');
const renderer = new THREE.WebGLRenderer({ canvas, antialias: true });
renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
const scene = new THREE.Scene();
scene.background = new THREE.Color(0x0f1214);
const camera = new THREE.PerspectiveCamera(45, 16 / 10, 0.005, 60);
const controls = new OrbitControls(camera, canvas);
controls.enableDamping = true;
controls.minDistance = 0.03;
controls.maxDistance = 4;
scene.add(new THREE.HemisphereLight(0xffffff, 0x223344, 0.9));
const key = new THREE.DirectionalLight(0xffffff, 1.4);
key.position.set(0.4, -0.5, 0.9);
scene.add(key);
const grid = new THREE.GridHelper(0.6, 12, 0x2a3137, 0x1d2328);
grid.rotation.x = Math.PI / 2;
scene.add(grid);
const STOPS = [[0.27,0.08,0.33],[0.23,0.32,0.55],[0.13,0.57,0.55],[0.37,0.79,0.38],[0.99,0.91,0.15]];
function zcolor(t, out){
  const x = Math.min(1, Math.max(0, t)) * (STOPS.length - 1);
  const i = Math.min(STOPS.length - 2, Math.floor(x)), u = x - i;
  for (let k = 0; k < 3; k++) out[k] = STOPS[i][k] + (STOPS[i+1][k] - STOPS[i][k]) * u;
}
let group = null, homePos = null, homeTgt = null;
function sizeCanvas(){
  const w = canvas.clientWidth, h = canvas.clientHeight, pr = renderer.getPixelRatio();
  if (canvas.width !== Math.floor(w * pr) || canvas.height !== Math.floor(h * pr)){
    renderer.setSize(w, h, false);
    camera.aspect = w / h;
    camera.updateProjectionMatrix();
  }
}
window.addEventListener('resize', sizeCanvas);
function build(d){
  if (group){ scene.remove(group); group.traverse(o => { if (o.geometry) o.geometry.dispose(); }); }
  group = new THREE.Group();
  const pts = d.cloud, n = d.n_points;
  const pos = new Float32Array(pts.flat());
  let zmin = 1e9, zmax = -1e9;
  for (let i = 2; i < pos.length; i += 3){ if (pos[i] < zmin) zmin = pos[i]; if (pos[i] > zmax) zmax = pos[i]; }
  const col = new Float32Array(n * 3), c = [0, 0, 0];
  for (let i = 0; i < n; i++){ zcolor((pos[3*i+2] - zmin) / Math.max(1e-6, zmax - zmin), c); col[3*i] = c[0]; col[3*i+1] = c[1]; col[3*i+2] = c[2]; }
  const pg = new THREE.BufferGeometry();
  pg.setAttribute('position', new THREE.BufferAttribute(pos, 3));
  pg.setAttribute('color', new THREE.BufferAttribute(col, 3));
  group.add(new THREE.Points(pg, new THREE.PointsMaterial({ size: 0.0035, vertexColors: true, sizeAttenuation: true })));
  for (const part of d.gripper){
    const g = new THREE.BufferGeometry();
    g.setAttribute('position', new THREE.BufferAttribute(new Float32Array(part.tris.flat()), 3));
    g.computeVertexNormals();
    group.add(new THREE.Mesh(g, new THREE.MeshStandardMaterial({
      color: part.kind === 'hand' ? 0xcfd4d9 : 0xe8b86d, roughness: 0.55, metalness: 0.08 })));
  }
  const tcp = new THREE.Vector3(...d.tcp), pick = new THREE.Vector3(...d.pick);
  const dot = (p, r, color) => { const m = new THREE.Mesh(new THREE.SphereGeometry(r, 20, 14), new THREE.MeshBasicMaterial({ color })); m.position.copy(p); group.add(m); };
  dot(tcp, 0.004, 0xff2222);
  const pk = new THREE.Mesh(new THREE.OctahedronGeometry(0.007), new THREE.MeshBasicMaterial({ color: 0xff22ff }));
  pk.position.copy(pick); group.add(pk);
  dot(new THREE.Vector3(...d.centroid), 0.003, 0xffffff);
  const R = d.R, axc = [0xff2222, 0x22ff22, 0x2266ff];
  for (let i = 0; i < 3; i++)
    group.add(new THREE.ArrowHelper(new THREE.Vector3(R[0][i], R[1][i], R[2][i]), tcp, 0.05, axc[i], 0.012, 0.007));
  group.add(new THREE.ArrowHelper(new THREE.Vector3(R[0][2], R[1][2], R[2][2]), tcp, 0.1, 0xff8800, 0.02, 0.012));
  grid.position.set(tcp.x, tcp.y, 0);
  scene.add(group);
  document.getElementById('run').textContent = d.run;
  document.getElementById('src').textContent = d.source;
  document.getElementById('pPick').textContent = d.pick.map(v => v.toFixed(3)).join(', ');
  document.getElementById('pTcp').textContent = d.tcp.map(v => v.toFixed(3)).join(', ');
  document.getElementById('pDmin').textContent = d.tcp_to_cloud_mm + ' mm';
  document.getElementById('pN').textContent = d.n_points;
  homePos = new THREE.Vector3(tcp.x + 0.28, tcp.y - 0.32, tcp.z + 0.24);
  homeTgt = tcp.clone();
  camera.position.copy(homePos);
  controls.target.copy(homeTgt);
  controls.update();
}
async function load(){
  errBox.style.display = 'none';
  const r = await fetch('grasp.json' + location.search, { cache: 'no-store' });
  if (!r.ok) throw new Error('grasp.json ' + r.status + ': ' + (await r.text()));
  build(await r.json());
}
document.getElementById('bReload').addEventListener('click', () => load().catch(e => fail(String(e))));
document.getElementById('bHome').addEventListener('click', () => {
  if (!homePos) return;
  camera.position.copy(homePos); controls.target.copy(homeTgt); controls.update();
});
renderer.setAnimationLoop(() => { sizeCanvas(); controls.update(); renderer.render(scene, camera); });
load().catch(e => fail(String(e)));
</script></body></html>
""";

MERGE3D_PAGE = """<!doctype html>
<html lang="en"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>dual-ZED merged cloud (3D)</title>
<style>
:root{color-scheme:dark}
body{margin:0;background:#0f1214;color:#e8e6e1;
  font:14px/1.5 ui-sans-serif,system-ui,-apple-system,"Segoe UI",sans-serif;
  padding:16px;max-width:960px}
h1{margin:0 0 4px;font-size:18px;font-weight:650}
.sub{color:#8e959b;margin:0 0 12px;font-size:13px}
.sub a{color:#6aa7ff;text-decoration:none}
.stats{display:flex;flex-wrap:wrap;gap:6px;margin:0 0 12px}
.pill{border:1px solid #2a3137;border-radius:999px;padding:2px 10px;font-size:13px;
  color:#8e959b;background:#171b1e;font-variant-numeric:tabular-nums}
.pill b{color:#e8e6e1;font-weight:600}
#wrap{position:relative;border:1px solid #2a3137;border-radius:6px;background:#000}
#view{display:block;width:100%;aspect-ratio:16/10}
#err{position:absolute;left:8px;top:8px;right:8px;display:none;background:#3a1414;
  border:1px solid #e8776d;border-radius:6px;padding:8px 12px;font-size:13px}
#legend{position:absolute;left:8px;bottom:8px;background:rgba(15,18,20,.82);
  border:1px solid #2a3137;border-radius:6px;padding:6px 10px;font-size:12px;color:#8e959b}
#legend i{display:inline-block;width:10px;height:10px;border-radius:2px;margin-right:6px}
.row{display:flex;gap:8px;margin:0 0 12px;align-items:center;flex-wrap:wrap}
button{border:1px solid #2a3137;border-radius:6px;background:#171b1e;color:#e8e6e1;
  padding:6px 14px;font-size:13px;cursor:pointer}
button:hover{border-color:#6aa7ff}
.tint{font-size:13px;color:#8e959b;display:flex;gap:6px;align-items:center}
.hint{color:#8e959b;font-size:12px;margin:8px 0 0}
</style></head><body>
<h1>dual-ZED merged cloud (3D)</h1>
<p class="sub"><a href="/">&larr; viewers</a> &middot; <span id="src"></span> &middot; drag orbit / wheel zoom / right-drag pan</p>
<div class="stats">
<span class="pill">zed1 <b id="pN1">&mdash;</b></span>
<span class="pill">zed2 <b id="pN2">&mdash;</b></span>
<span class="pill">total <b id="pN">&mdash;</b></span>
</div>
<div class="row"><button type="button" id="bReload">Reload data</button><button type="button" id="bHome">Reset view</button>
<label class="tint"><input type="checkbox" id="cTint"> tint by camera</label></div>
<div id="wrap"><canvas id="view"></canvas><div id="err"></div>
<div id="legend"><span id="lRgb">true RGB from both cameras</span><span id="lCam" style="display:none"><i style="background:#22d3ee"></i>zed1 &nbsp;<i style="background:#fb923c"></i>zed2</span></div></div>
<p class="hint" id="note"></p>
<script type="importmap">{"imports":{"three":"https://cdn.jsdelivr.net/npm/three@0.160.0/build/three.module.js","three/addons/":"https://cdn.jsdelivr.net/npm/three@0.160.0/examples/jsm/"}}</script>
<script type="module">
import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';
const errBox = document.getElementById('err');
function fail(m){ errBox.style.display = 'block'; errBox.textContent = m; }
window.addEventListener('error', e => fail('page error: ' + (e.message || e.type)));
window.addEventListener('unhandledrejection', e => fail('load failed: ' + (e.reason && e.reason.message || e.reason)));
THREE.Object3D.DEFAULT_UP.set(0, 0, 1);
const canvas = document.getElementById('view');
const renderer = new THREE.WebGLRenderer({ canvas, antialias: true });
renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
const scene = new THREE.Scene();
scene.background = new THREE.Color(0x0f1214);
const camera = new THREE.PerspectiveCamera(45, 16 / 10, 0.005, 60);
const controls = new OrbitControls(camera, canvas);
controls.enableDamping = true;
controls.minDistance = 0.03;
controls.maxDistance = 6;
const grid = new THREE.GridHelper(1.2, 12, 0x2a3137, 0x1d2328);
grid.rotation.x = Math.PI / 2;
scene.add(grid);
let group = null, homePos = null, homeTgt = null, cur = null;
function sizeCanvas(){
  const w = canvas.clientWidth, h = canvas.clientHeight, pr = renderer.getPixelRatio();
  if (canvas.width !== Math.floor(w * pr) || canvas.height !== Math.floor(h * pr)){
    renderer.setSize(w, h, false);
    camera.aspect = w / h;
    camera.updateProjectionMatrix();
  }
}
window.addEventListener('resize', sizeCanvas);
const TINT = [[0.13, 0.83, 0.93], [0.98, 0.57, 0.24]];
function paint(){
  if (!cur) return;
  const tinted = document.getElementById('cTint').checked;
  const attr = cur.points.geometry.getAttribute('color');
  const a = attr.array, n = cur.n;
  if (tinted){
    for (let i = 0; i < n; i++){ const t = TINT[cur.cam[i]]; a[3*i] = t[0]; a[3*i+1] = t[1]; a[3*i+2] = t[2]; }
  } else {
    for (let i = 0; i < n; i++){ a[3*i] = cur.rgb[i][0] / 255; a[3*i+1] = cur.rgb[i][1] / 255; a[3*i+2] = cur.rgb[i][2] / 255; }
  }
  attr.needsUpdate = true;
  document.getElementById('lRgb').style.display = tinted ? 'none' : '';
  document.getElementById('lCam').style.display = tinted ? '' : 'none';
}
function build(d){
  if (group){ scene.remove(group); group.traverse(o => { if (o.geometry) o.geometry.dispose(); }); }
  group = new THREE.Group();
  const n = d.n_points;
  const pos = new Float32Array(d.points.flat());
  let mn = [1e9, 1e9, 1e9], mx = [-1e9, -1e9, -1e9];
  for (let i = 0; i < n; i++)
    for (let k = 0; k < 3; k++){
      const v = pos[3*i+k];
      if (v < mn[k]) mn[k] = v;
      if (v > mx[k]) mx[k] = v;
    }
  const pg = new THREE.BufferGeometry();
  pg.setAttribute('position', new THREE.BufferAttribute(pos, 3));
  pg.setAttribute('color', new THREE.BufferAttribute(new Float32Array(n * 3), 3));
  const pts = new THREE.Points(pg, new THREE.PointsMaterial({ size: 0.004, vertexColors: true, sizeAttenuation: true }));
  group.add(pts);
  scene.add(group);
  cur = { points: pts, n, rgb: d.colors, cam: d.cam };
  paint();
  document.getElementById('src').textContent = d.source;
  document.getElementById('note').textContent = d.note;
  document.getElementById('pN1').textContent = d.n_zed1;
  document.getElementById('pN2').textContent = d.n_zed2;
  document.getElementById('pN').textContent = n;
  const cx = (mn[0] + mx[0]) / 2, cy = (mn[1] + mx[1]) / 2, cz = (mn[2] + mx[2]) / 2;
  const span = Math.max(mx[0] - mn[0], mx[1] - mn[1], mx[2] - mn[2]);
  grid.position.set(cx, cy, 0);
  homeTgt = new THREE.Vector3(cx, cy, cz);
  homePos = new THREE.Vector3(cx + span * 0.55, cy - span * 0.75, cz + span * 0.6);
  camera.position.copy(homePos);
  controls.target.copy(homeTgt);
  controls.update();
}
async function load(){
  errBox.style.display = 'none';
  const r = await fetch('merge.json', { cache: 'no-store' });
  if (!r.ok) throw new Error('merge.json ' + r.status + ': ' + (await r.text()));
  build(await r.json());
}
document.getElementById('bReload').addEventListener('click', () => load().catch(e => fail(String(e))));
document.getElementById('bHome').addEventListener('click', () => {
  if (!homePos) return;
  camera.position.copy(homePos); controls.target.copy(homeTgt); controls.update();
});
document.getElementById('cTint').addEventListener('change', paint);
renderer.setAnimationLoop(() => { sizeCanvas(); controls.update(); renderer.render(scene, camera); });
load().catch(e => fail(String(e)));
</script></body></html>
""";


def _latest_run():
    ids = sorted((d for d in os.listdir(RUNS)
                  if os.path.isdir(os.path.join(RUNS, d))
                  and os.path.exists(os.path.join(RUNS, d, 'pick.json'))))
    return ids[-1] if ids else None


def _grasp_png(run_id):
    now = time.time()
    hit = _grasp_cache.get(run_id)
    if hit is not None and hit[0] >= now:
        return hit[1], hit[2], None
    run_dir = os.path.join(RUNS, run_id)
    if not os.path.isdir(run_dir):
        return None, None, f'unknown run: {run_id}'
    try:
        png, meta = grasp_render.render_grasp(run_dir)
    except (FileNotFoundError, KeyError, ValueError) as e:
        return None, None, f'cannot render {run_id}: {e}'
    _grasp_cache[run_id] = (now + _GRASP_TTL, png, meta)
    return png, meta, None


class Handler(BaseHTTPRequestHandler):
    def log_message(self, *a):
        pass

    def _send(self, code, ctype, body):
        raw = body if isinstance(body, bytes) else body.encode()
        self.send_response(code)
        self.send_header('Content-Type', ctype)
        self.send_header('Content-Length', str(len(raw)))
        self.send_header('Cache-Control', 'no-store')
        self.end_headers()
        self.wfile.write(raw)

    def do_GET(self):
        u = urlparse(self.path)
        if u.path == '/':
            return self._send(200, 'text/html', INDEX)
        if u.path == '/grasp':
            run_id = parse_qs(u.query).get('run', [None])[0] or _latest_run()
            if run_id is None:
                return self._send(404, 'text/plain', 'no runs with pick.json')
            _, meta, err = _grasp_png(run_id)
            if err:
                return self._send(500, 'text/plain', err)
            page = GRASP_PAGE.replace('{run}', run_id).replace(
                '{src}', meta['source']).replace(
                '{grip}', meta.get('gripper', '')).replace(
                '{pick}', ', '.join(f'{v:.3f}' for v in meta['pick_m'])).replace(
                '{tcp}', ', '.join(f'{v:.3f}' for v in meta['tcp_m'])).replace(
                '{dmin}', str(meta['tcp_to_cloud_mm'])).replace(
                '{n}', str(meta['n_points']))
            return self._send(200, 'text/html', page)
        if u.path == '/grasp.png':
            run_id = parse_qs(u.query).get('run', [None])[0] or _latest_run()
            if run_id is None:
                return self._send(404, 'text/plain', 'no runs with pick.json')
            png, _, err = _grasp_png(run_id)
            if err:
                return self._send(500, 'text/plain', err)
            return self._send(200, 'image/png', png)
        if u.path == '/grasp.json':
            run_id = parse_qs(u.query).get('run', [None])[0] or _latest_run()
            if run_id is None:
                return self._send(404, 'text/plain', 'no runs with pick.json')
            now = time.time()
            hit = _grasp_json_cache.get(run_id)
            if hit is not None and hit[0] >= now:
                d = hit[1]
            else:
                run_dir = os.path.join(RUNS, run_id)
                if not os.path.isdir(run_dir):
                    return self._send(404, 'text/plain',
                                      f'unknown run: {run_id}')
                try:
                    d = grasp_render.load_grasp(run_dir)
                except (FileNotFoundError, KeyError, ValueError) as e:
                    return self._send(500, 'text/plain',
                                      f'cannot load {run_id}: {e}')
                _grasp_json_cache[run_id] = (now + _GRASP_TTL, d)
            with state['lock']:
                lc = state.get('live_cloud')
            if lc is not None and time.time() - lc['wall'] < 6.0:
                W = lc['pts']
                lo, hi = _live_anchor_box(W)

                def _anchor(model, lo=lo, hi=hi):
                    # xy: live box center; z: align top surfaces (the grasp
                    # run saw only the drill top, the live cloud sees more)
                    return [(lo[0] + hi[0]) / 2.0, (lo[1] + hi[1]) / 2.0,
                            model['center'][2] + hi[2] - model['aabb_max'][2]]
                d = _live_shift_grasp(run_id, d, anchor=_anchor)
                d = dict(d)
                # per-update run tag: the :8788 page rebuilds the cloud only on
                # a new run id, and its live_3d "follow" offset (for the old
                # static cloud) turns off when run != drill.json source_run;
                # this cloud is already in fr3_link0, so it must not be moved.
                d['run'] = f"{run_id}@live{int(lc['wall'] * 10)}"
                d['cloud'] = np.round(W, 4).tolist()
                d['n_points'] = int(len(W))
                d['centroid'] = [round(float(v), 4) for v in W.mean(0)]
                live = dict(d.get('live') or {})
                _z2s = lc.get('zed2_src') or 'none'
                live['cloud'] = 'live_mask_depth+zed2_' + (
                    'mask' if _z2s in ('mask', 'mask-held') else 'none')
                live['n_zed2'] = lc.get('n_zed2', 0)
                live['zed2_T'] = lc.get('zed2_T')
                live['cloud_age_s'] = round(time.time() - lc['wall'], 1)
                d['live'] = live
            else:
                d = _live_shift_grasp(run_id, d)
            return self._send(200, 'application/json', json.dumps(d))
        if u.path == '/grasp3d':
            return self._send(200, 'text/html', GRASP3D_PAGE)
        if u.path == '/merge.json':
            try:
                return self._send(200, 'application/json', _merge_json())
            except (FileNotFoundError, KeyError, ValueError) as e:
                return self._send(500, 'text/plain', f'cannot load merge: {e}')
        if u.path == '/merge3d':
            return self._send(200, 'text/html', MERGE3D_PAGE)
        if u.path == '/seg':
            return self._send(200, 'text/html', PAGE)
        if self.path == '/stats.json':
            with state['lock']:
                payload = json.dumps({
                    'coverage_pct': state['cov'],
                    'mask_age_s': state['age'],
                    'view_fps': state['fps'],
                    'state': 'TRACKING' if state['tracking'] else 'LOST/IDLE',
                })
            return self._send(200, 'application/json', payload)
        if u.path == '/drill.json':
            with state['lock']:
                d = dict(state['drill'])
                t3 = state['track3d']
                mage, tracking = state['age'], state['tracking']
            d['age_s'] = round(time.time() - d.pop('updated_wall'), 1) \
                if d['updated_wall'] else 999.0
            if t3 is None:
                d['live_3d'] = {'ok': False}
            else:
                t3_age = time.time() - t3['wall']
                d['live_3d'] = {
                    'ok': _live3d_ok(t3_age, mage, tracking),
                    'point_m': [round(v, 4) for v in t3['point']],
                    'frame': t3['frame'],
                    'age_s': round(t3_age, 1)}
            return self._send(200, 'application/json', json.dumps(d))
        if u.path == '/grasp_target.json':
            try:
                with open(TARGET_FILE) as f:
                    return self._send(200, 'application/json', f.read())
            except OSError as e:
                return self._send(404, 'text/plain', f'no target file: {e}')
        if self.path == '/snap.jpg':
            with state['lock']:
                jpg = state['jpg']
            if jpg is None:
                return self._send(503, 'text/plain', 'no frame yet')
            return self._send(200, 'image/jpeg', jpg)
        if self.path == '/snap2.jpg':
            with state['lock']:
                jpg = state['jpg2']
            if jpg is None:
                return self._send(503, 'text/plain', 'no frame yet')
            return self._send(200, 'image/jpeg', jpg)
        if self.path == '/seg.mjpeg':
            self.send_response(200)
            self.send_header('Content-Type',
                             'multipart/x-mixed-replace; boundary=frame')
            self.send_header('Cache-Control', 'no-store')
            self.end_headers()
            try:
                while True:
                    with state['lock']:
                        jpg = state['jpg']
                    if jpg is None:
                        time.sleep(0.1)
                        continue
                    self.wfile.write(b'--frame\r\nContent-Type: image/jpeg\r\n\r\n'
                                     + jpg + b'\r\n')
                    time.sleep(0.1)
            except (BrokenPipeError, ConnectionResetError):
                pass
            return
        return self._send(404, 'text/plain', 'paths: /seg /seg.mjpeg /stats.json')


_ext_cache = {}  # run_id -> half extents (object shape, grasp-time data)
_live_hold = {'center': None, 'wall': 0.0, 'yaw': 0.0}  # last live tracker fix (held
# across the tracker's multi-second publish gaps; the drill is static)
_LIVE_HOLD_S = 10.0
_yaw0 = {}  # run_id -> baseline live yaw (rad) at first use
_lc_log = {}


def _live_shift_grasp(run_id, d, anchor=None):
    """Recenter a grasp scene on the live tracker fix (2026-11-04).

    Rigid translation + yaw of cloud/pick/tcp/centroid/gripper by
    (live_hold - grasp_model_center) and the live mask-PCA yaw when the
    hold is fresh and the run is the drill run the hold tracks; otherwise
    the forensic scene untouched. Relative geometry (tcp_to_cloud_mm
    etc.) is preserved either way. Yaw is approximate (image-plane PCA
    delta applied as world yaw). Returns a new dict.
    """
    out = dict(d)
    out['live'] = None
    try:
        mid, model = _target_model()
    except Exception:
        return out
    if mid != run_id or model is None:
        return out
    hold_age = time.time() - _live_hold['wall']
    if anchor is not None:  # measured live cloud wins over the 2D model fix
        if callable(anchor):
            anchor = anchor(model)
        c_anchor = np.asarray(anchor, float)
        hold_age = 0.0
    elif _live_hold['center'] is None or hold_age >= _LIVE_HOLD_S:
        return out
    else:
        c_anchor = np.array(_live_hold['center'])
    off = c_anchor - np.array(model['center'])
    # yaw relative to the first live yaw seen for this run (2026-11-05): a run
    # generated from the current scene must not be rotated; the mask-PCA vs
    # model-axis yaw has a per-model bias (13.6 deg on the dual-camera run).
    yaw_abs = float(_live_hold.get('yaw', 0.0) or 0.0)
    yaw = yaw_abs - _yaw0.setdefault(run_id, yaw_abs)
    if float(np.linalg.norm(off)) < 0.005 and abs(yaw) < 0.005:
        return out
    import copy
    out = copy.deepcopy(d)
    o = off.tolist()
    if abs(yaw) > 0.005:
        c_mod = np.array(model['center'])
        c_live = c_anchor
        cy, sy = np.cos(yaw), np.sin(yaw)
        Rz = np.array([[cy, -sy, 0.0], [sy, cy, 0.0], [0.0, 0.0, 1.0]])

        def _rot(P):
            return (Rz @ (np.asarray(P, float) - c_mod).T).T + c_live

        out['cloud'] = _rot(out['cloud']).round(4).tolist()
        for k in ('centroid', 'pick', 'tcp'):
            out[k] = [round(float(v), 4) for v in _rot([out[k]])[0]]
        for part in out.get('gripper', []):
            part['tris'] = _rot(part['tris']).round(4).tolist()
        out['live'] = dict(offset_m=[round(float(v), 4) for v in off],
                           age_s=round(hold_age, 1),
                           held=hold_age >= 2.0,
                           yaw_deg=round(float(np.degrees(yaw)), 1))
        return out
    out['cloud'] = (np.array(out['cloud']) + off).round(4).tolist()
    for k in ('centroid', 'pick', 'tcp'):
        out[k] = [round(float(v) + o[i], 4) for i, v in enumerate(out[k])]
    for part in out.get('gripper', []):
        part['tris'] = (np.array(part['tris']) + off).round(4).tolist()
    out['live'] = dict(offset_m=[round(float(v), 4) for v in off],
                       age_s=round(hold_age, 1),
                       held=hold_age >= 2.0)
    return out


def _live_anchor_box(W):
    """Live box for the /grasp.json prologue anchor (2026-11-06).

    Top-8cm p2/p98 box: like-for-like with _target_model. The old full-cloud
    box drifted ~26 mm in y on a static scene once mask-only ZED2 added the
    low, y-asymmetric far face. Falls back to the full-cloud box when the
    top subset is too small to trust."""
    top = W[W[:, 2] > W[:, 2].max() - 0.08]
    if len(top) < 25:
        top = W
    return (np.percentile(top, 2, axis=0), np.percentile(top, 98, axis=0))


def _target_model():
    """Grasp-time object model from the latest loop run's cloud.

    Top-cluster AABB + centroid (top 8 cm: the object; the ZED curtain
    below it is silhouette bleed, verified on the drill cloud). Pure
    pipeline data, no per-object numbers: a new loop run automatically
    refreshes the model. Returns (run_id, model_dict) or (None, None).
    """
    run_id = _latest_run()
    if run_id is None:
        return None, None
    if run_id in _ext_cache:
        return run_id, _ext_cache[run_id]
    try:
        P = np.load(os.path.join(RUNS, run_id, 'cloud.npy'))
    except OSError:
        return None, None
    top = P[P[:, 2] > P[:, 2].max() - 0.08]
    if len(top) < 25:
        return None, None
    # Robust box: min/max latches onto fringe outliers (2026-11-06: a -0.308
    # fringe pulled the model y-center 26 mm off the live anchor on a static
    # scene). Must match _live_anchor_box's estimator (like-for-like).
    lo, hi = np.percentile(top, 2, axis=0), np.percentile(top, 98, axis=0)
    he = np.maximum((hi - lo) / 2.0, 0.02)
    c = (lo + hi) / 2.0
    try:
        _C = np.cov(np.stack([top[:, 0], top[:, 1]]))
        _w, _V = np.linalg.eigh(_C)
        _ax = _V[:, -1] / (np.linalg.norm(_V[:, -1]) or 1.0)
    except Exception:
        _ax = np.array([1.0, 0.0])
    model = dict(aabb_min=[round(float(v), 4) for v in lo],
                 aabb_max=[round(float(v), 4) for v in hi],
                 center=[round(float(v), 4) for v in c],
                 half_extents=[round(float(v), 4) for v in he],
                 axis_xy=[round(float(v), 4) for v in _ax])
    _ext_cache.clear()
    _ext_cache[run_id] = model
    return run_id, model


def _tf_to_RT(tf):
    t, q = tf.transform.translation, tf.transform.rotation
    n = np.array([q.x, q.y, q.z, q.w], float)
    n /= (np.linalg.norm(n) or 1.0)
    x, y, zc, w = n
    R = np.array([[1 - 2 * (y * y + zc * zc), 2 * (x * y - zc * w), 2 * (x * zc + y * w)],
                  [2 * (x * y + zc * w), 1 - 2 * (x * x + zc * zc), 2 * (y * zc - x * w)],
                  [2 * (x * zc - y * w), 2 * (y * zc + x * w), 1 - 2 * (x * x + y * y)]])
    return R, np.array([t.x, t.y, t.z])


def _live_cloud(mask, depths, info, tfbuf, max_pts=4000):
    """Object cloud in fr3_link0 from the LIVE mask + the depth frame nearest
    its stamp (2026-11-04: the grasp-time cloud moved/rotated rigidly could not
    match a drill lying in a different pose). Mask eroded 2 px (rim depth
    bleeds to background); depth band [z10-2cm, z10+20cm] drops hole/background
    readings. Returns Nx3 float array or None."""
    import rclpy as _r
    if not depths:
        return None
    ms = mask.header.stamp.sec + mask.header.stamp.nanosec * 1e-9
    d = min(depths, key=lambda m: abs(
        m.header.stamp.sec + m.header.stamp.nanosec * 1e-9 - ms))
    ds = d.header.stamp.sec + d.header.stamp.nanosec * 1e-9
    if abs(ds - ms) > 0.3:
        return None
    if d.encoding == '32FC1':
        D = np.frombuffer(d.data, np.float32).reshape(
            d.height, d.step // 4)[:, :d.width]
    elif d.encoding == '16UC1':
        D = np.frombuffer(d.data, np.uint16).reshape(
            d.height, d.step // 2)[:, :d.width].astype(np.float32) / 1000.0
    else:
        return None
    M = np.frombuffer(mask.data, np.uint8).reshape(
        mask.height, mask.step)[:, :mask.width]
    if M.shape != D.shape:
        M = cv2.resize(M, (D.shape[1], D.shape[0]),
                       interpolation=cv2.INTER_NEAREST)
    M0 = (M > 0).astype(np.uint8)
    M = cv2.erode(M0, np.ones((5, 5), np.uint8))
    vs, us = np.nonzero(M)
    if len(us) < 150:
        return None
    z = D[vs, us]
    ok = np.isfinite(z) & (z > 0.15) & (z < 2.5)
    us, vs, z = us[ok], vs[ok], z[ok]
    if len(z) < 150:
        return None
    z10 = float(np.percentile(z, 10))
    ok = (z > z10 - 0.02) & (z < z10 + 0.20)
    us, vs, z = us[ok], vs[ok], z[ok]
    if len(z) < 150:
        return None
    K = np.array(info.k, np.float64).reshape(3, 3)
    sx = D.shape[1] / float(info.width) if info.width else 1.0
    fx, fy, cx, cy = K[0, 0] * sx, K[1, 1] * sx, K[0, 2] * sx, K[1, 2] * sx
    P = np.stack([(us - cx) * z / fx, (vs - cy) * z / fy, z], 1)
    tf = tfbuf.lookup_transform('fr3_link0', d.header.frame_id or
                                info.header.frame_id, _r.time.Time(),
                                timeout=_r.duration.Duration(seconds=0.5))
    R, t = _tf_to_RT(tf)
    sc = 1.0
    ov = _calib_override('zed1')
    if ov is not None:
        R, t, sc = ov
        P = P * sc
    W = P @ R.T + t
    med = np.median(W, 0)
    W = W[np.linalg.norm(W - med, axis=1) < 0.20]
    if len(W) < 150:
        return None
    if len(W) > max_pts:
        W = W[::int(len(W) / max_pts) + 1]
    support = _support_z(M0, D * sc, (fx, fy, cx, cy), R, t, W)
    return W, support


VIEWER_CALIB = '/home/nexus/spot-teleop/fr3_real_repro/zed2_align/calib_robot/viewer_calib.json'
_vc = {'mtime': None, 'd': None}


def _calib_override(cam):
    """VIEWER-ONLY calibration override (2026-11-05): {cam: {T_base_cam 4x4,
    depth_scale}} from the robot-fit (pose + depth scale). Used by the drill
    cloud instead of the lab TF while it exists; the map/TF are untouched.
    Delete the file to go back to the live TF. Returns (R, t, scale) or None."""
    try:
        mt = os.path.getmtime(VIEWER_CALIB)
    except OSError:
        return None
    if mt != _vc['mtime']:
        try:
            _vc['d'], _vc['mtime'] = json.load(open(VIEWER_CALIB)), mt
        except Exception:
            return None
    c = (_vc['d'] or {}).get(cam)
    if not c:
        return None
    T = np.array(c['T_base_cam'], float)
    return T[:3, :3], T[:3, 3], float(c.get('depth_scale', 1.0))


ZED2_CALIB = ('/home/nexus/spot-teleop/fr3_real_repro/zed2_align/'
              'zed2_extrinsic_20261105_robotfit7.json')
_zed2_T = {}
# touch this file to drop ZED2 from the drill cloud without a restart
ZED2_OFF_FLAG = '/home/nexus/spot-teleop/fr3_real_repro/vm_offload/ZED2_OFF'


def _zed2_RT(frame, tfbuf):
    """fr3_link0 <- ZED2 optical. Live TF first (lab publisher, planetag since
    2026-11-04 23:3x UTC); fallback = the deployed JSON (same numbers)."""
    import rclpy as _r
    try:
        tf = tfbuf.lookup_transform('fr3_link0', frame, _r.time.Time(),
                                    timeout=_r.duration.Duration(seconds=0.2))
        _zed2_T['src'] = 'tf'
        return _tf_to_RT(tf)
    except Exception:
        if 'json' not in _zed2_T:
            T = np.array(json.load(open(ZED2_CALIB))['T_base_cam'], float)
            _zed2_T['json'] = (T[:3, :3], T[:3, 3])
        _zed2_T['src'] = 'json'
        return _zed2_T['json']


# Drill-table height measured by fingertip touch, FK (2026-11-06): floor
# fallback when the ZED1 table ring fails (touches +27.5/+29.9 mm).
TABLE_Z_FK = 0.029
# Last mask-gated ZED2 set: continuity on pairing misses (<=2 s wall).
# Never a box crop: no matched mask means stale mask points or nothing.
_z2_hold = {'pts': None, 'wall': 0.0}


def _z2_miss():
    """No matched mask this tick: hold the last mask-gated set (<=4 s)."""
    h = _z2_hold
    if h['pts'] is not None and time.time() - h['wall'] <= 4.0:
        return h['pts'], 'mask-held'
    return None, 'none'


def _z2_pair_gate(depths2):
    """Pairing gate from the live depth cadence: 2.5x the median stamp
    spacing, clamped to [0.1, 0.5] s. The cable subsamples the 30 Hz depth
    irregularly (4-9 Hz observed), so a fixed 0.1 s gate misses whenever
    the mask stamp lands in a depth gap (seen live: dt 0.17-0.50 s)."""
    if len(depths2) < 3:
        return 0.1
    sts = sorted(m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
                 for _, m in depths2)
    med = float(np.median(np.diff(sts)))
    return min(0.5, max(0.1, 2.5 * med))


def _cap_points(P, max_pts):
    """Uniform random subsample to exactly max_pts (density, not ratio)."""
    if len(P) <= max_pts:
        return P
    return P[np.random.default_rng().choice(len(P), max_pts, replace=False)]


def _zed2_points(W, support_z, depths2, info2, tfbuf, masks2=None,
                 max_pts=4000):
    """Drill points seen by ZED2 (top-down view; fills what ZED1 misses).
    Mask-ONLY: the ZED2 SAM2 mask (/zed2/segmentation_mask, own video
    predictor) gates every point; there is no box fallback (removed
    2026-11-06: a box crop silently admits table/clutter as drill). Pairing
    is mask-anchored: the newest fresh mask picks its contemporary depth
    frame (|dt| <= adaptive gate, _z2_pair_gate), because masks lag ~1 s
    behind depth and the old latest-depth pairing missed most ticks. The
    mask is 5x5-eroded (same as ZED1) against edge flying pixels. A
    fresh-but-empty mask (tracker
    confirms the object gone) clears the hold and yields None. ZED2 depth
    via vm-cable-fanin2: since 23:48 UTC raw robot-masked
    /franka_seg_zed2/depth (FANIN2_DEPTH=seg, after the cable reassembly
    fix); with FANIN2_DEPTH=compressed the robot is NOT masked and a
    gripper inside the box would be kept.
    Calibration check 2026-11-04 (cap_1104c): ZED2 vs ZED1 on the drill
    14.9 mm median. Returns (Nx3 fr3_link0, src) with src in
    {'mask', 'mask-held', 'none'} or (None, None) when the ZED2 path itself
    is unavailable."""
    if not depths2 or info2 is None or os.path.exists(ZED2_OFF_FLAG):
        return None, None
    now = time.time()
    cand = [(m.header.stamp.sec + m.header.stamp.nanosec * 1e-9, m)
            for t_rxm, m in (masks2 or []) if now - t_rxm <= 1.5]
    if not cand:
        return _z2_miss()
    ms, _m = max(cand, key=lambda c: c[0])
    gate = _z2_pair_gate(depths2)
    d, bdt = None, gate  # depth contemporary with the newest fresh mask
    for _t_rx, d0 in depths2:
        dt = abs((d0.header.stamp.sec + d0.header.stamp.nanosec * 1e-9) - ms)
        if dt <= bdt:
            d, bdt = d0, dt
    if d is None or d.encoding != '32FC1':
        return _z2_miss()
    D = np.frombuffer(d.data, np.float32).reshape(d.height, d.step // 4)[:, :d.width]
    M2 = np.frombuffer(_m.data, np.uint8).reshape(_m.height, _m.width)
    if M2.shape != D.shape:
        M2 = cv2.resize(M2, (D.shape[1], D.shape[0]),
                        interpolation=cv2.INTER_NEAREST)
    # Erode like the ZED1 path: kills mask-edge flying pixels and detached
    # specks (the wide box no longer clips them for us).
    M2 = cv2.erode((M2 > 0).astype(np.uint8), np.ones((5, 5), np.uint8))
    if not M2.any():  # object confirmed gone: clear the hold, never a crop
        _z2_hold['pts'], _z2_hold['wall'] = None, 0.0
        return None, 'mask'
    lo = np.percentile(W, 2, axis=0) - [0.08, 0.08, 0.0]
    hi = np.percentile(W, 98, axis=0) + [0.08, 0.08, 0.08]
    lo[2] = (support_z if support_z is not None else TABLE_Z_FK) + 0.01
    R, t = _zed2_RT(d.header.frame_id or info2.header.frame_id, tfbuf)
    sc2 = 1.0
    ov = _calib_override('zed2')
    if ov is not None:
        R, t, sc2 = ov
        _zed2_T['src'] = 'viewer_calib'
    K = np.array(info2.k, np.float64).reshape(3, 3)
    sx = D.shape[1] / float(info2.width) if info2.width else 1.0
    fx, fy, cx, cy = K[0, 0] * sx, K[1, 1] * sx, K[0, 2] * sx, K[1, 2] * sx
    # pixel ROI = projection of the 8 box corners
    C = np.array([[x, y, z] for x in (lo[0], hi[0]) for y in (lo[1], hi[1])
                  for z in (lo[2], hi[2])])
    Pc = (C - t) @ R  # base -> optical
    if np.any(Pc[:, 2] <= 0.1):
        return None, None
    us, vs = fx * Pc[:, 0] / Pc[:, 2] + cx, fy * Pc[:, 1] / Pc[:, 2] + cy
    u0, u1 = int(max(us.min() - 5, 0)), int(min(us.max() + 5, D.shape[1]))
    v0, v1 = int(max(vs.min() - 5, 0)), int(min(vs.max() + 5, D.shape[0]))
    if u1 - u0 < 3 or v1 - v0 < 3:
        return None, None
    v, u = np.mgrid[v0:v1, u0:u1]
    z = D[v0:v1, u0:u1]
    ok = np.isfinite(z) & (z > 0.2) & (z < 2.5)
    ok = ok & (M2[v0:v1, u0:u1] > 0)  # mask gates every point, no fallback
    u, v, z = u[ok], v[ok], z[ok]
    P = (sc2 * np.stack([(u - cx) * z / fx, (v - cy) * z / fy, z], 1)) @ R.T + t
    P = P[np.all((P > lo) & (P < hi), 1)]
    if len(P) < 50:
        return _z2_miss()  # geometry miss: continuity hold applies
    P = _cap_points(P, max_pts)
    _z2_hold['pts'], _z2_hold['wall'] = P.copy(), now
    return P, 'mask'


def _support_z(M0, D, intr, R, t, W):
    """Height of the surface the object rests on, from the SAME depth frame:
    a ring 7-30 px around the mask, back-projected to fr3_link0, kept within
    25 cm (xy) of the object and >= 2 cm below its lowest seen points. ZED1
    sees the drill at the image's bottom edge (base out of frame, 2026-11-04),
    so the cloud alone stops ~7 cm above the table. None if < 80 ring points."""
    fx, fy, cx, cy = intr
    ring = cv2.dilate(M0, np.ones((61, 61), np.uint8)) & \
        (1 - cv2.dilate(M0, np.ones((15, 15), np.uint8)))
    vs, us = np.nonzero(ring)
    z = D[vs, us]
    ok = np.isfinite(z) & (z > 0.15) & (z < 2.5)
    us, vs, z = us[ok], vs[ok], z[ok]
    if len(z) < 80:
        return None
    P = np.stack([(us - cx) * z / fx, (vs - cy) * z / fy, z], 1) @ R.T + t
    c = np.median(W, 0)
    zlow = float(np.percentile(W[:, 2], 2))
    keep = (np.hypot(P[:, 0] - c[0], P[:, 1] - c[1]) < 0.25) & \
        (P[:, 2] < zlow - 0.02)
    if int(keep.sum()) < 80:
        return None
    return float(np.median(P[keep, 2]))


def _drill_box(mask, info, tfbuf, track3d=None, tracking=False):
    """Drill model + live consistency check (see _target_model).

    The box itself is grasp-time data (live 3D from depth is unusable: the
    filtered depth has HOLES on the drill and interior medians read
    background, verified 0.686 m vs seed-time 0.582 m). Liveness comes from
    a 2D check that needs no depth: project the model center into the image
    and compare with the live mask centroid. Moved/stale -> dashboard
    paints the box red. Returns a dict or None (caller keeps last good).

    The served center/aabb follow the live tracker 3D point when fresh
    (2026-11-04, Murilo order: the dashboard box must track the drill);
    the grasp-time record is preserved under 'grasp_model', and the box
    falls back to it when the track is unusable. 'live_model' carries the
    live projection details (None when unusable); the 'live' block keeps
    the static-model consistency check.
    """
    import rclpy as _r
    run_id, model = _target_model()
    if model is None:
        return None
    M = np.frombuffer(mask.data, np.uint8).reshape(mask.height, mask.width)
    ys, xs = np.nonzero(M > 0)
    if len(xs) < 25:
        return None
    mx, my = float(xs.mean()), float(ys.mean())
    cov = 100.0 * len(xs) / M.size
    mage = time.time() - (mask.header.stamp.sec
                          + mask.header.stamp.nanosec * 1e-9)
    Kk = np.array(info.k, np.float64).reshape(3, 3)
    fx, fy, cx, cy = Kk[0, 0], Kk[1, 1], Kk[0, 2], Kk[1, 2]
    # NOTE: mask pixels are left-rectified 640x360; depth camera_info shares
    # the same rectified geometry (verified identical K/size in S3 use).
    sx = info.width / mask.width if mask.width else 1.0
    try:
        tf = tfbuf.lookup_transform(
            info.header.frame_id, 'fr3_link0', _r.time.Time(),
            timeout=_r.duration.Duration(seconds=0.5))
    except Exception:
        return None
    t, q = tf.transform.translation, tf.transform.rotation
    n = np.array([q.x, q.y, q.z, q.w], float)
    n /= (np.linalg.norm(n) or 1.0)
    x, y, zc, w = n
    R = np.array([[1 - 2 * (y * y + zc * zc), 2 * (x * y - zc * w), 2 * (x * zc + y * w)],
                  [2 * (x * y + zc * w), 1 - 2 * (x * x + zc * zc), 2 * (y * zc - x * w)],
                  [2 * (x * zc - y * w), 2 * (y * zc + x * w), 1 - 2 * (x * x + y * y)]])
    p = R @ np.array(model['center']) + np.array([t.x, t.y, t.z])
    if p[2] <= 0.05:
        return None
    pu, pv = (fx * p[0] / p[2] + cx) / sx, (fy * p[1] / p[2] + cy) / sx
    err = float(np.hypot(pu - mx, pv - my))
    status = 'live' if (mage < 5.0 and err < 40.0) else \
        ('stale' if mage >= 5.0 else 'moved')
    grasp_static = dict(center=model['center'],
                        aabb_min=model['aabb_min'],
                        aabb_max=model['aabb_max'],
                        err_px=round(err, 1), status=status)
    # Yaw follow (2026-11-04): live-mask PCA vs projected model axis.
    # The image-plane delta is applied as world yaw (approximation for
    # tabletop rotation; sign derived at runtime from projected axes).
    yaw = 0.0
    if len(xs) >= 100:
        try:
            C = np.cov(np.stack([xs.astype(float), ys.astype(float)]))
            w, V = np.linalg.eigh(C)
            if w[-1] > 2.0 * max(w[-2], 1e-9):
                la = V[:, -1]
                live_ang = float(np.arctan2(la[1], la[0]))
                mc = np.array(model['center'])
                ma = np.array(model.get('axis_xy', [1.0, 0.0]), float)
                tvec = np.array([t.x, t.y, t.z])

                def _proj(p3):
                    q = R @ p3 + tvec
                    return q, (float(fx * q[0] / q[2] + cx) / sx,
                              float(fy * q[1] / q[2] + cy) / sx)

                q0, (u0, v0) = _proj(mc)
                q1, (u1, v1) = _proj(mc + np.array([ma[0], ma[1], 0.0]))
                qx, (ux, vx) = _proj(mc + np.array([0.05, 0.0, 0.0]))
                qy, (uy, vy) = _proj(mc + np.array([0.0, 0.05, 0.0]))
                if min(q0[2], q1[2], qx[2], qy[2]) > 0.05:
                    mod_ang = float(np.arctan2(v1 - v0, u1 - u0))
                    d = (live_ang - mod_ang + np.pi / 2) % np.pi - np.pi / 2
                    ax = float(np.arctan2(vx - v0, ux - u0))
                    ay = float(np.arctan2(vy - v0, uy - u0))
                    d90 = (ay - ax + np.pi) % (2 * np.pi) - np.pi
                    s = 1.0 if d90 > 0 else -1.0
                    yaw = float(s * d)
                    _live_hold['yaw'] = yaw
        except Exception:
            pass
    if track3d is not None:
        t3_age = time.time() - track3d.get('wall', 0.0)
        if t3_age < 2.0:
            try:
                tf2 = tfbuf.lookup_transform(
                    'fr3_link0', track3d['frame'], _r.time.Time(),
                    timeout=_r.duration.Duration(seconds=0.5))
                R2, t2 = _tf_to_RT(tf2)
                c_new = R2 @ np.array(track3d['point'], float) + t2
                if -0.2 <= float(c_new[2]) <= 1.2:
                    _live_hold['center'] = [float(v) for v in c_new]
                    _live_hold['wall'] = time.time()
            except Exception:
                pass
    live_model = None
    hold_age = time.time() - _live_hold['wall']
    if _live_hold['center'] is not None and hold_age < _LIVE_HOLD_S:
        c_live = np.array(_live_hold['center'])
        pl = R @ c_live + np.array([t.x, t.y, t.z])
        if pl[2] > 0.05:
            qu = (fx * pl[0] / pl[2] + cx) / sx
            qv = (fy * pl[1] / pl[2] + cy) / sx
            qe = float(np.hypot(qu - mx, qv - my))
            live_model = dict(
                center_m=[round(float(v), 4) for v in c_live],
                age_s=round(hold_age, 1),
                held=hold_age >= 2.0,
                model_px=[round(qu, 1), round(qv, 1)],
                err_px=round(qe, 1),
                yaw_deg=round(float(np.degrees(yaw)), 1),
                status='live' if qe < 40.0 else 'moved')
    if live_model is not None:
        lc = np.array(live_model['center_m'])
        he = np.array(model['half_extents'])
        if abs(yaw) > 0.005:
            cy, sy = np.cos(yaw), np.sin(yaw)
            Rz = np.array([[cy, -sy, 0.0], [sy, cy, 0.0], [0.0, 0.0, 1.0]])
            cor = np.array([[ax * he[0], ay * he[1], az * he[2]]
                            for ax in (-1.0, 1.0)
                            for ay in (-1.0, 1.0)
                            for az in (-1.0, 1.0)])
            rot = (Rz @ cor.T).T + lc
            lo = [round(float(v), 4) for v in rot.min(0)]
            hi = [round(float(v), 4) for v in rot.max(0)]
        else:
            lo = [round(float(lc[i] - he[i]), 4) for i in range(3)]
            hi = [round(float(lc[i] + he[i]), 4) for i in range(3)]
        box_status = live_model['status']
        box_center = [round(float(v), 4) for v in lc]
    else:
        lo, hi = model['aabb_min'], model['aabb_max']
        box_status, box_center = status, model['center']
    return dict(ok=True, status=box_status, source_run=run_id,
                n_points=int(len(xs)), frame='fr3_link0',
                aabb_min=lo, aabb_max=hi,
                center=box_center, centroid=box_center,
                half_extents=model['half_extents'],
                grasp_model=grasp_static,
                live_model=live_model,
                live=dict(coverage_pct=round(cov, 2),
                          mask_age_s=round(mage, 1),
                          centroid_px=[round(mx, 1), round(my, 1)],
                          model_px=[round(pu, 1), round(pv, 1)],
                          reproj_err_px=round(err, 1)))


def _overlay_snap(img, m, age_s):
    """Mask overlay snapshot (shared by /snap.jpg and /snap2.jpg).

    img: HxWx3 RGB uint8; m: HxW uint8 mask (resized to img when needed).
    Returns (out_rgb_uint8, coverage_pct). Style: red tint on mask pixels,
    green bbox + centroid cross, white 'mask %/age' label top-left.
    """
    if m.shape != img.shape[:2]:
        m = cv2.resize(m, (img.shape[1], img.shape[0]),
                       interpolation=cv2.INTER_NEAREST)
    out = img.copy()
    out[m > 0] = (out[m > 0] * 0.45 + np.array([255, 0, 0]) * 0.55)
    ys, xs = np.nonzero(m > 0)
    cov = 100.0 * len(xs) / m.size
    if len(xs) > 20:
        x1, y1, x2, y2 = xs.min(), ys.min(), xs.max(), ys.max()
        cv2.rectangle(out, (int(x1), int(y1)), (int(x2), int(y2)),
                      (0, 255, 0), 2)
        cv2.drawMarker(out, (int(xs.mean()), int(ys.mean())), (0, 255, 0),
                       cv2.MARKER_CROSS, 20, 2)
    cv2.putText(out, f'mask {cov:.2f}% age {age_s:.1f}s', (10, 28),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
    return out, cov


def ros_main():
    rclpy.init()
    node = Node('seg_view')
    qbe = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT,
                     history=HistoryPolicy.KEEP_LAST)
    got = {}
    node.create_subscription(Image, '/vm/cable/rgb',
                             lambda m: got.__setitem__('rgb', m), qbe)
    # NOTE 2026-11-04: mask subscription stays BEST_EFFORT like the
    # publisher's other readers; a RELIABLE trial received briefly then
    # went dead, worse than BE's intermittent loss. See drill notes.
    node.create_subscription(Image, '/zed1/segmentation_mask',
                             lambda m: got.__setitem__('mask', m), qbe)
    from sensor_msgs.msg import CameraInfo
    import tf2_ros
    node.create_subscription(CameraInfo, '/zed1/zed_node/depth/camera_info',
                             lambda m: got.__setitem__('info', m), qbe)
    from geometry_msgs.msg import PointStamped
    node.create_subscription(PointStamped, '/tracking_3d_point',
                             lambda m: got.__setitem__('track3d', m), qbe)
    import collections
    depths = collections.deque(maxlen=60)  # ~2.4 s: SAM2 masks arrive 0.4-1 s late
    node.create_subscription(Image, '/vm/cable/depth', depths.append, qbe)
    # ZED2 (2026-11-04): VM-local republish from vm-cable-fanin2; info tiny
    depths2 = collections.deque(maxlen=30)  # mask-anchored pairing: depth must reach back to the mask stamp (~1 s lag) at 4-9 Hz subsample
    node.create_subscription(Image, '/vm/cable2/depth',
                             lambda m: depths2.append((time.time(), m)), qbe)
    node.create_subscription(Image, '/vm/cable2/rgb',
                             lambda m: got.__setitem__('rgb2', m), qbe)
    # ZED2 SAM2 mask (own video predictor in vm-tracker-prod): gates the
    # ZED2 cloud points. No box fallback: a pairing miss holds the last
    # mask-gated set (<=2 s) or yields nothing (see _zed2_points).
    masks2 = collections.deque(maxlen=5)
    node.create_subscription(Image, '/zed2/segmentation_mask',
                             lambda m: masks2.append((time.time(), m)), qbe)
    node.create_subscription(CameraInfo, '/zed2/zed_node/depth/camera_info',
                             lambda m: got.__setitem__('info2', m), qbe)
    tfbuf = tf2_ros.Buffer()
    tf2_ros.TransformListener(tfbuf, node)
    pub = node.create_publisher(Image, '/zed1/segmentation_overlay', qbe)
    frames, t_fps, fps = 0, time.time(), 0.0
    last_drill = 0.0
    last_snap2_wall, last_snap2_stamp = 0.0, -1.0
    while rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.1)
        if 'rgb' not in got or 'mask' not in got:
            continue
        rgb, mask = got['rgb'], got['mask']
        img = np.frombuffer(rgb.data, np.uint8).reshape(rgb.height, rgb.width, 3)
        m = np.frombuffer(mask.data, np.uint8).reshape(mask.height, mask.width)
        age = time.time() - (mask.header.stamp.sec
                             + mask.header.stamp.nanosec * 1e-9)
        out, cov = _overlay_snap(img, m, age)
        msg = Image()
        msg.header.stamp = node.get_clock().now().to_msg()
        msg.header.frame_id = rgb.header.frame_id
        msg.height, msg.width, msg.encoding = out.shape[0], out.shape[1], 'rgb8'
        msg.is_bigendian = 0
        msg.step = out.shape[1] * 3
        msg.data = out.tobytes()
        pub.publish(msg)
        ok, jpg = cv2.imencode('.jpg', cv2.cvtColor(out, cv2.COLOR_RGB2BGR),
                               [cv2.IMWRITE_JPEG_QUALITY, 80])
        frames += 1
        if time.time() - t_fps >= 2.0:
            fps = frames / (time.time() - t_fps)
            frames, t_fps = 0, time.time()
        with state['lock']:
            if ok:
                state['jpg'] = jpg.tobytes()
            state['cov'], state['age'], state['fps'] = cov, age, fps
            state['tracking'] = cov > 0.05 and age < 5.0
            tm = got.pop('track3d', None)  # consume: wall/age must reflect
            # topic freshness, not overlay-loop liveness
            if tm is not None:
                state['track3d'] = {
                    'point': [tm.point.x, tm.point.y, tm.point.z],
                    'frame': tm.header.frame_id,
                    'stamp': (tm.header.stamp.sec
                              + tm.header.stamp.nanosec * 1e-9),
                    'wall': time.time()}
        # ZED2 snapshot (2026-11-06): /snap2.jpg = cable2 RGB + SAM2 mask,
        # same style as /snap.jpg. Rendered only on a NEW mask stamp and at
        # most every 0.5 s (masks run ~1.5 Hz). Display-only: a bad frame
        # must never kill the loop, the next tick retries.
        if 'rgb2' in got and masks2:
            _m2s = masks2[-1][1].header.stamp
            _m2t = _m2s.sec + _m2s.nanosec * 1e-9
            if _m2t != last_snap2_stamp \
                    and time.time() - last_snap2_wall >= 0.5:
                last_snap2_stamp, last_snap2_wall = _m2t, time.time()
                try:
                    _r2 = got['rgb2']
                    _img2 = np.frombuffer(_r2.data, np.uint8).reshape(
                        _r2.height, _r2.width, 3)
                    _mm2m = masks2[-1][1]
                    _mm2 = np.frombuffer(_mm2m.data, np.uint8).reshape(
                        _mm2m.height, _mm2m.width)
                    _out2, _ = _overlay_snap(_img2, _mm2,
                                             time.time() - _m2t)
                    _ok2, _jpg2 = cv2.imencode(
                        '.jpg', cv2.cvtColor(_out2, cv2.COLOR_RGB2BGR),
                        [cv2.IMWRITE_JPEG_QUALITY, 80])
                    if _ok2:
                        with state['lock']:
                            state['jpg2'] = _jpg2.tobytes()
                except Exception:
                    pass
        if time.time() - last_drill >= 2.0 and 'mask' in got \
                and 'info' in got:
            last_drill = time.time()
            with state['lock']:
                _t3, _trk = state['track3d'], state['tracking']
            try:
                box = _drill_box(got['mask'], got['info'], tfbuf, _t3, _trk)
            except Exception:
                box = None
            try:
                W, support_z = _live_cloud(got['mask'], list(depths),
                                           got['info'], tfbuf) or (None, None)
                _lc_why = None if W is not None else 'returned None (no depth near mask stamp / too few pts)'
            except Exception as _e:
                W, support_z = None, None
                _lc_why = f'{type(_e).__name__}: {_e}'
            if _lc_why and time.time() - _lc_log.get('t', 0) > 30:
                _lc_log['t'] = time.time()
                node.get_logger().warn(f'live cloud miss: {_lc_why[:200]} (depth buf {len(depths)})')
            W2, z2src = None, None
            if W is not None:
                try:
                    W2, z2src = _zed2_points(W, support_z, depths2,
                                             got.get('info2'), tfbuf,
                                             masks2)
                except Exception:
                    W2, z2src = None, None
                if W2 is not None:
                    W = np.vstack([W, W2])
                with state['lock']:
                    state['live_cloud'] = {
                        'pts': W, 'wall': time.time(),
                        'n_zed2': 0 if W2 is None else int(len(W2)),
                        'zed2_T': _zed2_T.get('src'),
                        'zed2_src': z2src}
                if box is not None:  # box from the measured live cloud
                    lo = np.percentile(W, 2, axis=0)
                    hi = np.percentile(W, 98, axis=0)
                    box['support_z'] = None if support_z is None else \
                        round(support_z, 4)
                    if support_z is not None and support_z < lo[2]:
                        lo[2] = support_z  # rests on the table (base unseen)
                    c = (lo + hi) / 2.0
                    box['aabb_min'] = [round(float(v), 4) for v in lo]
                    box['aabb_max'] = [round(float(v), 4) for v in hi]
                    box['center'] = box['centroid'] = [
                        round(float(v), 4) for v in c]
                    box['half_extents'] = [
                        round(float(v), 4) for v in (hi - lo) / 2.0]
                    box['status'] = 'live'
                    box['box_source'] = 'live_cloud'
                    box['live_cloud_n'] = int(len(W))
                    box['live_cloud_n_zed2'] = 0 if W2 is None else int(len(W2))
                    box['live_cloud_zed2_src'] = z2src
            elif box is not None:
                # live cloud missed this tick (depth stamp gap): keep the last
                # live-cloud box up to 6 s rather than flash the 2D-model box
                # (~10 cm off, seen 2026-11-04)
                with state['lock']:
                    prev, lc = state['drill'], state.get('live_cloud')
                if prev.get('box_source') == 'live_cloud' and lc is not None \
                        and time.time() - lc['wall'] < 6.0:
                    box = None
            if box is not None:  # on miss keep last box; its age grows
                box['frame'] = 'fr3_link0'
                box['updated_wall'] = time.time()
                with state['lock']:
                    state['drill'] = box


if __name__ == '__main__':
    threading.Thread(target=ros_main, daemon=True).start()
    ThreadingHTTPServer(('0.0.0.0', PORT), Handler).serve_forever()
