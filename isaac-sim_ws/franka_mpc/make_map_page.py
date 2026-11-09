"""Build the interactive 3D map page (one self-contained HTML, three.js) from glb_export.py outputs.
   /usr/bin/python3 make_map_page.py <page.html> "<label>=<export_dir>:<mesh tag>" [...] [--paths <export_dir>] [--note "..."]
   each map: the nvblox mesh recorded in that run (main or extra instance); true objects + cameras come from the first.
"""
import argparse, base64, json, os

ap = argparse.ArgumentParser()
ap.add_argument("out"); ap.add_argument("maps", nargs="+")
ap.add_argument("--paths", default=""); ap.add_argument("--note", default="")
a = ap.parse_args()
b64 = lambda p: base64.b64encode(open(p, "rb").read()).decode()
maps, first = [], None
for spec in a.maps:
    label, rest = spec.split("=", 1); d, tag = rest.rsplit(":", 1); sc = json.load(open(os.path.join(d, "scene.json")))
    m = sc["meshes"][tag]; first = first or (d, sc)
    maps.append({"label": label, "glb": b64(os.path.join(d, m["file"])), "triangles": m["triangles"], "timeline": m["timeline"]})
d0, sc0 = first
data = {"maps": maps, "objects": b64(os.path.join(d0, sc0["true_objects"])), "cameras": sc0["cameras"], "note": a.note}
if a.paths:
    data["paths"] = json.load(open(os.path.join(a.paths, "scene.json"))).get("paths")
    data["paths_run"] = json.load(open(os.path.join(a.paths, "scene.json")))["run"]

HTML = r"""<title>Franka Workcell Map</title>
<meta name="claude:viewport-fit-sides" content="cover">
<link rel="preconnect" href="https://fonts.googleapis.com">
<link rel="stylesheet" href="https://fonts.googleapis.com/css2?family=IBM+Plex+Mono:wght@400;500&family=IBM+Plex+Sans+Condensed:wght@500;600&family=IBM+Plex+Sans:wght@400;500&display=swap">
<style>
/* Layout: an RViz-like 3D viewport filling the screen, with one control column at the right (below it on phones). */
:root {
  --bg: #eef1f2; --panel: #f8fafa; --ink: #15201f; --ink2: #4c5a59; --line: #d3dbdb;
  --view: #dfe5e6; --grid: #b8c4c4; --accent: #0d7f80; --accent-ink: #ffffff;
  --cmd: #2a78d6; --exe: #eb6834;
  --display: "IBM Plex Sans Condensed", "Arial Narrow", system-ui, sans-serif;
  --body: "IBM Plex Sans", system-ui, -apple-system, "Segoe UI", sans-serif;
  --mono: "IBM Plex Mono", ui-monospace, "SFMono-Regular", Menlo, monospace;
}
@media (prefers-color-scheme: dark) { :root:not([data-theme="light"]) {
  --bg: #0f1415; --panel: #151c1d; --ink: #e4ecec; --ink2: #9fb0af; --line: #263233;
  --view: #1b2324; --grid: #344243; --accent: #31b4b2; --accent-ink: #062224; --cmd: #3987e5; --exe: #d95926; color-scheme: dark } }
:root[data-theme="dark"] {
  --bg: #0f1415; --panel: #151c1d; --ink: #e4ecec; --ink2: #9fb0af; --line: #263233;
  --view: #1b2324; --grid: #344243; --accent: #31b4b2; --accent-ink: #062224; --cmd: #3987e5; --exe: #d95926; color-scheme: dark }
html, body { height: 100% }
body { background: var(--bg); color: var(--ink); font: 14px/1.45 var(--body); display: flex; flex-direction: column;
  padding-left: max(16px, env(safe-area-inset-left, 0px)); padding-right: max(16px, env(safe-area-inset-right, 0px)); box-sizing: border-box }
header { display: flex; flex-wrap: wrap; align-items: baseline; gap: 4px 16px; padding-block: 12px 10px }
h1 { font: 600 20px/1.2 var(--display); letter-spacing: .01em; margin: 0; text-wrap: balance }
.sub { color: var(--ink2); font-size: 13px; min-width: 0 }
main { flex: 1; min-height: 0; display: grid; grid-template-columns: minmax(0, 1fr) 300px; gap: 12px; padding-block: 0 14px }
#view { position: relative; min-height: 360px; border-radius: 6px; overflow: hidden; background: var(--view); border: 1px solid var(--line) }
#view canvas { display: block; width: 100%; height: 100%; touch-action: none }
#hud { position: absolute; left: 10px; bottom: 8px; font: 12px var(--mono); color: var(--ink2); pointer-events: none; font-variant-numeric: tabular-nums }
#loading { position: absolute; inset: 0; display: grid; place-items: center; color: var(--ink2); font: 13px var(--mono) }
aside { background: var(--panel); border: 1px solid var(--line); border-radius: 6px; padding: 14px; display: flex; flex-direction: column;
  gap: 16px; overflow-y: auto; min-width: 0 }
.group { display: flex; flex-direction: column; gap: 8px }
.label { font: 500 11px var(--mono); text-transform: uppercase; letter-spacing: .08em; color: var(--ink2) }
.seg { display: flex; flex-direction: column; gap: 6px }
.seg button, .presets button, #play { font: 500 13px var(--body); color: var(--ink); background: transparent; border: 1px solid var(--line);
  border-radius: 4px; padding: 7px 10px; text-align: left; cursor: pointer }
.seg button[aria-pressed="true"] { background: var(--accent); color: var(--accent-ink); border-color: var(--accent) }
.seg button small { display: block; font: 12px var(--mono); opacity: .8 }
.presets { display: grid; grid-template-columns: repeat(2, minmax(0, 1fr)); gap: 6px }
button:focus-visible, input:focus-visible { outline: 2px solid var(--accent); outline-offset: 2px }
.check { display: flex; align-items: center; gap: 8px; cursor: pointer }
.sw { width: 10px; height: 10px; border-radius: 2px; flex: none }
.row { display: flex; align-items: center; gap: 8px }
#t { flex: 1; accent-color: var(--accent); min-width: 0 }
#tval { font: 12px var(--mono); color: var(--ink2); font-variant-numeric: tabular-nums; min-width: 9ch; text-align: right }
.note { font-size: 13px; color: var(--ink2); margin: 0 }
.keys { font: 12px var(--mono); color: var(--ink2); margin: 0 }
@media (max-width: 760px) { main { grid-template-columns: minmax(0, 1fr); } #view { min-height: 62vh } aside { overflow: visible } }
@media (prefers-reduced-motion: reduce) { * { scroll-behavior: auto } }
[hidden] { display: none !important }
</style>
<header><h1>Franka Workcell Map</h1><span class="sub" id="sub">nvblox reconstruction from the simulated ZED cameras, robot base frame</span></header>
<main>
  <div id="view"><div id="loading">loading map…</div><div id="hud"></div></div>
  <aside>
    <div class="group"><span class="label">Map</span><div class="seg" id="maps"></div></div>
    <div class="group"><span class="label">Reconstruction</span>
      <div class="row"><button id="play" type="button" aria-label="Play the reconstruction">Play</button>
        <input id="t" type="range" min="0" max="1000" value="1000" aria-label="Reconstruction time"><span id="tval"></span></div>
      <p class="note">Blocks appear when nvblox first meshes them (sim time); each shows its final shape.</p></div>
    <div class="group"><span class="label">Show</span>
      <label class="check"><input type="checkbox" id="shade"> shaded geometry (instead of camera colour)</label>
      <label class="check"><input type="checkbox" id="objs" checked><span class="sw" style="background:#2850c8"></span><span class="sw" style="background:#e67828"></span> true object surfaces (Isaac)</label>
      <label class="check"><input type="checkbox" id="cams" checked> ZED cameras</label>
      <label class="check" id="pathsrow"><input type="checkbox" id="paths" checked><span class="sw" style="background:var(--cmd)"></span>commanded <span class="sw" style="background:var(--exe)"></span>executed fingertips</label>
      <label class="check"><input type="checkbox" id="grid" checked> grid and base axes</label></div>
    <div class="group"><span class="label">View</span><div class="presets">
      <button type="button" data-v="iso">Overview</button><button type="button" data-v="top">Top</button>
      <button type="button" data-v="zed">From ZED 1</button><button type="button" data-v="zed2">From ZED 2</button>
      <button type="button" data-v="bottle">Blue bottle</button><button type="button" data-v="side">Side</button></div>
      <p class="keys">drag: rotate · right-drag or two fingers: pan · wheel or pinch: zoom</p></div>
    <p class="note" id="note"></p>
  </aside>
</main>
<script src="https://cdn.jsdelivr.net/npm/three@0.147.0/build/three.min.js"></script>
<script src="https://cdn.jsdelivr.net/npm/three@0.147.0/examples/js/controls/OrbitControls.js"></script>
<script src="https://cdn.jsdelivr.net/npm/three@0.147.0/examples/js/loaders/GLTFLoader.js"></script>
<script>
const DATA = __DATA__;
const css = n => getComputedStyle(document.documentElement).getPropertyValue(n).trim();
const view = document.getElementById("view");
const renderer = new THREE.WebGLRenderer({ antialias: true }); renderer.setPixelRatio(Math.min(2, devicePixelRatio));
renderer.outputEncoding = THREE.sRGBEncoding; view.prepend(renderer.domElement);
const scene = new THREE.Scene(); const camera = new THREE.PerspectiveCamera(45, 1, 0.01, 50);
const controls = new THREE.OrbitControls(camera, renderer.domElement); controls.enableDamping = true; controls.dampingFactor = 0.12;
scene.add(new THREE.HemisphereLight(0xffffff, 0x404040, 0.9)); const sun = new THREE.DirectionalLight(0xffffff, 0.7); sun.position.set(1.5, 3, 1); scene.add(sun);
const B = p => new THREE.Vector3(p[0], p[2], -p[1]);           // robot base frame (x fwd, y left, z up) -> three (y up)
const unb64 = s => { const b = atob(s), u = new Uint8Array(b.length); for (let i = 0; i < b.length; i++) u[i] = b.charCodeAt(i); return u.buffer; };
const loader = new THREE.GLTFLoader(); const parse = b => new Promise((ok, err) => loader.parse(unb64(b), "", ok, err));
const helpers = new THREE.Group(); scene.add(helpers);
function themeColors() { scene.background = new THREE.Color(css("--view")); grid.material.color.set(css("--grid")); }
const grid = new THREE.GridHelper(3, 30); grid.position.set(0.4, 0, 0); helpers.add(grid);   // table top plane, 10 cm cells
const axes = new THREE.Group(); [[1,0,0,0xd8453b],[0,1,0,0x35a852],[0,0,1,0x3b6fd8]].forEach(([x,y,z,c]) => {
  const g = new THREE.BufferGeometry().setFromPoints([B([0,0,0]), B([x*.15,y*.15,z*.15])]);
  axes.add(new THREE.Line(g, new THREE.LineBasicMaterial({ color: c }))); }); helpers.add(axes);
const camGroup = new THREE.Group(); scene.add(camGroup);
for (const [name, c] of Object.entries(DATA.cameras)) {      // a small frustum per ZED (left camera), 25 cm deep
  const p = new THREE.Vector3(...c.pos), f = new THREE.Vector3(...c.fwd).normalize(), u = new THREE.Vector3(...c.up).normalize();
  const r = new THREE.Vector3().crossVectors(f, u).normalize(), d = 0.25, w = d * 0.65, h = d * 0.37, pts = [];
  const cn = [[1,1],[-1,1],[-1,-1],[1,-1]].map(([a, b]) => p.clone().addScaledVector(f, d).addScaledVector(r, a*w).addScaledVector(u, b*h));
  cn.forEach((q, i) => { pts.push(p, q, q, cn[(i + 1) % 4]); });
  const ln = new THREE.LineSegments(new THREE.BufferGeometry().setFromPoints(pts), new THREE.LineBasicMaterial({ color: 0x0d7f80 }));
  camGroup.add(ln);
  const box = new THREE.Mesh(new THREE.BoxGeometry(0.19, 0.045, 0.035), new THREE.MeshLambertMaterial({ color: 0x2b2f31 }));
  box.position.copy(p); box.lookAt(p.clone().add(f)); camGroup.add(box);
}
let pathGroup = null;
if (DATA.paths) { pathGroup = new THREE.Group(); scene.add(pathGroup);
  [["commanded", "--cmd"], ["executed", "--exe"]].forEach(([k, tok]) => {
    const g = new THREE.BufferGeometry().setFromPoints(DATA.paths[k].map(q => new THREE.Vector3(...q)));
    const l = new THREE.Line(g, new THREE.LineBasicMaterial({ color: css(tok) })); l.userData.tok = tok; pathGroup.add(l); });
} else document.getElementById("pathsrow").hidden = true;
let objPoints = null, maps = [], cur = 0;
const shade = document.getElementById("shade");
function material() { return shade.checked ? new THREE.MeshLambertMaterial({ color: 0xc9d1d2, side: THREE.DoubleSide })
                                           : new THREE.MeshBasicMaterial({ vertexColors: true, side: THREE.DoubleSide }); }
async function load() {
  for (const m of DATA.maps) {
    const g = await parse(m.glb); let mesh = null; g.scene.traverse(o => { if (o.isMesh && !mesh) mesh = o; });
    mesh.geometry.computeVertexNormals(); mesh.material = material(); mesh.visible = false; scene.add(mesh); maps.push({ ...m, mesh });
  }
  const og = await parse(DATA.objects); og.scene.traverse(o => { if (o.isPoints) objPoints = o; });
  objPoints.material = new THREE.PointsMaterial({ size: 0.0025, vertexColors: true, transparent: true, opacity: 0.75 }); scene.add(objPoints);
  document.getElementById("loading").remove(); buildMapButtons(); select(maps.length - 1); setView("iso"); }
function buildMapButtons() { const box = document.getElementById("maps");
  maps.forEach((m, i) => { const b = document.createElement("button"); b.type = "button"; b.id = "map" + i;
    b.innerHTML = m.label + "<small>" + m.triangles.toLocaleString() + " triangles</small>"; b.onclick = () => select(i); box.append(b); }); }
function select(i) { cur = i; maps.forEach((m, j) => { m.mesh.visible = i === j; document.getElementById("map" + j).setAttribute("aria-pressed", String(i === j)); });
  applyTime(); }
const tIn = document.getElementById("t"), tVal = document.getElementById("tval");
function applyTime() { const m = maps[cur]; if (!m) return; const tl = m.timeline, T = tl.t[tl.t.length - 1], t = T * tIn.value / 1000;
  let k = 0; while (k < tl.t.length - 1 && tl.t[k + 1] <= t) k++;
  const n = tIn.value >= 1000 ? m.triangles : tl.count[k]; m.mesh.geometry.setDrawRange(0, n * 3);
  tVal.textContent = "t " + t.toFixed(1) + " s"; hud(); }
tIn.oninput = applyTime;
let playing = null; const playB = document.getElementById("play");
playB.onclick = () => { if (playing) { clearInterval(playing); playing = null; playB.textContent = "Play"; return; }
  if (+tIn.value >= 1000) tIn.value = 0; playB.textContent = "Pause";
  playing = setInterval(() => { tIn.value = Math.min(1000, +tIn.value + 8); applyTime(); if (+tIn.value >= 1000) playB.onclick(); }, 40); };
shade.onchange = () => maps.forEach(m => { m.mesh.material.dispose(); m.mesh.material = material(); });
const tog = (id, f) => { const e = document.getElementById(id); e.onchange = () => f(e.checked); };
tog("objs", v => objPoints && (objPoints.visible = v)); tog("cams", v => camGroup.visible = v);
tog("paths", v => pathGroup && (pathGroup.visible = v)); tog("grid", v => helpers.visible = v);
const VIEWS = { iso: [[1.45, -1.15, 1.0], [0.42, 0, 0.12]], top: [[0.42, 0.0, 2.1], [0.42, 0, 0.0]], side: [[0.45, -1.7, 0.45], [0.45, 0, 0.18]],
  bottle: [[0.9, -0.38, 0.5], [0.55, 0.02, 0.2]] };
function setView(v) { let eye, at;
  if (v === "zed" || v === "zed2") { const c = DATA.cameras[v]; if (!c) return; eye = new THREE.Vector3(...c.pos);
    at = eye.clone().addScaledVector(new THREE.Vector3(...c.fwd).normalize(), 1.0); }
  else { eye = B(VIEWS[v][0]); at = B(VIEWS[v][1]); }
  camera.position.copy(eye); controls.target.copy(at); camera.up.set(0, 1, 0); controls.update(); }
document.querySelectorAll(".presets button").forEach(b => b.onclick = () => setView(b.dataset.v));
const hudEl = document.getElementById("hud");
function hud() { const m = maps[cur]; if (!m) return; const n = m.mesh.geometry.drawRange.count / 3;
  hudEl.textContent = m.label + " · " + Math.min(n, m.triangles).toLocaleString() + " / " + m.triangles.toLocaleString() + " triangles"; }
function resize() { const w = view.clientWidth, h = view.clientHeight; renderer.setSize(w, h, false); camera.aspect = w / h; camera.updateProjectionMatrix(); }
new ResizeObserver(resize).observe(view);
const mq = matchMedia("(prefers-color-scheme: dark)"); const retheme = () => { themeColors(); if (pathGroup) pathGroup.children.forEach(l => l.material.color.set(css(l.userData.tok))); };
mq.addEventListener("change", retheme); new MutationObserver(retheme).observe(document.documentElement, { attributes: true, attributeFilter: ["data-theme"] });
themeColors(); resize();
if (DATA.note) document.getElementById("note").textContent = DATA.note;
if (DATA.paths_run) document.getElementById("sub").textContent += " · paths from " + DATA.paths_run;
(function loop() { requestAnimationFrame(loop); controls.update(); renderer.render(scene, camera); })();
load().catch(e => { document.getElementById("loading").textContent = "Could not load the map: " + e.message; });
</script>
"""
open(a.out, "w").write(HTML.replace("__DATA__", json.dumps(data)))
print(a.out, f"{os.path.getsize(a.out) / 1e6:.1f} MB")
