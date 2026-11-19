#!/usr/bin/env python3
"""Patch the FR3 lab dashboard with working camera routes and calibrated ZED markers."""
from pathlib import Path
import sys

path = Path(sys.argv[1] if len(sys.argv) > 1 else "lab_dashboard.html")
html = path.read_text()

def replace_once(old, new, label):
    global html
    count = html.count(old)
    if count == 0 and new in html:
        return
    if count != 1:
        raise SystemExit(f"Expected one {label} anchor, found {count}")
    html = html.replace(old, new, 1)

replace_once(
    "const cams = { cam1: '/zed1/zed_node/left/image_rect_color', cam2: '/zed2/zed_node/left/image_rect_color',",
    "const cams = { cam1: '/zed1/zed_node/rgb/image_rect_color', cam2: '/zed2/zed_node/rgb/image_rect_color',",
    "ZED RGB topic routes",
)
replace_once(
    "const STREAM_BASE = 'http://' + location.hostname + ':8787/stream?topic=';",
    "const STREAM_BASE = 'http://' + location.hostname + ':8787/stream?topic=';\nconst SAME_ORIGIN_CAMERAS = HTTPS || !['localhost', '127.0.0.1'].includes(location.hostname);",
    "camera route mode",
)
replace_once(
    "if (HTTPS || key === 'cam3' || key === 'cam4') {",
    "if (SAME_ORIGIN_CAMERAS || key === 'cam3' || key === 'cam4') {",
    "camera proxy branch",
)
replace_once(
    '<label><input type="checkbox" id="showRobot" checked> robot</label>',
    '<label><input type="checkbox" id="showRobot" checked> robot</label>\n      <label title="Calibrated deployed camera poses in fr3_link0"><input type="checkbox" id="showCameras" checked> ZED poses</label>',
    "camera visibility toggle",
)
marker = """// calibrated deployed ZED poses in fr3_link0, in metres
const zedPoses = [
  {name: 'ZED1', color: 0x22d3ee, T: [
     0.9316436344, -0.0637988354,  0.3577287104,  0.2189345235,
    -0.3454323734, -0.4610117079,  0.8174011111, -0.5994726926,
     0.1127678807, -0.8850975558, -0.4515367267,  0.6314107388]},
  {name: 'ZED2', color: 0xfb923c, T: [
    -0.8711057313,  0.3845373283, -0.3054600597,  0.7841657940,
     0.4910954969,  0.6819233027, -0.5420385799,  0.3635524722,
    -0.0001337346, -0.6221829734, -0.7828718476,  0.8054793167]},
];
const zedMarkers = new THREE.Group(); scene.add(zedMarkers);
for (const z of zedPoses) {
  const g = new THREE.Group();
  const T = z.T;
  const R = new THREE.Matrix4().set(
    T[0], T[1], T[2], 0, T[4], T[5], T[6], 0,
    T[8], T[9], T[10], 0, 0, 0, 0, 1);
  g.position.set(T[3], T[7], T[11]);
  g.quaternion.setFromRotationMatrix(R);
  g.add(new THREE.AxesHelper(0.075));
  const v = new Float32Array([
    0,0,0, -.09,-.06,.16, 0,0,0, .09,-.06,.16,
    0,0,0, .09,.06,.16, 0,0,0, -.09,.06,.16,
    -.09,-.06,.16, .09,-.06,.16, .09,-.06,.16, .09,.06,.16,
    .09,.06,.16, -.09,.06,.16, -.09,.06,.16, -.09,-.06,.16]);
  const fg = new THREE.BufferGeometry();
  fg.setAttribute('position', new THREE.BufferAttribute(v, 3));
  g.add(new THREE.LineSegments(fg, new THREE.LineBasicMaterial({color:z.color})));
  g.add(new THREE.Mesh(new THREE.SphereGeometry(0.018, 14, 10),
    new THREE.MeshBasicMaterial({color:z.color})));
  const canvas = document.createElement('canvas'); canvas.width = 128; canvas.height = 40;
  const ctx = canvas.getContext('2d'); ctx.font = 'bold 26px sans-serif';
  ctx.fillStyle = '#' + z.color.toString(16).padStart(6, '0'); ctx.fillText(z.name, 3, 29);
  const label = new THREE.Sprite(new THREE.SpriteMaterial({
    map: new THREE.CanvasTexture(canvas), transparent: true, depthTest: false}));
  label.position.set(0, 0, 0.035); label.scale.set(0.12, 0.038, 1); g.add(label);
  zedMarkers.add(g);
}
$('showCameras').onchange = () => { zedMarkers.visible = $('showCameras').checked; };
"""
if "const zedPoses = [" not in html:
    replace_once("scene.add(new THREE.AxesHelper(0.2));",
                 "scene.add(new THREE.AxesHelper(0.2));\n" + marker,
                 "ZED pose markers")
path.write_text(html)
print(f"patched {path}")
