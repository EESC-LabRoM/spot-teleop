/* Teleop source toggle: joystick <-> ZED wrist target (cuRobo).
 * Isolated <script type="module"> block for lab_dashboard.html — paste as its
 * own block, never into the main map module (an error here must not kill the
 * scene). Creates its own button row; does not reuse #showDrill; does not
 * touch the drill box, /drill.json, or the map.
 *
 * Talks to the joystick bridge on :8784 (GET /status is already CORS-open
 * and bound on 100.88.134.127; POST /source is the candidate endpoint in
 * bridge_source_endpoint.py — until it is deployed the buttons show the
 * bridge state read-only and switching stays operator-side).
 * node --check this file after any edit.
 */
const BRIDGE = 'http://100.88.134.127:8784';

function el(id, html) {
  let n = document.getElementById(id);
  if (!n) {
    n = document.createElement('div');
    n.id = id;
    n.style.cssText = 'font:12px system-ui,sans-serif;color:#222;margin:6px 0;';
    n.innerHTML = html;
    document.body.prepend(n);
  }
  return n;
}

function renderSourceToggle(state) {
  const box = el('source-toggle',
    '<span>Teleop source: <b id="src-current">?</b> ' +
    '<span id="src-reason" style="color:#666"></span></span> ' +
    '<button id="src-joy" type="button">Joystick</button> ' +
    '<button id="src-zed" type="button">ZED wrist</button>');
  const cur = box.querySelector('#src-current');
  const reason = box.querySelector('#src-reason');
  const bJoy = box.querySelector('#src-joy');
  const bZed = box.querySelector('#src-zed');
  cur.textContent = state.source || state.input_source || '?';
  reason.textContent = (state.engaged ? 'ENGAGED — release Start to switch'
                                     : (state.reason || ''));
  const locked = !!state.engaged;
  bJoy.disabled = locked;
  bZed.disabled = locked;
  bJoy.title = locked ? 'Release Start first' : 'Switch to joystick target';
  bZed.title = locked ? 'Release Start first' : 'Switch to ZED wrist target (starts calibration)';
  bJoy.onclick = () => setSource('joystick');
  bZed.onclick = () => {
    if (!confirm('Switch to ZED wrist target? This starts arm calibration.')) return;
    setSource('zed');
  };
}

async function setSource(wanted) {
  try {
    const r = await fetch(BRIDGE + '/source', {
      method: 'POST',
      headers: {'Content-Type': 'application/json'},
      body: JSON.stringify({source: wanted}),
    });
    const j = await r.json();
    alert((j.ok ? 'OK: ' : 'Refused: ') + (j.message || r.status));
    pollSource();
  } catch (e) {
    alert('Switch endpoint unreachable (not yet deployed?): ' + e);
  }
}

async function pollSource() {
  try {
    const r = await fetch(BRIDGE + '/status', {cache: 'no-store'});
    renderSourceToggle(await r.json());
  } catch (e) { /* bridge down: leave last state, retry next tick */ }
}

pollSource();
setInterval(pollSource, 1000);
