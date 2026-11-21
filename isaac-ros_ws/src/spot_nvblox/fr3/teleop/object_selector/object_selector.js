/* Grasp-target selector (2026-11-21). The server decides readiness; this page
   only shows it. Choose a button, then click the object on the ZED1 image. */
(() => {
  const root = document.getElementById('object-selector');
  if (!root) return;
  const TARGETS = ['drill', 'tennis_ball', 'sprite'];
  const buttons = Array.from(root.querySelectorAll('[data-object-target]'));
  const status = root.querySelector('[data-object-status]');
  const details = root.querySelector('[data-object-details]');
  const hint = root.querySelector('[data-object-hint]');
  const canvas = root.querySelector('[data-object-canvas]');
  const ctx = canvas.getContext('2d');
  const check = k => root.querySelector(`[data-object-check="${k}"]`);
  let last = null, pending = null, posting = false;
  const rgb = new Image(), mask = new Image();
  let maskOk = false;

  function draw() {
    if (!rgb.complete || !rgb.naturalWidth) return;
    ctx.drawImage(rgb, 0, 0, canvas.width, canvas.height);
    if (maskOk && mask.naturalWidth) {
      const off = document.createElement('canvas');
      off.width = canvas.width; off.height = canvas.height;
      const o = off.getContext('2d');
      o.drawImage(mask, 0, 0, off.width, off.height);
      const d = o.getImageData(0, 0, off.width, off.height);
      for (let i = 0; i < d.data.length; i += 4) {
        const on = d.data[i] > 127;
        d.data[i] = 255; d.data[i + 1] = 40; d.data[i + 2] = 40; d.data[i + 3] = on ? 110 : 0;
      }
      o.putImageData(d, 0, 0);
      ctx.drawImage(off, 0, 0);
    }
    if (pending) {
      ctx.strokeStyle = '#facc15'; ctx.lineWidth = 3;
      ctx.strokeRect(1, 1, canvas.width - 2, canvas.height - 2);
    }
  }
  rgb.onload = draw;
  mask.onload = () => { maskOk = true; draw(); };
  mask.onerror = () => { maskOk = false; draw(); };
  function refreshImages() {
    const t = Date.now();
    rgb.src = `snap?topic=/zed1/zed_node/rgb/image_rect_color&t=${t}`;
    mask.src = `snap?topic=/vm/view_mask&t=${t}`;
  }

  function setButtons() {
    const can = !!(last && last.can_select) && !posting;
    buttons.forEach(b => {
      b.disabled = !can;
      b.setAttribute('aria-pressed', String(b.dataset.objectTarget === (pending || (last && last.selected))));
    });
  }

  function render(d) {
    if (!d || typeof d.phase !== 'string') throw Error('invalid selector status');
    if (d.selected != null && !TARGETS.includes(d.selected)) throw Error('invalid target');
    last = d;
    const name = d.selected ? d.targets[d.selected].label : 'none';
    const cls = d.phase === 'ready' ? (d.y_enabled ? 'on' : 'warn') : d.phase === 'error' ? 'bad' : 'warn';
    status.className = `pill ${cls}`;
    status.textContent = `${name} · ${d.phase}${d.phase === 'ready' ? (d.y_enabled ? ' · Y ready' : ' · Y locked') : ''}`;
    for (const k of ['mask', 'map', 'grasp']) {
      const c = d[k] || {};
      check(k).textContent = `${c.ok ? 'OK' : '—'} ${c.detail || ''}`;
      check(k).style.color = c.ok ? 'var(--live)' : 'var(--warn)';
    }
    const t = d.tracking || {};
    details.textContent = `${d.reason}` + (t.tracking ? ` · tracker ${t.tracking}, map gate ${t.map_state || '?'}` : '') +
      (d.can_select ? '' : ` · selection blocked: ${d.select_block || 'busy'}`);
    if (!pending) hint.textContent = d.can_select ? 'Pick an object, then click it in the image.' :
      `Selection unavailable: ${d.select_block || 'busy'}`;
    setButtons();
  }

  async function update() {
    try {
      const r = await fetch('object-target', {cache: 'no-store', signal: AbortSignal.timeout(3000)});
      if (!r.ok) throw Error(`HTTP ${r.status}`);
      render(await r.json());
    } catch (e) {
      last = null; setButtons();
      status.className = 'pill bad'; status.textContent = 'Object selection unavailable';
      details.textContent = String(e.message || e);
    }
  }

  buttons.forEach(b => b.addEventListener('click', () => {
    if (b.disabled) return;
    pending = b.dataset.objectTarget;
    hint.textContent = `Now click the ${last.targets[pending].label.toLowerCase()} in the image (Esc cancels).`;
    setButtons(); draw();
  }));
  document.addEventListener('keydown', e => { if (e.key === 'Escape' && pending) { pending = null; update(); draw(); } });

  canvas.addEventListener('click', async ev => {
    if (!pending || posting) return;
    const r = canvas.getBoundingClientRect();
    const click = [(ev.clientX - r.left) / r.width, (ev.clientY - r.top) / r.height];
    const target = pending;
    posting = true; setButtons();
    hint.textContent = `Switching to ${last.targets[target].label}… (about 30–90 s)`;
    try {
      const resp = await fetch('object-target', {method: 'POST', cache: 'no-store',
        headers: {'Content-Type': 'application/json', 'X-Dashboard': '1'},
        body: JSON.stringify({target, click}), signal: AbortSignal.timeout(30000)});
      const d = await resp.json().catch(() => ({}));
      if (!resp.ok || d.ok !== true) throw Error(d.error || `HTTP ${resp.status}`);
    } catch (e) {
      hint.textContent = `Selection refused: ${e.message}`;
    } finally {
      pending = null; posting = false; await update(); draw();
    }
  });

  setButtons(); update(); refreshImages();
  setInterval(update, 2000);
  setInterval(refreshImages, 1000);
})();
