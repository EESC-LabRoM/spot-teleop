/* Staged UI: target-selection service must prove readiness server-side. */
(() => {
  const root = document.getElementById('object-selector');
  if (!root) return;
  const buttons = Array.from(root.querySelectorAll('[data-object-target]'));
  const status = root.querySelector('[data-object-status]');
  const details = root.querySelector('[data-object-details]');
  let available = false, busy = false;
  function disabled(value) { buttons.forEach(b => { b.disabled = value; }); }
  function render(data) {
    if (!data || typeof data !== 'object' || typeof data.status !== 'string') throw Error('Invalid selector status');
    const targets = ['drill', 'tennis_ball', 'sprite'];
    if (data.selected != null && !targets.includes(data.selected)) throw Error('Invalid selected target');
    status.textContent = data.selected ? `${data.selected} · ${data.status}` : 'No object selected';
    const checks = ['mask', 'map', 'grasp'].map(k => `${k}: ${data[k]?.ok === true ? 'OK' : 'waiting'}`).join(' · ');
    details.textContent = `${checks} — ${data.reason || 'Validation pending'}`;
    buttons.forEach(b => b.setAttribute('aria-pressed', String(b.dataset.objectTarget === data.selected)));
    available = data.can_select === true;
    disabled(!available || busy);
  }
  async function update() {
    try {
      const r = await fetch('object-target', {cache:'no-store', signal:AbortSignal.timeout(2500)});
      if (!r.ok) throw Error(`HTTP ${r.status}`);
      render(await r.json());
    } catch (_) {
      available = false; disabled(true);
      status.textContent = 'Object selection unavailable';
      details.textContent = 'Live target and grasp have not been verified.';
    }
  }
  buttons.forEach(b => b.addEventListener('click', async () => {
    if (!available || busy) return;
    busy = true; disabled(true); status.textContent = 'Preparing target…';
    try {
      const r = await fetch('object-target', {method:'POST',headers:{'Content-Type':'application/json','X-Dashboard':'1'},body:JSON.stringify({target:b.dataset.objectTarget}),signal:AbortSignal.timeout(5000)});
      const data = await r.json();
      if (!r.ok || data.ok !== true) throw Error(data.error || data.reason || `HTTP ${r.status}`);
      details.textContent = 'Selection requested. Waiting for mask, map and grasp checks.';
    } catch (e) { details.textContent = `Selection failed: ${e.message}`; }
    finally { busy = false; await update(); }
  }));
  disabled(true); update(); setInterval(update, 2000);
})();
