// Smoke test for NASA-TLX-teleop.html — run:  node smoke_test.js
// Dependency-free: stubs a minimal DOM, runs the form's exact embedded
// script, simulates a full operator fill, and asserts the live score,
// sticky bar, submit payload, and CSV export. Exit 0 = pass.
const fs = require("fs");
const path = require("path");
const vm = require("vm");

const html = fs.readFileSync(path.join(__dirname, "NASA-TLX-teleop.html"), "utf8");
const script = html.split("<script>")[1].split("</script>")[0];

// ---- minimal DOM stub ----
const radios = []; // {name, value, checked}
const texts = {};
["operator", "date", "condition", "task"].forEach(n => { texts[n] = { name: n, value: "" }; });
const byId = {};
function mkEl(id) {
  const el = { id, textContent: "", style: {}, value: "",
    appendChild() {}, click() {} };
  byId[id] = el;
  return el;
}
["scales", "pairs", "live", "status", "stickybar"].forEach(mkEl);
["mental", "physical", "temporal", "performance", "effort", "frustration"]
  .forEach(id => ["t-r-", "t-w-", "t-p-"].forEach(p => mkEl(p + id)));
["t-rtotal", "t-wtotal", "t-grand"].forEach(mkEl);

function parseInputs(inner) {
  const re = /<input\s+([^>]*?)>/g;
  let m;
  while ((m = re.exec(inner))) {
    const attrs = m[1];
    const name = (/name="([^"]*)"/.exec(attrs) || [])[1];
    const value = (/value="([^"]*)"/.exec(attrs) || [])[1];
    if (name !== undefined) radios.push({ name, value, checked: false });
  }
  const idre = /id="([^"]*)"/g;
  while ((m = idre.exec(inner))) if (!byId[m[1]]) mkEl(m[1]);
}
function makeNode() {
  return {
    className: "", textContent: "", style: {},
    set innerHTML(v) { parseInputs(v); },
    get innerHTML() { return ""; },
    appendChild() {}, click() {},
  };
}
function selName(sel, key) {
  const m = new RegExp(key + '="([^"]*)"').exec(sel);
  return m && m[1];
}
const listeners = {};
const documentStub = {
  getElementById: id => {
    if (!byId[id]) throw new Error("missing element #" + id);
    return byId[id];
  },
  createElement: () => makeNode(),
  querySelector(sel) {
    if (sel.includes(":checked")) {
      const name = selName(sel, "name");
      return radios.find(r => r.name === name && r.checked) || null;
    }
    if (sel.includes(".inputs")) return texts[selName(sel, "name")] || null;
    if (sel.startsWith("input[name=")) {
      return radios.find(r => r.name === selName(sel, "name") &&
        r.value === selName(sel, "value")) || null;
    }
    throw new Error("unsupported selector: " + sel);
  },
  querySelectorAll(sel) {
    if (sel === '.inputs input[type="text"]') return Object.values(texts);
    if (sel === 'input[type="radio"]:checked') return radios.filter(r => r.checked);
    throw new Error("unsupported selectorAll: " + sel);
  },
  addEventListener: (ev, fn) => { (listeners[ev] = listeners[ev] || []).push(fn); },
};
const store = {};
const localStorageStub = {
  getItem: k => (k in store ? store[k] : null),
  setItem: (k, v) => { store[k] = String(v); },
  removeItem: k => { delete store[k]; },
};
let fetched = null;
const fetchStub = async (url, opts) => {
  fetched = { url, opts };
  return { ok: true, status: 200, json: async () => ({ ok: true, n: 7 }) };
};
let blobText = null;
function BlobStub(parts) { blobText = parts.join(""); }
const URLStub = { createObjectURL: () => "blob:fake", revokeObjectURL: () => {} };

// ---- run the form's exact script ----
const ctx = vm.createContext({
  document: documentStub, localStorage: localStorageStub, CSS: { escape: s => s },
  fetch: fetchStub, Blob: BlobStub, URL: URLStub, console,
});
vm.runInContext(script, ctx, { filename: "NASA-TLX-teleop-embedded.js" });

// ---- assertions ----
let failures = 0;
function check(name, cond, extra) {
  console.log((cond ? "PASS " : "FAIL ") + name + (cond || extra === undefined ? "" : " — " + extra));
  if (!cond) failures++;
}
function setRadio(name, value) {
  const r = radios.find(r => r.name === name && String(r.value) === String(value));
  if (!r) throw new Error(`no radio ${name}=${value}`);
  radios.forEach(x => { if (x.name === name) x.checked = false; });
  r.checked = true;
}

(async () => {
  check("renders 6x21 rating + 15x2 pair radios", radios.length === 156, "got " + radios.length);

  // partial fill -> sticky bar shows progress
  setRadio("r-mental", 10);
  setRadio("r-physical", 20);
  ctx.updateLive();
  check("partial fill shows progress bar",
    byId.stickybar.style.display === "block" && byId.stickybar.textContent.includes("2/6"),
    JSON.stringify(byId.stickybar.textContent));

  // full fill: ratings 10..60 -> raw 35.0; weights M3 P2 T2 Pf3 E3 F2 -> weighted 34.7
  setRadio("r-mental", 10); setRadio("r-physical", 20); setRadio("r-temporal", 30);
  setRadio("r-performance", 40); setRadio("r-effort", 50); setRadio("r-frustration", 60);
  // winners mirror PAIRS order; each is verified to be an option of its pair
  const winners = ["Mental Demand", "Mental Demand", "Performance", "Mental Demand",
    "Frustration", "Physical Demand", "Performance", "Effort", "Physical Demand",
    "Performance", "Temporal Demand", "Temporal Demand", "Effort", "Frustration", "Effort"];
  winners.forEach((w, i) => {
    const opts = radios.filter(r => r.name === "p" + (i + 1)).map(r => r.value);
    if (!opts.includes(w)) throw new Error(`pair ${i + 1} has no option ${w}`);
    setRadio("p" + (i + 1), w);
  });
  ctx.updateLive();
  check("per-scale chip shows value", byId["c-mental"].textContent === "= 10",
    JSON.stringify(byId["c-mental"].textContent));
  check("live line shows Raw TLX 35.0", byId.live.textContent.includes("35.0"),
    JSON.stringify(byId.live.textContent));
  check("live line shows Weighted TLX 34.7", byId.live.textContent.includes("34.7"),
    JSON.stringify(byId.live.textContent));
  check("sticky bar shows both scores",
    byId.stickybar.textContent.includes("35.0") && byId.stickybar.textContent.includes("34.7"),
    JSON.stringify(byId.stickybar.textContent));
  check("score table auto-fills cells",
    String(byId["t-r-mental"].textContent) === "10" &&
    String(byId["t-w-effort"].textContent) === "3" &&
    String(byId["t-p-mental"].textContent) === "30",
    JSON.stringify({ r: byId["t-r-mental"].textContent, w: byId["t-w-effort"].textContent,
      p: byId["t-p-mental"].textContent }));
  check("score table totals",
    String(byId["t-rtotal"].textContent) === "210" &&
    byId["t-wtotal"].textContent === "15 / 15" &&
    byId["t-grand"].textContent === "÷ 15 = 34.7",
    JSON.stringify({ rt: byId["t-rtotal"].textContent, wt: byId["t-wtotal"].textContent,
      g: byId["t-grand"].textContent }));

  // submit posts the right payload
  texts.operator.value = "OP1"; texts.date.value = "2026-11-03";
  texts.condition.value = "full"; texts.task.value = "drill";
  await ctx.submitForm();
  const payload = JSON.parse(fetched.opts.body);
  check("submit posts to 'submit'", fetched.url === "submit", fetched.url);
  check("submit payload ratings/weights/scores",
    payload.ratings.mental === 10 && payload.weights.frustration === 2 &&
    payload.raw_tlx === 35 && payload.weighted_tlx === 34.7,
    fetched.opts.body.slice(0, 200));
  check("submit status confirms", byId.status.textContent.includes("#7"),
    JSON.stringify(byId.status.textContent));

  // CSV export carries the same numbers
  ctx.downloadCSV();
  check("CSV export has header + values",
    blobText.includes("weighted_tlx") && blobText.includes("OP1") && blobText.includes("35"),
    (blobText || "").slice(0, 160));

  // autosave round-trip
  ctx.saveState();
  radios.forEach(r => { r.checked = false; });
  Object.values(texts).forEach(t => { t.value = ""; });
  ctx.restoreState();
  const restored = documentStub.querySelector('input[name="r-mental"]:checked');
  check("autosave restores marks + header",
    restored && restored.value === "10" && texts.operator.value === "OP1",
    "mental=" + (restored && restored.value));

  // clearing everything empties the table and hides the bar
  radios.forEach(r => { r.checked = false; });
  ctx.updateLive();
  check("clearing marks resets table + bar",
    byId["t-r-mental"].textContent === "" && byId["t-p-effort"].textContent === "" &&
    byId["t-rtotal"].textContent === "" && byId.live.textContent === "" &&
    byId.stickybar.style.display === "none",
    JSON.stringify({ r: byId["t-r-mental"].textContent, live: byId.live.textContent }));

  console.log(failures ? `\n${failures} FAILING` : "\nALL PASS");
  process.exit(failures ? 1 : 0);
})().catch(e => { console.error("HARNESS ERROR:", e); process.exit(2); });
