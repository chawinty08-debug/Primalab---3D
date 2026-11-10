(function () {
'use strict';
const E = window.MocapEngine;
const $ = id => document.getElementById(id);
const viewer = new E.MocapViewer($('view'));
const chart = new E.ChartView($('chart')), chart2 = new E.ChartView($('chart2'));
const chartF = new E.ChartView($('cmjF')), chartV = new E.ChartView($('cmjV'));
chartF.noEventLabels = chartV.noEventLabels = true;
let cmj = null;
let ds = null, tab = 'marker', kin = null, fa = null, chartK = 1, staticDs = null;
let kin6 = null, kinIK = null, idr = null;   // segment-optimisation result, IK result, inverse-dynamics result
const cache = new Map();

/* ---------- theme tokens for the chart canvas */
function readTheme() {
  const cs = getComputedStyle(document.documentElement), g = n => cs.getPropertyValue(n).trim();
  const th = { fg: g('--fg'), fg2: g('--fg-2'), line: g('--line'), accent: g('--accent'), panel: g('--panel') };
  for (const c of [chart, chart2, chartF, chartV]) { c.theme = th; c.draw(); }
}
matchMedia('(prefers-color-scheme: dark)').addEventListener('change', readTheme);
new MutationObserver(readTheme).observe(document.documentElement, { attributes: true, attributeFilter: ['data-theme'] });
readTheme();


/* ---------- helpers */
function note(msg, kind) {
  const n = $('note');
  if (!msg) { n.hidden = true; return; }
  n.hidden = false; n.className = 'note' + (kind ? ' ' + kind : ''); n.textContent = msg;
}
function row(dl, k, v) {
  const dt = document.createElement('dt'), dd = document.createElement('dd');
  dt.textContent = k; dd.textContent = v; dl.append(dt, dd);
}

/* ---------- dataset load */
function load(d) {
  if (!d.platesAll) d.platesAll = d.plates.slice();   // every plate in the file; d.plates = the ones in use
  ds = d; cache.clear(); kin = null; viewer.kin = null; fa = null; chartK = 1; resetFa(); $('bCsv').disabled = true; $('tabA').disabled = true;
  kin6 = null; resetKinetics(true);
  viewer.setDataset(d);
  $('badge').hidden = !d.synthetic;
  // info
  const dl = $('info'); dl.textContent = '';
  const dur = d.nFrames / d.rate;
  let missing = 0, total = 0, nUnl = 0;
  for (let i = 0; i < d.nM; i++) if (d.side[i] === 'U') nUnl++;
  for (let i = 0; i < d.nM; i++) { if (d.side[i] === 'U') continue; for (let f = 0; f < d.nFrames; f++) { total++; if (!d.vis[f * d.nM + i]) missing++; } }
  row(dl, 'ชื่อ', d.name);
  row(dl, 'ความถี่', d.rate + ' Hz');
  row(dl, 'เฟรม', d.nFrames + ' (' + dur.toFixed(2) + ' s)');
  row(dl, 'หน่วยในไฟล์', d.info.unitsLabel + ' → แสดงเป็น m');
  row(dl, 'Prefix', d.prefix || '—');
  row(dl, 'Marker', (d.nM - nUnl) + ' + unlabeled ' + nUnl);
  row(dl, 'ข้อมูลขาด', total ? (100 * missing / total).toFixed(1) + ' %' : '—');
  if (d.info.analogChannels) row(dl, 'Analog', d.info.analogChannels + ' ch @ ' + d.info.analogRate + ' Hz');
  if (d.plates.length) d.plates.forEach(p => row(dl, 'FP' + p.index, p.supported ? p.note : '⚠ ' + p.note));
  else row(dl, 'Force plate', 'ไม่พบ');
  row(dl, 'Event', d.events.length ? d.events.length + ' รายการ' : '—');
  const warn = (d.warnings || []).concat(d.plates.filter(p => !p.supported).map(p => 'FP' + p.index + ': ' + p.note));
  note(warn.length ? warn.join(' · ') : '', warn.length ? 'warn' : '');
  // timeline
  const sl = $('slider'); sl.max = d.nFrames - 1; sl.value = 0;
  const tk = $('ticks'); tk.textContent = '';
  for (const ev of d.events) { const s = document.createElement('span'); s.style.left = (ev.frame / (d.nFrames - 1) * 100) + '%'; s.dataset.l = ev.label; tk.append(s); }
  // tabs
  buildPlates(); fillMsel();
  buildList(); syncChecks();
  // default selected marker
  let sel = d.base.findIndex(b => /^RKNE$/i.test(b)); if (sel < 0) sel = d.side.findIndex(s => s !== 'U');
  viewer.select(sel);
  $('oUnl').checked = false;
  updateLinkCount(); updateChart();
}

/* ---------- marker list */
function buildList() {
  const ul = $('mlist'); ul.textContent = '';
  for (let i = 0; i < ds.nM; i++) {
    const li = document.createElement('li');
    li.className = 'mrow' + (ds.side[i] === 'U' ? ' unl' : ''); li.dataset.i = i;
    const cb = document.createElement('input'); cb.type = 'checkbox'; cb.className = 'mchk'; cb.setAttribute('aria-label', 'แสดง ' + ds.base[i]);
    const sw = document.createElement('span'); sw.className = 'sw'; sw.style.background = E.SIDE_COLOR[ds.side[i]] || E.SIDE_COLOR.C;
    const nm = document.createElement('span'); nm.className = 'mname'; nm.textContent = ds.base[i]; nm.title = ds.labels[i];
    const gp = document.createElement('span'); gp.className = 'mgap';
    const vr = E.validRatio(ds, i); if (vr < 0.9995) { gp.textContent = Math.round(vr * 100) + '%'; gp.title = 'สัดส่วนเฟรมที่มีข้อมูล'; }
    li.append(cb, sw, nm, gp); ul.append(li);
  }
  $('mfilter').value = '';
}
function syncChecks() {
  document.querySelectorAll('#mlist .mrow').forEach(li => { li.querySelector('input').checked = !!viewer.visible[+li.dataset.i]; });
}
function markSelected(i) {
  document.querySelectorAll('#mlist .mrow').forEach(li => li.classList.toggle('sel', +li.dataset.i === i));
  const li = document.querySelector('#mlist .mrow.sel'), ul = $('mlist');
  if (li) { const r = li.getBoundingClientRect(), u = ul.getBoundingClientRect(); if (r.top < u.top) ul.scrollTop -= u.top - r.top; else if (r.bottom > u.bottom) ul.scrollTop += r.bottom - u.bottom; }
}
$('mlist').addEventListener('click', e => {
  const li = e.target.closest('.mrow'); if (!li) return;
  const i = +li.dataset.i;
  if (e.target.classList.contains('mchk')) { viewer.setVisible(i, e.target.checked); return; }
  viewer.select(i === viewer.selected ? -1 : i);
});
$('mfilter').addEventListener('input', e => {
  const q = e.target.value.trim().toLowerCase();
  document.querySelectorAll('#mlist .mrow').forEach(li => { li.hidden = q && !ds.base[+li.dataset.i].toLowerCase().includes(q); });
});
$('mAll').addEventListener('click', () => { for (let i = 0; i < ds.nM; i++) if (ds.side[i] !== 'U' || $('oUnl').checked) viewer.setVisible(i, true); syncChecks(); });
$('mNone').addEventListener('click', () => { for (let i = 0; i < ds.nM; i++) viewer.setVisible(i, false); syncChecks(); });

/* ---------- viewer events */
viewer.onselect = i => { markSelected(i); updateChart(); };
viewer.onlinks = () => updateLinkCount();
viewer.onframe = (f, playing) => {
  if (!ds) return;
  const fi = Math.floor(f);
  $('slider').value = fi;
  $('readout').textContent = 't ' + (f / ds.rate).toFixed(3) + ' s · f ' + (fi + 1) + '/' + ds.nFrames;
  $('icPlay').hidden = playing; $('icPause').hidden = !playing;
  $('bPlay').setAttribute('aria-label', playing ? 'หยุดชั่วคราว' : 'เล่น');
  chart.cursor = f * chartK; chart.draw();
  if (PB.on) { chart2.cursor = f * PB.k; chart2.draw(); }
  if (cmj) { const c = f * (fa.fs / ds.rate) - cmj.c0; chartF.cursor = c; chartV.cursor = c; chartF.draw(); chartV.draw(); }
};
chart.onseek = f => { viewer.setFrame(f / chartK); };
chart2.onseek = f => { viewer.setFrame(f / PB.k); };
chartF.onseek = chartV.onseek = f => { if (cmj) viewer.setFrame((f + cmj.c0) / (fa.fs / ds.rate)); };
function updateLinkCount() { $('linkCount').textContent = ds ? viewer.links.length + ' link' : ''; }

/* ---------- chart */
function markerSeries(i) {
  const key = 'm' + i; if (cache.has(key)) return cache.get(key);
  const n = ds.nFrames, nM = ds.nM, out = [new Float32Array(n), new Float32Array(n), new Float32Array(n)];
  for (let f = 0; f < n; f++) {
    const k = f * nM + i, ok = ds.vis[k];
    for (let a = 0; a < 3; a++) out[a][f] = ok ? ds.pts[k * 3 + a] : NaN;
  }
  cache.set(key, out); return out;
}
function grfSeries(pl) {
  const key = 'g' + pl.index; if (cache.has(key)) return cache.get(key);
  const n = ds.nFrames, out = [new Float32Array(n), new Float32Array(n), new Float32Array(n)];
  for (let f = 0; f < n; f++) for (let a = 0; a < 3; a++) out[a][f] = pl.F[f * 3 + a];
  cache.set(key, out); return out;
}
const PA = { c: chart, sfx: '', get tab() { return tab; }, set tab(v) { tab = v; }, k: 1 };
const PB = { c: chart2, sfx: '2', tab: 'ang', k: 1 };
function updateChart() { updatePanel(PA); if (PB.on) updatePanel(PB); }
function updatePanel(P) {
  if (!ds) return;
  const c = P.c, S = P.sfx, tb = P.tab; P.k = 1;
  const title = t => { $('ctitle' + S).textContent = t; };
  const base = { n: ds.nFrames, rate: ds.rate, events: ds.events, cursor: viewer.f, frameOffset: 0 };
  if (tb === 'marker') {
    const i = S ? +$('msel2').value : viewer.selected;
    if (i < 0 || isNaN(i)) { c.set(Object.assign({ series: [], empty: T('คลิก marker ในมุมมอง 3D หรือในรายการ เพื่อดูกราฟ') }, base)); title('—'); return; }
    const s = markerSeries(i);
    c.set(Object.assign({ series: [{ name: 'X', color: E.AXIS.x, data: s[0] }, { name: 'Y', color: E.AXIS.y, data: s[1] }, { name: 'Z', color: E.AXIS.z, data: s[2] }], dec: 3 }, base));
    title(ds.base[i] + ' · ตำแหน่งใน lab frame (m)');
  } else if (tb === 'force') {
    if (!fa || !fa.ok) { c.set(Object.assign({ series: [], empty: T('กด “วิเคราะห์ FP” ก่อน') }, base)); title('—'); return; }
    P.k = fa.fs / ds.rate;
    const Pp = fa.phases, ev = [['Start', Pp.s], ['Braking', Pp.b], ['Split', Pp.z], ['Take-off', Pp.t], ['Landing', Pp.l]].map(([label, frame]) => ({ frame, label, context: '' }));
    const ser = [{ name: T('Fz รวม'), color: E.AXIS.z, data: fa.traces.sf }];
    if (fa.lr) ser.push({ name: T('ซ้าย'), color: E.AXIS.x, data: fa.traces.sl }, { name: T('ขวา'), color: E.AXIS.y, data: fa.traces.sr });
    c.set({ series: ser, dec: 0, n: fa.n, rate: fa.fs, events: ev, cursor: viewer.f * P.k, frameOffset: null });
    title('Fz หลังกรอง ลบ offset flight (N) · FP' + fa.plates.join('+') + ' · ' + fa.fs + ' Hz');
  } else if (tb === 'ang') {
    const js = $('jsel' + S), A = kin && kin.angles[js.value];
    if (!A) { c.set(Object.assign({ series: [], empty: T('กด “คำนวณมุมข้อต่อ” ก่อน') }, base)); title('—'); return; }
    const pg = kin && kin.convention === 'pig';
    c.set(Object.assign({ series: pg ? [{ name: 'X flex(+)/ext', color: E.AXIS.x, data: A.a }, { name: 'Y add/varus(+)', color: E.AXIS.y, data: A.b }, { name: 'Z int.rot(+)', color: E.AXIS.z, data: A.g }] : [{ name: 'X flex/ext', color: E.AXIS.x, data: A.a }, { name: 'Y ab/add', color: E.AXIS.y, data: A.b }, { name: 'Z axial', color: E.AXIS.z, data: A.g }], dec: 1 }, base));
    title(js.selectedOptions[0].textContent + (pg ? ' · Plug-in Gait style Y-X-Z (°)' : ' · Cardan Z-X-Y (°)') + (kin.source === 'ik' ? ' · IK' : ' · 6DOF') + (kin.params.normalize ? ' · ลบ offset static' : ''));
  } else if (tb === 'kin') {
    const js = $('jsel' + S), J = idr && idr.ok ? idr.joints[js.value] : null, mode = $('ksel' + S).value;
    if (!J) { c.set(Object.assign({ series: [], empty: T('กด “คำนวณ Inverse Dynamics” ก่อน') }, base)); title('—'); return; }
    const pg = idr.convention === 'pig', U = idr.units, side = js.value[0];
    let ser, ttl;
    if (mode === 'M') {
      ser = pg ? [{ name: 'X flex(+)/ext', color: E.AXIS.x, data: J.a }, { name: 'Y add/varus(+)', color: E.AXIS.y, data: J.b }, { name: 'Z int.rot(+)', color: E.AXIS.z, data: J.g }]
        : [{ name: 'X flex/ext', color: E.AXIS.x, data: J.a }, { name: 'Y ab/add', color: E.AXIS.y, data: J.b }, { name: 'Z axial', color: E.AXIS.z, data: J.g }];
      ttl = 'Joint moment (' + U.M + ') · internal · + = ทิศที่เพิ่มมุม';
    } else if (mode === 'S') {
      const Sp = idr.support[side], pk = Sp.peak;
      ser = [{ name: 'Ms', color: E.AXIS.z, data: Sp.Ms }, { name: T('Hip ext'), color: '#B07A00', data: Sp.Hip }, { name: T('Knee ext'), color: E.AXIS.x, data: Sp.Knee }, { name: T('Ankle PF'), color: E.AXIS.y, data: Sp.Ankle }];
      ttl = 'Total support moment (Winter 1980) · ขา' + (side === 'R' ? 'ขวา' : 'ซ้าย') + ' (' + U.M + ') · + = extensor / plantarflexor · เฉพาะช่วง stance' +
        (pk ? ' · peak Ms ' + pk.Ms.toFixed(2) + ' @ ' + pk.t.toFixed(2) + ' s · hip ' + pk.pct.Hip.toFixed(0) + '% / knee ' + pk.pct.Knee.toFixed(0) + '% / ankle ' + pk.pct.Ankle.toFixed(0) + '%' : '');
      c.set(Object.assign({ series: ser, dec: 2 }, base));
      title(ttl + ' · ' + (idr.source === 'ik' ? 'IK' : '6DOF')); if (!S) chartK = P.k; return;
    } else if (mode === 'P') { ser = [{ name: 'P', color: E.AXIS.z, data: J.P }]; ttl = 'Joint power (' + U.P + ') · + = generation, − = absorption'; }
    else if (mode === 'F') { ser = [{ name: 'Fx', color: E.AXIS.x, data: J.Fx }, { name: 'Fy', color: E.AXIS.y, data: J.Fy }, { name: 'Fz', color: E.AXIS.z, data: J.Fz }]; ttl = 'Net joint force · intersegmental (' + U.F + ') · แกน segment'; }
    else { const g = idr.grf[side]; ser = [{ name: 'Fx', color: E.AXIS.x, data: g.x }, { name: 'Fy', color: E.AXIS.y, data: g.y }, { name: 'Fz', color: E.AXIS.z, data: g.z }]; ttl = 'GRF เท้า' + (side === 'R' ? 'ขวา' : 'ซ้าย') + ' (' + U.F + ') · lab · หลังกรอง'; }
    c.set(Object.assign({ series: ser, dec: 2 }, base));
    title(js.selectedOptions[0].textContent + ' · ' + ttl + (mode !== 'G' && mode !== 'P' ? (idr.resolve === 'dist' ? ' · resolve: distal' : ' · resolve: proximal') : '') + ' · ' + (idr.source === 'ik' ? 'IK' : '6DOF'));
  } else {
    const want = +$('psel' + S).value, pl = ds.plates.find(p => p.supported && p.index === want) || ds.plates.find(p => p.supported);
    if (!pl) { c.set(Object.assign({ series: [], empty: T('ไม่มีข้อมูล force plate') }, base)); title('—'); return; }
    const s = grfSeries(pl);
    c.set(Object.assign({ series: [{ name: 'Fx', color: E.AXIS.x, data: s[0] }, { name: 'Fy', color: E.AXIS.y, data: s[1] }, { name: 'Fz', color: E.AXIS.z, data: s[2] }], dec: 1 }, base));
    title('FP' + pl.index + ' · GRF ใน lab frame (N)' + (ds.synthetic ? ' · สังเคราะห์' : ''));
  }
  if (!S) chartK = P.k;
}
function syncTabs(P) {
  const S = P.sfx, t = P.tab;
  for (const [id, v] of [['tabM', 'marker'], ['tabG', 'grf'], ['tabA', 'ang'], ['tabK', 'kin'], ['tabF', 'force']]) $(id + S).setAttribute('aria-selected', t === v);
  $('jsel' + S).hidden = t !== 'ang' && t !== 'kin'; $('ksel' + S).hidden = t !== 'kin'; $('psel' + S).hidden = t !== 'grf'; if (S) $('msel2').hidden = t !== 'marker';
}
function setTab(t) { PA.tab = t; syncTabs(PA); updatePanel(PA); }
function setTabB(t) { PB.tab = t; syncTabs(PB); updatePanel(PB); }
$('tabM').addEventListener('click', () => setTab('marker'));
$('tabG').addEventListener('click', () => setTab('grf'));
$('tabA').addEventListener('click', () => setTab('ang'));
$('tabF').addEventListener('click', () => setTab('force'));
$('tabK').addEventListener('click', () => setTab('kin'));
$('jsel').addEventListener('input', () => updatePanel(PA));
$('ksel').addEventListener('input', () => updatePanel(PA));
$('tabM2').addEventListener('click', () => setTabB('marker'));
$('tabG2').addEventListener('click', () => setTabB('grf'));
$('tabA2').addEventListener('click', () => setTabB('ang'));
$('tabF2').addEventListener('click', () => setTabB('force'));
$('tabK2').addEventListener('click', () => setTabB('kin'));
$('jsel2').addEventListener('input', () => updatePanel(PB));
$('ksel2').addEventListener('input', () => updatePanel(PB));
$('msel2').addEventListener('input', () => updatePanel(PB));
$('psel').addEventListener('input', () => updatePanel(PA));
$('psel2').addEventListener('input', () => updatePanel(PB));

/* ---------- force plates in use (unused plates can be excluded from GRF, CMJ analysis, ID and the 3D view) */
const PL_FZ = 50, PL_T = 0.05;   // a plate counts as used when Fz > 50 N for at least 50 ms in total
function plateUse(p) {
  if (!p.supported || !p.F) return { used: false, peak: 0, t: 0 };
  let n = 0, pk = 0; for (let f = 0; f < ds.nFrames; f++) { const v = p.F[f * 3 + 2]; if (v > PL_FZ) n++; if (v > pk) pk = v; }
  return { used: n / ds.rate >= PL_T, peak: pk, t: n / ds.rate };
}
function buildPlates() {
  const all = ds.platesAll, card = $('plCard'), ul = $('plList');
  card.hidden = !all.length; ul.textContent = ''; $('psel').textContent = ''; $('psel2').textContent = '';   // new file: default to the most loaded plate
  for (const p of all) {
    const u = plateUse(p), li = document.createElement('li'), lb = document.createElement('label'), cb = document.createElement('input');
    cb.type = 'checkbox'; cb.checked = ds.plates.includes(p); cb.disabled = !p.supported; cb.dataset.i = p.index;
    cb.addEventListener('change', () => setPlates(all.filter(q => ul.querySelector('input[data-i="' + q.index + '"]').checked)));
    const nm = document.createElement('b'); nm.textContent = 'FP' + p.index;
    const info = document.createElement('span'); info.className = 'plinfo' + (u.used ? '' : ' idle');
    info.textContent = p.supported ? (u.used ? 'peak Fz ' + u.peak.toFixed(0) + ' N · ' + T('เวลามีแรง ') + u.t.toFixed(2) + ' s' : T('ไม่มีแรง (peak ') + u.peak.toFixed(0) + ' N)') : T('อ่านไม่ได้');
    lb.append(cb, nm, info); li.append(lb); ul.append(li);
  }
  syncPlates();
}
function syncPlates() {
  const all = ds.platesAll, on = ds.plates.filter(p => p.supported);
  const hasF = on.length > 0;
  $('tabG').disabled = !hasF; $('tabG2').disabled = !hasF;
  for (const id of ['psel', 'psel2']) {
    const sl = $(id), keep = +sl.value; sl.textContent = '';
    on.forEach(p => { const o = document.createElement('option'); o.value = p.index; o.textContent = 'FP' + p.index; sl.append(o); });
    if (on.some(p => p.index === keep)) sl.value = keep;
    else if (on.length) { let best = on[0], bp = -1; for (const p of on) { const u = plateUse(p); if (u.peak > bp) { bp = u.peak; best = p; } } sl.value = best.index; }
  }
  const off = all.filter(p => !ds.plates.includes(p)).map(p => 'FP' + p.index);
  $('plNote').textContent = T('แผ่นที่ใช้: ') + on.length + '/' + all.length + (off.length ? T(' · ตัดออก: ') + off.join(', ') : '') + T(' · มีผลกับกราฟ GRF, การวิเคราะห์ CMJ, Inverse Dynamics และภาพ 3D');
  $('bPlAll').disabled = off.length === 0;
}
function setPlates(list) {
  ds.plates = ds.platesAll.filter(p => list.includes(p));
  for (const cb of $('plList').querySelectorAll('input')) cb.checked = ds.plates.some(p => p.index === +cb.dataset.i);
  syncPlates();
  fa = null; resetFa(); resetKinetics(false);
  if (PA.tab === 'force' || (PA.tab === 'grf' && $('tabG').disabled)) setTab('marker');
  if (PB.on && (PB.tab === 'force' || (PB.tab === 'grf' && $('tabG2').disabled))) setTabB('ang');
  viewer.invalidate(); updateChart();
}
$('bPlTrim').addEventListener('click', () => { if (ds) setPlates(ds.platesAll.filter(p => plateUse(p).used)); });
$('bPlAll').addEventListener('click', () => { if (ds) setPlates(ds.platesAll.filter(p => p.supported)); });
$('bCmp').addEventListener('click', () => {
  PB.on = !PB.on; $('boxB').hidden = !PB.on; $('chartPair').classList.toggle('two', PB.on);
  $('bCmp').setAttribute('aria-pressed', PB.on); if (PB.on) { fillMsel(); syncTabs(PB); chart2.resize && chart2.resize(); updatePanel(PB); } chart.resize && chart.resize(); chart.draw();
});
function fillMsel() {
  const m = $('msel2'), keep = m.value; m.textContent = '';
  if (!ds) return;
  ds.base.forEach((b, i) => { if (ds.side[i] === 'U' && /^\*|^C_|unl/i.test(b)) return; const o = document.createElement('option'); o.value = i; o.textContent = b; m.appendChild(o); });
  if (keep && [...m.options].some(o => o.value === keep)) m.value = keep; else { const j = ds.base.findIndex(b => /^LKNE$/i.test(b)); if (j >= 0) m.value = j; }
}
$('tabA').disabled = true;

/* ---------- Subject Anthropometry */
function getAnthro() {
  const hRaw = $('subH') ? $('subH').value.trim() : '';
  const mRaw = $('subM') ? $('subM').value.trim() : '';
  if (!hRaw) return null;
  const hCm = parseFloat(hRaw);
  if (!isFinite(hCm) || hCm < 50 || hCm > 250) return null;
  const mKg = mRaw ? parseFloat(mRaw) : 70.0;
  const sex = $('subSex') ? $('subSex').value : 'M';
  return E.estimateAnthropometry(hCm / 100, mKg, sex);
}

function updateAnthroUI(applyToModel = false) {
  const at = getAnthro();
  const isEn = typeof LANG !== 'undefined' && LANG === 'en';
  const subNote = $('subNote');

  if (!at) {
    if ($('lblKneeW')) $('lblKneeW').textContent = '—';
    if ($('lblAnkleW')) $('lblAnkleW').textContent = '—';
    if ($('lblLegL')) $('lblLegL').textContent = '—';
    if ($('lblLegM')) $('lblLegM').textContent = '—';
    if (applyToModel && subNote) {
      subNote.className = 'note err';
      subNote.textContent = isEn
        ? 'Please enter a valid height (50–250 cm) before applying.'
        : 'กรุณากรอกส่วนสูงที่ถูกต้อง (50–250 cm) ก่อนนำไปใช้';
    }
    return;
  }

  if ($('lblKneeW')) $('lblKneeW').textContent = at.kneeW.toFixed(3) + ' m';
  if ($('lblAnkleW')) $('lblAnkleW').textContent = at.ankleW.toFixed(3) + ' m';
  if ($('lblLegL')) $('lblLegL').textContent = at.lengths.thigh.toFixed(2) + ' / ' + at.lengths.shank.toFixed(2) + ' m';
  if ($('lblLegM')) $('lblLegM').textContent = 'Thigh ' + at.masses.thigh + ' · Shank ' + at.masses.shank + ' kg';

  if (applyToModel) {
    if ($('aKR')) $('aKR').value = at.kneeW.toFixed(3);
    if ($('aKL')) $('aKL').value = at.kneeW.toFixed(3);
    if ($('aAR')) $('aAR').value = at.ankleW.toFixed(3);
    if ($('aAL')) $('aAL').value = at.ankleW.toFixed(3);
    if ($('fMass')) $('fMass').value = at.massKg.toFixed(1);

    if (subNote) {
      subNote.className = 'note';
      const sexLabel = at.sex === 'F' ? (isEn ? 'Female' : 'หญิง') : (isEn ? 'Male' : 'ชาย');
      subNote.textContent = isEn
        ? 'Applied estimates (H: ' + (at.heightM * 100).toFixed(0) + ' cm, BW: ' + at.massKg.toFixed(1) + ' kg, ' + sexLabel + ') to model inputs. *Direct measurement recommended.'
        : 'นำค่าประมาณการ (ส่วนสูง: ' + (at.heightM * 100).toFixed(0) + ' cm, น้ำหนัก: ' + at.massKg.toFixed(1) + ' kg, ' + sexLabel + ') ไปใส่ในช่องโมเดลแล้ว *แนะนำให้วัดจริง';
    }

    if (kin && ds) {
      runAngles();
    }
  } else if (subNote && subNote.classList.contains('err')) {
    subNote.className = 'note warn';
    subNote.textContent = isEn
      ? '* Knee/ankle widths are rough initial estimates. Direct measurement is strongly recommended.'
      : '* ค่า width เป็นเพียงค่าประมาณการเบื้องต้น แนะนำให้ใช้คาลิปเปอร์วัดจริง';
  }
}

/* ---------- joint angles (V3D-style, computed in the browser) */
function runAngles() {
  if (!ds) return;
  const num = id => parseFloat($(id).value);
  const anthro = getAnthro();
  const p = {
    staticStart: num('aS0') | 0,
    staticN: num('aSN') | 0,
    kneeW: { R: num('aKR'), L: num('aKL') },
    ankleW: { R: num('aAR'), L: num('aAL') },
    markerR: num('aMR'),
    normalize: $('aNorm').checked,
    staticDs,
    convention: $('aConv').value,
    anthro
  };
  const r = E.computeLowerBody(ds, p), n = $('angNote');
  resetKinetics(true);
  if (!r.ok) { kin = kin6 = null; viewer.kin = null; $('tabA').disabled = true; $('tabA2').disabled = true; $('bCsv').disabled = true; n.className = 'note err'; n.textContent = r.error; viewer.invalidate(); return; }
  kin = kin6 = r; viewer.kin = r; viewer.invalidate(); $('bIk').disabled = false; $('bId').disabled = false;
  if (r.measured) { const m = r.measured, fx = (id, v) => { if ($(id) && v === v) $(id).value = v.toFixed(3); }; fx('aKR', m.kneeW.R); fx('aKL', m.kneeW.L); fx('aAR', m.ankleW.R); fx('aAL', m.ankleW.L); }
  $('tabA').disabled = false; $('tabA2').disabled = false; $('bCsv').disabled = false;
  const fmt = j => { const g = r.ranges[j].a; return j + ' ' + g[0].toFixed(0) + '…' + g[1].toFixed(0) + '°'; };
  n.className = 'note' + (r.warnings.length ? ' warn' : '');
  n.textContent = (r.convention === 'pig' ? 'Plug-in Gait style (Y-X-Z; + = flexion/dorsiflexion, adduction/varus, internal rot; ankle หัก static offset R ' + r.pigOffset.R.toFixed(1) + '° / L ' + r.pigOffset.L.toFixed(1) + '°; virtual TIB) · ' : '') + 'ชุด marker: ' + r.markerSetLabel + (r.measured ? ' (width วัดจาก marker medial ในไฟล์ ช่อง width ไม่ถูกใช้)' : '') + ' · static: ' + (r.staticSource === 'trial' ? 'trial นี้' : r.staticSource) + ' f' + r.staticRange[0] + '–' + r.staticRange[1] + ' (marker ขยับสูงสุด ' + (r.staticMove * 1000).toFixed(1) + ' mm) · flex: ' + ['RHip', 'RKnee', 'RAnkle', 'LHip', 'LKnee', 'LAnkle'].map(fmt).join(' · ') + (r.warnings.length ? ' · ⚠ ' + r.warnings.join(' · ') : '') + ' · ยังไม่เทียบกับ Visual3D';
  setTab('ang');
}
if ($('bApplyAnthro')) $('bApplyAnthro').addEventListener('click', () => updateAnthroUI(true));
['subH', 'subM', 'subSex'].forEach(id => {
  const el = $(id);
  if (el) el.addEventListener('input', () => updateAnthroUI(false));
});
$('bAng').addEventListener('click', runAngles);
/* separate static trial (markers matched by name) */
function setStatic(d) {
  staticDs = d;
  $('bStaticClr').disabled = !d;
  const sn = $('staticNote');
  if (d) {
    $('aS0').value = 0; $('aSN').value = Math.min(60, d.nFrames);
    sn.className = 'note'; sn.textContent = 'Static: ' + d.name + ' · ' + d.nFrames + ' frames @ ' + d.rate + ' Hz · ใช้ ' + $('aSN').value + ' frame แรกเฉลี่ยเป็นท่า static';
  } else { sn.className = 'note'; sn.textContent = 'Static: ใช้ frame ต้นของ trial ที่เปิดอยู่'; }
  if (ds && kin) runAngles();
}
$('bStatic').addEventListener('click', () => $('fileStatic').click());
$('fileStatic').addEventListener('change', async e => {
  const f = e.target.files[0]; e.target.value = ''; if (!f) return;
  try { setStatic(E.readC3D(await f.arrayBuffer(), f.name)); } catch (err) { const sn = $('staticNote'); sn.className = 'note err'; sn.textContent = 'เปิดไฟล์ static ไม่ได้: ' + err.message; }
});
$('bStaticClr').addEventListener('click', () => setStatic(null));
$('bSide').addEventListener('click', () => { const on = document.querySelector('.app').classList.toggle('nosideb'); $('bSide').setAttribute('aria-pressed', on); $('bSide').textContent = on ? '◨ เมนู' : '◧ เมนู'; viewer.invalidate(); });
$('oBody').addEventListener('input', e => { viewer.opt.body = e.target.checked; viewer.invalidate(); });
$('oSegs').addEventListener('input', e => { viewer.opt.segs = e.target.checked; viewer.invalidate(); });
$('bCsv').addEventListener('click', () => {
  if (!kin) return;
  const J = ['RHip', 'RKnee', 'RAnkle', 'LHip', 'LKnee', 'LAnkle'], h = ['frame', 'time_s'];
  J.forEach(j => (kin.convention === 'pig' ? ['flex', 'add', 'introt'] : ['X_flex', 'Y_abd', 'Z_rot']).forEach(c => h.push(j + '_' + c)));
  const rows = [h.join(',')];
  for (let f = 0; f < ds.nFrames; f++) {
    const r = [f, (f / ds.rate).toFixed(4)];
    J.forEach(j => ['a', 'b', 'g'].forEach(c => { const v = kin.angles[j][c][f]; r.push(v === v ? v.toFixed(3) : ''); }));
    rows.push(r.join(','));
  }
  saveText((ds.name || 'take').replace(/\.c3d$/i, '') + '_JointAngles_' + (kin.convention === 'pig' ? 'PiGstyle' : 'ZXY') + (kin.source === 'ik' ? '_IK-' + kin.ikModel : '') + '.csv', rows.join('\r\n'));
});

/* ---------- inverse kinematics / inverse dynamics */
function resetKinetics(all) {
  if (all) { kinIK = null; const o = $('kSrc'); o.value = '6dof'; o.options[1].disabled = true; $('ikNote').className = 'note'; $('ikNote').textContent = 'IK ใช้ static calibration ชุดเดียวกับมุมข้อต่อ (กด “คำนวณมุมข้อต่อ” ก่อน)'; $('bIk').disabled = !kin6; }
  idr = null; $('tabK').disabled = true; $('tabK2').disabled = true; $('bIdCsv').disabled = true; $('bId').disabled = !kin6;
  $('idNote').className = 'note'; $('idNote').textContent = 'Newton–Euler bottom-up · BSIP De Leva (1996) · ใช้น้ำหนักและเพศจากช่อง Anthropometry · ต้องมี force plate แยกเท้าซ้าย/ขวา';
  if (PA.tab === 'kin') setTab(kin ? 'ang' : 'marker');
  if (PB.on && PB.tab === 'kin') setTabB('ang');
}
function applySource() {
  const useIK = $('kSrc').value === 'ik' && kinIK;
  kin = useIK ? kinIK : kin6; viewer.kin = kin; viewer.invalidate();
  resetKinetics(false); updateChart();
}
function runIK() {
  if (!ds || !kin6) return;
  const n = $('ikNote'), t0 = performance.now();
  const r = E.computeIK(ds, kin6, { model: $('ikModel').value });
  if (!r.ok) { kinIK = null; n.className = 'note err'; n.textContent = r.error; return; }
  kinIK = r; const o = $('kSrc'); o.options[1].disabled = false; o.value = 'ik'; applySource();
  const mm = v => (v * 1000).toFixed(1);
  const worst = Object.entries(r.residual.perMarker).sort((a, b) => b[1].mean - a[1].mean).slice(0, 3).map(([k, v]) => k + ' ' + mm(v.mean)).join(', ');
  n.className = 'note' + (r.warnings.length ? ' warn' : '');
  n.textContent = 'IK · ' + r.ikLabel + ' · ' + r.nPar + ' DOF · marker residual เฉลี่ย ' + mm(r.residual.mean) + ' mm (สูงสุด ' + mm(r.residual.max) + ' mm) · มากสุด: ' + worst + ' mm · ' + (performance.now() - t0).toFixed(0) + ' ms' + (r.warnings.length ? ' · ⚠ ' + r.warnings.join(' · ') : '') + ' · ยังไม่เทียบกับ OpenSim/Visual3D';
  setTab('ang');
}
function runID() {
  if (!ds || !kin) return;
  const num = id => { const v = $(id).value.trim(); return v === '' ? null : parseFloat(v); };
  const r = E.computeID(ds, kin, { mass: num('subM'), sex: $('subSex').value, cutoff: num('idCut') || 0, cutoffGRF: num('idCutF'), fzThr: num('idThr') || 20, resolve: $('idRes').value, normalize: $('idNorm').checked });
  const n = $('idNote');
  if (!r.ok) { idr = null; n.className = 'note err'; n.textContent = r.error; $('tabK').disabled = true; $('tabK2').disabled = true; $('bIdCsv').disabled = true; return; }
  idr = r; $('tabK').disabled = false; $('tabK2').disabled = false; $('bIdCsv').disabled = false;
  const pk = j => { const p = r.peaks[j].a; return p[0] === Infinity ? j + ' —' : j + ' ' + p[0].toFixed(2) + '…' + p[1].toFixed(2); };
  const sm = s => { const q = r.support[s].peak; return q ? s + ' ' + q.Ms.toFixed(2) + ' (H ' + q.pct.Hip.toFixed(0) + ' / K ' + q.pct.Knee.toFixed(0) + ' / A ' + q.pct.Ankle.toFixed(0) + ' %)' : s + ' —'; };
  n.className = 'note' + (r.warnings.length ? ' warn' : '');
  n.textContent = 'Newton–Euler · kinematics ' + (r.source === 'ik' ? 'IK' : '6DOF') + ' · BSIP De Leva (1996, ' + (r.sex === 'F' ? 'หญิง' : 'ชาย') + ') · มวล ' + r.mass.toFixed(1) + ' kg · cutoff ' + r.cutoff + ' / GRF ' + r.cutoffGRF + ' Hz · moment sagittal (' + r.units.M + '): ' + ['RHip', 'RKnee', 'RAnkle', 'LHip', 'LKnee', 'LAnkle'].map(pk).join(' · ') + ' · peak support moment ' + sm('R') + ' · ' + sm('L') + (r.warnings.length ? ' · ⚠ ' + r.warnings.join(' · ') : '') + ' · validate แล้วกับข้อมูลสังเคราะห์เท่านั้น ยังไม่เทียบกับ Visual3D/OpenSim';
  setTab('kin');
}
$('bIk').addEventListener('click', runIK);
$('kSrc').addEventListener('input', applySource);
$('bId').addEventListener('click', runID);
$('bIdCsv').addEventListener('click', () => {
  if (!idr) return;
  const J = ['RHip', 'RKnee', 'RAnkle', 'LHip', 'LKnee', 'LAnkle'], U = idr.units, h = ['frame', 'time_s'];
  const mc = idr.convention === 'pig' ? ['M_flex', 'M_add', 'M_introt'] : ['M_X', 'M_Y', 'M_Z'];
  J.forEach(j => { mc.forEach(c => h.push(j + '_' + c + '_' + U.M)); ['Fx', 'Fy', 'Fz'].forEach(c => h.push(j + '_' + c + '_' + U.F)); h.push(j + '_P_' + U.P); });
  ['R', 'L'].forEach(s => ['x', 'y', 'z'].forEach(c => h.push('GRF_' + s + '_' + c + '_' + U.F)));
  ['R', 'L'].forEach(s => ['Ms', 'Hip', 'Knee', 'Ankle'].forEach(c => h.push('Support_' + s + '_' + (c === 'Ms' ? 'Ms' : c + (c === 'Ankle' ? '_PF' : '_ext')) + '_' + U.M)));
  const rows = [h.join(',')], fx = v => (v === v ? v.toFixed(4) : '');
  for (let f = 0; f < ds.nFrames; f++) {
    const r = [f, (f / ds.rate).toFixed(4)];
    J.forEach(j => { const o = idr.joints[j]; r.push(fx(o.a[f]), fx(o.b[f]), fx(o.g[f]), fx(o.Fx[f]), fx(o.Fy[f]), fx(o.Fz[f]), fx(o.P[f])); });
    ['R', 'L'].forEach(s => ['x', 'y', 'z'].forEach(c => r.push(fx(idr.grf[s][c][f]))));
    ['R', 'L'].forEach(s => ['Ms', 'Hip', 'Knee', 'Ankle'].forEach(c => r.push(fx(idr.support[s][c][f]))));
    rows.push(r.join(','));
  }
  saveText((ds.name || 'take').replace(/\.c3d$/i, '') + '_InverseDynamics_' + (idr.source === 'ik' ? 'IK' : '6DOF') + '_' + (idr.resolve === 'dist' ? 'distal' : 'proximal') + '.csv', rows.join('\r\n'));
});

async function saveText(fname, text) {
  let dl = null;
  try { dl = window.claude && await window.claude.use('downloads'); } catch (e) { dl = null; }
  if (dl) { try { await dl.save({ filename: fname, data: text }); } catch (e) { if (e && e.code !== 'declined') note('บันทึกไฟล์ไม่ได้: ' + (e.message || e.code), 'err'); } return; }
  const url = URL.createObjectURL(new Blob([text], { type: 'text/csv' })), a = document.createElement('a');
  a.href = url; a.download = fname; document.body.append(a); a.click(); a.remove(); setTimeout(() => URL.revokeObjectURL(url), 2000);
}

/* ---------- Force analysis (FP embedded in the C3D only) */
const fnum = id => parseFloat($(id).value);
function resetFa() {
  cmj = null;
  $('faOut').hidden = true; $('bFaCsv').disabled = true; $('tabF').disabled = true; $('tabF2').disabled = true;
  const pls = ds ? ds.plates.filter(p => p.supported && p.fzFull) : [];
  $('bFa').disabled = !pls.length; $('faPanel').hidden = !pls.length;
  $('faHint').textContent = 'พบ force plate ที่ใช้ได้ ' + pls.length + ' แผ่นใน C3D' + (pls.length === 1 ? ' · แผ่นเดียว: แสดงเฉพาะค่ารวม (ไม่มี L/R)' : ' · เลือกแผ่นซ้าย/ขวาได้') + ' · พอร์ตจาก Force-Analysis';
  $('fLRbox').hidden = pls.length < 2;
  if (pls.length >= 2) {
    for (const id of ['fPL', 'fPR']) { const s = $(id); s.textContent = ''; pls.forEach(p => { const o = document.createElement('option'); o.value = p.index; o.textContent = 'FP' + p.index; s.append(o); }); }
    $('fPL').value = pls[0].index; $('fPR').value = pls[1].index;
  }
}
function faOptions(withPhases) {
  const o = { filterType: $('fFilter').value, cutoff: fnum('fCut') || 50, bwMethod: $('fBw').value, mass: fnum('fMass'), fn: fnum('fFn') || 0 };
  if (o.bwMethod === 'window') o.window = [fnum('fQ0'), fnum('fQ1')];
  if (!$('fLRbox').hidden) { o.leftPlate = +$('fPL').value; o.rightPlate = +$('fPR').value; }
  if (withPhases) o.phaseTimes = { s: fnum('pS'), b: fnum('pB'), z: fnum('pZ'), t: fnum('pT'), l: fnum('pL') };
  return o;
}
function renderFa() {
  const out = $('faOut'), note_ = $('faNote'), r = fa;
  out.hidden = false;
  if (!r.ok) { note_.className = 'note err'; note_.textContent = r.error + (r.warnings && r.warnings.length ? ' · ' + r.warnings.join(' · ') : ''); $('faTbl').textContent = ''; $('faImp').textContent = ''; $('bFaCsv').disabled = true; $('tabF').disabled = true; $('tabF2').disabled = true; return; }
  $('tabF').disabled = false; $('tabF2').disabled = false; $('bFaCsv').disabled = false;
  const P = r.phases, f3 = v => (v / r.fs).toFixed(3);
  $('pS').value = f3(P.s); $('pB').value = f3(P.b); $('pZ').value = f3(P.z); $('pT').value = f3(P.t); $('pL').value = f3(P.l);
  note_.className = 'note' + (r.warnings.length ? ' warn' : '');
  note_.textContent = 'BW ' + r.bw.toFixed(0) + ' N (' + r.mass.toFixed(1) + ' kg) จาก ' + r.bwSrc + ' · filter ' + (r.params.filterType === 'butter' ? 'Butterworth 4th zero-phase ' + r.params.cutoff + ' Hz' : r.params.filterType === 'ma' ? 'moving average 15' : 'raw') + (r.warnings.length ? ' · ⚠ ' + r.warnings.join(' · ') : '');
  const tb = $('faTbl'); tb.textContent = '';
  for (const g of r.groups) {
    const card = document.createElement('div'); card.className = 'fgcard';
    const h = document.createElement('div'); h.className = 'fgrp'; h.textContent = g.title; card.append(h);
    const t = document.createElement('table'); t.className = 'ftbl';
    const hr = t.insertRow(); (r.lr ? ['', 'L', 'R', 'รวม', 'Δ'] : ['', 'รวม']).forEach(x => { const c = document.createElement('th'); c.textContent = x; hr.append(c); });
    for (const x of g.rows) { const tr = t.insertRow(); (r.lr ? [x.name, x.l, x.r, x.t, x.d] : [x.name, x.t]).forEach(v => { tr.insertCell().textContent = v; }); }
    card.append(t); tb.append(card);
  }
  const im = $('faImp'); im.textContent = '';
  if (r.impactErr) { im.textContent = 'ข้ามการวิเคราะห์: ' + r.impactErr; }
  else for (const x of r.impact) {
    const card = document.createElement('div'); card.className = 'fgcard';
    const t = document.createElement('table'); t.className = 'ftbl';
    [['Channel', x.channel], ['Peak (N)', x.peak], ['Rise 10–90% (ms)', x.rise], ['f_rise (Hz)', x.fRise], ['f_dur (Hz)', x.fDur], ['f 99% energy (Hz)', x.fC99], ['f ที่ใช้ (Hz)', x.fUsed], ['fn / f', x.ratio], ['Amp. error (%)', x.err], ['Status', x.status]].forEach(([k, v]) => { const tr = t.insertRow(); tr.insertCell().textContent = k; tr.insertCell().textContent = v; });
    card.append(t); if (x.notes.length) { const p = document.createElement('p'); p.className = 'note'; p.style.margin = '6px 0 0'; p.textContent = x.notes.join(' · '); card.append(p); }
    im.append(card);
  }
  drawCmj();
}
/* CMJ-only charts: force cropped to start-20% … landing+20% of the jump, coloured by phase */
function drawCmj() {
  const r = fa, P = r.phases, span = Math.max(1, P.l - P.s), pad = Math.round(0.2 * span);
  const c0 = Math.max(0, P.s - pad), c1 = Math.min(r.n - 1, P.l + pad), len = c1 - c0 + 1, rel = v => v - c0;
  cmj = { c0, c1 };
  const sub = a => Float32Array.from(a.subarray(c0, c1 + 1));
  const ser = [{ name: T('Fz รวม'), color: E.AXIS.z, data: sub(r.traces.sf) }];
  if (r.lr) ser.push({ name: T('ซ้าย'), color: E.AXIS.x, data: sub(r.traces.sl) }, { name: T('ขวา'), color: E.AXIS.y, data: sub(r.traces.sr) });
  const ph = [['Unweighting', P.s, P.b, 'rgba(239,148,202,0.20)'], ['Braking', P.b, P.z, 'rgba(77,41,148,0.16)'], ['Propulsive', P.z, P.t, 'rgba(171,29,121,0.18)'], ['Flight', P.t, P.l, 'rgba(140,130,165,0.20)'], ['Landing', P.l, c1, 'rgba(208,195,241,0.30)']];
  const bands = ph.map(([label, a, b, color]) => ({ label, from: rel(a), to: rel(b), color }));
  const ev = ph.slice(0, 4).map(([label, a]) => ({ frame: rel(a), label, context: '' }));
  chartF.set({ series: ser, n: len, rate: r.fs, t0: c0 / r.fs, events: ev, bands, hlines: [{ v: r.bw, label: 'BW ' + r.bw.toFixed(0) + ' N' }], dec: 0, cursor: viewer.f * (r.fs / ds.rate) - c0 });
  $('cmjT1').textContent = 'Fz ช่วง CMJ (N) · ' + (c0 / r.fs).toFixed(2) + '–' + (c1 / r.fs).toFixed(2) + ' s · FP' + r.plates.join('+');
  $('cmjLeg').innerHTML = ph.map(([label, , , color]) => '<span><i style="background:' + color.replace(/0\.\d+\)/, '0.55)') + '"></i>' + label + '</span>').join('');
  const vel = new Float32Array(len).fill(NaN); for (let i = P.s; i <= Math.min(P.t, r.n - 1); i++) vel[rel(i)] = r.traces.vel[i];
  chartV.set({ series: [{ name: T('ความเร็ว COM'), color: E.AXIS.z, data: vel }], n: len, rate: r.fs, t0: c0 / r.fs, events: ev, bands, hlines: [{ v: 0, label: '0 m/s' }], dec: 2, cursor: viewer.f * (r.fs / ds.rate) - c0 });
  $('cmjT2').textContent = 'ความเร็วจุดศูนย์กลางมวล (m/s) · start → take-off (impulse-momentum)';
}
function runFa(withPhases) {
  if (!ds) return;
  fa = E.FA.analyzePlates(ds, faOptions(withPhases));
  renderFa();
  updateChart();
}
$('bFa').addEventListener('click', () => runFa(false));
$('bFaRe').addEventListener('click', () => runFa(true));
$('bFaAuto').addEventListener('click', () => runFa(false));
$('bFaCsv').addEventListener('click', () => {
  if (!fa || !fa.ok) return;
  const q = s => '"' + String(s).replace(/"/g, '""') + '"', rows = [['group', 'metric', 'left', 'right', 'total', 'deficit'].join(',')];
  for (const g of fa.groups) for (const x of g.rows) rows.push([q(g.title), q(x.name), q(x.l), q(x.r), q(x.t), q(x.d)].join(','));
  rows.push('', ['phase', 'time_s'].join(','));
  for (const [k, v] of [['start', fa.phases.s], ['braking', fa.phases.b], ['split', fa.phases.z], ['take-off', fa.phases.t], ['landing', fa.phases.l]]) rows.push(k + ',' + (v / fa.fs).toFixed(3));
  saveText((ds.name || 'take').replace(/\.c3d$/i, '') + '_ForceAnalysis.csv', rows.join('\r\n'));
});

/* ---------- toolbar + options */
document.querySelectorAll('.vp-tools [data-view]').forEach(b => b.addEventListener('click', () => {
  const v = b.dataset.view; if (v === 'fit') viewer.fit(); else viewer.view(v);
}));
const bindOpt = (id, key, fn) => $(id).addEventListener('input', e => { viewer.opt[key] = fn(e.target); viewer.invalidate(); });
bindOpt('oLinks', 'links', t => t.checked); bindOpt('oGrid', 'grid', t => t.checked); bindOpt('oAxes', 'axes', t => t.checked);
bindOpt('oPlates', 'plates', t => t.checked); bindOpt('oLabels', 'labels', t => t.value);
bindOpt('oTrail', 'trail', t => parseFloat(t.value)); bindOpt('oSize', 'size', t => parseFloat(t.value));
$('oUnl').addEventListener('input', e => {
  if (!ds) return;
  for (let i = 0; i < ds.nM; i++) if (ds.side[i] === 'U') viewer.setVisible(i, e.target.checked);
  syncChecks();
});
$('bLink').addEventListener('click', () => {
  const on = $('bLink').getAttribute('aria-pressed') !== 'true';
  $('bLink').setAttribute('aria-pressed', on); viewer.setLinkMode(on);
  $('hint').textContent = on ? 'โหมดแก้ไข link: คลิก marker สองตัวเพื่อเพิ่มหรือลบ link (Esc = ออก)' : 'ลาก = หมุน · Shift/คลิกขวา+ลาก = เลื่อน · ล้อ = ซูม · คลิก marker = เลือก';
});
$('bReset').addEventListener('click', () => { if (!ds) return; viewer.links = E.defaultLinks(ds.base); viewer.invalidate(); updateLinkCount(); });

/* ---------- transport */
$('bPlay').addEventListener('click', () => viewer.toggle());
$('bPrev').addEventListener('click', () => { viewer.pause(); viewer.setFrame(Math.floor(viewer.f) - 1); });
$('bNext').addEventListener('click', () => { viewer.pause(); viewer.setFrame(Math.floor(viewer.f) + 1); });
$('slider').addEventListener('input', e => viewer.setFrame(+e.target.value));
$('speed').addEventListener('input', e => { viewer.speed = parseFloat(e.target.value); });
$('loop').addEventListener('input', e => { viewer.loop = e.target.checked; });
document.addEventListener('keydown', e => {
  if (!ds || e.target.matches('input[type="search"], select')) return;
  const step = e.shiftKey ? 10 : 1;
  if (e.code === 'Space' && !e.target.matches('button')) { e.preventDefault(); viewer.toggle(); }
  else if (e.key === 'ArrowLeft') { e.preventDefault(); viewer.pause(); viewer.setFrame(Math.floor(viewer.f) - step); }
  else if (e.key === 'ArrowRight') { e.preventDefault(); viewer.pause(); viewer.setFrame(Math.floor(viewer.f) + step); }
  else if (e.key === 'Home') { viewer.setFrame(0); }
  else if (e.key === 'End') { viewer.setFrame(ds.nFrames - 1); }
  else if (e.key === 'f' || e.key === 'F') { viewer.fit(); }
  else if (e.key === 'Escape' && viewer.linkMode) { $('bLink').click(); }
});

/* ---------- file loading */
async function openFile(file) {
  try {
    const buf = await file.arrayBuffer();
    const d = E.readC3D(buf, file.name);
    load(d);
  } catch (err) { note('เปิดไฟล์ไม่ได้: ' + err.message, 'err'); }
}
$('file').addEventListener('change', e => { const f = e.target.files[0]; if (f) openFile(f); e.target.value = ''; });
$('btnSample').addEventListener('click', () => load(E.makeSyntheticCMJ()));
const vp = $('vp');
['dragenter', 'dragover'].forEach(t => vp.addEventListener(t, e => { e.preventDefault(); $('drop').hidden = false; }));
['dragleave', 'drop'].forEach(t => vp.addEventListener(t, e => { e.preventDefault(); $('drop').hidden = true; }));
vp.addEventListener('drop', e => { const f = e.dataTransfer.files[0]; if (f) openFile(f); });

updateAnthroUI(false);
load(E.makeSyntheticCMJ());
$('bLang').addEventListener('click', () => setLang(LANG === 'en' ? 'th' : 'en'));
window.langChanged = function () {
  try {
    updateChart();
    if (typeof cmj !== 'undefined' && cmj && fa && fa.ok) drawCmj(fa);
    updateAnthroUI(false);
  } catch (e) { /* charts not ready */ }
  viewer.invalidate();
};
if (LANG === 'en') setLang('en'); else setLang('th');
})();
