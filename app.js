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
let kin6 = null, kinIK = null, idr = null;
let cycles = [];
let detEv = [];   // detected events shown on the charts
let qc = null, qcSel = -1;
let ub = null;   // upper-body result   // marker QC result, selected pair   // movement cycles [[f0, f1], ...] (frames, inclusive) for cycle export   // segment-optimisation result, IK result, inverse-dynamics result
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
  ds = d; cache.clear(); kin = null; viewer.kin = null; msk = null; viewer.msk = null; $('bOsim').disabled = false; fa = null; chartK = 1; resetFa(); $('bCsv').disabled = true; $('tabA').disabled = true;
  kin6 = null; resetKinetics(true); cycles = []; detEv = []; renderCycles(); resetQC(); resetUB();
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
  const un = $('upNote');
  if (d.up) {
    row(dl, 'แกนตั้ง', d.up.applied + (d.up.applied !== '+Z' ? ' → หมุนเป็น +Z' : '') + ' · ' + (d.up.requested === 'auto' ? d.up.source : 'กำหนดเอง (ตรวจพบ ' + d.up.detected + ')'));
    un.hidden = !d.up.conflict; un.className = 'note warn'; un.textContent = d.up.conflict ? 'หลักฐานแกนตั้งขัดกัน: ' + d.up.conflict + ' · ถ้ารูปคนนอน/กลับหัว ให้เลือกแกนตั้งเอง' : '';
  } else un.hidden = true;
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
  buildPlates(); fillMsel(); renderMap(); $('bChCsv').disabled = false;
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
  const base = { n: ds.nFrames, rate: ds.rate, events: detEv.length ? ds.events.concat(detEv) : ds.events, cursor: viewer.f, frameOffset: 0, bands: cycleBands(1), hlines: [] };
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
    c.set({ series: ser, dec: 0, n: fa.n, rate: fa.fs, events: ev, cursor: viewer.f * P.k, frameOffset: null, bands: cycleBands(P.k) });
    title('Fz หลังกรอง ลบ offset flight (N) · FP' + fa.plates.join('+') + ' · ' + fa.fs + ' Hz');
  } else if (tb === 'ang' && E.UB_JOINTS.includes($('jsel' + S).value)) {
    const js = $('jsel' + S), A = ub && ub.angles[js.value];
    if (!A) { c.set(Object.assign({ series: [], empty: ub ? T('ข้อนี้ไม่มีข้อมูลในไฟล์ (marker ไม่ครบ)') : T('กด “คำนวณมุมส่วนบน” ก่อน') }, base)); title('—'); return; }
    const lb = ub.labels[js.value], sh = js.value.endsWith('Shoulder');
    c.set(Object.assign({ series: [{ name: lb[0], color: E.AXIS.x, data: A.a }, { name: lb[1], color: E.AXIS.y, data: A.b }, { name: lb[2], color: E.AXIS.z, data: A.g }], dec: 1 }, base));
    title(js.selectedOptions[0].textContent + (sh ? ' · humerothoracic Euler Y-X-Y, ISB (°)' : ' · Cardan Z-X-Y, ISB (°)') + ' · 6DOF' + (ub.normalize && !sh ? ' · ลบ offset static' : ''));
  } else if (tb === 'ang') {
    const js = $('jsel' + S), A = kin && kin.angles[js.value];
    if (!A) { c.set(Object.assign({ series: [], empty: T('กด “คำนวณมุมข้อต่อ” ก่อน') }, base)); title('—'); return; }
    const pg = kin && kin.convention === 'pig';
    c.set(Object.assign({ series: pg ? [{ name: 'X flex(+)/ext', color: E.AXIS.x, data: A.a }, { name: 'Y add/varus(+)', color: E.AXIS.y, data: A.b }, { name: 'Z int.rot(+)', color: E.AXIS.z, data: A.g }] : [{ name: 'X flex/ext', color: E.AXIS.x, data: A.a }, { name: 'Y ab/add', color: E.AXIS.y, data: A.b }, { name: 'Z axial', color: E.AXIS.z, data: A.g }], dec: 1 }, base));
    title(js.selectedOptions[0].textContent + (pg ? ' · Plug-in Gait style Y-X-Z (°)' : ' · Cardan Z-X-Y (°)') + (kin.source === 'ik' ? ' · IK' : ' · 6DOF') + (kin.params.normalize ? ' · ลบ offset static' : ''));
  } else if (tb === 'mtu') {
    if (!msk) { c.set(Object.assign({ series: [], empty: T('เปิด “แสดงกระดูกและกล้ามเนื้อ” ในการ์ด Musculoskeletal model ก่อน') }, base)); title('—'); return; }
    const nm = $('usel' + S).value;
    const ser = [];
    for (const sd of ['r', 'l']) { const mu = msk.muscles.find(m => m.name === nm + '_' + sd); if (mu) ser.push({ name: (sd === 'r' ? 'R ' : 'L ') + nm, color: sd === 'r' ? E.SIDE_COLOR.R : E.SIDE_COLOR.L, data: mu.len }); }
    if (!ser.length) { c.set(Object.assign({ series: [], empty: T('กล้ามเนื้อนี้ไม่มีข้อมูล (segment ไม่ครบ)') }, base)); title('—'); return; }
    c.set(Object.assign({ series: ser, dec: 4 }, base));
    const m0 = msk.muscles.find(m => m.name === nm + '_r') || msk.muscles.find(m => m.name === nm + '_l');
    title(nm + T(' · ความยาว muscle–tendon unit (m) · static ') + (m0.len0 === m0.len0 ? m0.len0.toFixed(3) + ' m' : '—') + ' · Gait 2392 · ' + (msk.source === 'ik' ? 'IK' : '6DOF'));
  } else if (tb === 'qc') {
    const pr = qc && qc.pairs[qcSel];
    if (!pr || !pr.ok) { c.set(Object.assign({ series: [], empty: T('กด “ตรวจ marker” แล้วคลิกคู่ในตาราง') }, base)); title('—'); return; }
    const mmD = Float32Array.from(pr.delta, v => v * 1000), t = E.QC_TOL[pr.grp];
    c.set(Object.assign({ series: [{ name: 'Δ ' + pr.a + '–' + pr.b, color: pr.level === 2 ? '#d63030' : pr.level ? '#c99400' : E.AXIS.z, data: mmD }], dec: 1 }, base,
      { hlines: [{ v: t.p95[0] * 1000, label: 'warn ' + (t.p95[0] * 1000).toFixed(0) + ' mm' }, { v: -t.p95[0] * 1000, label: '−' + (t.p95[0] * 1000).toFixed(0) }, { v: t.p95[1] * 1000, label: 'bad ' + (t.p95[1] * 1000).toFixed(0) + ' mm' }, { v: -t.p95[1] * 1000, label: '−' + (t.p95[1] * 1000).toFixed(0) }] }));
    title('QC · ' + pr.a + '–' + pr.b + ' (' + pr.grp + ') · Δ ระยะจาก static (mm) · static ' + (pr.d0 * 1000).toFixed(0) + ' mm · bias ' + (pr.median * 1000).toFixed(1) + ' · p95 |Δ| ' + (pr.p95 * 1000).toFixed(1) + ' mm');
  } else if (tb === 'kin') {
    const js = $('jsel' + S), J = idr && idr.ok ? idr.joints[js.value] : null, mode = $('ksel' + S).value;
    if (E.UB_JOINTS.includes(js.value)) { c.set(Object.assign({ series: [], empty: T('Kinetics มีเฉพาะขา (สะโพก เข่า ข้อเท้า)') }, base)); title('—'); return; }
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
  for (const [id, v] of [['tabM', 'marker'], ['tabG', 'grf'], ['tabA', 'ang'], ['tabK', 'kin'], ['tabQ', 'qc'], ['tabU', 'mtu'], ['tabF', 'force']]) $(id + S).setAttribute('aria-selected', t === v);
  $('usel' + S).hidden = t !== 'mtu';
  $('jsel' + S).hidden = t !== 'ang' && t !== 'kin'; $('ksel' + S).hidden = t !== 'kin'; $('psel' + S).hidden = t !== 'grf'; if (S) $('msel2').hidden = t !== 'marker';
}
function setTab(t) { PA.tab = t; syncTabs(PA); updatePanel(PA); }
function setTabB(t) { PB.tab = t; syncTabs(PB); updatePanel(PB); }
$('tabM').addEventListener('click', () => setTab('marker'));
$('tabG').addEventListener('click', () => setTab('grf'));
$('tabA').addEventListener('click', () => setTab('ang'));
$('tabF').addEventListener('click', () => setTab('force'));
$('tabK').addEventListener('click', () => setTab('kin'));
$('tabQ').addEventListener('click', () => setTab('qc'));
$('tabU').addEventListener('click', () => setTab('mtu'));
$('tabU2').addEventListener('click', () => setTabB('mtu'));
$('usel').addEventListener('input', () => { mskSelect(); updatePanel(PA); });
$('usel2').addEventListener('input', () => updatePanel(PB));
$('jsel').addEventListener('input', () => updatePanel(PA));
$('ksel').addEventListener('input', () => updatePanel(PA));
$('tabM2').addEventListener('click', () => setTabB('marker'));
$('tabG2').addEventListener('click', () => setTabB('grf'));
$('tabA2').addEventListener('click', () => setTabB('ang'));
$('tabF2').addEventListener('click', () => setTabB('force'));
$('tabK2').addEventListener('click', () => setTabB('kin'));
$('tabQ2').addEventListener('click', () => setTabB('qc'));
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

/* ---------- joint angles (computed in the browser) */
function angParams(staticD) {
  const num = id => parseFloat($(id).value);
  return {
    staticStart: num('aS0') | 0, staticN: num('aSN') | 0,
    kneeW: { R: num('aKR'), L: num('aKL') }, ankleW: { R: num('aAR'), L: num('aAL') }, markerR: num('aMR'),
    normalize: $('aNorm').checked, staticDs: staticD, convention: $('aConv').value, tracking: $('aTrack').value, anthro: getAnthro()
  };
}
function idParams() {
  const num = id => { const v = $(id).value.trim(); return v === '' ? null : parseFloat(v); };
  return { mass: num('subM'), sex: $('subSex').value, cutoff: num('idCut') || 0, cutoffGRF: num('idCutF'), fzThr: num('idThr') || 20, resolve: $('idRes').value, normalize: $('idNorm').checked };
}
function ubParams(staticD) { return { staticDs: staticD, staticStart: parseFloat($('aS0').value) | 0, staticN: parseFloat($('aSN').value) | 0, normalize: $('ubNorm').checked, swapWrist: { R: $('ubSwR').checked, L: $('ubSwL').checked } }; }
function runAngles() {
  if (!ds) return;
  const num = id => parseFloat($(id).value);
  const p = angParams(staticDs);
  const r = E.computeLowerBody(ds, p), n = $('angNote');
  resetKinetics(true);
  if (!r.ok) { kin = kin6 = null; viewer.kin = null; msk = null; viewer.msk = null; $('tabA').disabled = true; $('tabA2').disabled = true; $('bCsv').disabled = true; n.className = 'note err'; n.textContent = r.error; viewer.invalidate(); return; }
  kin = kin6 = r; viewer.kin = r; viewer.invalidate(); $('bIk').disabled = false; $('bId').disabled = false; if ($('mskOn').checked) updateMsk();
  if (r.measured) { const m = r.measured, fx = (id, v) => { if ($(id) && v === v) $(id).value = v.toFixed(3); }; fx('aKR', m.kneeW.R); fx('aKL', m.kneeW.L); fx('aAR', m.ankleW.R); fx('aAL', m.ankleW.L); }
  $('tabA').disabled = false; $('tabA2').disabled = false; $('bCsv').disabled = false;
  const fmt = j => { const g = r.ranges[j].a; return j + ' ' + g[0].toFixed(0) + '…' + g[1].toFixed(0) + '°'; };
  n.className = 'note' + (r.warnings.length ? ' warn' : '');
  n.textContent = (r.convention === 'pig' ? 'Plug-in Gait style (Y-X-Z; + = flexion/dorsiflexion, adduction/varus, internal rot; ankle หัก static offset R ' + r.pigOffset.R.toFixed(1) + '° / L ' + r.pigOffset.L.toFixed(1) + '°; virtual TIB) · ' : '') + 'ชุด marker: ' + r.markerSetLabel + ' · tracking: ' + (r.tracking === 'anat' ? 'anatomical' : 'mixed (v0.2)') + (r.measured ? ' (width วัดจาก marker medial ในไฟล์ ช่อง width ไม่ถูกใช้)' : '') + ' · static: ' + (r.staticSource === 'trial' ? 'trial นี้' : r.staticSource) + ' f' + r.staticRange[0] + '–' + r.staticRange[1] + ' (marker ขยับสูงสุด ' + (r.staticMove * 1000).toFixed(1) + ' mm) · flex: ' + ['RHip', 'RKnee', 'RAnkle', 'LHip', 'LKnee', 'LAnkle'].map(fmt).join(' · ') + (r.warnings.length ? ' · ⚠ ' + r.warnings.join(' · ') : '') + ' · ยังไม่ได้ตรวจเทียบกับซอฟต์แวร์อื่น';
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
  staticDs = d; if (ds) { resetQC(); resetUB(); }
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
  try { setStatic(readFile(await f.arrayBuffer(), f.name)); } catch (err) { const sn = $('staticNote'); sn.className = 'note err'; sn.textContent = 'เปิดไฟล์ static ไม่ได้: ' + err.message; }
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
  kin = useIK ? kinIK : kin6; viewer.kin = kin; viewer.invalidate(); if ($('mskOn').checked) updateMsk();
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
  n.textContent = 'IK · ' + r.ikLabel + ' · ' + r.nPar + ' DOF · marker residual เฉลี่ย ' + mm(r.residual.mean) + ' mm (สูงสุด ' + mm(r.residual.max) + ' mm) · มากสุด: ' + worst + ' mm · ' + (performance.now() - t0).toFixed(0) + ' ms' + (r.warnings.length ? ' · ⚠ ' + r.warnings.join(' · ') : '') + ' · ยังไม่ได้ตรวจเทียบกับซอฟต์แวร์อื่น';
  setTab('ang');
}
function runID() {
  if (!ds || !kin) return;
  const r = E.computeID(ds, kin, idParams());
  const n = $('idNote');
  if (!r.ok) { idr = null; n.className = 'note err'; n.textContent = r.error; $('tabK').disabled = true; $('tabK2').disabled = true; $('bIdCsv').disabled = true; return; }
  idr = r; $('tabK').disabled = false; $('tabK2').disabled = false; $('bIdCsv').disabled = false;
  const pk = j => { const p = r.peaks[j].a; return p[0] === Infinity ? j + ' —' : j + ' ' + p[0].toFixed(2) + '…' + p[1].toFixed(2); };
  const sm = s => { const q = r.support[s].peak; return q ? s + ' ' + q.Ms.toFixed(2) + ' (H ' + q.pct.Hip.toFixed(0) + ' / K ' + q.pct.Knee.toFixed(0) + ' / A ' + q.pct.Ankle.toFixed(0) + ' %)' : s + ' —'; };
  n.className = 'note' + (r.warnings.length ? ' warn' : '');
  n.textContent = 'Newton–Euler · kinematics ' + (r.source === 'ik' ? 'IK' : '6DOF') + ' · BSIP De Leva (1996, ' + (r.sex === 'F' ? 'หญิง' : 'ชาย') + ') · มวล ' + r.mass.toFixed(1) + ' kg · cutoff ' + r.cutoff + ' / GRF ' + r.cutoffGRF + ' Hz · moment sagittal (' + r.units.M + '): ' + ['RHip', 'RKnee', 'RAnkle', 'LHip', 'LKnee', 'LAnkle'].map(pk).join(' · ') + ' · peak support moment ' + sm('R') + ' · ' + sm('L') + (r.warnings.length ? ' · ⚠ ' + r.warnings.join(' · ') : '') + ' · validate แล้วกับข้อมูลสังเคราะห์เท่านั้น ยังไม่ได้ตรวจเทียบกับซอฟต์แวร์อื่น';
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

/* ---------- export what the chart panels show (A, plus B in compare mode) */
function panelData(P, tag) {
  const c = P.c, ser = (c.series || []).filter(x => x.data && x.data.length);
  if (!ser.length || !c.n) return null;
  const ttl = $('ctitle' + P.sfx).textContent.trim(), um = [...ttl.matchAll(/\(([^)]*)\)/g)].map(m => m[1]).find(u => /^(°|[NW]m?(\/kg)?|mm?|m\/s)$/.test(u)), unit = um ? ' (' + um + ')' : '';
  const opt = id => { const e = $(id + P.sfx); return e && e.selectedOptions[0] ? e.selectedOptions[0].textContent.trim() : ''; };
  const leg = () => ($('jsel' + P.sfx).value[0] === 'R' ? 'R' : 'L');
  let ctx;   // short, language-neutral column context
  if (P.tab === 'ang') ctx = opt('jsel') + ' angle';
  else if (P.tab === 'kin') { const k = $('ksel' + P.sfx).value; ctx = k === 'S' ? leg() + ' leg support moment' : k === 'G' ? leg() + ' foot GRF' : opt('jsel') + ' ' + opt('ksel').toLowerCase(); }
  else if (P.tab === 'grf') ctx = (opt('psel') || 'FP') + ' GRF';
  else if (P.tab === 'marker') ctx = (P.sfx ? opt('msel') : ($('msel').value || '')) + ' marker';
  else ctx = 'Force analysis';
  return { tag, n: c.n, rate: c.rate, t0: c.t0 || 0, frames: c.frameOffset != null, ttl, cols: ser.map(x => ({ name: tag + ' · ' + ctx + ' · ' + x.name + unit, data: x.data })) };
}
function exportCharts() {
  if (!ds) return;
  const list = [panelData(PA, 'A')]; if (PB.on) list.push(panelData(PB, 'B'));
  const D = list.filter(Boolean);
  if (!D.length) { note(T('กราฟยังไม่มีข้อมูลให้ส่งออก'), 'warn'); return; }
  const q = v => '"' + String(v).replace(/"/g, '""') + '"', fx = v => (v === v && v != null ? (+v).toFixed(5) : '');
  const same = D.every(d => d.n === D[0].n && d.rate === D[0].rate && d.t0 === D[0].t0 && d.frames === D[0].frames);
  const h = [], rows = [];
  if (same) { if (D[0].frames) h.push('frame'); h.push('time_s'); D.forEach(d => d.cols.forEach(c => h.push(q(c.name)))); }
  else D.forEach(d => { h.push(d.tag + '_time_s'); d.cols.forEach(c => h.push(q(c.name))); });   // different sampling (e.g. analog-rate force analysis): own time column per panel
  const N = Math.max(...D.map(d => d.n));
  for (let i = 0; i < N; i++) {
    const r = [];
    if (same) { if (D[0].frames) r.push(i); r.push((D[0].t0 + i / D[0].rate).toFixed(5)); D.forEach(d => d.cols.forEach(c => r.push(fx(c.data[i])))); }
    else D.forEach(d => { const ok = i < d.n; r.push(ok ? (d.t0 + i / d.rate).toFixed(5) : ''); d.cols.forEach(c => r.push(ok ? fx(c.data[i]) : '')); });
    rows.push(r.join(','));
  }
  const tabName = P => ({ marker: 'Marker', grf: 'GRF', ang: 'Angle', kin: 'Kinetics', force: 'Force' }[P.tab] || P.tab);
  const nm = (ds.name || 'take').replace(/\.c3d$/i, '') + '_Chart_' + (D.length > 1 ? tabName(PA) + '-vs-' + tabName(PB) : tabName(PA)) + '.csv';
  saveText(nm, [h.join(',')].concat(rows).join('\r\n'));
}
$('bChCsv').addEventListener('click', exportCharts);

/* ---------- movement cycles (cut by frame) */
function cyMsg(msg) { const n = $('cyNote'); n.className = 'note warn'; n.textContent = msg; }
function exMsg(msg, kind) { const n = $('exNote'); n.hidden = !msg; n.className = 'note' + (kind ? ' ' + kind : ''); n.textContent = msg || ''; }
function cycleBands(k) { return cycles.map((c, i) => ({ from: c[0] * k, to: c[1] * k, color: i % 2 ? 'rgba(120,90,200,.10)' : 'rgba(120,90,200,.17)', label: 'C' + (i + 1) })); }
function renderCycles() {
  const ul = $('cyList'); ul.textContent = '';
  cycles.forEach((c, i) => {
    const li = document.createElement('li'), sp = document.createElement('span'), go = document.createElement('button'), del = document.createElement('button');
    sp.textContent = 'C' + (i + 1) + ' · f ' + c[0] + '–' + c[1] + ' · ' + (c[0] / ds.rate).toFixed(2) + '–' + (c[1] / ds.rate).toFixed(2) + ' s (' + ((c[1] - c[0]) / ds.rate).toFixed(2) + ' s)';
    go.className = 'btn sm'; go.type = 'button'; go.textContent = '▶'; go.title = T('ไปที่จุดเริ่ม cycle'); go.addEventListener('click', () => { viewer.pause(); viewer.setFrame(c[0]); });
    del.className = 'btn sm'; del.type = 'button'; del.textContent = '✕'; del.title = T('ลบ cycle นี้'); del.addEventListener('click', () => { cycles.splice(i, 1); renderCycles(); updateChart(); });
    li.append(sp, go, del); ul.append(li);
  });
  $('cyClear').disabled = !cycles.length;
  $('cyNote').className = 'note';
  $('cyNote').textContent = cycles.length ? cycles.length + T(' cycle · ส่งออกแบบต่อ cycle จะ normalize แต่ละ cycle เป็น 0–100% (101 จุด) พร้อม mean ± SD') : T('ยังไม่มี cycle · เลื่อนไปเฟรมเริ่ม กด “เริ่ม = เฟรมนี้” แล้วไปเฟรมจบ กด “จบ = เฟรมนี้” จากนั้นกด “เพิ่ม cycle”');
}
function addCycle(f0, f1) {
  f0 = Math.round(f0); f1 = Math.round(f1);
  if (f1 < f0) [f0, f1] = [f1, f0];
  f0 = Math.max(0, f0); f1 = Math.min(ds.nFrames - 1, f1);
  if (!(f1 - f0 >= 2)) { cyMsg(T('cycle ต้องยาวอย่างน้อย 3 เฟรม')); return false; }
  cycles.push([f0, f1]); cycles.sort((a, b) => a[0] - b[0]); return true;
}
$('cyS').addEventListener('click', () => { if (ds) $('cyF0').value = Math.round(viewer.f); });
$('cyE').addEventListener('click', () => { if (ds) $('cyF1').value = Math.round(viewer.f); });
$('cyAdd').addEventListener('click', () => {
  if (!ds) return; const a = parseInt($('cyF0').value, 10), b = parseInt($('cyF1').value, 10);
  if (!(a === a && b === b)) { cyMsg(T('ใส่เฟรมเริ่มและเฟรมจบก่อน')); return; }
  if (addCycle(a, b)) { $('cyF0').value = $('cyF1').value = ''; renderCycles(); updateChart(); }
});
$('cyClear').addEventListener('click', () => { cycles = []; detEv = []; renderCycles(); updateChart(); });
/* automatic cycles: method 'src:side:kind' (src fp | zeni | c3d | id). Shared by the cycle card and batch mode. */
function autoCycles(d, method, idR) {
  const [src, side, kind] = method.split(':'), ctx = side === 'R' ? 'Right' : side === 'L' ? 'Left' : '';
  let ev = [];
  if (src === 'id') {
    if (!idR) return { error: T('สร้าง cycle จาก contact ต้องคำนวณ Inverse Dynamics ก่อน') };
    return { cycles: E.cyclesFromContact(idR.contact[side], kind === 'stance' ? 'stance' : 'stride', Math.round(0.05 * d.rate), Math.round(0.03 * d.rate)), events: [] };
  }
  if (src === 'fp') {
    if (!d.plates.some(p => p.supported)) return { error: T('ไม่มี force plate ที่อ่านได้') };
    ev = E.eventsFromPlates(d);
    if (!side) {   // every contact, whatever the foot
      const out = []; const fs = ev.filter(e => e.label === 'Foot Strike');
      for (const a of fs) { const b = ev.find(e => e.label === 'Foot Off' && e.source === a.source && e.frame > a.frame); if (b) out.push([a.frame, b.frame - 1]); }
      return { cycles: out.sort((x, y) => x[0] - y[0]), events: ev };
    }
  } else if (src === 'zeni') {
    const r = E.eventsGaitZeni(d); if (!r.ok) return { error: r.error }; ev = r.events;
  } else if (src === 'c3d') {
    ev = d.events || []; if (!ev.length) return { error: T('ไฟล์นี้ไม่มี event') };
  }
  const list = E.cyclesFromEvents(ev, ctx, kind);
  return { cycles: kind === 'stance' && src === 'fp' ? list.map(c => [c[0], c[1] - 1]) : list, events: ev };
}
function pkSeries() {
  const sl = $('cyPkS'), keep = sl.value; sl.textContent = '';
  (PA.c.series || []).forEach((s, i) => { const o = document.createElement('option'); o.value = i; o.textContent = s.name; sl.append(o); });
  if (keep && +keep < sl.options.length) sl.value = keep;
}
$('cyAuto').addEventListener('input', () => {
  const pk = /^(peak|valley)$/.test($('cyAuto').value);
  document.querySelectorAll('.pkopt').forEach(e => { e.hidden = !pk; }); if (pk) pkSeries();
});
$('cyPkS').addEventListener('focus', pkSeries);
$('cyAutoGo').addEventListener('click', () => {
  if (!ds) return; const v = $('cyAuto').value; if (!v) return;
  let list;
  if (v === 'peak' || v === 'valley') {
    const S = (PA.c.series || [])[+$('cyPkS').value];
    if (!S || !S.data) { cyMsg(T('กราฟบนยังไม่มีเส้นข้อมูล · เลือกแท็บ/ข้อต่อที่ต้องการก่อน')); return; }
    const y = S.data, k = PA.k || 1;
    let mn = Infinity, mx = -Infinity; for (const x of y) if (x === x) { if (x < mn) mn = x; if (x > mx) mx = x; }
    const r = E.cyclesFromPeaks(y, { valley: v === 'valley', minDist: Math.max(1, Math.round(parseFloat($('cyPkD').value) * ds.rate * k)), minProm: (mx - mn) * (parseFloat($('cyPkP').value) || 0) / 100 });
    list = r.cycles.map(c => [Math.round(c[0] / k), Math.round(c[1] / k)]);
    detEv = r.peaks.map(p => ({ frame: Math.round(p / k), label: v === 'peak' ? 'Peak' : 'Valley', context: S.name }));
  } else {
    const r = autoCycles(ds, v, idr);
    if (r.error) { cyMsg(r.error); return; }
    list = r.cycles; detEv = r.events.map(e => Object.assign({}, e, { label: e.label + (e.context ? ' ' + e.context[0] : '') }));
  }
  if (!list.length) { cyMsg(T('ไม่พบ event ที่ใช้สร้าง cycle ได้')); updateChart(); return; }
  cycles = []; list.forEach(c => addCycle(c[0], c[1])); renderCycles(); updateChart();
});

/* ---------- combined kinematics + kinetics export: per time or per cycle */
function exportVars(wantK, wantD) { return varsFor(kin, ub, idr, wantK, wantD); }
function varsFor(kin, ub, idr, wantK, wantD) {
  const J = ['RHip', 'RKnee', 'RAnkle', 'LHip', 'LKnee', 'LAnkle'], V = [];
  if (wantK && kin) {
    const pg = kin.convention === 'pig', ac = pg ? ['flex', 'add', 'introt'] : ['X', 'Y', 'Z'];
    J.forEach(j => ['a', 'b', 'g'].forEach((c, i) => V.push({ name: j + '_angle_' + ac[i] + '_deg', data: kin.angles[j][c] })));
  }
  if (wantK && ub) for (const j in ub.angles) { const lb = ub.labels[j]; ['a', 'b', 'g'].forEach((c, i) => V.push({ name: j + '_' + lb[i].replace(/[^A-Za-z0-9]+/g, '_').replace(/^_|_$/g, '') + '_deg', data: ub.angles[j][c] })); }
  if (wantD && idr) {
    const U = idr.units, pg = idr.convention === 'pig', mc = pg ? ['flex', 'add', 'introt'] : ['X', 'Y', 'Z'];
    J.forEach(j => { const o = idr.joints[j];
      ['a', 'b', 'g'].forEach((c, i) => V.push({ name: j + '_moment_' + mc[i] + '_' + U.M, data: o[c] }));
      V.push({ name: j + '_power_' + U.P, data: o.P });
      ['Fx', 'Fy', 'Fz'].forEach(c => V.push({ name: j + '_force_' + c + '_' + U.F, data: o[c] })); });
    ['R', 'L'].forEach(s => ['x', 'y', 'z'].forEach(c => V.push({ name: 'GRF_' + s + '_' + c + '_' + U.F, data: idr.grf[s][c] })));
    ['R', 'L'].forEach(s => [['Ms', 'Ms'], ['Hip', 'Hip_ext'], ['Knee', 'Knee_ext'], ['Ankle', 'Ankle_PF']].forEach(([k, n]) => V.push({ name: 'Support_' + s + '_' + n + '_' + U.M, data: idr.support[s][k] })));
  }
  return V;
}
async function exportCombined() {
  if (!ds) return;
  const wantK = $('exK').checked, wantD = $('exD').checked, mode = $('exMode').value;
  if (wantK && !kin && !ub) { exMsg(T('ยังไม่มีมุมข้อต่อ · กด “คำนวณมุมข้อต่อ” ก่อน หรือเอาเครื่องหมายถูกที่ Kinematics ออก'), 'warn'); return; }
  if (wantD && !idr) { exMsg(T('ยังไม่มีผล Inverse Dynamics · กด “คำนวณ Inverse Dynamics” ก่อน หรือเอาเครื่องหมายถูกที่ Kinetics ออก'), 'warn'); return; }
  const V = exportVars(wantK, wantD);
  if (!V.length) { exMsg(T('เลือกข้อมูลที่จะส่งออกอย่างน้อย 1 ชุด'), 'warn'); return; }
  const fx = v => (v === v ? (+v).toFixed(5) : ''), src = kin && kin.source === 'ik' ? 'IK' : '6DOF';
  const stem = (ds.name || 'take').replace(/\.c3d$/i, '') + '_' + [wantK ? 'Kinematics' : '', wantD ? 'Kinetics' : ''].filter(Boolean).join('+') + '_' + src;
  if (mode === 'time') {
    const only = $('exOnly').checked && cycles.length, cyc = new Int16Array(ds.nFrames);
    cycles.forEach((c, i) => { for (let f = c[0]; f <= c[1]; f++) if (!cyc[f]) cyc[f] = i + 1; });
    const rows = [['frame', 'time_s', 'cycle'].concat(V.map(v => v.name)).join(',')];
    for (let f = 0; f < ds.nFrames; f++) { if (only && !cyc[f]) continue; rows.push([f, (f / ds.rate).toFixed(5), cyc[f] ? 'C' + cyc[f] : ''].concat(V.map(v => fx(v.data[f]))).join(',')); }
    const fn = stem + '_time' + (only ? '_cycles-only' : '') + '.csv', r = await saveText(fn, rows.join('\r\n'));
    exMsg(r.ok ? T('ส่งออกแล้ว · ') + fn + ' · ' + (rows.length - 1) + T(' แถว × ') + V.length + T(' ตัวแปร') : r.msg, r.ok ? '' : 'warn');
  } else {
    let auto = false;
    if (!cycles.length) {   // frames typed in but "Add cycle" not pressed: use them
      const a = parseInt($('cyF0').value, 10), b = parseInt($('cyF1').value, 10);
      if (a === a && b === b && addCycle(a, b)) { auto = true; $('cyF0').value = $('cyF1').value = ''; renderCycles(); updateChart(); }
      else { exMsg(T('ยังไม่มี cycle · ใส่เฟรมเริ่ม/จบ แล้วกด “เพิ่ม cycle” ในการ์ด Cycle การเคลื่อนไหว'), 'warn'); return; }
    }
    const NP = 101, h = ['percent'], cols = [];
    for (const v of V) {
      const curves = cycles.map(c => E.cycleNormalize(v.data, c[0], c[1], NP)), st = E.cycleStats(curves);
      curves.forEach((cv, i) => { h.push(v.name + '_C' + (i + 1) + '[' + cycles[i][0] + '-' + cycles[i][1] + ']'); cols.push(cv); });
      h.push(v.name + '_mean', v.name + '_SD'); cols.push(st.mean, st.sd);
    }
    const rows = [h.join(',')];
    for (let k = 0; k < NP; k++) rows.push([k].concat(cols.map(c => fx(c[k]))).join(','));
    const fn = stem + '_cycles-' + cycles.length + 'x101.csv', r = await saveText(fn, rows.join('\r\n'));
    exMsg((auto ? T('เพิ่ม cycle จากเฟรมที่กรอกไว้ให้แล้ว · ') : '') + (r.ok ? T('ส่งออกแล้ว · ') + fn + ' · ' + cycles.length + ' cycle × ' + V.length + T(' ตัวแปร (0–100%, mean ± SD)') : r.msg), r.ok ? '' : 'warn');
  }
}
$('bExport').addEventListener('click', exportCombined);
$('exMode').addEventListener('input', () => { $('exOnlyL').hidden = $('exMode').value !== 'time'; });

/* ---------- marker quality control */
const QC_WHY = { jump: 'ตำแหน่งกระโดดระหว่างเฟรม (label สลับหรือ marker หาย)', mismatch: 'ระยะเข่า–ข้อเท้าด้าน medial กับ lateral เปลี่ยนไม่สอดคล้องกัน (gap-fill ผิด segment หรือ label ผิด)', pairs: 'ระยะกับ marker อื่นบนกระดูกเดียวกันไม่คงที่ (หลุด/ติดใหม่ หรือ soft-tissue artefact สูง)' };
function resetQC() {
  qc = null; qcSel = -1; $('qcCard').hidden = !ds; $('tabQ').disabled = true; $('tabQ2').disabled = true;
  $('qcTbl').hidden = true; $('qcUnit').hidden = true; $('qcSum').className = 'note';
  $('qcSum').textContent = T('เทียบระยะระหว่าง marker บนกระดูกชิ้นเดียวกันกับ static (ใช้ไฟล์ static และช่วงเฟรมเดียวกับการ์ดมุมข้อต่อ) · หา marker ที่ label ผิด, gap-fill ผิด segment, หลุด/ติดใหม่ หรือ soft-tissue artefact สูง');
  if (PA.tab === 'qc') setTab('marker'); if (PB.on && PB.tab === 'qc') setTabB('ang');
}
function runQC() {
  if (!ds) return;
  const r = E.markerQC(ds, { staticDs, staticStart: parseFloat($('aS0').value) | 0, staticN: parseFloat($('aSN').value) | 0 }), n = $('qcSum');
  if (!r.ok) { qc = null; n.className = 'note err'; n.textContent = r.error; $('qcTbl').hidden = true; return; }
  qc = r; $('tabQ').disabled = false; $('tabQ2').disabled = false;
  const mm = v => (v * 1000).toFixed(0), lv = ['ผ่าน', 'มีข้อควรระวัง', 'มีปัญหา'][r.level];
  const xs = r.cross.map(c => c.side + ': ' + c.pairs.join(' vs ') + ' ต่างกัน p95 ' + mm(c.p95) + ' mm').join(' · ');
  const gaps = r.markers.filter(m => m.gap || m.jump).map(m => m.name + (m.gap ? ' หาย ' + m.gap + ' fr' : '') + (m.jump ? ' กระโดด ' + m.jump + ' ครั้ง (แรกที่ f' + m.firstJump + ')' : '')).join(', ');
  n.className = 'note' + (r.level === 2 ? ' err' : r.level ? ' warn' : '');
  n.textContent = ['QC: ' + lv + ' · static: ' + (r.staticSource === 'trial' ? T('trial นี้') : r.staticSource) + ' f' + r.staticRange[0] + '–' + r.staticRange[1]]
    .concat(r.suspects.length ? r.suspects.map(s => '• ' + s.name + ' — ' + s.why.map(w => QC_WHY[w]).join('; ')) : ['ไม่พบ marker ที่น่าสงสัย'])
    .concat(['medial/lateral knee–ankle: ' + xs], gaps ? [gaps] : [], ['คลิกแถวเพื่อดูกราฟ Δ และไปเฟรมที่ห่างมากสุด']).join('\n');
  if (r.suspects.length) { const i = r.pairs.findIndex(p => p.ok && p.level === 2 && r.suspects.some(s => s.name === p.a || s.name === p.b)); qcSel = i >= 0 ? i : r.pairs.findIndex(p => p.ok && p.level > 0); }
  renderQC();
  if (qcSel >= 0) setTab('qc');
}
function renderQC() {
  const tb = $('qcTbl').tBodies[0]; tb.textContent = ''; if (!qc) return;
  const all = $('qcAll').checked, mm = v => (v * 1000).toFixed(0);
  const order = qc.pairs.map((p, i) => i).filter(i => qc.pairs[i].ok && (all || qc.pairs[i].level > 0)).sort((a, b) => qc.pairs[b].level - qc.pairs[a].level || qc.pairs[b].p95 - qc.pairs[a].p95);
  for (const i of order) {
    const p = qc.pairs[i], tr = document.createElement('tr'), td = t => { const d = document.createElement('td'); d.textContent = t; tr.append(d); };
    const b = document.createElement('td'), sp = document.createElement('span'); sp.className = 'qcl l' + p.level; sp.textContent = ['OK', 'WARN', 'BAD'][p.level]; b.append(sp); tr.append(b);
    td(p.a + '–' + p.b); td((p.median >= 0 ? '+' : '') + mm(p.median)); td(mm(p.p95)); td(mm(p.max) + ' @' + p.maxFrame);
    tr.title = p.grp + ' · static ' + mm(p.d0) + ' mm · bias (median Δ) ' + mm(p.median) + ' mm · p95 |Δ| ' + mm(p.p95) + ' mm · max ' + mm(p.max) + ' mm @frame ' + p.maxFrame;
    if (i === qcSel) tr.className = 'sel';
    tr.addEventListener('click', () => { qcSel = i; renderQC(); viewer.pause(); viewer.setFrame(p.maxFrame); setTab('qc'); });
    tb.append(tr);
  }
  $('qcTbl').hidden = !order.length; $('qcUnit').hidden = !order.length;
  if (!order.length) $('qcSum').textContent += T(' · ไม่มีคู่ที่เกินเกณฑ์ (ติ๊ก “แสดงทุกคู่” เพื่อดูทั้งหมด)');
  updateChart();
}
$('bQc').addEventListener('click', runQC);
$('qcAll').addEventListener('input', renderQC);

/* ---------- upper body */
function resetUB() {
  ub = null; viewer.ub = null; viewer.invalidate();
  const d = ds && E.detectUB(ds); $('bUb').disabled = !(d && d.thorax && d.sides.length);
  $('ubNote').className = 'note';
  $('ubNote').textContent = d && d.thorax && d.sides.length ? T('Thorax, trunk, neck, shoulder (Y-X-Y), elbow, wrist ตาม ISB (Wu et al. 2005) · shoulder joint centre: Rab et al. (2002) · ใช้ไฟล์ static และช่วงเฟรมเดียวกับการ์ดมุมข้อต่อ') : T('ไฟล์นี้ไม่มี marker ส่วนบนครบ (ต้องมี C7 T10 CLAV STRN และแขน SHO UPA ELB ELBM FRM WRA WRB)');
  if (!kin) $('tabA').disabled = $('tabA2').disabled = true;
}
function runUB() {
  if (!ds) return;
  const r = E.computeUpperBody(ds, ubParams(staticDs)), n = $('ubNote');
  if (!r.ok) { ub = null; viewer.ub = null; n.className = 'note err'; n.textContent = r.error; return; }
  ub = r; viewer.ub = r; viewer.invalidate(); $('tabA').disabled = false; $('tabA2').disabled = false; if ($('mskOn').checked) updateMsk();
  const rng = (j, c) => { const R = r.ranges[j] && r.ranges[j][c]; return R && R[0] !== Infinity ? R[0].toFixed(0) + '…' + R[1].toFixed(0) + '°' : '—'; };
  const sum = r.sides.map(s => s + ' shoulder elev ' + rng(s + 'Shoulder', 'b') + ', elbow flex ' + rng(s + 'Elbow', 'a') + ', pron ' + rng(s + 'Elbow', 'g')).join(' · ');
  n.className = 'note' + (r.warnings.length ? ' warn' : '');
  n.textContent = 'ISB · static: ' + (r.staticSource === 'trial' ? 'trial นี้' : r.staticSource) + ' f' + r.staticRange[0] + '–' + r.staticRange[1] + ' · fit RMS ' + Object.entries(r.fitSummary).map(([k, v]) => k + ' ' + (v.meanRms * 1000).toFixed(1)).join(' ') + ' mm · ' + sum + (r.warnings.length ? ' · ⚠ ' + r.warnings.join(' · ') : '') + ' · ยังไม่ได้ตรวจเทียบกับซอฟต์แวร์อื่น';
  const js = $('jsel'); if (!E.UB_JOINTS.includes(js.value)) js.value = r.sides.includes('R') ? 'RElbow' : 'LElbow';
  setTab('ang');
}
$('bUb').addEventListener('click', runUB);

/* Save a text file. Inside a Claude artifact the page is sandboxed: a plain browser download is blocked, so the platform's
   `downloads` capability is used (the viewer confirms the save). Elsewhere (own web server, file opened in a browser) a normal
   download. Resolves to { ok, how, msg } so callers report what really happened. */
async function saveText(fname, text) {
  const bin = text instanceof Uint8Array;   // binary (zip): no BOM
  let dl = null;
  try { dl = window.claude && typeof window.claude.use === 'function' ? await window.claude.use('downloads') : null; } catch (e) { dl = null; }
  if (dl) {
    try { await dl.save({ filename: fname, data: bin ? text : '\ufeff' + text }); return { ok: true, how: 'claude' }; }   // BOM: Excel reads UTF-8 (°, Δ)
    catch (e) {
      const c = e && e.code, msg = c === 'declined' ? T('ยกเลิกการบันทึกไฟล์') : c === 'rate_limited' ? T('มีหน้าต่างยืนยันการบันทึกเปิดอยู่ ลองใหม่อีกครั้ง') : T('บันทึกไฟล์ไม่ได้: ') + ((e && (e.message || c)) || '');
      if (c !== 'declined') note(msg, 'err');
      return { ok: false, how: 'claude', msg };
    }
  }
  if (window.claude) {   // in a Claude view without the downloads capability: the sandbox drops <a download> silently
    const msg = T('หน้านี้เปิดใน Claude ซึ่งบล็อกการดาวน์โหลดไฟล์ · เปิด index.html ใน Chrome/Edge หรือจากเว็บของ lab แล้วกดส่งออกอีกครั้ง');
    note(msg, 'err'); return { ok: false, how: 'blocked', msg };
  }
  const url = URL.createObjectURL(bin ? new Blob([text], { type: 'application/zip' }) : new Blob(['\ufeff' + text], { type: 'text/csv;charset=utf-8' })), a = document.createElement('a');
  a.href = url; a.download = fname; document.body.append(a); a.click(); a.remove(); setTimeout(() => URL.revokeObjectURL(url), 2000);
  return { ok: true, how: 'browser' };
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


/* ---------- marker-name mapping (file name -> model name), remembered in this browser */
const MK_KEY = 'primalab.markerMap.v1';
function markerAliases() { try { const v = JSON.parse(localStorage.getItem(MK_KEY) || 'null'); return v && typeof v === 'object' ? v : null; } catch (e) { return null; } }
function storeAliases(m) { try { if (m && Object.keys(m).length) localStorage.setItem(MK_KEY, JSON.stringify(m)); else localStorage.removeItem(MK_KEY); } catch (e) { /* storage blocked: mapping lives for this page only */ } memAliases = m; }
let memAliases = null;
const curAliases = () => markerAliases() || memAliases;
function renderMap() {
  const rep = E.markerReport(ds), ul = $('mkRep'); ul.textContent = '';
  for (const id in rep) {
    const r = rep[id], li = document.createElement('li'), b = document.createElement('b'), sp = document.createElement('span');
    b.className = r.ok ? 'ok' : 'no'; b.textContent = r.ok ? '✓' : '·';
    sp.textContent = T(r.label) + ' · ' + r.found + '/' + r.n + (r.ok ? '' : T(' · ขาด ') + r.missing.slice(0, 6).join(' ') + (r.missing.length > 6 ? ' …' : ''));
    li.append(b, sp); ul.append(li);
  }
  const mapped = Object.keys(ds.mapped || {});
  const id = $('mkModel').value, D = E.MODEL_DEFS[id], have = new Set(ds.base.map(x => x.toUpperCase()));
  const sug = E.suggestMarkerMap(ds, [id]);
  const names = D.required.concat(D.optional).filter(n => !have.has(n) || ds.mapped[n]);
  const tb = $('mkTbl').tBodies[0]; tb.textContent = '';
  const opts = ds.baseFile.map((b, i) => ({ b, i })).filter(o => ds.side[o.i] !== 'U');
  for (const n of names) {
    const tr = document.createElement('tr'), th = document.createElement('td'), td = document.createElement('td'), sel = document.createElement('select');
    th.textContent = n + (D.optional.includes(n) ? ' (opt)' : ''); sel.dataset.n = n;
    const o0 = document.createElement('option'); o0.value = ''; o0.textContent = '—'; sel.append(o0);
    for (const o of opts) { const op = document.createElement('option'); op.value = o.b; op.textContent = o.b + (ds.base[o.i] !== o.b ? ' → ' + ds.base[o.i] : ''); sel.append(op); }
    sel.value = ds.mapped[n] || sug[n] || '';
    sel.addEventListener('change', () => { $('bMkApply').disabled = false; });
    td.append(sel); tr.append(th, td); tb.append(tr);
  }
  $('mkTbl').hidden = !names.length;
  $('bMkApply').disabled = !names.length || !(Object.keys(sug).length || mapped.length);
  const mn = $('mkNote');
  mn.className = 'note';
  mn.textContent = (mapped.length ? T('ใช้ mapping อยู่ ') + mapped.length + T(' ชื่อ: ') + mapped.map(n => ds.mapped[n] + '→' + n).slice(0, 8).join(', ') + ' · ' : '')
    + (names.length ? T('เลือก marker ในไฟล์ให้ตรงกับชื่อที่ model ใช้ (เดาให้จากชื่อที่พบบ่อย) · ตำแหน่งทางกายวิภาคต้องตรงกับนิยามของ model') : T('ไฟล์นี้มี marker ครบตามชื่อของ model ที่เลือก'));
}
function applyMap(m) {
  storeAliases(m);
  const keep = staticDs;
  E.finishDataset(ds, m || {}); if (!m) { ds.aliasMap = null; }
  if (keep) { E.finishDataset(keep, m || {}); if (!m) keep.aliasMap = null; }
  load(ds); if (keep) setStatic(keep);
}
$('mkModel').addEventListener('change', () => ds && renderMap());
$('bMkApply').addEventListener('click', () => {
  const m = Object.assign({}, curAliases() || {});
  $('mkTbl').querySelectorAll('select').forEach(sel => {
    for (const k in m) if (m[k] === sel.dataset.n) delete m[k];
    if (sel.value) m[sel.value] = sel.dataset.n;
  });
  applyMap(m);
});
$('bMkClear').addEventListener('click', () => applyMap(null));
$('bMkSave').addEventListener('click', async () => {
  const m = curAliases() || {};
  const r = await saveText('primalab_marker_map.json', JSON.stringify({ format: 'primalab-marker-map', version: 1, map: m }, null, 2));
  if (r.ok) { $('mkNote').className = 'note'; $('mkNote').textContent = T('ส่งออก mapping ') + Object.keys(m).length + T(' ชื่อ'); }
});
$('bMkLoad').addEventListener('click', () => $('fileMap').click());
$('fileMap').addEventListener('change', async e => {
  const f = e.target.files[0]; e.target.value = ''; if (!f) return;
  try {
    const j = JSON.parse((await f.text()).replace(/^﻿/, ''));
    const m = j && j.map && typeof j.map === 'object' ? j.map : j;
    if (!m || typeof m !== 'object' || Array.isArray(m)) throw new Error(T('รูปแบบไฟล์ไม่ถูกต้อง'));
    const clean = {}; for (const k in m) if (typeof m[k] === 'string' && m[k]) clean[k] = m[k].toUpperCase();
    applyMap(clean);
  } catch (err) { $('mkNote').className = 'note err'; $('mkNote').textContent = T('นำเข้า mapping ไม่ได้: ') + err.message; }
});


/* ---------- batch: many trials with the current settings */
let batch = null;   // { rows: [...summary], curves: Map(varName -> ...), files }
function btLog(name, msg, kind) {
  const li = document.createElement('li'), sp = document.createElement('span');
  sp.textContent = name + ' · ' + msg; if (kind) sp.className = kind; li.append(sp); $('btLog').append(li);
}
function btMsg(msg, kind) { const n = $('btNote'); n.hidden = !msg; n.className = 'note' + (kind ? ' ' + kind : ''); n.textContent = msg || ''; }
async function runBatch(files) {
  const wantK = $('btK').checked, wantD = $('btD').checked, method = $('btCy').value, useIK = $('btSrc').value === 'ik';
  if (!wantK && !wantD) { btMsg(T('เลือก Kinematics หรือ Kinetics อย่างน้อย 1 อย่าง'), 'warn'); return; }
  $('btLog').textContent = ''; $('btSum').disabled = $('btCur').disabled = true;
  const NP = 101, rows = [], curves = [], varNames = [];
  let nOk = 0, nCyc = 0;
  for (let fi = 0; fi < files.length; fi++) {
    const file = files[fi];
    btMsg(T('กำลังประมวลผล ') + (fi + 1) + '/' + files.length + ' · ' + file.name);
    await new Promise(r => setTimeout(r, 0));
    try {
      const d = readFile(await file.arrayBuffer(), file.name);
      const lb = E.detectLBSet(d) ? E.computeLowerBody(d, angParams(staticDs)) : null;
      let kinR = lb && lb.ok ? lb : null, idR = null, ubR = null;
      if (kinR && useIK) { const r = E.computeIK(d, kinR, { model: $('ikModel').value }); if (r.ok) kinR = r; }
      if (wantK && E.detectUB(d).thorax) { const r = E.computeUpperBody(d, ubParams(staticDs)); if (r.ok) ubR = r; }
      if ((wantD || method.startsWith('id:')) && kinR) { const r = E.computeID(d, kinR, idParams()); if (r.ok) idR = r; else if (wantD) btLog(file.name, 'ID: ' + r.error, 'warn'); }
      const V = varsFor(kinR, ubR, idR, wantK, wantD);
      if (!V.length) { btLog(file.name, lb && !lb.ok ? lb.error : T('ไม่มีตัวแปรที่คำนวณได้ (marker set ไม่ตรง model? ดูการ์ด mapping)'), 'err'); continue; }
      let cyc;
      if (method === 'trial') cyc = [[0, d.nFrames - 1]];
      else { const r = autoCycles(d, method, idR); if (r.error) { btLog(file.name, r.error, 'err'); continue; } cyc = r.cycles.filter(c => c[1] - c[0] >= 2); }
      if (!cyc.length) { btLog(file.name, T('ไม่พบ cycle (ลองเลือก “ทั้ง trial = 1 cycle” หรือวิธีอื่น)'), 'warn'); continue; }
      for (const v of V) if (!varNames.includes(v.name)) varNames.push(v.name);
      cyc.forEach((c, ci) => {
        const cv = {};
        for (const v of V) {
          const sm = E.cycleSummary(v.data, c[0], c[1]);
          rows.push([file.name, ci + 1, c[0], c[1], (c[0] / d.rate).toFixed(4), ((c[1] - c[0]) / d.rate).toFixed(4), v.name, sm.mean, sm.min, sm.max, sm.range, sm.tMinPct, sm.tMaxPct, sm.start, sm.end, sm.validPct]);
          cv[v.name] = E.cycleNormalize(v.data, c[0], c[1], NP);
        }
        curves.push({ file: file.name, cycle: ci + 1, f0: c[0], f1: c[1], cv });
      });
      nOk++; nCyc += cyc.length;
      btLog(file.name, cyc.length + ' cycle · ' + V.length + T(' ตัวแปร') + (kinR ? ' · ' + (kinR.source === 'ik' ? 'IK' : '6DOF') + ' ' + kinR.markerSetLabel : '') + (ubR ? ' + UB' : '') + (idR ? ' + ID' : ''));
    } catch (err) { btLog(file.name, T('อ่าน/คำนวณไม่ได้: ') + err.message, 'err'); }
  }
  batch = { rows, curves, varNames, nFiles: files.length };
  $('btSum').disabled = $('btCur').disabled = !rows.length;
  btMsg(nOk + '/' + files.length + T(' ไฟล์สำเร็จ · ') + nCyc + T(' cycle · กดส่งออกด้านล่าง'), nOk === files.length ? '' : 'warn');
}
$('btPick').addEventListener('click', () => $('btFiles').click());
$('btFiles').addEventListener('change', e => { const f = [...e.target.files]; e.target.value = ''; if (f.length) runBatch(f); });
const csvq = v => { const t = String(v); return /[",\r\n]/.test(t) ? '"' + t.replace(/"/g, '""') + '"' : t; };
const cnum = v => (typeof v === 'number' ? (v === v ? +v.toFixed(5) : '') : v);
$('btSum').addEventListener('click', async () => {
  if (!batch) return;
  const h = ['file', 'cycle', 'frame_start', 'frame_end', 'time_start_s', 'duration_s', 'variable', 'mean', 'min', 'max', 'range', 'time_of_min_pct', 'time_of_max_pct', 'value_start', 'value_end', 'valid_pct'];
  const out = [h.join(',')].concat(batch.rows.map(r => r.map(v => csvq(cnum(v))).join(',')));
  const fn = 'PrimaLab_batch_summary_' + batch.nFiles + 'files.csv', r = await saveText(fn, out.join('\r\n'));
  btMsg(r.ok ? T('ส่งออกแล้ว · ') + fn + ' · ' + batch.rows.length + T(' แถว (long format: 1 แถว = ไฟล์ × cycle × ตัวแปร)') : r.msg, r.ok ? '' : 'warn');
});
$('btCur').addEventListener('click', async () => {
  if (!batch) return;
  const out = [['file', 'cycle', 'frame_start', 'frame_end', 'percent'].concat(batch.varNames).map(csvq).join(',')];
  for (const c of batch.curves) for (let k = 0; k < 101; k++) out.push([csvq(c.file), c.cycle, c.f0, c.f1, k].concat(batch.varNames.map(n => c.cv[n] ? cnum(c.cv[n][k]) : '')).join(','));
  const fn = 'PrimaLab_batch_curves_' + batch.nFiles + 'files.csv', r = await saveText(fn, out.join('\r\n'));
  btMsg(r.ok ? T('ส่งออกแล้ว · ') + fn + ' · ' + batch.curves.length + ' cycle × 101 × ' + batch.varNames.length + T(' ตัวแปร') : r.msg, r.ok ? '' : 'warn');
});


/* ---------- musculoskeletal overlay (bones + muscle lines) */
let msk = null;
const MSK_GROUPS = [['Hip', ['glut_max1', 'glut_max2', 'glut_max3', 'glut_med1', 'glut_med2', 'glut_med3', 'glut_min1', 'glut_min2', 'glut_min3', 'iliacus', 'psoas', 'add_long', 'add_brev', 'add_mag1', 'add_mag2', 'add_mag3', 'pect', 'grac', 'sar', 'tfl', 'gem', 'peri', 'quad_fem']],
  ['Knee', ['rect_fem', 'vas_med', 'vas_int', 'vas_lat', 'semimem', 'semiten', 'bifemlh', 'bifemsh']],
  ['Ankle / foot', ['med_gas', 'lat_gas', 'soleus', 'tib_post', 'tib_ant', 'flex_dig', 'flex_hal', 'per_brev', 'per_long', 'per_tert', 'ext_dig', 'ext_hal']],
  ['Trunk', ['ercspn', 'intobl', 'extobl']]];
function fillMuscleSel() {
  for (const S of ['', '2']) {
    const sl = $('usel' + S), keep = sl.value; sl.textContent = '';
    if (!msk) continue;
    const have = new Set(msk.muscles.map(m => m.name.replace(/_[rl]$/, ''))), done = new Set();
    for (const [g, list] of MSK_GROUPS) {
      const og = document.createElement('optgroup'); og.label = g;
      for (const n of list) if (have.has(n)) { const o = document.createElement('option'); o.value = n; o.textContent = n; og.append(o); done.add(n); }
      if (og.children.length) sl.append(og);
    }
    const rest = [...have].filter(n => !done.has(n)); if (rest.length) { const og = document.createElement('optgroup'); og.label = 'Other'; for (const n of rest) { const o = document.createElement('option'); o.value = n; o.textContent = n; og.append(o); } sl.append(og); }
    sl.value = keep && have.has(keep) ? keep : (have.has('rect_fem') ? 'rect_fem' : sl.options[0] && sl.options[0].value);
  }
}
function mskSelect() {
  if (!msk) return;
  const nm = $('usel').value, f = $('mskF').value;
  viewer.mskOpt.sel = msk.muscles.findIndex(m => m.name === nm + '_r');
  if (viewer.mskOpt.sel < 0) viewer.mskOpt.sel = msk.muscles.findIndex(m => m.name === nm + '_l');
  viewer.mskOpt.filter = f === 'all' ? null : f === 'sel' ? (m => m.name.replace(/_[rl]$/, '') === nm) : (m => m.name.endsWith(f === 'R' ? '_r' : '_l'));
  viewer.invalidate();
}
function renderMsk() {
  const ul = $('mskRep'); ul.textContent = '';
  if (!msk) return;
  for (const seg in msk.fit) {
    const r = msk.fit[seg], li = document.createElement('li'), b = document.createElement('b'), sp = document.createElement('span');
    b.className = r.rms < 0.03 ? 'ok' : 'no'; b.textContent = r.rms < 0.03 ? '✓' : '!';
    sp.textContent = T(E.MSK_SEG_LABEL[seg]) + ' · scale ' + r.scale.toFixed(2) + ' · landmark RMS ' + (r.rms * 1000).toFixed(0) + ' mm';
    li.append(b, sp); ul.append(li);
  }
}
function mskMsg(t, kind) { const n = $('mskNote'); n.className = 'note' + (kind ? ' ' + kind : ''); n.textContent = t; }
function updateMsk() {
  const on = $('mskOn').checked;
  if (!on || !ds || !kin) {
    msk = null; viewer.msk = null; $('tabU').disabled = $('tabU2').disabled = true; renderMsk(); fillMuscleSel(); viewer.invalidate();
    if (on && ds && !kin) mskMsg(T('ต้องคำนวณมุมข้อต่อส่วนล่าง (6DOF หรือ IK) ก่อน'), 'warn');
    if (PA.tab === 'mtu') updatePanel(PA); if (PB.tab === 'mtu') updatePanel(PB);
    return;
  }
  const r = E.buildMSK(ds, kin, ub && ub.ok ? ub : null);
  if (!r.ok) { msk = null; viewer.msk = null; mskMsg(r.error, 'err'); renderMsk(); return; }
  msk = r; viewer.msk = r; viewer.mskOpt.bones = $('mskB').checked; viewer.mskOpt.muscles = $('mskM').checked; viewer.mskOpt.alpha = parseFloat($('mskA').value);
  $('tabU').disabled = $('tabU2').disabled = false;
  fillMuscleSel(); mskSelect(); renderMsk();
  const gl = (viewer._mskR || (viewer._mskR = new E.MSKRenderer())).ok;
  mskMsg(r.meshes.length + T(' ชิ้นกระดูก · ') + r.muscles.length + T(' กล้ามเนื้อ · จาก ') + (kin.source === 'ik' ? 'IK' : '6DOF') + (ub && ub.ok ? T(' + ส่วนบน') : T(' (ไม่มีส่วนบน: ลำตัววางตาม pelvis)')) +
    (r.warnings.length ? ' · ' + r.warnings.join(' · ') : '') + (gl ? '' : T(' · เบราว์เซอร์นี้ไม่รองรับ WebGL: แสดงเฉพาะกล้ามเนื้อ')), r.warnings.length || !gl ? 'warn' : '');
  if (PA.tab === 'mtu') updatePanel(PA); if (PB.tab === 'mtu') updatePanel(PB);
  viewer.invalidate();
}
$('mskOn').addEventListener('input', updateMsk);
$('mskB').addEventListener('input', () => { viewer.mskOpt.bones = $('mskB').checked; viewer.invalidate(); });
$('mskM').addEventListener('input', () => { viewer.mskOpt.muscles = $('mskM').checked; viewer.invalidate(); });
$('mskA').addEventListener('input', () => { viewer.mskOpt.alpha = parseFloat($('mskA').value); viewer.invalidate(); });
$('mskF').addEventListener('input', mskSelect);


/* ---------- OpenSim export (zip: trc, grf mot/xml, marker set, setup files, model, run script) */
function osMsg(t, kind) { const n = $('osNote'); n.className = 'note' + (kind ? ' ' + kind : ''); n.textContent = t; }
$('bOsim').addEventListener('click', async () => {
  if (!ds) return;
  if (!kin) osMsg(T('ยังไม่ได้คำนวณมุมข้อต่อ: ไม่มี joint centre และแบ่งแรงให้เท้าจากช่วงสัมผัสแทน'), 'warn');
  const mass = parseFloat($('subM').value);
  const nm = (ds.name || 'trial').replace(/\.c3d$/i, '').replace(/^[0-9a-f]{8}-/, '');
  try {
    const b = await E.opensimZip(ds, { kin, staticDs, name: nm, mass: mass > 0 ? mass : 70, heading: $('osHead').value, includeJC: $('osJC').checked, grfCutoff: parseFloat($('osCut').value) || 0, cutoff: parseFloat($('idCut').value) || 6 });
    if (!b.ok) { osMsg(b.error, 'err'); return; }
    const fn = 'PrimaLab_OpenSim_' + b.info.name + '.zip', r = await saveText(fn, b.zip);
    if (!r.ok) { osMsg(r.msg, 'warn'); return; }
    osMsg(T('ส่งออกแล้ว · ') + fn + ' · ' + b.info.markers + ' marker · IK ' + b.info.ikTasks + ' marker · scale: ' + b.info.measurements.join(', ') + (b.info.grfRate ? ' · GRF ' + b.info.grfRate + ' Hz' : '') +
      (mass > 0 ? '' : T(' · ไม่ได้ใส่น้ำหนักตัว ใช้ 70 kg')) + (b.warnings.length ? ' · ' + b.warnings.join(' · ') : ''), b.warnings.length || !(mass > 0) ? 'warn' : '');
  } catch (err) { osMsg(T('ส่งออกไม่ได้: ') + err.message, 'err'); }
});

/* ---------- file loading */
let lastFile = null;   // {buf, name}: kept so the vertical axis can be changed without re-opening
function readFile(buf, name) { return E.readC3D(buf, name, { up: $('upSel').value, aliases: curAliases() }); }
async function openFile(file) {
  try {
    const buf = await file.arrayBuffer();
    lastFile = { buf, name: file.name };
    load(readFile(buf, file.name));
  } catch (err) { note('เปิดไฟล์ไม่ได้: ' + err.message, 'err'); }
}
$('upSel').addEventListener('change', () => {
  if (!lastFile || !ds || ds.synthetic) return;
  try { load(readFile(lastFile.buf, lastFile.name)); } catch (err) { note('อ่านไฟล์ใหม่ไม่ได้: ' + err.message, 'err'); }
});
$('file').addEventListener('change', e => { const f = e.target.files[0]; if (f) openFile(f); e.target.value = ''; });
$('btnSample').addEventListener('click', () => { lastFile = null; load(E.makeSyntheticCMJ()); });
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
