/* Gap filling: gaps are cut out of a complete trial (synthetic CMJ with simulated marker noise and drop-outs) and filled again;
   the fill is compared with the removed truth. Cases: random interior gaps (rigid / spline / auto), gaps at the
   start and end (rigid only), and a marker missing for the whole trial filled from the static. */
'use strict';
const E = require('../engine.js');
let fails = 0; const check = (ok, m) => { if (!ok) { fails++; console.log('  FAIL ' + m); } };
const d0 = E.makeSyntheticCMJ(), nF = d0.nFrames, nM = d0.nM;
let seed = 3; const rnd = () => (seed = (seed * 1103515245 + 12345) % 2147483648) / 2147483648;
const cut = (spec) => {   // spec: [markerName, f0, f1]
  const vis = Uint8Array.from(d0.vis), pts = Float32Array.from(d0.pts);
  for (const [n, a, b] of spec) { const i = d0.base.indexOf(n); for (let f = a; f <= b; f++) { vis[f * nM + i] = 0; pts.fill(0, (f * nM + i) * 3, (f * nM + i) * 3 + 3); } }
  return Object.assign(Object.create(Object.getPrototypeOf(d0)), d0, { vis, pts });
};
const err = (r, kind) => {
  let e = 0, n = 0, mx = 0;
  for (let k = 0; k < nF * nM; k++) if (r.ds.filled[k] === kind && d0.vis[k]) { const d = Math.hypot(r.ds.pts[k * 3] - d0.pts[k * 3], r.ds.pts[k * 3 + 1] - d0.pts[k * 3 + 1], r.ds.pts[k * 3 + 2] - d0.pts[k * 3 + 2]); e += d * d; n++; if (d > mx) mx = d; }
  return { rms: n ? Math.sqrt(e / n) * 1000 : 0, max: mx * 1000, n };
};
const labelled = d0.base.filter((b, i) => d0.side[i] !== 'U');
// 1. random interior gaps, 3 per marker, 3–40 frames (15–200 ms at 200 Hz)
const spec = [];
for (const n of labelled) for (let g = 0; g < 3; g++) { const L = 3 + Math.floor(rnd() * 38), a = 20 + Math.floor(rnd() * (nF - 80)); spec.push([n, a, a + L - 1]); }
const dc = cut(spec);
for (const method of ['rigid', 'spline', 'pattern', 'auto']) {
  const r = E.fillGaps(dc, { method, maxGap: 40 });
  const er = err(r, 1), es = err(r, 2), ep = err(r, 3);
  console.log(method.padEnd(8) + 'missing ' + r.totals.missing + ' · rigid ' + er.n + ' (RMS ' + er.rms.toFixed(2) + ' mm, max ' + er.max.toFixed(2) + ') · spline ' + es.n + ' (RMS ' + es.rms.toFixed(2) + ', max ' + es.max.toFixed(2) + ') · pattern ' + ep.n + ' (RMS ' + ep.rms.toFixed(2) + ', max ' + ep.max.toFixed(2) + ') · left ' + r.totals.left);
  if (method !== 'spline' && method !== 'pattern') check(er.n > 0 && er.rms < 6 && er.max < 15, method + ' rigid error RMS ' + er.rms.toFixed(2) + ' max ' + er.max.toFixed(2));
  if (es.n) check(es.rms < 15, method + ' spline error RMS ' + es.rms.toFixed(2));
  if (ep.n && method === 'auto') check(ep.rms < 30, method + ' pattern error RMS ' + ep.rms.toFixed(2));   // pattern = translation only: long gaps on rotating segments
  if (method === 'auto') check(r.totals.left === 0, 'auto left ' + r.totals.left);
  check(r.ds.pts !== dc.pts && dc.vis.reduce((a, v) => a + v, 0) < nF * nM, 'original arrays must not change');
}
// 2. gaps at the start and the end: spline cannot extrapolate, rigid can
{
  const r = E.fillGaps(cut([['RASI', 0, 49], ['LPSI', nF - 60, nF - 1]]), { method: 'auto' }), er = err(r, 1);
  console.log('edges  rigid ' + er.n + ' (RMS ' + er.rms.toFixed(2) + ' mm) · left ' + r.totals.left);
  check(er.n >= 100 && er.rms < 6 && r.totals.left === 0, 'edge gaps ' + er.n + ' ' + er.rms);
}
// 3. a marker missing in the whole trial, reference geometry from the static (first 0.5 s of the complete trial)
{
  const st = Object.assign(Object.create(Object.getPrototypeOf(d0)), d0, { nFrames: 100 });
  const r = E.fillGaps(cut([['LPSI', 0, nF - 1]]), { method: 'auto', staticDs: st }), er = err(r, 1);
  console.log('static LPSI from ' + r.report[0].donors.join(' ') + ': ' + er.n + ' frames, RMS ' + er.rms.toFixed(2) + ' mm, max ' + er.max.toFixed(2) + ' mm');
  check(er.n >= nF * 0.9 && er.rms < 6, 'whole-trial fill from static: ' + er.n + ' RMS ' + er.rms);
}
console.log(fails ? 'GAP FILL FAIL' : 'GAP FILL PASS'); process.exit(fails ? 1 : 0);
