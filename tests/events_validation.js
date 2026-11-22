/* Event detection: Zeni (2008) gait events on synthetic walking with known foot strike / foot off (both walking
   directions and a turn-free return), force-plate contact events with side assignment, generic peak detection,
   and events -> cycles. */
'use strict';
const E = require('../engine.js');
let fails = 0; const check = (ok, m) => { if (!ok) { fails++; console.log('  FAIL ' + m); } };

/* synthetic walk: pelvis at constant speed v along ±x, each foot fixed in stance, smooth swing (cosine) */
function walk(dirSign, rate) {
  const v = 1.2, T = 1.1, duty = 0.62, nF = Math.round(5.5 * rate), L = v * T;
  const names = ['LASI', 'RASI', 'LPSI', 'RPSI', 'LHEE', 'RHEE', 'LTOE', 'RTOE'];
  const nM = names.length, pts = new Float32Array(nF * nM * 3), vis = new Uint8Array(nF * nM).fill(1);
  const truth = { R: { FS: [], FO: [] }, L: { FS: [], FO: [] } };
  const phase0 = { R: 0, L: 0.5 };
  for (const s of ['R', 'L']) for (let k = -1; k < 8; k++) {
    const t0 = (k + phase0[s]) * T;                 // foot strike time
    const fs = Math.round(t0 * rate), fo = Math.round((t0 + duty * T) * rate);
    if (fs > 2 && fs < nF - 3) truth[s].FS.push(fs);
    if (fo > 2 && fo < nF - 3) truth[s].FO.push(fo);
  }
  const footX = (s, t) => {        // heel x of foot s at time t (relative to the start), along the walking axis
    const u = t / T - phase0[s], k = Math.floor(u), ph = u - k;
    const xs = (k + phase0[s]) * L + 0.25 * L;      // heel lands a bit in front of the pelvis
    if (ph < duty) return xs;                         // stance: fixed
    const w = (ph - duty) / (1 - duty);               // swing to the next landing
    return xs + L * (0.5 - 0.5 * Math.cos(Math.PI * w));
  };
  for (let f = 0; f < nF; f++) {
    const t = f / rate, px = v * t;
    const put = (n, x, y, z) => { const i = names.indexOf(n), k = (f * nM + i) * 3; pts[k] = dirSign * x; pts[k + 1] = dirSign * y; pts[k + 2] = z; };
    put('LASI', px + 0.1, 0.12, 0.95); put('RASI', px + 0.1, -0.12, 0.95); put('LPSI', px - 0.08, 0.05, 1.0); put('RPSI', px - 0.08, -0.05, 1.0);
    for (const s of ['R', 'L']) { const y = s === 'R' ? -0.1 : 0.1, hx = footX(s, t); put(s + 'HEE', hx, y, 0.06); put(s + 'TOE', hx + 0.2, y, 0.04); }
  }
  const ds = E.finishDataset({ name: 'walk', rate, nFrames: nF, firstFrame: 1, labels: names, pts, vis, plates: [], events: [], warnings: [], info: {} });
  return { ds, truth };
}
for (const [dirSign, rate] of [[1, 100], [-1, 100], [1, 200]]) {
  const { ds, truth } = walk(dirSign, rate);
  const r = E.eventsGaitZeni(ds);
  check(r.ok, 'zeni failed ' + r.error);
  const tol = Math.round(0.05 * rate);   // 50 ms vs the true contact: this synthetic swing decelerates slowly (cosine), so the heel–sacrum
                                         // maximum comes early; Zeni 2008 report ≈16 ms mean error vs force plates in real gait
  let worst = 0, missed = 0;
  for (const s of ['R', 'L']) for (const [lab, key] of [['Foot Strike', 'FS'], ['Foot Off', 'FO']]) {
    const got = r.events.filter(e => e.context === (s === 'R' ? 'Right' : 'Left') && e.label === lab).map(e => e.frame);
    for (const t of truth[s][key]) {
      const d = Math.min(...got.map(g => Math.abs(g - t)));
      if (!(d <= tol)) missed++; else worst = Math.max(worst, d);
    }
    check(got.length <= truth[s][key].length + 1, s + ' ' + lab + ' extra events ' + got.length + ' vs ' + truth[s][key].length);
  }
  check(missed === 0, 'zeni missed ' + missed + ' events (dir ' + dirSign + ', ' + rate + ' Hz)');
  // implementation check: events sit on the extrema of the (unfiltered) Zeni signals within 1 frame
  let impl = 0;
  for (const s of ['R', 'L']) {
    const nm = ds.base, iS = [nm.indexOf('LPSI'), nm.indexOf('RPSI')], iH = nm.indexOf(s + 'HEE'), iT = nm.indexOf(s + 'TOE');
    const pos = (i, f) => ds.pts[(f * ds.nM + i) * 3] * dirSign;
    const raw = (i, f) => pos(i, f) - (pos(iS[0], f) + pos(iS[1], f)) / 2;
    for (const e of r.events.filter(e => e.context === (s === 'R' ? 'Right' : 'Left'))) {
      const i = e.label === 'Foot Strike' ? iH : iT, sg = e.label === 'Foot Strike' ? 1 : -1;
      let best = e.frame; for (let f = Math.max(0, e.frame - 8); f <= Math.min(ds.nFrames - 1, e.frame + 8); f++) if (sg * raw(i, f) > sg * raw(i, best)) best = f;
      impl = Math.max(impl, Math.abs(best - e.frame));
    }
  }
  check(impl <= 1, 'events off the raw Zeni extrema by ' + impl + ' frames');
  const st = E.cyclesFromEvents(r.events, 'Right', 'stride'), sc = E.cyclesFromEvents(r.events, 'Right', 'stance');
  const T = Math.round(1.1 * rate), D = Math.round(0.62 * 1.1 * rate);
  check(st.length >= 3 && st.every(c => Math.abs(c[1] - c[0] - T) <= 2 * tol), 'stride lengths ' + JSON.stringify(st.map(c => c[1] - c[0])));
  check(sc.length >= 3 && sc.every(c => Math.abs(c[1] - c[0] - D) <= 2 * tol), 'stance lengths ' + JSON.stringify(sc.map(c => c[1] - c[0])));
  console.log('Zeni dir ' + (dirSign > 0 ? '+x' : '−x') + ' @' + rate + ' Hz: worst |Δ| vs true contact ' + (worst / rate * 1000).toFixed(0) + ' ms, ' + st.length + ' R strides, ' + sc.length + ' R stances');
}
/* force-plate events on the synthetic CMJ (both feet, one plate each) */
{
  const d = E.makeSyntheticCMJ(), ev = E.eventsFromPlates(d);
  const fo = ev.filter(e => e.label === 'Foot Off'), fs = ev.filter(e => e.label === 'Foot Strike');
  check(fo.length >= 1 && fs.length >= 1, 'CMJ plate events: ' + ev.length);
  const sides = new Set(ev.map(e => e.context));
  console.log('Force plate events (CMJ): ' + ev.map(e => e.label + '/' + e.context + '@' + e.frame).join(', '));
  check(!sides.has('') , 'side missing');
}
/* peaks */
{
  const y = Float64Array.from({ length: 400 }, (_, i) => Math.sin(2 * Math.PI * i / 50) + 0.05 * Math.sin(2 * Math.PI * i / 7));
  const p = E.detectPeaks(y, { minDist: 30, minProm: 0.5 });
  check(p.length === 8 && p.every((v, k) => k === 0 || Math.abs(v - p[k - 1] - 50) <= 6), 'peaks ' + JSON.stringify(p));
  const v = E.detectPeaks(y, { minDist: 30, minProm: 0.5, valley: true });
  check(v.length === 8, 'valleys ' + v.length);
  const c = E.cyclesFromPeaks(y, { minDist: 30, minProm: 0.5 });
  check(c.cycles.length === 7, 'peak cycles ' + c.cycles.length);
  console.log('Peaks: ' + p.join(' ') + ' · valleys: ' + v.join(' '));
}
/* cycle summary */
{
  const y = Float64Array.from({ length: 101 }, (_, i) => Math.sin(Math.PI * i / 100) * 10);
  const s = E.cycleSummary(y, 0, 100);
  check(Math.abs(s.max - 10) < 1e-9 && Math.abs(s.tMaxPct - 50) < 1e-9 && Math.abs(s.min) < 1e-9 && s.validPct === 100, 'cycleSummary ' + JSON.stringify(s));
}
console.log(fails ? 'EVENTS FAIL' : 'EVENTS PASS'); process.exit(fails ? 1 : 0);
