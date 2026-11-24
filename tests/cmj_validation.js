/* CMJ analyzer on the synthetic force-plate CMJ with known answers, both as a dataset and written to / read from a
   force-plate-only C3D (no markers): jump height (impulse–momentum and flight time), flight duration, body mass and
   the left/right split must be recovered. */
'use strict';
const E = require('../engine.js'), { writeC3D } = require('./c3d_writer.js');
let fails = 0; const check = (ok, m) => { if (!ok) { fails++; console.log('  FAIL ' + m); } };
const d = E.makeSyntheticCMJForce(), T = d.truth;
const metric = (r, re) => { for (const g of r.groups) for (const x of g.rows) if (re.test(x.name)) return x; return null; };
// force-only C3D: two type-2 plates, plate z down (Fz channel = −load), no markers
const spf = d.plates[0].spf, nF = d.nFrames, an = [];
for (let s = 0; s < nF * spf; s++) an.push([0, 0, -d.plates[0].fzFull[s], 0, 0, 0, 0, 0, -d.plates[1].fzFull[s], 0, 0, 0]);
const C = (x0, y0) => [[x0 + 300, y0 + 300, 0], [x0 - 300, y0 + 300, 0], [x0 - 300, y0 - 300, 0], [x0 + 300, y0 - 300, 0]];
const buf = writeC3D({ rate: d.rate, nFrames: nF, spf, labels: [], points: Array.from({ length: nF }, () => []), analogLabels: Array.from({ length: 12 }, (_, i) => 'FP' + (i < 6 ? 1 : 2) + '_' + i), analog: an,
  plates: [{ type: 2, corners: C(300, 300), origin: [0, 0, -40], channels: [1, 2, 3, 4, 5, 6] }, { type: 2, corners: C(300, -300), origin: [0, 0, -40], channels: [7, 8, 9, 10, 11, 12] }] });
const dc = E.readC3D(buf, 'cmj_forceonly.c3d');
check(dc.nM === 0 && dc.plates.every(p => p.supported), 'force-only C3D not read as 0 markers + 2 plates');
for (const [tag, ds] of [['dataset', d], ['force-only C3D', dc]]) {
  const r = E.FA.analyzePlates(ds, {});
  check(r.ok, tag + ' analysis failed ' + r.error);
  if (!r.ok) continue;
  const jhI = parseFloat(metric(r, /Jump Height – Impulse/).t), jhF = parseFloat(metric(r, /Jump Height – Flight/).t), ft = parseFloat(metric(r, /Flight Duration/).t), bm = metric(r, /Body Mass/);
  console.log(tag.padEnd(15) + ' JH impulse ' + jhI + ' cm (true ' + (T.jumpHeightImpulse * 100).toFixed(1) + ') · JH flight ' + jhF + ' cm (true ' + (T.jumpHeightFlight * 100).toFixed(1) + ') · flight ' + ft + ' s (true ' + T.flightTime.toFixed(3) + ') · mass L/R/total ' + bm.l + '/' + bm.r + '/' + bm.t);
  check(Math.abs(jhI - T.jumpHeightImpulse * 100) < 0.5, tag + ' jump height (impulse) off');
  check(Math.abs(jhF - T.jumpHeightFlight * 100) < 0.6, tag + ' jump height (flight) off');
  check(Math.abs(ft - T.flightTime) < 0.005, tag + ' flight duration off');
  check(Math.abs(parseFloat(bm.t) - T.mass) < 0.3 && Math.abs(parseFloat(bm.l) / parseFloat(bm.t) - T.leftShare) < 0.01, tag + ' body mass / split off');
}
console.log(fails ? 'CMJ FAIL' : 'CMJ PASS'); process.exit(fails ? 1 : 0);
