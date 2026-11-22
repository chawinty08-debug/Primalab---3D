/* Force-plate types 1–4, processor formats 84/85/86 and vertical-axis detection.
   Synthetic C3D files with a known load (force, COP, free moment) are written with tests/c3d_writer.js
   and read back; every quantity must be recovered. */
'use strict';
const E = require('../engine.js');
const { writeC3D } = require('./c3d_writer.js');

let fails = 0;
const check = (ok, msg) => { if (!ok) { fails++; console.log('  FAIL ' + msg); } };
const near = (a, b, tol) => Math.abs(a - b) <= tol;

/* load: lab Z-up, plate centred at (cx0, cy0, 0), 400 × 600 mm, plate x = lab x, plate y = −lab y, plate z down */
const PL = { cx0: 300, cy0: 500, w: 400, l: 600 };
const cornersZup = (dx = 0) => {
  const x0 = PL.cx0 + dx, y0 = PL.cy0, hx = PL.w / 2, hy = PL.l / 2;
  // plate +x+y, -x+y, -x-y, +x-y  (plate y = −lab y)
  return [[x0 + hx, y0 - hy, 0], [x0 - hx, y0 - hy, 0], [x0 - hx, y0 + hy, 0], [x0 + hx, y0 + hy, 0]];
};
const nF = 40, spf = 5, rate = 100;
// time-varying load on the body (lab frame, N, mm, N·mm)
const load = s => {
  const t = s / (rate * spf);
  return { F: [40 * Math.sin(3 * t), -25 + 10 * t, 650 + 200 * Math.sin(2 * t)], cop: [60 * Math.cos(2 * t), -80 + 50 * t], Tz: 4000 * Math.sin(5 * t) };
};
function plateSignals(type, L, origin) {
  // action on the plate in plate axes
  const Fa = [-L.F[0], +L.F[1], +L.F[2]];          // −F (action), then lab→plate: x→x, y→−y, z→−z
  const c = [L.cop[0], -L.cop[1]];                  // COP in plate axes relative to the centre
  const Tz = L.Tz;                                   // free moment about lab +z on the body → plate-z (down) action: (−Tz)·(−1) = Tz
  const [Fx, Fy, Fz] = Fa;
  // moment about the plate centre (surface): c × F + Tz ẑ
  const Mc = [c[1] * Fz, -c[0] * Fz, c[0] * Fy - c[1] * Fx + Tz];
  if (type === 1) return [Fx, Fy, Fz, c[0], c[1], Tz];
  if (type === 2 || type === 4) {
    const o = origin[2] > 0 ? origin.map(x => -x) : origin;   // reader convention: z ≤ 0
    // M_raw = M_c − F × o
    const Fxo = [Fy * o[2] - Fz * o[1], Fz * o[0] - Fx * o[2], Fx * o[1] - Fy * o[0]];
    return [Fx, Fy, Fz, Mc[0] - Fxo[0], Mc[1] - Fxo[1], Mc[2] - Fxo[2]];
  }
  if (type === 3) {
    const [a, b, az0] = origin;
    const A = (Mc[0] - Fy * az0) / b, B = (Mc[1] + Fx * az0) / a, S = Fz;
    const fx34mfx12 = Mc[2] / b;
    return [(Fx - fx34mfx12) / 2, (Fx + fx34mfx12) / 2, Fy / 2, Fy / 2, (S + A - B) / 4, (S + A + B) / 4, (S - A + B) / 4, (S - A - B) / 4];
  }
}
// markers (Z-up, mm): pelvis + feet + head, slowly moving
const LBL = ['LASI', 'RASI', 'LPSI', 'RPSI', 'LTOE', 'RTOE', 'LHEE', 'RHEE', 'LFHD', 'RFHD'];
const base = { LASI: [380, 420, 950], RASI: [220, 420, 950], LPSI: [370, 580, 980], RPSI: [230, 580, 980], LTOE: [380, 380, 40], RTOE: [220, 380, 40], LHEE: [370, 600, 60], RHEE: [230, 600, 60], LFHD: [360, 440, 1700], RFHD: [240, 440, 1700] };
const pointsZup = () => { const P = []; for (let f = 0; f < nF; f++) P.push(LBL.map(n => [base[n][0] + f, base[n][1], base[n][2] + 0.5 * f])); return P; };

function build(type, ptype, yUp) {
  const origin = type === 3 ? [210, 350, -53] : type === 1 ? [0, 0, 0] : [1.2, -0.8, 42];
  const nch = type === 3 ? 8 : 6;
  const cal = type === 4 ? (() => { const m = new Array(36).fill(0); for (let i = 0; i < 6; i++) m[i + 6 * i] = 2; m[1 + 6 * 0] = 0.1; return m; })() : null;
  // with CAL (column-major in C3D, cal(r,c) = CAL[r + 6c]): signals v = CAL⁻¹ w
  const inv = w => { if (!cal) return w; const v = w.slice(); v[0] = w[0] / 2; v[1] = (w[1] - 0.1 * v[0]) / 2; for (let i = 2; i < 6; i++) v[i] = w[i] / 2; return v; };
  const analog = [];
  for (let s = 0; s < nF * spf; s++) analog.push(inv(plateSignals(type, load(s), origin)));
  const toFile = p => yUp ? [p[0], p[2], -p[1]] : p;    // Z-up lab → file with Y up
  const pts = pointsZup().map(r => r.map(toFile));
  const corners = cornersZup().map(toFile);
  return writeC3D({
    processor: ptype, rate, nFrames: nF, spf, labels: LBL, points: pts, units: 'mm',
    analogLabels: Array.from({ length: nch }, (_, i) => 'FP1_' + (i + 1)), analog,
    plates: [{ type, corners, origin, channels: Array.from({ length: nch }, (_, i) => i + 1), cal }],
    yScreen: yUp ? '+Y' : '+Z'
  });
}

for (const type of [1, 2, 3, 4]) for (const ptype of [84, 85, 86]) for (const yUp of [false, true]) {
  const tag = 'type ' + type + ' proc ' + ptype + (yUp ? ' Y-up' : ' Z-up');
  let d;
  try { d = E.readC3D(build(type, ptype, yUp), tag); } catch (e) { check(false, tag + ' read error: ' + e.message); continue; }
  const pl = d.plates[0];
  check(d.up.detected === (yUp ? '+Y' : '+Z'), tag + ' up detected ' + d.up.detected + ' via ' + d.up.source);
  check(pl && pl.supported, tag + ' plate not supported: ' + (pl && pl.note));
  if (!pl || !pl.supported) continue;
  // markers back in Z-up metres
  const P0 = pointsZup();
  let ep = 0; for (let f = 0; f < nF; f++) LBL.forEach((n, i) => { for (let a = 0; a < 3; a++) ep = Math.max(ep, Math.abs(d.pts[(f * d.nM + i) * 3 + a] - P0[f][i][a] / 1000)); });
  let eF = 0, eC = 0, eT = 0;
  for (let f = 0; f < nF; f++) {
    const L = load(f * spf);
    for (let a = 0; a < 3; a++) eF = Math.max(eF, Math.abs(pl.F[f * 3 + a] - L.F[a]));
    const cop = [(PL.cx0 + L.cop[0]) / 1000, (PL.cy0 + L.cop[1]) / 1000, 0];
    for (let a = 0; a < 3; a++) eC = Math.max(eC, Math.abs(pl.cop[f * 3 + a] - cop[a]));
    eT = Math.max(eT, Math.abs(pl.T[f * 3 + 2] - L.Tz / 1000), Math.abs(pl.T[f * 3]), Math.abs(pl.T[f * 3 + 1]));
  }
  let eZ = 0; for (let s = 0; s < nF * spf; s++) eZ = Math.max(eZ, Math.abs(pl.fzFull[s] - load(s).F[2]));
  const tol = ptype === 85 ? 2e-3 : 1e-3;
  check(ep < 1e-6, tag + ' marker error ' + ep);
  check(eF < tol * 5, tag + ' force error ' + eF.toExponential(2) + ' N');
  check(eC < 1e-5, tag + ' COP error ' + (eC * 1000).toExponential(2) + ' mm');
  check(eT < 1e-4, tag + ' free moment error ' + eT.toExponential(2) + ' N·m');
  check(eZ < tol * 5, tag + ' analog-rate Fz error ' + eZ.toExponential(2));
  console.log(tag.padEnd(26) + ' up ' + d.up.detected + '  |ΔF| ' + eF.toExponential(1) + ' N  |ΔCOP| ' + (eC * 1000).toExponential(1) + ' mm  |ΔTz| ' + eT.toExponential(1) + ' N·m');
}
// override: forcing '+Z' on a Y-up file must leave coordinates untouched
{
  const d = E.readC3D(build(2, 84, true), 'override', { up: '+Z' });
  check(d.up.applied === '+Z' && d.up.detected === '+Y', 'override not honoured');
  check(Math.abs(d.pts[1] - 0.950) < 1e-6 && Math.abs(d.pts[2] + 0.420) < 1e-6, 'override changed coordinates');
}
console.log(fails ? 'PLATE TYPES FAIL (' + fails + ')' : 'PLATE TYPES PASS');
process.exit(fails ? 1 : 0);
