/* Musculoskeletal overlay: (1) placement maths: a subject that is the model scaled and moved must be recovered exactly
   (scale, landmark RMS ≈ 0, bone vertices, muscle-path points); (2) on the synthetic CMJ, muscle–tendon lengths must
   change with the physiological sign: knee flexion lengthens the vasti, hip flexion lengthens gluteus maximus, ankle
   dorsiflexion lengthens soleus. */
'use strict';
const fs = require('fs'), path = require('path');
eval(fs.readFileSync(path.join(__dirname, '..', 'msk_data.js'), 'utf8').replace('const MSK_DATA', 'global.MSK_DATA'));
const E = require('../engine.js');
let fails = 0; const check = (ok, m) => { if (!ok) { fails++; console.log('  FAIL ' + m); } };
const D = E.mskData();

// ---- 1. exact recovery: subject = model × s, rotated R0, shifted t0; every segment then moves rigidly on its own
const rot = (ax, a) => { const [x, y, z] = ax, c = Math.cos(a), s = Math.sin(a), C = 1 - c; return [c + x * x * C, x * y * C - z * s, x * z * C + y * s, y * x * C + z * s, c + y * y * C, y * z * C - x * s, z * x * C - y * s, z * y * C + x * s, c + z * z * C]; };
const mv = (R, v) => [R[0] * v[0] + R[1] * v[1] + R[2] * v[2], R[3] * v[0] + R[4] * v[1] + R[5] * v[2], R[6] * v[0] + R[7] * v[1] + R[8] * v[2]];
const mm = (A, B) => { const o = []; for (let r = 0; r < 3; r++) for (let c = 0; c < 3; c++) o.push(A[r * 3] * B[c] + A[r * 3 + 1] * B[3 + c] + A[r * 3 + 2] * B[6 + c]); return o; };
const s0 = 1.07, R0 = rot([0.3, 0.9, 0.2].map(v => v / Math.hypot(0.3, 0.9, 0.2)), 0.8), t0 = [0.4, -0.2, 0.95];
const toSub = p => mv(R0, p).map((v, i) => s0 * 0 + v * s0 + t0[i]);
const segs = ['PV', 'RTH', 'RSK', 'RFT', 'LTH', 'LSK', 'LFT'], key = { PV: 'RPV' };
const nF = 30, S = {}, L = {}, frames = {}, seg = {}, motion = {};
for (const sg of segs) for (const n in D.landmarks[sg]) { const v = toSub(D.landmarks[sg][n]); if (/JC$|KNM$|ANM$/.test(n)) L[n] = v; else S[n] = v; }
for (const sg of segs) {
  const k = key[sg] || sg, o0 = toSub(Object.values(D.landmarks[sg])[0]), Rs = R0;   // static segment frame: any orthonormal frame
  seg[k] = { o: o0, R: Rs };
  const a = new Float32Array(nF * 12); motion[sg] = [];
  for (let f = 0; f < nF; f++) {
    const Rm = rot([Math.sin(f + segs.indexOf(sg)), Math.cos(f), 0.5].map((v, _, A) => v / Math.hypot(...A)), 0.05 * f), tm = [0.01 * f, -0.005 * f, 0.002 * f];
    motion[sg].push({ R: Rm, t: tm });
    const R = mm(Rm, Rs), o = mv(Rm, o0).map((v, i) => v + tm[i]);
    a.set([o[0], o[1], o[2], R[0], R[3], R[6], R[1], R[4], R[7], R[2], R[5], R[8]], f * 12);
  }
  frames[k] = a;
}
const zero = new Float32Array(nF);
const lb = { ok: true, source: '6dof', convention: 'zxy', model: { S, L, seg }, landmarks: L, frames, segNames: Object.keys(frames), angles: { RKnee: { a: zero }, LKnee: { a: zero }, RHip: { a: zero }, LHip: { a: zero } } };
const ds = { nFrames: nF, rate: 100 };
const r = E.buildMSK(ds, lb, null);
check(r.ok, 'build ' + r.error);
let es = 0, er = 0; for (const sg of segs) { es = Math.max(es, Math.abs(r.fit[sg].scale - s0)); er = Math.max(er, r.fit[sg].rms); }
check(es < 1e-9 && er < 1e-9, 'scale/RMS not recovered: ' + es + ' ' + er);
// bone vertices: expected = motion(f) ∘ (model → subject)
let ev = 0;
for (const m of r.meshes.filter(m => segs.includes(m.seg))) for (const f of [0, 13, 29]) for (let i = 0; i < m.v.length; i += 3 * 7) {
  const p = [m.v[i], m.v[i + 1], m.v[i + 2]], M = r.mats[m.seg], b = f * 12;
  const got = [0, 1, 2].map(k => M[b + k * 3] * p[0] + M[b + k * 3 + 1] * p[1] + M[b + k * 3 + 2] * p[2] + M[b + 9 + k]);
  const mo = motion[m.seg][f], exp = mv(mo.R, toSub(p)).map((v, k) => v + mo.t[k]);
  ev = Math.max(ev, Math.hypot(got[0] - exp[0], got[1] - exp[1], got[2] - exp[2]));
}
check(ev < 1e-5, 'bone vertex error ' + ev);
// muscle lengths at frame 0 (no motion yet besides the f = 0 identity): equal to s0 × model polyline length at the neutral pose
let el = 0, nm = 0;
for (const mu of r.muscles) {
  if (mu.pts.some(p => p.mov)) continue;   // muscles with knee-dependent points are checked in part 2
  const P = []; for (const p of mu.pts) {
    if (p.cond && !(0 >= p.cond.lo && 0 <= p.cond.hi)) continue;
    P.push([p.o[0] + p.p[0], p.o[1] + p.p[1], p.o[2] + p.p[2]]);
  }
  let l = 0; for (let k = 1; k < P.length; k++) l += Math.hypot(...[0, 1, 2].map(c => P[k][c] - P[k - 1][c]));
  el = Math.max(el, Math.abs(mu.len[0] - s0 * l)); nm++;
}
check(el < 1e-6, 'muscle length error ' + el);
console.log('placement: |Δscale| ' + es.toExponential(1) + ', landmark RMS ' + er.toExponential(1) + ' m, bone vertices ' + ev.toExponential(1) + ' m, ' + nm + ' muscle lengths ' + el.toExponential(1) + ' m');

// ---- 2. physiological signs on the synthetic CMJ (knee/hip flexion, ankle dorsiflexion during the countermovement)
const d = E.makeSyntheticCMJ(), k = E.computeLowerBody(d), m = E.buildMSK(d, k, null);
check(m.ok, 'CMJ build ' + m.error);
const corr = (x, y) => { let n = 0, sx = 0, sy = 0, sxx = 0, syy = 0, sxy = 0; for (let i = 0; i < x.length; i++) { if (!(x[i] === x[i] && y[i] === y[i])) continue; n++; sx += x[i]; sy += y[i]; sxx += x[i] * x[i]; syy += y[i] * y[i]; sxy += x[i] * y[i]; } return (n * sxy - sx * sy) / Math.sqrt((n * sxx - sx * sx) * (n * syy - sy * sy)); };
const len = n => m.muscles.find(x => x.name === n).len;
const kneeFlex = Float32Array.from(k.angles.RKnee.a, v => -v), hipFlex = k.angles.RHip.a, dorsi = k.angles.RAnkle.a;
const tests = [['vas_lat_r', kneeFlex, +1, 'knee flexion'], ['vas_med_r', kneeFlex, +1, 'knee flexion'], ['glut_max2_r', hipFlex, +1, 'hip flexion'], ['soleus_r', dorsi, +1, 'ankle dorsiflexion'], ['tib_ant_r', dorsi, -1, 'ankle dorsiflexion'], ['iliacus_r', hipFlex, -1, 'hip flexion']];
for (const [n, q, sg, what] of tests) { const c = corr(len(n), q); console.log('  ' + n.padEnd(12) + ' vs ' + what.padEnd(18) + ' r = ' + c.toFixed(3)); check(sg * c > 0.8, n + ' should ' + (sg > 0 ? 'lengthen' : 'shorten') + ' with ' + what + ' (r ' + c.toFixed(2) + ')'); }
console.log(fails ? 'MSK FAIL' : 'MSK PASS'); process.exit(fails ? 1 : 0);
