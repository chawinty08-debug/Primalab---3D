/* OpenSim export: TRC / GRF .mot contents against the dataset (inverse of the lab -> OpenSim rotation),
   per-foot resultant of several plates (force, moment about the origin, free moment preserved), zip structure + CRC. */
'use strict';
const fs = require('fs'), path = require('path'), zlib = require('zlib');
eval(fs.readFileSync(path.join(__dirname, '..', 'msk_data.js'), 'utf8').replace('const MSK_DATA', 'global.MSK_DATA'));
const E = require('../engine.js');
let fails = 0; const check = (ok, m) => { if (!ok) { fails++; console.log('  FAIL ' + m); } };
(async () => {
  const d = E.makeSyntheticCMJ(), k = E.computeLowerBody(d);
  const b = await E.opensimZip(d, { kin: k, name: 'cmj', mass: 75 });
  check(b.ok, 'bundle ' + b.error);
  const need = ['cmj.trc', 'cmj_static.trc', 'cmj_grf.mot', 'cmj_grf.xml', 'PrimaLab_markers_gait2392.xml', 'setup_scale.xml', 'setup_ik.xml', 'setup_id.xml', 'run_opensim.py', 'README.txt', 'gait2392_thelen2003muscle.osim'];
  for (const n of need) check(b.files[n], 'missing ' + n);
  const C = b.C, inv = q => [0, 1, 2].map(a => C[0][a] * q[0] + C[1][a] * q[1] + C[2][a] * q[2]);   // Cᵀ (C is orthonormal)
  // C must be a proper rotation with lab +Z -> OpenSim +Y
  const det = C[0][0] * (C[1][1] * C[2][2] - C[1][2] * C[2][1]) - C[0][1] * (C[1][0] * C[2][2] - C[1][2] * C[2][0]) + C[0][2] * (C[1][0] * C[2][1] - C[1][1] * C[2][0]);
  check(Math.abs(det - 1) < 1e-12 && Math.abs(C[1][2] - 1) < 1e-12, 'C is not a rotation taking Z to Y');
  // TRC
  const T = b.files['cmj.trc'].split('\n'), names = T[3].split('\t').slice(2).filter(Boolean);
  check(+T[2].split('\t')[2] === d.nFrames && +T[2].split('\t')[3] === names.length, 'TRC header counts');
  let eP = 0;
  for (const f of [0, 250, 600]) {
    const row = T[6 + f].split('\t');
    names.forEach((n, j) => {
      const i = d.base.indexOf(n); if (i < 0) return;
      const q = [+row[2 + 3 * j], +row[3 + 3 * j], +row[4 + 3 * j]].map(v => v / 1000), p = inv(q), kk = (f * d.nM + i) * 3;
      for (let a = 0; a < 3; a++) eP = Math.max(eP, Math.abs(p[a] - d.pts[kk + a]));
    });
  }
  check(eP < 1e-6, 'TRC marker error ' + eP);
  // GRF .mot: R + L resultant = all plates (force and moment about the lab origin), whatever the foot assignment
  const M = b.files['cmj_grf.mot'].split('\n'), h = M.indexOf('endheader'), cols = M[h + 1].split('\t');
  const col = n => cols.indexOf(n);
  let eF = 0, eC = 0, nOn = 0;
  for (let f = 0; f < d.nFrames; f += 7) {
    if (!d.plates.every(p => p.F[f * 3 + 2] > 20 || p.F[f * 3 + 2] <= 1e-9)) continue;
    const row = M[h + 2 + f].split('\t').map(Number), Ft = [0, 0, 0], Mt = [0, 0, 0], Fr = [0, 0, 0], Mr = [0, 0, 0];
    for (const pre of ['', '1_']) {
      const F = inv([row[col(pre + 'ground_force_vx')], row[col(pre + 'ground_force_vy')], row[col(pre + 'ground_force_vz')]]);
      const P = inv([row[col(pre + 'ground_force_px')], row[col(pre + 'ground_force_py')], row[col(pre + 'ground_force_pz')]]);
      const t = row[col(pre + 'ground_torque_y')];
      for (let a = 0; a < 3; a++) Ft[a] += F[a];
      Mt[0] += P[1] * F[2] - P[2] * F[1]; Mt[1] += P[2] * F[0] - P[0] * F[2]; Mt[2] += P[0] * F[1] - P[1] * F[0] + t;
    }
    for (const p of d.plates) { if (!(p.F[f * 3 + 2] > 20)) continue; const F = [0, 1, 2].map(a => p.F[f * 3 + a]), c = [0, 1, 2].map(a => p.cop[f * 3 + a]); for (let a = 0; a < 3; a++) Fr[a] += F[a]; Mr[0] += c[1] * F[2] - c[2] * F[1]; Mr[1] += c[2] * F[0] - c[0] * F[2]; Mr[2] += c[0] * F[1] - c[1] * F[0]; }
    if (!(Fr[2] > 20)) continue; nOn++;
    for (let a = 0; a < 3; a++) { eF = Math.max(eF, Math.abs(Ft[a] - Fr[a])); eC = Math.max(eC, Math.abs(Mt[a] - Mr[a])); }
  }
  check(nOn > 50 && eF < 1e-2 && eC < 1e-2, 'GRF mot vs plates: n ' + nOn + ' |ΔF| ' + eF + ' |ΔM| ' + eC);
  // several plates under one foot: resultant force, moment about the origin and free moment preserved
  {
    const one = d.plates[0], mk = (dx, sF, tz) => { const n = d.nFrames, F = new Float32Array(n * 3), cop = new Float32Array(n * 3), T3 = new Float32Array(n * 3);
      for (let f = 0; f < n; f++) { for (let a = 0; a < 3; a++) F[f * 3 + a] = one.F[f * 3 + a] * sF; cop.set([one.cop[f * 3] + dx, one.cop[f * 3 + 1] + 0.05, 0.002], f * 3); T3[f * 3 + 2] = tz; }
      return Object.assign({}, one, { F, cop, T: T3, Ffull: null, index: 10 + dx }); };
    const P1 = mk(0, 0.6, 1.5), P2 = mk(0.12, 0.4, -0.7), dd = Object.assign({}, d, { plates: [P1, P2] });
    const g = E.footGRF(dd, { kin: k }), sd = g.R.F.some(v => v) ? 'R' : 'L', O = g[sd];
    let eM = 0, n = 0;
    for (let f = 0; f < d.nFrames; f++) {
      if (!(O.F[f * 3 + 2] > 20)) continue; n++;
      const F = [0, 1, 2].map(a => O.F[f * 3 + a]), Pp = [0, 1, 2].map(a => O.P[f * 3 + a]);
      const Mx = [Pp[1] * F[2] - Pp[2] * F[1], Pp[2] * F[0] - Pp[0] * F[2], Pp[0] * F[1] - Pp[1] * F[0] + O.T[f]];
      const ref = [0, 0, 0]; for (const p of [P1, P2]) { const f3 = [0, 1, 2].map(a => p.F[f * 3 + a]), c = [0, 1, 2].map(a => p.cop[f * 3 + a]); ref[0] += c[1] * f3[2] - c[2] * f3[1]; ref[1] += c[2] * f3[0] - c[0] * f3[2]; ref[2] += c[0] * f3[1] - c[1] * f3[0] + p.T[f * 3 + 2]; }
      for (let a = 0; a < 3; a++) eM = Math.max(eM, Math.abs(Mx[a] - ref[a]));
    }
    check(n > 20 && eM < 1e-3, 'two-plate resultant moment error ' + eM + ' over ' + n);
    console.log('two plates under one foot: max |ΔM about origin| ' + eM.toExponential(1) + ' N·m over ' + n + ' frames');
  }
  // zip: every local header present, CRC matches
  const z = Buffer.from(b.zip); let off = 0, nf = 0, crcOk = true;
  while (z.readUInt32LE(off) === 0x04034b50) {
    const crc = z.readUInt32LE(off + 14), sz = z.readUInt32LE(off + 18), nl = z.readUInt16LE(off + 26), name = z.toString('utf8', off + 30, off + 30 + nl);
    const data = z.subarray(off + 30 + nl, off + 30 + nl + sz); if (zlib.crc32(data) !== crc) crcOk = false;
    if (name.endsWith('.osim')) check(data.toString('utf8', 0, 60).includes('<?xml'), 'osim content');
    off += 30 + nl + sz; nf++;
  }
  check(crcOk && nf === Object.keys(b.files).length, 'zip entries ' + nf + ' crc ' + crcOk);
  console.log('TRC |Δ| ' + eP.toExponential(1) + ' m · GRF |ΔF| ' + eF.toExponential(1) + ' N, |ΔM| ' + eC.toExponential(1) + ' N·m (R+L vs plates) · zip ' + nf + ' files, ' + (b.zip.length / 1024).toFixed(0) + ' kB');
  console.log(fails ? 'OPENSIM EXPORT FAIL' : 'OPENSIM EXPORT PASS'); process.exit(fails ? 1 : 0);
})();
