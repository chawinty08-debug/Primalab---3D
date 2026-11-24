/* C3D reader + force-plate validation against ezc3d (independent C++ implementation)
   on the public ezc3d test files (https://github.com/pyomeca/ezc3d-testFiles).
   Reference values: tests/fixtures/ezc3d_ref.json (tests/tools/make_ezc3d_ref.py).
   The C3D files are not stored in this repo: set C3D_REF_DIR (CI clones them), otherwise the test is skipped. */
'use strict';
const fs = require('fs'), path = require('path');
const E = require('../engine.js');
const REF = JSON.parse(fs.readFileSync(process.env.C3D_REF_JSON || path.join(__dirname, 'fixtures', 'ezc3d_ref.json'), 'utf8'));
const DIR = process.env.C3D_REF_DIR || path.join(__dirname, '..', '..', 'ezc3d-testFiles');
if (!fs.existsSync(DIR)) { console.log('SKIP: ezc3d test files not found (set C3D_REF_DIR)'); process.exit(0); }

const TOL = { pt: 1e-4, an: 1e-3, F: 1e-2, cop: 1e-3, tz: 1e-2 };   // m, V/N, N, m, N·m
let fails = 0, checks = 0;
const bad = (m) => { fails++; console.log('  FAIL ' + m); };
for (const [fn, R] of Object.entries(REF.files)) {
  const fp = path.join(DIR, fn);
  if (!fs.existsSync(fp)) { console.log(fn + ': missing, skipped'); continue; }
  const b = fs.readFileSync(fp), ab = b.buffer.slice(b.byteOffset, b.byteOffset + b.length);
  let d;
  try { d = E.readC3D(ab, fn, { fpZero: false }); }   // auto vertical axis (reference rotated by the same R below); FP_ZERO off like ezc3d
  catch (e) { if (!R.nPoints) { console.log(fn + ': no markers (ezc3d agrees), reader says: ' + e.message); continue; } bad(fn + ' read error ' + e.message); continue; }
  const r = { pt: 0, an: 0, F: 0, cop: 0, tz: 0 };
  const RU = d.up.R, rot = v => v && v.every(x => x !== null) ? [0, 1, 2].map(a => RU[a][0] * v[0] + RU[a][1] * v[1] + RU[a][2] * v[2]) : null;
  if (d.nFrames !== R.nFrames) bad(fn + ' nFrames ' + d.nFrames + ' vs ' + R.nFrames);
  if (d.nM !== R.nPoints) bad(fn + ' nPoints ' + d.nM + ' vs ' + R.nPoints);
  if (R.nPoints === 0 && d.nM !== 0) bad(fn + ' should have no markers');
  R.frames.forEach((f, fi) => R.markers.forEach((m, mi) => {
    const ref = rot(R.points[fi][mi]), k = f * d.nM + m;
    if (!ref) return;
    if (!d.vis[k]) { if (Math.hypot(...ref) > 1e-6) bad(fn + ' frame ' + f + ' marker ' + m + ' missing'); return; }
    for (let a = 0; a < 3; a++) r.pt = Math.max(r.pt, Math.abs(d.pts[k * 3 + a] - ref[a]));
    checks++;
  }));
  if (R.analog && d.analog) {
    const na = d.info.analogChannels, spf = R.analogPerFrame;
    R.frames.forEach((f, fi) => R.analog[fi].forEach((v, c) => { if (v === null) return; r.an = Math.max(r.an, Math.abs(d.analog[(f * spf) * na + c] - v) / Math.max(1, Math.abs(v))); checks++; }));
  }
  R.plates.forEach((P, k) => {
    const pl = d.plates[k];
    if (!pl || !pl.supported) { console.log('  ' + fn + ' plate ' + (k + 1) + ' (type ' + P.type + ') not read: ' + (pl ? pl.note : 'missing')); return; }
    /* Force sign is a convention: the viewer always reports the reaction on the body with vertical
       component up, ezc3d keeps the sign of the calibrated signals. One global sign per plate is fitted;
       magnitude, direction, COP and free moment must then agree sample by sample. */
    let dot = 0; R.frames.forEach((f, fi) => { const Fr = rot(P.force[fi]); if (Fr) for (let a = 0; a < 3; a++) dot += Fr[a] * pl.F[f * 3 + a]; });
    const sg = dot < 0 ? -1 : 1;
    const C = pl.corners, e1 = [0, 1, 2].map(i => C[0][i] - C[1][i]), e2 = [0, 1, 2].map(i => C[0][i] - C[3][i]);
    const cr = [e1[1] * e2[2] - e1[2] * e2[1], e1[2] * e2[0] - e1[0] * e2[2], e1[0] * e2[1] - e1[1] * e2[0]], ln = Math.hypot(...cr), pn = cr.map(x => x / ln);
    R.frames.forEach((f, fi) => {
      const Fr = rot(P.force[fi]), Cr = rot(P.cop[fi]), Tr = rot(P.tz[fi]); if (!Fr) return;
      const fz = Math.hypot(...Fr);
      for (let a = 0; a < 3; a++) r.F = Math.max(r.F, Math.abs(sg * pl.F[f * 3 + a] - Fr[a]) / Math.max(1, fz) * (fz > 1 ? 1 : 0));
      if (fz > 50) {   // COP and free moment are ill-conditioned at low load
        if (Cr) for (let a = 0; a < 3; a++) r.cop = Math.max(r.cop, Math.abs(pl.cop[f * 3 + a] - Cr[a]));
        // free moment along the plate normal (ezc3d's Tz vector can carry small in-plane residuals after the Kistler COP correction)
        if (Tr) { const nz = pn; r.tz = Math.max(r.tz, Math.abs(sg * (pl.T[f * 3] * nz[0] + pl.T[f * 3 + 1] * nz[1] + pl.T[f * 3 + 2] * nz[2]) - (Tr[0] * nz[0] + Tr[1] * nz[1] + Tr[2] * nz[2]))); }
      }
      checks++;
    });
  });
  const line = Object.entries(r).map(([k, v]) => k + '=' + v.toExponential(1)).join(' ');
  console.log(fn + ' [up ' + d.up.detected + ']: ' + line + (d.plates.length ? ' plates ' + d.plates.map(p => p.type + (p.supported ? '' : 'x')).join(',') : ''));
  for (const k in r) if (r[k] > TOL[k]) bad(fn + ' ' + k + ' error ' + r[k].toExponential(2) + ' > ' + TOL[k]);
}
console.log(checks + ' comparisons, ' + fails + ' failures');
console.log(fails ? 'REF VALIDATION FAIL' : 'REF VALIDATION PASS');
process.exit(fails ? 1 : 0);
