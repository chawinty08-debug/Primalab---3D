/* Marker-QC validation (run: node tests/qc_validation.js)
   18-marker version of the synthetic CMJ (rigid segments): clean data must pass; then three injected faults must be found:
   (1) LKNEM gap-filled on the wrong segment for half the trial (moves with the shank), (2) RHEE re-placed 15 mm,
   (3) LTOE / RTOE labels swapped for 20 frames. */
const E = require('../engine.js');
function to18() {
  const d = E.makeSyntheticCMJ(), nF = d.nFrames, nM0 = d.nM;
  const names = d.base.slice(0, 16).filter(n => !/TIB/.test(n)).concat(['LKNEM', 'RKNEM', 'LANKM', 'RANKM']);
  const nM = names.length, pts = new Float32Array(nF * nM * 3), vis = new Uint8Array(nF * nM);
  for (let f = 0; f < nF; f++) names.forEach((n, i) => { let src = n, dy = 0; if (/KNEM|ANKM/.test(n)) { src = n.slice(0, 4); dy = (n[0] === 'L' ? -1 : 1) * (n.includes('KNE') ? 0.10 : 0.07); }
    const k = f * nM0 + d.base.indexOf(src); vis[f * nM + i] = d.vis[k]; for (let a = 0; a < 3; a++) pts[(f * nM + i) * 3 + a] = d.pts[k * 3 + a] + (a === 1 ? dy : 0); });
  return E.finishDataset({ name: 's18', rate: d.rate, nFrames: nF, firstFrame: 1, labels: names, pts, vis, plates: d.plates, events: [], warnings: [], info: {} });
}
const clean = to18(), qc0 = E.markerQC(clean, {});
console.log('clean: level', qc0.level, '· suspects', qc0.suspects.map(s => s.name).join(',') || 'none');
const bad = to18(), nM = bad.nM, I = n => bad.base.indexOf(n), P = (i, f) => [0, 1, 2].map(a => bad.pts[(f * nM + i) * 3 + a]), put = (i, f, v) => v.forEach((x, a) => { bad.pts[(f * nM + i) * 3 + a] = x; });
const iKM = I('LKNEM'), iAM = I('LANKM'), iRH = I('RHEE'), iLT = I('LTOE'), iRT = I('RTOE'), h = bad.nFrames >> 1;
const off = (() => { const a = P(iKM, h), b = P(iAM, h); return a.map((v, k) => v - b[k]); })();
for (let f = h; f < bad.nFrames; f++) { const b = P(iAM, f); put(iKM, f, b.map((v, k) => v + off[k])); }   // (1) rides on the shank
for (let f = 0; f < bad.nFrames; f++) { const v = P(iRH, f); v[0] += 0.015; put(iRH, f, v); }               // (2) 15 mm shift
for (let f = 300; f < 320; f++) { const a = P(iLT, f), b = P(iRT, f); put(iLT, f, b); put(iRT, f, a); }       // (3) label swap
const qc1 = E.markerQC(bad, { staticDs: clean });
console.log('faults: level', qc1.level, '· suspects', qc1.suspects.map(s => s.name + '(' + s.why.join('+') + ')').join(', '));
const pr = (a, b) => qc1.pairs.find(p => p.a === a && p.b === b), mm = v => (v * 1000).toFixed(1);
console.log('  RHEE–RTOE bias', mm(pr('RHEE', 'RTOE').median), 'mm level', pr('RHEE', 'RTOE').level, '· LKNEM–LANKM p95', mm(pr('LKNEM', 'LANKM').p95), 'level', pr('LKNEM', 'LANKM').level);
const ok = qc0.level === 0 && ['LKNEM', 'LTOE', 'RTOE'].every(n => qc1.suspects.some(s => s.name === n)) && pr('RHEE', 'RTOE').level >= 1;
console.log(ok ? 'QC VALIDATION PASS' : 'QC VALIDATION FAIL');
