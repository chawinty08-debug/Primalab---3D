/* Inverse-dynamics validation (run: node tests/id_validation.js; DBG=1 for IK pose / contact diagnostics)
   1) Dynamic: a 3D two-leg motion (every joint DOF moving) with a PRESCRIBED hip wrench; plate F, COP and free moment are
      generated from an independent whole-leg momentum balance. Recursive Newton–Euler must recover the hip moment and the
      knee/ankle moments given by the distal-sum formula. Stance is widened in this test copy so each foot stays on its plate.
   2) Sign semantics with static postures (no ground contact). */
const E = require('../engine.js');
const D2R = Math.PI / 180, g = [0, 0, -9.80665], MASS = 72;
const V = { sub: (a, b) => a.map((v, i) => v - b[i]), add: (a, b) => a.map((v, i) => v + b[i]), mul: (a, s) => a.map(v => v * s), dot: (a, b) => a[0] * b[0] + a[1] * b[1] + a[2] * b[2], cross: (a, b) => [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]] };
const mv = (R, v) => [R[0] * v[0] + R[1] * v[1] + R[2] * v[2], R[3] * v[0] + R[4] * v[1] + R[5] * v[2], R[6] * v[0] + R[7] * v[1] + R[8] * v[2]];
const mm = (A, B) => { const o = []; for (let r = 0; r < 3; r++) for (let c = 0; c < 3; c++) o.push(A[r * 3] * B[c] + A[r * 3 + 1] * B[3 + c] + A[r * 3 + 2] * B[6 + c]); return o; };
const T = A => [A[0], A[3], A[6], A[1], A[4], A[7], A[2], A[5], A[8]];
function to18() {
  const d = E.makeSyntheticCMJ(), nF = d.nFrames, nM0 = d.nM;
  const names = d.base.slice(0, 16).filter(n => !/TIB/.test(n)).concat(['LKNEM', 'RKNEM', 'LANKM', 'RANKM']);
  const nM = names.length, pts = new Float32Array(nF * nM * 3), vis = new Uint8Array(nF * nM);
  for (let f = 0; f < nF; f++) names.forEach((n, i) => { let src = n, dy = 0; if (/KNEM|ANKM/.test(n)) { src = n.slice(0, 4); dy = (n[0] === 'L' ? -1 : 1) * (n.includes('KNE') ? 0.10 : 0.07); }
    const k = f * nM0 + d.base.indexOf(src); vis[f * nM + i] = d.vis[k]; for (let a = 0; a < 3; a++) pts[(f * nM + i) * 3 + a] = d.pts[k * 3 + a] + (a === 1 ? dy : 0); });
  return E.finishDataset({ name: 's18', rate: d.rate, nFrames: nF, firstFrame: 1, labels: names, pts, vis, plates: d.plates, events: [], warnings: [], info: {} });
}
const big = s => s > 0 ? [[2, 1.5, 0], [-2, 1.5, 0], [-2, 0, 0], [2, 0, 0]] : [[2, 0, 0], [-2, 0, 0], [-2, -1.5, 0], [2, -1.5, 0]];

function dynamicTest(label, ds0, kopt, fc) {
  const kin = E.computeLowerBody(ds0, kopt), M = E.buildIKModel(kin, 'sph');
  const dbg = E.computeID(ds0, kin, { mass: MASS, debug: true, normalize: false }), B = dbg._bsip;   // BSIP of this model
  const rate = 200, nF = 600;
  const pose = t => {   // smooth 3D motion: pelvis translating/rotating, every joint DOF moving (≤ 1.5 Hz)
    const q = M.joints.map((j, i) => j.q0.map((v, c) => v + [0.5, 0.08, 0.2][c] * Math.sin(2 * Math.PI * (0.6 + 0.15 * i + 0.1 * c) * t + i + c)));
    return E.ikForward(M, mm(E.rotZXYr(0.3 * Math.sin(2 * t), 0.1 * Math.sin(3 * t), 0.15 * Math.sin(1.5 * t)), kin.model.seg.RPV.R), [0.2 * t, 0.05 * Math.sin(2 * t), 0.95 + 0.05 * Math.sin(5 * t)], q);
  };
  const h = 1e-4;
  const kinAt = (t, k) => {   // analytic-accuracy derivatives of the true motion
    const P0 = pose(t), Pm = pose(t - h), Pp = pose(t + h), c = P => V.add(P[k].o, mv(P[k].R, B[k] ? B[k].cLoc : [0, 0, 0]));
    const om = (Pa, Pb, Pc) => { const W = mm(Pc[k].R.map((v, i) => (v - Pa[k].R[i]) / (2 * h)), T(Pb[k].R)); return [(W[7] - W[5]) / 2, (W[2] - W[6]) / 2, (W[3] - W[1]) / 2]; };
    const w = om(Pm, P0, Pp), wm = om(pose(t - 2 * h), Pm, P0), wp = om(P0, Pp, pose(t + 2 * h));
    return { R: P0[k].R, o: P0[k].o, c: c(P0), a: V.mul(V.add(V.sub(c(Pp), V.mul(c(P0), 2)), c(Pm)), 1 / (h * h)), w, al: V.mul(V.sub(wp, wm), 1 / (2 * h)) };
  };
  const hipWrench = (t, s) => ({ F: [20 * Math.sin(3 * t), 10 * s, -380 + 40 * Math.sin(4 * t)], M: [15 * Math.sin(2 * t), 25 * Math.cos(1.5 * t) * s, 8 * Math.sin(3 * t)] });   // on the thigh, N / N·m
  const Iw = (K, k) => { const I = B[k].I, Il = mm(mm(K.R, [I[0], 0, 0, 0, I[1], 0, 0, 0, I[2]]), T(K.R)); return V.add(mv(Il, K.al), V.cross(K.w, mv(Il, K.w))); };
  // build markers + plates
  const names = M.marks.map(m => m.name), nM = names.length, pts = new Float32Array(nF * nM * 3), vis = new Uint8Array(nF * nM).fill(1);
  const plates = [1, -1].map((sg, i) => ({ index: i + 1, supported: true, corners: big(sg), F: new Float32Array(nF * 3), cop: new Float32Array(nF * 3), T: new Float32Array(nF * 3), on: new Uint8Array(nF) }));
  const expect = {};
  for (const jn of ['RHip', 'RKnee', 'RAnkle', 'LHip', 'LKnee', 'LAnkle']) expect[jn] = [];
  for (let f = 0; f < nF; f++) {
    const t = f / rate, P = pose(t);
    M.marks.forEach((m, i) => { const S = P[m.seg]; const p = V.add(S.o, mv(S.R, m.p)); for (let a = 0; a < 3; a++) pts[(f * nM + i) * 3 + a] = p[a]; });
    ['R', 'L'].forEach((s, pi) => {
      const sg = s === 'R' ? 1 : -1, hw = hipWrench(t, sg), K = {}; for (const k of [s + 'TH', s + 'SK', s + 'FT']) K[k] = kinAt(t, k);
      const rh = K[s + 'TH'].o;
      // whole-leg momentum balance -> GRF wrench about the origin
      let Fg = V.mul(hw.F, -1), M0 = V.sub(V.mul(V.cross(rh, hw.F), -1), hw.M);
      for (const k of [s + 'TH', s + 'SK', s + 'FT']) { const ma = V.mul(V.sub(K[k].a, g), B[k].m); Fg = V.add(Fg, ma); M0 = V.add(M0, V.add(Iw(K[k], k), V.cross(K[k].c, ma))); }
      const px = -M0[1] / Fg[2], py = M0[0] / Fg[2], Tz = M0[2] - (px * Fg[1] - py * Fg[0]), pl = plates[s === 'L' ? 0 : 1];
      pl.F.set(Fg, f * 3); pl.cop.set([px, py, 0], f * 3); pl.T.set([0, 0, Tz], f * 3); pl.on[f] = 1;
      // expected joint moments by the distal-sum formula (independent of the recursion)
      const pelR = P.RPV.R;
      for (const [jn, dist, ref, Dset] of [[s + 'Ankle', s + 'FT', s + 'SK', [s + 'FT']], [s + 'Knee', s + 'SK', s + 'TH', [s + 'SK', s + 'FT']], [s + 'Hip', s + 'TH', 'RPV', [s + 'TH', s + 'SK', s + 'FT']]]) {
        const rJ = K[dist].o; let Mj = V.mul(V.sub(M0, V.cross(rJ, Fg)), -1);
        for (const k of Dset) Mj = V.add(Mj, V.add(Iw(K[k], k), V.cross(V.sub(K[k].c, rJ), V.mul(V.sub(K[k].a, g), B[k].m))));
        const Rr = ref === 'RPV' ? pelR : K[ref].R, loc = mv(T(Rr), Mj);
        expect[jn].push(M.PIG ? [jn.endsWith('Knee') ? loc[1] : -loc[1], (s === 'L' ? -1 : 1) * loc[0], (s === 'L' ? -1 : 1) * loc[2]] : [loc[2], loc[0], loc[1]]);
        if (jn.endsWith('Hip') && f === 300) expect.hipPrescribedCheck = [Mj, hw.M];
      }
    });
  }
  const ds = E.finishDataset({ name: 'gen', rate, nFrames: nF, firstFrame: 1, labels: names, pts, vis, plates, events: [], warnings: [], info: {} });
  const kin2 = E.computeIK(ds, kin, { model: 'sph' });
  const id = E.computeID(ds, kin2, { mass: MASS, cutoff: fc, normalize: false, fzThr: 5 });
  if (process.env.DBG) {   // IK pose error vs truth and plate assignment
    let ePos = 0, eRot = 0, worstF = -1;
    for (let f = 0; f < nF; f++) { const P = pose(f / rate); for (const k of kin2.segNames) { const a = kin2.frames[k], b = f * 12; if (a[b] !== a[b]) continue; const d = Math.hypot(a[b] - P[k].o[0], a[b + 1] - P[k].o[1], a[b + 2] - P[k].o[2]); if (d > ePos) { ePos = d; worstF = f; } } }
    console.log('   IK max origin error', (ePos * 1000).toFixed(2), 'mm at frame', worstF, '· residual mean', (kin2.residual.mean * 1000).toFixed(3), 'mm');
    let swapped = 0; for (let f = 0; f < nF; f++) { if (id.contact.R[f] !== 1 || id.contact.L[f] !== 1) swapped++; }
    console.log('   frames where a foot has no contact:', swapped);
    const f = 300, P = pose(f / rate); console.log('   foot CoM y R/L at f300', ['R', 'L'].map(s => (P[s + 'FT'].o[1]).toFixed(3)).join(' / '), '· plate COP y FP1/FP2', ds.plates[0].cop[f * 3 + 1].toFixed(3), ds.plates[1].cop[f * 3 + 1].toFixed(3));
  }
  const [Mj, Mh] = expect.hipPrescribedCheck;
  const lo = fc ? 60 : 2, hi = nF - lo;   // skip filter edge transients
  let worst = 0, scale = 0; const per = {};
  for (const jn in expect) { if (jn === 'hipPrescribedCheck') continue; let e = 0, sc = 0, nOk = 0; for (let f = lo; f < hi; f++) { if (id.joints[jn].a[f] === id.joints[jn].a[f]) nOk++; ['a', 'b', 'g'].forEach((c, k) => { const v = id.joints[jn][c][f], x = expect[jn][f][k]; if (v === v) e = Math.max(e, Math.abs(v - x)); sc = Math.max(sc, Math.abs(x)); }); } per[jn] = (e).toFixed(e < 0.01 ? 5 : 2) + ' / ' + sc.toFixed(1) + ' (' + Math.round(100 * nOk / (hi - lo)) + '% frames)'; worst = Math.max(worst, e / sc); }
  console.log(label, '· cutoff', fc || 'none', '· distal-sum == prescribed hip M:', Math.max(...Mj.map((v, i) => Math.abs(v - Mh[i]))).toExponential(1), 'N·m');
  console.log('   max |ID − expected| / peak (N·m):', JSON.stringify(per), '→ worst relative', (100 * worst).toFixed(3) + '%', id.warnings.length ? '· ' + id.warnings.join(' | ') : '');
}
// sign semantics with static postures, no ground contact
function signTest(label, ds0, kopt) {
  const kin = E.computeLowerBody(ds0, kopt), M = E.buildIKModel(kin, 'sph'), nF = 40, out = [];
  const cases = [['hip flexed 90°, knee straight (gravity extends hip → hip FLEXOR moment)', 'Hip', 0, 90, 0],
                 ['hip flexed 90°, knee straight (gravity flexes knee → knee EXTENSOR moment)', 'Knee', 0, 90, 0],
                 ['hip 90°, knee 90° (shank vertical, foot free: gravity plantarflexes → DORSIFLEXOR moment)', 'Ankle', 2, 90, -90]];
  for (const [txt, jt, , hipDeg, kneeDeg] of cases) {
    const names = M.marks.map(m => m.name), nM = names.length, pts = new Float32Array(nF * nM * 3);
    const q = M.joints.map(j => j.q0.slice());
    M.joints.forEach((j, i) => {
      if (j.jn.endsWith('Hip')) q[i][0] += (M.PIG ? -1 : 1) * hipDeg * D2R;   // PiG raw Y rotation: flexion is negative raw
      if (j.jn.endsWith('Knee')) q[i][0] += (M.PIG ? -1 : 1) * kneeDeg * D2R;   // kneeDeg < 0 = knee flexion in both conventions
    });
    const P = E.ikForward(M, kin.model.seg.RPV.R, [0, 0, 1.2], q);
    for (let f = 0; f < nF; f++) M.marks.forEach((m, i) => { const S = P[m.seg], p = V.add(S.o, mv(S.R, m.p)); pts.set(p, (f * nM + i) * 3); });
    const pl = [{ index: 1, supported: true, corners: big(1), F: new Float32Array(nF * 3), cop: new Float32Array(nF * 3), on: new Uint8Array(nF) }];
    const ds = E.finishDataset({ name: 's', rate: 100, nFrames: nF, firstFrame: 1, labels: names, pts, vis: new Uint8Array(nF * nM).fill(1), plates: pl, events: [], warnings: [], info: {} });
    const k2 = E.computeIK(ds, kin, {}), id = E.computeID(ds, k2, { mass: MASS, cutoff: 0 });
    const jn = 'R' + jt, ang = k2.angles[jn].a[20], mom = id.joints[jn].a[20];
    out.push(`   ${txt}: angle a = ${ang.toFixed(1)}°, moment a = ${mom.toFixed(3)} Nm/kg`);
  }
  console.log(label); out.forEach(l => console.log(l));
}
function widen(d) {   // test copy: legs 0.12 m further apart (pelvis unchanged)
  const pts = Float32Array.from(d.pts);
  for (let f = 0; f < d.nFrames; f++) for (let i = 0; i < d.nM; i++) { const b = d.base[i]; if (/^[LR](THI|KNE|TIB|ANK|HEE|TOE)/.test(b)) pts[(f * d.nM + i) * 3 + 1] += b[0] === 'L' ? 0.30 : -0.30; }
  return E.finishDataset(Object.assign({}, d, { pts }));
}
const s16 = widen(E.makeSyntheticCMJ()), s18 = widen(to18());
dynamicTest('conv16 V3D Z-X-Y', s16, {}, 0);
dynamicTest('bad18 PiG Y-X-Z', s18, { convention: 'pig' }, 0);
dynamicTest('conv16 V3D Z-X-Y', s16, {}, 10);
signTest('SIGN conv16 V3D (raw Cardan; V3D knee flexion is negative)', s16, {});
signTest('SIGN bad18 PiG (+ = flexion/dorsiflexion)', s18, { convention: 'pig' });
