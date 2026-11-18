/* Upper-body validation (run: node tests/ub_validation.js)
   1) Euler Y-X-Y decomposition round trip.
   2) Sign conventions from known postures built with rotation matrices: shoulder abduction / flexion / internal rotation,
      elbow flexion / pronation, wrist flexion / ulnar deviation, thorax flexion / lateral bend / rotation — right and
      mirrored left (left limb frames keep Z pointing right, as in the engine), plus direction checks of each motion. */
const E = require('../engine.js'), D2R = Math.PI / 180;
const mm = (A, B) => { const o = []; for (let r = 0; r < 3; r++) for (let c = 0; c < 3; c++) o.push(A[r * 3] * B[c] + A[r * 3 + 1] * B[3 + c] + A[r * 3 + 2] * B[6 + c]); return o; };
const Rx = t => [1, 0, 0, 0, Math.cos(t), -Math.sin(t), 0, Math.sin(t), Math.cos(t)], Ry = t => [Math.cos(t), 0, Math.sin(t), 0, 1, 0, -Math.sin(t), 0, Math.cos(t)], Rz = t => [Math.cos(t), -Math.sin(t), 0, Math.sin(t), Math.cos(t), 0, 0, 0, 1];
const T = A => [A[0], A[3], A[6], A[1], A[4], A[7], A[2], A[5], A[8]], S = [1, 0, 0, 0, 1, 0, 0, 0, -1], mirror = R => mm(mm(S, R), S);
let fails = 0; const chk = (lbl, got, exp, tol = 1e-6) => { const ok = got.every((v, i) => Math.abs((((v - exp[i]) % 360) + 540) % 360 - 180) < tol); if (!ok) fails++; console.log((ok ? 'ok  ' : 'FAIL') + ' ' + lbl + ' → ' + got.map(v => v.toFixed(2)).join(', ') + (ok ? '' : '  (expected ' + exp.join(', ') + ')')); };
// 1) YXY round trip
let e1 = 0; for (let i = 0; i < 500; i++) { const a = (Math.random() * 2 - 1) * Math.PI, b = Math.random() * Math.PI * 0.98 + 0.01, g = (Math.random() * 2 - 1) * Math.PI; const e = E.eulerYXYr(mm(mm(Ry(a), Rx(b)), Ry(g))); e1 = Math.max(e1, Math.abs(e[0] - a), Math.abs(e[1] - b), Math.abs(e[2] - g)); }
console.log('YXY round trip max error', e1.toExponential(1)); if (e1 > 1e-9) fails++;
// 2) shoulder mapping exactly as in computeUpperBody
const w = v => ((v % 360) + 540) % 360 - 180;
const shoulder = (Rrel, s) => { const e = E.eulerYXYr(Rrel).map(v => v / D2R); return s === 'R' ? [w(e[0] + 180), e[1], w(e[2] + 180)] : [-e[0], e[1], -e[2]]; };
const ISB = (pl, el, ax) => mm(mm(Ry(pl * D2R), Rx(-el * D2R)), Ry(ax * D2R));   // right humerus (ISB: elevation is negative about X)
const colY = R => [R[1], R[4], R[7]];
for (const [lbl, pl, el, ax] of [['abduction 90', 0, 90, 0], ['flexion 90', 90, 90, 0], ['scaption 60 / plane 30', 30, 60, 0], ['abd 90 + internal rot 40', 0, 90, 40], ['flex 120 + external rot 30', 90, 120, -30]]) {
  const R = ISB(pl, el, ax);
  chk('R shoulder ' + lbl + ' (humerus Y = ' + colY(R).map(v => v.toFixed(2)).join(',') + ')', shoulder(R, 'R'), [pl, el, ax]);
  chk('L shoulder ' + lbl + ' (mirrored)', shoulder(mirror(R), 'L'), [pl, el, ax]);
}
// direction sanity: right abduction must move the distal humerus to the RIGHT (+Z): humerus Y (pointing proximally) -> −Z
{ const y = colY(ISB(0, 90, 0)); if (!(y[2] < -0.99)) { fails++; console.log('FAIL abduction direction', y); } else console.log('ok   right abduction: distal humerus moves laterally (+Z)'); }
{ const y = colY(ISB(90, 90, 0)); if (!(y[0] < -0.99)) { fails++; console.log('FAIL flexion direction', y); } else console.log('ok   right flexion: distal humerus moves anteriorly (+X)'); }
// 3) elbow / wrist / thorax mapping (Cardan Z-X-Y)
const zxy = (R, mirrorLeft) => { const e = E.cardanZXY(R); return mirrorLeft ? [e[0], -e[1], -e[2]] : e; };
const trunk = R => { const e = E.cardanZXY(R); return [-e[0], e[1], e[2]]; };
const elbowR = mm(mm(Rz(90 * D2R), Rx(5 * D2R)), Ry(60 * D2R));   // flexion 90, varus 5, pronation 60 (right)
chk('R elbow flex 90 / varus 5 / pron 60', zxy(elbowR, false), [90, 5, 60]);
chk('L elbow (mirrored)', zxy(mirror(elbowR), true), [90, 5, 60]);
{ const y = colY(Rz(90 * D2R)); console.log((y[0] < -0.99 ? 'ok  ' : (fails++, 'FAIL')) + ' elbow flexion: forearm proximal axis points posterior → distal forearm anterior'); }
{ const x = [Ry(60 * D2R)[0], Ry(60 * D2R)[3], Ry(60 * D2R)[6]]; console.log((x[2] < 0 ? 'ok  ' : (fails++, 'FAIL')) + ' pronation (right): anterior forearm surface turns medially (−Z)'); }
chk('thorax flexion 30 (lean forward)', trunk(Rz(-30 * D2R)), [30, 0, 0]);
{ const y = colY(Rz(-30 * D2R)); console.log((y[0] > 0.4 ? 'ok  ' : (fails++, 'FAIL')) + ' thorax flexion: superior axis tilts anteriorly'); }
chk('thorax lateral bend R 20', trunk(Rx(20 * D2R)), [0, 20, 0]);
{ const y = colY(Rx(20 * D2R)); console.log((y[2] > 0.3 ? 'ok  ' : (fails++, 'FAIL')) + ' lateral bend R: superior axis tilts to the right'); }
chk('thorax axial rot L 25', trunk(Ry(25 * D2R)), [0, 0, 25]);
{ const r = Ry(25 * D2R), x = [r[0], r[3], r[6]]; console.log((x[2] < 0 ? 'ok  ' : (fails++, 'FAIL')) + ' axial rot L: anterior axis turns to the left (−Z)'); }
console.log(fails ? 'UB SIGN VALIDATION FAIL (' + fails + ')' : 'UB SIGN VALIDATION PASS');
module.exports = { fails };
