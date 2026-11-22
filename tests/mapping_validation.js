/* Marker-name mapping: a lab-specific naming (ISB-style anatomical names) is mapped onto the model names and
   must give exactly the same joint angles as the original file. */
'use strict';
const E = require('../engine.js');
let fails = 0; const check = (ok, m) => { if (!ok) { fails++; console.log('  FAIL ' + m); } };
const d0 = E.makeSyntheticCMJ();
const REN = { LASI: 'L_ASIS', RASI: 'ASIS_R', LPSI: 'Left PSIS', RPSI: 'R.PSIS', LKNE: 'LLFE', RKNE: 'R_KNEE', LTHI: 'L_THIGH', RTHI: 'RTHIGH',
  LANK: 'LLMAL', RANK: 'R_ANKLE', LTIB: 'L_SHANK', RTIB: 'RTIBIA', LTOE: 'LMT2', RTOE: 'R_MH2', LHEE: 'L_HEEL', RHEE: 'RCAL' };
const labels = d0.labels.map(l => { const s = E.splitLabel(l); return REN[s.base] ? s.prefix + REN[s.base] : l; });
const mk = aliases => E.finishDataset({ name: 'renamed', rate: d0.rate, nFrames: d0.nFrames, firstFrame: 1, labels, pts: d0.pts, vis: d0.vis, plates: d0.plates, events: [], warnings: [], info: {} }, aliases);
const dr = mk(null);
check(E.detectLBSet(dr) === null, 'renamed file should not match a model before mapping');
const rep0 = E.markerReport(dr);
check(rep0.lb16.found === 0, 'report before mapping found ' + rep0.lb16.found);
const sug = E.suggestMarkerMap(dr, ['lb16']);
for (const k in REN) check(sug[k] === REN[k], 'suggest ' + k + ' -> ' + sug[k] + ' (expected ' + REN[k] + ')');
const inv = {}; for (const k in sug) inv[sug[k]] = k;
const dm = mk(inv);
check(E.detectLBSet(dm) === 'conv16', 'mapped file not detected as conv16');
check(E.markerReport(dm).lb16.ok, 'report after mapping not ok');
const a = E.computeLowerBody(d0), b = E.computeLowerBody(dm);
let dmax = 0; for (const j in a.angles) for (const c of ['a', 'b', 'g']) for (let f = 0; f < a.angles[j][c].length; f++) { const x = a.angles[j][c][f], y = b.angles[j][c][f]; if (x === x) dmax = Math.max(dmax, Math.abs(x - y)); }
check(a.ok && b.ok && dmax === 0, 'angles differ after mapping: ' + dmax);
// a model name already in the file must not be overwritten by a mapping
const dk = mk(Object.assign({}, inv, { 'Unlabeled 1001': 'LASI' }));
check(dk.base.filter(x => x === 'LASI').length === 1, 'duplicate model name created');
console.log('suggested ' + Object.keys(sug).length + '/16, max |Δangle| after mapping = ' + dmax);
console.log(fails ? 'MAPPING FAIL' : 'MAPPING PASS'); process.exit(fails ? 1 : 0);
