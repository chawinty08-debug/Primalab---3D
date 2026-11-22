/* Cross-format check on the c3d.org sample set (Sample01: the same trial saved as Intel, DEC and SGI/MIPS,
   integer and float). All variants of one trial must give the same markers and analog signals.
   Set C3D_SAMPLE_DIR to the unzipped folder (CI downloads it); skipped when absent. */
'use strict';
const fs = require('fs'), path = require('path');
const E = require('../engine.js');
const DIR = process.env.C3D_SAMPLE_DIR;
if (!DIR || !fs.existsSync(DIR)) { console.log('SKIP: c3d.org samples not found (set C3D_SAMPLE_DIR)'); process.exit(0); }
const walk = d => fs.readdirSync(d, { withFileTypes: true }).flatMap(e => e.isDirectory() ? walk(path.join(d, e.name)) : /\.c3d$/i.test(e.name) ? [path.join(d, e.name)] : []);
const files = walk(DIR);
let fails = 0; const check = (ok, m) => { if (!ok) { fails++; console.log('  FAIL ' + m); } };
const read = f => { const b = fs.readFileSync(f); return E.readC3D(b.buffer.slice(b.byteOffset, b.byteOffset + b.length), path.basename(f), { up: '+Z' }); };
// group by trial stem: Eb015pi / Eb015pr / Eb015si ... -> 'Eb015' (last two letters = storage + processor)
const groups = {};
for (const f of files) { const m = path.basename(f).match(/^(.*?)([a-z])([a-z])\.c3d$/i); const k = m ? m[1] : path.basename(f); (groups[k] = groups[k] || []).push(f); }
for (const [k, list] of Object.entries(groups)) {
  const D = [];
  for (const f of list) { try { const d = read(f); D.push({ f: path.basename(f), d }); console.log(path.basename(f) + ': proc ' + d.info.processor + (d.info.floatData ? ' float' : ' int') + ', ' + d.nFrames + ' fr, ' + d.nM + ' markers, ' + d.info.analogChannels + ' analog'); } catch (e) { check(false, path.basename(f) + ' read error: ' + e.message); } }
  if (D.length < 2) continue;
  const ref = D[0].d;
  for (const { f, d } of D.slice(1)) {
    check(d.nFrames === ref.nFrames && d.nM === ref.nM, f + ' size differs from ' + D[0].f);
    if (d.nFrames !== ref.nFrames || d.nM !== ref.nM) continue;
    check(d.labels.join() === ref.labels.join(), f + ' labels differ');
    let ep = 0, n = 0;
    for (let k2 = 0; k2 < ref.vis.length; k2++) if (ref.vis[k2] && d.vis[k2]) { n++; for (let a = 0; a < 3; a++) ep = Math.max(ep, Math.abs(ref.pts[k2 * 3 + a] - d.pts[k2 * 3 + a])); }
    let ea = 0;
    if (ref.analog && d.analog && ref.analog.length === d.analog.length) for (let i = 0; i < ref.analog.length; i++) ea = Math.max(ea, Math.abs(ref.analog[i] - d.analog[i]) / Math.max(1, Math.abs(ref.analog[i])));
    console.log('  ' + f + ' vs ' + D[0].f + ': max |Δmarker| ' + (ep * 1000).toFixed(3) + ' mm over ' + n + ' points, max rel |Δanalog| ' + ea.toExponential(1));
    check(ep < 2e-3, f + ' markers differ by ' + (ep * 1000).toFixed(2) + ' mm');   // integer storage quantises to POINT:SCALE
    check(ea < 1e-2, f + ' analog differs ' + ea);
  }
}
console.log(fails ? 'PROCESSOR SAMPLES FAIL' : 'PROCESSOR SAMPLES PASS'); process.exit(fails ? 1 : 0);
