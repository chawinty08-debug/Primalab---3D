/* Run every test suite: node tests/run_all.js  (or npm test). Full output with VERBOSE=1 (always on CI). */
'use strict';
const { spawnSync } = require('child_process'), fs = require('fs'), path = require('path');
const skip = new Set(['run_all.js', 'c3d_writer.js']);
const suites = fs.readdirSync(__dirname).filter(f => f.endsWith('.js') && !skip.has(f)).sort();
let bad = 0;
for (const f of suites) {
  const t0 = Date.now(), r = spawnSync(process.execPath, [path.join(__dirname, f)], { encoding: 'utf8', env: process.env });
  const out = (r.stdout || '') + (r.stderr || ''), last = out.trim().split('\n').pop();
  const ok = r.status === 0;
  if (!ok) bad++;
  console.log((ok ? 'ok   ' : 'FAIL ') + f.padEnd(24) + ((Date.now() - t0) / 1000).toFixed(1).padStart(6) + ' s  ' + last);
  if (!ok || process.env.VERBOSE || process.env.CI) console.log(out.split('\n').map(l => '     ' + l).join('\n'));
}
console.log(bad ? bad + ' suite(s) failed' : 'all ' + suites.length + ' suites passed');
process.exit(bad ? 1 : 0);
