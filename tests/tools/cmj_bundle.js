/* writes the OpenSim bundle of the synthetic CMJ + PrimaLab angles/moments (JSON) into argv[2] (used by opensim_roundtrip.py) */
'use strict';
const fs = require('fs'), path = require('path');
eval(fs.readFileSync(path.join(__dirname, '..', '..', 'msk_data.js'), 'utf8').replace('const MSK_DATA', 'global.MSK_DATA'));
const E = require('../../engine.js');
(async () => {
  const out = process.argv[2]; fs.mkdirSync(out, { recursive: true });
  const d = E.makeSyntheticCMJ(), k = E.computeLowerBody(d), id = E.computeID(d, k, { mass: 75, normalize: false, cutoff: 6, cutoffGRF: 6 });
  const b = await E.opensimZip(d, { kin: k, name: 'cmj', mass: 75, grfCutoff: 6, cutoff: 6 });
  for (const [n, v] of Object.entries(b.files)) fs.writeFileSync(path.join(out, n), typeof v === 'string' ? v : Buffer.from(v));
  const A = n => Array.from(k.angles[n].a), Mm = n => Array.from(id.joints[n].a);
  fs.writeFileSync(path.join(out, 'primalab.json'), JSON.stringify({ rate: d.rate, angles: { hip_flexion_r: A('RHip'), knee_angle_r: A('RKnee'), ankle_angle_r: A('RAnkle'), hip_flexion_l: A('LHip'), knee_angle_l: A('LKnee'), ankle_angle_l: A('LAnkle') },
    moments: { hip_flexion_r_moment: Mm('RHip'), knee_angle_r_moment: Mm('RKnee'), ankle_angle_r_moment: Mm('RAnkle'), hip_flexion_l_moment: Mm('LHip'), knee_angle_l_moment: Mm('LKnee'), ankle_angle_l_moment: Mm('LAnkle') } }));
})();
