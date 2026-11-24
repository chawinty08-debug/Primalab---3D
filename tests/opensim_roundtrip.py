"""OpenSim round trip: export the synthetic CMJ from PrimaLab, run OpenSim 4.x Scale -> IK -> ID on it
(pip install opensim) and compare joint angles and net joint moments with PrimaLab's own 6DOF and inverse dynamics.
Skipped when the opensim package is not installed. Run from the repo root: python3 tests/opensim_roundtrip.py"""
import json, os, subprocess, sys, tempfile
try:
    import opensim as osim
except ImportError:
    print('SKIP: opensim Python package not installed (pip install opensim)'); sys.exit(0)
import numpy as np

root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
tmp = tempfile.mkdtemp(prefix='primalab_osim_')
subprocess.check_call(['node', os.path.join(root, 'tests', 'tools', 'cmj_bundle.js'), tmp])
os.chdir(tmp)
osim.Logger.setLevelString('warn')
assert osim.ScaleTool('setup_scale.xml').run(), 'Scale failed'
assert osim.InverseKinematicsTool('setup_ik.xml').run(), 'IK failed'
assert osim.InverseDynamicsTool('setup_id.xml').run(), 'ID failed'


def sto(fn):
    with open(fn) as f: lines = f.read().split('\n')
    h = next(i for i, l in enumerate(lines) if l.startswith('endheader'))
    cols = lines[h + 1].split('\t'); rows = np.array([[float(v) for v in l.split('\t')] for l in lines[h + 2:] if l.strip()])
    return {c: rows[:, i] for i, c in enumerate(cols)}


P = json.load(open('primalab.json')); ik = sto('cmj_ik.mot'); idr = sto('cmj_id.sto'); err = sto('cmj_ik_marker_errors.sto')
fails = 0
rms = err['marker_error_RMS'].mean() * 1000
print('OpenSim IK marker RMS %.1f mm (mean), max %.1f mm' % (rms, err['marker_error_max'].max() * 1000))
if not rms < 25: fails += 1; print('  FAIL marker RMS')
for name, ref in P['angles'].items():
    a = np.array(ref, float); b = ik[name]; ok = np.isfinite(a)
    r = np.corrcoef(a[ok], b[ok])[0, 1]; off = np.mean(a[ok] - b[ok]); crms = np.sqrt(np.mean((a[ok] - b[ok] - off) ** 2))
    print('  %-16s r %.3f  RMSD (offset removed) %.1f deg  offset %.1f deg' % (name, r, crms, off))
    if not (r > 0.97 and crms < 4): fails += 1; print('  FAIL', name)
for name, ref in P['moments'].items():
    a = np.array(ref, float); b = idr[name]; ok = np.isfinite(a)
    r = np.corrcoef(a[ok], b[ok])[0, 1]; d = np.sqrt(np.mean((a[ok] - b[ok]) ** 2)); pk = np.abs(b[ok]).max()
    print('  %-22s r %.3f  RMSD %.1f N.m (peak %.0f)' % (name, r, d, pk))
    if not (r > 0.9): fails += 1; print('  FAIL', name)
print('OPENSIM ROUNDTRIP ' + ('FAIL' if fails else 'PASS'))
sys.exit(1 if fails else 0)
